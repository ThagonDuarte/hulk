import gzip
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from typing import Any, cast
from unittest import mock

import numpy as np
import torch
import yaml
from click.testing import CliRunner
from PIL import Image

from ultralytics_dfine.data import DFINEDataset
from ultralytics_dfine.engine.metrics import OKS_THRESHOLDS
from ultralytics_dfine.engine.multitask_validator import (
    CROSS_POSE_PERSON_ON_ROBOT,
    CROSS_POSE_ROBOT_ON_PERSON,
    MultiTaskValidator,
)
from ultralytics_dfine.nn import DFINEMultiTaskModel
from ultralytics_dfine.schemas import HeadId
from utils.nv12_to_rgb import NV12ToRgb
from utils.prepare_annotato_dataset import (
    _field_complete,
    _field_validation_eligible,
)
from validation.compare_multitask import (
    _MATCH_THRESHOLDS,
    _RECALL_THRESHOLDS,
    _canonical_field_metrics,
    _canonical_object_metrics,
    _canonical_pose_metrics,
    _compute_cross_pose_scores,
    _prepare_cluster_bootstrap,
    _prepare_cross_pose_scores,
    _prepare_field_ap,
    _prepare_field_localization,
    _prepare_negative_objects,
    _prepare_object_ap,
    _prepare_pose_ap,
    _promotion_gates,
    compare,
    compatibility_report,
)
from validation.compare_multitask import (
    main as compare_main,
)
from validation.multitask_backends import (
    packed_nv12_to_rgb,
    rgb_to_packed_nv12,
)
from validation.multitask_data import (
    create_subset_manifest,
    dataset_fingerprint,
    dataset_target_fingerprint,
    subset_datasets_from_manifest,
)
from validation.validate_multitask import (
    _distributed_context,
    _merge_shards,
    _merge_subset_manifests,
    _with_process_group_cleanup,
)
from validation.validate_multitask import main as validate_main


class _CrossPoseModel(torch.nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.detector = SimpleNamespace(
            names=[
                "Ball",
                "GoalPost",
                "LSpot",
                "PenaltySpot",
                "Robot",
                "TSpot",
                "XSpot",
                "Person",
            ]
        )
        self.calls: list[HeadId] = []

    def forward_deploy_task(
        self,
        images: torch.Tensor,
        task: HeadId,
    ) -> dict[str, torch.Tensor]:
        self.calls.append(task)
        batch = images.shape[0]
        if task == HeadId.PERSON_POSE:
            output = torch.zeros(batch, 2, 57)
            output[..., 2:4] = 10
            output[..., 4] = torch.tensor([0.8, 0.2])
            keypoints = output[..., 6:].reshape(batch, 2, 17, 3)
            keypoints[..., :2] = 5
            keypoints[:, 0, :, 2] = 0.5
            keypoints[:, 1, :, 2] = 1.0
            return {"person_pose_output": output}
        if task == HeadId.ROBOT_POSE:
            objects = torch.zeros(batch, 2, 6)
            objects[..., 2:4] = 10
            objects[..., 4] = torch.tensor([0.8, 0.4])
            objects[..., 5] = 4
            poses = torch.zeros(batch, 2, 14, 3)
            poses[..., :2] = 5
            poses[:, 0, :, 2] = 0.25
            poses[:, 1, :, 2] = 1.0
            return {
                "object_output": objects,
                "robot_pose_output": poses,
            }
        raise AssertionError


class ValidationBackendTests(unittest.TestCase):
    def test_process_group_cleanup_runs_after_success_and_failure(self) -> None:
        def successful() -> int:
            return 7

        def failing() -> None:
            raise RuntimeError

        for callback in (successful, failing):
            with (
                self.subTest(callback=callback.__name__),
                mock.patch(
                    "torch.distributed.is_initialized", return_value=True
                ),
                mock.patch(
                    "torch.distributed.destroy_process_group"
                ) as destroy,
            ):
                wrapped = _with_process_group_cleanup(callback)
                if callback is failing:
                    with self.assertRaises(RuntimeError):
                        wrapped()
                else:
                    self.assertEqual(wrapped(), 7)
                destroy.assert_called_once_with()

    def test_torchrun_cuda_device_uses_local_rank_and_device_id(self) -> None:
        signature = SimpleNamespace(parameters={"device_id": object()})
        environment = {"RANK": "3", "WORLD_SIZE": "4", "LOCAL_RANK": "1"}
        with (
            mock.patch.dict("os.environ", environment, clear=True),
            mock.patch("torch.cuda.set_device") as set_device,
            mock.patch("torch.distributed.is_initialized", return_value=False),
            mock.patch("torch.distributed.init_process_group") as initialize,
            mock.patch("inspect.signature", return_value=signature),
        ):
            rank, world_size, device = _distributed_context("cuda:3")

        self.assertEqual((rank, world_size), (3, 4))
        self.assertEqual(device, torch.device("cuda:1"))
        set_device.assert_called_once_with(torch.device("cuda:1"))
        initialize.assert_called_once_with(
            backend="nccl",
            device_id=torch.device("cuda:1"),
        )

    def test_nv12_roundtrip_has_public_packed_shape(self) -> None:
        image = torch.rand(2, 3, 48, 64)

        packed = rgb_to_packed_nv12(image)
        decoded = packed_nv12_to_rgb(packed)

        self.assertEqual(packed.shape, (2, 24, 32, 6))
        self.assertEqual(packed.dtype, torch.uint8)
        self.assertEqual(decoded.shape, image.shape)
        self.assertTrue(torch.isfinite(decoded).all())

    def test_nv12_roundtrip_matches_unclamped_deployment_converter(
        self,
    ) -> None:
        image = torch.zeros(1, 3, 2, 2)
        image[:, 0] = 1.0

        packed = rgb_to_packed_nv12(image)
        decoded = packed_nv12_to_rgb(packed)
        deployment = NV12ToRgb(subsample=False)(packed[0]).permute(2, 0, 1)

        torch.testing.assert_close(decoded[0], deployment)
        self.assertTrue((decoded < 0).any() or (decoded > 1).any())

    def test_validator_record_sink_payload_uses_metric_coordinates(
        self,
    ) -> None:
        names = [
            "Ball",
            "GoalPost",
            "LSpot",
            "PenaltySpot",
            "Robot",
            "TSpot",
            "XSpot",
            "Person",
        ]
        model = cast(
            DFINEMultiTaskModel,
            SimpleNamespace(detector=SimpleNamespace(names=names)),
        )
        validator = MultiTaskValidator(model, {}, device="cpu")
        outputs = {
            "object_output": torch.tensor(
                [[[10, 20, 30, 40, 0.9, 0], [0, 0, 1, 1, 0.8, 7]]],
                dtype=torch.float32,
            )
        }
        target: dict[str, object] = {
            "path": Path("image.png"),
            "image_id": torch.tensor(4),
            "orig_size": torch.tensor([100, 200]),
            "boxes": torch.tensor([[0.5, 0.5, 0.2, 0.4]]),
            "labels": torch.tensor([0]),
            "valid_detection_classes": torch.tensor(
                [True, True, True, True, True, True, True, False]
            ),
        }

        record = validator._prediction_record(
            HeadId.OBJECT,
            outputs,
            0,
            target,
            100,
            200,
        )

        predictions = cast(dict[str, object], record["predictions"])
        targets = cast(dict[str, object], record["targets"])
        self.assertEqual(predictions["labels"], [0])
        self.assertEqual(targets["boxes_xyxy"], [[80.0, 30.0, 120.0, 70.0]])

    def test_robot_visibility_calibration_does_not_change_objects(self) -> None:
        names = [
            "Ball",
            "GoalPost",
            "LSpot",
            "PenaltySpot",
            "Robot",
            "TSpot",
            "XSpot",
            "Person",
        ]
        model = cast(
            DFINEMultiTaskModel,
            SimpleNamespace(detector=SimpleNamespace(names=names)),
        )
        validator = MultiTaskValidator(
            model,
            {},
            device="cpu",
            robot_visibility_alphas=(1.0,),
        )
        objects = torch.tensor(
            [[0, 0, 1, 1, 0.8, 4], [0, 0, 1, 1, 0.7, 4]],
            dtype=torch.float32,
        )
        original = objects.clone()
        poses = torch.zeros((2, 14, 3), dtype=torch.float32)
        poses[0, :, 2] = 0.25
        poses[1, :, 2] = 1.0

        _, scores, visibility = validator._robot_predictions(
            objects,
            poses,
            1.0,
        )

        torch.testing.assert_close(objects, original)
        torch.testing.assert_close(scores, torch.tensor([0.7, 0.2]))
        torch.testing.assert_close(visibility, torch.tensor([1.0, 0.25]))

    def test_person_visibility_calibration_preserves_alpha_zero_and_input(
        self,
    ) -> None:
        model = cast(
            DFINEMultiTaskModel,
            SimpleNamespace(detector=SimpleNamespace(names=[])),
        )
        validator = MultiTaskValidator(model, {}, device="cpu")
        output = torch.zeros((3, 57), dtype=torch.float32)
        output[:, 0] = torch.tensor([1.0, 2.0, 3.0])
        output[:, 4] = torch.tensor([0.8, 0.7, 0.0005])
        keypoints = output[:, 6:].reshape(3, 17, 3)
        keypoints[0, :, 2] = 0.25
        keypoints[1, :, 2] = 1.0
        keypoints[2, :, 2] = 1.0
        original = output.clone()

        alpha_zero, zero_scores, zero_visibility = (
            validator._person_predictions(output, 0.0)
        )
        alpha_one, one_scores, one_visibility = validator._person_predictions(
            output, 1.0
        )

        torch.testing.assert_close(output, original)
        torch.testing.assert_close(alpha_zero[:, 0], torch.tensor([1.0, 2.0]))
        torch.testing.assert_close(zero_scores, torch.tensor([0.8, 0.7]))
        torch.testing.assert_close(
            zero_visibility,
            torch.tensor([0.25, 1.0]),
        )
        torch.testing.assert_close(alpha_one[:, 0], torch.tensor([2.0, 1.0]))
        torch.testing.assert_close(one_scores, torch.tensor([0.7, 0.2]))
        torch.testing.assert_close(
            one_visibility,
            torch.tensor([1.0, 0.25]),
        )

    def test_person_visibility_sweep_uses_first_alpha_as_canonical(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as directory:
            target: dict[str, object] = {
                "path": Path(directory) / "person.jpg",
                "image_id": torch.tensor(7),
                "orig_size": torch.tensor([10, 10]),
                "boxes": torch.tensor([[0.5, 0.5, 1.0, 1.0]]),
                "keypoints": torch.tensor([[[0.5, 0.5, 1.0]] * 17]),
                "visibility": torch.ones((1, 17), dtype=torch.bool),
                "robot_negative_verified": True,
                "robot_negative_eligible": True,
                "robot_negative_reviewed": True,
                "robot_negative_excluded": False,
            }
            loader = [(torch.zeros(1, 3, 10, 10), [target])]
            model = _CrossPoseModel()
            validator = MultiTaskValidator(
                cast(DFINEMultiTaskModel, model),
                cast(Any, {HeadId.PERSON_POSE: loader}),
                device="cpu",
                person_visibility_alphas=(1.0, 0.0, 1.0),
            )
            records: list[dict[str, object]] = []

            metrics = validator.run(record_sink=records.append)

        self.assertEqual(validator.person_visibility_alphas, (1.0, 0.0))
        self.assertEqual(
            metrics["person/map"],
            metrics["person/visibility_alpha/1/map"],
        )
        self.assertIn("person/visibility_alpha/0/map", metrics)
        predictions = cast(dict[str, object], records[0]["predictions"])
        self.assertEqual(predictions["visibility_alpha"], 1.0)
        np.testing.assert_allclose(predictions["scores"], [0.4, 0.2])
        self.assertEqual(predictions["mean_visibility"], [0.5, 1.0])

    def test_person_visibility_alphas_must_be_valid(self) -> None:
        model = cast(
            DFINEMultiTaskModel,
            SimpleNamespace(detector=SimpleNamespace(names=[])),
        )
        for values in ((), (-0.1,), (float("inf"),)):
            with (
                self.subTest(values=values),
                self.assertRaisesRegex(ValueError, "[Pp]erson visibility"),
            ):
                MultiTaskValidator(
                    model,
                    {},
                    device="cpu",
                    person_visibility_alphas=values,
                )

    def test_validator_rejects_non_finite_scalar_configuration(self) -> None:
        model = cast(
            DFINEMultiTaskModel,
            SimpleNamespace(detector=SimpleNamespace(names=[])),
        )
        invalid = (
            {"confidence": float("nan")},
            {"render_field_confidence": float("inf")},
            {"field_normalization": float("nan")},
            {"field_normalization": 0.0},
        )
        for values in invalid:
            with (
                self.subTest(values=values),
                self.assertRaisesRegex(ValueError, "[Ff]inite|positive"),
            ):
                MultiTaskValidator(model, {}, device="cpu", **values)

    def test_confidence_filters_prediction_records(self) -> None:
        names = [
            "Ball",
            "GoalPost",
            "LSpot",
            "PenaltySpot",
            "Robot",
            "TSpot",
            "XSpot",
            "Person",
        ]
        model = cast(
            DFINEMultiTaskModel,
            SimpleNamespace(detector=SimpleNamespace(names=names)),
        )
        validator = MultiTaskValidator(
            model,
            {},
            device="cpu",
            confidence=0.5,
        )
        outputs = {
            "object_output": torch.tensor(
                [[[0, 0, 1, 1, 0.49, 0], [0, 0, 1, 1, 0.5, 0]]],
                dtype=torch.float32,
            )
        }
        target: dict[str, object] = {
            "path": Path("image.png"),
            "image_id": torch.tensor(4),
            "orig_size": torch.tensor([100, 200]),
            "boxes": torch.empty((0, 4)),
            "labels": torch.empty(0, dtype=torch.long),
            "valid_detection_classes": torch.ones(8, dtype=torch.bool),
        }

        record = validator._prediction_record(
            HeadId.OBJECT,
            outputs,
            0,
            target,
            100,
            200,
        )

        predictions = cast(dict[str, object], record["predictions"])
        self.assertEqual(predictions["scores"], [0.5])

    def test_cross_pose_eval_is_opt_in_and_uses_opposite_head(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            target: dict[str, object] = {
                "path": Path(directory) / "person.jpg",
                "image_id": torch.tensor(7),
                "orig_size": torch.tensor([10, 10]),
                "boxes": torch.tensor([[0.5, 0.5, 1.0, 1.0]]),
                "keypoints": torch.tensor([[[0.5, 0.5, 1.0]] * 17]),
                "visibility": torch.ones((1, 17), dtype=torch.bool),
                "robot_negative_verified": True,
                "robot_negative_eligible": True,
                "robot_negative_reviewed": True,
                "robot_negative_excluded": False,
            }
            loader = [(torch.zeros(1, 3, 10, 10), [target])]

            default_model = _CrossPoseModel()
            default_validator = MultiTaskValidator(
                cast(DFINEMultiTaskModel, default_model),
                cast(Any, {HeadId.PERSON_POSE: loader}),
                device="cpu",
            )
            default_metrics = default_validator.run()

            model = _CrossPoseModel()
            validator = MultiTaskValidator(
                cast(DFINEMultiTaskModel, model),
                cast(Any, {HeadId.PERSON_POSE: loader}),
                device="cpu",
                cross_pose_visibility_alphas=(0.0, 1.0),
            )
            records: list[dict[str, object]] = []
            metrics = validator.run(cross_pose_record_sink=records.append)

        self.assertEqual(default_model.calls, [HeadId.PERSON_POSE])
        self.assertFalse(
            any(name.startswith("cross_pose/") for name in default_metrics)
        )
        self.assertEqual(
            model.calls,
            [HeadId.PERSON_POSE, HeadId.ROBOT_POSE],
        )
        self.assertEqual(metrics[f"{CROSS_POSE_ROBOT_ON_PERSON}/images"], 1)
        alpha_zero = f"{CROSS_POSE_ROBOT_ON_PERSON}/visibility_alpha/0"
        alpha_one = f"{CROSS_POSE_ROBOT_ON_PERSON}/visibility_alpha/1"
        self.assertAlmostEqual(metrics[f"{alpha_zero}/max_score_mean"], 0.8)
        self.assertEqual(
            metrics[f"{alpha_zero}/detections_at_50_per_image"],
            1,
        )
        self.assertAlmostEqual(metrics[f"{alpha_one}/max_score_mean"], 0.4)
        self.assertEqual(
            metrics[f"{alpha_one}/detections_at_25_per_image"],
            1,
        )
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0]["task"], CROSS_POSE_ROBOT_ON_PERSON)
        self.assertEqual(records[0]["targets"], {"scores": []})
        person_outputs = model.forward_deploy_task(
            torch.zeros(1, 3, 10, 10),
            HeadId.PERSON_POSE,
        )
        direction, scores, visibility = validator._cross_pose_scores(
            HeadId.ROBOT_POSE,
            person_outputs,
            0,
        )
        self.assertEqual(direction, CROSS_POSE_PERSON_ON_ROBOT)
        torch.testing.assert_close(scores, torch.tensor([0.8, 0.2]))
        torch.testing.assert_close(visibility, torch.tensor([0.5, 1.0]))

    def test_robot_on_person_cross_pose_requires_record_eligibility(
        self,
    ) -> None:
        def run(
            *, eligible: bool
        ) -> tuple[
            _CrossPoseModel,
            dict[str, object],
            list[dict[str, object]],
        ]:
            target: dict[str, object] = {
                "path": Path("person.jpg"),
                "image_id": torch.tensor(7),
                "orig_size": torch.tensor([10, 10]),
                "boxes": torch.tensor([[0.5, 0.5, 1.0, 1.0]]),
                "keypoints": torch.tensor([[[0.5, 0.5, 1.0]] * 17]),
                "visibility": torch.ones((1, 17), dtype=torch.bool),
                "robot_negative_verified": eligible,
                "robot_negative_eligible": eligible,
                "robot_negative_reviewed": eligible,
                "robot_negative_excluded": False,
            }
            loader = [(torch.zeros(1, 3, 10, 10), [target])]
            model = _CrossPoseModel()
            validator = MultiTaskValidator(
                cast(DFINEMultiTaskModel, model),
                cast(Any, {HeadId.PERSON_POSE: loader}),
                device="cpu",
                cross_pose_visibility_alphas=(0.0,),
            )
            records: list[dict[str, object]] = []
            metrics = validator.run(cross_pose_record_sink=records.append)
            return model, metrics, records

        unsafe_model, unsafe_metrics, unsafe_records = run(eligible=False)
        safe_model, safe_metrics, safe_records = run(eligible=True)

        self.assertEqual(unsafe_model.calls, [HeadId.PERSON_POSE])
        self.assertNotIn(f"{CROSS_POSE_ROBOT_ON_PERSON}/images", unsafe_metrics)
        self.assertEqual(unsafe_records, [])
        self.assertEqual(
            safe_model.calls,
            [HeadId.PERSON_POSE, HeadId.ROBOT_POSE],
        )
        self.assertEqual(
            safe_metrics[f"{CROSS_POSE_ROBOT_ON_PERSON}/images"], 1
        )
        eligibility = safe_records[0]["negative_target_eligibility"]
        self.assertEqual(
            eligibility,
            {
                "policy": "explicit-verified-robot-free-keypoint-v2",
                "reviewed": True,
                "verified": True,
                "eligible": True,
                "excluded": False,
            },
        )

    def test_cross_pose_cli_documents_default_off_alpha_sweep(self) -> None:
        result = CliRunner().invoke(validate_main, ["--help"])

        self.assertEqual(result.exit_code, 0)
        self.assertIn("--person-visibility-alpha", result.output)
        self.assertIn(
            "--cross-pose-negatives / --no-cross-pose-negatives",
            result.output,
        )
        self.assertIn("[default: 0.0, 1.0; x>=0]", result.output)
        self.assertIn("--coco-robot-negative-manifest", result.output)
        self.assertIn("--coco-robot-negative-role", result.output)
        self.assertIn("--dhrp-person-negative-manifest", result.output)
        self.assertIn("primary_evaluation", result.output)
        self.assertIn("stress_evaluation", result.output)
        self.assertIn("loss_holdout_validation", result.output)

    def test_person_on_robot_cross_pose_requires_record_eligibility(
        self,
    ) -> None:
        def run(
            *, eligible: bool
        ) -> tuple[
            _CrossPoseModel,
            dict[str, object],
            list[dict[str, object]],
        ]:
            target: dict[str, object] = {
                "path": Path("robot.jpg"),
                "image_id": torch.tensor(3),
                "orig_size": torch.tensor([10, 10]),
                "boxes": torch.tensor([[0.5, 0.5, 1.0, 1.0]]),
                "labels": torch.tensor([4]),
                "keypoints": torch.tensor([[[0.5, 0.5, 1.0]] * 14]),
                "visibility": torch.ones((1, 14), dtype=torch.bool),
                "person_negative_verified": eligible,
                "person_negative_eligible": eligible,
            }
            loader = [(torch.zeros(1, 3, 10, 10), [target])]
            model = _CrossPoseModel()
            validator = MultiTaskValidator(
                cast(DFINEMultiTaskModel, model),
                cast(Any, {HeadId.ROBOT_POSE: loader}),
                device="cpu",
                cross_pose_visibility_alphas=(0.0,),
            )
            records: list[dict[str, object]] = []
            metrics = validator.run(cross_pose_record_sink=records.append)
            return model, metrics, records

        unsafe_model, unsafe_metrics, unsafe_records = run(eligible=False)
        safe_model, safe_metrics, safe_records = run(eligible=True)

        self.assertEqual(unsafe_model.calls, [HeadId.ROBOT_POSE])
        self.assertNotIn(f"{CROSS_POSE_PERSON_ON_ROBOT}/images", unsafe_metrics)
        self.assertEqual(unsafe_records, [])
        self.assertEqual(
            safe_model.calls,
            [HeadId.ROBOT_POSE, HeadId.PERSON_POSE],
        )
        self.assertEqual(
            safe_metrics[f"{CROSS_POSE_PERSON_ON_ROBOT}/images"],
            1,
        )
        self.assertEqual(len(safe_records), 1)


class ValidationDataTests(unittest.TestCase):
    def test_target_fingerprint_tracks_order_and_label_contents(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "images").mkdir()
            (root / "labels").mkdir()
            images = []
            for index in range(2):
                image = root / "images" / f"{index}.png"
                Image.new("RGB", (8, 8), (index, index, index)).save(image)
                (root / "labels" / f"{index}.txt").write_text(
                    f"{index} 0.5 0.5 0.2 0.2\n",
                    encoding="utf-8",
                )
                images.append(image)
            image_list = root / "images.txt"
            image_list.write_text(
                "\n".join(str(path) for path in images) + "\n",
                encoding="utf-8",
            )
            data = root / "data.yaml"
            data.write_text(
                yaml.safe_dump(
                    {
                        "path": str(root),
                        "val": str(image_list),
                        "names": ["Ball", "GoalPost"],
                    }
                ),
                encoding="utf-8",
            )

            dataset = DFINEDataset(data, "val", image_size=32)
            baseline = dataset_target_fingerprint(dataset)
            self.assertEqual(baseline, dataset_target_fingerprint(dataset))

            (root / "labels" / "0.txt").write_text(
                "0 0.4 0.5 0.2 0.2\n",
                encoding="utf-8",
            )
            changed_label = dataset_target_fingerprint(dataset)
            self.assertNotEqual(baseline["digest"], changed_label["digest"])

            (root / "labels" / "0.txt").write_text(
                "0 0.5 0.5 0.2 0.2\n",
                encoding="utf-8",
            )
            image_list.write_text(
                "\n".join(str(path) for path in reversed(images)) + "\n",
                encoding="utf-8",
            )
            reordered = dataset_target_fingerprint(
                DFINEDataset(data, "val", image_size=32)
            )
            self.assertNotEqual(baseline["digest"], reordered["digest"])

    def test_distributed_merge_keeps_cross_pose_sidecar_separate(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for rank, task in enumerate(("person_pose", "robot_pose")):
                shard = root / "shards" / f"rank-{rank:02d}"
                shard.mkdir(parents=True)
                (shard / "metrics.json").write_text(
                    json.dumps({f"rank/{rank}": rank})
                )
                (shard / "metadata.json").write_text(
                    json.dumps(
                        {"dataset_fingerprints": {task: {"digest": task}}}
                    )
                )
                (shard / "config.json").write_text(
                    json.dumps({"tasks": [task]})
                )
                with gzip.open(
                    shard / "predictions.jsonl.gz",
                    "wt",
                    encoding="utf-8",
                ) as file:
                    file.write(json.dumps({"image_key": task}) + "\n")
                with gzip.open(
                    shard / "cross_pose_predictions.jsonl.gz",
                    "wt",
                    encoding="utf-8",
                ) as file:
                    file.write(
                        json.dumps({"image_key": f"cross:{task}"}) + "\n"
                    )

            _merge_shards(root, 2)

            with gzip.open(
                root / "cross_pose_predictions.jsonl.gz",
                "rt",
                encoding="utf-8",
            ) as file:
                records = [json.loads(line) for line in file]
            self.assertEqual(
                [record["image_key"] for record in records],
                ["cross:person_pose", "cross:robot_pose"],
            )
            with gzip.open(
                root / "predictions.jsonl.gz",
                "rt",
                encoding="utf-8",
            ) as file:
                canonical = [json.loads(line) for line in file]
            self.assertEqual(
                [record["image_key"] for record in canonical],
                ["person_pose", "robot_pose"],
            )

    def test_rank_local_subset_manifests_merge_all_tasks(self) -> None:
        header = {
            "version": 1,
            "strategy": "class-round-robin-sha256",
            "seed": 17,
            "count_per_task": 2,
        }
        object_manifest = {
            **header,
            "tasks": {"object": {"indices": [0, 1]}},
        }
        field_manifest = {
            **header,
            "tasks": {"field_features": {"indices": [2, 3]}},
        }

        merged = _merge_subset_manifests([object_manifest, field_manifest])

        self.assertEqual(
            list(cast(dict[str, object], merged["tasks"])),
            ["field_features", "object"],
        )

    def test_field_complete_accepts_authoritative_negative_images(self) -> None:
        negative = [{"class": "Ball"}]
        positive = [{"class": "GoalPost", "point": [0.2, 0.3]}]
        incomplete = [{"class": "GoalPost", "migration_skipped": True}]

        self.assertTrue(_field_complete(negative))
        self.assertTrue(_field_complete(positive))
        self.assertFalse(_field_complete(incomplete))
        self.assertFalse(_field_validation_eligible(negative))
        self.assertTrue(_field_validation_eligible(positive))
        self.assertFalse(_field_validation_eligible(incomplete))

    def test_subset_manifest_is_deterministic_and_fingerprint_checked(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for split in ("images/val", "labels/val"):
                (root / split).mkdir(parents=True)
            for index, class_id in enumerate((0, 0, 1, 1)):
                Image.new("RGB", (8, 8), (index, index, index)).save(
                    root / "images/val" / f"{index}.png"
                )
                (root / "labels/val" / f"{index}.txt").write_text(
                    f"{class_id} 0.5 0.5 0.2 0.2\n"
                )
            data = root / "data.yaml"
            data.write_text(
                yaml.safe_dump(
                    {
                        "path": str(root),
                        "val": "images/val",
                        "names": ["Ball", "GoalPost"],
                    }
                )
            )
            dataset = DFINEDataset(data, "val", image_size=32)
            fingerprint = dataset_fingerprint(dataset)

            manifest = create_subset_manifest(
                {HeadId.OBJECT: dataset},
                {HeadId.OBJECT: fingerprint},
                count_per_task=2,
                seed=17,
            )
            repeated = create_subset_manifest(
                {HeadId.OBJECT: dataset},
                {HeadId.OBJECT: fingerprint},
                count_per_task=2,
                seed=17,
            )
            subsets = subset_datasets_from_manifest(
                {HeadId.OBJECT: dataset},
                {HeadId.OBJECT: fingerprint},
                manifest,
            )

            self.assertEqual(manifest, repeated)
            self.assertEqual(len(subsets[HeadId.OBJECT]), 2)


class MultitaskComparisonTests(unittest.TestCase):
    def test_cli_uses_bounded_screening_default(self) -> None:
        result = CliRunner().invoke(compare_main, ["--help"])

        self.assertEqual(result.exit_code, 0)
        self.assertIn("[default: 200; x>=100]", result.output)

    def test_cross_pose_sufficient_statistics_apply_visibility_alpha(
        self,
    ) -> None:
        records = {
            "first": {
                "task": CROSS_POSE_PERSON_ON_ROBOT,
                "image_key": "first",
                "predictions": {
                    "base_scores": [0.8, 0.4],
                    "mean_visibility": [0.25, 1.0],
                },
                "targets": {"scores": []},
            },
            "second": {
                "task": CROSS_POSE_PERSON_ON_ROBOT,
                "image_key": "second",
                "predictions": {
                    "base_scores": [],
                    "mean_visibility": [],
                },
                "targets": {"scores": []},
            },
        }

        prepared, sizes = _prepare_cross_pose_scores(records, (0.0, 1.0))
        metrics = _compute_cross_pose_scores(
            prepared,
            {
                task: np.ones(count, dtype=np.int64)
                for task, count in sizes.items()
            },
        )

        prefix = CROSS_POSE_PERSON_ON_ROBOT
        self.assertEqual(metrics[f"{prefix}/images"], 2)
        self.assertAlmostEqual(
            metrics[f"{prefix}/visibility_alpha/0/max_score_mean"],
            0.4,
        )
        self.assertAlmostEqual(
            metrics[f"{prefix}/visibility_alpha/1/max_score_mean"],
            0.2,
        )
        self.assertEqual(
            metrics[f"{prefix}/visibility_alpha/1/detections_at_25_per_image"],
            0.5,
        )

    @staticmethod
    def _object_record(
        *,
        predictions: list[tuple[list[float], float, int]],
        targets: list[tuple[list[float], int]],
    ) -> dict[str, object]:
        return {
            "task": "object",
            "predictions": {
                "boxes_xyxy": [row[0] for row in predictions],
                "scores": [row[1] for row in predictions],
                "labels": [row[2] for row in predictions],
            },
            "targets": {
                "boxes_xyxy": [row[0] for row in targets],
                "labels": [row[1] for row in targets],
            },
        }

    def test_object_sufficient_statistics_match_faster_coco(self) -> None:
        record = {
            "task": "object",
            "predictions": {
                "boxes_xyxy": [[0, 0, 10, 10]],
                "scores": [0.9],
                "labels": [0],
            },
            "targets": {
                "boxes_xyxy": [[0, 0, 10, 10]],
                "labels": [0],
            },
        }

        canonical = _canonical_object_metrics([record])
        prepared = _prepare_object_ap([record]).compute(
            np.ones(1, dtype=np.int64)
        )

        self.assertAlmostEqual(canonical["object/map"], 1.0)
        self.assertAlmostEqual(prepared["map"], canonical["object/map"])
        self.assertAlmostEqual(
            prepared["map75"],
            canonical["object/map75"],
        )

    def test_absent_class_image_prediction_counts_as_global_ap_fp(self) -> None:
        records = [
            self._object_record(
                predictions=[([0, 0, 10, 10], 0.8, 0)],
                targets=[([0, 0, 10, 10], 0)],
            ),
            self._object_record(
                predictions=[([20, 20, 30, 30], 0.9, 0)],
                targets=[],
            ),
        ]

        canonical = _canonical_object_metrics(records)
        prepared = _prepare_object_ap(records).compute(
            np.ones(2, dtype=np.int64)
        )
        negatives = _prepare_negative_objects(records).compute(
            np.ones(2, dtype=np.int64)
        )

        self.assertAlmostEqual(prepared["map"], canonical["object/map"])
        self.assertAlmostEqual(canonical["object/map"], 0.5)
        self.assertEqual(negatives, 1.0)

    def test_object_cluster_weights_recompute_resampled_dataset(self) -> None:
        records = [
            self._object_record(
                predictions=[([0, 0, 10, 10], 0.8, 0)],
                targets=[([0, 0, 10, 10], 0)],
            ),
            self._object_record(
                predictions=[([20, 20, 30, 30], 0.9, 0)],
                targets=[],
            ),
        ]
        prepared = _prepare_object_ap(records)

        positive_only = prepared.compute(np.array([2, 0], dtype=np.int64))
        paired_sample = prepared.compute(np.array([1, 1], dtype=np.int64))

        self.assertEqual(positive_only["map"], 1.0)
        self.assertEqual(paired_sample["map"], 0.5)

    def test_cluster_bootstrap_couples_shared_paths_and_fixes_counts(
        self,
    ) -> None:
        records = {
            "field-a": {"task": "field_features", "path": "/shared/a.png"},
            "field-b": {"task": "field_features", "path": "/shared/b.png"},
            "object-a": {"task": "object", "path": "/shared/a.png"},
            "object-b": {"task": "object", "path": "/shared/b.png"},
            "object-c": {"task": "object", "path": "/object/c.png"},
            "person-a": {"task": "person_pose", "path": "/person/a.png"},
        }
        plan = _prepare_cluster_bootstrap(records)

        self.assertEqual(
            [(stratum.tasks, stratum.size) for stratum in plan.strata],
            [
                (("field_features", "object"), 2),
                (("object",), 1),
                (("person_pose",), 1),
            ],
        )
        generator = np.random.default_rng(17)
        for _ in range(20):
            weights = plan.sample(generator)
            self.assertEqual(int(weights["field_features"].sum()), 2)
            self.assertEqual(int(weights["object"].sum()), 3)
            self.assertEqual(int(weights["person_pose"].sum()), 1)
            np.testing.assert_array_equal(
                weights["field_features"],
                weights["object"][:2],
            )

    def test_cluster_bootstrap_preserves_cross_task_covariance(self) -> None:
        records = {
            "field-a": {"task": "field_features", "path": "/shared/a.png"},
            "field-b": {"task": "field_features", "path": "/shared/b.png"},
            "object-a": {"task": "object", "path": "/shared/a.png"},
            "object-b": {"task": "object", "path": "/shared/b.png"},
        }
        plan = _prepare_cluster_bootstrap(records)
        contribution = np.asarray([1.0, -1.0])
        aligned = []
        anti_aligned = []
        generator = np.random.default_rng(23)
        for _ in range(200):
            weights = plan.sample(generator)
            object_delta = float(weights["object"] @ contribution)
            aligned.append(
                object_delta + float(weights["field_features"] @ contribution)
            )
            anti_aligned.append(
                object_delta - float(weights["field_features"] @ contribution)
            )

        self.assertGreater(float(np.std(aligned)), 0.0)
        np.testing.assert_array_equal(anti_aligned, np.zeros(200))

    def test_cluster_bootstrap_legacy_keys_do_not_false_match(self) -> None:
        plan = _prepare_cluster_bootstrap(
            {
                "field_features:same": {"task": "field_features"},
                "object:same": {"task": "object"},
            }
        )

        self.assertEqual(plan.legacy_identity_records, 2)
        self.assertEqual(
            [(stratum.tasks, stratum.size) for stratum in plan.strata],
            [(("field_features",), 1), (("object",), 1)],
        )

    def test_cluster_bootstrap_rejects_duplicate_task_path(self) -> None:
        with self.assertRaisesRegex(
            ValueError,
            "Duplicate canonical image within task",
        ):
            _prepare_cluster_bootstrap(
                {
                    "object:a": {"task": "object", "path": "/same.png"},
                    "object:b": {"task": "object", "path": "/same.png"},
                }
            )

    def test_field_resampling_recomputes_ap_and_localization(self) -> None:
        records = [
            {
                "task": "field_features",
                "predictions": {"points": [[10, 10, 0.8, 0]]},
                "targets": {
                    "points": [[10, 10]],
                    "labels": [0],
                    "boxes_cxcywh": [[10, 10, 10, 10]],
                },
            },
            {
                "task": "field_features",
                "predictions": {"points": [[20, 20, 0.9, 0]]},
                "targets": {
                    "points": [],
                    "labels": [],
                    "boxes_cxcywh": [],
                },
            },
        ]

        canonical = _canonical_field_metrics(records, normalization=1.0)
        prepared = _prepare_field_ap(records, 1.0).compute(
            np.ones(2, dtype=np.int64)
        )
        localization = _prepare_field_localization(records).compute(
            np.ones(2, dtype=np.int64)
        )

        self.assertAlmostEqual(prepared["map"], canonical["field/map"])
        self.assertAlmostEqual(prepared["map"], 0.5)
        self.assertEqual(localization["field/localization/pck_5px"], 1.0)
        self.assertEqual(
            localization["field/localization/matched_fraction"],
            1.0,
        )
        self.assertEqual(
            localization["field/localization/median_error_px"],
            0.0,
        )

    def test_field_resampling_preserves_tied_detection_order(self) -> None:
        record = {
            "task": "field_features",
            "predictions": {
                "points": [
                    [10, 10, 0.9, 0],
                    [100, 100, 0.9, 0],
                ]
            },
            "targets": {
                "points": [[10, 10]],
                "labels": [0],
                "boxes_cxcywh": [[10, 10, 10, 10]],
            },
        }

        canonical = _canonical_field_metrics(
            [record, record],
            normalization=1.0,
        )
        prepared = _prepare_field_ap([record], 1.0).compute(
            np.array([2], dtype=np.int64)
        )

        self.assertAlmostEqual(prepared["map"], canonical["field/map"])
        self.assertAlmostEqual(prepared["map"], 0.8349834983498358)

    def test_field_ap_preserves_exact_recall_boundary(self) -> None:
        records = []
        for index in range(100):
            predictions = (
                {"points": [[10, 10, 1 - index / 1000, 0]]}
                if index < 35
                else {"points": []}
            )
            records.append(
                {
                    "task": "field_features",
                    "predictions": predictions,
                    "targets": {
                        "points": [[10, 10]],
                        "labels": [0],
                        "boxes_cxcywh": [[10, 10, 10, 10]],
                    },
                }
            )

        canonical = _canonical_field_metrics(records, normalization=1.0)
        prepared = _prepare_field_ap(records, 1.0).compute(
            np.ones(len(records), dtype=np.int64)
        )
        expected = 36 / 101

        self.assertAlmostEqual(canonical["field/map"], expected, places=15)
        self.assertAlmostEqual(prepared["map"], expected, places=15)

    def test_comparator_threshold_grids_match_metric_construction(self) -> None:
        np.testing.assert_array_equal(
            _MATCH_THRESHOLDS,
            np.asarray(OKS_THRESHOLDS, dtype=np.float64),
        )
        np.testing.assert_array_equal(
            _RECALL_THRESHOLDS,
            np.asarray(
                [level / 100 for level in range(101)],
                dtype=np.float64,
            ),
        )

    def test_pose_sufficient_statistics_match_pose_accumulator(self) -> None:
        keypoints = [[float(index), float(index), 1.0] for index in range(17)]
        record = {
            "task": "person_pose",
            "predictions": {
                "scores": [0.9],
                "keypoints": [keypoints],
            },
            "targets": {
                "boxes_cxcywh": [[8, 8, 20, 20]],
                "keypoints": [keypoints],
                "visibility": [[True] * 17],
            },
        }

        canonical = _canonical_pose_metrics([record], prefix="person")
        prepared = _prepare_pose_ap([record], prefix="person").compute(
            np.ones(1, dtype=np.int64)
        )

        self.assertAlmostEqual(prepared["map"], canonical["person/map"])
        self.assertEqual(prepared["map"], 1.0)

    def test_compatibility_rejects_different_dataset_fingerprint(self) -> None:
        config = dict.fromkeys(
            (
                "backend",
                "height",
                "width",
                "confidence",
                "field_normalization",
                "robot_visibility_alphas",
                "max_batches",
                "tasks",
                "subset_manifest",
            )
        )
        baseline = {"dataset_fingerprints": {"object": {"digest": "a"}}}
        candidate = {"dataset_fingerprints": {"object": {"digest": "b"}}}

        with self.assertRaises(ValueError):
            compatibility_report(config, config, baseline, candidate)

    def test_compatibility_rejects_robot_score_calibration_change(self) -> None:
        baseline_config = dict.fromkeys(
            (
                "backend",
                "height",
                "width",
                "confidence",
                "field_normalization",
                "robot_visibility_alphas",
                "max_batches",
                "tasks",
                "subset_manifest",
            )
        )
        candidate_config = dict(baseline_config)
        baseline_config["robot_visibility_alphas"] = [0.0]
        candidate_config["robot_visibility_alphas"] = [0.5]
        metadata = {"dataset_fingerprints": {"robot_pose": {"digest": "a"}}}

        with self.assertRaises(ValueError):
            compatibility_report(
                baseline_config,
                candidate_config,
                metadata,
                metadata,
            )

        del baseline_config["robot_visibility_alphas"]
        candidate_config["robot_visibility_alphas"] = [0.0]
        compatible = compatibility_report(
            baseline_config,
            candidate_config,
            metadata,
            metadata,
        )
        self.assertTrue(compatible["compatible"])

    def test_compatibility_rejects_person_alpha_change(self) -> None:
        config = dict.fromkeys(
            (
                "backend",
                "height",
                "width",
                "confidence",
                "field_normalization",
                "person_visibility_alphas",
                "robot_visibility_alphas",
                "max_batches",
                "tasks",
                "subset_manifest",
            )
        )
        candidate_config = dict(config)
        config["person_visibility_alphas"] = [0.0]
        candidate_config["person_visibility_alphas"] = [0.25]
        metadata = {"dataset_fingerprints": {"person_pose": {"digest": "a"}}}

        with self.assertRaises(ValueError):
            compatibility_report(
                config,
                candidate_config,
                metadata,
                metadata,
            )

        del config["person_visibility_alphas"]
        candidate_config["person_visibility_alphas"] = [0.0]
        compatible = compatibility_report(
            config,
            candidate_config,
            metadata,
            metadata,
        )
        self.assertTrue(compatible["compatible"])

    def test_promotion_gates_include_score_and_negative_images(self) -> None:
        def row(
            *,
            baseline: float = 0.5,
            delta: float = 0.0,
            lower: float = 0.0,
            upper: float = 0.0,
        ) -> dict[str, float]:
            return {
                "baseline": baseline,
                "delta": delta,
                "ci95_lower": lower,
                "ci95_upper": upper,
            }

        rows = {
            "object/map": row(delta=0.004, lower=0.001),
            "field/map": row(delta=0.001, lower=-0.004),
            "field/strict_map": row(delta=0.001, lower=-0.004),
            "field/localization/pck_5px": row(lower=-0.009),
            "field/localization/matched_fraction": row(lower=-0.009),
            "field/localization/median_error_px": row(upper=0.5),
            "person/map": row(delta=-0.01),
            "robot/map": row(delta=-0.01),
            "object/negative_fp_per_image_at_25": row(
                baseline=0.2,
                delta=0.05,
            ),
        }
        score = {"delta": 0.003, "ci95_lower": 0.0001}

        gates = _promotion_gates(rows, score)

        self.assertTrue(all(gates.values()))
        rows["object/negative_fp_per_image_at_25"]["delta"] = 0.05001
        failed = _promotion_gates(rows, score)
        self.assertFalse(
            failed["object/negative_fp_per_image_at_25/noninferior"]
        )

    def test_compare_recomputes_primary_score_for_paired_records(self) -> None:
        keypoints17 = [[float(index), float(index), 1.0] for index in range(17)]
        keypoints14 = [[float(index), float(index), 1.0] for index in range(14)]
        object_positive = self._object_record(
            predictions=[([0, 0, 10, 10], 0.9, 0)],
            targets=[([0, 0, 10, 10], 0)],
        )
        object_negative = self._object_record(predictions=[], targets=[])
        person = {
            "task": "person_pose",
            "predictions": {"scores": [0.9], "keypoints": [keypoints17]},
            "targets": {
                "boxes_cxcywh": [[8, 8, 20, 20]],
                "keypoints": [keypoints17],
                "visibility": [[True] * 17],
            },
        }
        robot = {
            "task": "robot_pose",
            "predictions": {"scores": [0.9], "keypoints": [keypoints14]},
            "targets": {
                "boxes_cxcywh": [[7, 7, 20, 20]],
                "keypoints": [keypoints14],
                "visibility": [[True] * 14],
            },
        }
        baseline_field = {
            "task": "field_features",
            "predictions": {"points": [[100, 100, 0.9, 0]]},
            "targets": {
                "points": [[10, 10]],
                "labels": [0],
                "boxes_cxcywh": [[10, 10, 10, 10]],
            },
        }
        candidate_field = {
            **baseline_field,
            "predictions": {"points": [[10, 10, 0.9, 0]]},
        }
        baseline_records = [
            object_positive,
            object_negative,
            person,
            robot,
            baseline_field,
        ]
        candidate_records = [
            object_positive,
            object_negative,
            person,
            robot,
            candidate_field,
        ]

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            baseline_dir = root / "baseline"
            candidate_dir = root / "candidate"
            tasks = (
                "object",
                "object",
                "person_pose",
                "robot_pose",
                "field_features",
            )
            keys = (
                "object:positive",
                "object:negative",
                "person_pose:positive",
                "robot_pose:positive",
                "field_features:positive",
            )
            paths = (
                "/shared/positive.png",
                "/object/negative.png",
                "/person/positive.png",
                "/robot/positive.png",
                "/shared/positive.png",
            )
            config = {
                "backend": "pytorch-rgb",
                "height": 448,
                "width": 544,
                "confidence": 0.001,
                "field_normalization": 1.0,
                "robot_visibility_alphas": [0.0],
                "max_batches": None,
                "tasks": sorted(set(tasks)),
                "subset_manifest": None,
                "cross_pose_negatives": True,
                "cross_pose_visibility_alphas": [0.0, 1.0],
            }
            metadata = {
                "dataset_fingerprints": {
                    task: {"digest": task} for task in set(tasks)
                }
            }
            for destination, records in (
                (baseline_dir, baseline_records),
                (candidate_dir, candidate_records),
            ):
                destination.mkdir()
                (destination / "metrics.json").write_text("{}")
                (destination / "config.json").write_text(json.dumps(config))
                (destination / "metadata.json").write_text(json.dumps(metadata))
                with gzip.open(
                    destination / "predictions.jsonl.gz",
                    "wt",
                    encoding="utf-8",
                ) as file:
                    for key, task, path, record in zip(
                        keys,
                        tasks,
                        paths,
                        records,
                        strict=True,
                    ):
                        file.write(
                            json.dumps(
                                {
                                    **record,
                                    "image_key": key,
                                    "task": task,
                                    "path": path,
                                }
                            )
                            + "\n"
                        )
                visibility = 1.0 if destination == baseline_dir else 0.25
                with gzip.open(
                    destination / "cross_pose_predictions.jsonl.gz",
                    "wt",
                    encoding="utf-8",
                ) as file:
                    for cross_task in (
                        CROSS_POSE_PERSON_ON_ROBOT,
                        CROSS_POSE_ROBOT_ON_PERSON,
                    ):
                        file.write(
                            json.dumps(
                                {
                                    "task": cross_task,
                                    "image_key": f"{cross_task}:paired",
                                    "predictions": {
                                        "base_scores": [0.8],
                                        "mean_visibility": [visibility],
                                    },
                                    "targets": {"scores": []},
                                }
                            )
                            + "\n"
                        )

            report = compare(
                baseline_dir,
                candidate_dir,
                iterations=100,
                seed=17,
            )
            config_without_cross = {
                key: value
                for key, value in config.items()
                if not key.startswith("cross_pose_")
            }
            for destination in (baseline_dir, candidate_dir):
                (destination / "cross_pose_predictions.jsonl.gz").unlink()
                (destination / "config.json").write_text(
                    json.dumps(config_without_cross)
                )
            report_without_cross = compare(
                baseline_dir,
                candidate_dir,
                iterations=100,
                seed=17,
            )
            candidate_predictions = candidate_dir / "predictions.jsonl.gz"
            with gzip.open(
                candidate_predictions,
                "rt",
                encoding="utf-8",
            ) as file:
                candidate_rows = [json.loads(line) for line in file]
            candidate_rows[0]["path"] = "/mismatched.png"
            with gzip.open(
                candidate_predictions,
                "wt",
                encoding="utf-8",
            ) as file:
                for row in candidate_rows:
                    file.write(json.dumps(row) + "\n")
            with self.assertRaisesRegex(
                ValueError,
                "Prediction pair canonical image differs",
            ):
                compare(
                    baseline_dir,
                    candidate_dir,
                    iterations=100,
                    seed=17,
                )

        score = cast(dict[str, float], report["primary_score"])
        self.assertGreater(score["delta"], 0.003)
        self.assertGreater(score["ci95_lower"], 0.0)
        self.assertTrue(report["passes_all_gates"])
        bootstrap = cast(dict[str, object], report["bootstrap"])
        self.assertEqual(report["version"], 3)
        canonical_counts = cast(
            dict[str, int],
            bootstrap["task_image_counts"],
        )
        cross_counts = cast(
            dict[str, int],
            bootstrap["cross_pose_image_counts"],
        )
        self.assertFalse(
            any(task.startswith("cross_pose/") for task in canonical_counts)
        )
        self.assertEqual(cross_counts[CROSS_POSE_PERSON_ON_ROBOT], 1)
        self.assertEqual(bootstrap["canonical_cluster_count"], 4)
        self.assertEqual(bootstrap["cross_task_cluster_count"], 1)
        self.assertIn(
            {
                "tasks": ["field_features", "object"],
                "clusters": 1,
            },
            bootstrap["task_membership_strata"],
        )
        rows = {
            row["metric"]: row
            for row in cast(list[dict[str, object]], report["metrics"])
        }
        cross_metric = (
            f"{CROSS_POSE_PERSON_ON_ROBOT}/visibility_alpha/1/max_score_mean"
        )
        self.assertAlmostEqual(float(rows[cross_metric]["delta"]), -0.6)
        self.assertEqual(
            report["primary_score"],
            report_without_cross["primary_score"],
        )
        rows_without_cross = {
            row["metric"]: row
            for row in cast(
                list[dict[str, object]],
                report_without_cross["metrics"],
            )
        }
        self.assertEqual(rows["field/map"], rows_without_cross["field/map"])

    def test_compare_rejects_one_sided_cross_pose_records(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            baseline = Path(directory) / "baseline"
            candidate = Path(directory) / "candidate"
            config = {
                "cross_pose_negatives": True,
                "cross_pose_visibility_alphas": [0.0, 1.0],
            }
            metadata = {"dataset_fingerprints": {}}
            for destination in (baseline, candidate):
                destination.mkdir()
                (destination / "metrics.json").write_text("{}")
                (destination / "config.json").write_text(json.dumps(config))
                (destination / "metadata.json").write_text(json.dumps(metadata))
            with gzip.open(
                baseline / "cross_pose_predictions.jsonl.gz",
                "wt",
                encoding="utf-8",
            ):
                pass

            with self.assertRaisesRegex(
                ValueError,
                "exactly one run has cross_pose_predictions",
            ):
                compare(baseline, candidate, iterations=100, seed=17)


if __name__ == "__main__":
    unittest.main()
