import os
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from typing import Literal, cast
from unittest.mock import patch

import torch
import torch.nn as nn
from torch.utils.data import DataLoader, Dataset, DistributedSampler

from model.hydra import Hydra
from ultralytics_dfine.engine import (
    MultiTaskTrainer,
    MultiTaskTrainingConfig,
    multitask_collate,
    stage_training_config,
)
from ultralytics_dfine.engine.multitask_trainer import _parameter_role
from ultralytics_dfine.loss import CriterionResult
from ultralytics_dfine.nn import DFINEDetectionModel, DFINEMultiTaskModel
from ultralytics_dfine.schemas import HeadId
from utils.export_hydra import HydraWrapper, export_onnx
from utils.model_naming import ModelFamily, TaskType


class _TaskDataset(Dataset[tuple[torch.Tensor, dict[str, object]]]):
    def __init__(self, task: HeadId, length: int = 1) -> None:
        self.task = task
        self.length = length

    def __len__(self) -> int:
        return self.length

    def __getitem__(self, index: int) -> tuple[torch.Tensor, dict[str, object]]:
        keypoint_count = 17 if self.task == HeadId.PERSON_POSE else 14
        class_id = 7 if self.task == HeadId.PERSON_POSE else 4
        target: dict[str, object] = {
            "labels": torch.tensor([class_id]),
            "boxes": torch.tensor([[0.5, 0.5, 0.4, 0.6]]),
            "valid_detection_classes": torch.nn.functional.one_hot(
                torch.tensor(class_id),
                num_classes=8,
            ).bool(),
            "schema_id": str(self.task),
            "orig_size": torch.tensor([128, 128]),
            "image_id": torch.tensor(index),
            "path": Path("synthetic.png"),
        }
        if self.task in {HeadId.PERSON_POSE, HeadId.ROBOT_POSE}:
            keypoints = torch.full((1, keypoint_count, 3), 0.5)
            keypoints[..., 2] = 1
            target["keypoints"] = keypoints
            target["visibility"] = keypoints[..., 2] > 0
        if self.task == HeadId.FIELD_FEATURES:
            target["labels"] = torch.tensor([1])
            target["valid_detection_classes"] = torch.tensor(
                [False, True, False, False, False, False, False, False]
            )
            target["point_labels"] = torch.tensor([0])
            target["points"] = torch.tensor([[0.5, 0.5]])
        if self.task == HeadId.OBJECT:
            target["labels"] = torch.tensor([0])
            target["valid_detection_classes"] = torch.tensor(
                [True, True, True, True, True, True, True, False]
            )
        return torch.rand(3, 128, 128), target


class _LightweightModel(nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.weight = nn.Parameter(torch.tensor(1.0))
        self.detector = SimpleNamespace(
            nc=8,
            names=[
                "Ball",
                "GoalPost",
                "LSpot",
                "PenaltySpot",
                "Robot",
                "TSpot",
                "XSpot",
                "Person",
            ],
        )
        self.schemas = {}

    def forward(
        self,
        images: torch.Tensor,
        targets: list[dict[str, object]],
        active_head: HeadId | None = None,
    ) -> dict[str, dict[str, object]]:
        del targets, active_head
        return {"weight": {"value": self.weight * images.mean()}}

    def forward_deploy(self, images: torch.Tensor) -> dict[str, torch.Tensor]:
        batch = images.shape[0]
        device = images.device
        return {
            "object_output": torch.zeros(batch, 300, 6, device=device),
            "person_pose_output": torch.zeros(batch, 300, 57, device=device),
            "robot_pose_output": torch.zeros(batch, 300, 14, 3, device=device),
            "field_feature_output": torch.zeros(batch, 300, 4, device=device),
        }

    def forward_deploy_task(
        self,
        images: torch.Tensor,
        task: HeadId,
    ) -> dict[str, torch.Tensor]:
        outputs = self.forward_deploy(images)
        required = {
            HeadId.OBJECT: ("object_output",),
            HeadId.PERSON_POSE: ("person_pose_output",),
            HeadId.ROBOT_POSE: ("object_output", "robot_pose_output"),
            HeadId.FIELD_FEATURES: ("field_feature_output",),
        }[task]
        return {name: outputs[name] for name in required}


class _LightweightCriterion(nn.Module):
    def forward(
        self,
        outputs: dict[str, dict[str, object]],
        targets: list[dict[str, object]],
        task: HeadId,
    ) -> CriterionResult:
        del targets, task
        value = outputs["weight"]["value"]
        if not isinstance(value, torch.Tensor):
            raise TypeError
        return CriterionResult({"smoke": value.square()}, [])


def _loader(task: HeadId) -> DataLoader:
    return DataLoader(
        _TaskDataset(task),
        batch_size=1,
        collate_fn=multitask_collate,
    )


class MultiTaskTrainingSmokeTests(unittest.TestCase):
    def test_classifier_role_matches_actual_decoder_parameter_names(
        self,
    ) -> None:
        self.assertEqual(
            _parameter_role("detector.core.model.decoder.class_embed.2.weight"),
            "classifiers",
        )

    def test_stage_presets_progressively_unfreeze_core(self) -> None:
        stage_one = stage_training_config(
            1,
            output_dir=Path("stage-one"),
            epochs=1,
        )
        stage_three = stage_training_config(
            3,
            output_dir=Path("stage-three"),
            epochs=1,
        )

        self.assertIn("heads", stage_one.trainable_roles)
        self.assertIn("proposal", stage_one.trainable_roles)
        self.assertNotIn("decoder", stage_one.trainable_roles)
        self.assertIn("decoder", stage_three.trainable_roles)
        self.assertIn("encoder_last", stage_three.trainable_roles)
        self.assertIn("backbone_last", stage_three.trainable_roles)
        self.assertLess(
            stage_three.role_learning_rates["backbone_last"],
            stage_three.role_learning_rates["decoder"],
        )

    def test_full_training_validation_and_all_renders(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output_dir = Path(directory)
            model = _LightweightModel()
            trainer = MultiTaskTrainer(
                cast(DFINEMultiTaskModel, model),
                {task: _loader(task) for task in HeadId},
                {task: _loader(task) for task in HeadId},
                MultiTaskTrainingConfig(
                    output_dir=output_dir,
                    epochs=2,
                    steps_per_epoch=4,
                    learning_rate=0.1,
                ),
            )
            trainer.criterion = _LightweightCriterion()

            result = trainer.fit()

            history = cast("list[dict[str, object]]", result["history"])
            self.assertEqual(len(history), 2)
            self.assertLess(model.weight.item(), 1.0)
            self.assertTrue((output_dir / "last.pt").is_file())
            self.assertTrue((output_dir / "best.pt").is_file())
            checkpoint = torch.load(output_dir / "last.pt", weights_only=True)
            self.assertIn("ema", checkpoint)
            self.assertIn("inference_model", checkpoint)
            self.assertTrue(
                torch.isfinite(torch.tensor(checkpoint["best_fitness"]))
            )
            render_dir = output_dir / "validation" / "epoch_001"
            for name in (
                "object_output",
                "person_pose_output",
                "robot_pose_output",
                "field_feature_output",
            ):
                path = render_dir / f"{name}.jpg"
                self.assertTrue(path.is_file())
                self.assertGreater(path.stat().st_size, 0)

            resumed_model = _LightweightModel()
            resumed = MultiTaskTrainer(
                cast(DFINEMultiTaskModel, resumed_model),
                {task: _loader(task) for task in HeadId},
                {task: _loader(task) for task in HeadId},
                MultiTaskTrainingConfig(
                    output_dir=output_dir,
                    epochs=3,
                    steps_per_epoch=4,
                    learning_rate=0.1,
                ),
            )
            resumed.criterion = _LightweightCriterion()
            resumed.resume(output_dir / "last.pt")
            resumed_result = resumed.fit()
            resumed_history = cast(
                "list[dict[str, object]]",
                resumed_result["history"],
            )
            self.assertEqual(len(resumed_history), 1)
            self.assertEqual(resumed.global_step, 12)

    def test_optimizer_uses_explicit_parameter_aware_decay(self) -> None:
        model = _LightweightModel()
        model.matrix = nn.Parameter(torch.ones(2, 2))
        trainer = MultiTaskTrainer(
            cast(DFINEMultiTaskModel, model),
            {HeadId.OBJECT: _loader(HeadId.OBJECT)},
            {HeadId.OBJECT: _loader(HeadId.OBJECT)},
            MultiTaskTrainingConfig(
                output_dir=Path("unused"),
                weight_decay=1e-4,
            ),
        )

        decays = {
            group["weight_decay"] for group in trainer.optimizer.param_groups
        }

        self.assertEqual(decays, {0.0, 1e-4})
        self.assertTrue(
            all("base_lr" in group for group in trainer.optimizer.param_groups)
        )

    def test_validation_failure_keeps_resumable_checkpoint(self) -> None:
        class FakeRun:
            def __init__(self) -> None:
                self.id = "test-run"
                self.summary: dict[str, object] = {}
                self.finish_codes: list[int] = []

            def define_metric(self, *_args: object, **_kwargs: object) -> None:
                pass

            def log(self, *_args: object, **_kwargs: object) -> None:
                pass

            def log_artifact(
                self,
                *_args: object,
                **_kwargs: object,
            ) -> None:
                pass

            def finish(self, *, exit_code: int) -> None:
                self.finish_codes.append(exit_code)

        class FakeArtifact:
            def __init__(self, *_args: object, **_kwargs: object) -> None:
                pass

            def add_file(self, *_args: object, **_kwargs: object) -> None:
                pass

        fake_run = FakeRun()
        fake_wandb = SimpleNamespace(
            init=lambda **_kwargs: fake_run,
            Error=RuntimeError,
            Image=lambda *args, **kwargs: (args, kwargs),
            Artifact=FakeArtifact,
        )
        with tempfile.TemporaryDirectory() as directory:
            output_dir = Path(directory)
            model = _LightweightModel()
            trainer = MultiTaskTrainer(
                cast(DFINEMultiTaskModel, model),
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                MultiTaskTrainingConfig(
                    output_dir=output_dir,
                    epochs=1,
                    steps_per_epoch=1,
                    wandb_mode="online",
                ),
            )
            trainer.criterion = _LightweightCriterion()
            with (
                patch.dict("sys.modules", {"wandb": fake_wandb}),
                patch.object(
                    trainer,
                    "validate",
                    side_effect=RuntimeError("validation failed"),
                ),
                self.assertRaisesRegex(RuntimeError, "validation failed"),
            ):
                trainer.fit()

            checkpoint = output_dir / "last.pt"
            self.assertTrue(checkpoint.is_file())
            payload = torch.load(checkpoint, weights_only=True)
            self.assertEqual(payload["pending_validation_epoch"], 0)
            self.assertEqual(fake_run.finish_codes, [1])
            self.assertEqual(fake_run.summary["status"], "failed")

            resumed_model = _LightweightModel()
            resumed = MultiTaskTrainer(
                cast(DFINEMultiTaskModel, resumed_model),
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                MultiTaskTrainingConfig(
                    output_dir=output_dir,
                    epochs=1,
                    steps_per_epoch=1,
                ),
            )
            resumed.criterion = _LightweightCriterion()
            resumed.resume(checkpoint)
            with patch.object(
                resumed,
                "validate",
                return_value={"renders": {}, "object/map": 0.5},
            ) as validate:
                resumed.fit()

            validate.assert_called_once_with(0)
            completed = torch.load(checkpoint, weights_only=True)
            self.assertIsNone(completed["pending_validation_epoch"])
            self.assertEqual(resumed.global_step, 1)


@unittest.skipUnless(
    os.environ.get("RUN_DFINE_DDP_SMOKE") == "1",
    "run through torchrun with RUN_DFINE_DDP_SMOKE=1",
)
class DistributedTrainingSmokeTests(unittest.TestCase):
    def test_two_process_training(self) -> None:
        dataset = _TaskDataset(HeadId.OBJECT, length=4)
        loader = DataLoader(
            dataset,
            batch_size=1,
            sampler=DistributedSampler(
                dataset,
                num_replicas=int(os.environ["WORLD_SIZE"]),
                rank=int(os.environ["RANK"]),
            ),
            collate_fn=multitask_collate,
        )
        model = _LightweightModel()
        output_dir = Path(os.environ["DFINE_DDP_OUTPUT_DIR"])
        trainer = MultiTaskTrainer(
            cast(DFINEMultiTaskModel, model),
            {HeadId.OBJECT: loader},
            {HeadId.OBJECT: loader},
            MultiTaskTrainingConfig(
                output_dir=output_dir,
                epochs=1,
                steps_per_epoch=2,
                learning_rate=0.1,
                device=os.environ.get("DFINE_DDP_DEVICE", "cpu"),
            ),
        )
        trainer.criterion = _LightweightCriterion()

        result = trainer.fit()

        history = cast("list[dict[str, object]]", result["history"])
        if trainer.rank == 0:
            self.assertEqual(len(history), 1)
            self.assertTrue((output_dir / "last.pt").is_file())
        else:
            self.assertEqual(history, [])
        self.assertLess(model.weight.item(), 1.0)


@unittest.skipUnless(
    os.environ.get("RUN_DFINE_FULL_TRAINING") == "1",
    "set RUN_DFINE_FULL_TRAINING=1 for the actual D-FINE training run",
)
class ActualDFINETrainingTests(unittest.TestCase):
    def test_train_validate_and_render_every_output(self) -> None:
        names = [
            "Ball",
            "GoalPost",
            "LSpot",
            "PenaltySpot",
            "Robot",
            "TSpot",
            "XSpot",
        ]
        detector = DFINEDetectionModel.from_pretrained(names)
        detector.append_detection_class("Person")
        model = DFINEMultiTaskModel(detector)
        device = os.environ.get("DFINE_TEST_DEVICE", "cuda")
        with tempfile.TemporaryDirectory() as directory:
            trainer = MultiTaskTrainer(
                model,
                {task: _loader(task) for task in HeadId},
                {task: _loader(task) for task in HeadId},
                MultiTaskTrainingConfig(
                    output_dir=Path(directory),
                    epochs=1,
                    steps_per_epoch=4,
                    learning_rate=1e-6,
                    device=device,
                    wandb_mode=cast(
                        "Literal['online', 'offline', 'disabled']",
                        os.environ.get("DFINE_TEST_WANDB_MODE", "disabled"),
                    ),
                    wandb_log_interval=1,
                ),
            )

            result = trainer.fit()

            history = cast("list[dict[str, object]]", result["history"])
            validation = cast("dict[str, object]", history[0]["validation"])
            renders = cast("dict[str, str]", validation["renders"])
            self.assertEqual(len(renders), 16)
            for metric in (
                "object/map",
                "person/map",
                "robot/map",
                "field/map",
            ):
                self.assertIn(metric, validation)
            checkpoint = Path(directory) / "last.pt"
            restored = DFINEMultiTaskModel.from_checkpoint(checkpoint)
            restored.to(device).eval()
            deployed = restored.forward_deploy(
                torch.rand(1, 3, 128, 128, device=device)
            )
            self.assertEqual(len(deployed), 4)
            hydra = Hydra(
                "dfine-s",
                {TaskType.OBJECT: checkpoint},
                number_of_frozen_modules=1,
                family=ModelFamily.DFINE,
            ).to(device)
            hydra_outputs = hydra(torch.rand(1, 3, 128, 128, device=device))
            self.assertEqual(len(hydra_outputs), 4)
            hydra.eval()
            wrapper = HydraWrapper(
                hydra,
                {TaskType.OBJECT: checkpoint},
            ).to(device)
            export_path = Path(directory) / "multitask.onnx"
            export_onnx(
                wrapper,
                torch.rand(1, 3, 128, 128, device=device),
                export_path,
                hydra.deployment_output_names,
                17,
                with_nv12=False,
                static_shapes=True,
            )
            self.assertTrue(export_path.is_file())


if __name__ == "__main__":
    unittest.main()
