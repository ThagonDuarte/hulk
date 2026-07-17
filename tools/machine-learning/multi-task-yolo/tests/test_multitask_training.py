import json
import os
import tempfile
import unittest
from collections import Counter
from pathlib import Path
from types import SimpleNamespace
from typing import Literal, cast
from unittest.mock import Mock, patch

import torch
import torch.nn as nn
from click.testing import CliRunner
from torch.utils.data import DataLoader, Dataset, DistributedSampler

from model.hydra import Hydra
from model.train_multitask import (
    _profile_training_tasks,
)
from model.train_multitask import (
    main as train_multitask_main,
)
from ultralytics_dfine.config import MultiTaskLossConfig
from ultralytics_dfine.engine import (
    MultiTaskTrainer,
    MultiTaskTrainingConfig,
    multitask_collate,
    stage_training_config,
)
from ultralytics_dfine.engine.multitask_trainer import (
    _broadcast_module_state,
    _merge_validation_shards,
    _ModelEMA,
    _parameter_role,
    _validation_loaders_for_rank,
)
from ultralytics_dfine.engine.multitask_validator import MultiTaskValidator
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


class _FieldOnlyModel(_LightweightModel):
    def __init__(self) -> None:
        super().__init__()
        self.field_feature_head = nn.Sequential(
            nn.Linear(3, 4),
            nn.GELU(),
            nn.Linear(4, 1),
        )

    def forward(
        self,
        images: torch.Tensor,
        targets: list[dict[str, object]],
        active_head: HeadId | None = None,
    ) -> dict[str, dict[str, object]]:
        del targets, active_head
        pooled = images.mean(dim=(-2, -1))
        value = self.field_feature_head(pooled).mean()
        return {"weight": {"value": value}}


def _loader(task: HeadId) -> DataLoader:
    return DataLoader(
        _TaskDataset(task),
        batch_size=1,
        collate_fn=multitask_collate,
    )


class MultiTaskTrainingSmokeTests(unittest.TestCase):
    def test_epoch_logs_global_cross_negative_eligibility_counts(self) -> None:
        class EligibleRobotDataset(_TaskDataset):
            def __getitem__(
                self,
                index: int,
            ) -> tuple[torch.Tensor, dict[str, object]]:
                image, target = super().__getitem__(index)
                target["person_negative_verified"] = index < 2
                target["person_negative_eligible"] = index == 0
                return image, target

        model = _LightweightModel()
        model.loss_config = MultiTaskLossConfig(
            robot_batch_person_detector_negative_weight=0.25,
        )
        loader = DataLoader(
            EligibleRobotDataset(HeadId.ROBOT_POSE, length=2),
            batch_size=2,
            collate_fn=multitask_collate,
        )
        trainer = MultiTaskTrainer(
            cast(DFINEMultiTaskModel, model),
            {HeadId.ROBOT_POSE: loader},
            {HeadId.ROBOT_POSE: _loader(HeadId.ROBOT_POSE)},
            MultiTaskTrainingConfig(
                output_dir=Path("unused"),
                steps_per_epoch=1,
            ),
        )
        trainer.criterion = _LightweightCriterion()

        trainer.train_epoch()

        self.assertEqual(
            trainer.last_epoch_diagnostics["cross_negative/robot_records_seen"],
            2,
        )
        self.assertEqual(
            trainer.last_epoch_diagnostics[
                "cross_negative/robot_person_verified_seen"
            ],
            2,
        )
        self.assertEqual(
            trainer.last_epoch_diagnostics[
                "cross_negative/robot_person_detector_applied_records"
            ],
            1,
        )

    def test_epoch_logs_person_robot_eligibility_counts(self) -> None:
        class EligiblePersonDataset(_TaskDataset):
            def __getitem__(
                self,
                index: int,
            ) -> tuple[torch.Tensor, dict[str, object]]:
                image, target = super().__getitem__(index)
                target["robot_negative_reviewed"] = True
                target["robot_negative_excluded"] = index == 1
                target["robot_negative_verified"] = index == 0
                target["robot_negative_eligible"] = index == 0
                return image, target

        model = _LightweightModel()
        model.loss_config = MultiTaskLossConfig(
            person_batch_robot_detector_negative_weight=0.25,
        )
        loader = DataLoader(
            EligiblePersonDataset(HeadId.PERSON_POSE, length=2),
            batch_size=2,
            collate_fn=multitask_collate,
        )
        trainer = MultiTaskTrainer(
            cast(DFINEMultiTaskModel, model),
            {HeadId.PERSON_POSE: loader},
            {HeadId.PERSON_POSE: _loader(HeadId.PERSON_POSE)},
            MultiTaskTrainingConfig(
                output_dir=Path("unused"),
                steps_per_epoch=1,
            ),
        )
        trainer.criterion = _LightweightCriterion()

        trainer.train_epoch()

        self.assertEqual(
            trainer.last_epoch_diagnostics[
                "cross_negative/person_records_seen"
            ],
            2,
        )
        self.assertEqual(
            trainer.last_epoch_diagnostics[
                "cross_negative/person_robot_reviewed_seen"
            ],
            2,
        )
        self.assertEqual(
            trainer.last_epoch_diagnostics[
                "cross_negative/person_robot_excluded_seen"
            ],
            1,
        )
        self.assertEqual(
            trainer.last_epoch_diagnostics[
                "cross_negative/person_robot_verified_seen"
            ],
            1,
        )
        self.assertEqual(
            trainer.last_epoch_diagnostics[
                "cross_negative/person_robot_detector_applied_records"
            ],
            1,
        )

    def test_training_configuration_rejects_non_finite_values(self) -> None:
        invalid = (
            {"learning_rate": float("nan")},
            {"clip_max_norm": float("inf")},
            {"sampling_weights": {HeadId.OBJECT: float("nan")}},
            {"role_learning_rates": {"decoder": float("-inf")}},
        )
        for values in invalid:
            with (
                self.subTest(values=values),
                self.assertRaisesRegex(ValueError, "finite"),
            ):
                MultiTaskTrainingConfig(output_dir=Path("unused"), **values)

    def test_training_cli_rejects_non_finite_float_options(self) -> None:
        for option in ("--learning-rate", "--object-weight"):
            with self.subTest(option=option):
                result = CliRunner().invoke(
                    train_multitask_main,
                    [option, "nan"],
                )
                self.assertNotEqual(result.exit_code, 0)
                self.assertIn("finite", result.output)

    def test_validation_tasks_are_sharded_in_stable_schema_order(self) -> None:
        loaders = {task: _loader(task) for task in reversed(tuple(HeadId))}

        rank_zero = _validation_loaders_for_rank(
            loaders,
            rank=0,
            world_size=2,
        )
        rank_one = _validation_loaders_for_rank(
            loaders,
            rank=1,
            world_size=2,
        )

        self.assertEqual(
            tuple(rank_zero),
            (HeadId.OBJECT, HeadId.ROBOT_POSE),
        )
        self.assertEqual(
            tuple(rank_one),
            (HeadId.PERSON_POSE, HeadId.FIELD_FEATURES),
        )

    def test_validation_shards_merge_metrics_and_renders_deterministically(
        self,
    ) -> None:
        merged = _merge_validation_shards(
            [
                {
                    "renders": {"object/output": "rank-0.jpg"},
                    "robot/map": 0.2,
                    "object/map": 0.5,
                },
                {
                    "renders": {"field/output": "rank-1.jpg"},
                    "field/map": 0.7,
                    "person/map": 0.1,
                },
            ]
        )

        self.assertEqual(
            list(cast("dict[str, str]", merged["renders"])),
            ["field/output", "object/output"],
        )
        self.assertEqual(
            [name for name in merged if name != "renders"],
            ["field/map", "object/map", "person/map", "robot/map"],
        )
        with self.assertRaisesRegex(ValueError, "Duplicate validation metric"):
            _merge_validation_shards([{"object/map": 0.5}, {"object/map": 0.6}])

    def test_validation_ema_state_broadcasts_parameters_and_buffers(
        self,
    ) -> None:
        module = nn.BatchNorm1d(2)

        with patch("torch.distributed.broadcast") as broadcast:
            _broadcast_module_state(module, source=1)

        self.assertEqual(broadcast.call_count, 5)
        self.assertTrue(
            all(call.kwargs == {"src": 1} for call in broadcast.call_args_list)
        )

    def test_distributed_validation_gathers_complete_rank_zero_result(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as directory:
            trainer = MultiTaskTrainer(
                cast(DFINEMultiTaskModel, _LightweightModel()),
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                {task: _loader(task) for task in HeadId},
                MultiTaskTrainingConfig(output_dir=Path(directory)),
            )
            trainer.distributed = True
            trainer.world_size = 2
            trainer.rank = 0

            def gather(
                results: list[dict[str, object] | None],
                local: dict[str, object],
            ) -> None:
                results[0] = local
                results[1] = {
                    "rank": 1,
                    "validation": {
                        "renders": {"person/output": "rank-1.jpg"},
                        "person/map": 0.1,
                        "field/map": 0.7,
                    },
                }

            local = {
                "renders": {"object/output": "rank-0.jpg"},
                "object/map": 0.5,
                "robot/map": 0.2,
            }
            with (
                patch(
                    "ultralytics_dfine.engine.multitask_trainer."
                    "_broadcast_module_state"
                ) as synchronize,
                patch.object(
                    MultiTaskValidator,
                    "run",
                    autospec=True,
                    return_value=local,
                ) as run,
                patch(
                    "torch.distributed.all_gather_object",
                    side_effect=gather,
                ),
                patch("torch.distributed.broadcast_object_list") as broadcast,
            ):
                result = trainer.validate(3)

            synchronize.assert_called_once_with(trainer.ema.module)
            validator = run.call_args.args[0]
            self.assertEqual(
                tuple(validator.loaders),
                (HeadId.OBJECT, HeadId.ROBOT_POSE),
            )
            self.assertEqual(
                run.call_args.kwargs["render_dir"],
                Path(directory)
                / "validation"
                / "epoch_003"
                / "shards"
                / "rank-00",
            )
            broadcast.assert_called_once()
            self.assertEqual(result["object/map"], 0.5)
            self.assertEqual(result["person/map"], 0.1)
            self.assertEqual(result["field/map"], 0.7)
            self.assertEqual(result["robot/map"], 0.2)

    def test_distributed_validation_propagates_remote_rank_failure(
        self,
    ) -> None:
        trainer = MultiTaskTrainer(
            cast(DFINEMultiTaskModel, _LightweightModel()),
            {HeadId.OBJECT: _loader(HeadId.OBJECT)},
            {task: _loader(task) for task in HeadId},
            MultiTaskTrainingConfig(output_dir=Path("unused")),
        )
        trainer.distributed = True
        trainer.world_size = 2
        trainer.rank = 0

        def gather(
            results: list[dict[str, object] | None],
            local: dict[str, object],
        ) -> None:
            results[0] = local
            results[1] = {
                "rank": 1,
                "error_type": "RuntimeError",
                "error": "field validation failed",
            }

        with (
            patch(
                "ultralytics_dfine.engine.multitask_trainer."
                "_broadcast_module_state"
            ),
            patch.object(
                trainer,
                "_run_validation_loaders",
                return_value={"renders": {}, "object/map": 0.5},
            ),
            patch(
                "torch.distributed.all_gather_object",
                side_effect=gather,
            ),
            self.assertRaisesRegex(
                RuntimeError,
                "rank 1: RuntimeError: field validation failed",
            ),
        ):
            trainer.validate(0)

    def test_ema_copies_frozen_state_exactly(self) -> None:
        model = nn.Module()
        model.register_parameter("trainable", nn.Parameter(torch.tensor(1.0)))
        model.register_parameter(
            "frozen",
            nn.Parameter(torch.tensor(2.0), requires_grad=False),
        )
        model.register_buffer("running", torch.tensor(3.0))
        ema = _ModelEMA(model, decay=0.5, warmups=0)
        with torch.no_grad():
            model.trainable.fill_(3.0)
            model.frozen.fill_(2.0000002)
            model.running.fill_(3.0000002)

        ema.update(model)

        self.assertEqual(ema.module.trainable.item(), 2.0)
        self.assertTrue(torch.equal(ema.module.frozen, model.frozen))
        self.assertTrue(torch.equal(ema.module.running, model.running))

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

    def test_cross_negative_profile_is_classifier_only_with_pose_batches(
        self,
    ) -> None:
        config = stage_training_config(
            3,
            output_dir=Path("cross-negative"),
            epochs=1,
            trainable_profile="cross_negative_classifier_only",
        )

        self.assertEqual(config.trainable_roles, ("classifiers",))
        self.assertEqual(
            _profile_training_tasks("cross_negative_classifier_only"),
            {
                HeadId.OBJECT,
                HeadId.PERSON_POSE,
                HeadId.ROBOT_POSE,
            },
        )

    def test_pose_aligned_profiles_have_exact_roles_and_tasks(self) -> None:
        expected_tasks = {
            HeadId.OBJECT,
            HeadId.PERSON_POSE,
            HeadId.ROBOT_POSE,
        }
        cases = {
            "pose_aligned_decoder_no_classifier": (
                "proposal",
                "decoder",
            ),
            "pose_aligned_decoder": (
                "classifiers",
                "proposal",
                "decoder",
            ),
        }
        for profile, roles in cases.items():
            with self.subTest(profile=profile):
                config = stage_training_config(
                    3,
                    output_dir=Path("pose-aligned"),
                    epochs=1,
                    trainable_profile=profile,
                )
                self.assertEqual(config.trainable_roles, roles)
                self.assertEqual(
                    _profile_training_tasks(profile),
                    expected_tasks,
                )
        result = CliRunner().invoke(train_multitask_main, ["--help"])
        self.assertEqual(result.exit_code, 0, result.output)
        for profile in cases:
            self.assertIn(profile, result.output)

    def test_pose_aligned_schedule_is_exactly_two_one_one(self) -> None:
        model = _LightweightModel()
        loaders = {
            task: _loader(task)
            for task in (
                HeadId.OBJECT,
                HeadId.PERSON_POSE,
                HeadId.ROBOT_POSE,
            )
        }
        trainer = MultiTaskTrainer(
            cast(DFINEMultiTaskModel, model),
            loaders,
            loaders,
            MultiTaskTrainingConfig(
                output_dir=Path("unused"),
                steps_per_epoch=500,
                sampling_weights={
                    HeadId.OBJECT: 2.0,
                    HeadId.PERSON_POSE: 1.0,
                    HeadId.ROBOT_POSE: 1.0,
                },
            ),
        )

        self.assertEqual(
            Counter(trainer._task_schedule(epoch=0, steps=500)),
            {
                HeadId.OBJECT: 250,
                HeadId.PERSON_POSE: 125,
                HeadId.ROBOT_POSE: 125,
            },
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
            for filename in (
                "best_object_field.pt",
                "best_object.pt",
                "best_field.pt",
                "best_strict_field.pt",
                "metrics.jsonl",
                "run_config.json",
            ):
                self.assertTrue((output_dir / filename).is_file())
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

    def test_cosine_schedule_reaches_configured_minimum(self) -> None:
        trainer = MultiTaskTrainer(
            cast(DFINEMultiTaskModel, _LightweightModel()),
            {HeadId.OBJECT: _loader(HeadId.OBJECT)},
            {HeadId.OBJECT: _loader(HeadId.OBJECT)},
            MultiTaskTrainingConfig(
                output_dir=Path("unused"),
                epochs=1,
                steps_per_epoch=3,
                warmup_steps=0,
                learning_rate_schedule="cosine",
                minimum_learning_rate_ratio=0.1,
            ),
        )

        self.assertAlmostEqual(trainer._learning_rate_scale(), 1.0)
        trainer.scheduler_step = 2
        self.assertAlmostEqual(trainer._learning_rate_scale(), 0.1)

    def test_branch_resume_resets_run_scheduler_and_selection(self) -> None:
        with (
            tempfile.TemporaryDirectory() as source_directory,
            tempfile.TemporaryDirectory() as branch_directory,
        ):
            source = MultiTaskTrainer(
                cast(DFINEMultiTaskModel, _LightweightModel()),
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                MultiTaskTrainingConfig(
                    output_dir=Path(source_directory),
                    epochs=1,
                    steps_per_epoch=1,
                ),
            )
            source.criterion = _LightweightCriterion()
            source.fit()

            branch = MultiTaskTrainer(
                cast(DFINEMultiTaskModel, _LightweightModel()),
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                MultiTaskTrainingConfig(
                    output_dir=Path(branch_directory),
                    epochs=1,
                    steps_per_epoch=1,
                    learning_rate=0.02,
                ),
            )
            branch.criterion = _LightweightCriterion()
            branch.resume(Path(source_directory) / "last.pt", mode="branch")

            self.assertEqual(branch.start_epoch, 0)
            self.assertEqual(branch.scheduler_step, 0)
            self.assertEqual(branch.best_fitness, float("-inf"))
            self.assertIsNone(branch.wandb_run_id)
            self.assertTrue(
                all(
                    group["base_lr"] == 0.02
                    for group in branch.optimizer.param_groups
                )
            )

    def test_exact_resume_matches_uninterrupted_training(self) -> None:
        with (
            tempfile.TemporaryDirectory() as full_directory,
            tempfile.TemporaryDirectory() as split_directory,
        ):
            full_model = _LightweightModel()
            full = MultiTaskTrainer(
                cast(DFINEMultiTaskModel, full_model),
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                MultiTaskTrainingConfig(
                    output_dir=Path(full_directory),
                    epochs=2,
                    steps_per_epoch=1,
                    learning_rate=0.1,
                    seed=17,
                ),
            )
            full.criterion = _LightweightCriterion()
            full.fit()

            first_model = _LightweightModel()
            first = MultiTaskTrainer(
                cast(DFINEMultiTaskModel, first_model),
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                MultiTaskTrainingConfig(
                    output_dir=Path(split_directory),
                    epochs=1,
                    steps_per_epoch=1,
                    learning_rate=0.1,
                    seed=17,
                ),
            )
            first.criterion = _LightweightCriterion()
            first.fit()
            resumed_model = _LightweightModel()
            resumed = MultiTaskTrainer(
                cast(DFINEMultiTaskModel, resumed_model),
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                MultiTaskTrainingConfig(
                    output_dir=Path(split_directory),
                    epochs=2,
                    steps_per_epoch=1,
                    learning_rate=0.1,
                    seed=17,
                ),
            )
            resumed.criterion = _LightweightCriterion()
            resumed.resume(Path(split_directory) / "last.pt")
            resumed.fit()

            self.assertEqual(resumed.global_step, full.global_step)
            self.assertTrue(
                torch.equal(resumed_model.weight, full_model.weight)
            )

    def test_exact_resume_validates_loader_execution_settings(self) -> None:
        def loader(*, workers: int, persistent: bool = False) -> DataLoader:
            return DataLoader(
                _TaskDataset(HeadId.OBJECT),
                batch_size=1,
                num_workers=workers,
                persistent_workers=persistent,
                collate_fn=multitask_collate,
            )

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = MultiTaskTrainer(
                cast(DFINEMultiTaskModel, _LightweightModel()),
                {HeadId.OBJECT: loader(workers=0)},
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                MultiTaskTrainingConfig(output_dir=root / "source"),
            )
            checkpoint = source.save_checkpoint(root / "source.pt", 0)
            saved = torch.load(checkpoint, weights_only=True)
            execution = saved["training_config"]["train_loader_execution"]
            self.assertEqual(execution["object"]["num_workers"], 0)
            self.assertFalse(execution["object"]["persistent_workers"])

            changed = MultiTaskTrainer(
                cast(DFINEMultiTaskModel, _LightweightModel()),
                {HeadId.OBJECT: loader(workers=1)},
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                MultiTaskTrainingConfig(output_dir=root / "changed"),
            )
            with self.assertRaisesRegex(
                ValueError,
                "train_loader_execution",
            ):
                changed.resume(checkpoint)

            matching = MultiTaskTrainer(
                cast(DFINEMultiTaskModel, _LightweightModel()),
                {HeadId.OBJECT: loader(workers=0)},
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                MultiTaskTrainingConfig(output_dir=root / "matching"),
            )
            matching.resume(checkpoint)

    def test_exact_resume_rejects_persistent_worker_state(self) -> None:
        def loader() -> DataLoader:
            return DataLoader(
                _TaskDataset(HeadId.OBJECT),
                batch_size=1,
                num_workers=1,
                persistent_workers=True,
                collate_fn=multitask_collate,
            )

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = MultiTaskTrainer(
                cast(DFINEMultiTaskModel, _LightweightModel()),
                {HeadId.OBJECT: loader()},
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                MultiTaskTrainingConfig(output_dir=root / "source"),
            )
            checkpoint = source.save_checkpoint(root / "source.pt", 0)
            resumed = MultiTaskTrainer(
                cast(DFINEMultiTaskModel, _LightweightModel()),
                {HeadId.OBJECT: loader()},
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                MultiTaskTrainingConfig(output_dir=root / "resumed"),
            )

            with self.assertRaisesRegex(ValueError, "persistent-worker RNG"):
                resumed.resume(checkpoint)
            resumed.resume(checkpoint, mode="branch")

    def test_exact_resume_validates_dataset_fingerprints(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = MultiTaskTrainer(
                cast(DFINEMultiTaskModel, _LightweightModel()),
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                MultiTaskTrainingConfig(
                    output_dir=root / "source",
                    dataset_fingerprints={"train/object": {"digest": "before"}},
                ),
            )
            checkpoint = source.save_checkpoint(root / "source.pt", 0)
            changed = MultiTaskTrainer(
                cast(DFINEMultiTaskModel, _LightweightModel()),
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                MultiTaskTrainingConfig(
                    output_dir=root / "changed",
                    dataset_fingerprints={"train/object": {"digest": "after"}},
                ),
            )

            with self.assertRaisesRegex(ValueError, "dataset_fingerprints"):
                changed.resume(checkpoint)
            changed.resume(checkpoint, mode="branch")

    def test_rank_zero_action_failure_is_broadcast_to_peers(self) -> None:
        trainer = MultiTaskTrainer(
            cast(DFINEMultiTaskModel, _LightweightModel()),
            {HeadId.OBJECT: _loader(HeadId.OBJECT)},
            {HeadId.OBJECT: _loader(HeadId.OBJECT)},
            MultiTaskTrainingConfig(output_dir=Path("unused")),
        )
        trainer.distributed = True
        trainer.world_size = 2
        trainer.rank = 1

        action = Mock()

        def broadcast(
            status: list[dict[str, str] | None],
            *,
            src: int,
        ) -> None:
            self.assertEqual(src, 0)
            status[0] = {
                "status": "error",
                "error_type": "FileExistsError",
                "error": "occupied",
            }

        with (
            patch(
                "ultralytics_dfine.engine.multitask_trainer.dist."
                "broadcast_object_list",
                side_effect=broadcast,
            ),
            self.assertRaisesRegex(
                RuntimeError,
                "FileExistsError: occupied",
            ),
        ):
            trainer._synchronize_rank_zero_action(
                action,
                description="output-directory preparation",
            )
        action.assert_not_called()

    def test_freezes_only_batch_norm_without_trainable_parameters(self) -> None:
        model = _LightweightModel()
        model.frozen_bn = nn.BatchNorm1d(2)
        model.field_feature_head = nn.Sequential(nn.BatchNorm1d(2))
        trainer = MultiTaskTrainer(
            cast(DFINEMultiTaskModel, model),
            {HeadId.FIELD_FEATURES: _loader(HeadId.FIELD_FEATURES)},
            {HeadId.FIELD_FEATURES: _loader(HeadId.FIELD_FEATURES)},
            MultiTaskTrainingConfig(
                output_dir=Path("unused"),
                trainable_roles=("field_head",),
                freeze_frozen_bn_stats=True,
            ),
        )
        trainer.training_model.train()
        trainer._enforce_batch_norm_stats_policy()

        self.assertFalse(model.frozen_bn.training)
        self.assertTrue(model.field_feature_head[0].training)

    def test_freeze_all_bn_stats_keeps_affine_parameters_trainable(
        self,
    ) -> None:
        class BatchNormModel(_LightweightModel):
            def __init__(self) -> None:
                super().__init__()
                self.field_feature_head = nn.Sequential(
                    nn.BatchNorm1d(3),
                    nn.SyncBatchNorm(3),
                )

            def forward(
                self,
                images: torch.Tensor,
                targets: list[dict[str, object]],
                active_head: HeadId | None = None,
            ) -> dict[str, dict[str, object]]:
                del targets, active_head
                pooled = images.mean(dim=(-2, -1))
                value = self.field_feature_head(pooled).sum()
                return {"weight": {"value": value}}

        model = BatchNormModel()
        trainer = MultiTaskTrainer(
            cast(DFINEMultiTaskModel, model),
            {HeadId.FIELD_FEATURES: _loader(HeadId.FIELD_FEATURES)},
            {HeadId.FIELD_FEATURES: _loader(HeadId.FIELD_FEATURES)},
            MultiTaskTrainingConfig(
                output_dir=Path("unused"),
                steps_per_epoch=1,
                trainable_roles=("field_head",),
                freeze_all_bn_stats=True,
            ),
        )
        trainer.criterion = _LightweightCriterion()
        batch_norms = tuple(
            module
            for module in model.modules()
            if isinstance(module, nn.modules.batchnorm._BatchNorm)
        )
        running_state = [
            (
                module.running_mean.clone(),
                module.running_var.clone(),
                module.num_batches_tracked.clone(),
            )
            for module in batch_norms
        ]

        trainer.train_epoch(0)
        trainer.train_epoch(1)

        self.assertEqual(len(batch_norms), 2)
        for module, initial_state in zip(
            batch_norms,
            running_state,
            strict=True,
        ):
            self.assertFalse(module.training)
            self.assertTrue(torch.equal(module.running_mean, initial_state[0]))
            self.assertTrue(torch.equal(module.running_var, initial_state[1]))
            self.assertTrue(
                torch.equal(module.num_batches_tracked, initial_state[2])
            )
            self.assertTrue(module.weight.requires_grad)
            self.assertTrue(module.bias.requires_grad)
            self.assertIsNotNone(module.weight.grad)
            self.assertIsNotNone(module.bias.grad)
            self.assertTrue(torch.count_nonzero(module.weight.grad).item())
            self.assertTrue(torch.count_nonzero(module.bias.grad).item())

    def test_freeze_all_bn_stats_is_serialized_and_exposed_by_cli(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output_dir = Path(directory)
            trainer = MultiTaskTrainer(
                cast(DFINEMultiTaskModel, _LightweightModel()),
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                MultiTaskTrainingConfig(
                    output_dir=output_dir,
                    freeze_all_bn_stats=True,
                ),
            )
            trainer._prepare_output_directory()
            checkpoint_path = trainer.save_checkpoint(
                output_dir / "last.pt",
                0,
            )

            run_config = json.loads(
                (output_dir / "run_config.json").read_text()
            )
            checkpoint = torch.load(checkpoint_path, weights_only=True)

        self.assertTrue(run_config["freeze_all_bn_stats"])
        self.assertTrue(checkpoint["training_config"]["freeze_all_bn_stats"])
        result = CliRunner().invoke(train_multitask_main, ["--help"])
        self.assertEqual(result.exit_code, 0, result.output)
        self.assertIn("--freeze-all-bn-stats", result.output)
        self.assertIn("--update-trainable-bn-stats", result.output)
        option = next(
            parameter
            for parameter in train_multitask_main.params
            if parameter.name == "freeze_all_bn_stats"
        )
        self.assertFalse(option.default)

    def test_field_only_can_require_every_ddp_parameter_to_participate(
        self,
    ) -> None:
        model = _FieldOnlyModel()
        trainer = MultiTaskTrainer(
            cast(DFINEMultiTaskModel, model),
            {HeadId.FIELD_FEATURES: _loader(HeadId.FIELD_FEATURES)},
            {HeadId.FIELD_FEATURES: _loader(HeadId.FIELD_FEATURES)},
            MultiTaskTrainingConfig(
                output_dir=Path("unused"),
                steps_per_epoch=1,
                trainable_roles=("field_head",),
                ddp_find_unused_parameters=False,
            ),
        )
        trainer.criterion = _LightweightCriterion()

        trainer.train_epoch()

        trainable = {
            name: parameter
            for name, parameter in model.named_parameters()
            if parameter.requires_grad
        }
        self.assertTrue(trainable)
        self.assertTrue(
            all(name.startswith("field_feature_head.") for name in trainable)
        )
        self.assertTrue(
            all(parameter.grad is not None for parameter in trainable.values())
        )

    def test_disabling_ddp_unused_search_rejects_other_graphs(self) -> None:
        cases = (
            (
                _FieldOnlyModel(),
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                ("field_head",),
            ),
            (
                _FieldOnlyModel(),
                {HeadId.FIELD_FEATURES: _loader(HeadId.FIELD_FEATURES)},
                ("all",),
            ),
        )
        for model, loaders, roles in cases:
            with (
                self.subTest(tasks=tuple(loaders), roles=roles),
                self.assertRaisesRegex(
                    ValueError,
                    "sole training task is field_features",
                ),
            ):
                MultiTaskTrainer(
                    cast(DFINEMultiTaskModel, model),
                    loaders,
                    loaders,
                    MultiTaskTrainingConfig(
                        output_dir=Path("unused"),
                        trainable_roles=roles,
                        ddp_find_unused_parameters=False,
                    ),
                )

    def test_ddp_and_determinism_controls_are_serialized_and_resumed(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = MultiTaskTrainingConfig(
                output_dir=root / "source",
                trainable_roles=("field_head",),
                ddp_find_unused_parameters=False,
                sdpa_backend="math",
            )
            source = MultiTaskTrainer(
                cast(DFINEMultiTaskModel, _FieldOnlyModel()),
                {HeadId.FIELD_FEATURES: _loader(HeadId.FIELD_FEATURES)},
                {HeadId.FIELD_FEATURES: _loader(HeadId.FIELD_FEATURES)},
                config,
            )
            source._prepare_output_directory()
            checkpoint = source.save_checkpoint(root / "last.pt", 0)
            saved = torch.load(checkpoint, weights_only=True)
            run_config = json.loads(
                (root / "source" / "run_config.json").read_text()
            )

            incompatible = MultiTaskTrainer(
                cast(DFINEMultiTaskModel, _FieldOnlyModel()),
                {HeadId.FIELD_FEATURES: _loader(HeadId.FIELD_FEATURES)},
                {HeadId.FIELD_FEATURES: _loader(HeadId.FIELD_FEATURES)},
                MultiTaskTrainingConfig(
                    output_dir=root / "incompatible",
                    trainable_roles=("field_head",),
                ),
            )
            with self.assertRaisesRegex(
                ValueError,
                "ddp_find_unused_parameters: checkpoint=False, current=True",
            ):
                incompatible.resume(checkpoint)

        training_config = saved["training_config"]
        self.assertFalse(training_config["ddp_find_unused_parameters"])
        self.assertEqual(training_config["sdpa_backend"], "math")
        self.assertFalse(run_config["ddp_find_unused_parameters"])
        self.assertEqual(run_config["sdpa_backend"], "math")

    def test_strict_math_sdpa_runtime_and_cli_are_opt_in(self) -> None:
        help_result = CliRunner().invoke(train_multitask_main, ["--help"])
        self.assertEqual(help_result.exit_code, 0, help_result.output)
        for flag in (
            "--no-ddp-find-unused-parameters",
            "--strict-deterministic",
            "--sdpa-backend",
        ):
            self.assertIn(flag, help_result.output)
        defaults = {
            parameter.name: parameter.default
            for parameter in train_multitask_main.params
        }
        self.assertTrue(defaults["ddp_find_unused_parameters"])
        self.assertFalse(defaults["strict_deterministic"])
        self.assertEqual(defaults["sdpa_backend"], "auto")

        with (
            patch("torch.backends.cuda.enable_flash_sdp") as flash,
            patch("torch.backends.cuda.enable_mem_efficient_sdp") as memory,
            patch("torch.backends.cuda.enable_cudnn_sdp") as cudnn,
            patch("torch.backends.cuda.enable_math_sdp") as math_backend,
            patch("torch.use_deterministic_algorithms") as deterministic,
        ):
            MultiTaskTrainer(
                cast(DFINEMultiTaskModel, _FieldOnlyModel()),
                {HeadId.FIELD_FEATURES: _loader(HeadId.FIELD_FEATURES)},
                {HeadId.FIELD_FEATURES: _loader(HeadId.FIELD_FEATURES)},
                MultiTaskTrainingConfig(
                    output_dir=Path("unused"),
                    trainable_roles=("field_head",),
                    strict_deterministic=True,
                    sdpa_backend="math",
                ),
            )

        for backend in (flash, memory, cudnn):
            backend.assert_called_once()
            self.assertFalse(backend.call_args.args[0])
        math_backend.assert_called_once_with(enabled=True)
        deterministic.assert_called_once_with(mode=True, warn_only=False)

    def test_strict_determinism_rejects_inconsistent_configuration(
        self,
    ) -> None:
        with self.assertRaisesRegex(
            ValueError,
            "require deterministic mode",
        ):
            MultiTaskTrainer(
                cast(DFINEMultiTaskModel, _LightweightModel()),
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                MultiTaskTrainingConfig(
                    output_dir=Path("unused"),
                    deterministic=False,
                    strict_deterministic=True,
                ),
            )

    def test_cross_pose_visibility_negative_cli_defaults_off(self) -> None:
        result = CliRunner().invoke(train_multitask_main, ["--help"])

        self.assertEqual(result.exit_code, 0, result.output)
        self.assertIn(
            "--cross-pose-visibility-negative-weight",
            result.output,
        )
        self.assertIn(
            "--cross-pose-detector-negative-weight",
            result.output,
        )
        directional = (
            "person_batch_robot_visibility_negative_weight",
            "robot_batch_person_visibility_negative_weight",
            "person_batch_robot_detector_negative_weight",
            "robot_batch_person_detector_negative_weight",
        )
        for option_name in directional:
            self.assertIn("--" + option_name.replace("_", "-"), result.output)
            option = next(
                parameter
                for parameter in train_multitask_main.params
                if parameter.name == option_name
            )
            self.assertIsNone(option.default)
        for option_name in (
            "cross_pose_visibility_negative_weight",
            "cross_pose_detector_negative_weight",
        ):
            option = next(
                parameter
                for parameter in train_multitask_main.params
                if parameter.name == option_name
            )
            self.assertEqual(option.default, 0.0)

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            object_data = root / "object.yaml"
            person_data = root / "person.yaml"
            object_data.touch()
            person_data.touch()
            dhrp_root = root / "dhrp"
            dhrp_root.mkdir()
            context = train_multitask_main.make_context(
                "train-multitask",
                [
                    "--object-data",
                    str(object_data),
                    "--person-data",
                    str(person_data),
                    "--dhrp-root",
                    str(dhrp_root),
                    "--output-dir",
                    str(root / "output"),
                    "--stage",
                    "3",
                    "--epochs",
                    "1",
                    "--cross-pose-visibility-negative-weight",
                    "0.4",
                    "--cross-pose-detector-negative-weight",
                    "0.3",
                ],
            )

        self.assertEqual(
            context.params["cross_pose_visibility_negative_weight"],
            0.4,
        )
        self.assertEqual(
            context.params["cross_pose_detector_negative_weight"],
            0.3,
        )

    def test_directional_robot_person_negative_cli_requires_manifest(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            object_data = root / "object.yaml"
            person_data = root / "person.yaml"
            object_data.touch()
            person_data.touch()
            dhrp_root = root / "dhrp"
            dhrp_root.mkdir()
            result = CliRunner().invoke(
                train_multitask_main,
                [
                    "--object-data",
                    str(object_data),
                    "--person-data",
                    str(person_data),
                    "--dhrp-root",
                    str(dhrp_root),
                    "--output-dir",
                    str(root / "output"),
                    "--stage",
                    "3",
                    "--epochs",
                    "1",
                    "--robot-batch-person-detector-negative-weight",
                    "0.25",
                ],
            )

        self.assertNotEqual(result.exit_code, 0)
        self.assertIn("--dhrp-person-negative-manifest", result.output)

    def test_directional_person_robot_negative_cli_requires_manifest(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            object_data = root / "object.yaml"
            person_data = root / "person.yaml"
            object_data.touch()
            person_data.touch()
            dhrp_root = root / "dhrp"
            dhrp_root.mkdir()
            result = CliRunner().invoke(
                train_multitask_main,
                [
                    "--object-data",
                    str(object_data),
                    "--person-data",
                    str(person_data),
                    "--dhrp-root",
                    str(dhrp_root),
                    "--output-dir",
                    str(root / "output"),
                    "--stage",
                    "3",
                    "--epochs",
                    "1",
                    "--person-batch-robot-detector-negative-weight",
                    "0.25",
                ],
            )

        self.assertNotEqual(result.exit_code, 0)
        self.assertIn("--coco-robot-negative-manifest", result.output)

    def test_area_normalized_field_loss_cli_is_opt_in(self) -> None:
        options = {
            "field_area_normalized_weight": "0.1",
            "field_area_normalized_beta": "0.2",
            "field_area_scale_floor": "0.02",
            "field_area_scale_cap": "0.2",
        }
        result = CliRunner().invoke(train_multitask_main, ["--help"])

        self.assertEqual(result.exit_code, 0, result.output)
        for option_name in options:
            cli_name = "--" + option_name.replace("_", "-")
            self.assertIn(cli_name, result.output)
            option = next(
                parameter
                for parameter in train_multitask_main.params
                if parameter.name == option_name
            )
            self.assertIsNone(option.default)

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            object_data = root / "object.yaml"
            person_data = root / "person.yaml"
            object_data.touch()
            person_data.touch()
            dhrp_root = root / "dhrp"
            dhrp_root.mkdir()
            arguments = [
                "--object-data",
                str(object_data),
                "--person-data",
                str(person_data),
                "--dhrp-root",
                str(dhrp_root),
                "--output-dir",
                str(root / "output"),
                "--stage",
                "3",
                "--epochs",
                "1",
            ]
            for option_name, value in options.items():
                arguments.extend(("--" + option_name.replace("_", "-"), value))
            context = train_multitask_main.make_context(
                "train-multitask",
                arguments,
            )

        for option_name, value in options.items():
            self.assertEqual(context.params[option_name], float(value))

    def test_exact_resume_cannot_silently_disable_freeze_all_bn_stats(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = MultiTaskTrainer(
                cast(DFINEMultiTaskModel, _LightweightModel()),
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                MultiTaskTrainingConfig(
                    output_dir=root / "source",
                    freeze_frozen_bn_stats=True,
                    freeze_all_bn_stats=True,
                ),
            )
            checkpoint = source.save_checkpoint(root / "last.pt", 0)
            omitted_option = MultiTaskTrainer(
                cast(DFINEMultiTaskModel, _LightweightModel()),
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                MultiTaskTrainingConfig(output_dir=root / "incompatible"),
            )

            with self.assertRaises(ValueError) as raised:
                omitted_option.resume(checkpoint)
            message = str(raised.exception)
            self.assertIn(
                "freeze_frozen_bn_stats: checkpoint=True, current=False",
                message,
            )
            self.assertIn(
                "freeze_all_bn_stats: checkpoint=True, current=False",
                message,
            )

            compatible = MultiTaskTrainer(
                cast(DFINEMultiTaskModel, _LightweightModel()),
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                {HeadId.OBJECT: _loader(HeadId.OBJECT)},
                MultiTaskTrainingConfig(
                    output_dir=root / "compatible",
                    epochs=2,
                    freeze_frozen_bn_stats=True,
                    freeze_all_bn_stats=True,
                ),
            )
            compatible.resume(checkpoint)

        self.assertTrue(compatible.config.freeze_all_bn_stats)
        self.assertEqual(compatible.start_epoch, 1)

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
    def test_rank_zero_startup_failure_reaches_every_process(self) -> None:
        dataset = _TaskDataset(HeadId.OBJECT, length=2)
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
        output_dir = (
            Path(os.environ["DFINE_DDP_OUTPUT_DIR"]) / "startup-failure"
        )
        trainer = MultiTaskTrainer(
            cast(DFINEMultiTaskModel, _LightweightModel()),
            {HeadId.OBJECT: loader},
            {HeadId.OBJECT: _loader(HeadId.OBJECT)},
            MultiTaskTrainingConfig(
                output_dir=output_dir,
                epochs=1,
                steps_per_epoch=1,
                device=os.environ.get("DFINE_DDP_DEVICE", "cpu"),
            ),
        )
        if trainer.rank == 0:
            output_dir.mkdir(parents=True)
            (output_dir / "occupied").touch()
        torch.distributed.barrier()

        with self.assertRaisesRegex(
            (FileExistsError, RuntimeError),
            "Refusing non-empty",
        ):
            trainer.fit()

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
        output_dir = Path(os.environ["DFINE_DDP_OUTPUT_DIR"]) / "success"
        trainer = MultiTaskTrainer(
            cast(DFINEMultiTaskModel, model),
            {HeadId.OBJECT: loader},
            {task: _loader(task) for task in HeadId},
            MultiTaskTrainingConfig(
                output_dir=output_dir,
                epochs=1,
                steps_per_epoch=2,
                learning_rate=0.1,
                device=os.environ.get("DFINE_DDP_DEVICE", "cpu"),
            ),
        )
        trainer.criterion = _LightweightCriterion()

        with patch.object(
            trainer,
            "validate",
            wraps=trainer.validate,
        ) as validate:
            result = trainer.fit()

        validate.assert_called_once_with(0)
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
