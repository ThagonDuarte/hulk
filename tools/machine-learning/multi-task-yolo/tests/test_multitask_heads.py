import unittest
from dataclasses import asdict
from types import SimpleNamespace
from unittest.mock import patch

import torch
import torch.nn as nn

from ultralytics_dfine.config import (
    DFINEArchitectureConfig,
    FieldHeadConfig,
    FieldLossConfig,
    MultiTaskHeadConfig,
    MultiTaskLossConfig,
    PoseHeadConfig,
    PoseLossConfig,
    build_multitask_manifest,
)
from ultralytics_dfine.engine.multitask_render import (
    _box_suppression_indices,
    _suppress_field_points,
)
from ultralytics_dfine.nn.multitask import (
    DFINEMultiTaskModel,
    FieldFeatureHead,
    QueryPoseHead,
)
from ultralytics_dfine.nn.postprocess import DFINEPostProcessorAdapter
from ultralytics_dfine.schemas import PERSON_POSE_SCHEMA, HeadId


class MultiTaskConfigurationTests(unittest.TestCase):
    def test_non_finite_configuration_values_are_rejected(self) -> None:
        cases = (
            (PoseHeadConfig, "refinement_scale"),
            (FieldHeadConfig, "refinement_scale"),
            (PoseLossConfig, "smooth_l1_beta"),
            (FieldLossConfig, "quality_sigma"),
            (MultiTaskLossConfig, "cross_pose_detector_negative_weight"),
            (
                MultiTaskLossConfig,
                "robot_batch_person_detector_negative_weight",
            ),
            (DFINEArchitectureConfig, "reg_scale"),
        )
        for factory, field_name in cases:
            for value in (float("nan"), float("inf"), float("-inf")):
                with (
                    self.subTest(
                        factory=factory.__name__,
                        field_name=field_name,
                        value=value,
                    ),
                    self.assertRaisesRegex(ValueError, "finite"),
                ):
                    factory(**{field_name: value})

    def test_shared_pose_refiner_requires_matching_configuration(self) -> None:
        mismatches = (
            {"refinement_dim": 32},
            {"refinement_scale": 0.5},
            {"feature_levels": 2},
            {"detach_sampling_grid": False},
        )
        for updates in mismatches:
            field_name = next(iter(updates))
            with (
                self.subTest(field_name=field_name),
                self.assertRaisesRegex(ValueError, field_name),
            ):
                MultiTaskHeadConfig(
                    person_pose=PoseHeadConfig(variant="shared_spatial_refine"),
                    robot_pose=PoseHeadConfig(
                        variant="shared_spatial_refine",
                        **updates,
                    ),
                )

        compatible = MultiTaskHeadConfig(
            person_pose=PoseHeadConfig(
                variant="shared_spatial_refine",
                visibility_score_alpha=0.0,
            ),
            robot_pose=PoseHeadConfig(
                variant="shared_spatial_refine",
                visibility_score_alpha=1.0,
            ),
        )
        self.assertNotEqual(
            compatible.person_pose.visibility_score_alpha,
            compatible.robot_pose.visibility_score_alpha,
        )

    def test_nested_head_and_loss_configs_round_trip(self) -> None:
        heads = MultiTaskHeadConfig(
            person_pose=PoseHeadConfig(variant="shared_spatial_refine"),
            robot_pose=PoseHeadConfig(variant="shared_spatial_refine"),
            field_features=FieldHeadConfig(variant="spatial_refine"),
        )
        losses = MultiTaskLossConfig(
            person_pose=PoseLossConfig(
                coordinate_space="box",
                smooth_l1_beta=0.05,
            ),
            field_features=FieldLossConfig(
                classification_mode="strict_quality",
                point_weight=10,
                area_normalized_weight=0.1,
                area_normalized_beta=0.2,
                area_scale_floor=0.02,
                area_scale_cap=0.2,
            ),
            cross_pose_visibility_negative_weight=0.75,
            cross_pose_detector_negative_weight=0.5,
            person_batch_robot_visibility_negative_weight=0.2,
            robot_batch_person_visibility_negative_weight=0.3,
            person_batch_robot_detector_negative_weight=0.4,
            robot_batch_person_detector_negative_weight=0.6,
        )

        self.assertEqual(
            MultiTaskHeadConfig.from_dict(asdict(heads)),
            heads,
        )
        self.assertEqual(
            MultiTaskLossConfig.from_dict(asdict(losses)),
            losses,
        )
        manifest = build_multitask_manifest(
            DFINEArchitectureConfig(),
            ["Ball", "Robot", "Person"],
            heads,
            losses,
        )
        self.assertEqual(
            manifest.loss_config["cross_pose_visibility_negative_weight"],
            0.75,
        )
        self.assertEqual(
            manifest.loss_config["cross_pose_detector_negative_weight"],
            0.5,
        )
        self.assertEqual(
            manifest.loss_config["robot_batch_person_detector_negative_weight"],
            0.6,
        )
        self.assertEqual(
            manifest.loss_config["field_features"]["area_normalized_weight"],
            0.1,
        )

    def test_legacy_loss_config_defaults_cross_pose_negatives_off(self) -> None:
        legacy = asdict(MultiTaskLossConfig())
        legacy.pop("cross_pose_visibility_negative_weight")
        legacy.pop("cross_pose_detector_negative_weight")
        for field_name in (
            "person_batch_robot_visibility_negative_weight",
            "robot_batch_person_visibility_negative_weight",
            "person_batch_robot_detector_negative_weight",
            "robot_batch_person_detector_negative_weight",
        ):
            legacy.pop(field_name)
        for field_name in (
            "area_normalized_weight",
            "area_normalized_beta",
            "area_scale_floor",
            "area_scale_cap",
        ):
            legacy["field_features"].pop(field_name)

        restored = MultiTaskLossConfig.from_dict(legacy)

        self.assertEqual(restored.cross_pose_visibility_negative_weight, 0.0)
        self.assertEqual(restored.cross_pose_detector_negative_weight, 0.0)
        self.assertIsNone(restored.person_batch_robot_detector_negative_weight)
        self.assertEqual(restored.person_batch_robot_detector_weight, 0.0)
        self.assertEqual(
            restored.field_features,
            FieldLossConfig(),
        )

    def test_area_normalized_field_loss_settings_are_validated(self) -> None:
        invalid = (
            ({"area_normalized_weight": -0.1}, "weights"),
            ({"area_normalized_beta": 0.0}, "beta"),
            ({"area_scale_floor": 0.0}, "floor"),
            (
                {"area_scale_floor": 0.2, "area_scale_cap": 0.1},
                "cap",
            ),
        )
        for values, message in invalid:
            with (
                self.subTest(values=values),
                self.assertRaisesRegex(ValueError, message),
            ):
                FieldLossConfig(**values)

    def test_cross_pose_negative_weight_must_be_non_negative(self) -> None:
        for field_name in (
            "cross_pose_visibility_negative_weight",
            "cross_pose_detector_negative_weight",
            "person_batch_robot_visibility_negative_weight",
            "robot_batch_person_visibility_negative_weight",
            "person_batch_robot_detector_negative_weight",
            "robot_batch_person_detector_negative_weight",
        ):
            with (
                self.subTest(field_name=field_name),
                self.assertRaisesRegex(
                    ValueError,
                    "must be non-negative",
                ),
            ):
                MultiTaskLossConfig(**{field_name: -0.1})

    def test_legacy_symmetric_weights_feed_both_directions(self) -> None:
        config = MultiTaskLossConfig(
            cross_pose_visibility_negative_weight=0.2,
            cross_pose_detector_negative_weight=0.3,
        )

        self.assertEqual(config.person_batch_robot_visibility_weight, 0.2)
        self.assertEqual(config.robot_batch_person_visibility_weight, 0.2)
        self.assertEqual(config.person_batch_robot_detector_weight, 0.3)
        self.assertEqual(config.robot_batch_person_detector_weight, 0.3)


class QueryPoseHeadTests(unittest.TestCase):
    def test_pose_output_is_query_aligned_and_normalized(self) -> None:
        head = QueryPoseHead(16, PERSON_POSE_SCHEMA)
        features = torch.randn(2, 5, 16)
        boxes = torch.tensor([[[0.5, 0.5, 0.2, 0.4]]]).expand(2, 5, 4)

        output = head(features, boxes)

        self.assertEqual(output["pred_keypoints"].shape, (2, 5, 17, 2))
        self.assertEqual(output["pred_visibility"].shape, (2, 5, 17))
        self.assertTrue((output["pred_keypoints"] >= 0).all())
        self.assertTrue((output["pred_keypoints"] <= 1).all())

    def test_frozen_pose_head_backpropagates_to_queries_and_boxes(
        self,
    ) -> None:
        torch.manual_seed(7)
        head = QueryPoseHead(16, PERSON_POSE_SCHEMA)
        head.requires_grad_(requires_grad=False)
        queries = torch.randn(1, 3, 16, requires_grad=True)
        boxes = torch.tensor(
            [
                [
                    [0.5, 0.5, 0.4, 0.5],
                    [0.3, 0.3, 0.2, 0.2],
                    [0.7, 0.7, 0.2, 0.2],
                ]
            ],
            requires_grad=True,
        )

        output = head(queries, boxes)
        loss = output["pred_keypoints"].sum()
        loss = loss + output["pred_visibility"].square().mean()
        loss.backward()

        self.assertGreater(queries.grad.abs().sum().item(), 0)
        self.assertGreater(boxes.grad.abs().sum().item(), 0)
        self.assertTrue(
            all(parameter.grad is None for parameter in head.parameters())
        )

    def test_spatial_refiner_is_an_exact_zero_initialized_residual(
        self,
    ) -> None:
        baseline = QueryPoseHead(16, PERSON_POSE_SCHEMA)
        refined = QueryPoseHead(
            16,
            PERSON_POSE_SCHEMA,
            PoseHeadConfig(
                variant="spatial_refine",
                refinement_dim=8,
            ),
        )
        refined.network.load_state_dict(baseline.network.state_dict())
        queries = torch.randn(2, 5, 16)
        boxes = torch.tensor([[[0.5, 0.5, 0.2, 0.4]]]).expand(2, 5, 4)
        features = (
            torch.randn(2, 16, 8, 8),
            torch.randn(2, 16, 4, 4),
            torch.randn(2, 16, 2, 2),
        )

        coarse = baseline(queries, boxes)
        output = refined(queries, boxes, features)

        torch.testing.assert_close(
            output["pred_keypoints"],
            coarse["pred_keypoints"],
        )
        torch.testing.assert_close(
            output["pred_visibility"],
            coarse["pred_visibility"],
        )


class FieldFeatureHeadTests(unittest.TestCase):
    def test_all_field_parameters_participate_in_every_variant_graph(
        self,
    ) -> None:
        features = (
            torch.randn(1, 16, 4, 5),
            torch.randn(1, 16, 2, 3),
            torch.randn(1, 16, 1, 2),
        )
        for variant in ("query_decoder", "spatial_refine"):
            with self.subTest(variant=variant):
                head = FieldFeatureHead(
                    16,
                    attention_heads=4,
                    config=FieldHeadConfig(
                        variant=variant,  # type: ignore[arg-type]
                        refinement_dim=8,
                    ),
                )
                output = head(features)

                (
                    output["pred_logits"].square().sum()
                    + output["pred_points"].sum()
                ).backward()

                missing = [
                    name
                    for name, parameter in head.named_parameters()
                    if parameter.grad is None
                ]
                self.assertEqual(missing, [])

    def test_field_output_has_fixed_query_count(self) -> None:
        head = FieldFeatureHead(16, attention_heads=4)
        features = (
            torch.randn(2, 16, 8, 8),
            torch.randn(2, 16, 4, 4),
            torch.randn(2, 16, 2, 2),
        )

        output = head(features)

        self.assertEqual(output["pred_logits"].shape, (2, 300, 5))
        self.assertEqual(output["pred_points"].shape, (2, 300, 2))
        self.assertTrue((output["pred_points"] >= 0).all())
        self.assertTrue((output["pred_points"] <= 1).all())
        self.assertLess(
            float(output["pred_logits"].sigmoid().max().detach()),
            0.02,
        )

    def test_memory_contains_two_dimensional_position_encoding(self) -> None:
        head = FieldFeatureHead(16, attention_heads=4)
        features = (
            torch.zeros(1, 16, 2, 3),
            torch.zeros(1, 16, 1, 2),
            torch.zeros(1, 16, 1, 1),
        )

        memory = head._flatten_memory(features)

        self.assertFalse(torch.equal(memory[:, 0], memory[:, 1]))
        self.assertFalse(torch.equal(memory[:, 0], memory[:, 3]))

    def test_initial_points_cover_the_image(self) -> None:
        head = FieldFeatureHead(16, attention_heads=4)
        features = (
            torch.zeros(1, 16, 2, 2),
            torch.zeros(1, 16, 1, 1),
            torch.zeros(1, 16, 1, 1),
        )

        points = head(features)["pred_points"][0].detach()

        self.assertLess(float(points[:, 0].min()), 0.05)
        self.assertGreater(float(points[:, 0].max()), 0.95)
        self.assertLess(float(points[:, 1].min()), 0.05)
        self.assertGreater(float(points[:, 1].max()), 0.95)

    def test_field_spatial_refiner_is_zero_initialized(self) -> None:
        baseline = FieldFeatureHead(16, attention_heads=4)
        refined = FieldFeatureHead(
            16,
            attention_heads=4,
            config=FieldHeadConfig(
                variant="spatial_refine",
                refinement_dim=8,
            ),
        )
        compatible = {
            name: value
            for name, value in baseline.state_dict().items()
            if name in refined.state_dict()
        }
        refined.load_state_dict(compatible, strict=False)
        features = (
            torch.randn(2, 16, 8, 8),
            torch.randn(2, 16, 4, 4),
            torch.randn(2, 16, 2, 2),
        )

        coarse = baseline(features)
        output = refined(features)

        torch.testing.assert_close(
            output["pred_points"],
            coarse["pred_points"],
        )
        torch.testing.assert_close(
            output["pred_logits"],
            coarse["pred_logits"],
        )

    def test_field_render_suppression_is_class_aware(self) -> None:
        candidates = torch.tensor(
            [
                [10.0, 10.0, 0.8, 0.0],
                [12.0, 10.0, 0.9, 0.0],
                [12.0, 10.0, 0.7, 1.0],
            ]
        )

        filtered = _suppress_field_points(candidates, max_detections=20)

        self.assertEqual(filtered.shape[0], 2)
        self.assertEqual(float(filtered[0, 2]), float(candidates[1, 2]))

    def test_box_render_suppression_removes_duplicate(self) -> None:
        candidates = torch.tensor(
            [
                [0.0, 0.0, 10.0, 10.0, 0.8, 0.0],
                [0.0, 0.0, 10.0, 10.0, 0.9, 0.0],
                [0.0, 0.0, 10.0, 10.0, 0.7, 1.0],
            ]
        )

        indices = _box_suppression_indices(
            candidates,
            class_aware=True,
            max_detections=20,
        )

        self.assertEqual(indices.tolist(), [1, 2])


class _FakeDetector(nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.names = [
            "Ball",
            "GoalPost",
            "LSpot",
            "PenaltySpot",
            "Robot",
            "TSpot",
            "XSpot",
            "Person",
        ]
        self.architecture = SimpleNamespace(hidden_dim=16, num_top_queries=3)
        self.postprocessor = DFINEPostProcessorAdapter(8, 3)

    def forward_raw(
        self,
        images: torch.Tensor,
        targets: list[dict[str, object]] | None = None,
    ) -> dict[str, object]:
        del targets
        batch = images.shape[0]
        return {
            "pred_logits": torch.zeros(batch, 3, 8),
            "pred_boxes": torch.full((batch, 3, 4), 0.5),
            "query_features": torch.ones(batch, 3, 16),
            "encoder_features": (
                torch.ones(batch, 16, 4, 4),
                torch.ones(batch, 16, 2, 2),
                torch.ones(batch, 16, 1, 1),
            ),
        }


class CrossPoseForwardRoutingTests(unittest.TestCase):
    def test_directional_visibility_only_adds_requested_opposite_head(
        self,
    ) -> None:
        model = DFINEMultiTaskModel(  # type: ignore[arg-type]
            _FakeDetector(),
            loss_config=MultiTaskLossConfig(
                person_batch_robot_visibility_negative_weight=0.5,
                robot_batch_person_visibility_negative_weight=0.0,
            ),
        )
        images = torch.zeros(1, 3, 32, 32)

        person = model.forward_raw(
            images,
            targets=[{}],
            active_head=HeadId.PERSON_POSE,
        )
        robot = model.forward_raw(
            images,
            targets=[{}],
            active_head=HeadId.ROBOT_POSE,
        )

        self.assertIn(str(HeadId.ROBOT_POSE), person)
        self.assertNotIn(str(HeadId.PERSON_POSE), robot)

    def test_detector_negatives_do_not_require_the_opposite_pose_head(
        self,
    ) -> None:
        model = DFINEMultiTaskModel(  # type: ignore[arg-type]
            _FakeDetector(),
            loss_config=MultiTaskLossConfig(
                cross_pose_detector_negative_weight=0.5,
            ),
        )

        outputs = model.forward_raw(
            torch.zeros(1, 3, 32, 32),
            targets=[{}],
            active_head=HeadId.PERSON_POSE,
        )

        self.assertEqual(
            set(outputs),
            {str(HeadId.OBJECT), str(HeadId.PERSON_POSE)},
        )

    def test_training_includes_only_the_required_opposite_pose_head(
        self,
    ) -> None:
        images = torch.zeros(1, 3, 32, 32)
        targets: list[dict[str, object]] = [{}]
        default = DFINEMultiTaskModel(  # type: ignore[arg-type]
            _FakeDetector()
        )
        enabled = DFINEMultiTaskModel(  # type: ignore[arg-type]
            _FakeDetector(),
            loss_config=MultiTaskLossConfig(
                cross_pose_visibility_negative_weight=0.5,
            ),
        )
        enabled.load_state_dict(default.state_dict())

        default_person = default.forward_raw(
            images,
            targets,
            HeadId.PERSON_POSE,
        )
        enabled_person = enabled.forward_raw(
            images,
            targets,
            HeadId.PERSON_POSE,
        )
        enabled_robot = enabled.forward_raw(
            images,
            targets,
            HeadId.ROBOT_POSE,
        )

        self.assertEqual(
            set(default_person),
            {str(HeadId.OBJECT), str(HeadId.PERSON_POSE)},
        )
        self.assertEqual(
            set(enabled_person),
            {
                str(HeadId.OBJECT),
                str(HeadId.PERSON_POSE),
                str(HeadId.ROBOT_POSE),
            },
        )
        self.assertEqual(
            set(enabled_robot),
            {
                str(HeadId.OBJECT),
                str(HeadId.PERSON_POSE),
                str(HeadId.ROBOT_POSE),
            },
        )
        self.assertNotIn(str(HeadId.FIELD_FEATURES), enabled_person)
        for head_id, default_output in default_person.items():
            enabled_output = enabled_person[head_id]
            for name, value in default_output.items():
                if isinstance(value, torch.Tensor):
                    self.assertTrue(
                        torch.equal(
                            value,
                            enabled_output[name],  # type: ignore[arg-type]
                        )
                    )

    def test_cross_pose_heads_remain_training_only(self) -> None:
        model = DFINEMultiTaskModel(  # type: ignore[arg-type]
            _FakeDetector(),
            loss_config=MultiTaskLossConfig(
                cross_pose_visibility_negative_weight=0.5,
            ),
        )

        outputs = model.forward_raw(
            torch.zeros(1, 3, 32, 32),
            active_head=HeadId.PERSON_POSE,
        )

        self.assertEqual(
            set(outputs),
            {str(HeadId.OBJECT), str(HeadId.PERSON_POSE)},
        )

    def test_v2_and_legacy_v3_checkpoints_restore_default_off(self) -> None:
        source = DFINEMultiTaskModel(  # type: ignore[arg-type]
            _FakeDetector()
        )
        base = {
            "architecture": "dfine-multitask",
            "names": source.detector.names,
            "architecture_config": asdict(DFINEArchitectureConfig()),
            "hf_config": {},
            "model": source.state_dict(),
        }
        for version in (2, 3):
            with self.subTest(version=version):
                checkpoint = {**base, "format_version": version}
                if version == 3:
                    loss_config = asdict(source.loss_config)
                    loss_config.pop("cross_pose_visibility_negative_weight")
                    loss_config.pop("cross_pose_detector_negative_weight")
                    for field_name in (
                        "area_normalized_weight",
                        "area_normalized_beta",
                        "area_scale_floor",
                        "area_scale_cap",
                    ):
                        loss_config["field_features"].pop(field_name)
                    checkpoint.update(
                        {
                            "head_config": asdict(source.head_config),
                            "loss_config": loss_config,
                        }
                    )
                with (
                    patch(
                        "ultralytics_dfine.nn.multitask.torch.load",
                        return_value=checkpoint,
                    ),
                    patch(
                        "ultralytics_dfine.nn.multitask.DFineConfig.from_dict",
                        return_value=SimpleNamespace(),
                    ),
                    patch(
                        "ultralytics_dfine.nn.multitask."
                        "DFineForObjectDetection",
                        return_value=nn.Identity(),
                    ),
                    patch(
                        "ultralytics_dfine.nn.multitask.DFINEDetectionModel",
                        return_value=_FakeDetector(),
                    ),
                ):
                    restored = DFINEMultiTaskModel.from_checkpoint("legacy.pt")

                self.assertEqual(
                    restored.loss_config.cross_pose_visibility_negative_weight,
                    0.0,
                )
                self.assertEqual(
                    restored.loss_config.cross_pose_detector_negative_weight,
                    0.0,
                )
                self.assertEqual(
                    restored.loss_config.field_features,
                    FieldLossConfig(),
                )


class MultiTaskDeploymentTests(unittest.TestCase):
    def test_fixed_outputs_have_expected_layouts(self) -> None:
        model = DFINEMultiTaskModel(_FakeDetector())  # type: ignore[arg-type]
        outputs = {
            str(HeadId.OBJECT): {
                "pred_logits": torch.randn(1, 3, 8),
                "pred_boxes": torch.rand(1, 3, 4),
            },
            str(HeadId.PERSON_POSE): {
                "pred_keypoints": torch.rand(1, 3, 17, 2),
                "pred_visibility": torch.randn(1, 3, 17),
            },
            str(HeadId.ROBOT_POSE): {
                "pred_keypoints": torch.rand(1, 3, 14, 2),
                "pred_visibility": torch.randn(1, 3, 14),
            },
            str(HeadId.FIELD_FEATURES): {
                "pred_logits": torch.randn(1, 300, 5),
                "pred_points": torch.rand(1, 300, 2),
            },
        }
        with patch.object(model, "forward_raw", return_value=outputs):
            deployed = model.forward_deploy(torch.zeros(1, 3, 64, 96))

        self.assertEqual(deployed["object_output"].shape, (1, 3, 6))
        self.assertEqual(deployed["person_pose_output"].shape, (1, 3, 57))
        self.assertEqual(deployed["robot_pose_output"].shape, (1, 3, 14, 3))
        self.assertEqual(
            deployed["field_feature_output"].shape,
            (1, 300, 4),
        )


if __name__ == "__main__":
    unittest.main()
