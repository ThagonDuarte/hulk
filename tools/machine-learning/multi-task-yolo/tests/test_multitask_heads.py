import unittest
from types import SimpleNamespace
from unittest.mock import patch

import torch
import torch.nn as nn

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


class FieldFeatureHeadTests(unittest.TestCase):
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
