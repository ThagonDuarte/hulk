import unittest

import torch

from ultralytics_dfine.loss import (
    FieldFeatureCriterion,
    QueryPoseCriterion,
)
from ultralytics_dfine.schemas import PERSON_POSE_SCHEMA


class QueryPoseCriterionTests(unittest.TestCase):
    def test_exact_visible_keypoints_have_zero_location_losses(self) -> None:
        keypoints = torch.full((1, 17, 3), 0.5)
        keypoints[..., 2] = 1
        output = {
            "pred_keypoints": keypoints[..., :2].unsqueeze(0).clone(),
            "pred_visibility": torch.full((1, 1, 17), 5.0),
        }
        targets = [
            {
                "labels": torch.tensor([7]),
                "boxes": torch.tensor([[0.5, 0.5, 0.4, 0.6]]),
                "keypoints": keypoints,
            }
        ]
        matches = [(torch.tensor([0]), torch.tensor([0]))]

        result = QueryPoseCriterion(PERSON_POSE_SCHEMA, 7)(
            output,
            targets,
            matches,
        )

        torch.testing.assert_close(result.coordinate, torch.tensor(0.0))
        torch.testing.assert_close(result.oks, torch.tensor(0.0))
        self.assertTrue(torch.isfinite(result.visibility))

    def test_inactive_pose_head_retains_gradient_connection(self) -> None:
        keypoints = torch.rand(1, 2, 17, 2, requires_grad=True)
        visibility = torch.rand(1, 2, 17, requires_grad=True)
        targets = [
            {
                "labels": torch.tensor([4]),
                "boxes": torch.tensor([[0.5, 0.5, 0.4, 0.6]]),
                "keypoints": torch.rand(1, 17, 3),
            }
        ]
        matches = [(torch.tensor([0]), torch.tensor([0]))]

        result = QueryPoseCriterion(PERSON_POSE_SCHEMA, 7)(
            {
                "pred_keypoints": keypoints,
                "pred_visibility": visibility,
            },
            targets,
            matches,
        )
        result.total.backward()

        self.assertIsNotNone(keypoints.grad)
        self.assertIsNotNone(visibility.grad)


class FieldFeatureCriterionTests(unittest.TestCase):
    def test_field_match_and_losses_are_finite(self) -> None:
        output = {
            "pred_logits": torch.tensor(
                [[[8.0, -8.0], [-8.0, 8.0]]],
                requires_grad=True,
            ),
            "pred_points": torch.tensor(
                [[[0.25, 0.25], [0.75, 0.75]]],
                requires_grad=True,
            ),
        }
        targets = [
            {
                "labels": torch.tensor([0, 1]),
                "points": torch.tensor([[0.25, 0.25], [0.75, 0.75]]),
            }
        ]

        losses, matches = FieldFeatureCriterion(2)(output, targets)

        self.assertEqual(len(matches[0][0]), 2)
        self.assertTrue(all(torch.isfinite(loss) for loss in losses.values()))
        torch.stack(tuple(losses.values())).sum().backward()
        self.assertIsNotNone(output["pred_logits"].grad)
        self.assertIsNotNone(output["pred_points"].grad)


if __name__ == "__main__":
    unittest.main()
