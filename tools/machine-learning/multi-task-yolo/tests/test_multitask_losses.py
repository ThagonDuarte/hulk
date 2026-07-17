import unittest
from unittest.mock import patch

import torch

from ultralytics_dfine.config import (
    FieldLossConfig,
    MultiTaskLossConfig,
    PoseLossConfig,
)
from ultralytics_dfine.loss import (
    CriterionResult,
    FieldFeatureCriterion,
    MultiTaskCriterion,
    QueryPoseCriterion,
)
from ultralytics_dfine.schemas import PERSON_POSE_SCHEMA, HeadId


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

    def test_box_relative_coordinates_scale_error_by_target_box(self) -> None:
        keypoints = torch.full((1, 17, 3), 0.5)
        keypoints[..., 2] = 1
        predicted = keypoints[..., :2].unsqueeze(0).clone()
        predicted[..., 0] += 0.01
        targets = [
            {
                "labels": torch.tensor([7]),
                "boxes": torch.tensor([[0.5, 0.5, 0.1, 0.2]]),
                "keypoints": keypoints,
            }
        ]
        matches = [(torch.tensor([0]), torch.tensor([0]))]

        result = QueryPoseCriterion(
            PERSON_POSE_SCHEMA,
            7,
            config=PoseLossConfig(
                coordinate_space="box",
                smooth_l1_beta=0.05,
                oks_weight=0,
                visibility_weight=0,
            ),
        )(
            {
                "pred_keypoints": predicted,
                "pred_visibility": torch.zeros(1, 1, 17),
            },
            targets,
            matches,
        )

        expected_per_joint = 0.1 - 0.5 * 0.05
        torch.testing.assert_close(
            result.coordinate,
            torch.tensor(expected_per_joint),
        )


class CrossPoseVisibilityNegativeTests(unittest.TestCase):
    @staticmethod
    def _pose_output(keypoint_count: int) -> dict[str, torch.Tensor]:
        return {
            "pred_keypoints": torch.full(
                (1, 1, keypoint_count, 2),
                0.5,
                requires_grad=True,
            ),
            "pred_visibility": torch.zeros(
                1,
                1,
                keypoint_count,
                requires_grad=True,
            ),
        }

    def test_cross_pose_visibility_negative_value_and_gradient_routing(
        self,
    ) -> None:
        weight = 0.25
        criterion = MultiTaskCriterion(
            loss_config=MultiTaskLossConfig(
                cross_pose_visibility_negative_weight=weight,
            )
        )
        cases = (
            (
                HeadId.PERSON_POSE,
                7,
                17,
                HeadId.ROBOT_POSE,
                "robot_cross_visibility_negative",
            ),
            (
                HeadId.ROBOT_POSE,
                4,
                14,
                HeadId.PERSON_POSE,
                "person_cross_visibility_negative",
            ),
        )
        for active, class_id, count, opposite, loss_key in cases:
            with self.subTest(active=active):
                object_logits = torch.zeros(1, 1, 8, requires_grad=True)
                field_logits = torch.zeros(1, 1, 5, requires_grad=True)
                person = self._pose_output(17)
                robot = self._pose_output(14)
                outputs = {
                    str(HeadId.OBJECT): {
                        "pred_logits": object_logits,
                        "pred_boxes": torch.full((1, 1, 4), 0.5),
                    },
                    str(HeadId.PERSON_POSE): person,
                    str(HeadId.ROBOT_POSE): robot,
                    str(HeadId.FIELD_FEATURES): {
                        "pred_logits": field_logits,
                        "pred_points": torch.full((1, 1, 2), 0.5),
                    },
                }
                keypoints = torch.full((1, count, 3), 0.5)
                keypoints[..., 2] = 1
                targets = [
                    {
                        "labels": torch.tensor([class_id]),
                        "boxes": torch.tensor([[0.5, 0.5, 0.4, 0.6]]),
                        "keypoints": keypoints,
                        **(
                            {"person_negative_eligible": True}
                            if active == HeadId.ROBOT_POSE
                            else {"robot_negative_eligible": True}
                        ),
                    }
                ]
                matches = [(torch.tensor([0]), torch.tensor([0]))]
                detection = CriterionResult(
                    {"detection_stub": torch.tensor(0.0)},
                    matches,
                )
                with patch.object(
                    criterion.detection,
                    "forward_with_matches",
                    return_value=detection,
                ):
                    result = criterion(outputs, targets, active)

                expected = torch.log(torch.tensor(2.0)) * weight
                torch.testing.assert_close(result.losses[loss_key], expected)
                total = torch.stack(tuple(result.losses.values())).sum()
                total.backward()

                active_output = (
                    person if active == HeadId.PERSON_POSE else robot
                )
                opposite_output = (
                    person if opposite == HeadId.PERSON_POSE else robot
                )
                self.assertIsNotNone(active_output["pred_visibility"].grad)
                self.assertIsNotNone(opposite_output["pred_visibility"].grad)
                self.assertIsNone(opposite_output["pred_keypoints"].grad)
                self.assertIsNone(object_logits.grad)
                self.assertIsNone(field_logits.grad)

    def test_default_off_does_not_require_the_opposite_head(self) -> None:
        criterion = MultiTaskCriterion()
        person = self._pose_output(17)
        outputs = {
            str(HeadId.OBJECT): {
                "pred_logits": torch.zeros(1, 1, 8),
                "pred_boxes": torch.full((1, 1, 4), 0.5),
            },
            str(HeadId.PERSON_POSE): person,
        }
        keypoints = torch.full((1, 17, 3), 0.5)
        keypoints[..., 2] = 1
        targets = [
            {
                "labels": torch.tensor([7]),
                "boxes": torch.tensor([[0.5, 0.5, 0.4, 0.6]]),
                "keypoints": keypoints,
            }
        ]
        matches = [(torch.tensor([0]), torch.tensor([0]))]
        with patch.object(
            criterion.detection,
            "forward_with_matches",
            return_value=CriterionResult(
                {"detection_stub": torch.tensor(0.0)},
                matches,
            ),
        ):
            result = criterion(outputs, targets, HeadId.PERSON_POSE)

        self.assertNotIn("robot_cross_visibility_negative", result.losses)

    def test_robot_batch_person_visibility_is_record_masked(self) -> None:
        weight = 0.25
        criterion = MultiTaskCriterion(
            loss_config=MultiTaskLossConfig(
                robot_batch_person_visibility_negative_weight=weight,
            )
        )
        logits = torch.zeros(2, 1, 17, requires_grad=True)
        output = {
            "pred_keypoints": torch.zeros(2, 1, 17, 2),
            "pred_visibility": logits,
        }
        targets = [
            {"person_negative_eligible": False},
            {"person_negative_eligible": True},
        ]
        eligible = criterion._negative_eligibility(
            targets,
            logits.device,
            key="person_negative_eligible",
            description="Robot-to-Person negative",
        )

        loss = criterion._cross_pose_visibility_negative(
            output,
            eligible,
            weight,
        )
        loss.backward()

        torch.testing.assert_close(loss, torch.log(torch.tensor(2.0)) * weight)
        self.assertTrue((logits.grad[0] == 0).all())
        self.assertTrue((logits.grad[1] > 0).all())

    def test_person_batch_robot_visibility_is_record_masked(self) -> None:
        weight = 0.25
        criterion = MultiTaskCriterion(
            loss_config=MultiTaskLossConfig(
                person_batch_robot_visibility_negative_weight=weight,
            )
        )
        logits = torch.zeros(2, 1, 14, requires_grad=True)
        output = {
            "pred_keypoints": torch.zeros(2, 1, 14, 2),
            "pred_visibility": logits,
        }
        targets = [
            {"robot_negative_eligible": False},
            {"robot_negative_eligible": True},
        ]
        eligible = criterion._negative_eligibility(
            targets,
            logits.device,
            key="robot_negative_eligible",
            description="Person-to-Robot negative",
        )

        loss = criterion._cross_pose_visibility_negative(
            output,
            eligible,
            weight,
        )
        loss.backward()

        torch.testing.assert_close(loss, torch.log(torch.tensor(2.0)) * weight)
        self.assertTrue((logits.grad[0] == 0).all())
        self.assertTrue((logits.grad[1] > 0).all())

    def test_cross_visibility_uses_global_ddp_eligible_mean(self) -> None:
        criterion = MultiTaskCriterion()
        weight = 0.25
        logits = torch.zeros(2, 1, 14, requires_grad=True)
        output = {
            "pred_keypoints": torch.zeros(2, 1, 14, 2),
            "pred_visibility": logits,
        }
        eligible = torch.tensor([True, False])

        def add_remote_count(value: torch.Tensor) -> None:
            value.add_(3)

        with (
            patch(
                "ultralytics_dfine.loss.multitask.distributed.is_available",
                return_value=True,
            ),
            patch(
                "ultralytics_dfine.loss.multitask.distributed.is_initialized",
                return_value=True,
            ),
            patch(
                "ultralytics_dfine.loss.multitask.distributed.all_reduce",
                side_effect=add_remote_count,
            ),
            patch(
                "ultralytics_dfine.loss.multitask.distributed.get_world_size",
                return_value=2,
            ),
        ):
            loss = criterion._cross_pose_visibility_negative(
                output,
                eligible,
                weight,
            )

        expected = torch.log(torch.tensor(2.0)) * weight / 2
        torch.testing.assert_close(loss, expected)


class CrossPoseDetectorNegativeTests(unittest.TestCase):
    def test_only_opposite_detector_class_receives_configured_weight(
        self,
    ) -> None:
        weight = 0.25
        criterion = MultiTaskCriterion(
            loss_config=MultiTaskLossConfig(
                cross_pose_detector_negative_weight=weight,
            )
        )
        cases = (
            (HeadId.PERSON_POSE, 7, 4),
            (HeadId.ROBOT_POSE, 4, 7),
        )
        for active_head, positive_class, negative_class in cases:
            with self.subTest(active_head=active_head):
                valid = torch.nn.functional.one_hot(
                    torch.tensor(positive_class),
                    num_classes=8,
                ).bool()
                target = {
                    "labels": torch.tensor([positive_class]),
                    "boxes": torch.tensor([[0.5, 0.5, 0.4, 0.6]]),
                    "valid_detection_classes": valid,
                    **(
                        {"person_negative_eligible": True}
                        if active_head == HeadId.ROBOT_POSE
                        else {"robot_negative_eligible": True}
                    ),
                }

                weighted = criterion._detection_targets(
                    [target],
                    active_head,
                )

                self.assertIsNot(weighted[0], target)
                self.assertNotIn("detection_class_loss_weights", target)
                expected = valid.float()
                expected[negative_class] = weight
                torch.testing.assert_close(
                    weighted[0]["detection_class_loss_weights"],
                    expected,
                )

    def test_robot_batch_person_negative_is_record_masked(self) -> None:
        criterion = MultiTaskCriterion(
            loss_config=MultiTaskLossConfig(
                person_batch_robot_detector_negative_weight=0.0,
                robot_batch_person_detector_negative_weight=0.25,
            )
        )
        valid = torch.nn.functional.one_hot(
            torch.tensor(4),
            num_classes=8,
        ).bool()
        targets = [
            {
                "labels": torch.tensor([4]),
                "boxes": torch.tensor([[0.5, 0.5, 0.4, 0.6]]),
                "valid_detection_classes": valid,
                "person_negative_eligible": eligible,
            }
            for eligible in (False, True)
        ]

        weighted = criterion._detection_targets(
            targets,
            HeadId.ROBOT_POSE,
        )

        self.assertIs(weighted[0], targets[0])
        self.assertNotIn("detection_class_loss_weights", weighted[0])
        expected = valid.float()
        expected[7] = 0.25
        torch.testing.assert_close(
            weighted[1]["detection_class_loss_weights"],
            expected,
        )

    def test_person_batch_robot_negative_is_record_masked(self) -> None:
        criterion = MultiTaskCriterion(
            loss_config=MultiTaskLossConfig(
                person_batch_robot_detector_negative_weight=0.25,
                robot_batch_person_detector_negative_weight=0.0,
            )
        )
        valid = torch.nn.functional.one_hot(
            torch.tensor(7),
            num_classes=8,
        ).bool()
        targets = [
            {
                "labels": torch.tensor([7]),
                "boxes": torch.tensor([[0.5, 0.5, 0.4, 0.6]]),
                "valid_detection_classes": valid,
                "robot_negative_eligible": eligible,
            }
            for eligible in (False, True)
        ]

        weighted = criterion._detection_targets(
            targets,
            HeadId.PERSON_POSE,
        )

        self.assertIs(weighted[0], targets[0])
        self.assertNotIn("detection_class_loss_weights", weighted[0])
        expected = valid.float()
        expected[4] = 0.25
        torch.testing.assert_close(
            weighted[1]["detection_class_loss_weights"],
            expected,
        )

    def test_person_batch_robot_negative_requires_explicit_eligibility(
        self,
    ) -> None:
        criterion = MultiTaskCriterion(
            loss_config=MultiTaskLossConfig(
                person_batch_robot_detector_negative_weight=0.25,
            )
        )
        target = {
            "labels": torch.tensor([7]),
            "boxes": torch.tensor([[0.5, 0.5, 0.4, 0.6]]),
        }

        with self.assertRaisesRegex(TypeError, "boolean eligibility"):
            criterion._detection_targets([target], HeadId.PERSON_POSE)

    def test_robot_batch_person_negative_requires_explicit_eligibility(
        self,
    ) -> None:
        criterion = MultiTaskCriterion(
            loss_config=MultiTaskLossConfig(
                robot_batch_person_detector_negative_weight=0.25,
            )
        )
        target = {
            "labels": torch.tensor([4]),
            "boxes": torch.tensor([[0.5, 0.5, 0.4, 0.6]]),
        }

        with self.assertRaisesRegex(TypeError, "boolean eligibility"):
            criterion._detection_targets([target], HeadId.ROBOT_POSE)

    def test_default_off_and_non_pose_batches_preserve_target_identity(
        self,
    ) -> None:
        targets = [
            {
                "labels": torch.tensor([7]),
                "boxes": torch.tensor([[0.5, 0.5, 0.4, 0.6]]),
            }
        ]
        default = MultiTaskCriterion()
        enabled = MultiTaskCriterion(
            loss_config=MultiTaskLossConfig(
                cross_pose_detector_negative_weight=0.5,
            )
        )

        self.assertIs(
            default._detection_targets(targets, HeadId.PERSON_POSE),
            targets,
        )
        self.assertIs(
            enabled._detection_targets(targets, HeadId.OBJECT),
            targets,
        )


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
        self.assertEqual(
            set(losses),
            {"field_class", "field_point"},
        )
        self.assertTrue(all(torch.isfinite(loss) for loss in losses.values()))
        torch.stack(tuple(losses.values())).sum().backward()
        self.assertIsNotNone(output["pred_logits"].grad)
        self.assertIsNotNone(output["pred_points"].grad)

    def test_strict_quality_uses_detached_localization_target(self) -> None:
        predicted_point = torch.tensor([[[0.55, 0.5]]], requires_grad=True)
        area = 0.5 * 0.5
        quality = torch.exp(torch.tensor(-(0.05**2) / (2 * area * 0.1**2)))
        logit = torch.logit(quality).reshape(1, 1, 1).requires_grad_()
        output = {
            "pred_logits": logit,
            "pred_points": predicted_point,
        }
        targets = [
            {
                "labels": torch.tensor([0]),
                "points": torch.tensor([[0.5, 0.5]]),
                "boxes": torch.tensor([[0.5, 0.5, 0.5, 0.5]]),
            }
        ]

        losses, _ = FieldFeatureCriterion(
            1,
            config=FieldLossConfig(
                classification_mode="strict_quality",
                point_weight=10,
            ),
        )(output, targets)

        torch.testing.assert_close(
            losses["field_class"],
            torch.tensor(0.0),
            atol=1e-6,
            rtol=0,
        )
        torch.testing.assert_close(
            losses["field_point"],
            torch.tensor(0.5),
        )

    def test_area_normalized_loss_uses_box_scale_and_smooth_l1(self) -> None:
        predicted_point = torch.tensor(
            [[[0.502, 0.5]]],
            requires_grad=True,
        )
        output = {
            "pred_logits": torch.tensor([[[8.0]]], requires_grad=True),
            "pred_points": predicted_point,
        }
        targets = [
            {
                "labels": torch.tensor([0]),
                "points": torch.tensor([[0.5, 0.5]]),
                "boxes": torch.tensor([[0.5, 0.5, 0.02, 0.02]]),
            }
        ]

        losses, _ = FieldFeatureCriterion(
            1,
            config=FieldLossConfig(
                class_weight=0,
                point_weight=0,
                area_normalized_weight=0.1,
            ),
        )(output, targets)

        self.assertEqual(
            set(losses),
            {
                "field_class",
                "field_point",
                "field_point_area_normalized",
            },
        )
        torch.testing.assert_close(
            losses["field_point_area_normalized"],
            torch.tensor(0.005),
            atol=1e-6,
            rtol=0,
        )
        sum(losses.values()).backward()
        torch.testing.assert_close(
            predicted_point.grad,
            torch.tensor([[[5.0, 0.0]]]),
            atol=1e-4,
            rtol=0,
        )

    def test_area_normalized_loss_clamps_box_scale(self) -> None:
        output = {
            "pred_logits": torch.tensor([[[8.0, -8.0], [-8.0, 8.0]]]),
            "pred_points": torch.tensor([[[0.201, 0.2], [0.81, 0.8]]]),
        }
        targets = [
            {
                "labels": torch.tensor([0, 1]),
                "points": torch.tensor([[0.2, 0.2], [0.8, 0.8]]),
                "boxes": torch.tensor(
                    [
                        [0.2, 0.2, 0.001, 0.001],
                        [0.8, 0.8, 0.5, 0.5],
                    ]
                ),
            }
        ]

        losses, _ = FieldFeatureCriterion(
            2,
            config=FieldLossConfig(
                class_weight=0,
                point_weight=0,
                area_normalized_weight=1,
            ),
        )(output, targets)

        torch.testing.assert_close(
            losses["field_point_area_normalized"],
            torch.tensor(0.05),
            atol=1e-6,
            rtol=0,
        )

    def test_area_normalized_loss_requires_boxes_only_for_matches(
        self,
    ) -> None:
        criterion = FieldFeatureCriterion(
            1,
            config=FieldLossConfig(area_normalized_weight=0.1),
        )
        output = {
            "pred_logits": torch.zeros(1, 1, 1),
            "pred_points": torch.zeros(1, 1, 2),
        }
        with self.assertRaisesRegex(ValueError, "requires target boxes"):
            criterion(
                output,
                [
                    {
                        "labels": torch.tensor([0]),
                        "points": torch.tensor([[0.5, 0.5]]),
                    }
                ],
            )

        losses, _ = criterion(
            output,
            [
                {
                    "labels": torch.empty(0, dtype=torch.long),
                    "points": torch.empty(0, 2),
                }
            ],
        )
        torch.testing.assert_close(
            losses["field_point_area_normalized"],
            torch.tensor(0.0),
        )


if __name__ == "__main__":
    unittest.main()
