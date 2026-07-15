import unittest
from types import SimpleNamespace
from typing import Any, cast

import torch

from ultralytics_dfine.engine.metrics import (
    COCO_OKS_K,
    DHRP_OKS_K,
    FieldPointAPAccumulator,
    FieldPointLocalizationAccumulator,
    KeypointOKSAccumulator,
)
from ultralytics_dfine.engine.multitask_validator import MultiTaskValidator
from ultralytics_dfine.nn import DFINEMultiTaskModel
from ultralytics_dfine.schemas import (
    COCO_OKS_SIGMAS,
)
from ultralytics_dfine.schemas import (
    DHRP_OKS_K as SCHEMA_DHRP_K,
)


class KeypointOKSAccumulatorTests(unittest.TestCase):
    def test_schema_constants_have_required_conversion(self) -> None:
        self.assertEqual(
            COCO_OKS_K, tuple(2 * value for value in COCO_OKS_SIGMAS)
        )
        self.assertEqual(DHRP_OKS_K, SCHEMA_DHRP_K)

    def test_perfect_predictions_have_full_ap_and_ar(self) -> None:
        metric = KeypointOKSAccumulator((0.1, 0.1))
        targets = torch.tensor([[[20.0, 30.0, 2.0], [40.0, 50.0, 2.0]]])
        metric.update(
            targets[..., :2],
            targets,
            torch.tensor([[0.0, 0.0, 100.0, 100.0]]),
            predicted_scores=torch.tensor([0.9]),
        )

        self.assertEqual(
            metric.compute(), {"ap": 1.0, "ap50": 1.0, "ap75": 1.0, "ar": 1.0}
        )

    def test_incorrect_predictions_have_zero_ap_and_ar(self) -> None:
        metric = KeypointOKSAccumulator((0.05,))
        metric.update(
            torch.tensor([[[100.0, 100.0]]]),
            torch.tensor([[[0.0, 0.0, 1.0]]]),
            torch.tensor([[0.0, 0.0, 10.0, 10.0]]),
            predicted_scores=torch.tensor([0.9]),
        )

        self.assertEqual(metric.compute()["ap"], 0.0)
        self.assertEqual(metric.compute()["ar"], 0.0)

    def test_invisible_keypoints_do_not_affect_oks(self) -> None:
        metric = KeypointOKSAccumulator((0.1, 0.1))
        metric.update(
            torch.tensor([[[10.0, 10.0], [500.0, 500.0]]]),
            torch.tensor([[[10.0, 10.0, 1.0], [20.0, 20.0, 0.0]]]),
            torch.tensor([[0.0, 0.0, 100.0, 100.0]]),
            predicted_scores=torch.tensor([0.8]),
        )

        self.assertEqual(metric.compute()["ap"], 1.0)


class FieldPointAPAccumulatorTests(unittest.TestCase):
    def test_perfect_predictions_have_full_ap_and_ar(self) -> None:
        metric = FieldPointAPAccumulator(0.1)
        metric.update(
            torch.tensor([[10.0, 10.0, 0.9, 0.0], [30.0, 30.0, 0.8, 1.0]]),
            torch.tensor([[10.0, 10.0], [30.0, 30.0]]),
            torch.tensor([0, 1]),
            torch.tensor([[0.0, 0.0, 100.0, 100.0]]).repeat(2, 1),
        )

        self.assertEqual(metric.compute()["ap"], 1.0)
        self.assertEqual(metric.compute()["ar"], 1.0)

    def test_wrong_class_does_not_match_same_location(self) -> None:
        metric = FieldPointAPAccumulator(0.1)
        metric.update(
            torch.tensor([[10.0, 10.0, 0.9, 1.0]]),
            torch.tensor([[10.0, 10.0]]),
            torch.tensor([0]),
            torch.tensor([[0.0, 0.0, 100.0, 100.0]]),
        )

        self.assertEqual(metric.compute()["ap"], 0.0)
        self.assertEqual(metric.compute()["ar"], 0.0)

    def test_per_class_results_are_exposed(self) -> None:
        metric = FieldPointAPAccumulator(0.1)
        metric.update(
            torch.tensor([[10.0, 10.0, 0.9, 0.0]]),
            torch.tensor([[10.0, 10.0]]),
            torch.tensor([0]),
            torch.tensor([[0.0, 0.0, 100.0, 100.0]]),
        )

        results = metric.compute_per_class()

        self.assertEqual(results[0]["ap"], 1.0)


class FieldPointLocalizationAccumulatorTests(unittest.TestCase):
    def test_reports_pixel_error_and_pck(self) -> None:
        metric = FieldPointLocalizationAccumulator(confidence=0.25)
        metric.update(
            torch.tensor(
                [
                    [13.0, 14.0, 0.9, 0.0],
                    [30.0, 30.0, 0.1, 0.0],
                ]
            ),
            torch.tensor([[10.0, 10.0]]),
            torch.tensor([0]),
        )

        result = metric.compute()

        self.assertEqual(result["mean_error_px"], 5.0)
        self.assertEqual(result["pck_5px"], 1.0)
        self.assertEqual(result["prediction_count"], 1.0)

    def test_wrong_class_cannot_match(self) -> None:
        metric = FieldPointLocalizationAccumulator(confidence=0.25)
        metric.update(
            torch.tensor([[10.0, 10.0, 0.9, 1.0]]),
            torch.tensor([[10.0, 10.0]]),
            torch.tensor([0]),
        )

        result = metric.compute()

        self.assertEqual(result["matched_fraction"], 0.0)
        self.assertEqual(result["pck_20px"], 0.0)


class MultiTaskValidatorTests(unittest.TestCase):
    def test_object_metric_ignores_predictions_for_invalid_classes(
        self,
    ) -> None:
        class CaptureMetric:
            def __init__(self) -> None:
                self.predictions: list[dict[str, torch.Tensor]] = []

            def update(
                self,
                predictions: list[dict[str, torch.Tensor]],
                _targets: list[dict[str, torch.Tensor]],
            ) -> None:
                self.predictions = predictions

        validator = MultiTaskValidator(
            cast(DFINEMultiTaskModel, SimpleNamespace()),
            {},
            device="cpu",
        )
        metric = CaptureMetric()
        output = torch.tensor(
            [
                [0, 0, 10, 10, 0.9, 0],
                [0, 0, 10, 10, 0.8, 7],
            ],
            dtype=torch.float32,
        )
        target: dict[str, Any] = {
            "boxes": torch.tensor([[0.5, 0.5, 0.2, 0.2]]),
            "labels": torch.tensor([0]),
            "valid_detection_classes": torch.tensor(
                [True, True, True, True, True, True, True, False]
            ),
        }

        validator._update_objects(
            cast(Any, metric),
            output,
            target,
            100,
            100,
        )

        self.assertEqual(metric.predictions[0]["labels"].tolist(), [0])


if __name__ == "__main__":
    unittest.main()
