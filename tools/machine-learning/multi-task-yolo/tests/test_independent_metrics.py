import unittest
from types import SimpleNamespace
from typing import Any, cast

import torch

from ultralytics_dfine.engine import metrics as independent_metrics
from ultralytics_dfine.engine.metrics import (
    COCO_OKS_K,
    DHRP_OKS_K,
    OKS_THRESHOLDS,
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


def _legacy_average_precision(
    records: list[dict[str, Any]], target_count: int
) -> tuple[float, ...]:
    if target_count == 0:
        return tuple(0.0 for _ in OKS_THRESHOLDS)
    ordered = sorted(records, key=lambda record: record["score"], reverse=True)
    values = []
    for threshold_index in range(len(OKS_THRESHOLDS)):
        true_positives = 0
        false_positives = 0
        recalls = []
        precisions = []
        for record in ordered:
            if record["matches"][threshold_index]:
                true_positives += 1
            else:
                false_positives += 1
            recalls.append(true_positives / target_count)
            precisions.append(
                true_positives / (true_positives + false_positives)
            )

        for index in range(len(precisions) - 2, -1, -1):
            precisions[index] = max(precisions[index], precisions[index + 1])
        interpolated = 0.0
        for recall_level in range(101):
            recall = recall_level / 100
            candidates = [
                precision
                for precision, observed_recall in zip(
                    precisions,
                    recalls,
                    strict=True,
                )
                if observed_recall >= recall
            ]
            interpolated += max(candidates, default=0.0)
        values.append(interpolated / 101)
    return tuple(values)


def _legacy_greedy_matches(
    similarities: torch.Tensor,
    scores: torch.Tensor,
) -> list[dict[str, Any]]:
    order = torch.argsort(scores, descending=True, stable=True).tolist()
    matched_targets = [set[int]() for _ in OKS_THRESHOLDS]
    records = []
    for prediction_index in order:
        matches = []
        for threshold_index, threshold in enumerate(OKS_THRESHOLDS):
            available = [
                target_index
                for target_index in range(similarities.shape[1])
                if target_index not in matched_targets[threshold_index]
            ]
            if not available:
                matches.append(False)
                continue
            available_similarities = similarities[prediction_index, available]
            best_offset = int(torch.argmax(available_similarities).item())
            best_target = available[best_offset]
            is_match = (
                float(similarities[prediction_index, best_target]) >= threshold
            )
            matches.append(is_match)
            if is_match:
                matched_targets[threshold_index].add(best_target)
        records.append(
            {
                "score": float(scores[prediction_index]),
                "matches": tuple(matches),
            }
        )
    return records


class MetricFastPathEquivalenceTests(unittest.TestCase):
    def test_average_precision_matches_legacy_reference_exactly(self) -> None:
        generator = torch.Generator().manual_seed(20260716)
        for record_count in (0, 1, 2, 17, 301):
            for target_count in (0, 1, 7, 53):
                scores = torch.randint(
                    0,
                    8,
                    (record_count,),
                    generator=generator,
                ).to(torch.float64)
                matches = torch.randint(
                    0,
                    2,
                    (record_count, len(OKS_THRESHOLDS)),
                    generator=generator,
                    dtype=torch.bool,
                )
                records = [
                    {
                        "score": float(scores[index]),
                        "matches": tuple(matches[index].tolist()),
                    }
                    for index in range(record_count)
                ]

                self.assertEqual(
                    independent_metrics._average_precision(
                        records,
                        target_count,
                    ),
                    _legacy_average_precision(records, target_count),
                )

    def test_greedy_matching_matches_legacy_reference_exactly(self) -> None:
        generator = torch.Generator().manual_seed(20260716)
        for prediction_count, target_count in (
            (0, 0),
            (5, 0),
            (0, 4),
            (1, 1),
            (8, 3),
            (300, 21),
        ):
            similarities = (
                torch.randint(
                    0,
                    21,
                    (prediction_count, target_count),
                    generator=generator,
                ).to(torch.float64)
                / 20
            )
            scores = torch.randint(
                0,
                8,
                (prediction_count,),
                generator=generator,
            ).to(torch.float64)

            self.assertEqual(
                independent_metrics._greedy_matches(similarities, scores),
                _legacy_greedy_matches(similarities, scores),
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
