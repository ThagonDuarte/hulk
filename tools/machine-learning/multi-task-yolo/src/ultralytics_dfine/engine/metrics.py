"""Independent average-precision metrics for multi-task validation."""

# ruff: noqa: TRY003

import math
from bisect import bisect_left
from collections import defaultdict
from collections.abc import Sequence
from typing import Literal, TypedDict

import torch
from scipy.optimize import linear_sum_assignment
from torch import Tensor

from ultralytics_dfine.schemas import (
    COCO_OKS_SIGMAS,
    PoseSchemaConfig,
)
from ultralytics_dfine.schemas import (
    DHRP_OKS_K as _DHRP_OKS_K,
)

OKS_THRESHOLDS = tuple(value / 100 for value in range(50, 100, 5))
COCO_OKS_K = tuple(2 * sigma for sigma in COCO_OKS_SIGMAS)
DHRP_OKS_K = _DHRP_OKS_K


class APResult(TypedDict):
    """Summary values averaged over the configured match thresholds."""

    ap: float
    ap50: float
    ap75: float
    ar: float


class _DetectionRecord(TypedDict):
    score: float
    matches: tuple[bool, ...]


def oks_k_from_schema(schema: PoseSchemaConfig) -> tuple[float, ...]:
    """Return OKS ``k`` values, converting COCO sigmas when required."""
    if schema.oks_uses_direct_k:
        return schema.oks_sigmas
    return tuple(2 * sigma for sigma in schema.oks_sigmas)


def _box_areas(boxes: Tensor, box_format: Literal["xywh", "xyxy"]) -> Tensor:
    if box_format == "xywh":
        sizes = boxes[:, 2:4]
    else:
        sizes = boxes[:, 2:4] - boxes[:, 0:2]
    return (
        sizes.clamp_min(0).prod(dim=1).clamp_min(torch.finfo(torch.float64).eps)
    )


def _average_precision(
    records: list[_DetectionRecord], target_count: int
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
            index = bisect_left(recalls, recall)
            interpolated += (
                precisions[index] if index < len(precisions) else 0.0
            )
        values.append(interpolated / 101)
    return tuple(values)


def _result(records: list[_DetectionRecord], target_count: int) -> APResult:
    per_threshold = _average_precision(records, target_count)
    recalls = [
        sum(record["matches"][index] for record in records) / target_count
        if target_count
        else 0.0
        for index in range(len(OKS_THRESHOLDS))
    ]
    return {
        "ap": sum(per_threshold) / len(per_threshold),
        "ap50": per_threshold[0],
        "ap75": per_threshold[5],
        "ar": sum(recalls) / len(recalls),
    }


def _greedy_matches(
    similarities: Tensor,
    scores: Tensor,
) -> list[_DetectionRecord]:
    order = torch.argsort(scores, descending=True, stable=True).tolist()
    no_matches = tuple(False for _ in OKS_THRESHOLDS)
    if similarities.shape[1] == 0:
        return [
            {
                "score": float(scores[prediction_index]),
                "matches": no_matches,
            }
            for prediction_index in order
        ]
    matched_targets = [set[int]() for _ in OKS_THRESHOLDS]
    saturated_thresholds = 0
    records: list[_DetectionRecord] = []
    for order_index, prediction_index in enumerate(order):
        if saturated_thresholds == len(OKS_THRESHOLDS):
            records.extend(
                {
                    "score": float(scores[remaining_index]),
                    "matches": no_matches,
                }
                for remaining_index in order[order_index:]
            )
            break
        matches = []
        for threshold_index, threshold in enumerate(OKS_THRESHOLDS):
            if len(matched_targets[threshold_index]) == similarities.shape[1]:
                matches.append(False)
                continue
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
                if (
                    len(matched_targets[threshold_index])
                    == similarities.shape[1]
                ):
                    saturated_thresholds += 1
        records.append(
            {
                "score": float(scores[prediction_index]),
                "matches": tuple(matches),
            }
        )
    return records


class KeypointOKSAccumulator:
    """Accumulate single-class pose AP/AR using target-area-normalized OKS."""

    def __init__(
        self,
        k: Sequence[float],
        *,
        box_format: Literal["xywh", "xyxy"] = "xywh",
    ) -> None:
        if not k or any(value <= 0 for value in k):
            raise ValueError("OKS k values must be positive")
        self.k = torch.tensor(tuple(k), dtype=torch.float64)
        self.box_format: Literal["xywh", "xyxy"] = box_format
        self.reset()

    @classmethod
    def from_schema(
        cls,
        schema: PoseSchemaConfig,
        *,
        box_format: Literal["xywh", "xyxy"] = "xywh",
    ) -> "KeypointOKSAccumulator":
        """Create an accumulator for the person or robot pose schema."""
        return cls(oks_k_from_schema(schema), box_format=box_format)

    def reset(self) -> None:
        """Discard all accumulated images."""
        self._records: list[_DetectionRecord] = []
        self._target_count = 0

    def update(
        self,
        predicted_keypoints: Tensor,
        target_keypoints: Tensor,
        target_boxes: Tensor,
        *,
        predicted_scores: Tensor | None = None,
        visibility: Tensor | None = None,
    ) -> None:
        """Add one image of pose predictions and targets.

        Keypoints are ``[instances, keypoints, xy]``. A third prediction
        channel can supply keypoint confidence when ``predicted_scores`` is
        omitted. A third target channel supplies visibility when an explicit
        mask is omitted.
        """
        predictions = predicted_keypoints.detach().to("cpu", torch.float64)
        targets = target_keypoints.detach().to("cpu", torch.float64)
        boxes = target_boxes.detach().to("cpu", torch.float64)
        self._validate_pose_inputs(predictions, targets, boxes)
        if predicted_scores is None:
            if predictions.shape[-1] < 3:
                raise ValueError(
                    "Prediction scores or confidence channels required"
                )
            scores = predictions[..., 2].mean(dim=1)
        else:
            scores = predicted_scores.detach().to("cpu", torch.float64)
        if scores.shape != (predictions.shape[0],):
            raise ValueError("Prediction scores must have shape [instances]")
        if visibility is None:
            visible = (
                targets[..., 2] > 0
                if targets.shape[-1] >= 3
                else torch.ones(targets.shape[:2], dtype=torch.bool)
            )
        else:
            visible = visibility.detach().to("cpu", torch.bool)
        if visible.shape != targets.shape[:2]:
            raise ValueError("Visibility must match target keypoint dimensions")

        keep = visible.any(dim=1)
        targets = targets[keep]
        boxes = boxes[keep]
        visible = visible[keep]
        self._target_count += targets.shape[0]
        similarities = self._oks(
            predictions[..., :2], targets[..., :2], boxes, visible
        )
        self._records.extend(_greedy_matches(similarities, scores))

    def _validate_pose_inputs(
        self,
        predictions: Tensor,
        targets: Tensor,
        boxes: Tensor,
    ) -> None:
        if predictions.ndim != 3 or predictions.shape[1] != self.k.numel():
            raise ValueError(
                "Predictions do not match the configured keypoints"
            )
        if targets.ndim != 3 or targets.shape[1] != self.k.numel():
            raise ValueError("Targets do not match the configured keypoints")
        if predictions.shape[2] < 2 or targets.shape[2] < 2:
            raise ValueError("Keypoints must contain x and y coordinates")
        if boxes.shape != (targets.shape[0], 4):
            raise ValueError("Target boxes must have shape [instances, 4]")

    def _oks(
        self,
        predictions: Tensor,
        targets: Tensor,
        boxes: Tensor,
        visible: Tensor,
    ) -> Tensor:
        if predictions.shape[0] == 0 or targets.shape[0] == 0:
            return torch.empty((predictions.shape[0], targets.shape[0]))
        squared_distance = (
            (predictions[:, None] - targets[None, :]).square().sum(dim=-1)
        )
        denominator = (
            2
            * _box_areas(boxes, self.box_format)[None, :, None]
            * self.k.square()[None, None, :]
        )
        per_keypoint = torch.exp(-squared_distance / denominator)
        weights = visible.to(torch.float64)[None, :, :]
        return (per_keypoint * weights).sum(dim=-1) / weights.sum(dim=-1)

    def compute(self) -> APResult:
        """Compute accumulated 101-point AP and maximum-detection AR."""
        return _result(self._records, self._target_count)


class FieldPointAPAccumulator:
    """Accumulate class-aware, target-area-normalized field point AP."""

    def __init__(
        self,
        normalization: float,
        *,
        box_format: Literal["xywh", "xyxy"] = "xywh",
    ) -> None:
        if normalization <= 0:
            raise ValueError("Point normalization must be positive")
        self.normalization = normalization
        self.box_format: Literal["xywh", "xyxy"] = box_format
        self.reset()

    def reset(self) -> None:
        """Discard all accumulated images."""
        self._records: defaultdict[int, list[_DetectionRecord]] = defaultdict(
            list
        )
        self._target_counts: defaultdict[int, int] = defaultdict(int)

    def update(
        self,
        predictions: Tensor,
        target_points: Tensor,
        target_classes: Tensor,
        target_boxes: Tensor,
    ) -> None:
        """Add one image; predictions have columns ``[x, y, score, class]``."""
        predicted = predictions.detach().to("cpu", torch.float64)
        points = target_points.detach().to("cpu", torch.float64)
        classes = target_classes.detach().to("cpu", torch.int64)
        boxes = target_boxes.detach().to("cpu", torch.float64)
        if predicted.ndim != 2 or predicted.shape[1] != 4:
            raise ValueError("Predictions must have shape [instances, 4]")
        if points.ndim != 2 or points.shape[1] != 2:
            raise ValueError("Target points must have shape [instances, 2]")
        if classes.shape != (points.shape[0],):
            raise ValueError("Target classes must have shape [instances]")
        if boxes.shape != (points.shape[0], 4):
            raise ValueError("Target boxes must have shape [instances, 4]")

        class_ids = {int(value) for value in predicted[:, 3].tolist()}
        class_ids.update(int(value) for value in classes.tolist())
        for class_id in class_ids:
            predicted_mask = predicted[:, 3] == class_id
            target_mask = classes == class_id
            class_predictions = predicted[predicted_mask]
            class_points = points[target_mask]
            class_boxes = boxes[target_mask]
            self._target_counts[class_id] += class_points.shape[0]
            similarities = self._similarities(
                class_predictions[:, :2],
                class_points,
                class_boxes,
            )
            self._records[class_id].extend(
                _greedy_matches(similarities, class_predictions[:, 2])
            )

    def _similarities(
        self,
        predictions: Tensor,
        targets: Tensor,
        boxes: Tensor,
    ) -> Tensor:
        if predictions.shape[0] == 0 or targets.shape[0] == 0:
            return torch.empty((predictions.shape[0], targets.shape[0]))
        squared_distance = (
            (predictions[:, None] - targets[None, :]).square().sum(dim=-1)
        )
        denominator = (
            2
            * _box_areas(boxes, self.box_format)[None, :]
            * self.normalization**2
        )
        return torch.exp(-squared_distance / denominator)

    def compute(self) -> APResult:
        """Compute target-class macro AP and AR."""
        results_by_class = self.compute_per_class()
        if not results_by_class:
            return {"ap": 0.0, "ap50": 0.0, "ap75": 0.0, "ar": 0.0}
        results = list(results_by_class.values())
        return {
            "ap": sum(result["ap"] for result in results) / len(results),
            "ap50": sum(result["ap50"] for result in results) / len(results),
            "ap75": sum(result["ap75"] for result in results) / len(results),
            "ar": sum(result["ar"] for result in results) / len(results),
        }

    def compute_per_class(self) -> dict[int, APResult]:
        """Compute AP and AR for every class containing targets."""
        target_classes = [
            class_id
            for class_id, count in self._target_counts.items()
            if count > 0
        ]
        return {
            class_id: _result(
                self._records[class_id],
                self._target_counts[class_id],
            )
            for class_id in target_classes
        }


class FieldPointLocalizationAccumulator:
    """Class-aware field localization in validation-input pixels."""

    def __init__(
        self,
        *,
        confidence: float = 0.25,
        pck_thresholds: Sequence[float] = (5.0, 10.0, 20.0),
    ) -> None:
        self.confidence = confidence
        self.pck_thresholds = tuple(pck_thresholds)
        self.errors: list[float] = []
        self.target_count = 0
        self.prediction_count = 0

    def update(
        self,
        predictions: Tensor,
        target_points: Tensor,
        target_classes: Tensor,
    ) -> None:
        predicted = predictions.detach().to("cpu", torch.float64)
        targets = target_points.detach().to("cpu", torch.float64)
        classes = target_classes.detach().to("cpu", torch.int64)
        predicted = predicted[
            predicted[:, 2].isfinite() & (predicted[:, 2] >= self.confidence)
        ]
        self.target_count += targets.shape[0]
        self.prediction_count += predicted.shape[0]
        class_ids = {int(value) for value in classes.tolist()}
        for class_id in class_ids:
            class_predictions = predicted[predicted[:, 3] == class_id, :2]
            class_targets = targets[classes == class_id]
            if class_predictions.shape[0] == 0 or class_targets.shape[0] == 0:
                continue
            distances = torch.cdist(class_predictions, class_targets)
            prediction_indices, target_indices = linear_sum_assignment(
                distances.numpy()
            )
            self.errors.extend(
                float(distances[prediction_index, target_index])
                for prediction_index, target_index in zip(
                    prediction_indices,
                    target_indices,
                    strict=True,
                )
            )

    def compute(self) -> dict[str, float]:
        errors = torch.tensor(self.errors, dtype=torch.float64)
        values = {
            "target_count": float(self.target_count),
            "prediction_count": float(self.prediction_count),
            "matched_fraction": (
                len(self.errors) / self.target_count
                if self.target_count
                else math.nan
            ),
            "mean_error_px": float(errors.mean())
            if errors.numel()
            else math.nan,
            "median_error_px": (
                float(errors.median()) if errors.numel() else math.nan
            ),
            "rmse_error_px": (
                float(errors.square().mean().sqrt())
                if errors.numel()
                else math.nan
            ),
        }
        for threshold in self.pck_thresholds:
            key = f"pck_{int(threshold)}px"
            values[key] = (
                float((errors <= threshold).sum()) / self.target_count
                if self.target_count
                else math.nan
            )
        return values
