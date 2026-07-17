"""Exact paired comparison of multi-task validation prediction records."""

# ruff: noqa: TRY003

import contextlib
import copy
import gzip
import io
import json
import math
from collections import defaultdict
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any, cast

import click
import faster_coco_eval.faster_eval_api_cpp as _coco_cpp
import numpy as np
import torch
from scipy.optimize import linear_sum_assignment
from torch import Tensor
from torchmetrics.detection.mean_ap import MeanAveragePrecision

from ultralytics_dfine.engine.metrics import (
    OKS_THRESHOLDS,
    FieldPointAPAccumulator,
    FieldPointLocalizationAccumulator,
    KeypointOKSAccumulator,
    oks_k_from_schema,
)
from ultralytics_dfine.engine.multitask_validator import (
    CROSS_POSE_PERSON_ON_ROBOT,
    CROSS_POSE_ROBOT_ON_PERSON,
)
from ultralytics_dfine.schemas import PERSON_POSE_SCHEMA, ROBOT_POSE_SCHEMA
from validation.multitask_data import write_json

COMPATIBLE_CONFIG_KEYS = (
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
    "cross_pose_negatives",
    "cross_pose_visibility_alphas",
)
PRIMARY_WEIGHTS = {
    "object/map": 0.5,
    "field/map": 0.2,
    "field/strict_map": 0.2,
    "field/localization/pck_5px": 0.1,
}
BOOTSTRAP_METRICS = (
    "object/map",
    "object/negative_fp_per_image_at_25",
    "field/map",
    "field/strict_map",
    "field/localization/pck_5px",
    "field/localization/matched_fraction",
    "field/localization/median_error_px",
    "person/map",
    "robot/map",
)
METRIC_TASK = {
    "object/map": "object",
    "object/negative_fp_per_image_at_25": "object",
    "field/map": "field_features",
    "field/strict_map": "field_features",
    "field/localization/pck_5px": "field_features",
    "field/localization/matched_fraction": "field_features",
    "field/localization/median_error_px": "field_features",
    "person/map": "person_pose",
    "robot/map": "robot_pose",
}
# Preserve the production metric's Python-division values bit-for-bit. NumPy
# linspace/arange land one or more ULPs above some exact AP boundaries.
_MATCH_THRESHOLDS = np.asarray(OKS_THRESHOLDS, dtype=np.float64)
_RECALL_THRESHOLDS = np.asarray(
    [level / 100 for level in range(101)],
    dtype=np.float64,
)
_SAVED_METRIC_TOLERANCE = 5e-6
_CROSS_POSE_RECORDS = "cross_pose_predictions.jsonl.gz"
_CROSS_POSE_TASKS = (
    CROSS_POSE_PERSON_ON_ROBOT,
    CROSS_POSE_ROBOT_ON_PERSON,
)
_SCORE_THRESHOLDS = (
    ("001", 0.001),
    ("05", 0.05),
    ("10", 0.1),
    ("25", 0.25),
    ("50", 0.5),
    ("75", 0.75),
)


def _load_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise TypeError(f"Expected JSON object: {path}")
    return cast("dict[str, Any]", value)


def _load_records(path: Path) -> dict[str, dict[str, object]]:
    if not path.is_file():
        raise FileNotFoundError(f"Predictions not found: {path}")
    records = {}
    with gzip.open(path, "rt", encoding="utf-8") as file:
        for line_number, line in enumerate(file, start=1):
            value = json.loads(line)
            if not isinstance(value, dict):
                raise TypeError(
                    f"Prediction line {line_number} is not an object"
                )
            key = value.get("image_key")
            if not isinstance(key, str):
                raise TypeError(
                    f"Prediction line {line_number} has no image_key"
                )
            if key in records:
                raise ValueError(f"Duplicate prediction image_key: {key}")
            records[key] = cast("dict[str, object]", value)
    return records


def _record_cluster_identity(
    key: str,
    record: Mapping[str, object],
) -> tuple[tuple[str, str], bool]:
    """Return a stable canonical-image identity without touching the FS."""
    task = record.get("task")
    if not isinstance(task, str):
        raise TypeError(f"Prediction record has no task: {key}")
    raw_path = record.get("path")
    if raw_path is not None:
        if not isinstance(raw_path, str):
            raise TypeError(f"Prediction record path is not a string: {key}")
        path = Path(raw_path)
        if not raw_path or not path.is_absolute():
            raise ValueError(f"Prediction record path must be absolute: {key}")
        # Validation writes raw_path.resolve(). Re-resolving here would make a
        # saved comparison depend on whether files or symlinks later move.
        return ("path", str(path)), False

    prefix = f"{task}:"
    if key.startswith(prefix):
        suffix = key[len(prefix) :]
        path = Path(suffix)
        if suffix and path.is_absolute():
            return ("path", str(path)), True
    # Synthetic and very old records have no recoverable path. Keep the full
    # task-qualified key so unrelated tasks can never be coupled by suffix.
    return ("image_key", key), True


def _fingerprint_digests(metadata: Mapping[str, object]) -> dict[str, str]:
    raw = metadata.get("dataset_fingerprints")
    if not isinstance(raw, dict):
        raise TypeError("Run metadata has no dataset_fingerprints mapping")
    result = {}
    for task, value in raw.items():
        if not isinstance(task, str) or not isinstance(value, dict):
            raise TypeError("Invalid dataset fingerprint entry")
        digest = value.get("digest")
        if not isinstance(digest, str):
            raise TypeError("Dataset fingerprint has no digest")
        result[task] = digest
    return result


def compatibility_report(  # noqa: C901
    baseline_config: Mapping[str, object],
    candidate_config: Mapping[str, object],
    baseline_metadata: Mapping[str, object],
    candidate_metadata: Mapping[str, object],
) -> dict[str, object]:
    """Return exact compatibility details and reject invalid comparisons."""

    def config_value(
        config: Mapping[str, object],
        key: str,
    ) -> object:
        value = config.get(key)
        if key == "cross_pose_negatives":
            return bool(value) if value is not None else False
        if key == "cross_pose_visibility_alphas":
            if not bool(config.get("cross_pose_negatives", False)):
                return None
            if isinstance(value, (list, tuple)):
                return tuple(value)
        if key in {
            "person_visibility_alphas",
            "robot_visibility_alphas",
        }:
            # Prediction records written before score calibration was added
            # are semantically the alpha-zero default.
            if value is None:
                return (0.0,)
            if isinstance(value, (list, tuple)):
                return tuple(value)
        return value

    mismatches = {
        key: {
            "baseline": config_value(baseline_config, key),
            "candidate": config_value(candidate_config, key),
        }
        for key in COMPATIBLE_CONFIG_KEYS
        if config_value(baseline_config, key)
        != config_value(candidate_config, key)
    }
    baseline_fingerprints = _fingerprint_digests(baseline_metadata)
    candidate_fingerprints = _fingerprint_digests(candidate_metadata)
    if baseline_fingerprints != candidate_fingerprints:
        mismatches["dataset_fingerprints"] = {
            "baseline": baseline_fingerprints,
            "candidate": candidate_fingerprints,
        }
    if mismatches:
        raise ValueError(
            "Incompatible validation runs: " + ", ".join(sorted(mismatches))
        )
    return {
        "compatible": True,
        "checked_config_keys": list(COMPATIBLE_CONFIG_KEYS),
        "dataset_fingerprints": baseline_fingerprints,
    }


def _mapping(value: object, context: str) -> Mapping[str, object]:
    if not isinstance(value, dict):
        raise TypeError(f"{context} must be an object")
    return cast("Mapping[str, object]", value)


def _tensor(value: object, *, dtype: torch.dtype = torch.float32) -> Tensor:
    return torch.tensor(value, dtype=dtype)


def _match_predictions(similarities: Tensor, scores: Tensor) -> np.ndarray:
    """Match one image exactly as the independent AP accumulators do."""
    matches = np.zeros(
        (scores.numel(), len(_MATCH_THRESHOLDS)),
        dtype=np.bool_,
    )
    target_count = similarities.shape[1]
    if scores.numel() == 0 or target_count == 0:
        return matches
    order = torch.argsort(scores, descending=True, stable=True)
    ordered = similarities[order]
    if target_count == 1:
        similarities_to_target = ordered[:, 0]
        for threshold_index, threshold in enumerate(_MATCH_THRESHOLDS):
            candidates = torch.nonzero(
                similarities_to_target >= threshold,
                as_tuple=False,
            )
            if candidates.numel():
                matches[int(candidates[0]), threshold_index] = True
        return matches
    claimed = [set[int]() for _ in _MATCH_THRESHOLDS]
    for prediction_index in range(ordered.shape[0]):
        for threshold_index, threshold in enumerate(_MATCH_THRESHOLDS):
            if len(claimed[threshold_index]) == target_count:
                continue
            available = [
                target_index
                for target_index in range(ordered.shape[1])
                if target_index not in claimed[threshold_index]
            ]
            if not available:
                continue
            values = ordered[prediction_index, available]
            best_offset = int(torch.argmax(values).item())
            best_target = available[best_offset]
            if float(ordered[prediction_index, best_target]) >= threshold:
                matches[prediction_index, threshold_index] = True
                claimed[threshold_index].add(best_target)
    return matches


@dataclass(frozen=True)
class _ClassAPData:
    target_counts: np.ndarray
    scores: np.ndarray
    image_indices: np.ndarray
    matches: np.ndarray
    interleaving_blocks: tuple[tuple[int, int], ...]


@dataclass(frozen=True)
class _APData:
    """Per-image matching sufficient statistics for exact case-weighted AP."""

    image_count: int
    classes: Mapping[int, _ClassAPData]

    def compute(self, weights: np.ndarray) -> dict[str, float]:
        if weights.shape != (self.image_count,):
            raise ValueError("Bootstrap weights do not match AP images")
        class_results = []
        for data in self.classes.values():
            target_count = int(np.dot(weights, data.target_counts))
            if target_count <= 0:
                continue
            detection_weights = weights[data.image_indices]
            class_results.append(
                _weighted_ap_result(
                    data.matches,
                    detection_weights,
                    target_count,
                    data.interleaving_blocks,
                )
            )
        if not class_results:
            return {"map": math.nan, "map50": math.nan, "map75": math.nan}
        return {
            "map": float(np.mean([result[0] for result in class_results])),
            "map50": float(np.mean([result[1] for result in class_results])),
            "map75": float(np.mean([result[2] for result in class_results])),
        }


def _weighted_ap_result(
    matches: np.ndarray,
    detection_weights: np.ndarray,
    target_count: int,
    interleaving_blocks: tuple[tuple[int, int], ...],
) -> tuple[float, float, float]:
    """Return COCO-style AP for a clustered empirical resample.

    Repeated copies of one detection can normally be collapsed into an integer
    weight. Equal-score detections from the same image are the exception: a
    repeated image contributes its full stable detection sequence once per
    copy. Expand only blocks whose match outcomes differ so that tied TP/FP
    ordering remains identical to an explicitly repeated record list.
    """
    if any(detection_weights[start] > 1 for start, _ in interleaving_blocks):
        match_parts = []
        weight_parts = []
        cursor = 0
        for start, end in interleaving_blocks:
            match_parts.append(matches[cursor:start])
            weight_parts.append(detection_weights[cursor:start])
            repeats = int(detection_weights[start])
            if not np.all(detection_weights[start:end] == repeats):
                raise RuntimeError(
                    "Tied detections from one image have different weights"
                )
            if repeats > 1:
                match_parts.append(np.tile(matches[start:end], (repeats, 1)))
                weight_parts.append(
                    np.ones((end - start) * repeats, dtype=np.int64)
                )
            else:
                match_parts.append(matches[start:end])
                weight_parts.append(detection_weights[start:end])
            cursor = end
        match_parts.append(matches[cursor:])
        weight_parts.append(detection_weights[cursor:])
        matches = np.concatenate(match_parts, axis=0)
        detection_weights = np.concatenate(weight_parts)
    active = detection_weights > 0
    if not np.any(active):
        return 0.0, 0.0, 0.0
    weights = detection_weights[active].astype(np.float64, copy=False)
    selected = matches[active]
    values = np.zeros(len(_MATCH_THRESHOLDS), dtype=np.float64)
    for threshold_index in range(len(_MATCH_THRESHOLDS)):
        true_positive = weights * selected[:, threshold_index]
        false_positive = weights - true_positive
        true_positive = np.cumsum(true_positive)
        false_positive = np.cumsum(false_positive)
        recall = true_positive / target_count
        precision = true_positive / (true_positive + false_positive)
        precision = np.maximum.accumulate(precision[::-1])[::-1]
        indices = np.searchsorted(recall, _RECALL_THRESHOLDS, side="left")
        valid = indices < precision.size
        sampled = np.zeros_like(_RECALL_THRESHOLDS)
        sampled[valid] = precision[indices[valid]]
        values[threshold_index] = sampled.mean()
    return float(values.mean()), float(values[0]), float(values[5])


def _finalize_ap_builders(
    image_count: int,
    target_counts: Mapping[int, np.ndarray],
    detections: Mapping[int, list[tuple[float, int, np.ndarray]]],
) -> _APData:
    classes = {}
    for class_id in sorted(set(target_counts) | set(detections)):
        entries = detections.get(class_id, [])
        # Python's stable sort matches COCO's stable mergesort for equal scores.
        entries = sorted(entries, key=lambda entry: entry[0], reverse=True)
        scores = np.asarray([entry[0] for entry in entries])
        image_indices = np.asarray(
            [entry[1] for entry in entries],
            dtype=np.int64,
        )
        matches = np.asarray(
            [entry[2] for entry in entries],
            dtype=np.bool_,
        ).reshape(-1, len(_MATCH_THRESHOLDS))
        interleaving_blocks = []
        start = 0
        while start < len(entries):
            end = start + 1
            while (
                end < len(entries)
                and scores[end] == scores[start]
                and image_indices[end] == image_indices[start]
            ):
                end += 1
            if end - start > 1 and np.any(matches[start:end] != matches[start]):
                interleaving_blocks.append((start, end))
            start = end
        classes[class_id] = _ClassAPData(
            target_counts=target_counts.get(
                class_id,
                np.zeros(image_count, dtype=np.int64),
            ),
            scores=scores,
            image_indices=image_indices,
            matches=matches,
            interleaving_blocks=tuple(interleaving_blocks),
        )
    return _APData(image_count=image_count, classes=classes)


def _build_object_metric(
    records: Sequence[Mapping[str, object]],
) -> MeanAveragePrecision:
    metric = MeanAveragePrecision(
        box_format="xyxy",
        iou_type="bbox",
        max_detection_thresholds=[1, 10, 300],
        backend="faster_coco_eval",
        sync_on_compute=False,
        class_metrics=False,
    )
    metric.warn_on_many_detections = False
    predictions = []
    targets = []
    for record in records:
        predicted = _mapping(record.get("predictions"), "predictions")
        expected = _mapping(record.get("targets"), "targets")
        predictions.append(
            {
                "boxes": _tensor(predicted.get("boxes_xyxy", [])).reshape(
                    -1,
                    4,
                ),
                "scores": _tensor(predicted.get("scores", [])),
                "labels": _tensor(
                    predicted.get("labels", []),
                    dtype=torch.long,
                ),
            }
        )
        targets.append(
            {
                "boxes": _tensor(expected.get("boxes_xyxy", [])).reshape(
                    -1,
                    4,
                ),
                "labels": _tensor(
                    expected.get("labels", []),
                    dtype=torch.long,
                ),
            }
        )
    metric.update(predictions, targets)
    return metric


@dataclass(frozen=True)
class _ObjectCocoAPData:
    """Opaque per-image records produced by the production COCO backend."""

    image_count: int
    image_ids: np.ndarray
    evaluations: np.ndarray
    parameters: object

    def compute(self, weights: np.ndarray) -> dict[str, float]:
        if weights.shape != (self.image_count,):
            raise ValueError("Bootstrap weights do not match object images")
        if not np.issubdtype(weights.dtype, np.integer) or np.any(weights < 0):
            raise ValueError(
                "Object bootstrap weights must be nonnegative ints"
            )
        if int(weights.sum()) == 0:
            return {"map": math.nan, "map50": math.nan, "map75": math.nan}

        parameters = copy.copy(self.parameters)
        parameters.imgIds = np.repeat(self.image_ids, weights).tolist()
        evaluations = np.repeat(self.evaluations, weights, axis=2)
        accumulated = _coco_cpp.COCOevalAccumulate(
            parameters,
            evaluations.reshape(-1).tolist(),
        )
        precision = np.asarray(accumulated["precision"])
        area_index = parameters.areaRngLbl.index("all")
        max_detection_index = len(parameters.maxDets) - 1
        precision = precision[:, :, :, area_index, max_detection_index]

        def mean_valid(values: np.ndarray) -> float:
            valid = values[values > -1]
            return float(valid.mean()) if valid.size else math.nan

        iou_thresholds = np.asarray(parameters.iouThrs)
        map50_index = int(np.flatnonzero(np.isclose(iou_thresholds, 0.5))[0])
        map75_index = int(np.flatnonzero(np.isclose(iou_thresholds, 0.75))[0])
        return {
            "map": mean_valid(precision),
            "map50": mean_valid(precision[map50_index]),
            "map75": mean_valid(precision[map75_index]),
        }


def _prepare_object_ap(
    records: Sequence[Mapping[str, object]],
) -> _ObjectCocoAPData:
    """Prepare the faster-coco-eval backend's exact per-image matches."""
    metric = _build_object_metric(records)
    backend = metric._coco_backend
    coco_predictions, coco_targets = backend._get_coco_datasets(
        metric.groundtruth_labels,
        metric.groundtruth_box,
        metric.groundtruth_mask,
        metric.groundtruth_crowds,
        metric.groundtruth_area,
        metric.detection_labels,
        metric.detection_box,
        metric.detection_mask,
        metric.detection_scores,
        metric.iou_type,
        average=metric.average,
    )
    evaluator = backend.cocoeval(
        coco_targets,
        coco_predictions,
        iouType="bbox",
        separate_eval=True,
    )
    evaluator.params.iouThrs = np.asarray(
        metric.iou_thresholds,
        dtype=np.float64,
    )
    evaluator.params.recThrs = np.asarray(
        metric.rec_thresholds,
        dtype=np.float64,
    )
    evaluator.params.maxDets = metric.max_detection_thresholds
    with contextlib.redirect_stdout(io.StringIO()):
        evaluator.evaluate()
    category_count = len(evaluator.params.catIds)
    area_count = len(evaluator.params.areaRng)
    image_ids = np.asarray(evaluator.params.imgIds)
    evaluations = np.asarray(
        evaluator._evalImgs_cpp,
        dtype=object,
    ).reshape(category_count, area_count, len(image_ids))
    return _ObjectCocoAPData(
        image_count=len(records),
        image_ids=image_ids,
        evaluations=evaluations,
        parameters=evaluator._paramsEval,
    )


def _field_similarities(
    predicted_points: Tensor,
    target_points: Tensor,
    target_boxes: Tensor,
    normalization: float,
) -> Tensor:
    if predicted_points.shape[0] == 0 or target_points.shape[0] == 0:
        return torch.empty((predicted_points.shape[0], target_points.shape[0]))
    squared_distance = (
        (predicted_points[:, None] - target_points[None, :])
        .square()
        .sum(dim=-1)
    )
    areas = target_boxes[:, 2:4].clamp_min(0).prod(dim=1)
    areas = areas.clamp_min(torch.finfo(target_boxes.dtype).eps)
    return torch.exp(
        -squared_distance / (2 * areas[None, :] * normalization**2)
    )


def _prepare_field_ap(
    records: Sequence[Mapping[str, object]],
    normalization: float,
) -> _APData:
    target_counts: defaultdict[int, np.ndarray] = defaultdict(
        lambda: np.zeros(len(records), dtype=np.int64)
    )
    detections: defaultdict[int, list[tuple[float, int, np.ndarray]]] = (
        defaultdict(list)
    )
    for image_index, record in enumerate(records):
        predictions = _mapping(record.get("predictions"), "predictions")
        targets = _mapping(record.get("targets"), "targets")
        predicted = _tensor(predictions.get("points", [])).reshape(-1, 4)
        points = _tensor(targets.get("points", [])).reshape(-1, 2)
        labels = _tensor(targets.get("labels", []), dtype=torch.long)
        boxes = _tensor(targets.get("boxes_cxcywh", [])).reshape(-1, 4)
        predicted_classes = predicted[:, 3].long()
        class_ids = set(predicted_classes.tolist()) | set(labels.tolist())
        for class_id in class_ids:
            predicted_mask = predicted_classes == class_id
            target_mask = labels == class_id
            class_predictions = predicted[predicted_mask].to(torch.float64)
            class_points = points[target_mask].to(torch.float64)
            class_boxes = boxes[target_mask].to(torch.float64)
            scores = class_predictions[:, 2]
            similarities = _field_similarities(
                class_predictions[:, :2],
                class_points,
                class_boxes,
                normalization,
            )
            matches = _match_predictions(similarities, scores)
            order = torch.argsort(scores, descending=True, stable=True)
            scores = scores[order]
            target_counts[class_id][image_index] = int(target_mask.sum())
            detections[class_id].extend(
                (float(score), image_index, matched)
                for score, matched in zip(
                    scores.tolist(),
                    matches,
                    strict=True,
                )
            )
    return _finalize_ap_builders(len(records), target_counts, detections)


def _pose_similarities(
    predictions: Tensor,
    targets: Tensor,
    boxes: Tensor,
    visible: Tensor,
    k: Tensor,
) -> Tensor:
    if predictions.shape[0] == 0 or targets.shape[0] == 0:
        return torch.empty((predictions.shape[0], targets.shape[0]))
    squared_distance = (
        (predictions[:, None] - targets[None, :]).square().sum(dim=-1)
    )
    areas = boxes[:, 2:4].clamp_min(0).prod(dim=1)
    areas = areas.clamp_min(torch.finfo(torch.float64).eps)
    denominator = 2 * areas[None, :, None] * k.square()[None, None, :]
    values = torch.exp(-squared_distance / denominator)
    weights = visible.to(torch.float64)[None, :, :]
    return (values * weights).sum(dim=-1) / weights.sum(dim=-1)


def _prepare_pose_ap(
    records: Sequence[Mapping[str, object]],
    *,
    prefix: str,
) -> _APData:
    schema = PERSON_POSE_SCHEMA if prefix == "person" else ROBOT_POSE_SCHEMA
    k = torch.tensor(oks_k_from_schema(schema), dtype=torch.float64)
    target_counts = {0: np.zeros(len(records), dtype=np.int64)}
    detections: dict[int, list[tuple[float, int, np.ndarray]]] = {0: []}
    for image_index, record in enumerate(records):
        predictions = _mapping(record.get("predictions"), "predictions")
        targets = _mapping(record.get("targets"), "targets")
        predicted = _tensor(predictions.get("keypoints", [])).reshape(
            -1,
            schema.keypoint_count,
            3,
        )
        scores = _tensor(predictions.get("scores", []))
        expected = _tensor(targets.get("keypoints", [])).reshape(
            -1,
            schema.keypoint_count,
            3,
        )
        boxes = _tensor(targets.get("boxes_cxcywh", [])).reshape(-1, 4)
        visible = _tensor(
            targets.get("visibility", []),
            dtype=torch.bool,
        ).reshape(-1, schema.keypoint_count)
        keep = visible.any(dim=1)
        expected = expected[keep]
        boxes = boxes[keep]
        visible = visible[keep]
        similarities = _pose_similarities(
            predicted[..., :2].to(torch.float64),
            expected[..., :2].to(torch.float64),
            boxes.to(torch.float64),
            visible,
            k,
        )
        matches = _match_predictions(similarities, scores)
        order = torch.argsort(scores, descending=True, stable=True)
        scores = scores[order]
        target_counts[0][image_index] = expected.shape[0]
        detections[0].extend(
            (float(score), image_index, matched)
            for score, matched in zip(
                scores.tolist(),
                matches,
                strict=True,
            )
        )
    return _finalize_ap_builders(len(records), target_counts, detections)


@dataclass(frozen=True)
class _FieldLocalizationData:
    image_count: int
    target_counts: np.ndarray
    prediction_counts: np.ndarray
    error_values: np.ndarray
    error_image_indices: np.ndarray

    def compute(self, weights: np.ndarray) -> dict[str, float]:
        target_count = int(np.dot(weights, self.target_counts))
        prediction_count = int(np.dot(weights, self.prediction_counts))
        error_weights = weights[self.error_image_indices]
        matched_count = int(error_weights.sum())
        if target_count == 0:
            matched_fraction = math.nan
            pck_5px = math.nan
        else:
            matched_fraction = matched_count / target_count
            pck_5px = float(
                error_weights[self.error_values <= 5.0].sum() / target_count
            )
        if matched_count == 0:
            median_error = math.nan
        else:
            order = np.argsort(self.error_values, kind="stable")
            cumulative = np.cumsum(error_weights[order])
            rank = (matched_count + 1) // 2
            median_index = int(np.searchsorted(cumulative, rank, side="left"))
            median_error = float(self.error_values[order[median_index]])
        return {
            "field/localization/target_count": float(target_count),
            "field/localization/prediction_count": float(prediction_count),
            "field/localization/matched_fraction": matched_fraction,
            "field/localization/pck_5px": pck_5px,
            "field/localization/median_error_px": median_error,
        }


def _prepare_field_localization(
    records: Sequence[Mapping[str, object]],
) -> _FieldLocalizationData:
    target_counts = np.zeros(len(records), dtype=np.int64)
    prediction_counts = np.zeros(len(records), dtype=np.int64)
    errors = []
    error_images = []
    for image_index, record in enumerate(records):
        predictions = _mapping(record.get("predictions"), "predictions")
        targets = _mapping(record.get("targets"), "targets")
        predicted = _tensor(predictions.get("points", [])).reshape(-1, 4)
        points = _tensor(targets.get("points", [])).reshape(-1, 2)
        labels = _tensor(targets.get("labels", []), dtype=torch.long)
        predicted = predicted[
            predicted[:, 2].isfinite() & (predicted[:, 2] >= 0.25)
        ]
        target_counts[image_index] = points.shape[0]
        prediction_counts[image_index] = predicted.shape[0]
        for class_id in set(labels.tolist()):
            class_predictions = predicted[predicted[:, 3] == class_id, :2]
            class_targets = points[labels == class_id]
            if class_predictions.shape[0] == 0 or class_targets.shape[0] == 0:
                continue
            distances = torch.cdist(
                class_predictions.to(torch.float64),
                class_targets.to(torch.float64),
            )
            predicted_indices, target_indices = linear_sum_assignment(
                distances.numpy()
            )
            for predicted_index, target_index in zip(
                predicted_indices,
                target_indices,
                strict=True,
            ):
                errors.append(float(distances[predicted_index, target_index]))
                error_images.append(image_index)
    return _FieldLocalizationData(
        image_count=len(records),
        target_counts=target_counts,
        prediction_counts=prediction_counts,
        error_values=np.asarray(errors, dtype=np.float64),
        error_image_indices=np.asarray(error_images, dtype=np.int64),
    )


@dataclass(frozen=True)
class _NegativeObjectData:
    negative: np.ndarray
    false_positives: np.ndarray

    def compute(self, weights: np.ndarray) -> float:
        negative_images = int(np.dot(weights, self.negative))
        if negative_images == 0:
            return math.nan
        return float(np.dot(weights, self.false_positives) / negative_images)


def _prepare_negative_objects(
    records: Sequence[Mapping[str, object]],
) -> _NegativeObjectData:
    negative = np.zeros(len(records), dtype=np.int64)
    false_positives = np.zeros(len(records), dtype=np.int64)
    for image_index, record in enumerate(records):
        predictions = _mapping(record.get("predictions"), "predictions")
        targets = _mapping(record.get("targets"), "targets")
        target_boxes = _tensor(targets.get("boxes_xyxy", [])).reshape(-1, 4)
        if target_boxes.shape[0] != 0:
            continue
        negative[image_index] = 1
        scores = _tensor(predictions.get("scores", []))
        false_positives[image_index] = int((scores >= 0.25).sum())
    return _NegativeObjectData(negative, false_positives)


@dataclass(frozen=True)
class _ScoreDiagnosticData:
    values: Mapping[str, np.ndarray]

    def compute(self, weights: np.ndarray) -> dict[str, float]:
        images = int(weights.sum())
        if images == 0:
            return dict.fromkeys(self.values, math.nan)
        return {
            name: float(
                np.dot(weights, values)
                if name.endswith("/images")
                else np.dot(weights, values) / images
            )
            for name, values in self.values.items()
        }


def _cross_pose_alphas(config: Mapping[str, object]) -> tuple[float, ...]:
    raw = config.get("cross_pose_visibility_alphas", (0.0, 1.0))
    if not isinstance(raw, (list, tuple)) or not raw:
        raise TypeError("cross_pose_visibility_alphas must be a non-empty list")
    values = []
    for value in raw:
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            or float(value) < 0
        ):
            raise ValueError(
                "cross_pose_visibility_alphas must be finite and >= 0"
            )
        values.append(float(value))
    return tuple(dict.fromkeys(values))


def _prepare_cross_pose_scores(  # noqa: C901
    records: Mapping[str, Mapping[str, object]],
    alphas: tuple[float, ...],
) -> tuple[dict[str, _ScoreDiagnosticData], dict[str, int]]:
    grouped = _group_records(records)
    unknown = grouped.keys() - set(_CROSS_POSE_TASKS)
    if unknown:
        raise ValueError(
            "Unknown cross-pose diagnostic tasks: " + ", ".join(sorted(unknown))
        )
    prepared = {}
    sizes = {}
    for task, task_records in grouped.items():
        count = len(task_records)
        sizes[task] = count
        values: dict[str, np.ndarray] = {
            f"{task}/images": np.ones(count, dtype=np.float64)
        }
        for alpha in alphas:
            prefix = f"{task}/visibility_alpha/{alpha:g}"
            values[f"{prefix}/max_score_mean"] = np.zeros(
                count,
                dtype=np.float64,
            )
            for name, _ in _SCORE_THRESHOLDS:
                values[f"{prefix}/detections_at_{name}_per_image"] = np.zeros(
                    count, dtype=np.float64
                )
        for image_index, record in enumerate(task_records):
            predictions = _mapping(record.get("predictions"), "predictions")
            targets = _mapping(record.get("targets"), "targets")
            target_scores = np.asarray(
                targets.get("scores", []),
                dtype=np.float64,
            )
            if target_scores.size != 0:
                raise ValueError(
                    "Cross-pose diagnostics require an empty target"
                )
            base_scores = np.asarray(
                predictions.get("base_scores", []),
                dtype=np.float64,
            )
            visibility = np.asarray(
                predictions.get("mean_visibility", []),
                dtype=np.float64,
            )
            if base_scores.ndim != 1 or visibility.ndim != 1:
                raise ValueError(
                    "Cross-pose score arrays must be one-dimensional"
                )
            if base_scores.shape != visibility.shape:
                raise ValueError(
                    "Cross-pose base scores and visibility lengths differ"
                )
            if (
                not np.isfinite(base_scores).all()
                or not np.isfinite(visibility).all()
            ):
                raise ValueError("Cross-pose scores must be finite")
            if ((visibility < 0) | (visibility > 1)).any():
                raise ValueError("Cross-pose visibility must be within [0, 1]")
            for alpha in alphas:
                prefix = f"{task}/visibility_alpha/{alpha:g}"
                scores = base_scores * np.power(visibility, alpha)
                values[f"{prefix}/max_score_mean"][image_index] = (
                    float(scores.max()) if scores.size else 0.0
                )
                for name, threshold in _SCORE_THRESHOLDS:
                    values[f"{prefix}/detections_at_{name}_per_image"][
                        image_index
                    ] = float((scores >= threshold).sum())
        prepared[task] = _ScoreDiagnosticData(values)
    return prepared, sizes


def _compute_cross_pose_scores(
    prepared: Mapping[str, _ScoreDiagnosticData],
    weights_by_task: Mapping[str, np.ndarray],
) -> dict[str, float]:
    values = {}
    for task, metric in prepared.items():
        values.update(metric.compute(weights_by_task[task]))
    return values


@dataclass(frozen=True)
class _PreparedRun:
    object_ap: _ObjectCocoAPData | None = None
    negative_objects: _NegativeObjectData | None = None
    person_ap: _APData | None = None
    robot_ap: _APData | None = None
    field_ap: _APData | None = None
    strict_field_ap: _APData | None = None
    field_localization: _FieldLocalizationData | None = None

    def compute(
        self,
        weights_by_task: Mapping[str, np.ndarray],
    ) -> dict[str, float]:
        values = {}
        if self.object_ap is not None:
            weights = weights_by_task["object"]
            result = self.object_ap.compute(weights)
            values["object/map"] = result["map"]
            values["object/map50"] = result["map50"]
            values["object/map75"] = result["map75"]
            if self.negative_objects is None:
                raise RuntimeError("Object negatives were not prepared")
            values["object/negative_fp_per_image_at_25"] = (
                self.negative_objects.compute(weights)
            )
        for prefix, task, metric in (
            ("person", "person_pose", self.person_ap),
            ("robot", "robot_pose", self.robot_ap),
        ):
            if metric is not None:
                result = metric.compute(weights_by_task[task])
                values[f"{prefix}/map"] = result["map"]
                values[f"{prefix}/map50"] = result["map50"]
                values[f"{prefix}/map75"] = result["map75"]
        if self.field_ap is not None:
            weights = weights_by_task["field_features"]
            result = self.field_ap.compute(weights)
            strict = cast("_APData", self.strict_field_ap).compute(weights)
            values.update(
                {
                    "field/map": result["map"],
                    "field/map50": result["map50"],
                    "field/map75": result["map75"],
                    "field/strict_map": strict["map"],
                    "field/strict_map50": strict["map50"],
                }
            )
            localization = cast(
                "_FieldLocalizationData",
                self.field_localization,
            )
            values.update(localization.compute(weights))
        return values


def _group_records(
    records: Mapping[str, Mapping[str, object]],
) -> dict[str, list[Mapping[str, object]]]:
    grouped: defaultdict[str, list[Mapping[str, object]]] = defaultdict(list)
    for key in sorted(records):
        record = records[key]
        task = record.get("task")
        if not isinstance(task, str):
            raise TypeError(f"Prediction record has no task: {key}")
        grouped[task].append(record)
    return dict(grouped)


@dataclass(frozen=True)
class _BootstrapStratum:
    tasks: tuple[str, ...]
    indices_by_task: tuple[np.ndarray, ...]

    @property
    def size(self) -> int:
        return int(self.indices_by_task[0].size)


@dataclass(frozen=True)
class _ClusterBootstrapPlan:
    task_sizes: dict[str, int]
    strata: tuple[_BootstrapStratum, ...]
    legacy_identity_records: int

    def sample(
        self,
        generator: np.random.Generator,
    ) -> dict[str, np.ndarray]:
        """Draw fixed-size membership strata with shared cluster weights."""
        weights = {
            task: np.zeros(count, dtype=np.int64)
            for task, count in self.task_sizes.items()
        }
        for stratum in self.strata:
            counts = np.bincount(
                generator.integers(0, stratum.size, size=stratum.size),
                minlength=stratum.size,
            )
            for task, indices in zip(
                stratum.tasks,
                stratum.indices_by_task,
                strict=True,
            ):
                weights[task][indices] = counts
        for task, expected in self.task_sizes.items():
            actual = int(weights[task].sum())
            if actual != expected:
                raise RuntimeError(
                    f"Bootstrap task count changed for {task}: "
                    f"{actual} != {expected}"
                )
        return weights


def _prepare_cluster_bootstrap(
    records: Mapping[str, Mapping[str, object]],
) -> _ClusterBootstrapPlan:
    """Partition canonical images by their exact task-membership signature."""
    task_sizes: defaultdict[str, int] = defaultdict(int)
    clusters: defaultdict[
        tuple[str, str],
        dict[str, int],
    ] = defaultdict(dict)
    legacy_identity_records = 0
    for key in sorted(records):
        record = records[key]
        task = record.get("task")
        if not isinstance(task, str):
            raise TypeError(f"Prediction record has no task: {key}")
        image_index = task_sizes[task]
        task_sizes[task] += 1
        identity, used_legacy_identity = _record_cluster_identity(key, record)
        legacy_identity_records += int(used_legacy_identity)
        if task in clusters[identity]:
            raise ValueError(
                "Duplicate canonical image within task: "
                f"task={task}, identity={identity[1]}"
            )
        clusters[identity][task] = image_index

    grouped: defaultdict[
        tuple[str, ...],
        list[dict[str, int]],
    ] = defaultdict(list)
    for members in clusters.values():
        grouped[tuple(sorted(members))].append(members)

    strata = []
    for tasks in sorted(grouped):
        member_rows = grouped[tasks]
        indices_by_task = tuple(
            np.asarray(
                [members[task] for members in member_rows],
                dtype=np.int64,
            )
            for task in tasks
        )
        strata.append(
            _BootstrapStratum(
                tasks=tasks,
                indices_by_task=indices_by_task,
            )
        )
    return _ClusterBootstrapPlan(
        task_sizes=dict(task_sizes),
        strata=tuple(strata),
        legacy_identity_records=legacy_identity_records,
    )


def _prepare_run(
    records: Mapping[str, Mapping[str, object]],
    *,
    field_normalization: float,
) -> tuple[_PreparedRun, dict[str, int]]:
    grouped = _group_records(records)
    task_sizes = {task: len(values) for task, values in grouped.items()}
    object_records = grouped.get("object")
    person_records = grouped.get("person_pose")
    robot_records = grouped.get("robot_pose")
    field_records = grouped.get("field_features")
    return (
        _PreparedRun(
            object_ap=(
                _prepare_object_ap(object_records) if object_records else None
            ),
            negative_objects=(
                _prepare_negative_objects(object_records)
                if object_records
                else None
            ),
            person_ap=(
                _prepare_pose_ap(person_records, prefix="person")
                if person_records
                else None
            ),
            robot_ap=(
                _prepare_pose_ap(robot_records, prefix="robot")
                if robot_records
                else None
            ),
            field_ap=(
                _prepare_field_ap(field_records, field_normalization)
                if field_records
                else None
            ),
            strict_field_ap=(
                _prepare_field_ap(field_records, 0.1) if field_records else None
            ),
            field_localization=(
                _prepare_field_localization(field_records)
                if field_records
                else None
            ),
        ),
        task_sizes,
    )


def _canonical_object_metrics(
    records: Sequence[Mapping[str, object]],
) -> dict[str, float]:
    metric = _build_object_metric(records)
    result = metric.compute()
    return {
        "object/map": float(result["map"]),
        "object/map50": float(result["map_50"]),
        "object/map75": float(result["map_75"]),
    }


def _canonical_pose_metrics(
    records: Sequence[Mapping[str, object]],
    *,
    prefix: str,
) -> dict[str, float]:
    schema = PERSON_POSE_SCHEMA if prefix == "person" else ROBOT_POSE_SCHEMA
    metric = KeypointOKSAccumulator.from_schema(schema, box_format="xywh")
    for record in records:
        predictions = _mapping(record.get("predictions"), "predictions")
        targets = _mapping(record.get("targets"), "targets")
        metric.update(
            _tensor(predictions.get("keypoints", [])).reshape(
                -1,
                schema.keypoint_count,
                3,
            ),
            _tensor(targets.get("keypoints", [])).reshape(
                -1,
                schema.keypoint_count,
                3,
            ),
            _tensor(targets.get("boxes_cxcywh", [])).reshape(-1, 4),
            predicted_scores=_tensor(predictions.get("scores", [])),
            visibility=_tensor(
                targets.get("visibility", []),
                dtype=torch.bool,
            ).reshape(-1, schema.keypoint_count),
        )
    result = metric.compute()
    return {
        f"{prefix}/map": result["ap"],
        f"{prefix}/map50": result["ap50"],
        f"{prefix}/map75": result["ap75"],
    }


def _canonical_field_metrics(
    records: Sequence[Mapping[str, object]],
    *,
    normalization: float,
) -> dict[str, float]:
    metric = FieldPointAPAccumulator(normalization, box_format="xywh")
    strict = FieldPointAPAccumulator(0.1, box_format="xywh")
    localization = FieldPointLocalizationAccumulator()
    for record in records:
        predictions = _mapping(record.get("predictions"), "predictions")
        targets = _mapping(record.get("targets"), "targets")
        predicted = _tensor(predictions.get("points", [])).reshape(-1, 4)
        points = _tensor(targets.get("points", [])).reshape(-1, 2)
        labels = _tensor(targets.get("labels", []), dtype=torch.long)
        boxes = _tensor(targets.get("boxes_cxcywh", [])).reshape(-1, 4)
        metric.update(predicted, points, labels, boxes)
        strict.update(predicted, points, labels, boxes)
        localization.update(predicted, points, labels)
    result = metric.compute()
    strict_result = strict.compute()
    values = {
        "field/map": result["ap"],
        "field/map50": result["ap50"],
        "field/map75": result["ap75"],
        "field/strict_map": strict_result["ap"],
        "field/strict_map50": strict_result["ap50"],
    }
    values.update(
        {
            f"field/localization/{name}": value
            for name, value in localization.compute().items()
        }
    )
    return values


def _canonical_metrics(
    records: Mapping[str, Mapping[str, object]],
    *,
    field_normalization: float,
) -> dict[str, float]:
    grouped = _group_records(records)
    values = {}
    if object_records := grouped.get("object"):
        values.update(_canonical_object_metrics(object_records))
        values["object/negative_fp_per_image_at_25"] = (
            _prepare_negative_objects(object_records).compute(
                np.ones(len(object_records), dtype=np.int64)
            )
        )
    if person_records := grouped.get("person_pose"):
        values.update(_canonical_pose_metrics(person_records, prefix="person"))
    if robot_records := grouped.get("robot_pose"):
        values.update(_canonical_pose_metrics(robot_records, prefix="robot"))
    if field_records := grouped.get("field_features"):
        values.update(
            _canonical_field_metrics(
                field_records,
                normalization=field_normalization,
            )
        )
    return values


def _assert_close_metrics(
    expected: Mapping[str, object],
    actual: Mapping[str, float],
    *,
    context: str,
) -> None:
    for name, actual_value in actual.items():
        expected_value = expected.get(name)
        if not isinstance(expected_value, (int, float)):
            continue
        expected_float = float(expected_value)
        if math.isnan(expected_float) and math.isnan(actual_value):
            continue
        if not math.isclose(
            expected_float,
            actual_value,
            rel_tol=0.0,
            abs_tol=_SAVED_METRIC_TOLERANCE,
        ):
            raise ValueError(
                f"{context} metric {name!r} does not match raw records: "
                f"saved={expected_float}, recomputed={actual_value}"
            )


def _interval(values: np.ndarray) -> tuple[float, float, float]:
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return math.nan, math.nan, math.nan
    lower, upper = np.quantile(finite, [0.025, 0.975])
    return float(finite.mean()), float(lower), float(upper)


def _safe_float(value: object) -> float:
    return float(value) if isinstance(value, (int, float)) else math.nan


def _promotion_gates(
    rows: Mapping[str, Mapping[str, object]],
    primary_score: Mapping[str, float],
) -> dict[str, bool]:
    gates = {}
    for name, margin in (
        ("object/map", -0.005),
        ("field/map", -0.005),
        ("field/strict_map", -0.005),
        ("field/localization/pck_5px", -0.01),
        ("field/localization/matched_fraction", -0.01),
    ):
        lower = _safe_float(rows.get(name, {}).get("ci95_lower"))
        gates[f"{name}/noninferior"] = math.isfinite(lower) and lower >= margin

    median_upper = _safe_float(
        rows.get("field/localization/median_error_px", {}).get("ci95_upper")
    )
    gates["field/localization/median_error_px/noninferior"] = (
        math.isfinite(median_upper) and median_upper <= 0.5
    )

    for name in ("person/map", "robot/map"):
        delta = _safe_float(rows.get(name, {}).get("delta"))
        gates[f"{name}/noninferior"] = math.isfinite(delta) and delta >= -0.01

    negative = rows.get("object/negative_fp_per_image_at_25", {})
    negative_baseline = _safe_float(negative.get("baseline"))
    negative_delta = _safe_float(negative.get("delta"))
    allowed_increase = (
        max(negative_baseline * 0.10, 0.05)
        if math.isfinite(negative_baseline)
        else math.nan
    )
    gates["object/negative_fp_per_image_at_25/noninferior"] = (
        math.isfinite(negative_delta)
        and math.isfinite(allowed_increase)
        and negative_delta <= allowed_increase
    )

    score_delta = _safe_float(primary_score.get("delta"))
    score_lower = _safe_float(primary_score.get("ci95_lower"))
    gates["primary_score/minimum_gain"] = (
        math.isfinite(score_delta) and score_delta >= 0.003
    )
    gates["primary_score/positive_lcb"] = (
        math.isfinite(score_lower) and score_lower > 0.0
    )
    primary_ap_lowers = [
        _safe_float(rows.get(name, {}).get("ci95_lower"))
        for name in ("object/map", "field/map", "field/strict_map")
    ]
    gates["primary_ap/one_positive_lcb"] = any(
        math.isfinite(value) and value > 0.0 for value in primary_ap_lowers
    )
    return gates


def compare(  # noqa: C901
    baseline_dir: Path,
    candidate_dir: Path,
    *,
    iterations: int,
    seed: int,
) -> dict[str, object]:
    """Recompute metrics for genuine paired image-cluster resamples."""
    baseline_saved = _load_object(baseline_dir / "metrics.json")
    candidate_saved = _load_object(candidate_dir / "metrics.json")
    baseline_config = _load_object(baseline_dir / "config.json")
    candidate_config = _load_object(candidate_dir / "config.json")
    baseline_metadata = _load_object(baseline_dir / "metadata.json")
    candidate_metadata = _load_object(candidate_dir / "metadata.json")
    baseline_cross_path = baseline_dir / _CROSS_POSE_RECORDS
    candidate_cross_path = candidate_dir / _CROSS_POSE_RECORDS
    baseline_has_cross = baseline_cross_path.is_file()
    candidate_has_cross = candidate_cross_path.is_file()
    if baseline_has_cross != candidate_has_cross:
        raise ValueError(
            "Cross-pose diagnostic records differ: exactly one run has "
            f"{_CROSS_POSE_RECORDS}"
        )
    compatibility = compatibility_report(
        baseline_config,
        candidate_config,
        baseline_metadata,
        candidate_metadata,
    )
    baseline_records = _load_records(baseline_dir / "predictions.jsonl.gz")
    candidate_records = _load_records(candidate_dir / "predictions.jsonl.gz")
    if baseline_records.keys() != candidate_records.keys():
        missing = baseline_records.keys() - candidate_records.keys()
        extra = candidate_records.keys() - baseline_records.keys()
        raise ValueError(
            "Prediction pairs differ: "
            f"missing={len(missing)}, extra={len(extra)}"
        )
    for key in baseline_records:
        if baseline_records[key].get("task") != candidate_records[key].get(
            "task"
        ):
            raise ValueError(f"Prediction pair task differs: {key}")
        baseline_identity, _ = _record_cluster_identity(
            key,
            baseline_records[key],
        )
        candidate_identity, _ = _record_cluster_identity(
            key,
            candidate_records[key],
        )
        if baseline_identity != candidate_identity:
            raise ValueError(f"Prediction pair canonical image differs: {key}")

    baseline_cross_records: dict[str, dict[str, object]] = {}
    candidate_cross_records: dict[str, dict[str, object]] = {}
    if baseline_has_cross:
        baseline_cross_records = _load_records(baseline_cross_path)
        candidate_cross_records = _load_records(candidate_cross_path)
        if baseline_cross_records.keys() != candidate_cross_records.keys():
            missing = (
                baseline_cross_records.keys() - candidate_cross_records.keys()
            )
            extra = (
                candidate_cross_records.keys() - baseline_cross_records.keys()
            )
            raise ValueError(
                "Cross-pose prediction pairs differ: "
                f"missing={len(missing)}, extra={len(extra)}"
            )
        for key in baseline_cross_records:
            if baseline_cross_records[key].get("task") != (
                candidate_cross_records[key].get("task")
            ):
                raise ValueError(f"Cross-pose prediction task differs: {key}")

    raw_normalization = baseline_config.get("field_normalization", 1.0)
    if not isinstance(raw_normalization, (int, float)):
        raise TypeError("field_normalization must be numeric")
    field_normalization = float(raw_normalization)
    if not math.isfinite(field_normalization) or field_normalization <= 0:
        raise ValueError("field_normalization must be finite and positive")
    raw_confidence = baseline_config.get("confidence", 0.001)
    if not isinstance(raw_confidence, (int, float)):
        raise TypeError("confidence must be numeric")
    confidence = float(raw_confidence)
    if not math.isfinite(confidence) or confidence < 0:
        raise ValueError("confidence must be finite and non-negative")
    if confidence > 0.25:
        raise ValueError(
            "Promotion comparison requires confidence <= 0.25 so negative-"
            "image false positives at 0.25 are present in prediction records"
        )
    baseline_prepared, task_sizes = _prepare_run(
        baseline_records,
        field_normalization=field_normalization,
    )
    candidate_prepared, candidate_task_sizes = _prepare_run(
        candidate_records,
        field_normalization=field_normalization,
    )
    if task_sizes != candidate_task_sizes:
        raise ValueError("Paired task image counts differ")
    bootstrap_plan = _prepare_cluster_bootstrap(baseline_records)
    if bootstrap_plan.task_sizes != task_sizes:
        raise RuntimeError("Bootstrap task ordering differs from prepared run")

    cross_pose_alphas: tuple[float, ...] = ()
    baseline_cross_prepared: dict[str, _ScoreDiagnosticData] = {}
    candidate_cross_prepared: dict[str, _ScoreDiagnosticData] = {}
    cross_pose_task_sizes: dict[str, int] = {}
    if baseline_cross_records:
        cross_pose_alphas = _cross_pose_alphas(baseline_config)
        baseline_cross_prepared, cross_pose_task_sizes = (
            _prepare_cross_pose_scores(
                baseline_cross_records,
                cross_pose_alphas,
            )
        )
        candidate_cross_prepared, candidate_cross_sizes = (
            _prepare_cross_pose_scores(
                candidate_cross_records,
                cross_pose_alphas,
            )
        )
        if cross_pose_task_sizes != candidate_cross_sizes:
            raise ValueError("Paired cross-pose task image counts differ")

    baseline_metrics = _canonical_metrics(
        baseline_records,
        field_normalization=field_normalization,
    )
    candidate_metrics = _canonical_metrics(
        candidate_records,
        field_normalization=field_normalization,
    )
    canonical_metric_names = set(baseline_metrics)
    cross_pose_unit_weights = {
        task: np.ones(count, dtype=np.int64)
        for task, count in cross_pose_task_sizes.items()
    }
    if baseline_cross_prepared:
        baseline_metrics.update(
            _compute_cross_pose_scores(
                baseline_cross_prepared,
                cross_pose_unit_weights,
            )
        )
        candidate_metrics.update(
            _compute_cross_pose_scores(
                candidate_cross_prepared,
                cross_pose_unit_weights,
            )
        )
    _assert_close_metrics(
        baseline_saved,
        baseline_metrics,
        context="Baseline",
    )
    _assert_close_metrics(
        candidate_saved,
        candidate_metrics,
        context="Candidate",
    )

    unit_weights = {
        task: np.ones(count, dtype=np.int64)
        for task, count in task_sizes.items()
    }
    _assert_close_metrics(
        baseline_metrics,
        baseline_prepared.compute(unit_weights),
        context="Baseline bootstrap sufficient statistics",
    )
    _assert_close_metrics(
        candidate_metrics,
        candidate_prepared.compute(unit_weights),
        context="Candidate bootstrap sufficient statistics",
    )

    distributions = {
        name: np.full(iterations, math.nan, dtype=np.float64)
        for name in BOOTSTRAP_METRICS
    }
    score_distribution = np.full(iterations, math.nan, dtype=np.float64)
    generator = np.random.default_rng(seed)
    for iteration in range(iterations):
        weights = bootstrap_plan.sample(generator)
        baseline_sample = baseline_prepared.compute(weights)
        candidate_sample = candidate_prepared.compute(weights)
        for name in BOOTSTRAP_METRICS:
            baseline_value = baseline_sample.get(name, math.nan)
            candidate_value = candidate_sample.get(name, math.nan)
            distributions[name][iteration] = candidate_value - baseline_value
        if all(name in baseline_sample for name in PRIMARY_WEIGHTS) and all(
            name in candidate_sample for name in PRIMARY_WEIGHTS
        ):
            score_distribution[iteration] = sum(
                (candidate_sample[name] - baseline_sample[name]) * weight
                for name, weight in PRIMARY_WEIGHTS.items()
            )

    if baseline_cross_prepared:
        cross_metric_names = sorted(
            baseline_metrics.keys() - canonical_metric_names
        )
        distributions.update(
            {
                name: np.full(iterations, math.nan, dtype=np.float64)
                for name in cross_metric_names
            }
        )
        cross_generator = np.random.default_rng(seed ^ 0x43524F53)
        for iteration in range(iterations):
            cross_weights = {
                task: np.bincount(
                    cross_generator.integers(0, count, size=count),
                    minlength=count,
                )
                for task, count in cross_pose_task_sizes.items()
            }
            baseline_sample = _compute_cross_pose_scores(
                baseline_cross_prepared,
                cross_weights,
            )
            candidate_sample = _compute_cross_pose_scores(
                candidate_cross_prepared,
                cross_weights,
            )
            for name in cross_metric_names:
                distributions[name][iteration] = (
                    candidate_sample[name] - baseline_sample[name]
                )

    rows = []
    shared_metrics = sorted(baseline_metrics.keys() & candidate_metrics.keys())
    for name in shared_metrics:
        baseline_value = baseline_metrics[name]
        candidate_value = candidate_metrics[name]
        bootstrap_mean, lower, upper = _interval(
            distributions.get(
                name,
                np.asarray([], dtype=np.float64),
            )
        )
        paired_images = task_sizes.get(METRIC_TASK.get(name, ""), 0)
        if name.startswith("cross_pose/"):
            paired_images = next(
                (
                    count
                    for task, count in cross_pose_task_sizes.items()
                    if name == f"{task}/images" or name.startswith(f"{task}/")
                ),
                0,
            )
        rows.append(
            {
                "metric": name,
                "baseline": baseline_value,
                "candidate": candidate_value,
                "delta": candidate_value - baseline_value,
                "bootstrap_delta_mean": bootstrap_mean,
                "ci95_lower": lower,
                "ci95_upper": upper,
                "paired_images": paired_images,
            }
        )
    row_by_name = {cast("str", row["metric"]): row for row in rows}

    baseline_score = (
        sum(
            baseline_metrics[name] * weight
            for name, weight in PRIMARY_WEIGHTS.items()
        )
        if all(name in baseline_metrics for name in PRIMARY_WEIGHTS)
        else math.nan
    )
    candidate_score = (
        sum(
            candidate_metrics[name] * weight
            for name, weight in PRIMARY_WEIGHTS.items()
        )
        if all(name in candidate_metrics for name in PRIMARY_WEIGHTS)
        else math.nan
    )
    score_mean, score_lower, score_upper = _interval(score_distribution)
    primary_score = {
        "baseline": baseline_score,
        "candidate": candidate_score,
        "delta": candidate_score - baseline_score,
        "bootstrap_delta_mean": score_mean,
        "ci95_lower": score_lower,
        "ci95_upper": score_upper,
    }
    gates = _promotion_gates(row_by_name, primary_score)
    passes = all(gates.values())
    bootstrap: dict[str, object] = {
        "iterations": iterations,
        "seed": seed,
        "unit": (
            "paired canonical image cluster within fixed task-membership "
            "stratum"
        ),
        "method": (
            "paired nonparametric canonical-image cluster bootstrap, "
            "stratified by task-membership signature, with exact "
            "case-weighted global AP recomputation and fixed stratum counts"
        ),
        "interval": "percentile",
        "task_image_counts": task_sizes,
        "canonical_cluster_count": sum(
            stratum.size for stratum in bootstrap_plan.strata
        ),
        "cross_task_cluster_count": sum(
            stratum.size
            for stratum in bootstrap_plan.strata
            if len(stratum.tasks) > 1
        ),
        "task_membership_strata": [
            {
                "tasks": list(stratum.tasks),
                "clusters": stratum.size,
            }
            for stratum in bootstrap_plan.strata
        ],
        "legacy_identity_records": bootstrap_plan.legacy_identity_records,
    }
    if cross_pose_task_sizes:
        bootstrap["cross_pose_image_counts"] = cross_pose_task_sizes
        bootstrap["cross_pose_visibility_alphas"] = list(cross_pose_alphas)
    return {
        "version": 3,
        "compatibility": compatibility,
        "bootstrap": bootstrap,
        "primary_score": primary_score,
        "metrics": rows,
        "gates": gates,
        "passes_all_gates": passes,
        "passes_all_available_gates": passes,
    }


@click.command()
@click.argument(
    "baseline_dir",
    type=click.Path(exists=True, file_okay=False, path_type=Path),
)
@click.argument(
    "candidate_dir",
    type=click.Path(exists=True, file_okay=False, path_type=Path),
)
@click.option("--output", type=click.Path(dir_okay=False, path_type=Path))
@click.option(
    "--iterations",
    type=click.IntRange(min=100),
    default=200,
    show_default=True,
)
@click.option("--seed", type=int, default=20260716, show_default=True)
def main(
    baseline_dir: Path,
    candidate_dir: Path,
    output: Path | None,
    iterations: int,
    seed: int,
) -> None:
    """Compare two compatible standalone multi-task validation runs."""
    report = compare(
        baseline_dir,
        candidate_dir,
        iterations=iterations,
        seed=seed,
    )
    destination = output or candidate_dir / "comparison.json"
    write_json(destination, report)
    score = cast("dict[str, float]", report["primary_score"])
    click.echo(f"Primary score delta: {score['delta']:+.6f}")
    click.echo(f"Passes all gates: {report['passes_all_gates']}")
    click.echo(f"Comparison report: {destination.resolve()}")


if __name__ == "__main__":
    main()
