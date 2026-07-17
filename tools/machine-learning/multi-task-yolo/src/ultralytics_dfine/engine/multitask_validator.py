"""Independent validation for every multi-task D-FINE output."""

# ruff: noqa: C901, TRY003

import math
from collections.abc import Callable, Mapping
from pathlib import Path

import torch
from torch import Tensor
from torch.utils.data import DataLoader
from torchmetrics.detection.mean_ap import MeanAveragePrecision

from ultralytics_dfine.engine.metrics import (
    FieldPointAPAccumulator,
    FieldPointLocalizationAccumulator,
    KeypointOKSAccumulator,
)
from ultralytics_dfine.engine.multitask_render import render_all_outputs
from ultralytics_dfine.nn import DFINEMultiTaskModel
from ultralytics_dfine.schemas import (
    FIELD_FEATURE_SCHEMA,
    PERSON_POSE_SCHEMA,
    ROBOT_POSE_SCHEMA,
    HeadId,
)

CROSS_POSE_PERSON_ON_ROBOT = "cross_pose/person_on_robot"
CROSS_POSE_ROBOT_ON_PERSON = "cross_pose/robot_on_person"
_SCORE_THRESHOLDS = (
    ("001", 0.001),
    ("05", 0.05),
    ("10", 0.1),
    ("25", 0.25),
    ("50", 0.5),
    ("75", 0.75),
)


def _empty_score_diagnostics() -> dict[str, float]:
    return {
        "images": 0.0,
        "max_sum": 0.0,
        **{f"at_{name}": 0.0 for name, _ in _SCORE_THRESHOLDS},
    }


def _target_tensor(target: Mapping[str, object], key: str) -> Tensor:
    value = target.get(key)
    if not isinstance(value, Tensor):
        raise TypeError(f"Validation target is missing tensor '{key}'")
    return value


def _pixel_cxcywh(boxes: Tensor, height: int, width: int) -> Tensor:
    scale = torch.tensor(
        [width, height, width, height],
        dtype=boxes.dtype,
        device=boxes.device,
    )
    return boxes * scale


def _pixel_xyxy(boxes: Tensor, height: int, width: int) -> Tensor:
    boxes = _pixel_cxcywh(boxes, height, width)
    center = boxes[:, :2]
    half_size = boxes[:, 2:] / 2
    return torch.cat((center - half_size, center + half_size), dim=1)


def _pixel_keypoints(keypoints: Tensor, height: int, width: int) -> Tensor:
    result = keypoints.clone()
    result[..., 0] *= width
    result[..., 1] *= height
    return result


class MultiTaskValidator:
    """Evaluate object, person, robot, and field outputs independently."""

    def __init__(
        self,
        model: DFINEMultiTaskModel,
        loaders: Mapping[HeadId, DataLoader],
        *,
        device: str | torch.device,
        confidence: float = 0.001,
        field_normalization: float = 1.0,
        render_object_confidence: float = 0.25,
        render_person_confidence: float = 0.5,
        render_robot_confidence: float = 0.5,
        render_field_confidence: float = 0.35,
        render_keypoint_confidence: float = 0.5,
        render_max_detections: int = 20,
        pose_max_detections: int = 20,
        person_visibility_alphas: tuple[float, ...] = (0.0,),
        robot_visibility_alphas: tuple[float, ...] = (0.0,),
        cross_pose_visibility_alphas: tuple[float, ...] | None = None,
    ) -> None:
        confidence_values = {
            "confidence": confidence,
            "render_object_confidence": render_object_confidence,
            "render_person_confidence": render_person_confidence,
            "render_robot_confidence": render_robot_confidence,
            "render_field_confidence": render_field_confidence,
            "render_keypoint_confidence": render_keypoint_confidence,
        }
        invalid_confidence = [
            name
            for name, value in confidence_values.items()
            if not math.isfinite(value) or not 0 <= value <= 1
        ]
        if invalid_confidence:
            raise ValueError(
                "Confidence values must be finite and in [0, 1]: "
                + ", ".join(invalid_confidence)
            )
        if not math.isfinite(field_normalization) or field_normalization <= 0:
            raise ValueError("Field normalization must be finite and positive")
        self.model = model
        self.loaders = dict(loaders)
        self.device = torch.device(device)
        self.confidence = confidence
        self.field_normalization = field_normalization
        self.render_object_confidence = render_object_confidence
        self.render_person_confidence = render_person_confidence
        self.render_robot_confidence = render_robot_confidence
        self.render_field_confidence = render_field_confidence
        self.render_keypoint_confidence = render_keypoint_confidence
        self.render_max_detections = render_max_detections
        self.pose_max_detections = pose_max_detections
        if not person_visibility_alphas:
            raise ValueError("At least one person visibility alpha is required")
        if any(
            not math.isfinite(alpha) or alpha < 0
            for alpha in person_visibility_alphas
        ):
            raise ValueError("Person visibility alphas must be finite and >= 0")
        self.person_visibility_alphas = tuple(
            dict.fromkeys(person_visibility_alphas)
        )
        if not robot_visibility_alphas:
            raise ValueError("At least one robot visibility alpha is required")
        if any(
            not math.isfinite(alpha) or alpha < 0
            for alpha in robot_visibility_alphas
        ):
            raise ValueError("Robot visibility alphas must be finite and >= 0")
        self.robot_visibility_alphas = tuple(
            dict.fromkeys(robot_visibility_alphas)
        )
        if cross_pose_visibility_alphas is not None and (
            not cross_pose_visibility_alphas
            or any(
                not math.isfinite(alpha) or alpha < 0
                for alpha in cross_pose_visibility_alphas
            )
        ):
            raise ValueError(
                "Cross-pose visibility alphas must be non-empty, finite, "
                "and >= 0"
            )
        self.cross_pose_visibility_alphas = (
            tuple(dict.fromkeys(cross_pose_visibility_alphas))
            if cross_pose_visibility_alphas is not None
            else None
        )

    @torch.no_grad()
    def run(
        self,
        *,
        render_dir: str | Path | None = None,
        max_batches: int | None = None,
        record_sink: Callable[[dict[str, object]], None] | None = None,
        cross_pose_record_sink: (
            Callable[[dict[str, object]], None] | None
        ) = None,
    ) -> dict[str, float | dict[str, str]]:
        self.model.eval()
        object_metric = MeanAveragePrecision(
            box_format="xyxy",
            iou_type="bbox",
            max_detection_thresholds=[1, 10, 300],
            backend="faster_coco_eval",
            sync_on_compute=False,
            class_metrics=True,
        )
        object_metric.warn_on_many_detections = False
        person_metrics = {
            alpha: KeypointOKSAccumulator.from_schema(
                PERSON_POSE_SCHEMA,
                box_format="xywh",
            )
            for alpha in self.person_visibility_alphas
        }
        robot_metrics = {
            alpha: KeypointOKSAccumulator.from_schema(
                ROBOT_POSE_SCHEMA,
                box_format="xywh",
            )
            for alpha in self.robot_visibility_alphas
        }
        field_metric = FieldPointAPAccumulator(
            self.field_normalization,
            box_format="xywh",
        )
        strict_field_metric = FieldPointAPAccumulator(
            0.1,
            box_format="xywh",
        )
        field_localization = FieldPointLocalizationAccumulator()
        rendered: dict[str, str] = {}
        score_diagnostics = {
            prefix: _empty_score_diagnostics()
            for prefix in ("person", "robot", "field", "object")
        }
        cross_pose_diagnostics: dict[
            str,
            dict[float, dict[str, float]],
        ] = {}
        if self.cross_pose_visibility_alphas is not None:
            cross_pose_diagnostics = {
                direction: {
                    alpha: _empty_score_diagnostics()
                    for alpha in self.cross_pose_visibility_alphas
                }
                for direction in (
                    CROSS_POSE_PERSON_ON_ROBOT,
                    CROSS_POSE_ROBOT_ON_PERSON,
                )
            }

        for task, loader in self.loaders.items():
            for batch_index, (images, targets) in enumerate(loader):
                if max_batches is not None and batch_index >= max_batches:
                    break
                images = images.to(self.device, non_blocking=True)
                outputs = (
                    self.model.forward_deploy(images)
                    if render_dir is not None and batch_index == 0
                    else self.model.forward_deploy_task(images, task)
                )
                cross_pose_outputs: Mapping[str, Tensor] | None = None
                if self.cross_pose_visibility_alphas is not None:
                    if task == HeadId.PERSON_POSE and any(
                        target.get("robot_negative_eligible") is True
                        for target in targets
                    ):
                        cross_pose_outputs = self.model.forward_deploy_task(
                            images,
                            HeadId.ROBOT_POSE,
                        )
                    elif task == HeadId.ROBOT_POSE and any(
                        target.get("person_negative_eligible") is True
                        for target in targets
                    ):
                        cross_pose_outputs = self.model.forward_deploy_task(
                            images,
                            HeadId.PERSON_POSE,
                        )
                if render_dir is not None and batch_index == 0:
                    destination = Path(render_dir)
                    if rendered:
                        destination /= str(task)
                    paths = render_all_outputs(
                        images[0],
                        outputs,
                        destination,
                        object_confidence=self.render_object_confidence,
                        person_confidence=self.render_person_confidence,
                        robot_confidence=self.render_robot_confidence,
                        field_confidence=self.render_field_confidence,
                        keypoint_confidence=self.render_keypoint_confidence,
                        max_detections=self.render_max_detections,
                        robot_class_id=self.model.detector.names.index("Robot"),
                        class_names=self.model.detector.names,
                        target=targets[0],
                        target_task=task,
                        valid_object_classes=_target_tensor(
                            targets[0],
                            "valid_detection_classes",
                        ),
                    )
                    rendered.update(
                        {
                            f"{task}/{name}": str(path)
                            for name, path in paths.items()
                        }
                    )
                for image_index, target in enumerate(targets):
                    height, width = images.shape[-2:]
                    cross_pose_eligible = (
                        target.get("robot_negative_eligible") is True
                        if task == HeadId.PERSON_POSE
                        else target.get("person_negative_eligible") is True
                    )
                    if cross_pose_outputs is not None and cross_pose_eligible:
                        direction, base_scores, mean_visibility = (
                            self._cross_pose_scores(
                                task,
                                cross_pose_outputs,
                                image_index,
                            )
                        )
                        for alpha, values in cross_pose_diagnostics[
                            direction
                        ].items():
                            self._update_score_diagnostics(
                                values,
                                base_scores * mean_visibility.pow(alpha),
                            )
                        if cross_pose_record_sink is not None:
                            cross_pose_record_sink(
                                self._cross_pose_prediction_record(
                                    direction,
                                    base_scores,
                                    mean_visibility,
                                    target,
                                    height,
                                    width,
                                )
                            )
                    if task == HeadId.OBJECT:
                        object_rows = outputs["object_output"][image_index]
                        labels = object_rows[:, 5].long()
                        valid = _target_tensor(
                            target,
                            "valid_detection_classes",
                        ).to(labels.device)
                        in_range = (labels >= 0) & (labels < valid.numel())
                        supported = torch.zeros_like(in_range)
                        supported[in_range] = valid[labels[in_range]]
                        self._update_score_diagnostics(
                            score_diagnostics["object"],
                            object_rows[supported, 4],
                        )
                        self._update_objects(
                            object_metric,
                            outputs["object_output"][image_index],
                            target,
                            height,
                            width,
                        )
                    elif task == HeadId.PERSON_POSE:
                        person_output = outputs["person_pose_output"][
                            image_index
                        ]
                        primary_scores = self._person_calibrated_scores(
                            person_output,
                            self.person_visibility_alphas[0],
                        )
                        self._update_score_diagnostics(
                            score_diagnostics["person"],
                            primary_scores,
                        )
                        for alpha, person_metric in person_metrics.items():
                            self._update_person(
                                person_metric,
                                person_output,
                                target,
                                height,
                                width,
                                visibility_alpha=alpha,
                            )
                    elif task == HeadId.ROBOT_POSE:
                        object_rows = outputs["object_output"][image_index]
                        _, primary_scores, _ = self._robot_predictions(
                            object_rows,
                            outputs["robot_pose_output"][image_index],
                            self.robot_visibility_alphas[0],
                        )
                        self._update_score_diagnostics(
                            score_diagnostics["robot"],
                            primary_scores,
                        )
                        for alpha, robot_metric in robot_metrics.items():
                            self._update_robot(
                                robot_metric,
                                outputs["object_output"][image_index],
                                outputs["robot_pose_output"][image_index],
                                target,
                                height,
                                width,
                                visibility_alpha=alpha,
                            )
                    elif task == HeadId.FIELD_FEATURES:
                        self._update_score_diagnostics(
                            score_diagnostics["field"],
                            outputs["field_feature_output"][image_index, :, 2],
                        )
                        self._update_field(
                            field_metric,
                            outputs["field_feature_output"][image_index],
                            target,
                            height,
                            width,
                        )
                        points = _target_tensor(target, "points").clone()
                        points[:, 0] *= width
                        points[:, 1] *= height
                        field_localization.update(
                            self._confident_field_predictions(
                                outputs["field_feature_output"][image_index]
                            ),
                            points,
                            _target_tensor(target, "point_labels"),
                        )
                        self._update_field(
                            strict_field_metric,
                            outputs["field_feature_output"][image_index],
                            target,
                            height,
                            width,
                        )
                    if record_sink is not None:
                        record_sink(
                            self._prediction_record(
                                task,
                                outputs,
                                image_index,
                                target,
                                height,
                                width,
                            )
                        )

        metrics: dict[str, float | dict[str, str]] = {"renders": rendered}
        if HeadId.OBJECT in self.loaders:
            computed = object_metric.compute()
            metrics.update(
                {
                    "object/map": float(computed["map"]),
                    "object/map50": float(computed["map_50"]),
                    "object/map75": float(computed["map_75"]),
                    "object/mar300": float(computed["mar_300"]),
                }
            )
            for class_id, value in zip(
                computed["classes"].long().reshape(-1).tolist(),
                computed["map_per_class"].reshape(-1).tolist(),
                strict=True,
            ):
                class_name = self.model.detector.names[class_id]
                metrics[f"object/class/{class_name}/map"] = float(value)
        if HeadId.PERSON_POSE in self.loaders:
            for index, (alpha, metric) in enumerate(person_metrics.items()):
                computed = metric.compute()
                prefix = f"person/visibility_alpha/{alpha:g}"
                values = {
                    "map": computed["ap"],
                    "map50": computed["ap50"],
                    "map75": computed["ap75"],
                    "mar": computed["ar"],
                }
                metrics.update(
                    {
                        f"{prefix}/{name}": value
                        for name, value in values.items()
                    }
                )
                if index == 0:
                    metrics.update(
                        {
                            f"person/{name}": value
                            for name, value in values.items()
                        }
                    )
        if HeadId.ROBOT_POSE in self.loaders:
            for index, (alpha, metric) in enumerate(robot_metrics.items()):
                computed = metric.compute()
                prefix = f"robot/visibility_alpha/{alpha:g}"
                values = {
                    "map": computed["ap"],
                    "map50": computed["ap50"],
                    "map75": computed["ap75"],
                    "mar": computed["ar"],
                }
                metrics.update(
                    {
                        f"{prefix}/{name}": value
                        for name, value in values.items()
                    }
                )
                if index == 0:
                    metrics.update(
                        {
                            f"robot/{name}": value
                            for name, value in values.items()
                        }
                    )
        if HeadId.FIELD_FEATURES in self.loaders:
            computed = field_metric.compute()
            strict = strict_field_metric.compute()
            metrics.update(
                {
                    "field/map": computed["ap"],
                    "field/map50": computed["ap50"],
                    "field/map75": computed["ap75"],
                    "field/mar": computed["ar"],
                    "field/strict_map": strict["ap"],
                    "field/strict_map50": strict["ap50"],
                }
            )
            for class_id, result in field_metric.compute_per_class().items():
                class_name = FIELD_FEATURE_SCHEMA.class_names[class_id]
                metrics[f"field/class/{class_name}/map"] = result["ap"]
                metrics[f"field/class/{class_name}/map50"] = result["ap50"]
            for (
                class_id,
                result,
            ) in strict_field_metric.compute_per_class().items():
                class_name = FIELD_FEATURE_SCHEMA.class_names[class_id]
                metrics[f"field/class/{class_name}/strict_map"] = result["ap"]
            metrics.update(
                {
                    f"field/localization/{name}": value
                    for name, value in field_localization.compute().items()
                }
            )
        for prefix, values in score_diagnostics.items():
            if values["images"] == 0:
                continue
            metrics.update(
                {
                    f"{prefix}/max_score_mean": values["max_sum"]
                    / values["images"],
                    f"{prefix}/detections_at_001_per_image": values["at_001"]
                    / values["images"],
                    f"{prefix}/detections_at_25_per_image": values["at_25"]
                    / values["images"],
                    f"{prefix}/detections_at_05_per_image": values["at_05"]
                    / values["images"],
                    f"{prefix}/detections_at_10_per_image": values["at_10"]
                    / values["images"],
                    f"{prefix}/detections_at_50_per_image": values["at_50"]
                    / values["images"],
                    f"{prefix}/detections_at_75_per_image": values["at_75"]
                    / values["images"],
                }
            )
        for direction, by_alpha in cross_pose_diagnostics.items():
            first = next(iter(by_alpha.values()))
            if first["images"] == 0:
                continue
            metrics[f"{direction}/images"] = first["images"]
            for alpha, values in by_alpha.items():
                prefix = f"{direction}/visibility_alpha/{alpha:g}"
                metrics.update(self._score_diagnostic_metrics(prefix, values))
        return metrics

    @staticmethod
    def _cross_pose_prediction_record(
        direction: str,
        base_scores: Tensor,
        mean_visibility: Tensor,
        target: Mapping[str, object],
        height: int,
        width: int,
    ) -> dict[str, object]:
        """Record opposite-head scores against an explicitly empty target."""
        raw_path = target.get("path")
        if not isinstance(raw_path, Path):
            raise TypeError("Validation target is missing a Path 'path'")
        image_id = _target_tensor(target, "image_id")
        original_size = _target_tensor(target, "orig_size")
        eligibility = (
            {
                "policy": "explicit-verified-robot-free-keypoint-v2",
                "reviewed": target.get("robot_negative_reviewed") is True,
                "verified": target.get("robot_negative_verified") is True,
                "eligible": target.get("robot_negative_eligible") is True,
                "excluded": target.get("robot_negative_excluded") is True,
            }
            if direction == CROSS_POSE_ROBOT_ON_PERSON
            else {
                "policy": "verified-person-free-dhrp-v1",
                "annotation_file": target.get("annotation_file"),
                "verified": target.get("person_negative_verified") is True,
                "eligible": target.get("person_negative_eligible") is True,
            }
        )
        return {
            "version": 1,
            "task": direction,
            "image_key": f"{direction}:{raw_path.resolve()}",
            "image_id": int(image_id.item()),
            "path": str(raw_path.resolve()),
            "evaluation_size": [height, width],
            "original_size": original_size.detach().cpu().tolist(),
            "predictions": {
                "base_scores": base_scores.detach().cpu().tolist(),
                "mean_visibility": mean_visibility.detach().cpu().tolist(),
            },
            "targets": {"scores": []},
            "negative_target_eligibility": eligibility,
        }

    def _cross_pose_scores(
        self,
        source_task: HeadId,
        outputs: Mapping[str, Tensor],
        image_index: int,
    ) -> tuple[str, Tensor, Tensor]:
        """Return public opposite-head scores without target-dependent logic."""
        if source_task == HeadId.PERSON_POSE:
            poses, scores, visibility = self._robot_candidates(
                outputs["object_output"][image_index],
                outputs["robot_pose_output"][image_index],
            )
            del poses
            direction = CROSS_POSE_ROBOT_ON_PERSON
        elif source_task == HeadId.ROBOT_POSE:
            output = outputs["person_pose_output"][image_index]
            scores = output[:, 4]
            visibility = output[:, 6:].reshape(-1, 17, 3)[..., 2].mean(-1)
            direction = CROSS_POSE_PERSON_ON_ROBOT
        else:
            raise ValueError(
                f"Cross-pose source must be a pose task: {source_task}"
            )
        keep = scores.isfinite() & visibility.isfinite()
        return direction, scores[keep], visibility[keep].clamp(0, 1)

    def _prediction_record(
        self,
        task: HeadId,
        outputs: Mapping[str, Tensor],
        image_index: int,
        target: Mapping[str, object],
        height: int,
        width: int,
    ) -> dict[str, object]:
        """Build a JSON-safe record of the exact tensors used by metrics."""
        raw_path = target.get("path")
        if not isinstance(raw_path, Path):
            raise TypeError("Validation target is missing a Path 'path'")
        image_id = _target_tensor(target, "image_id")
        original_size = _target_tensor(target, "orig_size")
        record: dict[str, object] = {
            "version": 1,
            "task": str(task),
            "image_key": f"{task}:{raw_path.resolve()}",
            "image_id": int(image_id.item()),
            "path": str(raw_path.resolve()),
            "evaluation_size": [height, width],
            "original_size": original_size.detach().cpu().tolist(),
        }

        if task == HeadId.OBJECT:
            output = outputs["object_output"][image_index]
            labels = output[:, 5].long()
            valid_classes = _target_tensor(
                target,
                "valid_detection_classes",
            ).to(output.device)
            in_range = (labels >= 0) & (labels < valid_classes.numel())
            supported = torch.zeros_like(in_range)
            supported[in_range] = valid_classes[labels[in_range]]
            keep = (
                output[:, 4].isfinite()
                & (output[:, 4] >= self.confidence)
                & supported
            )
            record["predictions"] = {
                "boxes_xyxy": output[keep, :4].detach().cpu().tolist(),
                "scores": output[keep, 4].detach().cpu().tolist(),
                "labels": labels[keep].detach().cpu().tolist(),
            }
            record["targets"] = {
                "boxes_xyxy": _pixel_xyxy(
                    _target_tensor(target, "boxes"),
                    height,
                    width,
                )
                .detach()
                .cpu()
                .tolist(),
                "labels": _target_tensor(target, "labels")
                .long()
                .detach()
                .cpu()
                .tolist(),
            }
            return record

        if task == HeadId.PERSON_POSE:
            output = outputs["person_pose_output"][image_index]
            selected, scores, mean_visibility = self._person_predictions(
                output,
                self.person_visibility_alphas[0],
            )
            record["predictions"] = {
                "boxes_xyxy": selected[:, :4].detach().cpu().tolist(),
                "scores": scores.detach().cpu().tolist(),
                "mean_visibility": mean_visibility.detach().cpu().tolist(),
                "visibility_alpha": self.person_visibility_alphas[0],
                "keypoints": selected[:, 6:]
                .reshape(-1, 17, 3)
                .detach()
                .cpu()
                .tolist(),
            }
            record["targets"] = {
                "boxes_cxcywh": _pixel_cxcywh(
                    _target_tensor(target, "boxes"),
                    height,
                    width,
                )
                .detach()
                .cpu()
                .tolist(),
                "keypoints": _pixel_keypoints(
                    _target_tensor(target, "keypoints"),
                    height,
                    width,
                )
                .detach()
                .cpu()
                .tolist(),
                "visibility": _target_tensor(target, "visibility")
                .bool()
                .detach()
                .cpu()
                .tolist(),
            }
            return record

        if task == HeadId.ROBOT_POSE:
            objects = outputs["object_output"][image_index]
            poses = outputs["robot_pose_output"][image_index]
            selected_poses, scores, mean_visibility = self._robot_predictions(
                objects,
                poses,
                self.robot_visibility_alphas[0],
            )
            record["predictions"] = {
                "scores": scores.detach().cpu().tolist(),
                "mean_visibility": mean_visibility.detach().cpu().tolist(),
                "visibility_alpha": self.robot_visibility_alphas[0],
                "keypoints": selected_poses.detach().cpu().tolist(),
            }
            record["targets"] = {
                "boxes_cxcywh": _pixel_cxcywh(
                    _target_tensor(target, "boxes"),
                    height,
                    width,
                )
                .detach()
                .cpu()
                .tolist(),
                "keypoints": _pixel_keypoints(
                    _target_tensor(target, "keypoints"),
                    height,
                    width,
                )
                .detach()
                .cpu()
                .tolist(),
                "visibility": _target_tensor(target, "visibility")
                .bool()
                .detach()
                .cpu()
                .tolist(),
            }
            return record

        if task == HeadId.FIELD_FEATURES:
            output = outputs["field_feature_output"][image_index]
            keep = output[:, 2].isfinite() & (output[:, 2] >= self.confidence)
            points = _target_tensor(target, "points").clone()
            points[:, 0] *= width
            points[:, 1] *= height
            record["predictions"] = {
                "points": output[keep].detach().cpu().tolist(),
            }
            record["targets"] = {
                "points": points.detach().cpu().tolist(),
                "labels": _target_tensor(target, "point_labels")
                .long()
                .detach()
                .cpu()
                .tolist(),
                "boxes_cxcywh": _pixel_cxcywh(
                    _target_tensor(target, "boxes"),
                    height,
                    width,
                )
                .detach()
                .cpu()
                .tolist(),
            }
            return record

        raise ValueError(f"Unsupported validation task: {task}")

    @staticmethod
    def _update_score_diagnostics(
        values: dict[str, float],
        scores: Tensor,
    ) -> None:
        scores = scores.detach()
        values["images"] += 1
        values["max_sum"] += float(scores.max()) if scores.numel() else 0.0
        for name, threshold in _SCORE_THRESHOLDS:
            values[f"at_{name}"] += float((scores >= threshold).sum())

    @staticmethod
    def _score_diagnostic_metrics(
        prefix: str,
        values: Mapping[str, float],
    ) -> dict[str, float]:
        images = values["images"]
        if images == 0:
            return {}
        return {
            f"{prefix}/max_score_mean": values["max_sum"] / images,
            **{
                f"{prefix}/detections_at_{name}_per_image": (
                    values[f"at_{name}"] / images
                )
                for name, _ in _SCORE_THRESHOLDS
            },
        }

    def _update_objects(
        self,
        metric: MeanAveragePrecision,
        output: Tensor,
        target: Mapping[str, object],
        height: int,
        width: int,
    ) -> None:
        keep = output[:, 4].isfinite() & (output[:, 4] >= self.confidence)
        predicted_labels = output[:, 5].long()
        valid_classes = _target_tensor(
            target,
            "valid_detection_classes",
        ).to(output.device)
        in_range = (predicted_labels >= 0) & (
            predicted_labels < valid_classes.numel()
        )
        supported = torch.zeros_like(in_range)
        supported[in_range] = valid_classes[predicted_labels[in_range]]
        keep &= supported
        metric.update(
            [
                {
                    "boxes": output[keep, :4].cpu(),
                    "scores": output[keep, 4].cpu(),
                    "labels": output[keep, 5].long().cpu(),
                }
            ],
            [
                {
                    "boxes": _pixel_xyxy(
                        _target_tensor(target, "boxes"),
                        height,
                        width,
                    ).cpu(),
                    "labels": _target_tensor(target, "labels").long().cpu(),
                }
            ],
        )

    def _update_person(
        self,
        metric: KeypointOKSAccumulator,
        output: Tensor,
        target: Mapping[str, object],
        height: int,
        width: int,
        *,
        visibility_alpha: float,
    ) -> None:
        selected, scores, _ = self._person_predictions(
            output,
            visibility_alpha,
        )
        metric.update(
            selected[:, 6:].reshape(-1, 17, 3),
            _pixel_keypoints(
                _target_tensor(target, "keypoints"),
                height,
                width,
            ),
            _pixel_cxcywh(
                _target_tensor(target, "boxes"),
                height,
                width,
            ),
            predicted_scores=scores,
            visibility=_target_tensor(target, "visibility"),
        )

    def _person_predictions(
        self,
        output: Tensor,
        visibility_alpha: float,
    ) -> tuple[Tensor, Tensor, Tensor]:
        """Select person poses and calibrate only their evaluator scores."""
        mean_visibility = (
            output[:, 6:]
            .reshape(-1, PERSON_POSE_SCHEMA.keypoint_count, 3)[..., 2]
            .mean(dim=-1)
            .clamp(0, 1)
        )
        scores = self._person_calibrated_scores(output, visibility_alpha)
        keep = scores.isfinite() & (scores >= self.confidence)
        selected = output[keep]
        scores = scores[keep]
        mean_visibility = mean_visibility[keep]
        if visibility_alpha > 0:
            order = torch.argsort(scores, descending=True, stable=True)
            selected = selected[order]
            scores = scores[order]
            mean_visibility = mean_visibility[order]
        return (
            selected[: self.pose_max_detections],
            scores[: self.pose_max_detections],
            mean_visibility[: self.pose_max_detections],
        )

    @staticmethod
    def _person_calibrated_scores(
        output: Tensor,
        visibility_alpha: float,
    ) -> Tensor:
        """Return public person scores with no filtering or mutation."""
        scores = output[:, 4].clone()
        if visibility_alpha == 0:
            return scores
        mean_visibility = (
            output[:, 6:]
            .reshape(-1, PERSON_POSE_SCHEMA.keypoint_count, 3)[..., 2]
            .mean(dim=-1)
            .clamp(0, 1)
        )
        return scores * mean_visibility.pow(visibility_alpha)

    def _update_robot(
        self,
        metric: KeypointOKSAccumulator,
        objects: Tensor,
        poses: Tensor,
        target: Mapping[str, object],
        height: int,
        width: int,
        *,
        visibility_alpha: float,
    ) -> None:
        selected_poses, scores, _ = self._robot_predictions(
            objects,
            poses,
            visibility_alpha,
        )
        metric.update(
            selected_poses,
            _pixel_keypoints(
                _target_tensor(target, "keypoints"),
                height,
                width,
            ),
            _pixel_cxcywh(
                _target_tensor(target, "boxes"),
                height,
                width,
            ),
            predicted_scores=scores,
            visibility=_target_tensor(target, "visibility"),
        )

    def _robot_predictions(
        self,
        objects: Tensor,
        poses: Tensor,
        visibility_alpha: float,
    ) -> tuple[Tensor, Tensor, Tensor]:
        """Select robot poses and calibrate only their evaluator scores."""
        robot_poses, base_scores, mean_visibility = self._robot_candidates(
            objects,
            poses,
        )
        scores = base_scores.clone()
        if visibility_alpha > 0:
            scores = scores * mean_visibility.pow(visibility_alpha)
        keep = scores.isfinite() & (scores >= self.confidence)
        scores = scores[keep]
        robot_poses = robot_poses[keep]
        mean_visibility = mean_visibility[keep]
        order = torch.argsort(scores, descending=True, stable=True)
        order = order[: self.pose_max_detections]
        return robot_poses[order], scores[order], mean_visibility[order]

    def _robot_candidates(
        self,
        objects: Tensor,
        poses: Tensor,
    ) -> tuple[Tensor, Tensor, Tensor]:
        """Return every public Robot-class pose before evaluator filtering."""
        robot_class = self.model.detector.names.index("Robot")
        robot_mask = objects[:, 5].long() == robot_class
        robot_poses = poses[robot_mask]
        base_scores = objects[robot_mask, 4]
        mean_visibility = robot_poses[..., 2].mean(dim=-1).clamp(0, 1)
        return robot_poses, base_scores, mean_visibility

    def _update_field(
        self,
        metric: FieldPointAPAccumulator,
        output: Tensor,
        target: Mapping[str, object],
        height: int,
        width: int,
    ) -> None:
        selected = self._confident_field_predictions(output)
        points = _target_tensor(target, "points").clone()
        points[:, 0] *= width
        points[:, 1] *= height
        metric.update(
            selected,
            points,
            _target_tensor(target, "point_labels"),
            _pixel_cxcywh(
                _target_tensor(target, "boxes"),
                height,
                width,
            ),
        )

    def _confident_field_predictions(self, output: Tensor) -> Tensor:
        keep = output[:, 2].isfinite() & (output[:, 2] >= self.confidence)
        return output[keep]
