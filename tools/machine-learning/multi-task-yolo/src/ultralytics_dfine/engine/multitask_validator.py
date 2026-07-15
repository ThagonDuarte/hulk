"""Independent validation for every multi-task D-FINE output."""

# ruff: noqa: C901, TRY003

from collections.abc import Mapping
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
    ) -> None:
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

    @torch.no_grad()
    def run(
        self,
        *,
        render_dir: str | Path | None = None,
        max_batches: int | None = None,
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
        person_metric = KeypointOKSAccumulator.from_schema(
            PERSON_POSE_SCHEMA,
            box_format="xywh",
        )
        robot_metric = KeypointOKSAccumulator.from_schema(
            ROBOT_POSE_SCHEMA,
            box_format="xywh",
        )
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
            "person": {
                "images": 0.0,
                "max_sum": 0.0,
                "at_001": 0.0,
                "at_25": 0.0,
                "at_05": 0.0,
                "at_10": 0.0,
                "at_50": 0.0,
                "at_75": 0.0,
            },
            "robot": {
                "images": 0.0,
                "max_sum": 0.0,
                "at_001": 0.0,
                "at_25": 0.0,
                "at_05": 0.0,
                "at_10": 0.0,
                "at_50": 0.0,
                "at_75": 0.0,
            },
            "field": {
                "images": 0.0,
                "max_sum": 0.0,
                "at_001": 0.0,
                "at_25": 0.0,
                "at_05": 0.0,
                "at_10": 0.0,
                "at_50": 0.0,
                "at_75": 0.0,
            },
            "object": {
                "images": 0.0,
                "max_sum": 0.0,
                "at_001": 0.0,
                "at_05": 0.0,
                "at_10": 0.0,
                "at_25": 0.0,
                "at_50": 0.0,
                "at_75": 0.0,
            },
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
                        self._update_score_diagnostics(
                            score_diagnostics["person"],
                            outputs["person_pose_output"][image_index, :, 4],
                        )
                        self._update_person(
                            person_metric,
                            outputs["person_pose_output"][image_index],
                            target,
                            height,
                            width,
                        )
                    elif task == HeadId.ROBOT_POSE:
                        object_rows = outputs["object_output"][image_index]
                        robot_class = self.model.detector.names.index("Robot")
                        self._update_score_diagnostics(
                            score_diagnostics["robot"],
                            object_rows[
                                object_rows[:, 5].long() == robot_class,
                                4,
                            ],
                        )
                        self._update_robot(
                            robot_metric,
                            outputs["object_output"][image_index],
                            outputs["robot_pose_output"][image_index],
                            target,
                            height,
                            width,
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
                            outputs["field_feature_output"][image_index],
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
        for prefix, metric, task in (
            ("person", person_metric, HeadId.PERSON_POSE),
            ("robot", robot_metric, HeadId.ROBOT_POSE),
        ):
            if task not in self.loaders:
                continue
            computed = metric.compute()
            metrics.update(
                {
                    f"{prefix}/map": computed["ap"],
                    f"{prefix}/map50": computed["ap50"],
                    f"{prefix}/map75": computed["ap75"],
                    f"{prefix}/mar": computed["ar"],
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
        return metrics

    @staticmethod
    def _update_score_diagnostics(
        values: dict[str, float],
        scores: Tensor,
    ) -> None:
        scores = scores.detach()
        values["images"] += 1
        values["max_sum"] += float(scores.max()) if scores.numel() else 0.0
        values["at_001"] += float((scores >= 0.001).sum())
        values["at_25"] += float((scores >= 0.25).sum())
        values["at_05"] += float((scores >= 0.05).sum())
        values["at_10"] += float((scores >= 0.1).sum())
        values["at_50"] += float((scores >= 0.5).sum())
        values["at_75"] += float((scores >= 0.75).sum())

    def _update_objects(
        self,
        metric: MeanAveragePrecision,
        output: Tensor,
        target: Mapping[str, object],
        height: int,
        width: int,
    ) -> None:
        keep = output[:, 4].isfinite()
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
    ) -> None:
        keep = output[:, 4].isfinite()
        selected = output[keep][: self.pose_max_detections]
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
            predicted_scores=selected[:, 4],
            visibility=_target_tensor(target, "visibility"),
        )

    def _update_robot(
        self,
        metric: KeypointOKSAccumulator,
        objects: Tensor,
        poses: Tensor,
        target: Mapping[str, object],
        height: int,
        width: int,
    ) -> None:
        robot_class = self.model.detector.names.index("Robot")
        keep = objects[:, 4].isfinite() & (objects[:, 5].long() == robot_class)
        selected_objects = objects[keep][: self.pose_max_detections]
        selected_poses = poses[keep][: self.pose_max_detections]
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
            predicted_scores=selected_objects[:, 4],
            visibility=_target_tensor(target, "visibility"),
        )

    def _update_field(
        self,
        metric: FieldPointAPAccumulator,
        output: Tensor,
        target: Mapping[str, object],
        height: int,
        width: int,
    ) -> None:
        keep = output[:, 2].isfinite()
        points = _target_tensor(target, "points").clone()
        points[:, 0] *= width
        points[:, 1] *= height
        metric.update(
            output[keep],
            points,
            _target_tensor(target, "point_labels"),
            _pixel_cxcywh(
                _target_tensor(target, "boxes"),
                height,
                width,
            ),
        )
