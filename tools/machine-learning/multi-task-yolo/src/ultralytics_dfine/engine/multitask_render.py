"""Filtered validation rendering for fixed-shape multi-task outputs."""

from collections.abc import Mapping, Sequence
from pathlib import Path

import torch
from PIL import Image, ImageDraw
from torch import Tensor

from ultralytics_dfine.schemas import (
    FIELD_FEATURE_SCHEMA,
    PERSON_POSE_SCHEMA,
    ROBOT_POSE_SCHEMA,
    HeadId,
    PoseSchemaConfig,
)


def _image(tensor: Tensor) -> Image.Image:
    values = tensor.detach().cpu().clamp(0, 1)
    values = (values * 255).to(torch.uint8).permute(1, 2, 0).numpy()
    return Image.fromarray(values)


def _save(image: Image.Image, destination: Path) -> Path:
    destination.parent.mkdir(parents=True, exist_ok=True)
    image.save(destination)
    return destination


def _box_iou(box: Tensor, boxes: Tensor) -> Tensor:
    top_left = torch.maximum(box[:2], boxes[:, :2])
    bottom_right = torch.minimum(box[2:4], boxes[:, 2:4])
    intersection = (bottom_right - top_left).clamp_min(0).prod(dim=1)
    box_area = (box[2:4] - box[:2]).clamp_min(0).prod()
    areas = (boxes[:, 2:4] - boxes[:, :2]).clamp_min(0).prod(dim=1)
    return intersection / (box_area + areas - intersection).clamp_min(1e-9)


def _box_suppression_indices(
    candidates: Tensor,
    *,
    maximum_iou: float = 0.5,
    class_aware: bool,
    max_detections: int,
) -> Tensor:
    """Return score-ordered indices after box-IoU suppression."""
    if candidates.numel() == 0:
        return torch.empty(0, dtype=torch.long)
    order = candidates[:, 4].argsort(descending=True)
    kept: list[int] = []
    while order.numel() and len(kept) < max_detections:
        selected = int(order[0])
        kept.append(selected)
        remaining = order[1:]
        if remaining.numel() == 0:
            break
        suppress = (
            _box_iou(
                candidates[selected, :4],
                candidates[remaining, :4],
            )
            >= maximum_iou
        )
        if class_aware:
            suppress &= (
                candidates[remaining, 5].long()
                == candidates[selected, 5].long()
            )
        order = remaining[~suppress]
    return torch.tensor(kept, dtype=torch.long)


def _suppress_field_points(
    candidates: Tensor,
    *,
    radius: float = 8.0,
    max_detections: int,
) -> Tensor:
    """Suppress nearby same-class points and return score-ordered rows."""
    if candidates.numel() == 0:
        return candidates
    remaining = candidates[candidates[:, 2].argsort(descending=True)]
    kept = []
    while remaining.shape[0] and len(kept) < max_detections:
        selected = remaining[0]
        kept.append(selected)
        remaining = remaining[1:]
        if remaining.shape[0] == 0:
            break
        distance = torch.linalg.vector_norm(
            remaining[:, :2] - selected[:2],
            dim=1,
        )
        same_class = remaining[:, 3].long() == selected[3].long()
        remaining = remaining[~(same_class & (distance <= radius))]
    return torch.stack(kept) if kept else candidates[:0]


def _draw_box(
    draw: ImageDraw.ImageDraw,
    row: Tensor,
    *,
    color: str,
    label: str,
) -> None:
    box = tuple(float(value) for value in row[:4])
    draw.rectangle(box, outline=color, width=2)
    draw.text((box[0] + 2, box[1] + 2), label, fill=color)


def _draw_pose(
    draw: ImageDraw.ImageDraw,
    keypoints: Tensor,
    schema: PoseSchemaConfig,
    *,
    confidence: float,
    color: str = "lime",
) -> None:
    for start, end in schema.skeleton:
        if (
            keypoints[start, 2] >= confidence
            and keypoints[end, 2] >= confidence
        ):
            draw.line(
                (
                    float(keypoints[start, 0]),
                    float(keypoints[start, 1]),
                    float(keypoints[end, 0]),
                    float(keypoints[end, 1]),
                ),
                fill=color,
                width=2,
            )
    for x, y, score in keypoints:
        if float(score) >= confidence:
            draw.ellipse(
                (float(x) - 2, float(y) - 2, float(x) + 2, float(y) + 2),
                fill="yellow" if color == "lime" else color,
            )


def _normalized_boxes(
    boxes: Tensor,
    width: int,
    height: int,
) -> Tensor:
    scale = boxes.new_tensor([width, height, width, height])
    pixel = boxes.detach().cpu() * scale
    center = pixel[:, :2]
    half_size = pixel[:, 2:] / 2
    return torch.cat((center - half_size, center + half_size), dim=1)


def _draw_object_targets(
    draw: ImageDraw.ImageDraw,
    target: Mapping[str, object] | None,
    *,
    width: int,
    height: int,
    class_names: Sequence[str],
) -> None:
    if target is None:
        return
    boxes = target.get("boxes")
    labels = target.get("labels")
    if not isinstance(boxes, Tensor) or not isinstance(labels, Tensor):
        return
    for box, class_id in zip(
        _normalized_boxes(boxes, width, height),
        labels.detach().cpu().long(),
        strict=True,
    ):
        index = int(class_id)
        name = (
            class_names[index] if 0 <= index < len(class_names) else str(index)
        )
        _draw_box(draw, box, color="cyan", label=f"GT {name}")


def _draw_pose_targets(
    draw: ImageDraw.ImageDraw,
    target: Mapping[str, object] | None,
    schema: PoseSchemaConfig,
    *,
    width: int,
    height: int,
) -> None:
    if target is None:
        return
    keypoints = target.get("keypoints")
    visibility = target.get("visibility")
    boxes = target.get("boxes")
    if not isinstance(keypoints, Tensor) or not isinstance(boxes, Tensor):
        return
    points = keypoints.detach().cpu().clone()
    points[..., 0] *= width
    points[..., 1] *= height
    if isinstance(visibility, Tensor):
        visible = visibility.detach().cpu().to(points.dtype)
    elif points.shape[-1] >= 3:
        visible = points[..., 2]
    else:
        visible = torch.ones(points.shape[:2], dtype=points.dtype)
    pixel_boxes = _normalized_boxes(boxes, width, height)
    for box, pose, scores in zip(
        pixel_boxes,
        points[..., :2],
        visible,
        strict=True,
    ):
        _draw_box(draw, box, color="cyan", label="GT")
        _draw_pose(
            draw,
            torch.cat((pose, scores[..., None]), dim=-1),
            schema,
            confidence=0.5,
            color="cyan",
        )


def _draw_field_targets(
    draw: ImageDraw.ImageDraw,
    target: Mapping[str, object] | None,
    *,
    width: int,
    height: int,
) -> None:
    if target is None:
        return
    points = target.get("points")
    labels = target.get("point_labels")
    if not isinstance(points, Tensor) or not isinstance(labels, Tensor):
        return
    pixel = points.detach().cpu().clone()
    pixel[:, 0] *= width
    pixel[:, 1] *= height
    for point, class_id in zip(pixel, labels.detach().cpu(), strict=True):
        x, y = (float(value) for value in point)
        name = FIELD_FEATURE_SCHEMA.class_names[int(class_id)]
        draw.rectangle((x - 3, y - 3, x + 3, y + 3), outline="cyan")
        draw.text((x + 4, y + 4), f"GT {name}", fill="cyan")


def render_objects(
    image: Tensor,
    output: Tensor,
    destination: Path,
    *,
    confidence: float = 0.25,
    max_detections: int = 20,
    valid_classes: Tensor | None = None,
    class_names: Sequence[str] = (),
    target: Mapping[str, object] | None = None,
) -> Path:
    canvas = _image(image)
    draw = ImageDraw.Draw(canvas)
    candidates = output.detach().cpu()
    keep = candidates[:, 4].isfinite() & (candidates[:, 4] >= confidence)
    if valid_classes is not None:
        labels = candidates[:, 5].long()
        in_range = (labels >= 0) & (labels < valid_classes.numel())
        supported = torch.zeros_like(in_range)
        supported[in_range] = valid_classes.cpu()[labels[in_range]]
        keep &= supported
    candidates = candidates[keep]
    indices = _box_suppression_indices(
        candidates,
        class_aware=True,
        max_detections=max_detections,
    )
    for row in candidates[indices]:
        class_id = int(row[5])
        name = (
            class_names[class_id]
            if 0 <= class_id < len(class_names)
            else str(class_id)
        )
        _draw_box(
            draw,
            row,
            color="red",
            label=f"{name} {float(row[4]):.2f}",
        )
    _draw_object_targets(
        draw,
        target,
        width=canvas.width,
        height=canvas.height,
        class_names=class_names,
    )
    return _save(canvas, destination)


def render_pose(
    image: Tensor,
    output: Tensor,
    schema: PoseSchemaConfig,
    destination: Path,
    *,
    instance_confidence: float = 0.5,
    keypoint_confidence: float = 0.5,
    max_detections: int = 20,
    target: Mapping[str, object] | None = None,
) -> Path:
    canvas = _image(image)
    draw = ImageDraw.Draw(canvas)
    candidates = output.detach().cpu()
    candidates = candidates[
        candidates[:, 4].isfinite() & (candidates[:, 4] >= instance_confidence)
    ]
    indices = _box_suppression_indices(
        candidates,
        class_aware=False,
        max_detections=max_detections,
    )
    for row in candidates[indices]:
        _draw_box(
            draw,
            row,
            color="blue",
            label=f"Person {float(row[4]):.2f}",
        )
        _draw_pose(
            draw,
            row[6:].reshape(schema.keypoint_count, 3),
            schema,
            confidence=keypoint_confidence,
        )
    _draw_pose_targets(
        draw,
        target,
        schema,
        width=canvas.width,
        height=canvas.height,
    )
    return _save(canvas, destination)


def render_robot_pose(
    image: Tensor,
    objects: Tensor,
    poses: Tensor,
    destination: Path,
    *,
    robot_class_id: int,
    instance_confidence: float = 0.5,
    keypoint_confidence: float = 0.5,
    max_detections: int = 20,
    target: Mapping[str, object] | None = None,
) -> Path:
    canvas = _image(image)
    draw = ImageDraw.Draw(canvas)
    objects = objects.detach().cpu()
    poses = poses.detach().cpu()
    mask = (
        objects[:, 4].isfinite()
        & (objects[:, 4] >= instance_confidence)
        & (objects[:, 5].long() == robot_class_id)
    )
    objects = objects[mask]
    poses = poses[mask]
    indices = _box_suppression_indices(
        objects,
        class_aware=False,
        max_detections=max_detections,
    )
    for object_row, pose in zip(objects[indices], poses[indices], strict=True):
        _draw_box(
            draw,
            object_row,
            color="blue",
            label=f"Robot {float(object_row[4]):.2f}",
        )
        _draw_pose(
            draw,
            pose,
            ROBOT_POSE_SCHEMA,
            confidence=keypoint_confidence,
        )
    _draw_pose_targets(
        draw,
        target,
        ROBOT_POSE_SCHEMA,
        width=canvas.width,
        height=canvas.height,
    )
    return _save(canvas, destination)


def render_field_features(
    image: Tensor,
    output: Tensor,
    destination: Path,
    *,
    confidence: float = 0.35,
    max_detections: int = 20,
    target: Mapping[str, object] | None = None,
) -> Path:
    canvas = _image(image)
    draw = ImageDraw.Draw(canvas)
    candidates = output.detach().cpu()
    candidates = candidates[
        candidates[:, 2].isfinite() & (candidates[:, 2] >= confidence)
    ]
    candidates = _suppress_field_points(
        candidates,
        max_detections=max_detections,
    )
    for x, y, score, class_id in candidates:
        class_index = int(class_id)
        color = (
            255,
            class_index * 41 % 255,
            255 - class_index * 31 % 255,
        )
        name = FIELD_FEATURE_SCHEMA.class_names[class_index]
        draw.ellipse(
            (float(x) - 3, float(y) - 3, float(x) + 3, float(y) + 3),
            fill=color,
        )
        draw.text(
            (float(x) + 4, float(y) + 4),
            f"{name} {float(score):.2f}",
            fill=color,
        )
    _draw_field_targets(
        draw,
        target,
        width=canvas.width,
        height=canvas.height,
    )
    return _save(canvas, destination)


def render_all_outputs(
    image: Tensor,
    outputs: dict[str, Tensor],
    destination: str | Path,
    *,
    object_confidence: float = 0.25,
    person_confidence: float = 0.5,
    robot_confidence: float = 0.5,
    field_confidence: float = 0.35,
    keypoint_confidence: float = 0.5,
    max_detections: int = 20,
    robot_class_id: int = 4,
    valid_object_classes: Tensor | None = None,
    class_names: Sequence[str] = (),
    target: Mapping[str, object] | None = None,
    target_task: HeadId | None = None,
) -> dict[str, Path]:
    """Render filtered predictions and task-matched ground truth."""
    root = Path(destination)
    object_target = target if target_task == HeadId.OBJECT else None
    person_target = target if target_task == HeadId.PERSON_POSE else None
    robot_target = target if target_task == HeadId.ROBOT_POSE else None
    field_target = target if target_task == HeadId.FIELD_FEATURES else None
    return {
        "object_output": render_objects(
            image,
            outputs["object_output"][0],
            root / "object_output.jpg",
            confidence=object_confidence,
            max_detections=max_detections,
            valid_classes=valid_object_classes,
            class_names=class_names,
            target=object_target,
        ),
        "person_pose_output": render_pose(
            image,
            outputs["person_pose_output"][0],
            PERSON_POSE_SCHEMA,
            root / "person_pose_output.jpg",
            instance_confidence=person_confidence,
            keypoint_confidence=keypoint_confidence,
            max_detections=max_detections,
            target=person_target,
        ),
        "robot_pose_output": render_robot_pose(
            image,
            outputs["object_output"][0],
            outputs["robot_pose_output"][0],
            root / "robot_pose_output.jpg",
            robot_class_id=robot_class_id,
            instance_confidence=robot_confidence,
            keypoint_confidence=keypoint_confidence,
            max_detections=max_detections,
            target=robot_target,
        ),
        "field_feature_output": render_field_features(
            image,
            outputs["field_feature_output"][0],
            root / "field_feature_output.jpg",
            confidence=field_confidence,
            max_detections=max_detections,
            target=field_target,
        ),
    }
