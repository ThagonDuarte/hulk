"""Native adapter for the Diverse Humanoid Robot Pose dataset."""

# ruff: noqa: S311, TRY003

import json
import math
import random
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import TypedDict, cast

import torch
import torchvision.transforms.v2.functional as transform_functional
from PIL import Image
from torch import Tensor
from torch.utils.data import Dataset

from ultralytics_dfine.schemas import (
    DHRP_FLIP_IDX,
    DHRP_JOINT_PARENTS,
    DHRP_KEYPOINT_NAMES,
)

DHRP_NUM_KEYPOINTS = len(DHRP_KEYPOINT_NAMES)
DHRP_ROBOT_CLASS_ID = 4
DHRP_BOX_PADDING = 0.15
DHRP_MIN_BOX_SIZE = 1.0

_EVE_ANNOTATION_FILE = "train_set_TargetHumanoidRobots_EVE.json"
_EVE_BAD_ANNOTATION_INDEX = 303
_EVE_CORRECTED_IMAGE = "train/TargetHumanoidRobots/EVE/eve4_0.png"


class DHRPFormatError(ValueError):
    """Raised when native DHRP data does not match its published schema."""

    def __init__(self, context: object, detail: str) -> None:
        super().__init__(f"{context}: {detail}")


class DHRPTypeError(TypeError):
    """Raised when a DHRP JSON value has the wrong type."""

    def __init__(self, context: object, expected: str) -> None:
        super().__init__(f"{context} must be {expected}")


class DHRPImageNotFoundError(FileNotFoundError):
    """Raised when an annotation refers to a missing image."""

    def __init__(self, context: str, image_path: Path) -> None:
        super().__init__(f"{context}: image not found: {image_path}")


class DHRPTarget(TypedDict):
    """Untransformed normalized target for one DHRP image."""

    labels: Tensor
    boxes: Tensor
    keypoints: Tensor
    visibility: Tensor
    schema_id: str
    valid_detection_classes: Tensor
    orig_size: Tensor
    image_id: Tensor
    path: Path


@dataclass(frozen=True)
class DHRPRecord:
    """Validated annotation data without a decoded image."""

    image_path: Path
    width: int
    height: int
    class_id: int
    box: tuple[float, float, float, float] | None
    keypoints: tuple[tuple[float, float, float], ...]

    def target(self, image_id: int) -> DHRPTarget:
        """Build fresh tensors so transforms may mutate them safely."""
        if self.box is None:
            labels = torch.empty((0,), dtype=torch.long)
            boxes = torch.empty((0, 4), dtype=torch.float32)
            keypoints = torch.empty(
                (0, DHRP_NUM_KEYPOINTS, 3),
                dtype=torch.float32,
            )
        else:
            labels = torch.tensor([self.class_id], dtype=torch.long)
            boxes = torch.tensor([self.box], dtype=torch.float32)
            keypoints = torch.tensor(
                [self.keypoints],
                dtype=torch.float32,
            )
        return {
            "labels": labels,
            "boxes": boxes,
            "keypoints": keypoints,
            "visibility": keypoints[..., 2] > 0,
            "schema_id": "robot_pose",
            "valid_detection_classes": torch.nn.functional.one_hot(
                torch.tensor(self.class_id),
                num_classes=8,
            ).bool(),
            "orig_size": torch.tensor(
                [self.height, self.width],
                dtype=torch.long,
            ),
            "image_id": torch.tensor(image_id, dtype=torch.long),
            "path": self.image_path,
        }


def correct_dhrp_image_path(
    annotation_file: str | Path,
    annotation_index: int,
    image_path: str,
) -> str:
    """Apply corrections for known errors in the published annotations."""
    if (
        Path(annotation_file).name == _EVE_ANNOTATION_FILE
        and annotation_index == _EVE_BAD_ANNOTATION_INDEX
    ):
        return _EVE_CORRECTED_IMAGE
    return image_path


def _require_mapping(value: object, context: str) -> dict[str, object]:
    if not isinstance(value, dict):
        raise DHRPTypeError(context, "an object")
    return cast("dict[str, object]", value)


def _require_sequence(value: object, context: str) -> Sequence[object]:
    if not isinstance(value, list):
        raise DHRPTypeError(context, "an array")
    return value


def _validate_metadata(data: dict[str, object], annotation_path: Path) -> None:
    names = tuple(
        str(value)
        for value in _require_sequence(
            data.get("joint_names"),
            f"{annotation_path}: joint_names",
        )
    )
    if names != DHRP_KEYPOINT_NAMES:
        raise DHRPFormatError(
            annotation_path,
            "joint_names do not match the DHRP schema",
        )
    raw_parents = _require_sequence(
        data.get("joint_parents"),
        f"{annotation_path}: joint_parents",
    )
    if not all(isinstance(value, int) for value in raw_parents):
        raise DHRPTypeError(f"{annotation_path}: joint_parents", "integers")
    parents = tuple(cast("int", value) for value in raw_parents)
    if parents != DHRP_JOINT_PARENTS:
        raise DHRPFormatError(
            annotation_path,
            "joint_parents do not match the DHRP schema",
        )
    if data.get("num_joints") != DHRP_NUM_KEYPOINTS:
        raise DHRPFormatError(
            annotation_path,
            f"num_joints must be {DHRP_NUM_KEYPOINTS}",
        )


def _parse_keypoints(
    value: object,
    *,
    width: int,
    height: int,
    context: str,
) -> tuple[tuple[tuple[float, float, float], ...], list[tuple[float, float]]]:
    nodes = _require_sequence(value, context)
    if len(nodes) != DHRP_NUM_KEYPOINTS:
        raise DHRPFormatError(
            context,
            f"must contain {DHRP_NUM_KEYPOINTS} points",
        )

    normalized = []
    box_points = []
    for index, raw_node in enumerate(nodes):
        node_context = f"{context}[{index}]"
        values_context = f"{node_context} values"
        visibility_context = f"{node_context} visibility"
        node = _require_sequence(raw_node, node_context)
        if len(node) != 3:
            raise DHRPFormatError(
                node_context,
                "must contain x, y, visibility",
            )
        if not all(isinstance(item, (int, float)) for item in node):
            raise DHRPTypeError(values_context, "numeric")
        x = float(cast("int | float", node[0]))
        y = float(cast("int | float", node[1]))
        raw_visibility = float(cast("int | float", node[2]))
        if raw_visibility not in (0.0, 1.0):
            raise DHRPFormatError(
                visibility_context,
                "must be binary",
            )
        finite = math.isfinite(x) and math.isfinite(y)
        if finite and (x != 0.0 or y != 0.0):
            box_points.append((x, y))
        normalized.append(
            (
                x / width if math.isfinite(x) else 0.0,
                y / height if math.isfinite(y) else 0.0,
                raw_visibility,
            )
        )
    return tuple(normalized), box_points


def _axis_bounds(
    low: float,
    high: float,
    limit: float,
    padding: float,
    minimum_size: float,
) -> tuple[float, float]:
    span = high - low
    low = max(0.0, low - span * padding)
    high = min(limit, high + span * padding)
    desired = min(limit, max(minimum_size, high - low))
    center = (low + high) / 2.0
    low = max(0.0, min(center - desired / 2.0, limit - desired))
    return low, low + desired


def _box_from_points(
    points: Sequence[tuple[float, float]],
    *,
    width: int,
    height: int,
    padding: float,
    minimum_size: float,
) -> tuple[float, float, float, float] | None:
    if not points:
        return None
    x_values, y_values = zip(*points, strict=True)
    left, right = _axis_bounds(
        min(x_values),
        max(x_values),
        float(width),
        padding,
        minimum_size,
    )
    top, bottom = _axis_bounds(
        min(y_values),
        max(y_values),
        float(height),
        padding,
        minimum_size,
    )
    return (
        (left + right) / (2.0 * width),
        (top + bottom) / (2.0 * height),
        (right - left) / width,
        (bottom - top) / height,
    )


def _load_annotation_file(
    dataset_root: Path,
    annotation_path: Path,
    *,
    class_id: int,
    box_padding: float,
    minimum_box_size: float,
) -> list[DHRPRecord]:
    with annotation_path.open(encoding="utf-8") as file:
        data = _require_mapping(json.load(file), str(annotation_path))
    _validate_metadata(data, annotation_path)
    annotations = _require_sequence(
        data.get("annotations"),
        f"{annotation_path}: annotations",
    )
    if data.get("num_images") != len(annotations):
        raise DHRPFormatError(
            annotation_path,
            "num_images does not match annotations",
        )

    records = []
    for index, value in enumerate(annotations):
        context = f"{annotation_path}: annotation {index}"
        annotation = _require_mapping(value, context)
        raw_image_path = annotation.get("img")
        if not isinstance(raw_image_path, str) or not raw_image_path:
            image_context = f"{context}: img"
            raise DHRPTypeError(image_context, "a non-empty string")
        corrected_path = correct_dhrp_image_path(
            annotation_path,
            index,
            raw_image_path,
        )
        image_path = dataset_root / Path(corrected_path)
        if not image_path.is_file():
            raise DHRPImageNotFoundError(context, image_path)
        with Image.open(image_path) as image:
            width, height = image.size
        if width <= 0 or height <= 0:
            raise DHRPFormatError(context, "image dimensions must be positive")
        keypoints, box_points = _parse_keypoints(
            annotation.get("key"),
            width=width,
            height=height,
            context=f"{context}: key",
        )
        records.append(
            DHRPRecord(
                image_path=image_path,
                width=width,
                height=height,
                class_id=class_id,
                box=_box_from_points(
                    box_points,
                    width=width,
                    height=height,
                    padding=box_padding,
                    minimum_size=minimum_box_size,
                ),
                keypoints=keypoints,
            )
        )
    return records


def load_dhrp_records(
    root: str | Path,
    annotation_files: str | Path | Sequence[str | Path],
    *,
    class_id: int = DHRP_ROBOT_CLASS_ID,
    box_padding: float = DHRP_BOX_PADDING,
    minimum_box_size: float = DHRP_MIN_BOX_SIZE,
) -> tuple[DHRPRecord, ...]:
    """Parse one or more native DHRP train/eval annotation files."""
    if class_id < 0:
        raise DHRPFormatError("class_id", "must be non-negative")
    if box_padding < 0.0:
        raise DHRPFormatError("box_padding", "must be non-negative")
    if minimum_box_size <= 0.0:
        raise DHRPFormatError("minimum_box_size", "must be positive")

    dataset_root = Path(root).expanduser().resolve()
    files = (
        [annotation_files]
        if isinstance(annotation_files, (str, Path))
        else annotation_files
    )
    records = []
    for annotation_file in files:
        annotation_path = Path(annotation_file).expanduser()
        if not annotation_path.is_absolute():
            annotation_path = dataset_root / annotation_path
        records.extend(
            _load_annotation_file(
                dataset_root,
                annotation_path,
                class_id=class_id,
                box_padding=box_padding,
                minimum_box_size=minimum_box_size,
            )
        )
    return tuple(records)


class DHRPDataset(Dataset[tuple[Tensor, DHRPTarget]]):
    """Transform-free DHRP dataset backed by validated lightweight records."""

    def __init__(
        self,
        root: str | Path,
        annotation_files: str | Path | Sequence[str | Path],
        *,
        class_id: int = DHRP_ROBOT_CLASS_ID,
        box_padding: float = DHRP_BOX_PADDING,
        minimum_box_size: float = DHRP_MIN_BOX_SIZE,
        image_size: int = 640,
        horizontal_flip_probability: float = 0.5,
        training: bool = False,
    ) -> None:
        self.records = load_dhrp_records(
            root,
            annotation_files,
            class_id=class_id,
            box_padding=box_padding,
            minimum_box_size=minimum_box_size,
        )
        self.image_size = image_size
        self.horizontal_flip_probability = horizontal_flip_probability
        self.training = training

    def __len__(self) -> int:
        return len(self.records)

    def __getitem__(self, index: int) -> tuple[Tensor, DHRPTarget]:
        record = self.records[index]
        with Image.open(record.image_path) as loaded_image:
            image = loaded_image.convert("RGB")
        image_tensor = transform_functional.to_image(image)
        target = record.target(index)
        if self.training and random.random() < self.horizontal_flip_probability:
            image_tensor = transform_functional.horizontal_flip(image_tensor)
            if target["boxes"].numel() > 0:
                target["boxes"][:, 0] = 1 - target["boxes"][:, 0]
                target["keypoints"][..., 0] = 1 - target["keypoints"][..., 0]
                target["keypoints"] = target["keypoints"][:, DHRP_FLIP_IDX]
                target["visibility"] = target["visibility"][:, DHRP_FLIP_IDX]
        image = transform_functional.resize(
            image_tensor,
            [self.image_size, self.image_size],
            antialias=True,
        )
        image = transform_functional.to_dtype(
            image,
            torch.float32,
            scale=True,
        ).as_subclass(Tensor)
        return image, target
