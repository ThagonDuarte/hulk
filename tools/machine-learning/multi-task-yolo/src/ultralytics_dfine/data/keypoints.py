"""YOLO-format keypoint datasets with global detector-class mapping."""

# ruff: noqa: S311, TRY003

import random
from pathlib import Path
from typing import Literal

import torch
import torchvision.transforms.v2.functional as transform_functional
from PIL import Image
from torch import Tensor
from torch.utils.data import Dataset

from ultralytics_dfine.data.dataset import (
    DatasetDefinition,
    DatasetTarget,
    _images_from_source,
    _label_path,
    load_dataset_yaml,
)
from ultralytics_dfine.schemas import HeadId


class YOLOKeypointDataset(Dataset[tuple[Tensor, DatasetTarget]]):
    """Load homogeneous YOLO pose rows and map classes into D-FINE."""

    def __init__(
        self,
        data: str | Path | DatasetDefinition,
        split: Literal["train", "val", "test"],
        *,
        schema_id: HeadId,
        keypoint_count: int,
        flip_idx: tuple[int, ...],
        global_class_ids: tuple[int, ...],
        keypoint_dimensions: Literal[2, 3] = 3,
        num_detection_classes: int = 8,
        image_size: int = 640,
        horizontal_flip_probability: float = 0.5,
        point_set: bool = False,
        point_label_ids: tuple[int, ...] | None = None,
    ) -> None:
        self.definition = (
            data
            if isinstance(data, DatasetDefinition)
            else load_dataset_yaml(data)
        )
        if len(self.definition.names) != len(global_class_ids):
            raise ValueError("Dataset classes and global mappings must match")
        if len(flip_idx) != keypoint_count:
            raise ValueError("flip_idx must match the keypoint count")
        if any(
            class_id < 0 or class_id >= num_detection_classes
            for class_id in global_class_ids
        ):
            raise ValueError("Global class mapping is out of range")
        self.schema_id = schema_id
        self.keypoint_count = keypoint_count
        self.keypoint_dimensions = keypoint_dimensions
        self.flip_idx = flip_idx
        self.global_class_ids = global_class_ids
        self.num_detection_classes = num_detection_classes
        self.image_size = image_size
        self.horizontal_flip_probability = horizontal_flip_probability
        self.point_set = point_set
        self.point_label_ids = point_label_ids
        if point_label_ids is not None and len(point_label_ids) != len(
            global_class_ids
        ):
            raise ValueError("Point label mapping and classes must match")
        self.split = split
        sources = getattr(self.definition, split)
        if not sources:
            raise ValueError(f"Dataset has no '{split}' split")
        self.images = [
            image for source in sources for image in _images_from_source(source)
        ]
        if not self.images:
            raise ValueError(f"Dataset '{split}' split contains no images")

    def __len__(self) -> int:
        return len(self.images)

    def audit_annotations(self) -> dict[str, int]:
        """Validate every keypoint label file before training starts."""
        files = 0
        missing_files = 0
        rows = 0
        for image_path in self.images:
            label_path = _label_path(image_path)
            if not label_path.exists():
                missing_files += 1
                continue
            files += 1
            labels, _, _ = self._load_labels(image_path)
            rows += labels.numel()
        return {
            "images": len(self.images),
            "label_files": files,
            "missing_label_files": missing_files,
            "rows": rows,
            "ignored_rows": 0,
        }

    def _load_labels(  # noqa: C901
        self,
        image_path: Path,
    ) -> tuple[Tensor, Tensor, Tensor]:
        label_path = _label_path(image_path)
        if not label_path.exists():
            return (
                torch.empty(0, dtype=torch.long),
                torch.empty((0, 4), dtype=torch.float32),
                torch.empty(
                    (0, self.keypoint_count, 3),
                    dtype=torch.float32,
                ),
            )
        local_labels = []
        boxes = []
        keypoints = []
        expected = 5 + self.keypoint_count * self.keypoint_dimensions
        with label_path.open(encoding="utf-8") as file:
            for line_number, line in enumerate(file, start=1):
                values = line.split()
                if not values:
                    continue
                if len(values) != expected:
                    raise ValueError(
                        f"{label_path}:{line_number}: expected {expected} "
                        "YOLO pose values"
                    )
                local_class = int(values[0])
                if not 0 <= local_class < len(self.global_class_ids):
                    raise ValueError(
                        f"{label_path}:{line_number}: class is out of range"
                    )
                numeric = torch.tensor(
                    [float(value) for value in values[1:]],
                    dtype=torch.float32,
                )
                if not torch.isfinite(numeric).all():
                    raise ValueError(
                        f"{label_path}:{line_number}: values must be finite"
                    )
                box = numeric[:4]
                points = numeric[4:].reshape(
                    self.keypoint_count,
                    self.keypoint_dimensions,
                )
                coordinates = points[:, :2]
                if (coordinates < 0).any() or (coordinates > 1).any():
                    raise ValueError(
                        f"{label_path}:{line_number}: "
                        "keypoints must be normalized"
                    )
                if self.keypoint_dimensions == 3:
                    visibility = points[:, 2]
                    if (visibility < 0).any() or (visibility > 2).any():
                        raise ValueError(
                            f"{label_path}:{line_number}: invalid visibility"
                        )
                if self.keypoint_dimensions == 2:
                    points = torch.cat(
                        (points, torch.ones((self.keypoint_count, 1))),
                        dim=1,
                    )
                if (box < 0).any() or (box > 1).any():
                    raise ValueError(
                        f"{label_path}:{line_number}: box must be normalized"
                    )
                local_labels.append(local_class)
                boxes.append(box)
                keypoints.append(points)
        return (
            torch.tensor(local_labels, dtype=torch.long),
            torch.stack(boxes) if boxes else torch.empty((0, 4)),
            torch.stack(keypoints)
            if keypoints
            else torch.empty((0, self.keypoint_count, 3)),
        )

    def __getitem__(self, index: int) -> tuple[Tensor, DatasetTarget]:
        image_path = self.images[index]
        with Image.open(image_path) as loaded_image:
            image = loaded_image.convert("RGB")
        width, height = image.size
        image_tensor = transform_functional.to_image(image)
        local_labels, boxes, keypoints = self._load_labels(image_path)
        if (
            self.split == "train"
            and random.random() < self.horizontal_flip_probability
        ):
            image_tensor = transform_functional.horizontal_flip(image_tensor)
            if boxes.numel() > 0:
                boxes[:, 0] = 1 - boxes[:, 0]
                keypoints[..., 0] = 1 - keypoints[..., 0]
                keypoints = keypoints[:, self.flip_idx]
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
        mapping = torch.tensor(self.global_class_ids, dtype=torch.long)
        labels = mapping[local_labels]
        valid_classes = torch.zeros(
            self.num_detection_classes,
            dtype=torch.bool,
        )
        valid_classes[mapping] = True
        target: DatasetTarget = {
            "labels": labels,
            "boxes": boxes,
            "keypoints": keypoints,
            "visibility": keypoints[..., 2] > 0,
            "schema_id": str(self.schema_id),
            "valid_detection_classes": valid_classes,
            "orig_size": torch.tensor([height, width], dtype=torch.long),
            "image_id": torch.tensor(index, dtype=torch.long),
            "path": image_path,
        }
        if self.point_set:
            point_mapping = torch.tensor(
                self.point_label_ids
                if self.point_label_ids is not None
                else tuple(range(len(self.global_class_ids))),
                dtype=torch.long,
            )
            target["point_labels"] = point_mapping[local_labels]
            target["points"] = keypoints[:, 0, :2]
        return image, target
