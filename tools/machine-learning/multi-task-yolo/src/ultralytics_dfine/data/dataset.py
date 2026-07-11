# ruff: noqa: S311, TRY003

import random
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal, TypedDict

import torch
import torch.nn.functional as functional
import torchvision.transforms.v2 as transforms
import yaml
from PIL import Image
from torch import Tensor
from torch.utils.data import Dataset
from torchvision import ops, tv_tensors

IMAGE_SUFFIXES = {".bmp", ".jpeg", ".jpg", ".png", ".tif", ".tiff", ".webp"}


class DatasetTarget(TypedDict):
    labels: Tensor
    boxes: Tensor
    orig_size: Tensor
    image_id: Tensor
    path: Path


@dataclass(frozen=True)
class DatasetDefinition:
    yaml_path: Path
    root: Path
    names: list[str]
    train: tuple[Path, ...]
    val: tuple[Path, ...]
    test: tuple[Path, ...]


def _resolve_sources(root: Path, value: object) -> tuple[Path, ...]:
    if value is None:
        return ()
    values = value if isinstance(value, list) else [value]
    result = []
    for item in values:
        if not isinstance(item, str):
            raise TypeError("Dataset split entries must be paths")
        path = Path(item).expanduser()
        result.append(path if path.is_absolute() else root / path)
    return tuple(result)


def load_dataset_yaml(path: str | Path) -> DatasetDefinition:
    yaml_path = Path(path).expanduser().resolve()
    with yaml_path.open(encoding="utf-8") as file:
        values = yaml.safe_load(file)
    if not isinstance(values, dict):
        raise TypeError("Dataset YAML must contain a mapping")
    root_value = values.get("path", yaml_path.parent)
    root = Path(root_value).expanduser()
    if not root.is_absolute():
        root = (yaml_path.parent / root).resolve()
    raw_names = values.get("names")
    if isinstance(raw_names, dict):
        names = [
            str(
                raw_names[index]
                if index in raw_names
                else raw_names[str(index)]
            )
            for index in range(len(raw_names))
        ]
    elif isinstance(raw_names, list):
        names = [str(name) for name in raw_names]
    else:
        raise TypeError("Dataset YAML must define class names")
    if not names:
        raise ValueError("Dataset must define at least one class")
    return DatasetDefinition(
        yaml_path=yaml_path,
        root=root,
        names=names,
        train=_resolve_sources(root, values.get("train")),
        val=_resolve_sources(root, values.get("val")),
        test=_resolve_sources(root, values.get("test")),
    )


def _images_from_source(source: Path) -> list[Path]:
    if source.is_dir():
        return sorted(
            path
            for path in source.rglob("*")
            if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
        )
    if source.is_file() and source.suffix.lower() == ".txt":
        paths = []
        with source.open(encoding="utf-8") as file:
            for line in file:
                value = line.strip()
                if not value:
                    continue
                path = Path(value).expanduser()
                paths.append(
                    path if path.is_absolute() else source.parent / path
                )
        return paths
    if source.is_file() and source.suffix.lower() in IMAGE_SUFFIXES:
        return [source]
    raise FileNotFoundError(f"Dataset image source does not exist: {source}")


def _label_path(image_path: Path) -> Path:
    parts = list(image_path.parts)
    try:
        image_index = len(parts) - 1 - parts[::-1].index("images")
    except ValueError:
        return image_path.with_suffix(".txt")
    parts[image_index] = "labels"
    return Path(*parts).with_suffix(".txt")


class DFINEDataset(Dataset[tuple[Tensor, DatasetTarget]]):
    """YOLO-label dataset with the official D-FINE transform recipe."""

    def __init__(
        self,
        data: str | Path | DatasetDefinition,
        split: Literal["train", "val", "test"],
        *,
        image_size: int = 640,
        transition_epoch: int = 120,
    ) -> None:
        self.definition = (
            data
            if isinstance(data, DatasetDefinition)
            else load_dataset_yaml(data)
        )
        self.split = split
        self.image_size = image_size
        self.transition_epoch = transition_epoch
        self.epoch = 0
        sources = getattr(self.definition, split)
        if not sources:
            raise ValueError(f"Dataset has no '{split}' split")
        self.images = [
            image for source in sources for image in _images_from_source(source)
        ]
        if not self.images:
            raise ValueError(f"Dataset '{split}' split contains no images")
        self._augment = transforms.Compose(
            [
                transforms.RandomPhotometricDistort(p=0.5),
                transforms.RandomZoomOut(fill=0),
                transforms.RandomApply([transforms.RandomIoUCrop()], p=0.8),
                transforms.SanitizeBoundingBoxes(min_size=1),
                transforms.RandomHorizontalFlip(),
            ]
        )
        self._finalize = transforms.Compose(
            [
                transforms.Resize((image_size, image_size), antialias=True),
                transforms.SanitizeBoundingBoxes(min_size=1),
                transforms.ToImage(),
                transforms.ToDtype(torch.float32, scale=True),
            ]
        )

    @property
    def names(self) -> list[str]:
        return self.definition.names

    def set_epoch(self, epoch: int) -> None:
        self.epoch = epoch

    def __len__(self) -> int:
        return len(self.images)

    def _load_labels(
        self,
        image_path: Path,
        width: int,
        height: int,
    ) -> tuple[Tensor, Tensor]:
        label_path = _label_path(image_path)
        if not label_path.exists():
            return torch.empty((0,), dtype=torch.long), torch.empty((0, 4))
        labels: list[int] = []
        boxes: list[list[float]] = []
        with label_path.open(encoding="utf-8") as file:
            for line_number, line in enumerate(file, start=1):
                values = line.split()
                if not values:
                    continue
                if len(values) != 5:
                    raise ValueError(
                        f"{label_path}:{line_number}: expected five YOLO values"
                    )
                class_id = int(values[0])
                box = [float(value) for value in values[1:]]
                if not 0 <= class_id < len(self.names):
                    raise ValueError(
                        f"{label_path}:{line_number}: class {class_id} is out "
                        "of range"
                    )
                if not all(0.0 <= value <= 1.0 for value in box):
                    raise ValueError(
                        f"{label_path}:{line_number}: box must be normalized"
                    )
                center_x, center_y, box_width, box_height = box
                boxes.append(
                    [
                        (center_x - box_width / 2) * width,
                        (center_y - box_height / 2) * height,
                        (center_x + box_width / 2) * width,
                        (center_y + box_height / 2) * height,
                    ]
                )
                labels.append(class_id)
        return torch.tensor(labels, dtype=torch.long), torch.tensor(
            boxes,
            dtype=torch.float32,
        ).reshape(-1, 4)

    def __getitem__(self, index: int) -> tuple[Tensor, DatasetTarget]:
        image_path = self.images[index]
        with Image.open(image_path) as loaded_image:
            image = loaded_image.convert("RGB")
        width, height = image.size
        labels, boxes = self._load_labels(image_path, width, height)
        transform_target: dict[str, Any] = {
            "boxes": tv_tensors.BoundingBoxes(
                boxes,
                format="XYXY",
                canvas_size=(height, width),
            ),  # pyright: ignore[reportCallIssue]
            "labels": labels,
        }
        if self.split == "train" and self.epoch < self.transition_epoch:
            image, transform_target = self._augment(image, transform_target)
        image, transform_target = self._finalize(image, transform_target)
        transformed_boxes = ops.box_convert(
            transform_target["boxes"],
            in_fmt="xyxy",
            out_fmt="cxcywh",
        ).as_subclass(Tensor)
        transformed_boxes = transformed_boxes / torch.tensor(
            [self.image_size, self.image_size, self.image_size, self.image_size]
        )
        target: DatasetTarget = {
            "labels": transform_target["labels"].long(),
            "boxes": transformed_boxes.float(),
            "orig_size": torch.tensor([height, width], dtype=torch.long),
            "image_id": torch.tensor(index, dtype=torch.long),
            "path": image_path,
        }
        return image.as_subclass(Tensor), target


def generate_scales(base_size: int, base_size_repeat: int) -> list[int]:
    start = int(base_size * 0.75 / 32) * 32
    repeat = (base_size - start) // 32
    scales = [start + index * 32 for index in range(repeat)]
    scales.extend([base_size] * base_size_repeat)
    scales.extend(
        int(base_size * 1.25 / 32) * 32 - index * 32 for index in range(repeat)
    )
    return scales


class BatchImageCollateFunction:
    def __init__(
        self,
        *,
        base_size: int = 640,
        base_size_repeat: int = 20,
        stop_epoch: int = 120,
    ) -> None:
        self.scales = generate_scales(base_size, base_size_repeat)
        self.stop_epoch = stop_epoch
        self.epoch = 0

    def set_epoch(self, epoch: int) -> None:
        self.epoch = epoch

    def __call__(
        self,
        items: list[tuple[Tensor, DatasetTarget]],
    ) -> tuple[Tensor, list[DatasetTarget]]:
        images = torch.stack([item[0] for item in items])
        targets = [item[1] for item in items]
        if self.epoch < self.stop_epoch:
            size = random.choice(self.scales)
            images = functional.interpolate(
                images,
                size=(size, size),
                mode="bilinear",
                align_corners=False,
            )
        return images, targets
