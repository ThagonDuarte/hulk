"""YOLO-format keypoint datasets with global detector-class mapping."""

# ruff: noqa: C901, S311, TRY003

import hashlib
import json
import random
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, cast

import torch
import torchvision.transforms.v2.functional as transform_functional
from PIL import Image
from torch import Tensor
from torch.utils.data import Dataset
from torchvision.transforms import InterpolationMode
from torchvision.tv_tensors import BoundingBoxFormat

from ultralytics_dfine.data.dataset import (
    DatasetDefinition,
    DatasetTarget,
    _images_from_source,
    _label_path,
    load_dataset_yaml,
)
from ultralytics_dfine.schemas import HeadId

ROBOT_NEGATIVE_MANIFEST_VERSION = 2
ROBOT_NEGATIVE_MANIFEST_POLICY = "explicit-verified-robot-free-keypoint-v2"
ROBOT_NEGATIVE_POPULATION_DIGEST_ALGORITHM = (
    r"sha256(keypoint-robot-negative-population-v1\0 + split + \0 + "
    r"ordered(relative_path + \0 + image_sha256 + \n))"
)
ROBOT_NEGATIVE_ROLES = frozenset(
    {"train_negative", "primary_evaluation", "stress_evaluation"}
)


@dataclass(frozen=True)
class RobotNegativeEvidence:
    """Byte-locked approval for one complete keypoint dataset split."""

    role: str
    records: int
    population_sha256: str
    reviewed_records: int
    eligible_records: int
    excluded_records: int
    eligible_image_sha256: tuple[tuple[str, str], ...]
    excluded_image_sha256: tuple[tuple[str, str], ...]
    eligible_relative_paths: frozenset[str]
    excluded_relative_paths: frozenset[str]


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        while block := file.read(1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def _require_mapping(value: object, context: str) -> dict[str, object]:
    if not isinstance(value, dict):
        raise TypeError(f"{context} must be an object")
    return cast("dict[str, object]", value)


def _require_sha256(value: object, context: str) -> str:
    if (
        not isinstance(value, str)
        or len(value) != 64
        or any(character not in "0123456789abcdef" for character in value)
    ):
        raise ValueError(f"{context} must be a lowercase SHA-256")
    return value


def _normalized_relative_path(value: object, context: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{context} must be a non-empty relative path")
    path = Path(value)
    if path.is_absolute() or ".." in path.parts:
        raise ValueError(f"{context} must be a normalized relative path")
    normalized = path.as_posix()
    if normalized != value:
        raise ValueError(f"{context} must use normalized POSIX separators")
    return normalized


def _load_reviewed_entries(
    manifest_path: Path,
    raw_reference: object,
    *,
    split: str,
    decision: Literal["eligible", "excluded"],
) -> dict[str, str]:
    context = f"splits.{split}.{decision}"
    reference = _require_mapping(raw_reference, context)
    relative_reference = _normalized_relative_path(
        reference.get("path"),
        f"{context}.path",
    )
    reference_path = (manifest_path.parent / relative_reference).resolve()
    try:
        reference_path.relative_to(manifest_path.parent.resolve())
    except ValueError as error:
        raise ValueError(
            f"{context}.path must stay inside the manifest directory"
        ) from error
    expected_sha = _require_sha256(
        reference.get("sha256"),
        f"{context}.sha256",
    )
    if not reference_path.is_file():
        raise FileNotFoundError(reference_path)
    if _sha256_file(reference_path) != expected_sha:
        raise ValueError(f"{context} list SHA-256 mismatch")
    records = reference.get("records")
    if not isinstance(records, int) or records < 0:
        raise ValueError(f"{context}.records must be non-negative")

    entries: dict[str, str] = {}
    with reference_path.open(encoding="utf-8") as file:
        for line_number, line in enumerate(file, start=1):
            if not line.strip():
                raise ValueError(
                    f"{reference_path}:{line_number}: blank lines are invalid"
                )
            raw_entry = _require_mapping(
                json.loads(line),
                f"{reference_path}:{line_number}",
            )
            relative_path = _normalized_relative_path(
                raw_entry.get("relative_path"),
                f"{reference_path}:{line_number}.relative_path",
            )
            image_sha = _require_sha256(
                raw_entry.get("image_sha256"),
                f"{reference_path}:{line_number}.image_sha256",
            )
            evidence_field = "review" if decision == "eligible" else "reason"
            evidence = raw_entry.get(evidence_field)
            if not isinstance(evidence, str) or not evidence.strip():
                raise ValueError(
                    f"{reference_path}:{line_number}.{evidence_field} "
                    "must be non-empty"
                )
            if relative_path in entries:
                raise ValueError(
                    f"{reference_path}: duplicate path {relative_path!r}"
                )
            entries[relative_path] = image_sha
    if len(entries) != records:
        raise ValueError(f"{context} record count mismatch")
    return entries


def _load_robot_negative_manifest(
    path: str | Path,
    *,
    dataset_yaml: Path,
    split: str,
) -> tuple[RobotNegativeEvidence, str]:
    manifest_path = Path(path).expanduser().resolve()
    with manifest_path.open(encoding="utf-8") as file:
        values = _require_mapping(json.load(file), str(manifest_path))
    if values.get("version") != ROBOT_NEGATIVE_MANIFEST_VERSION:
        raise ValueError("Unsupported robot-negative manifest version")
    if values.get("policy") != ROBOT_NEGATIVE_MANIFEST_POLICY:
        raise ValueError("Unsupported robot-negative manifest policy")
    if values.get("population_digest_algorithm") != (
        ROBOT_NEGATIVE_POPULATION_DIGEST_ALGORITHM
    ):
        raise ValueError("Unsupported robot-negative population digest")
    expected_yaml_sha = _require_sha256(
        values.get("dataset_yaml_sha256"),
        "dataset_yaml_sha256",
    )
    if _sha256_file(dataset_yaml) != expected_yaml_sha:
        raise ValueError("Robot-negative dataset YAML SHA-256 mismatch")
    splits = _require_mapping(values.get("splits"), "splits")
    raw_evidence = _require_mapping(
        splits.get(split),
        f"splits.{split}",
    )
    role = raw_evidence.get("role")
    if role not in ROBOT_NEGATIVE_ROLES:
        raise ValueError(f"splits.{split}.role is unsupported")
    records = raw_evidence.get("records")
    if not isinstance(records, int) or records <= 0:
        raise ValueError(f"splits.{split}.records must be positive")

    eligible = _load_reviewed_entries(
        manifest_path,
        raw_evidence.get("eligible"),
        split=split,
        decision="eligible",
    )
    raw_excluded = raw_evidence.get("excluded")
    excluded = (
        {}
        if raw_excluded is None
        else _load_reviewed_entries(
            manifest_path,
            raw_excluded,
            split=split,
            decision="excluded",
        )
    )
    overlap = sorted(set(eligible) & set(excluded))
    if overlap:
        raise ValueError(
            "Robot-negative eligible/excluded lists overlap: "
            + ", ".join(overlap)
        )

    expected_counts = {
        "reviewed_records": len(eligible) + len(excluded),
        "eligible_records": len(eligible),
        "excluded_records": len(excluded),
    }
    observed_counts = {key: raw_evidence.get(key) for key in expected_counts}
    if observed_counts != expected_counts:
        raise ValueError(
            "Robot-negative reviewed decision counts are inconsistent"
        )
    if expected_counts["reviewed_records"] > records:
        raise ValueError("Robot-negative reviewed records exceed population")

    evidence = RobotNegativeEvidence(
        role=cast("str", role),
        records=records,
        population_sha256=_require_sha256(
            raw_evidence.get("population_sha256"),
            f"splits.{split}.population_sha256",
        ),
        reviewed_records=expected_counts["reviewed_records"],
        eligible_records=expected_counts["eligible_records"],
        excluded_records=expected_counts["excluded_records"],
        eligible_image_sha256=tuple(sorted(eligible.items())),
        excluded_image_sha256=tuple(sorted(excluded.items())),
        eligible_relative_paths=frozenset(eligible),
        excluded_relative_paths=frozenset(excluded),
    )
    return evidence, _sha256_file(manifest_path)


def _relative_image_path(image_path: Path, root: Path) -> str:
    try:
        return image_path.resolve().relative_to(root.resolve()).as_posix()
    except ValueError as error:
        raise ValueError(
            f"Robot-negative image is outside the dataset root: {image_path}"
        ) from error


def _robot_negative_population_sha256(
    images: Sequence[Path],
    *,
    root: Path,
    split: str,
) -> tuple[str, tuple[str, ...], dict[str, str]]:
    digest = hashlib.sha256()
    digest.update(b"keypoint-robot-negative-population-v1\0")
    digest.update(f"{split}\0".encode())
    relative_paths = []
    image_sha256 = {}
    for image_path in images:
        relative_path = _relative_image_path(image_path, root)
        relative_paths.append(relative_path)
        image_sha = _sha256_file(image_path)
        image_sha256[relative_path] = image_sha
        digest.update(f"{relative_path}\0{image_sha}\n".encode())
    return digest.hexdigest(), tuple(relative_paths), image_sha256


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
        image_size: int | tuple[int, int] = 640,
        horizontal_flip_probability: float = 0.5,
        augmentation_profile: Literal["basic", "field-v1"] = "basic",
        point_set: bool = False,
        point_label_ids: tuple[int, ...] | None = None,
        robot_negative_manifest: str | Path | None = None,
        robot_negative_roles: Sequence[str] = ("train_negative",),
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
        self.output_size = (
            (image_size, image_size)
            if isinstance(image_size, int)
            else image_size
        )
        if len(self.output_size) != 2 or min(self.output_size) <= 0:
            raise ValueError("image_size must contain positive height/width")
        self.image_size = image_size
        if not 0 <= horizontal_flip_probability <= 1:
            raise ValueError("horizontal_flip_probability must be in [0, 1]")
        self.horizontal_flip_probability = horizontal_flip_probability
        if augmentation_profile not in {"basic", "field-v1"}:
            raise ValueError("Unsupported keypoint augmentation profile")
        self.augmentation_profile = augmentation_profile
        self.point_set = point_set
        self.point_label_ids = point_label_ids
        if point_label_ids is not None and len(point_label_ids) != len(
            global_class_ids
        ):
            raise ValueError("Point label mapping and classes must match")
        if augmentation_profile == "field-v1" and not point_set:
            raise ValueError("field-v1 augmentation requires a point set")
        self.split = split
        sources = getattr(self.definition, split)
        if not sources:
            raise ValueError(f"Dataset has no '{split}' split")
        self.images = [
            image for source in sources for image in _images_from_source(source)
        ]
        if not self.images:
            raise ValueError(f"Dataset '{split}' split contains no images")
        selected_roles = frozenset(robot_negative_roles)
        if not selected_roles.issubset(ROBOT_NEGATIVE_ROLES):
            raise ValueError(
                "robot_negative_roles contains an unsupported role"
            )
        if (
            robot_negative_manifest is not None
            and schema_id != HeadId.PERSON_POSE
        ):
            raise ValueError(
                "Robot-negative evidence may only be used with Person pose"
            )
        self.robot_negative_manifest_sha256: str | None = None
        self.robot_negative_reviewed_records = 0
        self.robot_negative_verified_records = 0
        self.robot_negative_eligible_records = 0
        self.robot_negative_excluded_records = 0
        self.robot_negative_unreviewed_records = len(self.images)
        self._robot_negative_reviewed = tuple(False for _ in self.images)
        self._robot_negative_verified = tuple(False for _ in self.images)
        self._robot_negative_eligible = tuple(False for _ in self.images)
        self._robot_negative_excluded = tuple(False for _ in self.images)
        if robot_negative_manifest is not None:
            evidence, manifest_sha = _load_robot_negative_manifest(
                robot_negative_manifest,
                dataset_yaml=self.definition.yaml_path,
                split=split,
            )
            if len(self.images) != evidence.records:
                raise ValueError("Robot-negative record count mismatch")
            population_sha, relative_paths, population_image_sha = (
                _robot_negative_population_sha256(
                    self.images,
                    root=self.definition.root,
                    split=split,
                )
            )
            if population_sha != evidence.population_sha256:
                raise ValueError("Robot-negative population SHA-256 mismatch")
            reviewed_paths = (
                evidence.eligible_relative_paths
                | evidence.excluded_relative_paths
            )
            unknown_reviewed = sorted(reviewed_paths - set(relative_paths))
            if unknown_reviewed:
                raise ValueError(
                    "Robot-negative reviewed paths are absent from the "
                    "dataset: " + ", ".join(unknown_reviewed)
                )
            expected_reviewed_sha = dict(evidence.eligible_image_sha256)
            expected_reviewed_sha.update(evidence.excluded_image_sha256)
            mismatched_reviewed_sha = sorted(
                path
                for path, expected in expected_reviewed_sha.items()
                if population_image_sha[path] != expected
            )
            if mismatched_reviewed_sha:
                raise ValueError(
                    "Robot-negative reviewed image SHA-256 mismatch: "
                    + ", ".join(mismatched_reviewed_sha)
                )
            reviewed = tuple(path in reviewed_paths for path in relative_paths)
            verified = tuple(
                path in evidence.eligible_relative_paths
                for path in relative_paths
            )
            eligible = tuple(
                value and evidence.role in selected_roles for value in verified
            )
            excluded = tuple(
                path in evidence.excluded_relative_paths
                for path in relative_paths
            )
            if sum(reviewed) != evidence.reviewed_records:
                raise ValueError("Robot-negative reviewed records do not match")
            if sum(verified) != evidence.eligible_records:
                raise ValueError("Robot-negative eligible records do not match")
            if sum(excluded) != evidence.excluded_records:
                raise ValueError("Robot-negative excluded records do not match")
            self.robot_negative_manifest_sha256 = manifest_sha
            self.robot_negative_reviewed_records = sum(reviewed)
            self.robot_negative_verified_records = sum(verified)
            self.robot_negative_eligible_records = sum(eligible)
            self.robot_negative_excluded_records = sum(excluded)
            self.robot_negative_unreviewed_records = len(self.images) - sum(
                reviewed
            )
            self._robot_negative_reviewed = reviewed
            self._robot_negative_verified = verified
            self._robot_negative_eligible = eligible
            self._robot_negative_excluded = excluded

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
            "robot_negative_reviewed_records": (
                self.robot_negative_reviewed_records
            ),
            "robot_negative_verified_records": (
                self.robot_negative_verified_records
            ),
            "robot_negative_eligible_records": (
                self.robot_negative_eligible_records
            ),
            "robot_negative_excluded_records": (
                self.robot_negative_excluded_records
            ),
            "robot_negative_unreviewed_records": (
                self.robot_negative_unreviewed_records
            ),
        }

    def _load_labels(
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

    @staticmethod
    def _photometric_distortion(image: Tensor) -> Tensor:
        image = transform_functional.adjust_brightness(
            image,
            random.uniform(0.8, 1.2),
        )
        image = transform_functional.adjust_contrast(
            image,
            random.uniform(0.8, 1.2),
        )
        image = transform_functional.adjust_saturation(
            image,
            random.uniform(0.8, 1.2),
        )
        return transform_functional.adjust_hue(
            image,
            random.uniform(-0.05, 0.05),
        )

    @staticmethod
    def _field_affine(
        image: Tensor,
        local_labels: Tensor,
        boxes: Tensor,
        keypoints: Tensor,
    ) -> tuple[Tensor, Tensor, Tensor, Tensor]:
        height, width = image.shape[-2:]
        angle = random.uniform(-5, 5)
        translate = [
            random.uniform(-0.08, 0.08) * width,
            random.uniform(-0.08, 0.08) * height,
        ]
        scale = random.uniform(0.85, 1.15)
        shear = [random.uniform(-2, 2), random.uniform(-2, 2)]
        image = transform_functional.affine(
            image,
            angle,
            translate,
            scale,
            shear,
            interpolation=InterpolationMode.BILINEAR,
            fill=[0.0],
        )
        if boxes.numel() == 0:
            return image, local_labels, boxes, keypoints
        box_scale = boxes.new_tensor([width, height, width, height])
        pixel_boxes = transform_functional.affine_bounding_boxes(
            boxes * box_scale,
            BoundingBoxFormat.CXCYWH,
            (height, width),
            angle,
            translate,
            scale,
            shear,
            clamping_mode="hard",
        )
        pixel_points = keypoints[..., :2] * keypoints.new_tensor(
            [width, height]
        )
        pixel_points, _ = transform_functional.affine_keypoints(
            pixel_points,
            (height, width),
            angle,
            translate,
            scale,
            shear,
        )
        point_in_frame = (
            (pixel_points[..., 0] >= 0)
            & (pixel_points[..., 0] < width)
            & (pixel_points[..., 1] >= 0)
            & (pixel_points[..., 1] < height)
        ).all(dim=1)
        box_is_valid = (pixel_boxes[:, 2:] >= 1).all(dim=1)
        keep = point_in_frame & box_is_valid
        transformed_keypoints = keypoints.clone()
        transformed_keypoints[..., :2] = pixel_points / keypoints.new_tensor(
            [width, height]
        )
        return (
            image,
            local_labels[keep],
            pixel_boxes[keep] / box_scale,
            transformed_keypoints[keep],
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
        if self.split == "train" and self.augmentation_profile == "field-v1":
            if random.random() < 0.5:
                image_tensor = self._photometric_distortion(image_tensor)
            if random.random() < 0.5:
                image_tensor, local_labels, boxes, keypoints = (
                    self._field_affine(
                        image_tensor,
                        local_labels,
                        boxes,
                        keypoints,
                    )
                )
        image = transform_functional.resize(
            image_tensor,
            list(self.output_size),
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
            "robot_negative_reviewed": self._robot_negative_reviewed[index],
            "robot_negative_verified": self._robot_negative_verified[index],
            "robot_negative_eligible": self._robot_negative_eligible[index],
            "robot_negative_excluded": self._robot_negative_excluded[index],
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
