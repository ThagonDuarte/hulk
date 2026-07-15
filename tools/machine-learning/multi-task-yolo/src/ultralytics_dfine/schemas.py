"""Task-head identities and annotation schemas for multi-task D-FINE."""

# ruff: noqa: TRY003

from collections.abc import Mapping
from dataclasses import asdict, dataclass
from enum import StrEnum
from types import MappingProxyType
from typing import Literal


class HeadId(StrEnum):
    """Stable identity for each independently supervised model output."""

    OBJECT = "object"
    PERSON_POSE = "person_pose"
    ROBOT_POSE = "robot_pose"
    FIELD_FEATURES = "field_features"


@dataclass(frozen=True)
class PoseSchemaConfig:
    """Complete semantics for one query-aligned pose head."""

    head_id: HeadId
    detector_classes: tuple[str, ...]
    keypoint_names: tuple[str, ...]
    flip_idx: tuple[int, ...]
    skeleton: tuple[tuple[int, int], ...]
    visibility_policy: Literal["coco", "binary"]
    oks_sigmas: tuple[float, ...]
    oks_uses_direct_k: bool = False

    def __post_init__(self) -> None:
        count = len(self.keypoint_names)
        if count == 0:
            raise ValueError("Pose schemas must contain keypoints")
        if len(self.flip_idx) != count or sorted(self.flip_idx) != list(
            range(count)
        ):
            raise ValueError("flip_idx must be a keypoint permutation")
        if len(self.oks_sigmas) != count:
            raise ValueError("OKS sigmas must match the keypoint count")
        if any(
            start < 0 or end < 0 or start >= count or end >= count
            for start, end in self.skeleton
        ):
            raise ValueError("Skeleton indices must reference keypoints")

    @property
    def keypoint_count(self) -> int:
        return len(self.keypoint_names)


@dataclass(frozen=True)
class PointSetSchemaConfig:
    """Semantics for one encoder-memory point-set head."""

    head_id: HeadId
    class_names: tuple[str, ...]
    num_queries: int
    implicit_background: bool = True

    def __post_init__(self) -> None:
        if not self.class_names:
            raise ValueError("Point-set schemas must contain classes")
        if self.num_queries <= 0:
            raise ValueError("Point-set query count must be positive")


COCO_KEYPOINT_NAMES = (
    "nose",
    "left_eye",
    "right_eye",
    "left_ear",
    "right_ear",
    "left_shoulder",
    "right_shoulder",
    "left_elbow",
    "right_elbow",
    "left_wrist",
    "right_wrist",
    "left_hip",
    "right_hip",
    "left_knee",
    "right_knee",
    "left_ankle",
    "right_ankle",
)
COCO_FLIP_IDX = (0, 2, 1, 4, 3, 6, 5, 8, 7, 10, 9, 12, 11, 14, 13, 16, 15)
COCO_SKELETON = (
    (15, 13),
    (13, 11),
    (16, 14),
    (14, 12),
    (11, 12),
    (5, 11),
    (6, 12),
    (5, 6),
    (5, 7),
    (6, 8),
    (7, 9),
    (8, 10),
    (1, 2),
    (0, 1),
    (0, 2),
    (1, 3),
    (2, 4),
    (3, 5),
    (4, 6),
)
COCO_OKS_SIGMAS = (
    0.026,
    0.025,
    0.025,
    0.035,
    0.035,
    0.079,
    0.079,
    0.072,
    0.072,
    0.062,
    0.062,
    0.107,
    0.107,
    0.087,
    0.087,
    0.089,
    0.089,
)

DHRP_KEYPOINT_NAMES = (
    "Nose",
    "Neck",
    "RShoulder",
    "RElbow",
    "RWrist",
    "LShoulder",
    "LElbow",
    "LWrist",
    "RHip",
    "RKnee",
    "RAnkle",
    "LHip",
    "LKnee",
    "LAnkle",
)
DHRP_JOINT_PARENTS = (1, -1, 1, 2, 3, 1, 5, 6, 1, 8, 9, 1, 11, 12)
DHRP_FLIP_IDX = (0, 1, 5, 6, 7, 2, 3, 4, 11, 12, 13, 8, 9, 10)
DHRP_SKELETON = tuple(
    (parent, index)
    for index, parent in enumerate(DHRP_JOINT_PARENTS)
    if parent >= 0
)
DHRP_OKS_K = (
    0.079,
    0.079,
    0.079,
    0.072,
    0.062,
    0.079,
    0.072,
    0.062,
    0.107,
    0.087,
    0.089,
    0.107,
    0.087,
    0.089,
)

FIELD_FEATURE_CLASSES = (
    "GoalPost",
    "LSpot",
    "TSpot",
    "PenaltySpot",
    "XSpot",
)

PERSON_POSE_SCHEMA = PoseSchemaConfig(
    head_id=HeadId.PERSON_POSE,
    detector_classes=("Person",),
    keypoint_names=COCO_KEYPOINT_NAMES,
    flip_idx=COCO_FLIP_IDX,
    skeleton=COCO_SKELETON,
    visibility_policy="coco",
    oks_sigmas=COCO_OKS_SIGMAS,
)
ROBOT_POSE_SCHEMA = PoseSchemaConfig(
    head_id=HeadId.ROBOT_POSE,
    detector_classes=("Robot",),
    keypoint_names=DHRP_KEYPOINT_NAMES,
    flip_idx=DHRP_FLIP_IDX,
    skeleton=DHRP_SKELETON,
    visibility_policy="binary",
    oks_sigmas=DHRP_OKS_K,
    oks_uses_direct_k=True,
)
FIELD_FEATURE_SCHEMA = PointSetSchemaConfig(
    head_id=HeadId.FIELD_FEATURES,
    class_names=FIELD_FEATURE_CLASSES,
    num_queries=300,
)

SchemaConfig = PoseSchemaConfig | PointSetSchemaConfig
SCHEMA_REGISTRY: Mapping[HeadId, SchemaConfig] = MappingProxyType(
    {
        HeadId.PERSON_POSE: PERSON_POSE_SCHEMA,
        HeadId.ROBOT_POSE: ROBOT_POSE_SCHEMA,
        HeadId.FIELD_FEATURES: FIELD_FEATURE_SCHEMA,
    }
)


def schema_to_dict(schema: SchemaConfig) -> dict[str, object]:
    """Convert a schema to checkpoint-safe primitive values."""
    values = asdict(schema)
    values["head_id"] = str(schema.head_id)
    return values
