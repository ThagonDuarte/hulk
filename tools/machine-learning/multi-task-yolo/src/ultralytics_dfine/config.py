# ruff: noqa: TRY003

import hashlib
import json
import math
from dataclasses import asdict, dataclass, field
from typing import Any, Literal

DFINE_SOURCE_REVISION = "7fe2f8889f0b7b817f20c315b40fc15a4fb64ae6"
TRANSFORMERS_VERSION = "4.57.5"
DFINE_S_CHECKPOINT = "ustc-community/dfine-small-coco"
DFINE_S_CHECKPOINT_REVISION = "f79e65b5fbb33ceb9d3ebba042955d7410c608f8"


def _require_finite(**values: float) -> None:
    invalid = [
        name for name, value in values.items() if not math.isfinite(value)
    ]
    if invalid:
        raise ValueError(
            "Configuration values must be finite: " + ", ".join(invalid)
        )


@dataclass(frozen=True)
class PoseHeadConfig:
    """Architecture and deployment scoring for a query-aligned pose head."""

    variant: Literal["query_mlp"] = "query_mlp"
    refinement_dim: int = 64
    refinement_scale: float = 0.25
    feature_levels: int = 3
    detach_sampling_grid: bool = True
    visibility_score_alpha: float = 0.0

    def __post_init__(self) -> None:
        _require_finite(
            refinement_scale=self.refinement_scale,
            visibility_score_alpha=self.visibility_score_alpha,
        )
        if self.variant != "query_mlp":
            raise ValueError("Unsupported pose-head variant")
        if self.refinement_dim <= 0:
            raise ValueError("Pose refinement_dim must be positive")
        if self.refinement_scale <= 0:
            raise ValueError("Pose refinement_scale must be positive")
        if self.feature_levels <= 0:
            raise ValueError("Pose feature_levels must be positive")
        if self.visibility_score_alpha != 0.0:
            raise ValueError("visibility_score_alpha must be zero")


@dataclass(frozen=True)
class FieldHeadConfig:
    """Architecture for the field-feature point decoder."""

    variant: Literal["query_decoder", "spatial_refine"] = "query_decoder"
    refinement_dim: int = 64
    refinement_scale: float = 0.25
    feature_levels: int = 3
    detach_sampling_grid: bool = True

    def __post_init__(self) -> None:
        _require_finite(refinement_scale=self.refinement_scale)
        if self.variant not in {"query_decoder", "spatial_refine"}:
            raise ValueError("Unsupported field-head variant")
        if self.refinement_dim <= 0:
            raise ValueError("Field refinement_dim must be positive")
        if self.refinement_scale <= 0:
            raise ValueError("Field refinement_scale must be positive")
        if self.feature_levels <= 0:
            raise ValueError("Field feature_levels must be positive")


@dataclass(frozen=True)
class MultiTaskHeadConfig:
    """Serializable architecture choices for all auxiliary heads."""

    person_pose: PoseHeadConfig = field(default_factory=PoseHeadConfig)
    robot_pose: PoseHeadConfig = field(default_factory=PoseHeadConfig)
    field_features: FieldHeadConfig = field(default_factory=FieldHeadConfig)

    @classmethod
    def from_dict(cls, values: dict[str, Any]) -> "MultiTaskHeadConfig":
        """Restore nested dataclasses from checkpoint-safe dictionaries."""
        person = values.get("person_pose", {})
        robot = values.get("robot_pose", {})
        field_values = values.get("field_features", {})
        if not all(
            isinstance(value, dict) for value in (person, robot, field_values)
        ):
            raise TypeError("Invalid multi-task head configuration")
        return cls(
            person_pose=PoseHeadConfig(**person),
            robot_pose=PoseHeadConfig(**robot),
            field_features=FieldHeadConfig(**field_values),
        )


@dataclass(frozen=True)
class PoseLossConfig:
    """Loss settings for a pose task."""

    coordinate_space: Literal["image", "box"] = "image"
    smooth_l1_beta: float = 1.0
    coordinate_weight: float = 1.0
    oks_weight: float = 1.0
    visibility_weight: float = 1.0

    def __post_init__(self) -> None:
        _require_finite(
            smooth_l1_beta=self.smooth_l1_beta,
            coordinate_weight=self.coordinate_weight,
            oks_weight=self.oks_weight,
            visibility_weight=self.visibility_weight,
        )
        if self.coordinate_space not in {"image", "box"}:
            raise ValueError("Unsupported pose coordinate space")
        if self.smooth_l1_beta <= 0:
            raise ValueError("Pose smooth_l1_beta must be positive")
        if (
            min(
                self.coordinate_weight,
                self.oks_weight,
                self.visibility_weight,
            )
            < 0
        ):
            raise ValueError("Pose loss weights must be non-negative")


@dataclass(frozen=True)
class FieldLossConfig:
    """Loss settings for field-feature classification and localization."""

    classification_mode: Literal["binary", "strict_quality"] = "binary"
    class_weight: float = 1.0
    point_weight: float = 5.0
    focal_alpha: float = 0.25
    focal_gamma: float = 2.0
    quality_sigma: float = 0.1
    area_normalized_weight: float = 0.0
    area_normalized_beta: float = 0.1
    area_scale_floor: float = 0.01
    area_scale_cap: float = 0.1

    def __post_init__(self) -> None:
        _require_finite(
            class_weight=self.class_weight,
            point_weight=self.point_weight,
            focal_alpha=self.focal_alpha,
            focal_gamma=self.focal_gamma,
            quality_sigma=self.quality_sigma,
            area_normalized_weight=self.area_normalized_weight,
            area_normalized_beta=self.area_normalized_beta,
            area_scale_floor=self.area_scale_floor,
            area_scale_cap=self.area_scale_cap,
        )
        if self.classification_mode != "binary":
            raise ValueError("Only binary field classification is supported")
        if (
            min(
                self.class_weight,
                self.point_weight,
                self.area_normalized_weight,
            )
            < 0
        ):
            raise ValueError("Field loss weights must be non-negative")
        if not 0 <= self.focal_alpha <= 1:
            raise ValueError("Field focal_alpha must be in [0, 1]")
        if self.focal_gamma < 0:
            raise ValueError("Field focal_gamma must be non-negative")
        if self.quality_sigma <= 0:
            raise ValueError("Field quality_sigma must be positive")
        if self.area_normalized_beta <= 0:
            raise ValueError("Field area_normalized_beta must be positive")
        if self.area_scale_floor <= 0:
            raise ValueError("Field area_scale_floor must be positive")
        if self.area_scale_cap < self.area_scale_floor:
            raise ValueError(
                "Field area_scale_cap must be at least area_scale_floor"
            )
        if self.area_normalized_weight != 0:
            raise ValueError("Area-normalized field loss is not supported")


@dataclass(frozen=True)
class MultiTaskLossConfig:
    """Serializable loss choices for all auxiliary heads."""

    person_pose: PoseLossConfig = field(default_factory=PoseLossConfig)
    robot_pose: PoseLossConfig = field(default_factory=PoseLossConfig)
    field_features: FieldLossConfig = field(default_factory=FieldLossConfig)
    cross_pose_visibility_negative_weight: float = 0.0
    cross_pose_detector_negative_weight: float = 0.0

    def __post_init__(self) -> None:
        values = {
            "cross_pose_visibility_negative_weight": (
                self.cross_pose_visibility_negative_weight
            ),
            "cross_pose_detector_negative_weight": (
                self.cross_pose_detector_negative_weight
            ),
        }
        _require_finite(
            **values,
        )
        if any(value != 0 for value in values.values()):
            raise ValueError("Cross-pose negative training is not supported")

    @classmethod
    def from_dict(cls, values: dict[str, Any]) -> "MultiTaskLossConfig":
        """Restore nested dataclasses from checkpoint-safe dictionaries."""
        person = values.get("person_pose", {})
        robot = values.get("robot_pose", {})
        field_values = values.get("field_features", {})
        if not all(
            isinstance(value, dict) for value in (person, robot, field_values)
        ):
            raise TypeError("Invalid multi-task loss configuration")
        return cls(
            person_pose=PoseLossConfig(**person),
            robot_pose=PoseLossConfig(**robot),
            field_features=FieldLossConfig(**field_values),
            cross_pose_visibility_negative_weight=float(
                values.get("cross_pose_visibility_negative_weight", 0.0)
            ),
            cross_pose_detector_negative_weight=float(
                values.get("cross_pose_detector_negative_weight", 0.0)
            ),
        )


@dataclass(frozen=True)
class DFINEArchitectureConfig:
    variant: Literal["s"] = "s"
    image_size: int = 640
    num_queries: int = 300
    num_top_queries: int = 300
    decoder_layers: int = 3
    hidden_dim: int = 256
    feature_strides: tuple[int, ...] = (8, 16, 32)
    feature_channels: tuple[int, ...] = (256, 256, 256)
    reg_max: int = 32
    reg_scale: float = 4.0

    def __post_init__(self) -> None:
        _require_finite(reg_scale=self.reg_scale)
        if self.image_size <= 0 or self.image_size % 32:
            raise ValueError(
                "D-FINE image_size must be a positive multiple of 32"
            )
        if self.num_top_queries > self.num_queries * 1_000:
            raise ValueError("num_top_queries is unreasonably large")
        if self.reg_scale <= 0:
            raise ValueError("reg_scale must be positive")


@dataclass(frozen=True)
class DFINERecipeConfig:
    epochs: int = 132
    transition_epoch: int = 120
    total_batch_size: int = 32
    base_lr: float = 2e-4
    backbone_lr: float = 1e-4
    weight_decay: float = 1e-4
    warmup_steps: int = 500
    clip_max_norm: float = 0.1
    ema_decay: float = 0.9999
    ema_warmups: int = 1_000
    multiscale_repeat: int = 20
    amp: bool = True

    def __post_init__(self) -> None:
        _require_finite(
            base_lr=self.base_lr,
            backbone_lr=self.backbone_lr,
            weight_decay=self.weight_decay,
            clip_max_norm=self.clip_max_norm,
            ema_decay=self.ema_decay,
        )
        if self.epochs <= 0:
            raise ValueError("epochs must be positive")
        if not 0 < self.transition_epoch < self.epochs:
            raise ValueError("transition_epoch must be between 0 and epochs")
        if self.total_batch_size <= 0:
            raise ValueError("total_batch_size must be positive")

    @classmethod
    def for_epochs(
        cls,
        epochs: int,
        *,
        transition_epoch: int | None = None,
        total_batch_size: int = 32,
    ) -> "DFINERecipeConfig":
        transition = transition_epoch
        if transition is None:
            transition = 120 if epochs == 132 else max(1, epochs - 1)
        return cls(
            epochs=epochs,
            transition_epoch=transition,
            total_batch_size=total_batch_size,
        )


@dataclass
class DFINEStageState:
    stage: Literal[1, 2] = 1
    transition_epoch: int = 120
    stage1_best_fitness: float = float("-inf")
    stage2_best_fitness: float = float("-inf")
    stage1_best_checkpoint: str | None = None
    stage2_best_checkpoint: str | None = None
    augmentation_active: bool = True
    multiscale_active: bool = True
    ema_decay: float = 0.9999


@dataclass(frozen=True)
class DFINEManifest:
    architecture: str
    variant: str
    num_classes: int
    class_names: tuple[str, ...]
    architecture_config: dict[str, Any]
    preprocessing: dict[str, Any]
    postprocessing: dict[str, Any]
    source_dfine_revision: str = DFINE_SOURCE_REVISION
    source_transformers_version: str = TRANSFORMERS_VERSION
    source_checkpoint: str = DFINE_S_CHECKPOINT
    source_checkpoint_revision: str = DFINE_S_CHECKPOINT_REVISION
    schema_version: int = 1
    config_hash: str = field(init=False, default="")

    def __post_init__(self) -> None:
        values = asdict(self)
        values.pop("config_hash", None)
        encoded = json.dumps(values, sort_keys=True).encode()
        object.__setattr__(
            self, "config_hash", hashlib.sha256(encoded).hexdigest()
        )

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def build_manifest(
    architecture: DFINEArchitectureConfig,
    names: list[str] | tuple[str, ...],
) -> DFINEManifest:
    return DFINEManifest(
        architecture="dfine",
        variant=architecture.variant,
        num_classes=len(names),
        class_names=tuple(names),
        architecture_config=asdict(architecture),
        preprocessing={
            "resize": "scale_fill",
            "pixel_range": [0.0, 1.0],
            "input_size": [architecture.image_size, architecture.image_size],
        },
        postprocessing={
            "type": "topk_no_nms",
            "topk": architecture.num_top_queries,
            "output_contract": "normalized_cxcywh_score_class",
        },
    )


@dataclass(frozen=True)
class DFINEMultiTaskManifest:
    """Self-contained deployment contract for multi-task D-FINE."""

    architecture: str
    variant: str
    class_names: tuple[str, ...]
    architecture_config: dict[str, Any]
    head_config: dict[str, Any]
    loss_config: dict[str, Any]
    schemas: dict[str, dict[str, Any]]
    outputs: dict[str, dict[str, Any]]
    preprocessing: dict[str, Any]
    source_dfine_revision: str = DFINE_SOURCE_REVISION
    source_transformers_version: str = TRANSFORMERS_VERSION
    source_checkpoint: str = DFINE_S_CHECKPOINT
    source_checkpoint_revision: str = DFINE_S_CHECKPOINT_REVISION
    schema_version: int = 3
    config_hash: str = field(init=False, default="")

    def __post_init__(self) -> None:
        values = asdict(self)
        values.pop("config_hash", None)
        encoded = json.dumps(values, sort_keys=True).encode()
        object.__setattr__(
            self,
            "config_hash",
            hashlib.sha256(encoded).hexdigest(),
        )

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def build_multitask_manifest(
    architecture: DFINEArchitectureConfig,
    names: list[str] | tuple[str, ...],
    head_config: MultiTaskHeadConfig | None = None,
    loss_config: MultiTaskLossConfig | None = None,
) -> DFINEMultiTaskManifest:
    """Build the fixed four-output model and schema contract."""
    from ultralytics_dfine.schemas import SCHEMA_REGISTRY, schema_to_dict

    schemas = {
        str(head_id): schema_to_dict(schema)
        for head_id, schema in SCHEMA_REGISTRY.items()
    }
    return DFINEMultiTaskManifest(
        architecture="dfine-multitask",
        variant=architecture.variant,
        class_names=tuple(names),
        architecture_config=asdict(architecture),
        head_config=asdict(head_config or MultiTaskHeadConfig()),
        loss_config=asdict(loss_config or MultiTaskLossConfig()),
        schemas=schemas,
        outputs={
            "object_output": {
                "shape": ["batch", architecture.num_top_queries, 6],
                "layout": "pixel_xyxy_score_global_class",
            },
            "person_pose_output": {
                "shape": ["batch", architecture.num_top_queries, 57],
                "layout": "pixel_xyxy_score_local_class_coco17_xyv",
            },
            "robot_pose_output": {
                "shape": ["batch", architecture.num_top_queries, 14, 3],
                "layout": "object_aligned_dhrp14_pixel_xyv",
            },
            "field_feature_output": {
                "shape": ["batch", 300, 4],
                "layout": "pixel_xy_score_field_class",
            },
        },
        preprocessing={
            "resize": "scale_fill",
            "pixel_range": [0.0, 1.0],
            "input_size": [architecture.image_size, architecture.image_size],
        },
    )
