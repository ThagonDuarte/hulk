# ruff: noqa: TRY003

import hashlib
import json
from dataclasses import asdict, dataclass, field
from typing import Any, Literal

DFINE_SOURCE_REVISION = "7fe2f8889f0b7b817f20c315b40fc15a4fb64ae6"
TRANSFORMERS_VERSION = "4.57.5"
DFINE_S_CHECKPOINT = "ustc-community/dfine-small-coco"
DFINE_S_CHECKPOINT_REVISION = "f79e65b5fbb33ceb9d3ebba042955d7410c608f8"


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
        if self.image_size <= 0 or self.image_size % 32:
            raise ValueError(
                "D-FINE image_size must be a positive multiple of 32"
            )
        if self.num_top_queries > self.num_queries * 1_000:
            raise ValueError("num_top_queries is unreasonably large")


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
    schemas: dict[str, dict[str, Any]]
    outputs: dict[str, dict[str, Any]]
    preprocessing: dict[str, Any]
    source_dfine_revision: str = DFINE_SOURCE_REVISION
    source_transformers_version: str = TRANSFORMERS_VERSION
    source_checkpoint: str = DFINE_S_CHECKPOINT
    source_checkpoint_revision: str = DFINE_S_CHECKPOINT_REVISION
    schema_version: int = 2
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
