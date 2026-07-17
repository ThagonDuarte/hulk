"""Deterministic training and rendered validation for multi-task D-FINE."""

# ruff: noqa: C901, S311, TRY003

import json
import math
import os
import random
import time
from collections.abc import Callable, Iterator, Mapping
from contextlib import suppress
from copy import deepcopy
from dataclasses import asdict, dataclass, field
from datetime import timedelta
from functools import partial
from pathlib import Path
from typing import Any, Literal, cast

import torch
import torch.distributed as dist
import torch.nn as nn
from torch import Tensor
from torch.nn.parallel import DistributedDataParallel
from torch.utils.data import DataLoader

from ultralytics_dfine.engine.multitask_validator import MultiTaskValidator
from ultralytics_dfine.loss import MultiTaskCriterion
from ultralytics_dfine.loss.matcher import Target
from ultralytics_dfine.nn import DFINEMultiTaskModel
from ultralytics_dfine.schemas import HeadId

Batch = tuple[Tensor, list[dict[str, object]]]


def multitask_collate(items: list[tuple[Tensor, dict[str, object]]]) -> Batch:
    return torch.stack([image for image, _ in items]), [
        target for _, target in items
    ]


def _move_targets(
    targets: list[dict[str, object]],
    device: torch.device,
) -> list[Target]:
    moved = []
    for target in targets:
        moved.append(
            {
                key: value.to(device, non_blocking=True)
                if isinstance(value, Tensor)
                else value
                for key, value in target.items()
            }
        )
    return cast("list[Target]", moved)


@dataclass(frozen=True)
class MultiTaskTrainingConfig:
    output_dir: Path
    epochs: int = 1
    steps_per_epoch: int | None = None
    batch_size_per_rank: int = 1
    validation_batch_size: int = 1
    learning_rate: float = 1e-4
    device: str = "cpu"
    seed: int = 0
    stage_name: str = "all"
    sampling_weights: Mapping[HeadId, float] = field(default_factory=dict)
    trainable_roles: tuple[str, ...] = ("all",)
    role_learning_rates: Mapping[str, float] = field(default_factory=dict)
    run_name: str = "dfine-multitask"
    wandb_project: str = "multi-task-yolo-dfine"
    wandb_group: str | None = None
    wandb_mode: Literal["online", "offline", "disabled"] = "disabled"
    wandb_log_interval: int = 20
    wandb_log_checkpoints: bool = True
    save_named_best_checkpoints: bool = True
    max_validation_batches: int | None = None
    render_object_confidence: float = 0.25
    render_person_confidence: float = 0.5
    render_robot_confidence: float = 0.5
    render_field_confidence: float = 0.35
    render_keypoint_confidence: float = 0.5
    render_max_detections: int = 20
    weight_decay: float = 1e-4
    warmup_steps: int = 500
    learning_rate_schedule: Literal["constant", "cosine"] = "constant"
    minimum_learning_rate_ratio: float = 0.1
    clip_max_norm: float = 0.1
    ema_decay: float = 0.9999
    ema_warmups: int = 1_000
    amp: bool = False
    validation_interval: int = 1
    freeze_frozen_bn_stats: bool = False
    freeze_all_bn_stats: bool = False
    sync_batch_norm: bool = False
    deterministic: bool = True
    strict_deterministic: bool = False
    sdpa_backend: Literal["auto", "math"] = "auto"
    ddp_find_unused_parameters: bool = True
    allow_existing_output: bool = False
    dataset_audits: Mapping[str, Mapping[str, int]] = field(
        default_factory=dict
    )
    dataset_fingerprints: Mapping[str, Mapping[str, object]] = field(
        default_factory=dict
    )
    provenance: Mapping[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        values = {
            "learning_rate": self.learning_rate,
            "render_object_confidence": self.render_object_confidence,
            "render_person_confidence": self.render_person_confidence,
            "render_robot_confidence": self.render_robot_confidence,
            "render_field_confidence": self.render_field_confidence,
            "render_keypoint_confidence": self.render_keypoint_confidence,
            "weight_decay": self.weight_decay,
            "minimum_learning_rate_ratio": self.minimum_learning_rate_ratio,
            "clip_max_norm": self.clip_max_norm,
            "ema_decay": self.ema_decay,
            **{
                f"sampling_weights.{task}": value
                for task, value in self.sampling_weights.items()
            },
            **{
                f"role_learning_rates.{role}": value
                for role, value in self.role_learning_rates.items()
            },
        }
        invalid = [
            name for name, value in values.items() if not math.isfinite(value)
        ]
        if invalid:
            raise ValueError(
                "Training configuration values must be finite: "
                + ", ".join(invalid)
            )


_ROLE_PREFIXES = {
    "person_head": ("person_pose_head.", "shared_pose_refiner."),
    "robot_head": ("robot_pose_head.",),
    "field_head": ("field_feature_head.",),
    "classifiers": ("detector.core.model.decoder.class_embed.",),
    "decoder": (
        "detector.core.bbox_embed.",
        "detector.core.model.decoder.",
    ),
    "proposal": (
        "detector.core.model.enc_score_head.",
        "detector.core.model.enc_bbox_head.",
        "detector.core.model.denoising_class_embed.",
    ),
    "encoder_last": ("detector.core.model.encoder.pan_blocks.1.",),
    "backbone_last": ("detector.core.model.backbone.model.encoder.stages.3.",),
}

_HEAD_ROLES = ("person_head", "robot_head", "field_head")

STAGE_TRAINABLE_ROLES = {
    1: ("heads", "classifiers", "proposal"),
    2: ("heads", "classifiers", "proposal", "decoder"),
    3: (
        "heads",
        "classifiers",
        "proposal",
        "decoder",
        "encoder_last",
        "backbone_last",
    ),
}

TRAINABLE_PROFILES = {
    "stage1": STAGE_TRAINABLE_ROLES[1],
    "stage2": STAGE_TRAINABLE_ROLES[2],
    "stage3": STAGE_TRAINABLE_ROLES[3],
    "object_decoder_only": ("classifiers", "proposal", "decoder"),
    "pose_aligned_decoder_no_classifier": ("proposal", "decoder"),
    "pose_aligned_decoder": (
        "classifiers",
        "proposal",
        "decoder",
    ),
    "cross_negative_classifier_only": ("classifiers",),
    "field_head_only": ("field_head",),
    "pose_head_only": ("person_head", "robot_head"),
}

_EXACT_RESUME_CONFIG_FIELDS = (
    "steps_per_epoch",
    "batch_size_per_rank",
    "learning_rate",
    "device",
    "seed",
    "stage_name",
    "sampling_weights",
    "trainable_roles",
    "role_learning_rates",
    "weight_decay",
    "warmup_steps",
    "learning_rate_schedule",
    "minimum_learning_rate_ratio",
    "clip_max_norm",
    "ema_decay",
    "ema_warmups",
    "amp",
    "freeze_frozen_bn_stats",
    "freeze_all_bn_stats",
    "sync_batch_norm",
    "deterministic",
    "strict_deterministic",
    "sdpa_backend",
    "ddp_find_unused_parameters",
    "dataset_audits",
    "dataset_fingerprints",
    "train_loader_execution",
)

_EXACT_RESUME_PROVENANCE_FIELDS = (
    "source_code_sha256",
    "head_config",
    "loss_config",
    "train_image_size",
    "validation_image_size",
    "field_augmentation_profile",
    "object_data_sha256",
    "person_data_sha256",
    "field_data_sha256",
    "coco_robot_negative_manifest_sha256",
    "dhrp_person_negative_manifest_sha256",
)


def stage_training_config(
    stage: int,
    *,
    output_dir: Path,
    epochs: int,
    device: str = "cpu",
    learning_rate: float = 1e-4,
    sampling_weights: Mapping[HeadId, float] | None = None,
    trainable_profile: str | None = None,
) -> MultiTaskTrainingConfig:
    """Build one independently resumable stage configuration."""
    if stage not in STAGE_TRAINABLE_ROLES:
        raise ValueError("Training stage must be 1, 2, or 3")
    profile = trainable_profile or f"stage{stage}"
    roles = TRAINABLE_PROFILES.get(profile)
    if roles is None:
        raise ValueError(f"Unknown trainable profile: {profile}")
    role_learning_rates = {
        "heads": learning_rate,
        "person_head": learning_rate,
        "robot_head": learning_rate,
        "field_head": learning_rate,
        "classifiers": learning_rate,
        "proposal": learning_rate,
        "decoder": learning_rate * 0.5,
        "encoder_last": learning_rate * 0.1,
        "backbone_last": learning_rate * 0.05,
    }
    return MultiTaskTrainingConfig(
        output_dir=output_dir,
        epochs=epochs,
        learning_rate=learning_rate,
        device=device,
        stage_name=(f"stage_{stage}" if trainable_profile is None else profile),
        sampling_weights=sampling_weights or {},
        trainable_roles=roles,
        role_learning_rates=role_learning_rates,
    )


def _parameter_role(name: str) -> str | None:
    for role, prefixes in _ROLE_PREFIXES.items():
        if name.startswith(prefixes):
            return role
    return None


def _json_safe(value: object) -> object:
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


def _validation_loaders_for_rank(
    loaders: Mapping[HeadId, DataLoader],
    *,
    rank: int,
    world_size: int,
) -> dict[HeadId, DataLoader]:
    """Assign whole validation tasks to ranks in stable schema order."""
    if world_size < 1 or rank < 0 or rank >= world_size:
        raise ValueError("Invalid distributed validation rank")
    tasks = tuple(task for task in HeadId if task in loaders)
    return {
        task: loaders[task]
        for index, task in enumerate(tasks)
        if index % world_size == rank
    }


def _merge_validation_shards(
    shards: list[Mapping[str, object]],
) -> dict[str, object]:
    """Merge disjoint task metrics and renders in deterministic order."""
    metrics: dict[str, object] = {}
    renders: dict[str, str] = {}
    for shard in shards:
        raw_renders = shard.get("renders", {})
        if not isinstance(raw_renders, Mapping):
            raise TypeError("Validation renders must be a mapping")
        for name, path in raw_renders.items():
            key = str(name)
            if key in renders:
                raise ValueError(f"Duplicate validation render: {key}")
            renders[key] = str(path)
        for name, value in shard.items():
            if name == "renders":
                continue
            if name in metrics:
                raise ValueError(f"Duplicate validation metric: {name}")
            metrics[name] = value
    return {
        "renders": dict(sorted(renders.items())),
        **dict(sorted(metrics.items())),
    }


def _merge_gathered_validation_results(
    results: list[Mapping[str, object]],
) -> dict[str, object]:
    shards = []
    for result in results:
        validation = result.get("validation")
        if not isinstance(validation, Mapping):
            raise TypeError("Distributed validation result is invalid")
        shards.append(validation)
    return _merge_validation_shards(shards)


@torch.no_grad()
def _broadcast_module_state(module: nn.Module, *, source: int = 0) -> None:
    """Make distributed validation use exactly the source rank's EMA state."""
    for tensors in (module.parameters(), module.buffers()):
        for tensor in tensors:
            dist.broadcast(tensor, src=source)


class _ModelEMA:
    def __init__(
        self,
        model: nn.Module,
        *,
        decay: float,
        warmups: int,
    ) -> None:
        self.module = deepcopy(model).eval()
        self.module.requires_grad_(requires_grad=False)
        self.trainable_names = {
            name
            for name, parameter in model.named_parameters()
            if parameter.requires_grad
        }
        self.decay = decay
        self.warmups = warmups
        self.updates = 0

    def _current_decay(self) -> float:
        if self.warmups == 0:
            return self.decay
        return self.decay * (1 - math.exp(-self.updates / self.warmups))

    @torch.no_grad()
    def update(self, model: nn.Module) -> None:
        self.updates += 1
        decay = self._current_decay()
        source = model.state_dict()
        for name, value in self.module.state_dict().items():
            current = source[name].detach()
            if name in self.trainable_names and value.is_floating_point():
                value.mul_(decay).add_(current, alpha=1 - decay)
            else:
                value.copy_(current)

    def state_dict(self) -> dict[str, object]:
        return {
            "module": self.module.state_dict(),
            "decay": self.decay,
            "warmups": self.warmups,
            "updates": self.updates,
        }

    def load_state_dict(self, state: Mapping[str, object]) -> None:
        module = state.get("module")
        if not isinstance(module, dict):
            raise TypeError("EMA checkpoint is missing module weights")
        self.module.load_state_dict(module, strict=True)
        decay = state.get("decay", self.decay)
        warmups = state.get("warmups", self.warmups)
        updates = state.get("updates", 0)
        if not isinstance(decay, (float, int)):
            raise TypeError("EMA decay must be numeric")
        if not isinstance(warmups, int) or not isinstance(updates, int):
            raise TypeError("EMA counters must be integers")
        self.decay = float(decay)
        self.warmups = warmups
        self.updates = updates


class MultiTaskTrainer:
    """Train homogeneous task batches and validate all deployment outputs."""

    def __init__(
        self,
        model: DFINEMultiTaskModel,
        train_loaders: Mapping[HeadId, DataLoader],
        validation_loader: DataLoader | Mapping[HeadId, DataLoader],
        config: MultiTaskTrainingConfig,
    ) -> None:
        if not train_loaders:
            raise ValueError("At least one training loader is required")
        if config.validation_interval < 1:
            raise ValueError("Validation interval must be positive")
        if not 0 <= config.minimum_learning_rate_ratio <= 1:
            raise ValueError(
                "Minimum learning-rate ratio must be between zero and one"
            )
        if config.strict_deterministic and not config.deterministic:
            raise ValueError(
                "Strict deterministic algorithms require deterministic mode"
            )
        if config.sdpa_backend not in {"auto", "math"}:
            raise ValueError(f"Unknown SDPA backend: {config.sdpa_backend}")
        if (
            config.strict_deterministic
            and config.device.startswith("cuda")
            and os.environ.get("CUBLAS_WORKSPACE_CONFIG")
            not in {":4096:8", ":16:8"}
        ):
            raise ValueError(
                "Strict deterministic CUDA training requires "
                "CUBLAS_WORKSPACE_CONFIG=:4096:8 (or :16:8) to be set "
                "before starting torchrun"
            )
        self.world_size = int(os.environ.get("WORLD_SIZE", "1"))
        self.rank = int(os.environ.get("RANK", "0"))
        self.local_rank = int(os.environ.get("LOCAL_RANK", "0"))
        self.distributed = self.world_size > 1
        self.config = config
        self._seed_runtime(config.seed + self.rank)
        self.owns_process_group = False
        if self.distributed and not dist.is_initialized():
            backend = (
                "nccl"
                if config.device.startswith("cuda")
                and torch.cuda.is_available()
                else "gloo"
            )
            dist.init_process_group(
                backend=backend,
                timeout=timedelta(hours=2),
            )
            self.owns_process_group = True
        if self.distributed and config.device.startswith("cuda"):
            torch.cuda.set_device(self.local_rank)
            self.device = torch.device("cuda", self.local_rank)
        else:
            self.device = torch.device(config.device)
        if config.sync_batch_norm:
            if not self.distributed or self.device.type != "cuda":
                raise ValueError(
                    "SyncBatchNorm requires distributed CUDA training"
                )
            model = cast(
                "DFINEMultiTaskModel",
                nn.SyncBatchNorm.convert_sync_batchnorm(model),
            )
        self.model = model.to(self.device)
        self.train_loaders = dict(train_loaders)
        self.validation_loaders = (
            dict(validation_loader)
            if isinstance(validation_loader, Mapping)
            else {HeadId.OBJECT: validation_loader}
        )
        self.global_step = 0
        self.scheduler_step = 0
        self.start_epoch = 0
        self.loader_cycles = dict.fromkeys(self.train_loaders, 0)
        self.wandb_run: Any | None = None
        self.wandb_run_id: str | None = None
        self.pending_validation_epoch: int | None = None
        self.pending_train_losses: dict[str, float] = {}
        self.last_epoch_diagnostics: dict[str, float] = {}
        self.best_fitness = float("-inf")
        self.best_scores = {
            "object_field": float("-inf"),
            "object": float("-inf"),
            "field": float("-inf"),
            "strict_field": float("-inf"),
        }
        self._resume_mode: Literal["exact", "branch"] | None = None
        self._resume_path: Path | None = None
        self._runtime_states: list[dict[str, object]] = []
        self._configure_trainable_parameters()
        self._validate_ddp_unused_parameter_policy()
        self.training_model: nn.Module = self.model
        if self.distributed:
            device_ids = (
                [self.local_rank] if self.device.type == "cuda" else None
            )
            self.training_model = DistributedDataParallel(
                self.model,
                device_ids=device_ids,
                find_unused_parameters=config.ddp_find_unused_parameters,
            )
        self.criterion: nn.Module = MultiTaskCriterion(
            self.model.detector.nc,
            loss_config=getattr(self.model, "loss_config", None),
        ).to(self.device)
        self.optimizer = self._build_optimizer()
        self.amp_enabled = config.amp and self.device.type == "cuda"
        self.scaler = torch.GradScaler("cuda", enabled=self.amp_enabled)
        self.ema = _ModelEMA(
            self.model,
            decay=config.ema_decay,
            warmups=config.ema_warmups,
        )

    def _seed_runtime(self, seed: int) -> None:
        random.seed(seed)
        torch.manual_seed(seed)
        if torch.cuda.is_available():
            torch.cuda.manual_seed_all(seed)
        math_sdpa = self.config.sdpa_backend == "math"
        torch.backends.cuda.enable_flash_sdp(not math_sdpa)
        torch.backends.cuda.enable_mem_efficient_sdp(not math_sdpa)
        torch.backends.cuda.enable_cudnn_sdp(not math_sdpa)
        torch.backends.cuda.enable_math_sdp(enabled=True)
        if self.config.deterministic:
            torch.backends.cudnn.benchmark = False
            torch.backends.cudnn.deterministic = True
            torch.use_deterministic_algorithms(
                mode=True,
                warn_only=not self.config.strict_deterministic,
            )

    def _local_runtime_state(self) -> dict[str, object]:
        generators = {}
        for task, loader in self.train_loaders.items():
            generator = getattr(loader, "generator", None)
            if isinstance(generator, torch.Generator):
                generators[str(task)] = generator.get_state()
        state: dict[str, object] = {
            "python": random.getstate(),
            "torch": torch.random.get_rng_state(),
            "loader_generators": generators,
        }
        if self.device.type == "cuda":
            state["cuda"] = torch.cuda.get_rng_state(self.device)
        return state

    def _capture_runtime_states(self) -> None:
        local = self._local_runtime_state()
        if self.distributed:
            gathered: list[dict[str, object] | None] = [
                None for _ in range(self.world_size)
            ]
            dist.all_gather_object(gathered, local)
            if any(state is None for state in gathered):
                raise RuntimeError("Failed to gather distributed RNG state")
            self._runtime_states = cast("list[dict[str, object]]", gathered)
        else:
            self._runtime_states = [local]

    def _refresh_local_runtime_state(self) -> None:
        local = self._local_runtime_state()
        if not self._runtime_states:
            self._runtime_states = [local]
            return
        if self.rank < len(self._runtime_states):
            self._runtime_states[self.rank] = local

    def _restore_runtime_state(
        self,
        checkpoint: Mapping[str, object],
    ) -> None:
        raw_states = checkpoint.get("runtime_states")
        if not isinstance(raw_states, list) or not raw_states:
            return
        if self._resume_mode == "exact" and len(raw_states) != self.world_size:
            raise ValueError(
                "Exact resume requires the checkpoint's original world size"
            )
        raw_state = raw_states[min(self.rank, len(raw_states) - 1)]
        if not isinstance(raw_state, dict):
            raise TypeError("Checkpoint RNG state is invalid")
        python_state = raw_state.get("python")
        torch_state = raw_state.get("torch")
        if isinstance(python_state, tuple):
            random.setstate(python_state)
        if isinstance(torch_state, Tensor):
            torch.random.set_rng_state(torch_state.cpu())
        cuda_state = raw_state.get("cuda")
        if self.device.type == "cuda" and isinstance(cuda_state, Tensor):
            torch.cuda.set_rng_state(cuda_state.cpu(), self.device)
        loader_states = raw_state.get("loader_generators", {})
        if isinstance(loader_states, dict):
            for task, loader in self.train_loaders.items():
                generator = getattr(loader, "generator", None)
                state = loader_states.get(str(task))
                if isinstance(generator, torch.Generator) and isinstance(
                    state,
                    Tensor,
                ):
                    generator.set_state(state.cpu())

    def _prepare_output_directory(self) -> None:
        if self.rank != 0:
            return
        output_dir = self.config.output_dir
        existing = list(output_dir.iterdir()) if output_dir.is_dir() else []
        exact_in_place = (
            self._resume_mode == "exact"
            and self._resume_path is not None
            and self._resume_path.resolve().parent == output_dir.resolve()
        )
        if (
            existing
            and not exact_in_place
            and not self.config.allow_existing_output
        ):
            raise FileExistsError(
                f"Refusing non-empty training output directory: {output_dir}"
            )
        output_dir.mkdir(parents=True, exist_ok=True)
        config_values = _json_safe(self._training_config_payload())
        (output_dir / "run_config.json").write_text(
            json.dumps(config_values, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    def _append_local_metrics(
        self,
        event: str,
        values: Mapping[str, object],
    ) -> None:
        if self.rank != 0:
            return
        record = {
            "event": event,
            "time_unix": time.time(),
            **values,
        }
        with (self.config.output_dir / "metrics.jsonl").open(
            "a",
            encoding="utf-8",
        ) as file:
            file.write(json.dumps(_json_safe(record), sort_keys=True) + "\n")

    def _initialize_wandb(self) -> None:
        if self.rank != 0 or self.config.wandb_mode == "disabled":
            return
        import wandb

        training_config = asdict(self.config)
        training_config["output_dir"] = str(self.config.output_dir)
        training_config["sampling_weights"] = {
            str(task): value
            for task, value in self.config.sampling_weights.items()
        }
        values = {
            "training": training_config,
            "world_size": self.world_size,
            "global_batch_size": (
                self.config.batch_size_per_rank * self.world_size
            ),
            "planned_optimizer_updates": (
                self.config.epochs * self.config.steps_per_epoch
                if self.config.steps_per_epoch is not None
                else None
            ),
            "model": {
                "classes": self.model.detector.names,
                "head_config": _json_safe(
                    asdict(self.model.head_config)
                    if hasattr(self.model, "head_config")
                    else {}
                ),
                "loss_config": _json_safe(
                    asdict(self.model.loss_config)
                    if hasattr(self.model, "loss_config")
                    else {}
                ),
                "schemas": {
                    str(head): asdict(schema)
                    for head, schema in self.model.schemas.items()
                },
            },
        }
        try:
            self.wandb_run = wandb.init(
                project=self.config.wandb_project,
                name=self.config.run_name,
                group=self.config.wandb_group,
                config=values,
                mode=self.config.wandb_mode,
                id=self.wandb_run_id,
                resume="allow" if self.wandb_run_id is not None else None,
            )
        except wandb.Error:
            self.wandb_run = wandb.init(
                project=self.config.wandb_project,
                name=self.config.run_name,
                group=self.config.wandb_group,
                config=values,
                mode="offline",
                id=self.wandb_run_id,
                resume="allow" if self.wandb_run_id is not None else None,
                reinit=True,
            )
            self.wandb_run.summary["online_upload_blocked"] = True
        self.wandb_run_id = self.wandb_run.id
        self.wandb_run.define_metric("global_step")
        self.wandb_run.define_metric("train/step/*", step_metric="global_step")
        self.wandb_run.define_metric("epoch")
        self.wandb_run.define_metric("train/epoch/*", step_metric="epoch")
        self.wandb_run.define_metric("validation/*", step_metric="epoch")

    def _log_training_step(
        self,
        *,
        task: HeadId,
        losses: Mapping[str, Tensor],
        total: Tensor,
        epoch: int,
        duration: float,
        raw_gradient_norm: float,
        post_clip_gradient_norm: float,
    ) -> None:
        if self.global_step % self.config.wandb_log_interval != 0:
            return
        values: dict[str, object] = {
            "global_step": self.global_step,
            "epoch": epoch,
            "train/step/task": str(task),
            "train/step/total_loss": float(total.detach()),
            "train/step/duration_seconds": duration,
            "train/step/gradient_norm": raw_gradient_norm,
            "train/step/gradient_norm_raw": raw_gradient_norm,
            "train/step/gradient_norm_post_clip": post_clip_gradient_norm,
            "train/step/gradient_was_clipped": float(
                raw_gradient_norm > self.config.clip_max_norm
            ),
        }
        head_prefix = {
            HeadId.PERSON_POSE: "person_",
            HeadId.ROBOT_POSE: "robot_",
            HeadId.FIELD_FEATURES: "field_",
        }.get(task)
        if head_prefix is not None:
            head_loss = sum(
                float(loss.detach())
                for name, loss in losses.items()
                if name.startswith(head_prefix)
                and not name.endswith("inactive")
            )
            values["train/step/head_loss"] = head_loss
            values["train/step/detection_loss"] = (
                float(total.detach()) - head_loss
            )
            values[f"train/step/{task}/head_total"] = head_loss
            values[f"train/step/{task}/detection_total"] = (
                float(total.detach()) - head_loss
            )
        values.update(
            {
                f"train/step/{task}/{name}": float(loss.detach())
                for name, loss in losses.items()
            }
        )
        values.update(
            {
                f"train/step/lr_{group.get('role', index)}": group["lr"]
                for index, group in enumerate(self.optimizer.param_groups)
            }
        )
        if self.device.type == "cuda":
            values["system/gpu_memory_allocated_bytes"] = (
                torch.cuda.memory_allocated(self.device)
            )
            values["system/gpu_memory_reserved_bytes"] = (
                torch.cuda.memory_reserved(self.device)
            )
        self._append_local_metrics("train_step", values)
        if self.wandb_run is not None:
            self.wandb_run.log(values)

    def _log_train_epoch(
        self,
        *,
        epoch: int,
        losses: Mapping[str, float],
        duration: float,
    ) -> None:
        values: dict[str, object] = {
            "epoch": epoch,
            "global_step": self.global_step,
            "stage": self.config.stage_name,
            "train/epoch/duration_seconds": duration,
        }
        values.update(
            {f"train/epoch/{name}": value for name, value in losses.items()}
        )
        values.update(
            {
                f"train/epoch/{name}": value
                for name, value in self.last_epoch_diagnostics.items()
            }
        )
        self._append_local_metrics("train_epoch", values)
        if self.wandb_run is not None:
            self.wandb_run.log(values)

    def _log_validation(
        self,
        *,
        epoch: int,
        validation: Mapping[str, object],
        checkpoint: Path,
        is_best: bool,
    ) -> None:
        values: dict[str, object] = {
            "epoch": epoch,
            "global_step": self.global_step,
            "stage": self.config.stage_name,
        }
        values.update(
            {
                f"validation/{name}": value
                for name, value in validation.items()
                if isinstance(value, (float, int))
            }
        )
        self._append_local_metrics("validation", values)
        if self.wandb_run is None:
            return
        import wandb

        renders = validation.get("renders", {})
        if isinstance(renders, Mapping):
            values["validation/renders"] = [
                wandb.Image(str(path), caption=str(name))
                for name, path in renders.items()
            ]
        self.wandb_run.log(values)
        if self.config.wandb_log_checkpoints:
            self._log_checkpoint_artifact(
                checkpoint,
                epoch=epoch,
                aliases=[
                    "latest",
                    self.config.stage_name,
                    *(["best"] if is_best else []),
                ],
            )

    def _log_checkpoint_artifact(
        self,
        checkpoint: Path,
        *,
        epoch: int,
        aliases: list[str],
    ) -> None:
        if self.wandb_run is None or not checkpoint.is_file():
            return
        import wandb

        artifact = wandb.Artifact(
            f"{self.config.run_name}-checkpoint",
            type="model",
            metadata={
                "epoch": epoch,
                "global_step": self.global_step,
                "stage": self.config.stage_name,
            },
        )
        artifact.add_file(str(checkpoint), name="last.pt")
        self.wandb_run.log_artifact(artifact, aliases=aliases)

    def _configure_trainable_parameters(self) -> None:
        roles = set(self.config.trainable_roles)
        unknown = roles - {*_ROLE_PREFIXES, "heads", "all"}
        if unknown:
            raise ValueError(f"Unknown trainable roles: {sorted(unknown)}")
        expanded_roles = (roles - {"heads"}) | (
            set(_HEAD_ROLES) if "heads" in roles else set()
        )
        matched_roles = {
            role
            for name, _ in self.model.named_parameters()
            if (role := _parameter_role(name)) is not None
        }
        missing = expanded_roles - matched_roles - {"all"}
        if missing:
            raise ValueError(
                f"Trainable roles matched no parameters: {sorted(missing)}"
            )
        for name, parameter in self.model.named_parameters():
            role = _parameter_role(name)
            parameter.requires_grad_(
                "all" in expanded_roles or role in expanded_roles
            )

    def _validate_ddp_unused_parameter_policy(self) -> None:
        if self.config.ddp_find_unused_parameters:
            return
        tasks = set(self.train_loaders)
        roles = set(self.config.trainable_roles)
        trainable = [
            name
            for name, parameter in self.model.named_parameters()
            if parameter.requires_grad
        ]
        invalid_trainable = [
            name for name in trainable if _parameter_role(name) != "field_head"
        ]
        if (
            tasks != {HeadId.FIELD_FEATURES}
            or roles != {"field_head"}
            or not trainable
            or invalid_trainable
        ):
            details = (
                f"tasks={sorted(map(str, tasks))}, "
                f"roles={sorted(roles)}, "
                f"non_field_trainable={invalid_trainable[:5]}"
            )
            raise ValueError(
                "Disabling DDP unused-parameter discovery is only supported "
                "when the sole training task is field_features and the sole "
                f"trainable role is field_head; {details}"
            )

    def _assert_all_trainable_gradients_present(self) -> None:
        if self.config.ddp_find_unused_parameters:
            return
        missing = [
            name
            for name, parameter in self.model.named_parameters()
            if parameter.requires_grad and parameter.grad is None
        ]
        if missing:
            raise RuntimeError(
                "DDP unused-parameter discovery is disabled, but trainable "
                "parameters were absent from the current field loss graph: "
                + ", ".join(missing[:10])
            )

    def _role_learning_rate(self, role: str) -> float:
        if role in self.config.role_learning_rates:
            return self.config.role_learning_rates[role]
        if role in _HEAD_ROLES and "heads" in self.config.role_learning_rates:
            return self.config.role_learning_rates["heads"]
        return self.config.learning_rate

    def _build_optimizer(self) -> torch.optim.AdamW:
        groups: dict[tuple[str, bool], list[Tensor]] = {}
        visited: set[int] = set()
        for name, parameter in self.model.named_parameters():
            if not parameter.requires_grad:
                continue
            role = _parameter_role(name) or "other"
            use_decay = (
                parameter.ndim > 1
                and not name.endswith(".bias")
                and "norm" not in name.lower()
                and ".bn" not in name.lower()
            )
            groups.setdefault((role, use_decay), []).append(parameter)
            if id(parameter) in visited:
                raise ValueError(
                    f"Parameter appears twice in optimizer: {name}"
                )
            visited.add(id(parameter))
        if not groups:
            raise ValueError("Training stage has no trainable parameters")
        parameter_groups = [
            {
                "params": parameters,
                "lr": self._role_learning_rate(role),
                "base_lr": self._role_learning_rate(role),
                "weight_decay": self.config.weight_decay if use_decay else 0.0,
                "role": role,
            }
            for (role, use_decay), parameters in groups.items()
        ]
        return torch.optim.AdamW(parameter_groups, betas=(0.9, 0.999))

    def _planned_optimizer_updates(self) -> int:
        steps = self.config.steps_per_epoch or sum(
            len(loader) for loader in self.train_loaders.values()
        )
        return steps * self.config.epochs

    def _learning_rate_scale(self) -> float:
        step = self.scheduler_step
        if self.config.warmup_steps > 0 and step < self.config.warmup_steps:
            return (step + 1) / self.config.warmup_steps
        if self.config.learning_rate_schedule == "constant":
            return 1.0
        decay_steps = self._planned_optimizer_updates() - (
            self.config.warmup_steps
        )
        progress = min(
            1.0,
            max(
                0.0,
                (step - self.config.warmup_steps) / max(decay_steps - 1, 1),
            ),
        )
        cosine = (1 + math.cos(math.pi * progress)) / 2
        minimum = self.config.minimum_learning_rate_ratio
        return minimum + (1 - minimum) * cosine

    def _set_learning_rates(self) -> None:
        scale = self._learning_rate_scale()
        for group in self.optimizer.param_groups:
            group["lr"] = float(group["base_lr"]) * scale

    def _task_schedule(self, epoch: int, steps: int) -> list[HeadId]:
        tasks = tuple(self.train_loaders)
        weights = [
            float(self.config.sampling_weights.get(task, 1.0)) for task in tasks
        ]
        if any(weight < 0 for weight in weights) or sum(weights) <= 0:
            raise ValueError(
                "Sampling weights must be non-negative and nonzero"
            )
        exact = [steps * weight / sum(weights) for weight in weights]
        counts = [int(value) for value in exact]
        remaining = steps - sum(counts)
        order = sorted(
            range(len(tasks)),
            key=lambda index: exact[index] - counts[index],
            reverse=True,
        )
        for index in order[:remaining]:
            counts[index] += 1
        schedule = [
            task
            for task, count in zip(tasks, counts, strict=True)
            for _ in range(count)
        ]
        random.Random(self.config.seed + epoch).shuffle(schedule)
        return schedule

    def _scheduled_batches(self, epoch: int) -> Iterator[tuple[HeadId, Batch]]:
        iterators = {
            task: iter(loader) for task, loader in self.train_loaders.items()
        }
        default_steps = sum(
            len(loader) for loader in self.train_loaders.values()
        )
        steps = self.config.steps_per_epoch or default_steps
        for task in self._task_schedule(epoch, steps):
            try:
                batch = next(iterators[task])
            except StopIteration:
                iterators[task] = iter(self.train_loaders[task])
                self.loader_cycles[task] += 1
                batch = next(iterators[task])
            yield task, batch

    def _enforce_batch_norm_stats_policy(self) -> None:
        specialized = self.config.stage_name in {
            "object_decoder_only",
            "pose_aligned_decoder_no_classifier",
            "pose_aligned_decoder",
            "cross_negative_classifier_only",
            "field_head_only",
            "pose_head_only",
        }
        freeze_frozen = self.config.freeze_frozen_bn_stats or specialized
        if not self.config.freeze_all_bn_stats and not freeze_frozen:
            return
        for module in self.model.modules():
            if not isinstance(module, nn.modules.batchnorm._BatchNorm):
                continue
            has_trainable_affine = any(
                parameter.requires_grad
                for parameter in module.parameters(recurse=False)
            )
            if self.config.freeze_all_bn_stats or not has_trainable_affine:
                module.eval()

    def train_epoch(self, epoch: int = 0) -> dict[str, float]:
        self.training_model.train()
        self._enforce_batch_norm_stats_policy()
        for loader in self.train_loaders.values():
            set_epoch = getattr(loader.sampler, "set_epoch", None)
            if callable(set_epoch):
                set_epoch(epoch)
            set_dataset_epoch = getattr(loader.dataset, "set_epoch", None)
            if callable(set_dataset_epoch):
                set_dataset_epoch(epoch)
        totals: dict[str, float] = {}
        counts: dict[str, int] = {}
        cross_negative_counts = torch.zeros(
            8,
            dtype=torch.float64,
            device=self.device,
        )
        for task, (images, targets) in self._scheduled_batches(epoch):
            if task == HeadId.PERSON_POSE:
                cross_negative_counts[0] += len(targets)
                cross_negative_counts[1] += sum(
                    target.get("robot_negative_reviewed") is True
                    for target in targets
                )
                cross_negative_counts[2] += sum(
                    target.get("robot_negative_verified") is True
                    for target in targets
                )
                cross_negative_counts[3] += sum(
                    target.get("robot_negative_eligible") is True
                    for target in targets
                )
                cross_negative_counts[4] += sum(
                    target.get("robot_negative_excluded") is True
                    for target in targets
                )
            elif task == HeadId.ROBOT_POSE:
                cross_negative_counts[5] += len(targets)
                cross_negative_counts[6] += sum(
                    target.get("person_negative_verified") is True
                    for target in targets
                )
                cross_negative_counts[7] += sum(
                    target.get("person_negative_eligible") is True
                    for target in targets
                )
            step_started = time.monotonic()
            images = images.to(self.device, non_blocking=True)
            moved_targets = _move_targets(targets, self.device)
            self._set_learning_rates()
            self.optimizer.zero_grad(set_to_none=True)
            with torch.autocast(
                device_type=self.device.type,
                enabled=self.amp_enabled,
            ):
                outputs = self.training_model(images, moved_targets, task)
            with torch.autocast(
                device_type=self.device.type,
                enabled=False,
            ):
                result = self.criterion(outputs, moved_targets, task)
                total = torch.stack(tuple(result.losses.values())).sum()
            if not torch.isfinite(total):
                raise FloatingPointError("Non-finite multi-task loss")
            self.scaler.scale(total).backward()
            self._assert_all_trainable_gradients_present()
            self.scaler.unscale_(self.optimizer)
            raw_gradient_norm = nn.utils.clip_grad_norm_(
                self.model.parameters(),
                self.config.clip_max_norm,
                error_if_nonfinite=True,
            )
            post_clip_gradient_norm = min(
                float(raw_gradient_norm),
                self.config.clip_max_norm,
            )
            self.scaler.step(self.optimizer)
            self.scaler.update()
            self.ema.update(self.model)
            self.global_step += 1
            self.scheduler_step += 1
            if self.rank == 0:
                self._log_training_step(
                    task=task,
                    losses=result.losses,
                    total=total,
                    epoch=epoch,
                    duration=time.monotonic() - step_started,
                    raw_gradient_norm=float(raw_gradient_norm),
                    post_clip_gradient_norm=post_clip_gradient_norm,
                )
            for name, loss in result.losses.items():
                totals[name] = totals.get(name, 0.0) + float(loss.detach())
                counts[name] = counts.get(name, 0) + 1
        averages = {
            name: value / counts[name] for name, value in totals.items()
        }
        if self.distributed:
            for name in sorted(averages):
                value = torch.tensor(averages[name], device=self.device)
                dist.all_reduce(value, op=dist.ReduceOp.SUM)
                averages[name] = float(value) / self.world_size
            dist.all_reduce(cross_negative_counts, op=dist.ReduceOp.SUM)
        (
            person_records,
            robot_negative_reviewed_records,
            robot_negative_verified_records,
            robot_negative_eligible_records,
            robot_negative_excluded_records,
            robot_records,
            person_negative_verified_records,
            person_negative_eligible_records,
        ) = (float(value) for value in cross_negative_counts.tolist())
        loss_config = getattr(self.model, "loss_config", None)
        person_robot_detector = float(
            getattr(loss_config, "person_batch_robot_detector_weight", 0.0)
        )
        robot_person_detector = float(
            getattr(loss_config, "robot_batch_person_detector_weight", 0.0)
        )
        person_robot_visibility = float(
            getattr(loss_config, "person_batch_robot_visibility_weight", 0.0)
        )
        robot_person_visibility = float(
            getattr(loss_config, "robot_batch_person_visibility_weight", 0.0)
        )
        self.last_epoch_diagnostics = {
            "cross_negative/person_records_seen": person_records,
            "cross_negative/robot_records_seen": robot_records,
            "cross_negative/person_robot_reviewed_seen": (
                robot_negative_reviewed_records
            ),
            "cross_negative/person_robot_excluded_seen": (
                robot_negative_excluded_records
            ),
            "cross_negative/person_robot_unreviewed_seen": (
                person_records - robot_negative_reviewed_records
            ),
            "cross_negative/person_robot_verified_seen": (
                robot_negative_verified_records
            ),
            "cross_negative/person_robot_eligible_seen": (
                robot_negative_eligible_records
            ),
            "cross_negative/person_robot_eligible_fraction": (
                robot_negative_eligible_records / person_records
                if person_records
                else 0.0
            ),
            "cross_negative/robot_person_verified_seen": (
                person_negative_verified_records
            ),
            "cross_negative/robot_person_eligible_seen": (
                person_negative_eligible_records
            ),
            "cross_negative/robot_person_eligible_fraction": (
                person_negative_eligible_records / robot_records
                if robot_records
                else 0.0
            ),
            "cross_negative/person_robot_detector_applied_records": (
                robot_negative_eligible_records
                if person_robot_detector > 0
                else 0.0
            ),
            "cross_negative/robot_person_detector_applied_records": (
                person_negative_eligible_records
                if robot_person_detector > 0
                else 0.0
            ),
            "cross_negative/person_robot_visibility_applied_records": (
                robot_negative_eligible_records
                if person_robot_visibility > 0
                else 0.0
            ),
            "cross_negative/robot_person_visibility_applied_records": (
                person_negative_eligible_records
                if robot_person_visibility > 0
                else 0.0
            ),
        }
        self._capture_runtime_states()
        return averages

    def _run_validation_loaders(
        self,
        loaders: Mapping[HeadId, DataLoader],
        render_dir: Path,
    ) -> dict[str, object]:
        validator = MultiTaskValidator(
            cast("DFINEMultiTaskModel", self.ema.module),
            loaders,
            device=self.device,
            render_object_confidence=self.config.render_object_confidence,
            render_person_confidence=self.config.render_person_confidence,
            render_robot_confidence=self.config.render_robot_confidence,
            render_field_confidence=self.config.render_field_confidence,
            render_keypoint_confidence=self.config.render_keypoint_confidence,
            render_max_detections=self.config.render_max_detections,
        )
        return cast(
            "dict[str, object]",
            validator.run(
                render_dir=render_dir,
                max_batches=self.config.max_validation_batches,
            ),
        )

    @torch.no_grad()
    def validate(self, epoch: int) -> dict[str, object]:
        if not self.distributed:
            return self._run_validation_loaders(
                self.validation_loaders,
                self.config.output_dir / "validation" / f"epoch_{epoch:03d}",
            )

        _broadcast_module_state(self.ema.module)
        loaders = _validation_loaders_for_rank(
            self.validation_loaders,
            rank=self.rank,
            world_size=self.world_size,
        )
        render_dir = (
            self.config.output_dir
            / "validation"
            / f"epoch_{epoch:03d}"
            / "shards"
            / f"rank-{self.rank:02d}"
        )
        local_error: BaseException | None = None
        try:
            local_validation = self._run_validation_loaders(
                loaders,
                render_dir,
            )
            local_result: dict[str, object] = {
                "rank": self.rank,
                "validation": local_validation,
            }
        except BaseException as error:
            local_error = error
            local_result = {
                "rank": self.rank,
                "error_type": type(error).__name__,
                "error": str(error),
            }

        gathered: list[dict[str, object] | None] = [
            None for _ in range(self.world_size)
        ]
        dist.all_gather_object(gathered, local_result)
        complete = [result for result in gathered if result is not None]
        if len(complete) != self.world_size:
            raise RuntimeError("A distributed validation rank is missing")
        failures = [result for result in complete if "validation" not in result]
        if failures:
            if local_error is not None:
                raise local_error
            details = "; ".join(
                f"rank {result['rank']}: "
                f"{result.get('error_type', 'Error')}: "
                f"{result.get('error', '')}"
                for result in failures
            )
            raise RuntimeError(f"Distributed validation failed: {details}")

        merge_error: BaseException | None = None
        payload: list[dict[str, object] | None] = [None]
        if self.rank == 0:
            try:
                payload[0] = {
                    "status": "ok",
                    "validation": _merge_gathered_validation_results(complete),
                }
            except BaseException as error:
                merge_error = error
                payload[0] = {
                    "status": "error",
                    "error_type": type(error).__name__,
                    "error": str(error),
                }
        dist.broadcast_object_list(payload, src=0)
        result = payload[0]
        if result is None:
            raise RuntimeError(
                "Rank zero did not broadcast distributed validation metrics"
            )
        if result.get("status") == "error":
            if merge_error is not None:
                raise merge_error
            raise RuntimeError(
                "Distributed validation merge failed: "
                f"{result.get('error_type', 'Error')}: "
                f"{result.get('error', '')}"
            )
        validation = result.get("validation")
        if not isinstance(validation, dict):
            raise TypeError("Distributed validation metrics are invalid")
        return cast("dict[str, object]", validation)

    def checkpoint_payload(self, epoch: int) -> dict[str, object]:
        if isinstance(self.model, DFINEMultiTaskModel):
            payload = self.model.checkpoint_payload()
        else:
            payload = {
                "format_version": 2,
                "architecture": "test-multitask",
                "model": self.model.state_dict(),
            }
        payload.update(
            {
                "epoch": epoch,
                "global_step": self.global_step,
                "scheduler_step": self.scheduler_step,
                "world_size": self.world_size,
                "optimizer": self.optimizer.state_dict(),
                "scaler": self.scaler.state_dict(),
                "ema": self.ema.state_dict(),
                "inference_model": self.ema.module.state_dict(),
                "best_fitness": self.best_fitness,
                "best_scores": self.best_scores,
                "stage": self.config.stage_name,
                "loader_cycles": {
                    str(task): count
                    for task, count in self.loader_cycles.items()
                },
                "wandb_run_id": self.wandb_run_id,
                "pending_validation_epoch": self.pending_validation_epoch,
                "pending_train_losses": self.pending_train_losses,
                "runtime_states": self._runtime_states
                or [self._local_runtime_state()],
                "training_config": self._training_config_payload(),
            }
        )
        return payload

    def _training_config_payload(self) -> dict[str, object]:
        return {
            **asdict(self.config),
            "output_dir": str(self.config.output_dir),
            "sampling_weights": {
                str(task): value
                for task, value in self.config.sampling_weights.items()
            },
            "train_loader_execution": {
                str(task): {
                    "num_workers": loader.num_workers,
                    "persistent_workers": loader.persistent_workers,
                    "prefetch_factor": loader.prefetch_factor,
                }
                for task, loader in self.train_loaders.items()
            },
        }

    def _validate_exact_resume_config(
        self,
        checkpoint: Mapping[str, object],
    ) -> None:
        saved = checkpoint.get("training_config")
        if not isinstance(saved, Mapping):
            raise TypeError(
                "Exact resume requires checkpoint training_config metadata"
            )
        saved_values = dict(saved)
        saved_values.setdefault("freeze_all_bn_stats", False)
        saved_values.setdefault("strict_deterministic", False)
        saved_values.setdefault("sdpa_backend", "auto")
        saved_values.setdefault("ddp_find_unused_parameters", True)
        current = self._training_config_payload()
        mismatches = []
        for name in _EXACT_RESUME_CONFIG_FIELDS:
            if name not in saved_values:
                mismatches.append(f"{name}: missing from checkpoint")
                continue
            saved_value = _json_safe(saved_values[name])
            current_value = _json_safe(current[name])
            if saved_value != current_value:
                mismatches.append(
                    f"{name}: checkpoint={saved_value!r}, "
                    f"current={current_value!r}"
                )

        saved_provenance = saved_values.get("provenance", {})
        current_provenance = current.get("provenance", {})
        if not isinstance(saved_provenance, Mapping) or not isinstance(
            current_provenance,
            Mapping,
        ):
            mismatches.append("provenance: invalid mapping")
        else:
            for name in _EXACT_RESUME_PROVENANCE_FIELDS:
                if (
                    name not in saved_provenance
                    and name not in current_provenance
                ):
                    continue
                saved_value = _json_safe(saved_provenance.get(name))
                current_value = _json_safe(current_provenance.get(name))
                if saved_value != current_value:
                    mismatches.append(
                        f"provenance.{name}: checkpoint={saved_value!r}, "
                        f"current={current_value!r}"
                    )

        persistent_tasks = []
        for source_name, execution in (
            ("checkpoint", saved_values.get("train_loader_execution")),
            ("current", current.get("train_loader_execution")),
        ):
            if not isinstance(execution, Mapping):
                continue
            persistent_tasks.extend(
                f"{source_name}:{task}"
                for task, settings in execution.items()
                if isinstance(settings, Mapping)
                and settings.get("persistent_workers") is True
            )
        if persistent_tasks:
            mismatches.append(
                "train_loader_execution: exact resume cannot restore "
                "persistent-worker RNG, dataset, and prefetch state for "
                + ", ".join(persistent_tasks)
            )

        saved_world_size = checkpoint.get("world_size")
        if saved_world_size != self.world_size:
            mismatches.append(
                f"world_size: checkpoint={saved_world_size!r}, "
                f"current={self.world_size!r}"
            )

        saved_epochs = saved_values.get("epochs")
        if not isinstance(saved_epochs, int):
            mismatches.append("epochs: missing or invalid in checkpoint")
        elif self.config.epochs < saved_epochs:
            mismatches.append(
                f"epochs: checkpoint={saved_epochs!r}, "
                f"current={self.config.epochs!r}; exact resume only permits "
                "extending the run"
            )
        elif (
            self.config.epochs > saved_epochs
            and self.config.learning_rate_schedule != "constant"
        ):
            mismatches.append(
                "epochs: extending a non-constant learning-rate schedule "
                "would change optimizer semantics"
            )

        if mismatches:
            details = "\n- ".join(mismatches)
            raise ValueError(
                "Exact resume configuration mismatch:\n- "
                f"{details}\nRepeat the original training-semantic options or "
                "use --resume-mode branch for an intentional change."
            )

    def save_checkpoint(self, path: str | Path, epoch: int) -> Path:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_suffix(f"{destination.suffix}.tmp")
        torch.save(self.checkpoint_payload(epoch), temporary)
        os.replace(temporary, destination)
        return destination

    def resume(
        self,
        path: str | Path,
        *,
        mode: Literal["exact", "branch"] = "exact",
    ) -> None:
        if mode not in {"exact", "branch"}:
            raise ValueError(f"Unknown resume mode: {mode}")
        self._resume_mode = mode
        self._resume_path = Path(path)
        checkpoint = torch.load(
            path,
            map_location=self.device,
            weights_only=True,
        )
        if not isinstance(checkpoint, dict):
            raise TypeError("Training checkpoint must contain a dictionary")
        if mode == "exact":
            self._validate_exact_resume_config(checkpoint)
        model_state = checkpoint.get("model")
        optimizer_state = checkpoint.get("optimizer")
        scaler_state = checkpoint.get("scaler")
        ema_state = checkpoint.get("ema")
        if not isinstance(model_state, dict) or not isinstance(
            optimizer_state,
            dict,
        ):
            raise TypeError("Training checkpoint is incomplete")
        self.model.load_state_dict(model_state, strict=True)
        self.optimizer.load_state_dict(optimizer_state)
        if not isinstance(ema_state, dict):
            raise TypeError("Training checkpoint has no EMA state")
        self.ema.load_state_dict(ema_state)
        if isinstance(scaler_state, dict):
            self.scaler.load_state_dict(scaler_state)
        self.global_step = int(checkpoint.get("global_step", 0))
        if mode == "exact":
            self.scheduler_step = int(
                checkpoint.get("scheduler_step", self.global_step)
            )
            self.best_fitness = float(
                checkpoint.get("best_fitness", float("-inf"))
            )
            raw_best_scores = checkpoint.get("best_scores", {})
            if isinstance(raw_best_scores, dict):
                self.best_scores.update(
                    {
                        name: float(value)
                        for name, value in raw_best_scores.items()
                        if name in self.best_scores
                        and isinstance(value, (float, int))
                    }
                )
            run_id = checkpoint.get("wandb_run_id")
            self.wandb_run_id = run_id if isinstance(run_id, str) else None
            self.start_epoch = int(checkpoint.get("epoch", -1)) + 1
            pending_epoch = checkpoint.get("pending_validation_epoch")
            self.pending_validation_epoch = (
                int(pending_epoch) if isinstance(pending_epoch, int) else None
            )
            pending_losses = checkpoint.get("pending_train_losses", {})
            self.pending_train_losses = (
                {
                    str(name): float(value)
                    for name, value in pending_losses.items()
                    if isinstance(name, str) and isinstance(value, (float, int))
                }
                if isinstance(pending_losses, dict)
                else {}
            )
            raw_cycles = checkpoint.get("loader_cycles", {})
            if isinstance(raw_cycles, dict):
                self.loader_cycles = {
                    task: int(raw_cycles.get(str(task), 0))
                    for task in self.train_loaders
                }
        else:
            self.scheduler_step = 0
            for group in self.optimizer.param_groups:
                role = str(group.get("role", "other"))
                base_lr = self._role_learning_rate(role)
                group["base_lr"] = base_lr
                group["lr"] = base_lr
        self._restore_runtime_state(checkpoint)

    def _composite_fitness(self, validation: Mapping[str, object]) -> float:
        metric_names = {
            HeadId.OBJECT: "object/map",
            HeadId.PERSON_POSE: "person/map",
            HeadId.ROBOT_POSE: "robot/map",
            HeadId.FIELD_FEATURES: "field/map",
        }
        values = []
        for task in self.validation_loaders:
            name = metric_names[task]
            value = validation.get(name)
            if not isinstance(value, (float, int)) or not math.isfinite(value):
                raise ValueError(
                    f"Validation metric is missing or invalid: {name}"
                )
            values.append(float(value))
        return sum(values) / len(values)

    @staticmethod
    def _finite_metric(
        validation: Mapping[str, object],
        name: str,
    ) -> float | None:
        value = validation.get(name)
        if isinstance(value, (float, int)) and math.isfinite(value):
            return float(value)
        return None

    def _checkpoint_scores(
        self,
        validation: Mapping[str, object],
    ) -> dict[str, float]:
        scores = {}
        for score_name, metric_name in (
            ("object", "object/map"),
            ("field", "field/map"),
            ("strict_field", "field/strict_map"),
        ):
            value = self._finite_metric(validation, metric_name)
            if value is not None:
                scores[score_name] = value
        pck5 = self._finite_metric(
            validation,
            "field/localization/pck_5px",
        )
        if {
            "object",
            "field",
            "strict_field",
        }.issubset(scores) and pck5 is not None:
            scores["object_field"] = (
                0.50 * scores["object"]
                + 0.20 * scores["field"]
                + 0.20 * scores["strict_field"]
                + 0.10 * pck5
            )
        return scores

    def _complete_validation(
        self,
        *,
        epoch: int,
        validation: dict[str, object],
        checkpoint: Path,
    ) -> None:
        fitness = self._composite_fitness(validation)
        validation["composite_fitness"] = fitness
        checkpoint_scores = self._checkpoint_scores(validation)
        validation.update(
            {
                f"checkpoint_score/{name}": value
                for name, value in checkpoint_scores.items()
            }
        )
        self._refresh_local_runtime_state()
        self.pending_validation_epoch = None
        self.pending_train_losses = {}
        is_best = fitness > self.best_fitness
        if is_best:
            self.best_fitness = fitness
        improved_scores = {
            name: score
            for name, score in checkpoint_scores.items()
            if score > self.best_scores[name]
        }
        self.best_scores.update(improved_scores)
        if is_best:
            self.save_checkpoint(self.config.output_dir / "best.pt", epoch)
        best_paths = {
            "object_field": "best_object_field.pt",
            "object": "best_object.pt",
            "field": "best_field.pt",
            "strict_field": "best_strict_field.pt",
        }
        if self.config.save_named_best_checkpoints:
            for name in improved_scores:
                self.save_checkpoint(
                    self.config.output_dir / best_paths[name],
                    epoch,
                )
        self.save_checkpoint(checkpoint, epoch)
        self._log_validation(
            epoch=epoch,
            validation=validation,
            checkpoint=(
                self.config.output_dir / "best.pt" if is_best else checkpoint
            ),
            is_best=is_best,
        )

    def _synchronize_rank_zero_action(
        self,
        action: Callable[[], None],
        *,
        description: str,
    ) -> None:
        """Run rank-zero state changes and propagate failures to every rank."""
        local_error: BaseException | None = None
        status: list[dict[str, str] | None] = [None]
        if self.rank == 0:
            try:
                action()
                status[0] = {"status": "ok"}
            except BaseException as error:
                local_error = error
                status[0] = {
                    "status": "error",
                    "error_type": type(error).__name__,
                    "error": str(error),
                }
        if self.distributed:
            dist.broadcast_object_list(status, src=0)
        result = status[0]
        if result is None:
            raise RuntimeError(f"Rank zero did not finish {description}")
        if result["status"] == "error":
            if local_error is not None:
                raise local_error
            raise RuntimeError(
                f"Rank-zero {description} failed: "
                f"{result.get('error_type', 'Error')}: "
                f"{result.get('error', '')}"
            )

    def _prepare_epoch_checkpoint(
        self,
        *,
        epoch: int,
        losses: Mapping[str, float],
        duration: float,
        validation_due: bool,
        checkpoint: Path,
    ) -> None:
        self.pending_validation_epoch = epoch if validation_due else None
        self.pending_train_losses = dict(losses) if validation_due else {}
        self.save_checkpoint(checkpoint, epoch)
        self._log_train_epoch(
            epoch=epoch,
            losses=losses,
            duration=duration,
        )

    def fit(self) -> dict[str, object]:
        started = time.monotonic()
        history = []
        failure: BaseException | None = None
        checkpoint = self.config.output_dir / "last.pt"
        try:
            self._synchronize_rank_zero_action(
                self._prepare_output_directory,
                description="output-directory preparation",
            )
            self._synchronize_rank_zero_action(
                self._initialize_wandb,
                description="W&B initialization",
            )
            pending_payload = [
                self.pending_validation_epoch if self.rank == 0 else None
            ]
            if self.distributed:
                dist.broadcast_object_list(pending_payload, src=0)
            pending_epoch = pending_payload[0]
            if pending_epoch is not None:
                validation = self.validate(pending_epoch)
                self._synchronize_rank_zero_action(
                    partial(
                        self._complete_validation,
                        epoch=pending_epoch,
                        validation=validation,
                        checkpoint=checkpoint,
                    ),
                    description="pending validation completion",
                )
            if self.distributed:
                dist.barrier()
            for epoch in range(self.start_epoch, self.config.epochs):
                epoch_started = time.monotonic()
                losses = self.train_epoch(epoch)
                validation_due = (
                    (epoch + 1) % self.config.validation_interval == 0
                    or epoch + 1 == self.config.epochs
                )
                record: dict[str, object] = {
                    "epoch": epoch,
                    "losses": losses,
                }
                self._synchronize_rank_zero_action(
                    partial(
                        self._prepare_epoch_checkpoint,
                        epoch=epoch,
                        losses=losses,
                        duration=time.monotonic() - epoch_started,
                        validation_due=validation_due,
                        checkpoint=checkpoint,
                    ),
                    description=f"epoch {epoch} checkpoint preparation",
                )
                if validation_due:
                    validation = self.validate(epoch)
                    self._synchronize_rank_zero_action(
                        partial(
                            self._complete_validation,
                            epoch=epoch,
                            validation=validation,
                            checkpoint=checkpoint,
                        ),
                        description=f"epoch {epoch} validation completion",
                    )
                    if self.rank == 0:
                        record["validation"] = validation
                if self.rank == 0:
                    history.append(record)
                if self.distributed:
                    dist.barrier(
                        device_ids=[self.local_rank]
                        if self.device.type == "cuda"
                        else None
                    )
        except BaseException as error:
            failure = error
            if self.rank == 0 and self.wandb_run is not None:
                self.wandb_run.summary["status"] = "failed"
                self.wandb_run.summary["failure_type"] = type(error).__name__
                self.wandb_run.summary["failure_message"] = str(error)
                if checkpoint.is_file() and self.config.wandb_log_checkpoints:
                    self._log_checkpoint_artifact(
                        checkpoint,
                        epoch=self.pending_validation_epoch
                        if self.pending_validation_epoch is not None
                        else max(self.start_epoch - 1, 0),
                        aliases=["latest", "recovery"],
                    )
            raise
        finally:
            if self.rank == 0 and self.wandb_run is not None:
                self.wandb_run.summary["training_seconds"] = (
                    time.monotonic() - started
                )
                self.wandb_run.summary["global_step"] = self.global_step
                self.wandb_run.finish(exit_code=1 if failure else 0)
            if self.owns_process_group and dist.is_initialized():
                with suppress(AssertionError):
                    dist.destroy_process_group()
        return {"history": history}
