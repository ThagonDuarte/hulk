"""Deterministic training and rendered validation for multi-task D-FINE."""

# ruff: noqa: C901, S311, TRY003

import math
import os
import random
import time
from collections.abc import Iterator, Mapping
from contextlib import suppress
from copy import deepcopy
from dataclasses import asdict, dataclass, field
from datetime import timedelta
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
    max_validation_batches: int | None = None
    render_object_confidence: float = 0.25
    render_person_confidence: float = 0.5
    render_robot_confidence: float = 0.5
    render_field_confidence: float = 0.35
    render_keypoint_confidence: float = 0.5
    render_max_detections: int = 20
    weight_decay: float = 1e-4
    warmup_steps: int = 500
    clip_max_norm: float = 0.1
    ema_decay: float = 0.9999
    ema_warmups: int = 1_000
    amp: bool = False
    dataset_audits: Mapping[str, Mapping[str, int]] = field(
        default_factory=dict
    )


_ROLE_PREFIXES = {
    "heads": (
        "person_pose_head.",
        "robot_pose_head.",
        "field_feature_head.",
    ),
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


def stage_training_config(
    stage: int,
    *,
    output_dir: Path,
    epochs: int,
    device: str = "cpu",
    learning_rate: float = 1e-4,
    sampling_weights: Mapping[HeadId, float] | None = None,
) -> MultiTaskTrainingConfig:
    """Build one independently resumable stage configuration."""
    roles = STAGE_TRAINABLE_ROLES.get(stage)
    if roles is None:
        raise ValueError("Training stage must be 1, 2, or 3")
    role_learning_rates = {
        "heads": learning_rate,
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
        stage_name=f"stage_{stage}",
        sampling_weights=sampling_weights or {},
        trainable_roles=roles,
        role_learning_rates=role_learning_rates,
    )


def _parameter_role(name: str) -> str | None:
    for role, prefixes in _ROLE_PREFIXES.items():
        if name.startswith(prefixes):
            return role
    return None


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
            if value.is_floating_point():
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
        self.world_size = int(os.environ.get("WORLD_SIZE", "1"))
        self.rank = int(os.environ.get("RANK", "0"))
        self.local_rank = int(os.environ.get("LOCAL_RANK", "0"))
        self.distributed = self.world_size > 1
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
        self.model = model.to(self.device)
        self.train_loaders = dict(train_loaders)
        self.validation_loaders = (
            dict(validation_loader)
            if isinstance(validation_loader, Mapping)
            else {HeadId.OBJECT: validation_loader}
        )
        self.config = config
        self.global_step = 0
        self.start_epoch = 0
        self.loader_cycles = dict.fromkeys(self.train_loaders, 0)
        self.wandb_run: Any | None = None
        self.wandb_run_id: str | None = None
        self.pending_validation_epoch: int | None = None
        self.pending_train_losses: dict[str, float] = {}
        self.best_fitness = float("-inf")
        self._configure_trainable_parameters()
        self.training_model: nn.Module = self.model
        if self.distributed:
            device_ids = (
                [self.local_rank] if self.device.type == "cuda" else None
            )
            self.training_model = DistributedDataParallel(
                self.model,
                device_ids=device_ids,
                find_unused_parameters=True,
            )
        self.criterion: nn.Module = MultiTaskCriterion(
            self.model.detector.nc
        ).to(self.device)
        self.optimizer = self._build_optimizer()
        self.amp_enabled = config.amp and self.device.type == "cuda"
        self.scaler = torch.GradScaler("cuda", enabled=self.amp_enabled)
        self.ema = _ModelEMA(
            self.model,
            decay=config.ema_decay,
            warmups=config.ema_warmups,
        )

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
        gradient_norm: float,
    ) -> None:
        if (
            self.wandb_run is None
            or self.global_step % self.config.wandb_log_interval != 0
        ):
            return
        values: dict[str, object] = {
            "global_step": self.global_step,
            "epoch": epoch,
            "train/step/task": str(task),
            "train/step/total_loss": float(total.detach()),
            "train/step/duration_seconds": duration,
            "train/step/gradient_norm": gradient_norm,
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
        self.wandb_run.log(values)

    def _log_train_epoch(
        self,
        *,
        epoch: int,
        losses: Mapping[str, float],
        duration: float,
    ) -> None:
        if self.wandb_run is None:
            return
        values: dict[str, object] = {
            "epoch": epoch,
            "global_step": self.global_step,
            "stage": self.config.stage_name,
            "train/epoch/duration_seconds": duration,
        }
        values.update(
            {f"train/epoch/{name}": value for name, value in losses.items()}
        )
        self.wandb_run.log(values)

    def _log_validation(
        self,
        *,
        epoch: int,
        validation: Mapping[str, object],
        checkpoint: Path,
        is_best: bool,
    ) -> None:
        if self.wandb_run is None:
            return
        import wandb

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
        unknown = roles - {*_ROLE_PREFIXES, "all"}
        if unknown:
            raise ValueError(f"Unknown trainable roles: {sorted(unknown)}")
        matched_roles = {
            role
            for name, _ in self.model.named_parameters()
            if (role := _parameter_role(name)) is not None
        }
        missing = roles - matched_roles - {"all"}
        if missing:
            raise ValueError(
                f"Trainable roles matched no parameters: {sorted(missing)}"
            )
        for name, parameter in self.model.named_parameters():
            role = _parameter_role(name)
            parameter.requires_grad_("all" in roles or role in roles)

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
                "lr": self.config.role_learning_rates.get(
                    role,
                    self.config.learning_rate,
                ),
                "base_lr": self.config.role_learning_rates.get(
                    role,
                    self.config.learning_rate,
                ),
                "weight_decay": self.config.weight_decay if use_decay else 0.0,
                "role": role,
            }
            for (role, use_decay), parameters in groups.items()
        ]
        return torch.optim.AdamW(parameter_groups, betas=(0.9, 0.999))

    def _set_warmup_learning_rates(self) -> None:
        scale = (
            1.0
            if self.config.warmup_steps == 0
            else min(1.0, (self.global_step + 1) / self.config.warmup_steps)
        )
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

    def train_epoch(self, epoch: int = 0) -> dict[str, float]:
        self.training_model.train()
        for loader in self.train_loaders.values():
            set_epoch = getattr(loader.sampler, "set_epoch", None)
            if callable(set_epoch):
                set_epoch(epoch)
        totals: dict[str, float] = {}
        counts: dict[str, int] = {}
        for task, (images, targets) in self._scheduled_batches(epoch):
            step_started = time.monotonic()
            images = images.to(self.device, non_blocking=True)
            moved_targets = _move_targets(targets, self.device)
            self._set_warmup_learning_rates()
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
            self.scaler.unscale_(self.optimizer)
            gradient_norm = nn.utils.clip_grad_norm_(
                self.model.parameters(),
                self.config.clip_max_norm,
                error_if_nonfinite=True,
            )
            self.scaler.step(self.optimizer)
            self.scaler.update()
            self.ema.update(self.model)
            self.global_step += 1
            if self.rank == 0:
                self._log_training_step(
                    task=task,
                    losses=result.losses,
                    total=total,
                    epoch=epoch,
                    duration=time.monotonic() - step_started,
                    gradient_norm=float(gradient_norm),
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
        return averages

    @torch.no_grad()
    def validate(self, epoch: int) -> dict[str, object]:
        validator = MultiTaskValidator(
            cast("DFINEMultiTaskModel", self.ema.module),
            self.validation_loaders,
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
                render_dir=(
                    self.config.output_dir / "validation" / f"epoch_{epoch:03d}"
                ),
                max_batches=self.config.max_validation_batches,
            ),
        )

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
                "optimizer": self.optimizer.state_dict(),
                "scaler": self.scaler.state_dict(),
                "ema": self.ema.state_dict(),
                "inference_model": self.ema.module.state_dict(),
                "best_fitness": self.best_fitness,
                "stage": self.config.stage_name,
                "loader_cycles": {
                    str(task): count
                    for task, count in self.loader_cycles.items()
                },
                "wandb_run_id": self.wandb_run_id,
                "pending_validation_epoch": self.pending_validation_epoch,
                "pending_train_losses": self.pending_train_losses,
                "training_config": {
                    **asdict(self.config),
                    "output_dir": str(self.config.output_dir),
                    "sampling_weights": {
                        str(task): value
                        for task, value in self.config.sampling_weights.items()
                    },
                },
            }
        )
        return payload

    def save_checkpoint(self, path: str | Path, epoch: int) -> Path:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_suffix(f"{destination.suffix}.tmp")
        torch.save(self.checkpoint_payload(epoch), temporary)
        os.replace(temporary, destination)
        return destination

    def resume(self, path: str | Path) -> None:
        checkpoint = torch.load(
            path,
            map_location=self.device,
            weights_only=True,
        )
        if not isinstance(checkpoint, dict):
            raise TypeError("Training checkpoint must contain a dictionary")
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
        self.best_fitness = float(checkpoint.get("best_fitness", float("-inf")))
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

    def _complete_validation(
        self,
        *,
        epoch: int,
        validation: dict[str, object],
        checkpoint: Path,
    ) -> None:
        fitness = self._composite_fitness(validation)
        validation["composite_fitness"] = fitness
        self.pending_validation_epoch = None
        self.pending_train_losses = {}
        is_best = fitness > self.best_fitness
        if is_best:
            self.best_fitness = fitness
            self.save_checkpoint(self.config.output_dir / "best.pt", epoch)
        self.save_checkpoint(checkpoint, epoch)
        self._log_validation(
            epoch=epoch,
            validation=validation,
            checkpoint=(
                self.config.output_dir / "best.pt" if is_best else checkpoint
            ),
            is_best=is_best,
        )

    def fit(self) -> dict[str, object]:
        self._initialize_wandb()
        started = time.monotonic()
        history = []
        failure: BaseException | None = None
        checkpoint = self.config.output_dir / "last.pt"
        try:
            if self.pending_validation_epoch is not None and self.rank == 0:
                pending_epoch = self.pending_validation_epoch
                validation = self.validate(pending_epoch)
                self._complete_validation(
                    epoch=pending_epoch,
                    validation=validation,
                    checkpoint=checkpoint,
                )
            if self.distributed:
                dist.barrier()
            for epoch in range(self.start_epoch, self.config.epochs):
                epoch_started = time.monotonic()
                losses = self.train_epoch(epoch)
                if self.rank == 0:
                    duration = time.monotonic() - epoch_started
                    self.pending_validation_epoch = epoch
                    self.pending_train_losses = dict(losses)
                    self.save_checkpoint(checkpoint, epoch)
                    self._log_train_epoch(
                        epoch=epoch,
                        losses=losses,
                        duration=duration,
                    )
                    validation = self.validate(epoch)
                    self._complete_validation(
                        epoch=epoch,
                        validation=validation,
                        checkpoint=checkpoint,
                    )
                    history.append(
                        {
                            "epoch": epoch,
                            "losses": losses,
                            "validation": validation,
                        }
                    )
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
