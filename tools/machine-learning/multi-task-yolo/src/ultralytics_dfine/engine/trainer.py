# ruff: noqa: C901, TRY003

import copy
import json
import math
import os
import random
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Literal, cast

import torch
import torch.distributed as distributed
import torch.nn as nn
import yaml
from torch import Tensor
from torch.nn.parallel import DistributedDataParallel
from torch.utils.data import DataLoader, DistributedSampler
from tqdm import tqdm

from ultralytics_dfine.config import (
    DFINERecipeConfig,
    DFINEStageState,
    build_manifest,
)
from ultralytics_dfine.data import (
    BatchImageCollateFunction,
    DatasetTarget,
    DFINEDataset,
)
from ultralytics_dfine.engine.validator import DFINEValidator
from ultralytics_dfine.loss import DFINECriterion
from ultralytics_dfine.loss.matcher import Target
from ultralytics_dfine.nn import DFINEDetectionModel


@dataclass(frozen=True)
class DFINETrainingConfig:
    data: Path
    output_dir: Path
    recipe: DFINERecipeConfig
    device: str = "cuda"
    workers: int = 4
    validation_batch_size: int = 16
    seed: int = 0
    run_name: str = "dfine-s"
    wandb_project: str = "multi-task-yolo-dfine"
    wandb_mode: Literal["online", "offline", "disabled"] = "online"
    resume: Path | None = None
    max_train_batches: int | None = None
    max_val_batches: int | None = None
    render_count: int = 4


def seed_everything(seed: int) -> None:
    random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)


def _unwrap(
    model: DFINEDetectionModel | DistributedDataParallel,
) -> DFINEDetectionModel:
    if isinstance(model, DistributedDataParallel):
        module = model.module
        if not isinstance(module, DFINEDetectionModel):
            raise TypeError("DDP module is not a D-FINE model")
        return module
    return model


class ModelEMA:
    def __init__(
        self,
        model: DFINEDetectionModel,
        *,
        decay: float,
        warmups: int,
    ) -> None:
        self.module = copy.deepcopy(model).eval()
        self.decay = decay
        self.warmups = warmups
        self.updates = 0
        self.module.requires_grad_(requires_grad=False)

    def _decay(self) -> float:
        if self.warmups == 0:
            return self.decay
        return self.decay * (1 - math.exp(-self.updates / self.warmups))

    @torch.no_grad()
    def update(self, model: DFINEDetectionModel) -> None:
        self.updates += 1
        decay = self._decay()
        source = model.state_dict()
        for key, value in self.module.state_dict().items():
            if value.dtype.is_floating_point:
                value.mul_(decay).add_(source[key].detach(), alpha=1 - decay)
            else:
                value.copy_(source[key])

    def state_dict(self) -> dict[str, object]:
        return {
            "module": self.module.state_dict(),
            "updates": self.updates,
            "decay": self.decay,
            "warmups": self.warmups,
        }

    def load_state_dict(self, state: dict[str, object]) -> None:
        module = state.get("module")
        if not isinstance(module, dict):
            raise TypeError("EMA checkpoint is missing weights")
        self.module.load_state_dict(module, strict=True)
        updates = state.get("updates", 0)
        decay = state.get("decay", self.decay)
        warmups = state.get("warmups", self.warmups)
        if not isinstance(updates, int) or not isinstance(warmups, int):
            raise TypeError("EMA update counts must be integers")
        if not isinstance(decay, (float, int)):
            raise TypeError("EMA decay must be numeric")
        self.updates = updates
        self.decay = float(decay)
        self.warmups = warmups


def build_optimizer(
    model: DFINEDetectionModel,
    recipe: DFINERecipeConfig,
) -> torch.optim.AdamW:
    groups: dict[tuple[str, bool], list[Tensor]] = {
        ("backbone", True): [],
        ("backbone", False): [],
        ("main", True): [],
        ("main", False): [],
    }
    visited: set[int] = set()
    for name, parameter in model.named_parameters():
        if not parameter.requires_grad:
            continue
        role = "backbone" if ".backbone." in name else "main"
        lowered = name.lower()
        no_decay = (
            parameter.ndim == 1
            or name.endswith(".bias")
            or "norm" in lowered
            or ".bn" in lowered
        )
        groups[(role, not no_decay)].append(parameter)
        if id(parameter) in visited:
            raise RuntimeError(
                f"Parameter '{name}' matched more than one group"
            )
        visited.add(id(parameter))

    expected = {
        id(parameter)
        for parameter in model.parameters()
        if parameter.requires_grad
    }
    if visited != expected:
        raise RuntimeError("Some trainable D-FINE parameters were not grouped")

    parameter_groups = []
    for (role, use_decay), parameters in groups.items():
        if not parameters:
            continue
        learning_rate = (
            recipe.backbone_lr if role == "backbone" else recipe.base_lr
        )
        parameter_groups.append(
            {
                "params": parameters,
                "lr": learning_rate,
                "base_lr": learning_rate,
                "weight_decay": recipe.weight_decay if use_decay else 0.0,
                "role": role,
            }
        )
    return torch.optim.AdamW(
        parameter_groups,
        betas=(0.9, 0.999),
    )


def _move_training_targets(
    targets: list[DatasetTarget],
    device: torch.device,
) -> list[Target]:
    return [
        {
            "labels": target["labels"].to(device, non_blocking=True),
            "boxes": target["boxes"].to(device, non_blocking=True),
        }
        for target in targets
    ]


def _aggregate_losses(losses: dict[str, float]) -> dict[str, float]:
    names = ("loss_vfl", "loss_bbox", "loss_giou", "loss_fgl", "loss_ddf")
    return {
        name: sum(
            value for key, value in losses.items() if key.startswith(name)
        )
        for name in names
    }


class DFINETrainer:
    def __init__(
        self,
        model: DFINEDetectionModel,
        config: DFINETrainingConfig,
    ) -> None:
        self.config = config
        self.rank = int(os.environ.get("RANK", "0"))
        self.local_rank = int(os.environ.get("LOCAL_RANK", "0"))
        self.world_size = int(os.environ.get("WORLD_SIZE", "1"))
        self.distributed = self.world_size > 1
        if self.distributed:
            torch.cuda.set_device(self.local_rank)
            self.device = torch.device("cuda", self.local_rank)
            if not distributed.is_initialized():
                distributed.init_process_group(
                    backend="nccl",
                    device_id=self.device,
                )
        else:
            self.device = torch.device(config.device)
        self.is_main = self.rank == 0
        seed_everything(config.seed + self.rank)

        if config.recipe.total_batch_size % self.world_size:
            raise ValueError("total_batch_size must be divisible by world size")
        self.batch_size = config.recipe.total_batch_size // self.world_size
        if self.batch_size <= 0:
            raise ValueError("Per-rank batch size must be positive")

        model = model.to(self.device)
        if self.distributed:
            model = cast(
                DFINEDetectionModel,
                nn.SyncBatchNorm.convert_sync_batchnorm(model),
            )
        self.model: DFINEDetectionModel | DistributedDataParallel = model
        self.ema = ModelEMA(
            model,
            decay=config.recipe.ema_decay,
            warmups=config.recipe.ema_warmups,
        )
        if self.distributed:
            self.model = DistributedDataParallel(
                model,
                device_ids=[self.local_rank],
                output_device=self.local_rank,
                find_unused_parameters=False,
            )
        self.criterion = DFINECriterion(model.nc).to(self.device)
        self.optimizer = build_optimizer(model, config.recipe)
        amp_enabled = config.recipe.amp and self.device.type == "cuda"
        self.scaler = torch.GradScaler("cuda", enabled=amp_enabled)
        self.amp_enabled = amp_enabled
        self.global_step = 0
        self.start_epoch = 0
        self.stage = DFINEStageState(
            transition_epoch=config.recipe.transition_epoch,
            ema_decay=config.recipe.ema_decay,
        )
        self.best_fitness = float("-inf")
        self.wandb_run: Any | None = None

        self.train_dataset = DFINEDataset(
            config.data,
            "train",
            transition_epoch=config.recipe.transition_epoch,
        )
        self.collate = BatchImageCollateFunction(
            base_size=model.architecture.image_size,
            base_size_repeat=config.recipe.multiscale_repeat,
            stop_epoch=config.recipe.transition_epoch,
        )
        self.sampler = (
            DistributedSampler(
                self.train_dataset,
                num_replicas=self.world_size,
                rank=self.rank,
                shuffle=True,
                seed=config.seed,
                drop_last=True,
            )
            if self.distributed
            else None
        )
        self.train_loader = DataLoader(
            self.train_dataset,
            batch_size=self.batch_size,
            shuffle=self.sampler is None,
            sampler=self.sampler,
            num_workers=config.workers,
            pin_memory=self.device.type == "cuda",
            drop_last=True,
            collate_fn=self.collate,
        )
        if self.is_main:
            config.output_dir.mkdir(parents=True, exist_ok=True)
        self._barrier()
        if config.resume is not None:
            self._load_checkpoint(config.resume, restore_epoch=True)

    def _barrier(self) -> None:
        if self.distributed:
            distributed.barrier()

    def _initialize_wandb(self) -> None:
        if not self.is_main:
            return
        import wandb

        values = {
            "training": {
                **asdict(self.config.recipe),
                "data": str(self.config.data),
                "world_size": self.world_size,
                "seed": self.config.seed,
            },
            "manifest": build_manifest(
                _unwrap(self.model).architecture,
                _unwrap(self.model).names,
            ).to_dict(),
        }
        try:
            self.wandb_run = wandb.init(
                project=self.config.wandb_project,
                name=self.config.run_name,
                config=values,
                mode=self.config.wandb_mode,
            )
        except wandb.Error:
            self.wandb_run = wandb.init(
                project=self.config.wandb_project,
                name=self.config.run_name,
                config=values,
                mode="offline",
                reinit=True,
            )
            self.wandb_run.summary["online_upload_blocked"] = True

    def _set_warmup_learning_rate(self) -> None:
        scale = min(
            1.0,
            (self.global_step + 1) / self.config.recipe.warmup_steps,
        )
        for group in self.optimizer.param_groups:
            group["lr"] = float(group["base_lr"]) * scale

    def _train_epoch(self, epoch: int) -> dict[str, float]:
        self.model.train()
        self.criterion.train()
        self.train_dataset.set_epoch(epoch)
        self.collate.set_epoch(epoch)
        if self.sampler is not None:
            self.sampler.set_epoch(epoch)
        totals: dict[str, float] = {}
        batches = 0
        progress = tqdm(
            self.train_loader,
            desc=f"Epoch {epoch + 1}/{self.config.recipe.epochs}",
            disable=not self.is_main,
        )
        for batch_index, (images, targets) in enumerate(progress):
            if (
                self.config.max_train_batches is not None
                and batch_index >= self.config.max_train_batches
            ):
                break
            images = images.to(self.device, non_blocking=True)
            training_targets = _move_training_targets(targets, self.device)
            self._set_warmup_learning_rate()
            self.optimizer.zero_grad(set_to_none=True)
            with torch.autocast(
                device_type=self.device.type,
                enabled=self.amp_enabled,
            ):
                outputs = self.model(images, training_targets)
            if not isinstance(outputs, dict):
                raise TypeError(
                    "D-FINE model must return a training dictionary"
                )
            for key in ("pred_logits", "pred_boxes"):
                value = outputs.get(key)
                if (
                    not isinstance(value, Tensor)
                    or not torch.isfinite(value).all()
                ):
                    raise FloatingPointError(f"Non-finite D-FINE output: {key}")
            with torch.autocast(
                device_type=self.device.type,
                enabled=False,
            ):
                loss_dict = self.criterion(outputs, training_targets)
                if not loss_dict:
                    raise RuntimeError("D-FINE criterion returned no losses")
                for key, value in loss_dict.items():
                    if not torch.isfinite(value).all():
                        raise FloatingPointError(
                            f"Non-finite D-FINE loss: {key}"
                        )
                total_loss = torch.stack(tuple(loss_dict.values())).sum()
                # Denoising is legitimately absent for an all-empty batch.
                denoising = _unwrap(self.model).core.model.denoising_class_embed
                total_loss = total_loss + denoising.weight.sum() * 0

            self.scaler.scale(total_loss).backward()
            self.scaler.unscale_(self.optimizer)
            gradient_norm = nn.utils.clip_grad_norm_(
                self.model.parameters(),
                self.config.recipe.clip_max_norm,
            )
            if not torch.isfinite(gradient_norm):
                raise FloatingPointError("Non-finite D-FINE gradient norm")
            self.scaler.step(self.optimizer)
            self.scaler.update()
            self.ema.update(_unwrap(self.model))
            self.global_step += 1
            batches += 1
            for key, value in loss_dict.items():
                totals[key] = totals.get(key, 0.0) + float(
                    value.detach().item()
                )
            if self.is_main:
                progress.set_postfix(
                    loss=f"{float(total_loss.detach().item()):.3f}",
                    grad=f"{float(gradient_norm):.3f}",
                )
                if self.wandb_run is not None and self.global_step % 20 == 0:
                    self.wandb_run.log(
                        {
                            "train/step_loss": float(
                                total_loss.detach().item()
                            ),
                            "train/gradient_norm": float(gradient_norm),
                            "train/learning_rate": self.optimizer.param_groups[
                                0
                            ]["lr"],
                            "global_step": self.global_step,
                            "epoch": epoch,
                        }
                    )
        if batches == 0:
            raise RuntimeError("Training epoch produced no batches")
        return {key: value / batches for key, value in totals.items()}

    def _validate(self, epoch: int) -> dict[str, float | list[float]]:
        if not self.is_main:
            return {}
        validator = DFINEValidator(
            self.ema.module,
            self.config.data,
            device=self.device,
            batch_size=self.config.validation_batch_size,
            workers=self.config.workers,
        )
        return validator.run(
            render_dir=self.config.output_dir / "validation_renders",
            epoch=epoch,
            wandb_run=self.wandb_run,
            max_batches=self.config.max_val_batches,
            render_count=self.config.render_count,
        )

    def _checkpoint_payload(self, epoch: int) -> dict[str, object]:
        payload = _unwrap(self.model).checkpoint_payload()
        payload.update(
            {
                "epoch": epoch,
                "global_step": self.global_step,
                "optimizer": self.optimizer.state_dict(),
                "scaler": self.scaler.state_dict(),
                "ema": self.ema.state_dict(),
                "inference_model": self.ema.module.state_dict(),
                "stage_state": asdict(self.stage),
                "recipe": asdict(self.config.recipe),
                "best_fitness": self.best_fitness,
            }
        )
        return payload

    def _save_checkpoint(self, path: Path, epoch: int) -> None:
        if self.is_main:
            torch.save(self._checkpoint_payload(epoch), path)

    def _save_neutral_artifacts(self, stem: str) -> None:
        if not self.is_main:
            return
        torch.save(
            self.ema.module.state_dict(),
            self.config.output_dir / f"{stem}.state_dict.pt",
        )
        model = _unwrap(self.model)
        manifest = build_manifest(model.architecture, model.names)
        with (self.config.output_dir / f"{stem}.manifest.yaml").open(
            "w",
            encoding="utf-8",
        ) as file:
            yaml.safe_dump(manifest.to_dict(), file, sort_keys=False)

    def _load_checkpoint(self, path: Path, *, restore_epoch: bool) -> None:
        checkpoint = torch.load(
            path,
            map_location=self.device,
            weights_only=True,
        )
        if not isinstance(checkpoint, dict):
            raise TypeError("D-FINE resume checkpoint must be a dictionary")
        model_state = checkpoint.get("model")
        if not isinstance(model_state, dict):
            raise TypeError("D-FINE resume checkpoint has no model weights")
        _unwrap(self.model).load_state_dict(model_state, strict=True)
        ema_state = checkpoint.get("ema")
        optimizer_state = checkpoint.get("optimizer")
        scaler_state = checkpoint.get("scaler")
        if not isinstance(ema_state, dict) or not isinstance(
            optimizer_state, dict
        ):
            raise TypeError("D-FINE resume checkpoint is incomplete")
        self.ema.load_state_dict(ema_state)
        self.optimizer.load_state_dict(optimizer_state)
        if isinstance(scaler_state, dict):
            self.scaler.load_state_dict(scaler_state)
        stage_values = checkpoint.get("stage_state")
        if isinstance(stage_values, dict):
            self.stage = DFINEStageState(**stage_values)
        self.global_step = int(checkpoint.get("global_step", 0))
        self.best_fitness = float(checkpoint.get("best_fitness", float("-inf")))
        if restore_epoch:
            self.start_epoch = int(checkpoint.get("epoch", -1)) + 1

    def _transition_to_stage_two(self) -> None:
        stage_one = self.config.output_dir / "best_stg1.pt"
        self._barrier()
        if not stage_one.exists():
            raise FileNotFoundError("Stage-one best checkpoint does not exist")
        self._load_checkpoint(stage_one, restore_epoch=False)
        self.stage.stage = 2
        self.stage.augmentation_active = False
        self.stage.multiscale_active = False
        self.stage.ema_decay = self.config.recipe.ema_decay
        self.ema.decay = self.stage.ema_decay
        self._barrier()

    def _write_epoch_log(
        self,
        epoch: int,
        train_losses: dict[str, float],
        metrics: dict[str, float | list[float]],
        duration: float,
    ) -> None:
        if not self.is_main:
            return
        values: dict[str, object] = {
            "epoch": epoch,
            "stage": self.stage.stage,
            "duration_seconds": duration,
            "train": train_losses,
            "train_aggregated": _aggregate_losses(train_losses),
            "validation": metrics,
        }
        with (self.config.output_dir / "metrics.jsonl").open(
            "a",
            encoding="utf-8",
        ) as file:
            file.write(json.dumps(values) + "\n")
        if self.wandb_run is not None:
            wandb_values: dict[str, object] = {
                f"train/{key}": value for key, value in train_losses.items()
            }
            wandb_values.update(
                {
                    f"train/aggregated_{key}": value
                    for key, value in _aggregate_losses(train_losses).items()
                }
            )
            wandb_values.update(metrics)
            wandb_values["epoch"] = epoch
            wandb_values["stage"] = self.stage.stage
            self.wandb_run.log(wandb_values)

    def fit(self) -> dict[str, object]:
        self._initialize_wandb()
        started = time.monotonic()
        final_metrics: dict[str, float | list[float]] = {}
        try:
            for epoch in range(self.start_epoch, self.config.recipe.epochs):
                if (
                    epoch == self.config.recipe.transition_epoch
                    and self.stage.stage == 1
                ):
                    self._transition_to_stage_two()
                epoch_started = time.monotonic()
                train_losses = self._train_epoch(epoch)
                self._save_checkpoint(
                    self.config.output_dir / "last.pt",
                    epoch,
                )
                self._barrier()
                final_metrics = self._validate(epoch)
                self._barrier()
                fitness_value = final_metrics.get(
                    "metrics/mAP50-95(B)",
                    float("-inf"),
                )
                fitness = (
                    fitness_value
                    if self.is_main and isinstance(fitness_value, float)
                    else float("-inf")
                )
                if self.distributed:
                    fitness_tensor = torch.tensor(fitness, device=self.device)
                    distributed.broadcast(fitness_tensor, src=0)
                    fitness = float(fitness_tensor.item())

                if (
                    self.stage.stage == 1
                    and fitness > self.stage.stage1_best_fitness
                ):
                    self.stage.stage1_best_fitness = fitness
                    self.stage.stage1_best_checkpoint = str(
                        self.config.output_dir / "best_stg1.pt"
                    )
                    self._save_checkpoint(
                        self.config.output_dir / "best_stg1.pt",
                        epoch,
                    )
                if (
                    self.stage.stage == 2
                    and fitness > self.stage.stage2_best_fitness
                ):
                    self.stage.stage2_best_fitness = fitness
                    self.stage.stage2_best_checkpoint = str(
                        self.config.output_dir / "best_stg2.pt"
                    )
                    self._save_checkpoint(
                        self.config.output_dir / "best_stg2.pt",
                        epoch,
                    )
                if fitness > self.best_fitness:
                    self.best_fitness = fitness
                    self._save_checkpoint(
                        self.config.output_dir / "best.pt",
                        epoch,
                    )
                    self._save_neutral_artifacts("best")
                self._save_checkpoint(
                    self.config.output_dir / "last.pt",
                    epoch,
                )
                self._write_epoch_log(
                    epoch,
                    train_losses,
                    final_metrics,
                    time.monotonic() - epoch_started,
                )
                self._barrier()
        finally:
            if self.is_main and self.wandb_run is not None:
                self.wandb_run.summary["training_seconds"] = (
                    time.monotonic() - started
                )
                self.wandb_run.finish()
            if self.distributed and distributed.is_initialized():
                distributed.destroy_process_group()
        return {
            "output_dir": str(self.config.output_dir),
            "best_fitness": self.best_fitness,
            "metrics": final_metrics,
            "stage": asdict(self.stage),
        }
