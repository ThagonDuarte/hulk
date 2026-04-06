from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, cast

import torch
from torch import nn, optim
from torch.optim.lr_scheduler import CosineAnnealingLR
from ultralytics.utils.torch_utils import (
    ModelEMA,
    autocast,
    init_seeds,
    select_device,
)

from model.hydra import Hydra, MissingHydraHeadError
from model.joint_config import JointTrainConfig, TaskName
from model.joint_data import (
    DynamicInterleavedLoader,
    TaskLoaderBundle,
    build_task_train_loader,
)
from model.joint_losses import DynamicUncertaintyWeighting, TaskCriterionAdapter
from validation.validator import MultiTaskHydraValidator, ValidationTaskConfig

logger = logging.getLogger(__name__)

TRAIN_YAML_FILENAME = "train_config.json"
LAST_FILENAME = "last.pt"
BEST_FILENAME = "best.pt"


class EmptyOptimizerParamsError(ValueError):
    def __init__(self) -> None:
        super().__init__("Optimizer cannot be created with empty params")


class UnsupportedOptimizerError(ValueError):
    def __init__(self, name: str) -> None:
        super().__init__(f"Unsupported optimizer '{name}'")


class MissingTaskMetricsError(KeyError):
    def __init__(self, task_name: TaskName) -> None:
        super().__init__(f"Missing validation metrics for task '{task_name}'")


class MissingTaskHeadOptimizerError(KeyError):
    def __init__(self, task_name: str) -> None:
        super().__init__(f"Missing head optimizer for task '{task_name}'")


@dataclass
class TrainState:
    epoch: int = 0
    global_step: int = 0
    best_fitness: float = float("-inf")


def _to_device_batch(
    batch: dict[str, Any], device: torch.device
) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in batch.items():
        if isinstance(value, torch.Tensor):
            result[key] = value.to(device, non_blocking=device.type == "cuda")
        else:
            result[key] = value
    result["img"] = result["img"].float() / 255
    return result


def _collect_task_params(
    hydra: Hydra,
    task_names: list[TaskName],
) -> tuple[dict[TaskName, list[nn.Parameter]], list[nn.Parameter]]:
    head_params: dict[TaskName, list[nn.Parameter]] = {}
    all_head_params: list[nn.Parameter] = []
    for task_name in task_names:
        if task_name not in hydra.heads:
            raise MissingHydraHeadError(task_name)
        head = hydra.heads[task_name]
        params = [p for p in head.parameters() if p.requires_grad]
        head_params[task_name] = params
        all_head_params.extend(params)
    return head_params, all_head_params


def _build_optimizer(
    params: list[nn.Parameter],
    *,
    name: str,
    lr: float,
    weight_decay: float,
    momentum: float,
) -> optim.Optimizer:
    if not params:
        raise EmptyOptimizerParamsError
    if name == "AdamW":
        return optim.AdamW(params, lr=lr, weight_decay=weight_decay)
    if name == "SGD":
        return optim.SGD(
            params,
            lr=lr,
            momentum=momentum,
            nesterov=True,
            weight_decay=weight_decay,
        )
    raise UnsupportedOptimizerError(name)


def _build_task_loaders(
    config: JointTrainConfig,
    stride: int,
) -> dict[TaskName, TaskLoaderBundle]:
    bundles: dict[TaskName, TaskLoaderBundle] = {}
    for task_name, task_config in config.tasks.items():
        bundles[task_name] = build_task_train_loader(
            task_name=task_name,
            task_config=task_config,
            fraction=config.fraction,
            stride=stride,
        )
    return bundles


def _prepare_run_dir(config: JointTrainConfig) -> Path:
    run_dir = config.run_dir
    run_dir.mkdir(parents=True, exist_ok=True)
    with open(run_dir / TRAIN_YAML_FILENAME, "w", encoding="utf-8") as f:
        json.dump(config.to_dict(), f, indent=2)
    return run_dir


def _build_validation_configs(
    config: JointTrainConfig,
    project_dir: Path,
) -> dict[TaskName, ValidationTaskConfig]:
    return {
        task_name: ValidationTaskConfig(
            data=task_cfg.data_yaml,
            imgsz=task_cfg.imgsz,
            batch=task_cfg.batch,
            workers=task_cfg.workers,
            device=config.device,
            project=project_dir,
        )
        for task_name, task_cfg in config.tasks.items()
    }


class JointHydraTrainer:
    def __init__(self, config: JointTrainConfig) -> None:
        self.config = config
        self.state = TrainState()
        self.run_dir = _prepare_run_dir(config)

        init_seeds(config.seed, deterministic=config.deterministic)
        self.device = select_device(config.device or "")

        self.hydra = Hydra(
            foundation_path=str(config.foundation_path),
            task_dict={
                task_name: str(task_cfg.model_path)
                for task_name, task_cfg in config.tasks.items()
            },
        ).to(self.device)

        stride = int(max(self.hydra.head_strides["detection"]).item())
        self.loader_bundles = _build_task_loaders(config, stride=stride)
        self.interleaver = DynamicInterleavedLoader(
            {
                task_name: bundle.dataloader
                for task_name, bundle in self.loader_bundles.items()
            },
            task_order=list(config.tasks),
        )

        self.task_criteria: dict[TaskName, TaskCriterionAdapter] = {
            task_name: TaskCriterionAdapter(
                hydra_model=self.hydra,
                task_name=task_name,
                epochs=config.epochs,
                end2end=self.hydra.head_end2end.get(task_name, True),
            )
            for task_name in config.tasks
        }

        self.uncertainty = DynamicUncertaintyWeighting(list(config.tasks)).to(
            self.device
        )

        self.backbone_params = [
            p
            for p in self.hydra.shared_backbone.parameters()
            if p.requires_grad
        ]
        self.head_params, self.all_head_params = _collect_task_params(
            self.hydra,
            list(config.tasks),
        )

        self.opt_backbone = _build_optimizer(
            self.backbone_params,
            name=config.optimizer.name,
            lr=config.optimizer.backbone_lr,
            weight_decay=config.optimizer.weight_decay,
            momentum=config.optimizer.momentum,
        )
        self.opt_heads = {
            task_name: _build_optimizer(
                params,
                name=config.optimizer.name,
                lr=config.optimizer.head_lr,
                weight_decay=config.optimizer.weight_decay,
                momentum=config.optimizer.momentum,
            )
            for task_name, params in self.head_params.items()
        }
        self.opt_uncertainty = optim.Adam(
            self.uncertainty.parameters(),
            lr=config.optimizer.uncertainty_lr,
        )

        self.schedulers: dict[str, CosineAnnealingLR] = {
            "backbone": CosineAnnealingLR(
                self.opt_backbone,
                T_max=config.epochs,
                eta_min=config.optimizer.backbone_lr
                * config.scheduler.min_lr_ratio,
            ),
            "uncertainty": CosineAnnealingLR(
                self.opt_uncertainty,
                T_max=config.epochs,
                eta_min=config.optimizer.uncertainty_lr
                * config.scheduler.min_lr_ratio,
            ),
        }
        for task_name, optimizer in self.opt_heads.items():
            self.schedulers[task_name] = CosineAnnealingLR(
                optimizer,
                T_max=config.epochs,
                eta_min=config.optimizer.head_lr
                * config.scheduler.min_lr_ratio,
            )

        amp_enabled = config.amp and self.device.type == "cuda"
        self.amp = amp_enabled
        self.scaler = cast(
            torch.cuda.amp.GradScaler,
            torch.cuda.amp.GradScaler(enabled=amp_enabled),
        )
        self.ema = ModelEMA(self.hydra) if config.use_ema else None

        self.validator = MultiTaskHydraValidator(
            self.hydra,
            device=str(self.device),
        )

        if config.resume_checkpoint:
            self._resume(config.resume_checkpoint)

    def train(self) -> None:
        logger.info(
            "Starting joint Hydra training | epochs=%d | run_dir=%s",
            self.config.epochs,
            self.run_dir,
        )
        for epoch in range(self.state.epoch, self.config.epochs):
            self.state.epoch = epoch
            self.hydra.train()
            epoch_stats = self._train_epoch(epoch)
            self._step_schedulers()
            logger.info(
                "epoch=%d/%d total=%.4f detect=%.4f pose=%.4f "
                "log_var_detect=%.4f log_var_pose=%.4f",
                epoch + 1,
                self.config.epochs,
                epoch_stats["total_loss"],
                epoch_stats["detection_raw_loss"],
                epoch_stats["pose_raw_loss"],
                epoch_stats["log_var_detection"],
                epoch_stats["log_var_pose"],
            )

            should_validate = (epoch + 1) % self.config.validate_every == 0 or (
                epoch + 1
            ) == self.config.epochs
            fitness = float("nan")
            metrics: dict[TaskName, dict[str, float]] = {}
            if should_validate:
                metrics, fitness = self._validate()

            self._save_checkpoint(
                epoch=epoch,
                metrics=metrics,
                fitness=fitness,
            )

    def _train_epoch(self, epoch: int) -> dict[str, float]:
        del epoch
        step_losses: list[float] = []
        step_detection_raw: list[float] = []
        step_pose_raw: list[float] = []

        for step, step_batches in enumerate(
            self.interleaver.iter_epoch(), start=1
        ):
            self._zero_all_grads()
            per_task_losses: dict[TaskName, torch.Tensor] = {}

            for task_name, batch in step_batches.items():
                batch = _to_device_batch(batch, self.device)
                with autocast(enabled=self.amp, device=self.device.type):
                    preds = self.hydra.forward_head(batch["img"], task_name)
                    loss_result = self.task_criteria[task_name](preds, batch)
                    raw_task_loss = loss_result.total_loss
                    weighted_task_loss = self.uncertainty.weighted_loss(
                        task_name,
                        raw_task_loss,
                    )
                self.scaler.scale(weighted_task_loss).backward()
                per_task_losses[task_name] = raw_task_loss.detach()

            self._optimizer_step()
            self.state.global_step += 1

            detect_loss_value = float(per_task_losses["detection"].item())
            pose_loss_value = float(per_task_losses["pose"].item())
            step_detection_raw.append(detect_loss_value)
            step_pose_raw.append(pose_loss_value)
            step_losses.append(detect_loss_value + pose_loss_value)

            if step % self.config.log_every == 0:
                logger.info(
                    "step=%d/%d detect=%.4f pose=%.4f",
                    step,
                    len(self.interleaver),
                    detect_loss_value,
                    pose_loss_value,
                )

        return {
            "total_loss": sum(step_losses) / max(len(step_losses), 1),
            "detection_raw_loss": sum(step_detection_raw)
            / max(len(step_detection_raw), 1),
            "pose_raw_loss": sum(step_pose_raw) / max(len(step_pose_raw), 1),
            "log_var_detection": float(
                self.uncertainty.log_vars["detection"].item()
            ),
            "log_var_pose": float(self.uncertainty.log_vars["pose"].item()),
        }

    def _optimizer_step(self) -> None:
        optimizers = [
            self.opt_backbone,
            *self.opt_heads.values(),
            self.opt_uncertainty,
        ]
        for optimizer in optimizers:
            self.scaler.unscale_(optimizer)

        torch.nn.utils.clip_grad_norm_(
            self.backbone_params,
            max_norm=self.config.clip_grad_norm,
        )
        torch.nn.utils.clip_grad_norm_(
            self.all_head_params,
            max_norm=self.config.clip_grad_norm,
        )

        self.scaler.step(self.opt_backbone)
        for optimizer in self.opt_heads.values():
            self.scaler.step(optimizer)
        self.scaler.step(self.opt_uncertainty)
        self.scaler.update()

        if self.ema is not None:
            self.ema.update(self.hydra)

    def _zero_all_grads(self) -> None:
        self.opt_backbone.zero_grad(set_to_none=True)
        for optimizer in self.opt_heads.values():
            optimizer.zero_grad(set_to_none=True)
        self.opt_uncertainty.zero_grad(set_to_none=True)

    def _step_schedulers(self) -> None:
        for scheduler in self.schedulers.values():
            scheduler.step()

    def _validate(self) -> tuple[dict[TaskName, dict[str, float]], float]:
        model_for_val: Hydra = self.hydra
        if self.ema is not None:
            model_for_val = cast(Hydra, self.ema.ema)
        validator = MultiTaskHydraValidator(
            model_for_val, device=str(self.device)
        )
        task_configs = _build_validation_configs(self.config, self.run_dir)
        validator_task_configs = cast(
            dict[str, ValidationTaskConfig],
            dict(task_configs),
        )
        metrics = cast(
            dict[TaskName, dict[str, float]],
            validator.validate(validator_task_configs),
        )
        if "detection" not in metrics:
            raise MissingTaskMetricsError("detection")
        if "pose" not in metrics:
            raise MissingTaskMetricsError("pose")

        detect = float(metrics["detection"].get("metrics/mAP50-95(B)", 0.0))
        pose = float(metrics["pose"].get("metrics/mAP50-95(P)", 0.0))
        fitness = 0.5 * (detect + pose)
        logger.info(
            "validation fitness=%.4f detect_map=%.4f pose_map=%.4f",
            fitness,
            detect,
            pose,
        )
        return metrics, fitness

    def _save_checkpoint(
        self,
        *,
        epoch: int,
        metrics: dict[TaskName, dict[str, float]],
        fitness: float,
    ) -> None:
        checkpoint = {
            "epoch": epoch,
            "global_step": self.state.global_step,
            "best_fitness": self.state.best_fitness,
            "model": self.hydra.state_dict(),
            "ema": self.ema.ema.state_dict() if self.ema is not None else None,
            "ema_updates": self.ema.updates if self.ema is not None else 0,
            "optimizer_backbone": self.opt_backbone.state_dict(),
            "optimizer_heads": {
                task_name: optimizer.state_dict()
                for task_name, optimizer in self.opt_heads.items()
            },
            "optimizer_uncertainty": self.opt_uncertainty.state_dict(),
            "scheduler": {
                name: scheduler.state_dict()
                for name, scheduler in self.schedulers.items()
            },
            "scaler": self.scaler.state_dict(),
            "uncertainty": self.uncertainty.state_dict(),
            "metrics": metrics,
            "fitness": fitness,
            "train_config": self.config.to_dict(),
            "timestamp": datetime.now().isoformat(),
        }

        last_path = self.run_dir / LAST_FILENAME
        torch.save(checkpoint, last_path)

        if fitness > self.state.best_fitness:
            self.state.best_fitness = fitness
            best_path = self.run_dir / BEST_FILENAME
            torch.save(checkpoint, best_path)

        should_save_period = self.config.save_period > 0 and (
            (epoch + 1) % self.config.save_period == 0
        )
        if should_save_period:
            torch.save(checkpoint, self.run_dir / f"epoch{epoch + 1}.pt")

    def _resume(self, checkpoint_path: Path) -> None:
        checkpoint = torch.load(
            checkpoint_path,
            map_location="cpu",
            weights_only=False,
        )
        self.hydra.load_state_dict(checkpoint["model"])

        ema_state = checkpoint.get("ema")
        if self.ema is not None and ema_state is not None:
            self.ema.ema.load_state_dict(ema_state)
            self.ema.updates = int(checkpoint.get("ema_updates", 0))

        self.opt_backbone.load_state_dict(checkpoint["optimizer_backbone"])
        for raw_task_name, optimizer_state in checkpoint[
            "optimizer_heads"
        ].items():
            if raw_task_name not in self.opt_heads:
                continue
            task_name = cast(TaskName, raw_task_name)
            if task_name not in self.opt_heads:
                raise MissingTaskHeadOptimizerError(task_name)
            self.opt_heads[task_name].load_state_dict(optimizer_state)
        self.opt_uncertainty.load_state_dict(
            checkpoint["optimizer_uncertainty"]
        )

        for name, state in checkpoint["scheduler"].items():
            if name in self.schedulers:
                self.schedulers[name].load_state_dict(state)

        self.scaler.load_state_dict(checkpoint.get("scaler", {}))
        self.uncertainty.load_state_dict(checkpoint["uncertainty"])

        last_epoch = int(checkpoint.get("epoch", -1))
        self.state.epoch = last_epoch + 1
        self.state.global_step = int(checkpoint.get("global_step", 0))
        self.state.best_fitness = float(
            checkpoint.get("best_fitness", float("-inf"))
        )

        logger.info(
            "Resumed from checkpoint %s at epoch %d",
            checkpoint_path,
            self.state.epoch,
        )
