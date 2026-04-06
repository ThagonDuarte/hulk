from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import torch
from torch import nn
from ultralytics.cfg import get_cfg
from ultralytics.utils.loss import (
    E2ELoss,
    PoseLoss26,
    v8DetectionLoss,
    v8PoseLoss,
)

from model.hydra import Hydra
from model.joint_config import TaskName


class UnsupportedTaskCriterionError(ValueError):
    def __init__(self, task_name: TaskName) -> None:
        super().__init__(f"Unsupported task '{task_name}'")


@dataclass(frozen=True)
class TaskLossResult:
    total_loss: torch.Tensor
    loss_items: torch.Tensor


class _HydraLossModelProxy(nn.Module):
    def __init__(
        self,
        *,
        hydra_model: Hydra,
        head_name: TaskName,
        epochs: int,
    ) -> None:
        super().__init__()
        self.hydra = hydra_model
        self.model = hydra_model.heads[head_name]
        self.args = get_cfg(overrides={"epochs": epochs})


class TaskCriterionAdapter:
    def __init__(
        self,
        *,
        hydra_model: Hydra,
        task_name: TaskName,
        epochs: int,
        end2end: bool,
    ) -> None:
        self.task_name = task_name
        self.loss_proxy = _HydraLossModelProxy(
            hydra_model=hydra_model,
            head_name=task_name,
            epochs=epochs,
        )
        self.criterion = self._build_criterion(
            task_name=task_name,
            end2end=end2end,
            loss_proxy=self.loss_proxy,
        )

    def __call__(
        self, preds: Any, batch: dict[str, torch.Tensor]
    ) -> TaskLossResult:
        total_loss, loss_items = self.criterion(preds, batch)
        return TaskLossResult(total_loss=total_loss, loss_items=loss_items)

    @staticmethod
    def _build_criterion(
        *,
        task_name: TaskName,
        end2end: bool,
        loss_proxy: _HydraLossModelProxy,
    ) -> v8DetectionLoss | v8PoseLoss | E2ELoss:
        if task_name == "detection":
            if end2end:
                return E2ELoss(loss_proxy)
            return v8DetectionLoss(loss_proxy)

        if task_name == "pose":
            if end2end:
                return E2ELoss(loss_proxy, PoseLoss26)
            return v8PoseLoss(loss_proxy)

        raise UnsupportedTaskCriterionError(task_name)


class DynamicUncertaintyWeighting(nn.Module):
    def __init__(self, task_names: list[TaskName]) -> None:
        super().__init__()
        self.log_vars = nn.ParameterDict(
            {
                task_name: nn.Parameter(torch.zeros(()))
                for task_name in task_names
            }
        )

    def weighted_loss(
        self,
        task_name: TaskName,
        task_loss: torch.Tensor,
    ) -> torch.Tensor:
        log_var = self.log_vars[task_name]
        precision = torch.exp(-log_var)
        return 0.5 * precision * task_loss + 0.5 * log_var
