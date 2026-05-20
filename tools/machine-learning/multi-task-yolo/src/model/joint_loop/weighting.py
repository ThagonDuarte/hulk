"""Kendall-Gal homoscedastic uncertainty weighter for multi-task losses."""

from __future__ import annotations

from collections.abc import Iterable, Mapping

import torch
from torch import Tensor, nn

from utils.model_naming import TaskType


class UnknownTaskError(KeyError):
    def __init__(self, task: TaskType, known: Iterable[TaskType]) -> None:
        super().__init__(
            f"task {task!r} not registered in weighter; known={list(known)}"
        )


class UncertaintyWeighter(nn.Module):
    """Learnable per-task log-variance balancing of multi-task losses.

    Implements Kendall, Gal & Cipolla (2018) homoscedastic uncertainty:

        total = sum_t( exp(-log_var_t) * loss_t + log_var_t )

    The `+log_var_t` term penalises growing the variance, balancing the
    `exp(-log_var_t)` term that down-weights losses with large variance.
    """

    def __init__(
        self,
        tasks: list[TaskType],
        init_log_var: Mapping[TaskType, float] | None = None,
    ) -> None:
        super().__init__()
        self.tasks = list(tasks)
        self._index = {task: i for i, task in enumerate(self.tasks)}
        log_var = torch.zeros(len(self.tasks))
        if init_log_var:
            for task, value in init_log_var.items():
                if task not in self._index:
                    raise UnknownTaskError(task, self.tasks)
                log_var[self._index[task]] = value
        self.log_var = nn.Parameter(log_var)

    def weight_single(self, task: TaskType, loss: Tensor) -> Tensor:
        if task not in self._index:
            raise UnknownTaskError(task, self.tasks)
        log_var_t = self.log_var[self._index[task]]
        # Clamp log_var to a safe range to prevent numerical instability
        log_var_t_clamped = torch.clamp(log_var_t, -5.0, 5.0)
        return torch.exp(-log_var_t_clamped) * loss + log_var_t_clamped

    def forward(self, losses: Mapping[TaskType, Tensor]) -> Tensor:
        total = torch.zeros((), device=self.log_var.device)
        for task, loss in losses.items():
            total = total + self.weight_single(task, loss)
        return total
