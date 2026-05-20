"""Tests for the Kendall-Gal uncertainty weighter."""

from __future__ import annotations

import math

import pytest
import torch

from model.joint_loop.weighting import UncertaintyWeighter
from utils.model_naming import TaskType


def test_log_var_initial_zero() -> None:
    weighter = UncertaintyWeighter([TaskType.OBJECT, TaskType.POSE])
    assert torch.equal(weighter.log_var, torch.zeros(2))


def test_init_log_var_overrides() -> None:
    weighter = UncertaintyWeighter(
        [TaskType.OBJECT, TaskType.POSE],
        init_log_var={TaskType.POSE: 1.5},
    )
    assert weighter.log_var[0].item() == pytest.approx(0.0)
    assert weighter.log_var[1].item() == pytest.approx(1.5)


def test_forward_matches_formula() -> None:
    weighter = UncertaintyWeighter([TaskType.OBJECT, TaskType.POSE])
    losses = {
        TaskType.OBJECT: torch.tensor(2.0),
        TaskType.POSE: torch.tensor(3.0),
    }
    total = weighter(losses)
    # log_var=0 -> exp(0)*2 + 0 + exp(0)*3 + 0 = 5.0
    assert total.item() == pytest.approx(5.0)


def test_forward_respects_log_var() -> None:
    weighter = UncertaintyWeighter([TaskType.OBJECT, TaskType.POSE])
    with torch.no_grad():
        weighter.log_var.copy_(torch.tensor([0.0, 1.0]))
    losses = {
        TaskType.OBJECT: torch.tensor(2.0),
        TaskType.POSE: torch.tensor(3.0),
    }
    expected = math.exp(0.0) * 2.0 + 0.0 + math.exp(-1.0) * 3.0 + 1.0
    total = weighter(losses)
    assert total.item() == pytest.approx(expected, rel=1e-6)


def test_weight_single_sums_to_forward() -> None:
    weighter = UncertaintyWeighter([TaskType.OBJECT, TaskType.POSE])
    with torch.no_grad():
        weighter.log_var.copy_(torch.tensor([0.5, -0.2]))
    losses = {
        TaskType.OBJECT: torch.tensor(2.5),
        TaskType.POSE: torch.tensor(0.7),
    }
    summed = weighter.weight_single(
        TaskType.OBJECT, losses[TaskType.OBJECT]
    ) + weighter.weight_single(TaskType.POSE, losses[TaskType.POSE])
    assert summed.item() == pytest.approx(weighter(losses).item(), rel=1e-6)


def test_log_var_receives_gradient() -> None:
    weighter = UncertaintyWeighter([TaskType.OBJECT, TaskType.POSE])
    weighter.log_var.requires_grad_(requires_grad=True)
    loss_a = torch.tensor(2.0, requires_grad=True)
    loss_b = torch.tensor(3.0, requires_grad=True)
    total = weighter({TaskType.OBJECT: loss_a, TaskType.POSE: loss_b})
    total.backward()
    assert weighter.log_var.grad is not None
    assert weighter.log_var.grad.shape == (2,)


def test_unknown_task_raises() -> None:
    weighter = UncertaintyWeighter([TaskType.OBJECT])
    with pytest.raises(KeyError):
        weighter.weight_single(TaskType.POSE, torch.tensor(1.0))


def test_log_var_clamping() -> None:
    weighter = UncertaintyWeighter([TaskType.OBJECT])
    # Set log_var to a value far outside [-5, 5] range
    with torch.no_grad():
        weighter.log_var.copy_(torch.tensor([10.0]))
    
    loss = torch.tensor(2.0)
    # Without clamping: exp(-10)*2 + 10 ~= 10.00009
    # With clamping: exp(-5)*2 + 5 ~= 0.00673*2 + 5 = 5.01347
    result = weighter.weight_single(TaskType.OBJECT, loss)
    assert result.item() == pytest.approx(math.exp(-5.0) * 2.0 + 5.0, rel=1e-6)
