"""Tests for joint-training optimizer construction."""

from __future__ import annotations

import pytest
import torch
from torch import nn

from model.joint_loop.optim import _ema_update_state_dict, build_param_groups


class _FakeBackbone(nn.Module):
    """Tiny module covering all four MuSGD param-group cases."""

    def __init__(self) -> None:
        super().__init__()
        # 2D weight + decay -> g3 (muon) when use_muon, else g0
        self.linear = nn.Linear(4, 4, bias=True)
        # BatchNorm provides "norm" weight (1D) -> g1, and a bias -> g2
        self.bn = nn.BatchNorm1d(4)


def test_4_group_split_with_musgd() -> None:
    module = _FakeBackbone()
    groups = build_param_groups(
        module,
        optimizer_name="MuSGD",
        lr=0.01,
        momentum=0.9,
        decay=1e-5,
        head_last_layer_index=23,
    )
    # We expect muon and non-muon halves; cv3/proto regex shouldn't match
    # anything in this fake module so the x3 sub-groups have empty params.
    flat_params = {id(p) for g in groups for p in g["params"]}
    expected_params = {id(p) for p in module.parameters()}
    assert flat_params == expected_params, (
        "every parameter must end up in a group"
    )


def test_3_group_split_with_adamw() -> None:
    module = _FakeBackbone()
    groups = build_param_groups(
        module,
        optimizer_name="AdamW",
        lr=0.01,
        momentum=0.9,
        decay=1e-5,
        head_last_layer_index=23,
    )
    # AdamW collapses the muon group back into g0 - still all params covered
    flat_params = {id(p) for g in groups for p in g["params"]}
    expected_params = {id(p) for p in module.parameters()}
    assert flat_params == expected_params

    # And no group should carry the use_muon flag
    for g in groups:
        assert "use_muon" not in g or g["use_muon"] is False


class _FakeHeadWithCv3(nn.Module):
    """Mimics a YOLO26 head whose final block (index 5) contains cv3."""

    def __init__(self) -> None:
        super().__init__()
        self.layer_5 = _Cv3Block()


class _Cv3Block(nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.cv3 = nn.Conv2d(4, 4, 1)


def test_cv3_x3_sub_split_with_parameterized_index() -> None:
    head = _FakeHeadWithCv3()
    groups = build_param_groups(
        head,
        optimizer_name="MuSGD",
        lr=0.01,
        momentum=0.9,
        decay=1e-5,
        head_last_layer_index=5,
    )
    boosted = [g for g in groups if g.get("lr", 0) == 0.03]
    boosted_params = [p for g in boosted for p in g["params"]]
    cv3_params = list(head.layer_5.cv3.parameters())
    assert any(id(p) in {id(q) for q in boosted_params} for p in cv3_params), (
        "cv3 params under layer index 5 should land in the lr*3 sub-group"
    )


class _FakeDetectLikeFinal(nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.cv3 = nn.Conv2d(4, 4, 1)
        self.one2one_cv3 = nn.Conv2d(4, 4, 1)


def test_cv3_x3_sub_split_includes_one2one_cv3() -> None:
    head = nn.ModuleList(
        [nn.Identity() for _ in range(5)] + [_FakeDetectLikeFinal()]
    )
    groups = build_param_groups(
        head,
        optimizer_name="MuSGD",
        lr=0.01,
        momentum=0.9,
        decay=1e-5,
        head_last_layer_index=5,
    )
    boosted_params = {
        id(p) for g in groups if g.get("lr", 0) == 0.03 for p in g["params"]
    }

    final = head[5]
    assert isinstance(final, _FakeDetectLikeFinal)
    assert all(id(p) in boosted_params for p in final.cv3.parameters())
    assert all(id(p) in boosted_params for p in final.one2one_cv3.parameters())


class _FakeHeadWithCv3MultipleLayers(nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.layer_5 = _Cv3Block()
        self.layer_15 = _Cv3Block()


def test_cv3_x3_sub_split_bounds_exact_index() -> None:
    head = _FakeHeadWithCv3MultipleLayers()
    groups = build_param_groups(
        head,
        optimizer_name="MuSGD",
        lr=0.01,
        momentum=0.9,
        decay=1e-5,
        head_last_layer_index=5,
    )
    boosted = [g for g in groups if g.get("lr", 0) == 0.03]
    boosted_params = {id(p) for g in boosted for p in g["params"]}

    # Layer 5 parameters should be boosted
    layer_5_params = list(head.layer_5.cv3.parameters())
    assert all(id(p) in boosted_params for p in layer_5_params), (
        "layer_5 cv3 params should land in the lr*3 sub-group"
    )

    # Layer 15 parameters should NOT be boosted
    layer_15_params = list(head.layer_15.cv3.parameters())
    assert not any(id(p) in boosted_params for p in layer_15_params), (
        "layer_15 cv3 params should NOT land in the lr*3 sub-group"
    )


def test_lr_schedule() -> None:
    from model.joint_loop.optim import _lr_schedule

    # Linear warmup: factor ramps 1/warmup → 1.0
    assert _lr_schedule(0, warmup=3, total=100) == pytest.approx(1.0 / 3.0)
    assert _lr_schedule(1, warmup=3, total=100) == pytest.approx(2.0 / 3.0)
    assert _lr_schedule(2, warmup=3, total=100) == pytest.approx(1.0)
    # Cosine decay: immediately after warmup factor == 1.0; at end == 0.0
    assert _lr_schedule(3, warmup=3, total=100) == pytest.approx(1.0)
    assert _lr_schedule(100, warmup=3, total=100) == pytest.approx(0.0)


def test_backbone_gradient_accumulation_invariant() -> None:
    """A synchronized step must accumulate per-task grads on the backbone."""
    backbone = nn.Linear(4, 4, bias=False)
    head_a = nn.Linear(4, 1, bias=False)
    head_b = nn.Linear(4, 1, bias=False)

    x = torch.randn(2, 4)

    # Reference: compute each task's backbone grad in isolation, sum them.
    manual_grads = torch.zeros_like(backbone.weight)
    for head in (head_a, head_b):
        backbone.zero_grad(set_to_none=True)
        head.zero_grad(set_to_none=True)
        out = head(backbone(x)).sum()
        out.backward()
        assert backbone.weight.grad is not None
        manual_grads = manual_grads + backbone.weight.grad.detach().clone()

    # Joint accumulation: zero backbone once, run both backwards back-to-back,
    # zero each head between calls but NOT the backbone.
    backbone.zero_grad(set_to_none=True)
    for head in (head_a, head_b):
        head.zero_grad(set_to_none=True)
        out = head(backbone(x)).sum()
        (out / 2.0).backward()

    assert backbone.weight.grad is not None
    assert torch.allclose(
        backbone.weight.grad, manual_grads / 2.0, atol=1e-6
    ), "synchronized backbone grad must equal average of single-task grads"


def test_ema_update_state_dict_updates_float_buffers() -> None:
    live = nn.BatchNorm1d(2)
    ema = nn.BatchNorm1d(2)

    live.running_mean.fill_(4.0)
    live.running_var.fill_(9.0)
    ema.running_mean.zero_()
    ema.running_var.fill_(1.0)

    _ema_update_state_dict(ema, live, decay=0.5)

    assert torch.allclose(
        ema.running_mean, torch.full_like(ema.running_mean, 2.0)
    )
    assert torch.allclose(
        ema.running_var, torch.full_like(ema.running_var, 5.0)
    )
