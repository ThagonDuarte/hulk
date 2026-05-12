"""Tests for joint-training optimizer construction."""

from __future__ import annotations

from torch import nn

from model.joint_loop.optim import build_param_groups


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
