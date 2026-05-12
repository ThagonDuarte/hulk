"""Optimizer / scheduler / EMA / AMP wiring for joint Hydra training."""

from __future__ import annotations

import re
from collections.abc import Iterable
from typing import Any

from torch import nn

# Norm layer types - same probe Ultralytics' BaseTrainer uses.
_BN_TYPES: tuple[type, ...] = tuple(
    v for k, v in nn.__dict__.items() if "Norm" in k and isinstance(v, type)
)


def _cv3_proto_regex(head_last_layer_index: int) -> re.Pattern[str]:
    """Build the cv3/proto LR-boost regex for a given head-layer index.

    Mirrors Ultralytics' hardcoded ``(?=.*23)(?=.*cv3)|proto\\.semseg`` but
    with the layer index parameterized so non-yolo26m scales also match.
    """
    return re.compile(rf"(?=.*{head_last_layer_index})(?=.*cv3)|proto\.semseg")


def build_param_groups(
    module: nn.Module,
    *,
    optimizer_name: str,
    lr: float,
    momentum: float,
    decay: float,
    head_last_layer_index: int,
) -> list[dict[str, Any]]:
    """Construct optimizer parameter groups for a single module.

    Mirrors `BaseTrainer.build_optimizer`'s 4-group split when
    ``optimizer_name == "MuSGD"`` (g0=weights+decay, g1=norm/no-decay,
    g2=bias/no-decay, g3=muon-eligible weights). For AdamW (or any other
    non-MuSGD optimizer), g3 is collapsed back into g0 - the muon group
    only makes sense for Newton-Schulz orthogonalization.

    The cv3/proto regex (parameterized by ``head_last_layer_index``) splits
    every group into two sub-groups: an ``lr*3`` boosted sub-group for
    parameters whose fully-qualified name matches the regex, and the
    baseline sub-group for everything else.
    """
    use_muon = optimizer_name == "MuSGD"
    g_decay: dict[str, nn.Parameter] = {}
    g_norm: dict[str, nn.Parameter] = {}
    g_bias: dict[str, nn.Parameter] = {}
    g_muon: dict[str, nn.Parameter] = {}

    for module_name, sub in module.named_modules():
        for param_name, param in sub.named_parameters(recurse=False):
            full = f"{module_name}.{param_name}" if module_name else param_name
            if param.ndim >= 2 and use_muon:
                g_muon[full] = param
            elif "bias" in full:
                g_bias[full] = param
            elif isinstance(sub, _BN_TYPES) or "logit_scale" in full:
                g_norm[full] = param
            else:
                g_decay[full] = param

    optim_args = _optim_args(optimizer_name, lr=lr, momentum=momentum)

    groups: list[dict[str, Any]] = [
        {
            "params": g_decay,
            "weight_decay": decay,
            "param_group": "weight",
            **optim_args,
        },
        {
            "params": g_norm,
            "weight_decay": 0.0,
            "param_group": "bn",
            **optim_args,
        },
        {
            "params": g_bias,
            "weight_decay": 0.0,
            "param_group": "bias",
            **optim_args,
        },
    ]
    if use_muon:
        groups.append(
            {
                "params": g_muon,
                "weight_decay": decay,
                "use_muon": True,
                "param_group": "muon",
                **optim_args,
            }
        )

    pattern = _cv3_proto_regex(head_last_layer_index)
    boosted: list[dict[str, Any]] = []
    for group in groups:
        named = group.pop("params")
        p1 = [p for k, p in named.items() if pattern.search(k)]
        p2 = [p for k, p in named.items() if not pattern.search(k)]
        # boosted (lr*3)
        boosted.append({**group, "params": p1, "lr": lr * 3})
        # baseline (lr*1)
        boosted.append({**group, "params": p2})
    return boosted


def _optim_args(name: str, *, lr: float, momentum: float) -> dict[str, Any]:
    if name in {"Adam", "Adamax", "AdamW", "NAdam", "RAdam"}:
        return {"lr": lr, "betas": (momentum, 0.999), "weight_decay": 0.0}
    if name == "RMSProp":
        return {"lr": lr, "momentum": momentum}
    if name in {"SGD", "MuSGD"}:
        return {"lr": lr, "momentum": momentum, "nesterov": True}
    raise NotImplementedError(f"unsupported optimizer name: {name!r}")


def parameters_in_optimizer(opt: Any) -> Iterable[nn.Parameter]:
    """Flatten an optimizer's param groups into an iterable of parameters."""
    for group in opt.param_groups:
        yield from group["params"]
