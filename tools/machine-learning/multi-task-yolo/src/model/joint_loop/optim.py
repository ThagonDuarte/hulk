"""Optimizer / scheduler / EMA / AMP wiring for joint Hydra training."""

from __future__ import annotations

import copy
import math
import re
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from functools import partial
from typing import Any

import torch
from torch import nn, optim
from ultralytics.optim.muon import MuSGD

from model.hydra import Hydra
from model.joint_loop.weighting import UncertaintyWeighter
from utils.model_naming import TaskType

# Norm layer types - same probe Ultralytics' BaseTrainer uses.
_BN_TYPES: tuple[type, ...] = tuple(
    v for k, v in nn.__dict__.items() if "Norm" in k and isinstance(v, type)
)


def _cv3_proto_regex(head_last_layer_index: int) -> re.Pattern[str]:
    """Build the cv3/proto LR-boost regex for a given head-layer index.

    Mirrors Ultralytics' hardcoded ``(?=.*23)(?=.*cv3)|proto\\.semseg`` but
    with the layer index parameterized so non-yolo26m scales also match.
    """
    return re.compile(
        rf"(?:^|[\._]){head_last_layer_index}\.(?=cv3)|proto\.semseg"
    )


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
        boosted.append({**group, "params": p1, "lr": lr * 3})
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


# ---------------------------------------------------------------------------
# Joint optimizer / scheduler / EMA / AMP wiring
# ---------------------------------------------------------------------------


@dataclass
class JointOptimizers:
    backbone: optim.Optimizer
    heads: dict[TaskType, optim.Optimizer]
    log_var: optim.Optimizer

    def all(self) -> list[optim.Optimizer]:
        return [self.backbone, *self.heads.values(), self.log_var]


def _instantiate_optimizer(
    name: str, groups: list[dict[str, Any]]
) -> optim.Optimizer:
    if name == "MuSGD":
        return MuSGD(params=groups, muon=0.2, sgd=1.0)
    if not hasattr(optim, name):
        raise NotImplementedError(f"unsupported optimizer: {name!r}")
    return getattr(optim, name)(groups)


def head_last_layer_index(hydra: Hydra, task: TaskType) -> int:
    """Index of the head's final module within the sliced head module."""
    head = hydra.heads[str(task)]
    return len(head) - 1


def build_joint_optimizers(
    hydra: Hydra,
    weighter: UncertaintyWeighter,
    *,
    optimizer_name: str,
    lr_backbone: float,
    lr_heads: float,
    lr_logvar: float,
    momentum: float,
    weight_decay: float,
) -> JointOptimizers:
    """Build the per-module optimizer set per the spec (Section 7.2).

    `opt_logvar` is always AdamW (Newton-Schulz needs ndim >= 2).
    """
    backbone_groups = build_param_groups(
        hydra.shared_backbone,
        optimizer_name=optimizer_name,
        lr=lr_backbone,
        momentum=momentum,
        decay=weight_decay,
        # The backbone has no cv3 final-layer to boost; pass an unmatchable
        # index so the regex never fires for the backbone module.
        head_last_layer_index=-1,
    )
    opt_backbone = _instantiate_optimizer(optimizer_name, backbone_groups)

    heads: dict[TaskType, optim.Optimizer] = {}
    for task_str, head_module in hydra.heads.items():
        task = TaskType(task_str)
        groups = build_param_groups(
            head_module,
            optimizer_name=optimizer_name,
            lr=lr_heads,
            momentum=momentum,
            decay=weight_decay,
            head_last_layer_index=head_last_layer_index(hydra, task),
        )
        heads[task] = _instantiate_optimizer(optimizer_name, groups)

    opt_logvar = optim.AdamW([weighter.log_var], lr=lr_logvar, weight_decay=0.0)
    return JointOptimizers(
        backbone=opt_backbone, heads=heads, log_var=opt_logvar
    )


def build_schedulers(
    optimizers: JointOptimizers,
    *,
    epochs: int,
    warmup_epochs: int,
) -> list[optim.lr_scheduler.LRScheduler]:
    """Build per-optimizer LR schedulers (linear warmup + cosine decay).

    ``LambdaLR`` calls ``step()`` on construction (last_epoch -1 → 0), so
    the warmup factor is applied to the optimizer before epoch-0 training.
    """
    schedulers: list[optim.lr_scheduler.LRScheduler] = []
    for opt in optimizers.all():
        sched = optim.lr_scheduler.LambdaLR(
            opt,
            lr_lambda=partial(
                _lr_schedule, warmup=max(warmup_epochs, 1), total=epochs
            ),
        )
        schedulers.append(sched)
    return schedulers


def _lr_schedule(epoch: int, *, warmup: int, total: int) -> float:
    """Linear warmup (epochs 0..warmup-1) then cosine decay to 0.

    ``ChainedScheduler([LambdaLR, CosineAnnealingLR])`` does NOT compose —
    the cosine scheduler overwrites the warmup LR on every step.  A single
    ``LambdaLR`` with this combined function is the correct approach.
    """
    if epoch < warmup:
        return (epoch + 1) / warmup
    progress = (epoch - warmup) / max(total - warmup, 1)
    return 0.5 * (1.0 + math.cos(math.pi * progress))


class EMAHydra:
    """Exponential moving average of a Hydra + UncertaintyWeighter pair.

    Decay schedule matches Ultralytics' `ModelEMA`:

        decay(step) = 0.9999 * (1 - exp(-step / 2000))
    """

    def __init__(
        self,
        hydra: Hydra,
        weighter: UncertaintyWeighter,
        *,
        max_decay: float = 0.9999,
        warmup: float = 2000.0,
    ) -> None:
        self.hydra = copy.deepcopy(hydra).eval()
        self.weighter = copy.deepcopy(weighter).eval()
        for p in self.hydra.parameters():
            p.requires_grad_(requires_grad=False)
        for p in self.weighter.parameters():
            p.requires_grad_(requires_grad=False)
        self._max_decay = max_decay
        self._warmup = warmup
        self.updates = 0

    def decay(self) -> float:
        return self._max_decay * (1.0 - math.exp(-self.updates / self._warmup))

    @torch.no_grad()
    def update(self, hydra: Hydra, weighter: UncertaintyWeighter) -> None:
        self.updates += 1
        d = self.decay()
        for ema_p, p in zip(
            self.hydra.parameters(), hydra.parameters(), strict=True
        ):
            ema_p.mul_(d).add_(p.detach(), alpha=1.0 - d)
        for ema_p, p in zip(
            self.weighter.parameters(),
            weighter.parameters(),
            strict=True,
        ):
            ema_p.mul_(d).add_(p.detach(), alpha=1.0 - d)

    def state_dict(self) -> dict[str, Any]:
        return {
            "updates": self.updates,
            "hydra": self.hydra.state_dict(),
            "weighter": self.weighter.state_dict(),
        }

    def load_state_dict(self, state: Mapping[str, Any]) -> None:
        self.updates = int(state["updates"])
        self.hydra.load_state_dict(state["hydra"])
        self.weighter.load_state_dict(state["weighter"])


def make_amp_scaler(
    *, enabled: bool, device_type: str = "cuda"
) -> torch.amp.GradScaler:
    return torch.amp.GradScaler(device_type, enabled=enabled)
