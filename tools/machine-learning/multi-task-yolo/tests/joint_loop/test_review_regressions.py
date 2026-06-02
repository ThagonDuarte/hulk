"""Regression tests for code-review findings in joint training."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
from typing import Any, ClassVar

import pytest
import torch
from torch import nn, optim

from model.joint_loop import checkpoints, validation
from model.joint_loop.criteria import JointLossHyp, build_criterion
from model.joint_loop.loop import JointTrainConfig, _step_all_optimizers
from model.joint_loop.optim import JointOptimizers, head_last_layer_index
from utils.model_naming import HydraModelName, TaskType


class _FakeYOLO:
    saved_children: ClassVar[list[nn.Module]] = []

    def __init__(self, _path: str) -> None:
        self.model = SimpleNamespace(
            model=nn.Sequential(nn.Identity(), nn.Linear(1, 1))
        )

    def save(self, path: str) -> None:
        _FakeYOLO.saved_children = list(self.model.model.children())
        Path(path).write_bytes(b"fake checkpoint")


def test_per_task_checkpoint_saves_trained_ema_head(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    ema_backbone = nn.Identity()
    ema_head = nn.Linear(1, 1)
    ema = SimpleNamespace(
        hydra=SimpleNamespace(
            shared_backbone=nn.ModuleList([ema_backbone]),
            backbone_length=1,
            heads={str(TaskType.OBJECT): nn.ModuleList([ema_head])},
        )
    )
    monkeypatch.setattr(checkpoints, "YOLO", _FakeYOLO)

    checkpoints.write_per_task_checkpoint(
        ema=ema,
        hydra_model=HydraModelName("yolo26m", ["yolo26m"], 1),
        task=TaskType.OBJECT,
        head_yolo_path=tmp_path / "source.pt",
        output_path=tmp_path / "last.pt",
    )

    assert _FakeYOLO.saved_children == [ema_backbone, ema_head]


def test_validation_materializes_trained_checkpoint_for_head_and_backbone(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    source = tmp_path / "pose-source.pt"
    source.write_bytes(b"source")
    hydra_model = HydraModelName("yolo26m", ["yolo26m-pose"], 1)

    def fake_write_per_task_checkpoint(
        *, output_path: Path, **_kwargs: Any
    ) -> None:
        output_path.write_bytes(b"trained")

    def fake_validate_hydra_model(
        _hydra_model: HydraModelName,
        _config: Any,
        assets_dir: Path,
        **_kwargs: Any,
    ) -> None:
        assert (assets_dir / "yolo26m.pt").read_bytes() == b"trained"
        assert (assets_dir / "yolo26m-pose.pt").read_bytes() == b"trained"

    monkeypatch.setattr(
        validation,
        "write_per_task_checkpoint",
        fake_write_per_task_checkpoint,
    )
    monkeypatch.setattr(
        validation, "validate_hydra_model", fake_validate_hydra_model
    )
    monkeypatch.setattr(validation, "_read_primary_metric", lambda *_: 0.5)

    score, metrics, all_metrics, task_visuals = validation.run_validation(
        ema=SimpleNamespace(),
        hydra_model=hydra_model,
        datasets_per_task={TaskType.POSE: tmp_path / "data.yaml"},
        head_source_paths={TaskType.POSE: source},
        run_dir=tmp_path / "runs",
        imgsz=320,
        batch=1,
        device="cpu",
        task_weights={TaskType.POSE: 1.0},
    )

    assert score == 0.5
    assert metrics == {TaskType.POSE: 0.5}
    metric_key = validation._PRIMARY_METRIC_KEY[TaskType.POSE]
    assert all_metrics == {TaskType.POSE: {metric_key: 0.5}}
    assert task_visuals == {TaskType.POSE: []}


class _FakeSegmentHead(nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.weight = nn.Parameter(torch.ones(1))
        self.stride = torch.tensor([8.0, 16.0, 32.0])
        self.nc = 1
        self.reg_max = 1


class _FakeSegHydra(nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.heads = nn.ModuleDict(
            {str(TaskType.SEGMENTATION): nn.ModuleList([_FakeSegmentHead()])}
        )
        self.head_strides = {
            str(TaskType.SEGMENTATION): torch.tensor([8.0, 16.0, 32.0])
        }
        self.head_class_names = {str(TaskType.SEGMENTATION): {0: "class"}}
        self.head_kpt_shapes = {str(TaskType.SEGMENTATION): None}
        self.head_end2end = {str(TaskType.SEGMENTATION): False}


def test_segmentation_criterion_has_overlap_mask_arg() -> None:
    criterion = build_criterion(
        _FakeSegHydra(), TaskType.SEGMENTATION, JointLossHyp()
    )

    assert criterion.overlap is True


def test_head_last_layer_index_is_relative_to_sliced_head() -> None:
    hydra = SimpleNamespace(
        backbone_length=11,
        heads={str(TaskType.OBJECT): nn.ModuleList([nn.Identity()] * 13)},
    )

    assert head_last_layer_index(hydra, TaskType.OBJECT) == 12


def test_segmentation_primary_metric_uses_mask_map() -> None:
    assert (
        validation._PRIMARY_METRIC_KEY[TaskType.SEGMENTATION]
        == "metrics/mAP50-95(M)"
    )


class _CountingOptimizer(optim.Optimizer):
    def __init__(self, param: nn.Parameter) -> None:
        super().__init__([param], {})
        self.steps = 0

    def step(self, closure: Any = None) -> None:
        self.steps += 1
        if closure is not None:
            closure()


def test_step_all_optimizers_skips_optimizers_without_gradients() -> None:
    backbone_param = nn.Parameter(torch.ones(1))
    head_param = nn.Parameter(torch.ones(1))
    log_var = nn.Parameter(torch.ones(1))
    backbone_param.grad = torch.ones_like(backbone_param)
    head_param.grad = torch.ones_like(head_param)

    backbone_opt = _CountingOptimizer(backbone_param)
    head_opt = _CountingOptimizer(head_param)
    logvar_opt = _CountingOptimizer(log_var)
    hydra = SimpleNamespace(
        shared_backbone=nn.ModuleList([nn.Linear(1, 1)]),
    )

    _step_all_optimizers(
        optimizers=JointOptimizers(
            backbone=backbone_opt,
            heads={TaskType.OBJECT: head_opt},
            log_var=logvar_opt,
        ),
        scaler=torch.amp.GradScaler(enabled=False),
        hydra=hydra,
        config=JointTrainConfig(clip_heads=True),
    )

    assert backbone_opt.steps == 1
    assert head_opt.steps == 1
    assert logvar_opt.steps == 0


def test_step_all_optimizers_scales_only_backbone_gradients() -> None:
    backbone = nn.Linear(1, 1, bias=False)
    head_param = nn.Parameter(torch.ones(1))
    log_var = nn.Parameter(torch.ones(1))
    assert backbone.weight is not None
    backbone.weight.grad = torch.full_like(backbone.weight, 2.0)
    head_param.grad = torch.full_like(head_param, 2.0)

    backbone_opt = _CountingOptimizer(backbone.weight)
    head_opt = _CountingOptimizer(head_param)
    logvar_opt = _CountingOptimizer(log_var)
    hydra = SimpleNamespace(shared_backbone=nn.ModuleList([backbone]))

    _step_all_optimizers(
        optimizers=JointOptimizers(
            backbone=backbone_opt,
            heads={TaskType.OBJECT: head_opt},
            log_var=logvar_opt,
        ),
        scaler=torch.amp.GradScaler(enabled=False),
        hydra=hydra,
        config=JointTrainConfig(clip_heads=True, max_grad_norm=100.0),
        backbone_grad_scale=0.5,
    )

    assert torch.allclose(
        backbone.weight.grad, torch.full_like(backbone.weight, 1.0)
    )
    assert torch.allclose(head_param.grad, torch.full_like(head_param, 2.0))


def test_log_wandb_epoch() -> None:
    from model.joint_loop.loop import _log_wandb_epoch
    from model.joint_loop.optim import JointOptimizers
    from model.joint_loop.weighting import UncertaintyWeighter

    logged_payloads = []
    logged_kwargs = []

    class FakeWandbRun:
        def log(self, payload: dict[str, Any], **kwargs: Any) -> None:
            logged_payloads.append(payload)
            logged_kwargs.append(kwargs)

    backbone_opt = SimpleNamespace(param_groups=[{"lr": 0.001}])
    head_opt = SimpleNamespace(param_groups=[{"lr": 0.01}])
    logvar_opt = SimpleNamespace(param_groups=[{"lr": 0.0001}])
    optimizers = JointOptimizers(
        backbone=backbone_opt,
        heads={TaskType.OBJECT: head_opt},
        log_var=logvar_opt,
    )
    weighter = UncertaintyWeighter([TaskType.OBJECT])
    train_metrics = {
        f"loss/{TaskType.OBJECT}": 1.5,
        f"loss/{TaskType.OBJECT}/box_loss": 0.5,
    }

    _log_wandb_epoch(
        epoch=1,
        global_step=20,
        optimizers=optimizers,
        weighter=weighter,
        tasks=[TaskType.OBJECT],
        train_metrics=train_metrics,
        wandb_run=FakeWandbRun(),
    )

    assert len(logged_payloads) == 1
    assert logged_kwargs == [{"commit": True}]
    payload = logged_payloads[0]
    assert payload["epoch"] == 1
    assert payload["global_step"] == 20
    assert "local_step" not in payload
    assert "step" not in payload
    assert payload["lr/backbone"] == 0.001
    assert payload[f"loss/{TaskType.OBJECT}"] == 1.5
    assert payload[f"loss/{TaskType.OBJECT}/box_loss"] == 0.5
