import pytest
import torch

from model import joint_train


class _FakeWandbRun:
    def __init__(self) -> None:
        self.calls: list[tuple[str, dict[str, object]]] = []

    def define_metric(self, name: str, **kwargs: object) -> None:
        self.calls.append((name, kwargs))


class _FakeGPUInfo:
    def select_idle_gpu(
        self,
        *,
        count: int,
        min_memory_fraction: float,
    ) -> list[int]:
        assert count == 1
        assert min_memory_fraction == 0.2
        return [1]


def test_configure_wandb_metrics_uses_hidden_epoch_axis() -> None:
    run = _FakeWandbRun()

    joint_train._configure_wandb_metrics(run)

    assert run.calls[0] == (
        "epoch",
        {"hidden": True, "summary": "none", "overwrite": True},
    )
    assert run.calls[-1] == run.calls[0]
    axis_calls = dict(run.calls[1:-1])
    assert axis_calls["*"] == {
        "step_metric": "epoch",
        "step_sync": True,
        "overwrite": True,
    }
    assert axis_calls["loss/*"] == axis_calls["*"]
    assert axis_calls["val/*"] == axis_calls["*"]
    assert axis_calls["global_step"] == axis_calls["*"]


def test_auto_device_selection_returns_physical_cuda_index(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    selected: list[int] = []

    monkeypatch.setattr(joint_train, "GPUInfo", _FakeGPUInfo, raising=False)
    monkeypatch.setattr(joint_train.torch.cuda, "is_available", lambda: True)
    monkeypatch.setattr(joint_train.torch.cuda, "device_count", lambda: 2)
    monkeypatch.setattr(joint_train.torch.cuda, "set_device", selected.append)
    monkeypatch.setattr(
        joint_train.torch.cuda, "get_device_name", lambda idx: f"GPU {idx}"
    )

    device = joint_train._select_training_device("-1")

    assert device == torch.device("cuda:1")
    assert selected == [1]
