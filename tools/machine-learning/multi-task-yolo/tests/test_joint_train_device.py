import pytest
import torch

from model import joint_train


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
