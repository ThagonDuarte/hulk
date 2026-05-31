from __future__ import annotations

import torch
from torch import nn

from model.hydra import Hydra
from utils.model_naming import TaskType


class _DetectLikeHead(nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.nc = 3
        self.reg_max = 1
        self.no = 7
        self.stride = torch.tensor([8.0, 16.0, 32.0])
        self.cv3 = nn.ModuleList([_class_branch() for _ in range(3)])
        self.one2one_cv3 = nn.ModuleList([_class_branch() for _ in range(3)])


def _class_branch() -> nn.Sequential:
    branch = nn.Sequential(nn.Conv2d(2, 2, 1), nn.Conv2d(2, 3, 1))
    with torch.no_grad():
        final = branch[-1]
        assert isinstance(final, nn.Conv2d)
        for row in range(3):
            final.weight[row].fill_(row + 1)
            final.bias[row].fill_(10 + row)
    return branch


def test_hydra_adapts_detect_head_to_custom_class_count() -> None:
    hydra = object.__new__(Hydra)
    nn.Module.__init__(hydra)
    head = _DetectLikeHead()
    hydra.heads = nn.ModuleDict(
        {str(TaskType.OBJECT): nn.ModuleList([head])}
    )
    hydra.head_class_names = {
        str(TaskType.OBJECT): {0: "cat", 1: "dog", 2: "car"}
    }

    Hydra.adapt_head_classes(
        hydra, TaskType.OBJECT, {0: "dog", 1: "custom"}
    )

    assert head.nc == 2
    assert head.no == 6
    assert hydra.head_class_names[str(TaskType.OBJECT)] == {
        0: "dog",
        1: "custom",
    }

    for class_head in (*head.cv3, *head.one2one_cv3):
        final = class_head[-1]
        assert isinstance(final, nn.Conv2d)
        assert final.out_channels == 2
        assert torch.allclose(
            final.weight[0], torch.full_like(final.weight[0], 2)
        )
        assert torch.allclose(
            final.bias[0], torch.full_like(final.bias[0], 11)
        )
