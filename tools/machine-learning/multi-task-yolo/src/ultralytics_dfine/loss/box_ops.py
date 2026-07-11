# ruff: noqa: TRY003

import torch
from torch import Tensor
from torchvision.ops.boxes import box_area


def box_cxcywh_to_xyxy(boxes: Tensor) -> Tensor:
    center_x, center_y, width, height = boxes.unbind(-1)
    width = width.clamp(min=0.0)
    height = height.clamp(min=0.0)
    return torch.stack(
        (
            center_x - 0.5 * width,
            center_y - 0.5 * height,
            center_x + 0.5 * width,
            center_y + 0.5 * height,
        ),
        dim=-1,
    )


def box_iou(boxes1: Tensor, boxes2: Tensor) -> tuple[Tensor, Tensor]:
    area1 = box_area(boxes1)
    area2 = box_area(boxes2)
    top_left = torch.maximum(boxes1[:, None, :2], boxes2[:, :2])
    bottom_right = torch.minimum(boxes1[:, None, 2:], boxes2[:, 2:])
    size = (bottom_right - top_left).clamp(min=0)
    intersection = size[:, :, 0] * size[:, :, 1]
    union = area1[:, None] + area2 - intersection
    return intersection / union.clamp(min=torch.finfo(union.dtype).eps), union


def generalized_box_iou(boxes1: Tensor, boxes2: Tensor) -> Tensor:
    if not (boxes1[:, 2:] >= boxes1[:, :2]).all():
        raise ValueError("boxes1 contains a degenerate box")
    if not (boxes2[:, 2:] >= boxes2[:, :2]).all():
        raise ValueError("boxes2 contains a degenerate box")

    iou, union = box_iou(boxes1, boxes2)
    top_left = torch.minimum(boxes1[:, None, :2], boxes2[:, :2])
    bottom_right = torch.maximum(boxes1[:, None, 2:], boxes2[:, 2:])
    size = (bottom_right - top_left).clamp(min=0)
    area = size[:, :, 0] * size[:, :, 1]
    return iou - (area - union) / area.clamp(min=torch.finfo(area.dtype).eps)
