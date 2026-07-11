# ruff: noqa: TRY003

from typing import TypedDict

import torch
import torch.nn as nn
import torch.nn.functional as functional
from scipy.optimize import linear_sum_assignment
from torch import Tensor

from ultralytics_dfine.loss.box_ops import (
    box_cxcywh_to_xyxy,
    generalized_box_iou,
)

Target = dict[str, Tensor]
Match = tuple[Tensor, Tensor]


class MatcherResult(TypedDict):
    indices: list[Match]


class HungarianMatcher(nn.Module):
    """Official D-FINE focal/L1/GIoU one-to-one matcher."""

    def __init__(
        self,
        *,
        cost_class: float = 2.0,
        cost_bbox: float = 5.0,
        cost_giou: float = 2.0,
        alpha: float = 0.25,
        gamma: float = 2.0,
    ) -> None:
        super().__init__()
        if cost_class == cost_bbox == cost_giou == 0:
            raise ValueError("At least one matching cost must be non-zero")
        self.cost_class = cost_class
        self.cost_bbox = cost_bbox
        self.cost_giou = cost_giou
        self.alpha = alpha
        self.gamma = gamma

    @torch.no_grad()
    def forward(
        self,
        outputs: dict[str, Tensor],
        targets: list[Target],
    ) -> MatcherResult:
        batch_size, num_queries = outputs["pred_logits"].shape[:2]
        probabilities = functional.sigmoid(outputs["pred_logits"].flatten(0, 1))
        predicted_boxes = outputs["pred_boxes"].flatten(0, 1)
        target_ids = torch.cat([target["labels"] for target in targets])
        target_boxes = torch.cat([target["boxes"] for target in targets])

        probabilities = probabilities[:, target_ids]
        negative_cost = (
            (1 - self.alpha)
            * probabilities**self.gamma
            * -(1 - probabilities + 1e-8).log()
        )
        positive_cost = (
            self.alpha
            * (1 - probabilities) ** self.gamma
            * -(probabilities + 1e-8).log()
        )
        classification_cost = positive_cost - negative_cost
        bbox_cost = torch.cdist(predicted_boxes, target_boxes, p=1)
        giou_cost = -generalized_box_iou(
            box_cxcywh_to_xyxy(predicted_boxes),
            box_cxcywh_to_xyxy(target_boxes),
        )
        cost = (
            self.cost_bbox * bbox_cost
            + self.cost_class * classification_cost
            + self.cost_giou * giou_cost
        )
        cost = torch.nan_to_num(
            cost.view(batch_size, num_queries, -1).cpu(),
            nan=1.0,
            posinf=1e6,
            neginf=-1e6,
        )
        sizes = [len(target["boxes"]) for target in targets]
        assignments = [
            linear_sum_assignment(batch_cost[index])
            for index, batch_cost in enumerate(cost.split(sizes, dim=-1))
        ]
        return {
            "indices": [
                (
                    torch.as_tensor(source, dtype=torch.int64),
                    torch.as_tensor(target, dtype=torch.int64),
                )
                for source, target in assignments
            ]
        }
