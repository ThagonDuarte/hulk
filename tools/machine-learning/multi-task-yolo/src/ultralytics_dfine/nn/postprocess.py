# ruff: noqa: TRY003

import torch
import torch.nn as nn
from torch import Tensor

from ultralytics_dfine.loss.box_ops import box_cxcywh_to_xyxy


class DFINEPostProcessorAdapter(nn.Module):
    """Convert raw outputs to deterministic top-k detections without NMS."""

    def __init__(self, num_classes: int, topk: int = 300) -> None:
        super().__init__()
        if num_classes <= 0:
            raise ValueError("num_classes must be positive")
        if topk <= 0:
            raise ValueError("topk must be positive")
        self.num_classes = num_classes
        self.topk = topk

    def select(self, outputs: dict[str, Tensor]) -> tuple[Tensor, Tensor]:
        """Return detections and the source query index for every row."""
        logits = outputs["pred_logits"]
        boxes = outputs["pred_boxes"]
        probabilities = logits.sigmoid().flatten(1)
        count = min(self.topk, probabilities.shape[1])
        scores, flat_indices = probabilities.topk(count, dim=1)
        labels = flat_indices.remainder(self.num_classes)
        query_indices = torch.div(
            flat_indices,
            self.num_classes,
            rounding_mode="floor",
        )
        selected_boxes = boxes.gather(
            1,
            query_indices[..., None].expand(-1, -1, 4),
        )
        detections = torch.cat(
            (
                selected_boxes,
                scores[..., None],
                labels.to(selected_boxes.dtype)[..., None],
            ),
            dim=-1,
        )
        return detections, query_indices

    def forward(self, outputs: dict[str, Tensor]) -> Tensor:
        detections, _ = self.select(outputs)
        return detections

    def to_pixel_xyxy(
        self,
        detections: Tensor,
        image_sizes: Tensor,
    ) -> Tensor:
        """Convert normalized cxcywh detections using [height, width] sizes."""
        boxes = box_cxcywh_to_xyxy(detections[..., :4])
        scale = image_sizes[:, [1, 0, 1, 0]].to(
            device=boxes.device,
            dtype=boxes.dtype,
        )
        boxes = boxes * scale[:, None]
        return torch.cat((boxes, detections[..., 4:]), dim=-1)
