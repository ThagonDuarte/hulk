# ruff: noqa: C901, TRY003

import copy
from dataclasses import dataclass

import torch
import torch.distributed as distributed
import torch.nn as nn
import torch.nn.functional as functional
from torch import Tensor

from ultralytics_dfine.loss.box_ops import (
    box_cxcywh_to_xyxy,
    box_iou,
    generalized_box_iou,
)
from ultralytics_dfine.loss.distribution import bbox_to_distance
from ultralytics_dfine.loss.matcher import (
    HungarianMatcher,
    Match,
    Target,
)

ModelOutput = dict[str, object]


@dataclass(frozen=True)
class CriterionResult:
    """Weighted D-FINE losses and final one-to-one assignments."""

    losses: dict[str, Tensor]
    final_matches: list[Match]


def _world_size() -> int:
    if distributed.is_available() and distributed.is_initialized():
        return distributed.get_world_size()
    return 1


class DFINECriterion(nn.Module):
    """Faithful D-FINE criterion with GO-LSD union matching."""

    def __init__(
        self,
        num_classes: int,
        *,
        matcher: HungarianMatcher | None = None,
        alpha: float = 0.75,
        gamma: float = 2.0,
        reg_max: int = 32,
    ) -> None:
        super().__init__()
        self.num_classes = num_classes
        self.matcher = matcher or HungarianMatcher()
        self.alpha = alpha
        self.gamma = gamma
        self.reg_max = reg_max
        self.weight_dict = {
            "loss_vfl": 1.0,
            "loss_bbox": 5.0,
            "loss_giou": 2.0,
            "loss_fgl": 0.15,
            "loss_ddf": 1.5,
        }
        self._fgl_targets: tuple[Tensor, Tensor, Tensor] | None = None
        self._fgl_targets_dn: tuple[Tensor, Tensor, Tensor] | None = None
        self._num_pos: Tensor | None = None
        self._num_neg: Tensor | None = None

    @staticmethod
    def _source_permutation(indices: list[Match]) -> tuple[Tensor, Tensor]:
        batch = torch.cat(
            [
                torch.full_like(source, index)
                for index, (source, _) in enumerate(indices)
            ]
        )
        source = torch.cat([source for source, _ in indices])
        return batch, source

    @staticmethod
    def _matching_union(
        indices: list[Match],
        auxiliary_indices: list[list[Match]],
    ) -> list[Match]:
        combined = [
            (source.clone(), target.clone()) for source, target in indices
        ]
        for auxiliary in auxiliary_indices:
            combined = [
                (
                    torch.cat((source, aux_source)),
                    torch.cat((target, aux_target)),
                )
                for (source, target), (aux_source, aux_target) in zip(
                    combined,
                    auxiliary,
                    strict=True,
                )
            ]

        result: list[Match] = []
        for source, target in combined:
            pairs = torch.cat((source[:, None], target[:, None]), dim=1)
            unique, counts = torch.unique(
                pairs,
                return_counts=True,
                dim=0,
            )
            order = torch.argsort(counts, descending=True)
            selected: dict[int, int] = {}
            for pair in unique[order]:
                source_index = int(pair[0].item())
                selected.setdefault(source_index, int(pair[1].item()))
            result.append(
                (
                    torch.tensor(
                        list(selected),
                        dtype=torch.int64,
                        device=pairs.device,
                    ),
                    torch.tensor(
                        list(selected.values()),
                        dtype=torch.int64,
                        device=pairs.device,
                    ),
                )
            )
        return result

    @staticmethod
    def _tensor(output: ModelOutput, key: str) -> Tensor:
        value = output.get(key)
        if not isinstance(value, Tensor):
            raise KeyError(f"D-FINE output is missing tensor '{key}'")
        return value

    def _loss_vfl(
        self,
        outputs: ModelOutput,
        targets: list[Target],
        indices: list[Match],
        num_boxes: float,
    ) -> dict[str, Tensor]:
        source_index = self._source_permutation(indices)
        boxes = self._tensor(outputs, "pred_boxes")[source_index]
        target_boxes = torch.cat(
            [
                target["boxes"][index]
                for target, (_, index) in zip(targets, indices, strict=True)
            ]
        )
        ious, _ = box_iou(
            box_cxcywh_to_xyxy(boxes),
            box_cxcywh_to_xyxy(target_boxes),
        )
        matched_ious = torch.diag(ious).detach()
        logits = self._tensor(outputs, "pred_logits")
        target_labels = torch.cat(
            [
                target["labels"][index]
                for target, (_, index) in zip(targets, indices, strict=True)
            ]
        )
        classes = torch.full(
            logits.shape[:2],
            self.num_classes,
            dtype=torch.int64,
            device=logits.device,
        )
        classes[source_index] = target_labels
        one_hot = functional.one_hot(
            classes,
            num_classes=self.num_classes + 1,
        )[..., :-1]
        target_scores = torch.zeros_like(classes, dtype=logits.dtype)
        target_scores[source_index] = matched_ious.to(logits.dtype)
        target_scores = target_scores.unsqueeze(-1) * one_hot
        predicted_scores = logits.sigmoid().detach()
        weight = (
            self.alpha * predicted_scores.pow(self.gamma) * (1 - one_hot)
            + target_scores
        )
        loss = functional.binary_cross_entropy_with_logits(
            logits,
            target_scores,
            weight=weight,
            reduction="none",
        )
        valid_classes = torch.stack(
            [
                target.get(
                    "valid_detection_classes",
                    torch.ones(
                        self.num_classes,
                        dtype=torch.bool,
                        device=logits.device,
                    ),
                ).to(device=logits.device, dtype=torch.bool)
                for target in targets
            ]
        )
        if valid_classes.shape != (logits.shape[0], self.num_classes):
            raise ValueError("valid_detection_classes must match D-FINE logits")
        if any("detection_class_loss_weights" in target for target in targets):
            class_weights = []
            for target, valid in zip(targets, valid_classes, strict=True):
                raw_weights = target.get("detection_class_loss_weights")
                weights = (
                    valid.to(dtype=logits.dtype)
                    if raw_weights is None
                    else raw_weights.to(
                        device=logits.device,
                        dtype=logits.dtype,
                    )
                )
                if weights.shape != (self.num_classes,):
                    raise ValueError(
                        "detection_class_loss_weights must match D-FINE logits"
                    )
                if not torch.isfinite(weights).all() or (weights < 0).any():
                    raise ValueError(
                        "detection_class_loss_weights must be finite and "
                        "non-negative"
                    )
                class_weights.append(weights)
            loss = loss * torch.stack(class_weights)[:, None, :]
        else:
            loss = loss * valid_classes[:, None, :]
        loss = loss.mean(1).sum() * logits.shape[1] / num_boxes
        return {"loss_vfl": loss}

    def _loss_boxes(
        self,
        outputs: ModelOutput,
        targets: list[Target],
        indices: list[Match],
        num_boxes: float,
    ) -> dict[str, Tensor]:
        source_index = self._source_permutation(indices)
        boxes = self._tensor(outputs, "pred_boxes")[source_index]
        target_boxes = torch.cat(
            [
                target["boxes"][index]
                for target, (_, index) in zip(targets, indices, strict=True)
            ]
        )
        bbox = functional.l1_loss(boxes, target_boxes, reduction="none")
        giou = 1 - torch.diag(
            generalized_box_iou(
                box_cxcywh_to_xyxy(boxes),
                box_cxcywh_to_xyxy(target_boxes),
            )
        )
        return {
            "loss_bbox": bbox.sum() / num_boxes,
            "loss_giou": giou.sum() / num_boxes,
        }

    def _distribution_focal_loss(
        self,
        prediction: Tensor,
        labels: Tensor,
        weight_right: Tensor,
        weight_left: Tensor,
        weights: Tensor,
        num_boxes: float,
    ) -> Tensor:
        left = labels.long()
        right = left + 1
        loss = functional.cross_entropy(
            prediction, left, reduction="none"
        ) * weight_left.reshape(-1) + functional.cross_entropy(
            prediction, right, reduction="none"
        ) * weight_right.reshape(-1)
        return (loss * weights.float()).sum() / num_boxes

    def _loss_local(
        self,
        outputs: ModelOutput,
        targets: list[Target],
        indices: list[Match],
        num_boxes: float,
    ) -> dict[str, Tensor]:
        if "pred_corners" not in outputs:
            return {}

        source_index = self._source_permutation(indices)
        target_boxes = torch.cat(
            [
                target["boxes"][index]
                for target, (_, index) in zip(targets, indices, strict=True)
            ]
        )
        corners = self._tensor(outputs, "pred_corners")[source_index].reshape(
            -1,
            self.reg_max + 1,
        )
        reference_points = self._tensor(outputs, "ref_points")[
            source_index
        ].detach()
        up = self._tensor(outputs, "up")
        reg_scale = self._tensor(outputs, "reg_scale")
        is_denoising = bool(outputs.get("is_dn", False))
        with torch.no_grad():
            if is_denoising and self._fgl_targets_dn is None:
                self._fgl_targets_dn = bbox_to_distance(
                    reference_points,
                    box_cxcywh_to_xyxy(target_boxes),
                    self.reg_max,
                    reg_scale,
                    up,
                )
            if not is_denoising and self._fgl_targets is None:
                self._fgl_targets = bbox_to_distance(
                    reference_points,
                    box_cxcywh_to_xyxy(target_boxes),
                    self.reg_max,
                    reg_scale,
                    up,
                )

        distribution_targets = (
            self._fgl_targets_dn if is_denoising else self._fgl_targets
        )
        if distribution_targets is None:
            raise RuntimeError("FGL targets were not initialized")
        labels, weight_right, weight_left = distribution_targets
        boxes = self._tensor(outputs, "pred_boxes")
        ious, _ = box_iou(
            box_cxcywh_to_xyxy(boxes[source_index]),
            box_cxcywh_to_xyxy(target_boxes),
        )
        matched_ious = torch.diag(ious)
        fgl_weights = (
            matched_ious.unsqueeze(-1).repeat(1, 1, 4).reshape(-1).detach()
        )
        losses = {
            "loss_fgl": self._distribution_focal_loss(
                corners,
                labels,
                weight_right,
                weight_left,
                fgl_weights,
                num_boxes,
            )
        }

        teacher_corners = outputs.get("teacher_corners")
        teacher_logits = outputs.get("teacher_logits")
        if not isinstance(teacher_corners, Tensor) or not isinstance(
            teacher_logits,
            Tensor,
        ):
            return losses

        all_corners = self._tensor(outputs, "pred_corners").reshape(
            -1,
            self.reg_max + 1,
        )
        teacher_corners = teacher_corners.reshape(-1, self.reg_max + 1)
        if torch.equal(all_corners, teacher_corners):
            losses["loss_ddf"] = all_corners.sum() * 0
            return losses

        confidence = teacher_logits.sigmoid().max(dim=-1).values.clone()
        matched = torch.zeros_like(confidence, dtype=torch.bool)
        matched[source_index] = True
        matched = matched.unsqueeze(-1).repeat(1, 1, 4).reshape(-1)
        confidence[source_index] = matched_ious.reshape_as(
            confidence[source_index]
        ).to(confidence.dtype)
        confidence = (
            confidence.unsqueeze(-1).repeat(1, 1, 4).reshape(-1).detach()
        )
        temperature = 5.0
        ddf = (
            confidence
            * temperature**2
            * functional.kl_div(
                functional.log_softmax(all_corners / temperature, dim=1),
                functional.softmax(
                    teacher_corners.detach() / temperature, dim=1
                ),
                reduction="none",
            ).sum(-1)
        )
        if not is_denoising:
            batch_scale = 8 / boxes.shape[0]
            self._num_pos = (matched.sum() * batch_scale).sqrt()
            self._num_neg = ((~matched).sum() * batch_scale).sqrt()
        if self._num_pos is None or self._num_neg is None:
            raise RuntimeError("DDF balancing factors were not initialized")
        positive = ddf[matched].mean() if matched.any() else ddf.sum() * 0
        negative = ddf[~matched].mean() if (~matched).any() else ddf.sum() * 0
        losses["loss_ddf"] = (
            positive * self._num_pos + negative * self._num_neg
        ) / (self._num_pos + self._num_neg)
        return losses

    def _compute_losses(
        self,
        outputs: ModelOutput,
        targets: list[Target],
        indices: list[Match],
        num_boxes: float,
    ) -> dict[str, Tensor]:
        losses: dict[str, Tensor] = {}
        for loss_function in (
            self._loss_vfl,
            self._loss_boxes,
            self._loss_local,
        ):
            values = loss_function(outputs, targets, indices, num_boxes)
            losses.update(
                {
                    key: value * self.weight_dict[key]
                    for key, value in values.items()
                }
            )
        return losses

    @staticmethod
    def _suffix(losses: dict[str, Tensor], suffix: str) -> dict[str, Tensor]:
        return {f"{key}{suffix}": value for key, value in losses.items()}

    @staticmethod
    def _output_list(outputs: ModelOutput, key: str) -> list[ModelOutput]:
        value = outputs.get(key, [])
        if not isinstance(value, list):
            raise TypeError(f"D-FINE output '{key}' must be a list")
        return value

    @staticmethod
    def _normalized_count(count: int, device: torch.device) -> float:
        value = torch.tensor([count], dtype=torch.float, device=device)
        if distributed.is_available() and distributed.is_initialized():
            distributed.all_reduce(value)
        return float((value / _world_size()).clamp(min=1).item())

    def _forward_result(
        self,
        outputs: ModelOutput,
        targets: list[Target],
    ) -> CriterionResult:
        tensor_output = self._tensor(outputs, "pred_logits")
        main = {
            key: value for key, value in outputs.items() if "aux" not in key
        }
        indices = self.matcher(
            {
                "pred_logits": self._tensor(main, "pred_logits"),
                "pred_boxes": self._tensor(main, "pred_boxes"),
            },
            targets,
        )["indices"]
        self._fgl_targets = None
        self._fgl_targets_dn = None
        self._num_pos = None
        self._num_neg = None

        auxiliary = self._output_list(outputs, "aux_outputs")
        encoder_auxiliary = self._output_list(outputs, "enc_aux_outputs")
        preliminary = outputs.get("pre_outputs")
        if not auxiliary or not isinstance(preliminary, dict):
            raise ValueError(
                "Training outputs must include decoder auxiliary and "
                "preliminary outputs"
            )

        cached_auxiliary = [
            self.matcher(
                {
                    "pred_logits": self._tensor(output, "pred_logits"),
                    "pred_boxes": self._tensor(output, "pred_boxes"),
                },
                targets,
            )["indices"]
            for output in [*auxiliary, preliminary]
        ]
        cached_encoder = [
            self.matcher(
                {
                    "pred_logits": self._tensor(output, "pred_logits"),
                    "pred_boxes": self._tensor(output, "pred_boxes"),
                },
                targets,
            )["indices"]
            for output in encoder_auxiliary
        ]
        union = self._matching_union(
            indices,
            [*cached_auxiliary, *cached_encoder],
        )
        num_boxes = self._normalized_count(
            sum(len(target["labels"]) for target in targets),
            tensor_output.device,
        )
        num_union = self._normalized_count(
            sum(len(source) for source, _ in union),
            tensor_output.device,
        )

        losses: dict[str, Tensor] = {}
        for function in (self._loss_vfl, self._loss_boxes, self._loss_local):
            selected = union if function != self._loss_vfl else indices
            denominator = num_union if function != self._loss_vfl else num_boxes
            values = function(outputs, targets, selected, denominator)
            losses.update(
                {
                    key: value * self.weight_dict[key]
                    for key, value in values.items()
                }
            )

        for index, auxiliary_output in enumerate(auxiliary):
            auxiliary_output["up"] = outputs["up"]
            auxiliary_output["reg_scale"] = outputs["reg_scale"]
            output_losses: dict[str, Tensor] = {}
            for function in (
                self._loss_vfl,
                self._loss_boxes,
                self._loss_local,
            ):
                selected = (
                    union
                    if function != self._loss_vfl
                    else cached_auxiliary[index]
                )
                denominator = (
                    num_union if function != self._loss_vfl else num_boxes
                )
                values = function(
                    auxiliary_output, targets, selected, denominator
                )
                output_losses.update(
                    {
                        key: value * self.weight_dict[key]
                        for key, value in values.items()
                    }
                )
            losses.update(self._suffix(output_losses, f"_aux_{index}"))

        preliminary_losses = self._compute_losses(
            preliminary,
            targets,
            cached_auxiliary[-1],
            num_boxes,
        )
        losses.update(self._suffix(preliminary_losses, "_pre"))

        for index, encoder_output in enumerate(encoder_auxiliary):
            encoder_targets = targets
            metadata = outputs.get("enc_meta")
            if isinstance(metadata, dict) and metadata.get("class_agnostic"):
                encoder_targets = copy.deepcopy(targets)
                for target in encoder_targets:
                    target["labels"] = torch.zeros_like(target["labels"])
            output_losses: dict[str, Tensor] = {}
            for function in (
                self._loss_vfl,
                self._loss_boxes,
                self._loss_local,
            ):
                selected = (
                    union
                    if function == self._loss_boxes
                    else cached_encoder[index]
                )
                denominator = (
                    num_union if function == self._loss_boxes else num_boxes
                )
                values = function(
                    encoder_output, encoder_targets, selected, denominator
                )
                output_losses.update(
                    {
                        key: value * self.weight_dict[key]
                        for key, value in values.items()
                    }
                )
            losses.update(self._suffix(output_losses, f"_enc_{index}"))

        denoising_outputs = self._output_list(outputs, "dn_outputs")
        denoising_metadata = outputs.get("dn_meta")
        if denoising_outputs and isinstance(denoising_metadata, dict):
            denoising_indices = self._denoising_indices(
                denoising_metadata,
                targets,
            )
            denoising_num_boxes = max(
                num_boxes * int(denoising_metadata["dn_num_group"]),
                1,
            )
            for index, denoising_output in enumerate(denoising_outputs):
                denoising_output["is_dn"] = True
                denoising_output["up"] = outputs["up"]
                denoising_output["reg_scale"] = outputs["reg_scale"]
                output_losses = self._compute_losses(
                    denoising_output,
                    targets,
                    denoising_indices,
                    denoising_num_boxes,
                )
                losses.update(self._suffix(output_losses, f"_dn_{index}"))

            denoising_preliminary = outputs.get("dn_pre_outputs")
            if isinstance(denoising_preliminary, dict):
                output_losses = self._compute_losses(
                    denoising_preliminary,
                    targets,
                    denoising_indices,
                    denoising_num_boxes,
                )
                losses.update(self._suffix(output_losses, "_dn_pre"))

        return CriterionResult(
            losses={
                key: torch.nan_to_num(value, nan=0.0)
                for key, value in losses.items()
            },
            final_matches=indices,
        )

    def forward(
        self,
        outputs: ModelOutput,
        targets: list[Target],
    ) -> dict[str, Tensor]:
        return self._forward_result(outputs, targets).losses

    def forward_with_matches(
        self,
        outputs: ModelOutput,
        targets: list[Target],
    ) -> CriterionResult:
        """Compute losses while exposing final decoder assignments."""
        return self._forward_result(outputs, targets)

    @staticmethod
    def _denoising_indices(
        metadata: dict[str, object],
        targets: list[Target],
    ) -> list[Match]:
        positive_indices = metadata.get("dn_positive_idx")
        groups = metadata.get("dn_num_group")
        if not isinstance(positive_indices, (list, tuple)) or not isinstance(
            groups, int
        ):
            raise TypeError("Invalid D-FINE denoising metadata")
        result: list[Match] = []
        for target, source in zip(targets, positive_indices, strict=True):
            if not isinstance(source, Tensor):
                raise TypeError("Denoising positive indices must be tensors")
            count = len(target["labels"])
            target_indices = torch.arange(
                count,
                dtype=torch.int64,
                device=target["labels"].device,
            ).tile(groups)
            if len(source) != len(target_indices):
                raise ValueError("Denoising assignment length mismatch")
            result.append((source, target_indices))
        return result
