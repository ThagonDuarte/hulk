"""Pose and point-set losses for native multi-task D-FINE heads."""

# ruff: noqa: TRY003

from dataclasses import dataclass

import torch
import torch.distributed as distributed
import torch.nn as nn
import torch.nn.functional as functional
from scipy.optimize import linear_sum_assignment
from torch import Tensor

from ultralytics_dfine.config import (
    FieldLossConfig,
    MultiTaskLossConfig,
    PoseLossConfig,
)
from ultralytics_dfine.loss.criterion import CriterionResult, DFINECriterion
from ultralytics_dfine.loss.matcher import Match, Target
from ultralytics_dfine.schemas import (
    FIELD_FEATURE_SCHEMA,
    PERSON_POSE_SCHEMA,
    ROBOT_POSE_SCHEMA,
    HeadId,
    PoseSchemaConfig,
)

HeadOutput = dict[str, Tensor]


def _normalized_count(count: Tensor, device: torch.device) -> Tensor:
    value = count.detach().to(device=device, dtype=torch.float32)
    if distributed.is_available() and distributed.is_initialized():
        distributed.all_reduce(value)
        value /= distributed.get_world_size()
    return value.clamp(min=1.0)


def _source_indices(
    matches: list[Match],
    device: torch.device,
) -> tuple[Tensor, Tensor]:
    batches = []
    sources = []
    for batch_index, (source, _) in enumerate(matches):
        sources.append(source.to(device))
        batches.append(
            torch.full(
                source.shape,
                batch_index,
                dtype=torch.long,
                device=device,
            )
        )
    if not sources:
        empty = torch.empty(0, dtype=torch.long, device=device)
        return empty, empty
    return torch.cat(batches), torch.cat(sources)


@dataclass(frozen=True)
class PoseLossResult:
    coordinate: Tensor
    oks: Tensor
    visibility: Tensor

    @property
    def total(self) -> Tensor:
        return self.coordinate + self.oks + self.visibility

    def to_dict(self, prefix: str) -> dict[str, Tensor]:
        return {
            f"{prefix}_coordinate": self.coordinate,
            f"{prefix}_oks": self.oks,
            f"{prefix}_visibility": self.visibility,
        }


class QueryPoseCriterion(nn.Module):
    """Keypoint losses over final D-FINE Hungarian assignments."""

    def __init__(
        self,
        schema: PoseSchemaConfig,
        class_id: int,
        *,
        config: PoseLossConfig | None = None,
        coordinate_weight: float | None = None,
        oks_weight: float | None = None,
        visibility_weight: float | None = None,
    ) -> None:
        super().__init__()
        settings = config or PoseLossConfig()
        self.schema = schema
        self.class_id = class_id
        self.coordinate_space = settings.coordinate_space
        self.smooth_l1_beta = settings.smooth_l1_beta
        self.coordinate_weight = (
            settings.coordinate_weight
            if coordinate_weight is None
            else coordinate_weight
        )
        self.oks_weight = (
            settings.oks_weight if oks_weight is None else oks_weight
        )
        self.visibility_weight = (
            settings.visibility_weight
            if visibility_weight is None
            else visibility_weight
        )
        self.oks_constants: Tensor
        self.register_buffer(
            "oks_constants",
            torch.tensor(schema.oks_sigmas),
            persistent=False,
        )

    def _applicable_matches(
        self,
        matches: list[Match],
        targets: list[Target],
    ) -> list[Match]:
        selected = []
        for (source, target_indices), target in zip(
            matches,
            targets,
            strict=True,
        ):
            source = source.to(target["labels"].device)
            target_indices = target_indices.to(target["labels"].device)
            labels = target["labels"][target_indices]
            keep = labels == self.class_id
            selected.append((source[keep], target_indices[keep]))
        return selected

    def forward(
        self,
        output: HeadOutput,
        targets: list[Target],
        matches: list[Match],
    ) -> PoseLossResult:
        predicted = output["pred_keypoints"]
        visibility_logits = output["pred_visibility"]
        selected = self._applicable_matches(matches, targets)
        batch_indices, source_indices = _source_indices(
            selected,
            predicted.device,
        )
        connected_zero = predicted.sum() * 0 + visibility_logits.sum() * 0
        if source_indices.numel() == 0:
            return PoseLossResult(
                connected_zero,
                connected_zero,
                connected_zero,
            )

        target_keypoints = torch.cat(
            [
                target["keypoints"][target_indices]
                for target, (_, target_indices) in zip(
                    targets,
                    selected,
                    strict=True,
                )
            ]
        ).to(predicted.device)
        target_boxes = torch.cat(
            [
                target["boxes"][target_indices]
                for target, (_, target_indices) in zip(
                    targets,
                    selected,
                    strict=True,
                )
            ]
        ).to(predicted.device)
        predicted = predicted[batch_indices, source_indices]
        visibility_logits = visibility_logits[batch_indices, source_indices]
        target_xy = target_keypoints[..., :2]
        visible = target_keypoints[..., 2] > 0
        visible_count = _normalized_count(
            visible.sum(),
            predicted.device,
        )

        coordinate_error = predicted - target_xy
        if self.coordinate_space == "box":
            coordinate_error = coordinate_error / target_boxes[
                :, None, 2:
            ].clamp(min=1e-3)
        coordinate = functional.smooth_l1_loss(
            coordinate_error,
            torch.zeros_like(coordinate_error),
            reduction="none",
            beta=self.smooth_l1_beta,
        ).sum(-1)
        coordinate = (
            coordinate.masked_select(visible).sum() / visible_count
        ) * self.coordinate_weight

        squared_distance = (predicted - target_xy).square().sum(-1)
        area = target_boxes[:, 2] * target_boxes[:, 3]
        constants = self.oks_constants.to(predicted)
        if not self.schema.oks_uses_direct_k:
            constants = constants * 2
        denominator = 2 * area[:, None].clamp(min=1e-8) * constants.square()
        similarity = torch.exp(-squared_distance / denominator)
        oks = (
            1 - similarity.masked_select(visible).sum() / visible_count
        ) * self.oks_weight

        visibility = functional.binary_cross_entropy_with_logits(
            visibility_logits,
            visible.to(visibility_logits.dtype),
            reduction="sum",
        )
        visibility = (
            visibility / visibility_logits.numel()
        ) * self.visibility_weight
        return PoseLossResult(coordinate, oks, visibility)


class PointHungarianMatcher(nn.Module):
    """One-to-one focal-class and coordinate matcher for field points."""

    def __init__(
        self,
        *,
        class_cost: float = 2.0,
        point_cost: float = 20.0,
        alpha: float = 0.25,
        gamma: float = 2.0,
    ) -> None:
        super().__init__()
        self.class_cost = class_cost
        self.point_cost = point_cost
        self.alpha = alpha
        self.gamma = gamma

    @torch.no_grad()
    def forward(
        self,
        output: HeadOutput,
        targets: list[Target],
    ) -> list[Match]:
        logits = output["pred_logits"]
        points = output["pred_points"]
        matches = []
        for batch_index, target in enumerate(targets):
            labels = target["labels"]
            target_points = target["points"]
            if labels.numel() == 0:
                empty = torch.empty(0, dtype=torch.long)
                matches.append((empty, empty))
                continue
            probabilities = logits[batch_index].sigmoid()[:, labels]
            negative = (
                (1 - self.alpha)
                * probabilities.pow(self.gamma)
                * -(1 - probabilities + 1e-8).log()
            )
            positive = (
                self.alpha
                * (1 - probabilities).pow(self.gamma)
                * -(probabilities + 1e-8).log()
            )
            class_cost = positive - negative
            coordinate_cost = torch.cdist(
                points[batch_index],
                target_points.to(points.device),
                p=1,
            )
            cost = (
                self.class_cost * class_cost + self.point_cost * coordinate_cost
            )
            source, destination = linear_sum_assignment(cost.cpu())
            matches.append(
                (
                    torch.as_tensor(source, dtype=torch.long),
                    torch.as_tensor(destination, dtype=torch.long),
                )
            )
        return matches


class FieldFeatureCriterion(nn.Module):
    """Sigmoid focal classification and point L1 loss."""

    def __init__(
        self,
        num_classes: int,
        *,
        matcher: PointHungarianMatcher | None = None,
        alpha: float = 0.25,
        gamma: float = 2.0,
        class_weight: float = 1.0,
        point_weight: float = 5.0,
        config: FieldLossConfig | None = None,
    ) -> None:
        super().__init__()
        settings = config or FieldLossConfig(
            class_weight=class_weight,
            point_weight=point_weight,
            focal_alpha=alpha,
            focal_gamma=gamma,
        )
        self.num_classes = num_classes
        self.matcher = matcher or PointHungarianMatcher()
        self.alpha = settings.focal_alpha
        self.gamma = settings.focal_gamma
        self.class_weight = settings.class_weight
        self.point_weight = settings.point_weight

    def forward(
        self,
        output: HeadOutput,
        targets: list[Target],
    ) -> tuple[dict[str, Tensor], list[Match]]:
        logits = output["pred_logits"]
        points = output["pred_points"]
        matches = self.matcher(output, targets)
        target_classes = torch.zeros_like(logits)
        batch_indices, source_indices = _source_indices(matches, logits.device)
        matched_labels = torch.cat(
            [
                target["labels"][target_indices]
                for target, (_, target_indices) in zip(
                    targets,
                    matches,
                    strict=True,
                )
            ]
        ).to(logits.device)
        target_points = torch.cat(
            [
                target["points"][target_indices]
                for target, (_, target_indices) in zip(
                    targets,
                    matches,
                    strict=True,
                )
            ]
        ).to(points.device)
        if source_indices.numel() > 0:
            class_targets = torch.ones_like(
                matched_labels,
                dtype=logits.dtype,
            )
            target_classes[
                batch_indices,
                source_indices,
                matched_labels,
            ] = class_targets
        probability = logits.sigmoid()
        cross_entropy = functional.binary_cross_entropy_with_logits(
            logits,
            target_classes,
            reduction="none",
        )
        modulation = (target_classes - probability).abs().pow(self.gamma)
        alpha = self.alpha * target_classes + (1 - self.alpha) * (
            1 - target_classes
        )
        weighted_class_loss = cross_entropy * modulation * alpha
        normalizer = _normalized_count(
            torch.tensor(matched_labels.numel(), device=logits.device),
            logits.device,
        )
        class_loss = weighted_class_loss.sum() / normalizer

        if source_indices.numel() == 0:
            point_loss = points.sum() * 0
        else:
            point_loss = (
                functional.l1_loss(
                    points[batch_indices, source_indices],
                    target_points,
                    reduction="sum",
                )
                / normalizer
            )
        return {
            "field_class": class_loss * self.class_weight,
            "field_point": point_loss * self.point_weight,
        }, matches


class MultiTaskCriterion(nn.Module):
    """Dispatch homogeneous task batches over one shared model output."""

    def __init__(
        self,
        num_detection_classes: int = 8,
        *,
        person_class_id: int = 7,
        robot_class_id: int = 4,
        loss_config: MultiTaskLossConfig | None = None,
    ) -> None:
        super().__init__()
        self.loss_config = loss_config or MultiTaskLossConfig()
        self.detection = DFINECriterion(num_detection_classes)
        self.person_class_id = person_class_id
        self.robot_class_id = robot_class_id
        self.person = QueryPoseCriterion(
            PERSON_POSE_SCHEMA,
            person_class_id,
            config=self.loss_config.person_pose,
        )
        self.robot = QueryPoseCriterion(
            ROBOT_POSE_SCHEMA,
            robot_class_id,
            config=self.loss_config.robot_pose,
        )
        self.field = FieldFeatureCriterion(
            len(FIELD_FEATURE_SCHEMA.class_names),
            config=self.loss_config.field_features,
        )

    @staticmethod
    def _head(
        outputs: dict[str, dict[str, object]],
        head_id: HeadId,
    ) -> dict[str, Tensor]:
        output = outputs[str(head_id)]
        if not all(isinstance(value, Tensor) for value in output.values()):
            raise TypeError(f"Head '{head_id}' must contain only tensors")
        return {
            key: value
            for key, value in output.items()
            if isinstance(value, Tensor)
        }

    @staticmethod
    def _inactive(output: dict[str, Tensor], prefix: str) -> dict[str, Tensor]:
        zero = torch.stack([value.sum() * 0 for value in output.values()]).sum()
        return {f"{prefix}_inactive": zero}

    def forward(
        self,
        outputs: dict[str, dict[str, object]],
        targets: list[Target],
        active_head: HeadId,
    ) -> CriterionResult:
        detection_output = outputs[str(HeadId.OBJECT)]
        detection = self.detection.forward_with_matches(
            detection_output,
            targets,
        )
        losses = dict(detection.losses)

        if active_head == HeadId.PERSON_POSE:
            person_output = self._head(outputs, HeadId.PERSON_POSE)
            losses.update(
                self.person(
                    person_output,
                    targets,
                    detection.final_matches,
                ).to_dict("person")
            )

        if active_head == HeadId.ROBOT_POSE:
            robot_output = self._head(outputs, HeadId.ROBOT_POSE)
            losses.update(
                self.robot(
                    robot_output,
                    targets,
                    detection.final_matches,
                ).to_dict("robot")
            )

        if active_head == HeadId.FIELD_FEATURES:
            field_output = self._head(outputs, HeadId.FIELD_FEATURES)
            point_targets: list[Target] = [
                {
                    "labels": target["point_labels"],
                    "points": target["points"],
                    "boxes": target["boxes"],
                }
                for target in targets
            ]
            field_losses, _ = self.field(field_output, point_targets)
            losses.update(field_losses)
        return CriterionResult(losses, detection.final_matches)
