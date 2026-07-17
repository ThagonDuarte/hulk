"""Native multi-task heads over one shared D-FINE execution."""

# ruff: noqa: TRY003

import math
from collections.abc import Mapping
from dataclasses import asdict
from pathlib import Path
from typing import cast

import torch
import torch.nn as nn
import torch.nn.functional as functional
from torch import Tensor
from transformers import DFineConfig, DFineForObjectDetection

from ultralytics_dfine.config import (
    DFINEArchitectureConfig,
    FieldHeadConfig,
    MultiTaskHeadConfig,
    MultiTaskLossConfig,
    PoseHeadConfig,
    build_multitask_manifest,
)
from ultralytics_dfine.loss.matcher import Target
from ultralytics_dfine.nn.tasks import DFINEDetectionModel
from ultralytics_dfine.schemas import (
    FIELD_FEATURE_SCHEMA,
    PERSON_POSE_SCHEMA,
    ROBOT_POSE_SCHEMA,
    HeadId,
    PointSetSchemaConfig,
    PoseSchemaConfig,
    schema_to_dict,
)

HeadOutput = dict[str, Tensor]
MultiTaskOutput = dict[str, dict[str, object]]


def _output_tensor(output: Mapping[str, object], key: str) -> Tensor:
    value = output.get(key)
    if not isinstance(value, Tensor):
        raise TypeError(f"D-FINE output is missing tensor '{key}'")
    return value


def _head_tensor(output: Mapping[str, object], key: str) -> Tensor:
    value = output.get(key)
    if not isinstance(value, Tensor):
        raise TypeError(f"Task head is missing tensor '{key}'")
    return value


def _border_coordinates(coordinates: Tensor, maximum: int) -> Tensor:
    """Apply ``grid_sample`` border clipping and its boundary gradient."""
    clipped = coordinates.clamp(0, maximum).detach()
    interior = (coordinates > 0) & (coordinates < maximum)
    return torch.where(interior, coordinates, clipped)


def _bilinear_sample_border(feature: Tensor, grid: Tensor) -> Tensor:
    """Sample a 2-D feature map like bilinear ``grid_sample``.

    This is the checkpoint-neutral, ONNX-compatible equivalent of
    ``grid_sample(..., padding_mode="border", align_corners=False)`` used by
    the multi-task heads.  Keeping it in basic tensor operations avoids the
    ONNX ``GridSample`` operator, which is unavailable on the deployment
    target.
    """
    height = feature.shape[-2]
    width = feature.shape[-1]
    x = ((grid[..., 0] + 1) * width - 1) / 2
    y = ((grid[..., 1] + 1) * height - 1) / 2
    x = _border_coordinates(x, width - 1)
    y = _border_coordinates(y, height - 1)

    x0 = x.floor()
    y0 = y.floor()
    x1 = x0 + 1
    y1 = y0 + 1
    x0_index = x0.to(torch.int64)
    y0_index = y0.to(torch.int64)
    x1_index = x1.clamp(max=width - 1).to(torch.int64)
    y1_index = y1.clamp(max=height - 1).to(torch.int64)

    flattened = feature.flatten(2)

    def gather(x_index: Tensor, y_index: Tensor) -> Tensor:
        indices = (y_index * width + x_index).flatten(1)
        indices = indices.unsqueeze(1).expand(-1, feature.shape[1], -1)
        values = flattened.gather(2, indices)
        return values.unflatten(2, grid.shape[1:-1])

    northwest = gather(x0_index, y0_index)
    northeast = gather(x1_index, y0_index)
    southwest = gather(x0_index, y1_index)
    southeast = gather(x1_index, y1_index)
    northwest_weight = (x1 - x) * (y1 - y)
    northeast_weight = (x - x0) * (y1 - y)
    southwest_weight = (x1 - x) * (y - y0)
    southeast_weight = (x - x0) * (y - y0)
    return (
        northwest * northwest_weight.unsqueeze(1)
        + northeast * northeast_weight.unsqueeze(1)
        + southwest * southwest_weight.unsqueeze(1)
        + southeast * southeast_weight.unsqueeze(1)
    )


def _sample_encoder_feature(feature: Tensor, grid: Tensor) -> Tensor:
    """Retain native training semantics and lower only ONNX exports."""
    if torch.onnx.is_in_onnx_export():
        return _bilinear_sample_border(feature, grid)
    return functional.grid_sample(
        feature,
        grid,
        mode="bilinear",
        padding_mode="border",
        align_corners=False,
    )


class SpatialPoseRefiner(nn.Module):
    """Refine coarse joints with encoder features sampled at each joint."""

    def __init__(
        self,
        hidden_dim: int,
        config: PoseHeadConfig,
    ) -> None:
        super().__init__()
        self.config = config
        self.level_projections = nn.ModuleList(
            nn.Linear(hidden_dim, config.refinement_dim)
            for _ in range(config.feature_levels)
        )
        input_dim = (
            hidden_dim
            + config.refinement_dim * config.feature_levels
            + config.refinement_dim
        )
        self.network = nn.Sequential(
            nn.LayerNorm(input_dim),
            nn.Linear(input_dim, config.refinement_dim),
            nn.GELU(),
            nn.Linear(config.refinement_dim, 3),
        )
        final = cast("nn.Linear", self.network[-1])
        nn.init.zeros_(final.weight)
        nn.init.zeros_(final.bias)

    def forward(
        self,
        query_features: Tensor,
        boxes: Tensor,
        coarse_keypoints: Tensor,
        coarse_visibility: Tensor,
        encoder_features: tuple[Tensor, ...],
        joint_embeddings: Tensor,
    ) -> HeadOutput:
        if len(encoder_features) != len(self.level_projections):
            raise ValueError("Unexpected number of pose encoder levels")
        sample_points = (
            coarse_keypoints.detach()
            if self.config.detach_sampling_grid
            else coarse_keypoints
        )
        batch, queries, keypoints, _ = sample_points.shape
        grid = (sample_points * 2 - 1).reshape(
            batch,
            queries * keypoints,
            1,
            2,
        )
        sampled_levels = []
        for feature, projection in zip(
            encoder_features,
            self.level_projections,
            strict=True,
        ):
            sampled = _sample_encoder_feature(feature, grid)
            sampled = (
                sampled.squeeze(-1)
                .transpose(1, 2)
                .reshape(
                    batch,
                    queries,
                    keypoints,
                    feature.shape[1],
                )
            )
            sampled_levels.append(projection(sampled))
        query_context = query_features[:, :, None, :].expand(
            -1,
            -1,
            keypoints,
            -1,
        )
        joint_context = joint_embeddings[None, None].expand(
            batch,
            queries,
            -1,
            -1,
        )
        values = self.network(
            torch.cat(
                (query_context, *sampled_levels, joint_context),
                dim=-1,
            )
        )
        sizes = boxes[..., None, 2:]
        keypoint_delta = (
            values[..., :2].tanh() * sizes * self.config.refinement_scale
        )
        return {
            "pred_keypoints": (coarse_keypoints + keypoint_delta).clamp(0, 1),
            "pred_visibility": coarse_visibility + values[..., 2],
        }


class QueryPoseHead(nn.Module):
    """Predict box-relative keypoints for each final D-FINE query."""

    def __init__(
        self,
        hidden_dim: int,
        schema: PoseSchemaConfig,
        config: PoseHeadConfig | None = None,
    ) -> None:
        super().__init__()
        self.schema = schema
        self.config = config or PoseHeadConfig()
        self.network = nn.Sequential(
            nn.Linear(hidden_dim + 4, hidden_dim),
            nn.GELU(),
            nn.Linear(hidden_dim, schema.keypoint_count * 3),
        )
        self.joint_embeddings: nn.Embedding | None = None
        self.spatial_refiner: SpatialPoseRefiner | None = None
        if self.config.variant != "query_mlp":
            self.joint_embeddings = nn.Embedding(
                schema.keypoint_count,
                self.config.refinement_dim,
            )
            if self.config.variant == "spatial_refine":
                self.spatial_refiner = SpatialPoseRefiner(
                    hidden_dim,
                    self.config,
                )

    def forward(
        self,
        query_features: Tensor,
        boxes: Tensor,
        encoder_features: tuple[Tensor, ...] | None = None,
        shared_refiner: SpatialPoseRefiner | None = None,
    ) -> HeadOutput:
        if query_features.shape[:2] != boxes.shape[:2]:
            raise ValueError("Pose queries and boxes must be aligned")
        values = self.network(torch.cat((query_features, boxes), dim=-1))
        values = values.unflatten(-1, (self.schema.keypoint_count, 3))
        offsets = values[..., :2].tanh()
        centers = boxes[..., None, :2]
        sizes = boxes[..., None, 2:]
        keypoints = (centers + offsets * sizes).clamp(0.0, 1.0)
        coarse = {
            "pred_keypoints": keypoints,
            "pred_visibility": values[..., 2],
        }
        if self.config.variant == "query_mlp":
            return coarse
        if encoder_features is None or self.joint_embeddings is None:
            raise ValueError("Spatial pose heads require encoder features")
        refiner = self.spatial_refiner or shared_refiner
        if refiner is None:
            raise ValueError("Shared spatial pose refiner is missing")
        return refiner(
            query_features,
            boxes,
            coarse["pred_keypoints"],
            coarse["pred_visibility"],
            encoder_features,
            self.joint_embeddings.weight,
        )


class FieldSpatialRefiner(nn.Module):
    """Refine field points from encoder samples at coarse predictions."""

    def __init__(
        self,
        hidden_dim: int,
        config: FieldHeadConfig,
    ) -> None:
        super().__init__()
        self.config = config
        self.level_projections = nn.ModuleList(
            nn.Linear(hidden_dim, config.refinement_dim)
            for _ in range(config.feature_levels)
        )
        input_dim = hidden_dim + config.refinement_dim * config.feature_levels
        self.network = nn.Sequential(
            nn.LayerNorm(input_dim),
            nn.Linear(input_dim, config.refinement_dim),
            nn.GELU(),
            nn.Linear(config.refinement_dim, 2),
        )
        final = cast("nn.Linear", self.network[-1])
        nn.init.zeros_(final.weight)
        nn.init.zeros_(final.bias)

    def forward(
        self,
        decoded: Tensor,
        points: Tensor,
        point_scale: Tensor,
        encoder_features: tuple[Tensor, ...],
    ) -> Tensor:
        if len(encoder_features) != len(self.level_projections):
            raise ValueError("Unexpected number of field encoder levels")
        sample_points = (
            points.detach() if self.config.detach_sampling_grid else points
        )
        grid = (sample_points * 2 - 1).unsqueeze(2)
        sampled_levels = []
        for feature, projection in zip(
            encoder_features,
            self.level_projections,
            strict=True,
        ):
            sampled = _sample_encoder_feature(feature, grid)
            sampled = sampled.squeeze(-1).transpose(1, 2)
            sampled_levels.append(projection(sampled))
        residual = self.network(
            torch.cat((decoded, *sampled_levels), dim=-1)
        ).tanh()
        return (
            points + residual * point_scale * self.config.refinement_scale
        ).clamp(0, 1)


class FieldFeatureHead(nn.Module):
    """Decode a fixed point set from shared HybridEncoder feature maps."""

    def __init__(
        self,
        hidden_dim: int,
        schema: PointSetSchemaConfig = FIELD_FEATURE_SCHEMA,
        *,
        decoder_layers: int = 2,
        attention_heads: int = 8,
        feature_levels: int | None = None,
        config: FieldHeadConfig | None = None,
    ) -> None:
        super().__init__()
        self.schema = schema
        self.hidden_dim = hidden_dim
        configured_levels = feature_levels or 3
        self.config = config or FieldHeadConfig(
            feature_levels=configured_levels
        )
        if (
            feature_levels is not None
            and feature_levels != self.config.feature_levels
        ):
            raise ValueError("Field feature-level settings disagree")
        layer = nn.TransformerDecoderLayer(
            d_model=hidden_dim,
            nhead=attention_heads,
            dim_feedforward=hidden_dim * 4,
            dropout=0.0,
            activation="gelu",
            batch_first=True,
            norm_first=True,
        )
        self.decoder = nn.TransformerDecoder(layer, decoder_layers)
        self.queries = nn.Embedding(schema.num_queries, hidden_dim)
        self.level_embeddings = nn.Parameter(
            torch.empty(self.config.feature_levels, hidden_dim)
        )
        self.classifier = nn.Linear(hidden_dim, len(schema.class_names))
        self.point_predictor = nn.Linear(hidden_dim, 2)
        self.spatial_refiner = (
            FieldSpatialRefiner(hidden_dim, self.config)
            if self.config.variant == "spatial_refine"
            else None
        )
        nn.init.normal_(self.level_embeddings, std=0.02)
        nn.init.zeros_(self.point_predictor.weight)
        nn.init.zeros_(self.point_predictor.bias)
        nn.init.zeros_(self.classifier.weight)
        nn.init.constant_(self.classifier.bias, -4.59511985013459)
        columns = math.ceil(math.sqrt(schema.num_queries))
        rows = math.ceil(schema.num_queries / columns)
        indices = torch.arange(schema.num_queries)
        x = (indices.remainder(columns) + 0.5) / columns
        y = (torch.div(indices, columns, rounding_mode="floor") + 0.5) / rows
        self.reference_points: Tensor
        self.register_buffer(
            "reference_points",
            torch.stack((x, y), dim=1),
        )
        self.point_scale: Tensor
        self.register_buffer(
            "point_scale",
            torch.tensor([1 / columns, 1 / rows]),
        )

    def _position_encoding(self, feature: Tensor) -> Tensor:
        if self.hidden_dim % 4:
            raise ValueError("Field hidden dimension must be divisible by four")
        height, width = feature.shape[-2:]
        half_dim = self.hidden_dim // 2
        dimensions = torch.arange(
            half_dim,
            dtype=feature.dtype,
            device=feature.device,
        )
        frequencies = 10_000 ** (
            2 * torch.div(dimensions, 2, rounding_mode="floor") / half_dim
        )
        y = torch.linspace(
            0,
            2 * torch.pi,
            height,
            dtype=feature.dtype,
            device=feature.device,
        )[:, None].expand(height, width)
        x = torch.linspace(
            0,
            2 * torch.pi,
            width,
            dtype=feature.dtype,
            device=feature.device,
        )[None, :].expand(height, width)

        def encode(coordinates: Tensor) -> Tensor:
            scaled = coordinates[..., None] / frequencies
            encoded = torch.empty_like(scaled)
            encoded[..., 0::2] = scaled[..., 0::2].sin()
            encoded[..., 1::2] = scaled[..., 1::2].cos()
            return encoded

        return torch.cat((encode(y), encode(x)), dim=-1).reshape(
            height * width,
            self.hidden_dim,
        )

    def _flatten_memory(self, features: tuple[Tensor, ...]) -> Tensor:
        if len(features) != len(self.level_embeddings):
            raise ValueError("Unexpected number of D-FINE encoder levels")
        memory = []
        for index, feature in enumerate(features):
            if feature.ndim != 4 or feature.shape[1] != self.hidden_dim:
                raise ValueError("Invalid D-FINE encoder feature shape")
            flattened = feature.flatten(2).transpose(1, 2)
            position = self._position_encoding(feature).unsqueeze(0)
            memory.append(flattened + position + self.level_embeddings[index])
        return torch.cat(memory, dim=1)

    def _local_queries(self, features: tuple[Tensor, ...]) -> Tensor:
        grid = (self.reference_points * 2 - 1).view(1, -1, 1, 2)
        grid = grid.expand(features[0].shape[0], -1, -1, -1)
        sampled_levels = []
        for index, feature in enumerate(features):
            sampled = _sample_encoder_feature(feature, grid)
            sampled = sampled.squeeze(-1).transpose(1, 2)
            sampled_levels.append(sampled + self.level_embeddings[index])
        local = torch.stack(sampled_levels).mean(0)
        return local + self.queries.weight.unsqueeze(0)

    def forward(self, features: tuple[Tensor, ...]) -> HeadOutput:
        memory = self._flatten_memory(features)
        queries = self._local_queries(features)
        decoded = self.decoder(queries, memory)
        offsets = self.point_predictor(decoded).tanh()
        points = (
            self.reference_points.unsqueeze(0) + offsets * self.point_scale
        ).clamp(0, 1)
        if self.spatial_refiner is not None:
            points = self.spatial_refiner(
                decoded,
                points,
                self.point_scale,
                features,
            )
        return {
            "pred_logits": self.classifier(decoded),
            "pred_points": points,
        }


class DFINEMultiTaskModel(nn.Module):
    """One D-FINE detector with person, robot, and field-feature heads."""

    checkpoint_format_version = 3

    def __init__(
        self,
        detector: DFINEDetectionModel,
        *,
        person_schema: PoseSchemaConfig = PERSON_POSE_SCHEMA,
        robot_schema: PoseSchemaConfig = ROBOT_POSE_SCHEMA,
        field_schema: PointSetSchemaConfig = FIELD_FEATURE_SCHEMA,
        head_config: MultiTaskHeadConfig | None = None,
        loss_config: MultiTaskLossConfig | None = None,
    ) -> None:
        super().__init__()
        if not detector.names or detector.names[-1] != "Person":
            raise ValueError("Person must be the final D-FINE detector class")
        if "Robot" not in detector.names:
            raise ValueError("D-FINE detector must contain the Robot class")
        self.detector = detector
        self.head_config = head_config or MultiTaskHeadConfig()
        self.loss_config = loss_config or MultiTaskLossConfig()
        hidden_dim = detector.architecture.hidden_dim
        self.person_pose_head = QueryPoseHead(
            hidden_dim,
            person_schema,
            self.head_config.person_pose,
        )
        self.robot_pose_head = QueryPoseHead(
            hidden_dim,
            robot_schema,
            self.head_config.robot_pose,
        )
        self.shared_pose_refiner = (
            SpatialPoseRefiner(hidden_dim, self.head_config.person_pose)
            if self.head_config.person_pose.variant == "shared_spatial_refine"
            else None
        )
        self.field_feature_head = FieldFeatureHead(
            hidden_dim,
            field_schema,
            config=self.head_config.field_features,
        )
        self.schemas = {
            HeadId.PERSON_POSE: person_schema,
            HeadId.ROBOT_POSE: robot_schema,
            HeadId.FIELD_FEATURES: field_schema,
        }

    @classmethod
    def from_detection_checkpoint(
        cls,
        path: str | Path,
        *,
        person_class_name: str = "Person",
        map_location: str | torch.device = "cpu",
    ) -> "DFINEMultiTaskModel":
        """Import a v1 detector and append the public Person class."""
        detector = DFINEDetectionModel.from_checkpoint(
            path,
            map_location=map_location,
        )
        if person_class_name not in detector.names:
            detector.append_detection_class(person_class_name)
        if detector.names[-1] != person_class_name:
            raise ValueError("Person must be the final imported class")
        return cls(detector)

    @classmethod
    def from_checkpoint(
        cls,
        path: str | Path,
        *,
        map_location: str | torch.device = "cpu",
    ) -> "DFINEMultiTaskModel":
        checkpoint = torch.load(
            path,
            map_location=map_location,
            weights_only=True,
        )
        if not isinstance(checkpoint, dict):
            raise TypeError("Multi-task checkpoint must contain a dictionary")
        format_version = checkpoint.get("format_version")
        if format_version not in {2, cls.checkpoint_format_version}:
            raise ValueError("Unsupported multi-task checkpoint format")
        names = checkpoint.get("names")
        config_values = checkpoint.get("hf_config")
        architecture_values = checkpoint.get("architecture_config")
        state_dict = checkpoint.get(
            "inference_model",
            checkpoint.get("model"),
        )
        if not isinstance(names, list) or not all(
            isinstance(name, str) for name in names
        ):
            raise TypeError("Multi-task checkpoint has invalid class names")
        if not isinstance(config_values, dict):
            raise TypeError("Multi-task checkpoint has no HF configuration")
        if not isinstance(architecture_values, dict):
            raise TypeError("Multi-task checkpoint has no architecture config")
        if not isinstance(state_dict, dict):
            raise TypeError("Multi-task checkpoint has no model weights")
        config = DFineConfig.from_dict(config_values)
        detector = DFINEDetectionModel(
            DFineForObjectDetection(config),
            names,
            DFINEArchitectureConfig(**architecture_values),
        )
        head_config = MultiTaskHeadConfig()
        loss_config = MultiTaskLossConfig()
        if format_version == cls.checkpoint_format_version:
            raw_head_config = checkpoint.get("head_config")
            raw_loss_config = checkpoint.get("loss_config")
            if not isinstance(raw_head_config, dict):
                raise TypeError("Multi-task checkpoint has no head config")
            if not isinstance(raw_loss_config, dict):
                raise TypeError("Multi-task checkpoint has no loss config")
            head_config = MultiTaskHeadConfig.from_dict(raw_head_config)
            loss_config = MultiTaskLossConfig.from_dict(raw_loss_config)
        model = cls(
            detector,
            head_config=head_config,
            loss_config=loss_config,
        )
        model.load_state_dict(state_dict, strict=True)
        return model

    def checkpoint_payload(self) -> dict[str, object]:
        return {
            "format_version": self.checkpoint_format_version,
            "architecture": "dfine-multitask",
            "names": self.detector.names,
            "architecture_config": asdict(self.detector.architecture),
            "hf_config": self.detector.core.config.to_dict(),
            "head_config": asdict(self.head_config),
            "loss_config": asdict(self.loss_config),
            "schemas": {
                str(head_id): schema_to_dict(schema)
                for head_id, schema in self.schemas.items()
            },
            "model": self.state_dict(),
            "manifest": build_multitask_manifest(
                self.detector.architecture,
                self.detector.names,
                self.head_config,
                self.loss_config,
            ).to_dict(),
        }

    def forward_raw(
        self,
        images: Tensor,
        targets: list[Target] | None = None,
        active_head: HeadId | None = None,
    ) -> MultiTaskOutput:
        detection = self.detector.forward_raw(images, targets)
        query_features = _output_tensor(detection, "query_features")
        boxes = _output_tensor(detection, "pred_boxes")
        raw_encoder_features = detection.get("encoder_features")
        if not isinstance(raw_encoder_features, tuple) or not all(
            isinstance(feature, Tensor) for feature in raw_encoder_features
        ):
            raise TypeError("D-FINE output is missing encoder feature maps")
        encoder_features = cast("tuple[Tensor, ...]", raw_encoder_features)
        outputs: MultiTaskOutput = {str(HeadId.OBJECT): detection}
        person_batch_robot_negative = (
            targets is not None
            and self.loss_config.person_batch_robot_visibility_weight > 0
        )
        robot_batch_person_negative = (
            targets is not None
            and self.loss_config.robot_batch_person_visibility_weight > 0
        )
        include_person = active_head in {None, HeadId.PERSON_POSE} or (
            robot_batch_person_negative and active_head == HeadId.ROBOT_POSE
        )
        include_robot = active_head in {None, HeadId.ROBOT_POSE} or (
            person_batch_robot_negative and active_head == HeadId.PERSON_POSE
        )
        if include_person:
            outputs[str(HeadId.PERSON_POSE)] = self.person_pose_head(
                query_features,
                boxes,
                encoder_features,
                self.shared_pose_refiner,
            )
        if include_robot:
            outputs[str(HeadId.ROBOT_POSE)] = self.robot_pose_head(
                query_features,
                boxes,
                encoder_features,
                self.shared_pose_refiner,
            )
        if active_head in {None, HeadId.FIELD_FEATURES}:
            outputs[str(HeadId.FIELD_FEATURES)] = self.field_feature_head(
                encoder_features
            )
        return outputs

    def forward(
        self,
        images: Tensor,
        targets: list[Target] | None = None,
        active_head: HeadId | None = None,
    ) -> MultiTaskOutput:
        return self.forward_raw(images, targets, active_head)

    @staticmethod
    def _pixel_keypoints(keypoints: Tensor, images: Tensor) -> Tensor:
        scale = torch.tensor(
            [images.shape[-1], images.shape[-2]],
            dtype=keypoints.dtype,
            device=keypoints.device,
        )
        return keypoints * scale

    def forward_deploy(self, images: Tensor) -> dict[str, Tensor]:
        """Emit fixed-shape deployment tensors for all task heads."""
        return self._forward_deploy(images, None)

    def forward_deploy_task(
        self,
        images: Tensor,
        task: HeadId,
    ) -> dict[str, Tensor]:
        """Emit only outputs required to validate one homogeneous task."""
        return self._forward_deploy(images, task)

    def _forward_deploy(
        self,
        images: Tensor,
        active_head: HeadId | None,
    ) -> dict[str, Tensor]:
        outputs = self.forward_raw(images, active_head=active_head)
        detection = outputs[str(HeadId.OBJECT)]
        raw_detection = {
            "pred_logits": _output_tensor(detection, "pred_logits"),
            "pred_boxes": _output_tensor(detection, "pred_boxes"),
        }
        sizes = torch.tensor(
            [[images.shape[-2], images.shape[-1]]],
            dtype=raw_detection["pred_boxes"].dtype,
            device=images.device,
        ).expand(images.shape[0], -1)
        deployed: dict[str, Tensor] = {}

        normalized_objects = None
        object_queries = None
        if active_head in {None, HeadId.OBJECT, HeadId.ROBOT_POSE}:
            normalized_objects, object_queries = (
                self.detector.postprocessor.select(raw_detection)
            )
            deployed["object_output"] = (
                self.detector.postprocessor.to_pixel_xyxy(
                    normalized_objects,
                    sizes,
                )
            )

        if active_head in {None, HeadId.PERSON_POSE}:
            person = outputs[str(HeadId.PERSON_POSE)]
            person_logits = raw_detection["pred_logits"][..., -1]
            person_scores, person_queries = person_logits.sigmoid().topk(
                self.detector.architecture.num_top_queries,
                dim=1,
            )
            person_boxes = raw_detection["pred_boxes"].gather(
                1,
                person_queries[..., None].expand(-1, -1, 4),
            )
            normalized_person = torch.cat(
                (
                    person_boxes,
                    person_scores[..., None],
                    torch.zeros_like(person_scores[..., None]),
                ),
                dim=-1,
            )
            pixel_person_boxes = self.detector.postprocessor.to_pixel_xyxy(
                normalized_person,
                sizes,
            )
            person_keypoints = _head_tensor(
                person,
                "pred_keypoints",
            ).gather(
                1,
                person_queries[..., None, None].expand(
                    -1,
                    -1,
                    self.person_pose_head.schema.keypoint_count,
                    2,
                ),
            )
            person_visibility = (
                _head_tensor(person, "pred_visibility")
                .gather(
                    1,
                    person_queries[..., None].expand(
                        -1,
                        -1,
                        self.person_pose_head.schema.keypoint_count,
                    ),
                )
                .sigmoid()
            )
            visibility_alpha = (
                self.head_config.person_pose.visibility_score_alpha
            )
            if visibility_alpha > 0:
                pixel_person_boxes[..., 4] = pixel_person_boxes[..., 4] * (
                    person_visibility.mean(-1).pow(visibility_alpha)
                )
            pixel_person = self._pixel_keypoints(person_keypoints, images)
            deployed["person_pose_output"] = torch.cat(
                (
                    pixel_person_boxes,
                    torch.cat(
                        (pixel_person, person_visibility[..., None]),
                        dim=-1,
                    ).flatten(2),
                ),
                dim=-1,
            )

        if active_head in {None, HeadId.ROBOT_POSE}:
            if object_queries is None or normalized_objects is None:
                raise RuntimeError("Robot deployment requires object queries")
            robot = outputs[str(HeadId.ROBOT_POSE)]
            robot_keypoints = _head_tensor(
                robot,
                "pred_keypoints",
            ).gather(
                1,
                object_queries[..., None, None].expand(
                    -1,
                    -1,
                    self.robot_pose_head.schema.keypoint_count,
                    2,
                ),
            )
            robot_visibility = (
                _head_tensor(robot, "pred_visibility")
                .gather(
                    1,
                    object_queries[..., None].expand(
                        -1,
                        -1,
                        self.robot_pose_head.schema.keypoint_count,
                    ),
                )
                .sigmoid()
            )
            robot_class = self.detector.names.index("Robot")
            is_robot = normalized_objects[..., 5].long() == robot_class
            robot_visibility = robot_visibility * is_robot[..., None]
            deployed["robot_pose_output"] = torch.cat(
                (
                    self._pixel_keypoints(robot_keypoints, images),
                    robot_visibility[..., None],
                ),
                dim=-1,
            )

        if active_head in {None, HeadId.FIELD_FEATURES}:
            field = outputs[str(HeadId.FIELD_FEATURES)]
            field_logits = _head_tensor(field, "pred_logits")
            class_priority = (
                -torch.arange(
                    field_logits.shape[-1],
                    dtype=field_logits.dtype,
                    device=field_logits.device,
                )
                * 1e-4
            )
            field_classes = (field_logits + class_priority).argmax(-1)
            field_scores = (
                field_logits.sigmoid()
                .gather(
                    -1,
                    field_classes[..., None],
                )
                .squeeze(-1)
            )
            field_points = self._pixel_keypoints(
                _head_tensor(field, "pred_points"),
                images,
            )
            deployed["field_feature_output"] = torch.cat(
                (
                    field_points,
                    field_scores[..., None],
                    field_classes.to(field_points.dtype)[..., None],
                ),
                dim=-1,
            )
        return deployed
