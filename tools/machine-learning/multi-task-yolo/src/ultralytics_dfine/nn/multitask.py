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


class QueryPoseHead(nn.Module):
    """Predict box-relative keypoints for each final D-FINE query."""

    def __init__(
        self,
        hidden_dim: int,
        schema: PoseSchemaConfig,
    ) -> None:
        super().__init__()
        self.schema = schema
        self.network = nn.Sequential(
            nn.Linear(hidden_dim + 4, hidden_dim),
            nn.GELU(),
            nn.Linear(hidden_dim, schema.keypoint_count * 3),
        )

    def forward(self, query_features: Tensor, boxes: Tensor) -> HeadOutput:
        if query_features.shape[:2] != boxes.shape[:2]:
            raise ValueError("Pose queries and boxes must be aligned")
        values = self.network(torch.cat((query_features, boxes), dim=-1))
        values = values.unflatten(-1, (self.schema.keypoint_count, 3))
        offsets = values[..., :2].tanh()
        centers = boxes[..., None, :2]
        sizes = boxes[..., None, 2:]
        keypoints = (centers + offsets * sizes).clamp(0.0, 1.0)
        return {
            "pred_keypoints": keypoints,
            "pred_visibility": values[..., 2],
        }


class FieldFeatureHead(nn.Module):
    """Decode a fixed point set from shared HybridEncoder feature maps."""

    def __init__(
        self,
        hidden_dim: int,
        schema: PointSetSchemaConfig = FIELD_FEATURE_SCHEMA,
        *,
        decoder_layers: int = 2,
        attention_heads: int = 8,
        feature_levels: int = 3,
    ) -> None:
        super().__init__()
        self.schema = schema
        self.hidden_dim = hidden_dim
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
            torch.empty(feature_levels, hidden_dim)
        )
        self.classifier = nn.Linear(hidden_dim, len(schema.class_names))
        self.point_predictor = nn.Linear(hidden_dim, 2)
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
            sampled = functional.grid_sample(
                feature,
                grid,
                mode="bilinear",
                padding_mode="border",
                align_corners=False,
            )
            sampled = sampled.squeeze(-1).transpose(1, 2)
            sampled_levels.append(sampled + self.level_embeddings[index])
        local = torch.stack(sampled_levels).mean(0)
        return local + self.queries.weight.unsqueeze(0)

    def forward(self, features: tuple[Tensor, ...]) -> HeadOutput:
        memory = self._flatten_memory(features)
        queries = self._local_queries(features)
        decoded = self.decoder(queries, memory)
        offsets = self.point_predictor(decoded).tanh()
        return {
            "pred_logits": self.classifier(decoded),
            "pred_points": (
                self.reference_points.unsqueeze(0) + offsets * self.point_scale
            ).clamp(0, 1),
        }


class DFINEMultiTaskModel(nn.Module):
    """One D-FINE detector with person, robot, and field-feature heads."""

    checkpoint_format_version = 2

    def __init__(
        self,
        detector: DFINEDetectionModel,
        *,
        person_schema: PoseSchemaConfig = PERSON_POSE_SCHEMA,
        robot_schema: PoseSchemaConfig = ROBOT_POSE_SCHEMA,
        field_schema: PointSetSchemaConfig = FIELD_FEATURE_SCHEMA,
    ) -> None:
        super().__init__()
        if not detector.names or detector.names[-1] != "Person":
            raise ValueError("Person must be the final D-FINE detector class")
        if "Robot" not in detector.names:
            raise ValueError("D-FINE detector must contain the Robot class")
        self.detector = detector
        hidden_dim = detector.architecture.hidden_dim
        self.person_pose_head = QueryPoseHead(hidden_dim, person_schema)
        self.robot_pose_head = QueryPoseHead(hidden_dim, robot_schema)
        self.field_feature_head = FieldFeatureHead(hidden_dim, field_schema)
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
        if checkpoint.get("format_version") != cls.checkpoint_format_version:
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
        model = cls(detector)
        model.load_state_dict(state_dict, strict=True)
        return model

    def checkpoint_payload(self) -> dict[str, object]:
        return {
            "format_version": self.checkpoint_format_version,
            "architecture": "dfine-multitask",
            "names": self.detector.names,
            "architecture_config": asdict(self.detector.architecture),
            "hf_config": self.detector.core.config.to_dict(),
            "schemas": {
                str(head_id): schema_to_dict(schema)
                for head_id, schema in self.schemas.items()
            },
            "model": self.state_dict(),
            "manifest": build_multitask_manifest(
                self.detector.architecture,
                self.detector.names,
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
        if active_head in {None, HeadId.PERSON_POSE}:
            outputs[str(HeadId.PERSON_POSE)] = self.person_pose_head(
                query_features,
                boxes,
            )
        if active_head in {None, HeadId.ROBOT_POSE}:
            outputs[str(HeadId.ROBOT_POSE)] = self.robot_pose_head(
                query_features,
                boxes,
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
