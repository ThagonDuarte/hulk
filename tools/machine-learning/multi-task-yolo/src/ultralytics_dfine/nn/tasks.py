# ruff: noqa: FBT001, FBT002, FBT003, TRY003

from collections import OrderedDict
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import torch
import torch.nn as nn
from torch import Tensor
from transformers import DFineConfig, DFineForObjectDetection

from ultralytics_dfine.config import (
    DFINE_S_CHECKPOINT,
    DFINE_S_CHECKPOINT_REVISION,
    DFINEArchitectureConfig,
    build_manifest,
)
from ultralytics_dfine.loss.criterion import ModelOutput
from ultralytics_dfine.loss.matcher import Target
from ultralytics_dfine.nn.postprocess import DFINEPostProcessorAdapter


@dataclass(frozen=True)
class FeaturePyramid:
    tensors: OrderedDict[str, Tensor]
    strides: tuple[int, ...]
    channels: tuple[int, ...]


class ONNXCompatibleDFINEIntegral(nn.Module):
    """D-FINE integral without a rank-one MatMul operand."""

    def __init__(self, max_num_bins: int) -> None:
        super().__init__()
        self.max_num_bins = max_num_bins

    def forward(self, pred_corners: Tensor, project: Tensor) -> Tensor:
        batch_size, num_queries, _ = pred_corners.shape
        probabilities = torch.softmax(
            pred_corners.reshape(-1, self.max_num_bins + 1),
            dim=1,
        )
        values = (
            probabilities * project.to(pred_corners.device).unsqueeze(0)
        ).sum(dim=1)
        return values.reshape(batch_size, num_queries, -1)


class DFINEDetectionModel(nn.Module):
    """D-FINE-S task model with official training output semantics."""

    checkpoint_format_version = 1

    def __init__(
        self,
        core: DFineForObjectDetection,
        names: list[str],
        architecture: DFINEArchitectureConfig | None = None,
    ) -> None:
        super().__init__()
        if len(names) != core.config.num_labels:
            raise ValueError("Class names do not match D-FINE num_labels")
        self.core = core
        self.names = list(names)
        self.nc = len(names)
        self.architecture = architecture or DFINEArchitectureConfig()
        self.stride = torch.tensor(self.architecture.feature_strides)
        self.end2end = True
        self.task = "detect"
        self.postprocessor = DFINEPostProcessorAdapter(
            self.nc,
            self.architecture.num_top_queries,
        )
        self._frozen_roles: set[str] = set()

    @classmethod
    def from_pretrained(
        cls,
        names: list[str],
        *,
        architecture: DFINEArchitectureConfig | None = None,
    ) -> "DFINEDetectionModel":
        id_to_label = dict(enumerate(names))
        label_to_id = {name: index for index, name in id_to_label.items()}
        core = DFineForObjectDetection.from_pretrained(
            DFINE_S_CHECKPOINT,
            revision=DFINE_S_CHECKPOINT_REVISION,
            num_labels=len(names),
            id2label=id_to_label,
            label2id=label_to_id,
            ignore_mismatched_sizes=True,
        )
        return cls(core, names, architecture)

    @classmethod
    def from_checkpoint(
        cls,
        path: str | Path,
        *,
        map_location: str | torch.device = "cpu",
    ) -> "DFINEDetectionModel":
        checkpoint = torch.load(
            path, map_location=map_location, weights_only=True
        )
        if not isinstance(checkpoint, dict):
            raise TypeError("D-FINE checkpoint must contain a dictionary")
        if checkpoint.get("format_version") != cls.checkpoint_format_version:
            raise ValueError("Unsupported D-FINE checkpoint format")
        names = checkpoint.get("names")
        config_values = checkpoint.get("hf_config")
        state_dict = checkpoint.get(
            "inference_model",
            checkpoint.get("model"),
        )
        architecture_values = checkpoint.get("architecture_config", {})
        if not isinstance(names, list) or not all(
            isinstance(name, str) for name in names
        ):
            raise TypeError("D-FINE checkpoint has invalid class names")
        if not isinstance(config_values, dict) or not isinstance(
            state_dict, dict
        ):
            raise TypeError(
                "D-FINE checkpoint is missing model configuration or weights"
            )
        config = DFineConfig.from_dict(config_values)
        model = cls(
            DFineForObjectDetection(config),
            names,
            DFINEArchitectureConfig(**architecture_values),
        )
        model.load_state_dict(state_dict, strict=True)
        return model

    @property
    def backbone(self) -> nn.Module:
        return self.core.model.backbone

    @property
    def encoder(self) -> nn.Module:
        return self.core.model.encoder

    @property
    def decoder(self) -> nn.Module:
        return self.core.model.decoder

    def replace_backbone(self, backbone: nn.Module) -> None:
        self.core.model.backbone = (  # pyright: ignore[reportAttributeAccessIssue]
            backbone
        )

    def enable_onnx_compatibility(self) -> None:
        self.core.model.decoder.integral = (  # pyright: ignore[reportAttributeAccessIssue]
            ONNXCompatibleDFINEIntegral(self.core.config.max_num_bins)
        )

    @staticmethod
    def _to_hf_targets(targets: list[Target]) -> list[dict[str, Tensor]]:
        return [
            {
                "class_labels": target["labels"],
                "boxes": target["boxes"],
            }
            for target in targets
        ]

    @staticmethod
    def _split_queries(
        tensor: Tensor,
        split: list[int] | tuple[int, ...],
        *,
        dimension: int,
    ) -> tuple[Tensor, Tensor]:
        values = torch.split(tensor, list(split), dim=dimension)
        if len(values) != 2:
            raise ValueError("D-FINE denoising split must have two parts")
        return values[0], values[1]

    def _training_outputs(self, raw: Any) -> ModelOutput:
        all_logits = raw.intermediate_logits
        all_boxes = raw.intermediate_reference_points
        all_corners = raw.intermediate_predicted_corners
        all_references = raw.initial_reference_points
        if not all(
            isinstance(value, Tensor)
            for value in (all_logits, all_boxes, all_corners, all_references)
        ):
            raise TypeError(
                "D-FINE model did not return complete training tensors"
            )

        metadata = raw.denoising_meta_values
        denoising: tuple[Tensor, Tensor, Tensor, Tensor] | None = None
        denoising_preliminary: tuple[Tensor, Tensor] | None = None
        if isinstance(metadata, dict):
            split = metadata.get("dn_num_split")
            if not isinstance(split, (list, tuple)):
                raise TypeError("D-FINE denoising split is invalid")
            dn_logits, all_logits = self._split_queries(
                all_logits,
                split,
                dimension=2,
            )
            dn_boxes, all_boxes = self._split_queries(
                all_boxes,
                split,
                dimension=2,
            )
            dn_corners, all_corners = self._split_queries(
                all_corners,
                split,
                dimension=2,
            )
            dn_references, all_references = self._split_queries(
                all_references,
                split,
                dimension=2,
            )
            denoising_preliminary = (dn_logits[:, 0], dn_boxes[:, 0])
            denoising = (
                dn_logits[:, 1:],
                dn_boxes[:, 1:],
                dn_corners,
                dn_references,
            )

        preliminary_logits = all_logits[:, 0]
        preliminary_boxes = all_boxes[:, 0]
        decoder_logits = all_logits[:, 1:]
        decoder_boxes = all_boxes[:, 1:]
        final_logits = decoder_logits[:, -1]
        final_boxes = decoder_boxes[:, -1]
        final_corners = all_corners[:, -1]
        final_references = all_references[:, -1]
        output: ModelOutput = {
            "pred_logits": final_logits,
            "pred_boxes": final_boxes,
            "pred_corners": final_corners,
            "ref_points": final_references,
            "up": self.core.model.decoder.up,
            "reg_scale": self.core.model.decoder.reg_scale,
            "aux_outputs": [
                {
                    "pred_logits": logits,
                    "pred_boxes": boxes,
                    "pred_corners": corners,
                    "ref_points": references,
                    "teacher_corners": final_corners,
                    "teacher_logits": final_logits,
                }
                for logits, boxes, corners, references in zip(
                    decoder_logits[:, :-1].unbind(1),
                    decoder_boxes[:, :-1].unbind(1),
                    all_corners[:, :-1].unbind(1),
                    all_references[:, :-1].unbind(1),
                    strict=True,
                )
            ],
            "enc_aux_outputs": [
                {
                    "pred_logits": raw.enc_topk_logits,
                    "pred_boxes": raw.enc_topk_bboxes,
                }
            ],
            "pre_outputs": {
                "pred_logits": preliminary_logits,
                "pred_boxes": preliminary_boxes,
            },
            "enc_meta": {"class_agnostic": False},
        }
        if denoising is not None and denoising_preliminary is not None:
            dn_logits, dn_boxes, dn_corners, dn_references = denoising
            dn_teacher_corners = dn_corners[:, -1]
            dn_teacher_logits = dn_logits[:, -1]
            output["dn_outputs"] = [
                {
                    "pred_logits": logits,
                    "pred_boxes": boxes,
                    "pred_corners": corners,
                    "ref_points": references,
                    "teacher_corners": dn_teacher_corners,
                    "teacher_logits": dn_teacher_logits,
                }
                for logits, boxes, corners, references in zip(
                    dn_logits.unbind(1),
                    dn_boxes.unbind(1),
                    dn_corners.unbind(1),
                    dn_references.unbind(1),
                    strict=True,
                )
            ]
            output["dn_pre_outputs"] = {
                "pred_logits": denoising_preliminary[0],
                "pred_boxes": denoising_preliminary[1],
            }
            output["dn_meta"] = metadata
        return output

    def forward_raw(
        self,
        images: Tensor,
        targets: list[Target] | None = None,
    ) -> ModelOutput:
        labels = self._to_hf_targets(targets) if targets is not None else None
        raw = self.core.model(
            pixel_values=images,
            labels=labels,
            return_dict=True,
        )
        if self.training and targets is not None:
            return self._training_outputs(raw)
        return {
            "pred_logits": raw.intermediate_logits[:, -1],
            "pred_boxes": raw.intermediate_reference_points[:, -1],
        }

    def forward(
        self,
        images: Tensor,
        targets: list[Target] | None = None,
    ) -> ModelOutput:
        return self.forward_raw(images, targets)

    def forward_deploy(self, images: Tensor) -> Tensor:
        return self.postprocessor(self.forward_raw(images))

    def forward_features(self, images: Tensor) -> FeaturePyramid:
        mask = torch.ones(
            images.shape[0],
            images.shape[2],
            images.shape[3],
            dtype=torch.bool,
            device=images.device,
        )
        features = self.core.model.backbone(images, mask)
        tensors = OrderedDict(
            (f"P{index + 3}", feature)
            for index, (feature, _) in enumerate(features)
        )
        return FeaturePyramid(
            tensors=tensors,
            strides=self.architecture.feature_strides,
            channels=(256, 512, 1024),
        )

    def freeze_roles(self, roles: list[str]) -> None:
        modules = {
            "backbone": self.backbone,
            "encoder": self.encoder,
            "decoder": self.decoder,
            "decoder.class_heads": self.core.class_embed,
            "decoder.box_heads": self.core.bbox_embed,
            "decoder.encoder_score_head": self.core.model.enc_score_head,
            "decoder.denoising_class_embed": (
                self.core.model.denoising_class_embed
            ),
        }
        unknown = set(roles) - modules.keys()
        if unknown:
            raise KeyError(f"Unknown D-FINE roles: {sorted(unknown)}")
        for role in roles:
            module = modules[role]
            module.requires_grad_(False)
            module.eval()
            self._frozen_roles.add(role)

    def train(self, mode: bool = True) -> "DFINEDetectionModel":
        super().train(mode)
        if mode and self._frozen_roles:
            modules = {
                "backbone": self.backbone,
                "encoder": self.encoder,
                "decoder": self.decoder,
                "decoder.class_heads": self.core.class_embed,
                "decoder.box_heads": self.core.bbox_embed,
                "decoder.encoder_score_head": self.core.model.enc_score_head,
                "decoder.denoising_class_embed": (
                    self.core.model.denoising_class_embed
                ),
            }
            for role in self._frozen_roles:
                modules[role].eval()
        return self

    def checkpoint_payload(self) -> dict[str, object]:
        return {
            "format_version": self.checkpoint_format_version,
            "architecture": "dfine",
            "variant": self.architecture.variant,
            "names": self.names,
            "architecture_config": asdict(self.architecture),
            "hf_config": self.core.config.to_dict(),
            "model": self.state_dict(),
            "manifest": build_manifest(
                self.architecture,
                self.names,
            ).to_dict(),
        }
