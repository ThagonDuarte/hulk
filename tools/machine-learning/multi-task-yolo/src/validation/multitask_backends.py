"""Inference adapters shared by standalone multi-task validation backends."""

# ruff: noqa: TRY003

from collections import defaultdict
from collections.abc import Mapping
from pathlib import Path
from types import SimpleNamespace
from typing import Any, cast

import numpy as np
import onnxruntime
import torch
import torch.nn as nn
import yaml
from torch import Tensor

from ultralytics_dfine.nn import DFINEMultiTaskModel
from ultralytics_dfine.schemas import HeadId
from utils.nv12_to_rgb import NV12ToRgb

OUTPUTS_BY_TASK: dict[HeadId, tuple[str, ...]] = {
    HeadId.OBJECT: ("object_output",),
    HeadId.PERSON_POSE: ("person_pose_output",),
    HeadId.ROBOT_POSE: ("object_output", "robot_pose_output"),
    HeadId.FIELD_FEATURES: ("field_feature_output",),
}
PUBLIC_OUTPUT_WIDTHS: dict[str, tuple[int, ...]] = {
    "object_output": (300, 6),
    "person_pose_output": (300, 57),
    "robot_pose_output": (300, 14, 3),
    "field_feature_output": (300, 4),
}


def rgb_to_packed_nv12(images: Tensor) -> Tensor:
    """Convert normalized RGB BCHW tensors to packed uint8 NV12 frames."""
    if images.ndim != 4 or images.shape[1] != 3:
        raise ValueError("RGB input must have shape [batch, 3, height, width]")
    height, width = images.shape[-2:]
    if height % 2 or width % 2:
        raise ValueError("NV12 requires even image height and width")
    rgb = images.to(torch.float32).clamp(0, 1) * 255.0
    red, green, blue = rgb.unbind(dim=1)
    luminance = 0.299 * red + 0.587 * green + 0.114 * blue
    chroma_u = -0.168736 * red - 0.331264 * green + 0.5 * blue + 128.0
    chroma_v = 0.5 * red - 0.418688 * green - 0.081312 * blue + 128.0
    chroma = torch.stack((chroma_u, chroma_v), dim=1)
    chroma = torch.nn.functional.avg_pool2d(chroma, kernel_size=2, stride=2)
    interleaved = chroma.permute(0, 2, 3, 1)
    flat = torch.cat(
        (luminance.flatten(1), interleaved.flatten(1)),
        dim=1,
    )
    return (
        flat.round()
        .clamp(0, 255)
        .to(torch.uint8)
        .reshape(
            images.shape[0],
            height // 2,
            width // 2,
            6,
        )
    )


def packed_nv12_to_rgb(frames: Tensor) -> Tensor:
    """Decode packed NV12 frames with the exact deployment preprocessor."""
    if frames.ndim != 4 or frames.shape[-1] != 6:
        raise ValueError("Packed NV12 must have shape [batch, h/2, w/2, 6]")
    converter = NV12ToRgb(subsample=False).to(frames.device)
    decoded = [converter(frame).permute(2, 0, 1) for frame in frames]
    return torch.stack(decoded)


class Nv12RoundtripModel(nn.Module):
    """Apply production chroma subsampling before PyTorch inference."""

    def __init__(self, model: DFINEMultiTaskModel) -> None:
        super().__init__()
        self.model = model

    @property
    def detector(self) -> nn.Module:
        return self.model.detector

    def forward_deploy(self, images: Tensor) -> dict[str, Tensor]:
        converted = packed_nv12_to_rgb(rgb_to_packed_nv12(images))
        return self.model.forward_deploy(converted)

    def forward_deploy_task(
        self,
        images: Tensor,
        task: HeadId,
    ) -> dict[str, Tensor]:
        converted = packed_nv12_to_rgb(rgb_to_packed_nv12(images))
        return self.model.forward_deploy_task(converted, task)


def _load_manifest(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as file:
        value = yaml.safe_load(file)
    if not isinstance(value, dict):
        raise TypeError("ONNX manifest must contain a mapping")
    return cast("dict[str, Any]", value)


class OnnxNv12Model(nn.Module):
    """Expose a packed-NV12 ONNX model through the validator interface."""

    def __init__(
        self,
        model_path: str | Path,
        manifest_path: str | Path,
        *,
        height: int,
        width: int,
        providers: list[str] | None = None,
    ) -> None:
        super().__init__()
        self.model_path = Path(model_path).resolve()
        self.manifest_path = Path(manifest_path).resolve()
        self.manifest = _load_manifest(self.manifest_path)
        class_names = self.manifest.get("class_names")
        if not isinstance(class_names, list) or not all(
            isinstance(name, str) for name in class_names
        ):
            raise TypeError("ONNX manifest has invalid class_names")
        preprocessing = self.manifest.get("preprocessing")
        if not isinstance(preprocessing, dict):
            raise TypeError("ONNX manifest has no preprocessing mapping")
        declared_size = preprocessing.get("input_size")
        if declared_size != [height, width]:
            raise ValueError(
                "ONNX manifest input size differs from validation size: "
                f"manifest={declared_size}, requested={[height, width]}"
            )
        self.detector = cast(
            nn.Module,
            SimpleNamespace(names=list(class_names)),
        )
        self.session = onnxruntime.InferenceSession(
            str(self.model_path),
            providers=providers or ["CPUExecutionProvider"],
        )
        inputs = self.session.get_inputs()
        if len(inputs) != 1:
            raise ValueError("ONNX model must have exactly one input")
        self.input_name = inputs[0].name
        self.input_rank = len(inputs[0].shape)
        if self.input_rank not in (3, 4):
            raise ValueError("ONNX NV12 input must have rank three or four")
        output_names = {output.name for output in self.session.get_outputs()}
        missing = set(PUBLIC_OUTPUT_WIDTHS).difference(output_names)
        if missing:
            raise ValueError(
                "ONNX model is missing public outputs: "
                + ", ".join(sorted(missing))
            )

    def _run(self, images: Tensor) -> dict[str, Tensor]:
        packed = rgb_to_packed_nv12(images).cpu().numpy()
        batches: dict[str, list[Tensor]] = defaultdict(list)
        for frame in packed:
            value = frame if self.input_rank == 3 else frame[None]
            outputs = self.session.run(None, {self.input_name: value})
            for metadata, output in zip(
                self.session.get_outputs(),
                outputs,
                strict=True,
            ):
                tensor = torch.from_numpy(cast(np.ndarray, output))
                if tensor.shape[0] != 1:
                    raise ValueError(
                        "ONNX frame inference must return batch one"
                    )
                batches[metadata.name].append(tensor)
        result = {
            name: torch.cat(values, dim=0) for name, values in batches.items()
        }
        for name, suffix in PUBLIC_OUTPUT_WIDTHS.items():
            if tuple(result[name].shape[1:]) != suffix:
                raise ValueError(
                    f"ONNX output '{name}' has shape "
                    f"{tuple(result[name].shape)}"
                )
        return result

    def forward_deploy(self, images: Tensor) -> dict[str, Tensor]:
        return self._run(images)

    def forward_deploy_task(
        self,
        images: Tensor,
        task: HeadId,
    ) -> dict[str, Tensor]:
        outputs = self._run(images)
        return {name: outputs[name] for name in OUTPUTS_BY_TASK[task]}


def backend_metadata(model: nn.Module) -> Mapping[str, object]:
    """Return stable backend details for compatibility comparisons."""
    if isinstance(model, OnnxNv12Model):
        return {
            "backend": "onnx-nv12",
            "model": str(model.model_path),
            "manifest": str(model.manifest_path),
            "providers": model.session.get_providers(),
        }
    if isinstance(model, Nv12RoundtripModel):
        return {"backend": "pytorch-nv12-roundtrip"}
    return {"backend": "pytorch-rgb"}
