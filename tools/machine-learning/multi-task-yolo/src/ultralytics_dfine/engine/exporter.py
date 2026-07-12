# ruff: noqa: TRY003

import copy
from pathlib import Path
from typing import Literal, cast

import numpy as np
import onnxruntime
import torch
import torch.nn as nn
import yaml
from scipy.optimize import linear_sum_assignment
from torch import Tensor

from ultralytics_dfine.config import build_manifest
from ultralytics_dfine.nn import DFINEDetectionModel


class DFINEExportWrapper(nn.Module):
    def __init__(
        self,
        model: DFINEDetectionModel,
        mode: Literal["raw", "end2end"] = "end2end",
    ) -> None:
        super().__init__()
        self.model = copy.deepcopy(model).eval()
        self.model.enable_onnx_compatibility()
        self.mode = mode

    def forward(self, images: Tensor) -> Tensor | tuple[Tensor, Tensor]:
        outputs = self.model.forward_raw(images)
        if self.mode == "raw":
            return cast(Tensor, outputs["pred_boxes"]), cast(
                Tensor,
                outputs["pred_logits"],
            )
        return self.model.postprocessor(outputs)


class DFINEExporter:
    def __init__(self, model: DFINEDetectionModel) -> None:
        self.model = model

    def export_onnx(
        self,
        path: str | Path,
        *,
        mode: Literal["raw", "end2end"] = "end2end",
        opset: int = 17,
        device: str | torch.device = "cpu",
        verify: bool = True,
    ) -> Path:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        torch_device = torch.device(device)
        wrapper = DFINEExportWrapper(self.model, mode).to(torch_device).eval()
        image_size = self.model.architecture.image_size
        example = torch.rand(
            1,
            3,
            image_size,
            image_size,
            device=torch_device,
        )
        output_names = ["boxes", "logits"] if mode == "raw" else ["detections"]
        torch.onnx.export(
            wrapper,
            (example,),
            destination,
            input_names=["images"],
            output_names=output_names,
            opset_version=opset,
            external_data=False,
            dynamo=False,
        )
        metadata = build_manifest(
            self.model.architecture,
            self.model.names,
        ).to_dict()
        metadata["export_mode"] = mode
        metadata["onnx_opset"] = opset
        with destination.with_suffix(".metadata.yaml").open(
            "w",
            encoding="utf-8",
        ) as file:
            yaml.safe_dump(metadata, file, sort_keys=False)
        if verify:
            self.verify_onnx(wrapper, example, destination)
        return destination

    @staticmethod
    @torch.no_grad()
    def verify_onnx(
        wrapper: DFINEExportWrapper,
        example: Tensor,
        path: str | Path,
    ) -> None:
        session = onnxruntime.InferenceSession(
            str(path),
            providers=["CPUExecutionProvider"],
        )
        runtime_outputs = [
            np.asarray(output)
            for output in session.run(
                None,
                {"images": example.detach().cpu().numpy()},
            )
        ]
        pytorch_outputs = wrapper(example)
        if isinstance(pytorch_outputs, Tensor):
            pytorch_outputs = (pytorch_outputs,)
        if len(runtime_outputs) != len(pytorch_outputs):
            raise RuntimeError("ONNX output count differs from PyTorch")
        expected_outputs = tuple(
            output.detach().cpu().numpy() for output in pytorch_outputs
        )
        if wrapper.mode == "raw":
            runtime_outputs = DFINEExporter._align_raw_queries(
                runtime_outputs,
                expected_outputs,
            )
        elif len(runtime_outputs) == 1:
            aligned, expected_prefix = DFINEExporter._align_detections(
                runtime_outputs[0],
                expected_outputs[0],
            )
            runtime_outputs = [aligned]
            expected_outputs = (expected_prefix,)
        for index, (runtime, pytorch) in enumerate(
            zip(runtime_outputs, expected_outputs, strict=True)
        ):
            np.testing.assert_allclose(
                runtime,
                pytorch,
                rtol=2e-3,
                atol=2e-4,
                err_msg=f"ONNX output {index} differs from PyTorch",
            )

    @staticmethod
    def _align_raw_queries(
        runtime: list[np.ndarray],
        expected: tuple[np.ndarray, ...],
    ) -> list[np.ndarray]:
        runtime_boxes, runtime_logits = runtime
        expected_boxes, _ = expected
        aligned_boxes = np.empty_like(runtime_boxes)
        aligned_logits = np.empty_like(runtime_logits)
        for batch_index in range(runtime_boxes.shape[0]):
            cost = np.abs(
                runtime_boxes[batch_index, :, None]
                - expected_boxes[batch_index, None, :]
            ).sum(axis=-1)
            runtime_indices, expected_indices = linear_sum_assignment(cost)
            aligned_boxes[batch_index, expected_indices] = runtime_boxes[
                batch_index, runtime_indices
            ]
            aligned_logits[batch_index, expected_indices] = runtime_logits[
                batch_index, runtime_indices
            ]
        return [aligned_boxes, aligned_logits]

    @staticmethod
    def _align_detections(
        runtime: np.ndarray,
        expected: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        aligned_batches = []
        expected_batches = []
        for runtime_batch, expected_batch in zip(
            runtime,
            expected,
            strict=True,
        ):
            confident = max(
                int((runtime_batch[:, 4] >= 0.1).sum()),
                int((expected_batch[:, 4] >= 0.1).sum()),
                20,
            )
            runtime_prefix = runtime_batch[:confident]
            expected_prefix = expected_batch[:confident]
            box_cost = np.abs(
                runtime_prefix[:, None, :4] - expected_prefix[None, :, :4]
            ).sum(axis=-1)
            score_cost = np.abs(
                runtime_prefix[:, None, 4] - expected_prefix[None, :, 4]
            )
            class_cost = (
                runtime_prefix[:, None, 5] != expected_prefix[None, :, 5]
            ) * 100
            runtime_indices, expected_indices = linear_sum_assignment(
                box_cost + score_cost + class_cost
            )
            aligned = np.empty_like(expected_prefix)
            aligned[expected_indices] = runtime_prefix[runtime_indices]
            aligned_batches.append(aligned)
            expected_batches.append(expected_prefix)
        return np.stack(aligned_batches), np.stack(expected_batches)
