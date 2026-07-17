# ruff: noqa: TRY003

import hashlib
import json
import os
from collections.abc import Iterable, Mapping
from pathlib import Path
from typing import Any, cast

import click
import numpy as np
import onnxruntime
import torch
import yaml
from scipy.optimize import linear_sum_assignment
from torch import ByteTensor, Tensor, nn

from model.hydra import Hydra
from ultralytics_dfine.nn import DFINEDetectionModel
from utils.model_naming import (
    HYDRA_MODEL_NAME_TYPE,
    HydraModelName,
    ModelFamily,
    TaskType,
)
from utils.nv12_to_rgb import NV12ToRgb


class InvalidHydraOutputError(TypeError):
    def __init__(self, output_name: str, actual_type: type) -> None:
        super().__init__(
            f"Hydra output '{output_name}' must be a tensor, got {actual_type}"
        )


class HydraWrapper(nn.Module):
    def __init__(
        self, hydra_model: Hydra, task_dict: dict[TaskType, Path]
    ) -> None:
        super().__init__()
        self.hydra = hydra_model
        self.task_dict = task_dict

    def forward(self, x: Tensor) -> Tensor | tuple[Tensor, ...]:
        outputs = self.hydra(x)
        if not isinstance(outputs, Mapping):
            raise TypeError("Hydra model output must be a mapping")

        selected_outputs: list[Tensor] = []
        for output_name in self.hydra.deployment_output_names:
            head_output = outputs.get(output_name)
            if not isinstance(head_output, torch.Tensor):
                raise InvalidHydraOutputError(output_name, type(head_output))
            selected_outputs.append(head_output)

        if len(selected_outputs) == 1:
            return selected_outputs[0]
        return tuple(selected_outputs)


class HydraNv12Wrapper(nn.Module):
    def __init__(self, hydra_wrapper: HydraWrapper) -> None:
        super().__init__()
        self.hydra_wrapper = hydra_wrapper
        self.preprocessor = NV12ToRgb(subsample=False)

    def forward(self, x: ByteTensor) -> Tensor | tuple[Tensor, ...]:
        rgb = self.preprocessor(x).unsqueeze(0).permute(0, 3, 1, 2)
        return self.hydra_wrapper(rgb)


def refresh_manifest_config_hash(manifest: dict[str, Any]) -> None:
    """Update the manifest digest after export-specific metadata changes."""
    values = dict(manifest)
    values.pop("config_hash", None)
    encoded = json.dumps(values, sort_keys=True).encode()
    manifest["config_hash"] = hashlib.sha256(encoded).hexdigest()


def set_export_mode(module: nn.Module) -> None:
    for child in module.modules():
        if isinstance(child, DFINEDetectionModel):
            child.enable_onnx_compatibility()
        if hasattr(child, "export"):
            cast(Any, child).export = True


def build_task_dict(
    hydra_model_name: HydraModelName,
    train_folder_path: Path,
    val_folder_path: Path,
) -> dict[TaskType, Path]:
    if hydra_model_name.family() == ModelFamily.DFINE:
        return {
            head.task_type(): (
                train_folder_path
                / hydra_model_name.integrated_model_name(head)
                / "best.pt"
                if head.is_finetuned_model()
                else val_folder_path
                / hydra_model_name.integrated_model_name(head)
                / (hydra_model_name.integrated_model_name(head) + ".pt")
            )
            for head in hydra_model_name.heads
        }
    return {
        head.task_type(): (
            train_folder_path
            / hydra_model_name.integrated_model_name(head)
            / "weights/best.pt"
            if head.is_finetuned_model()
            else val_folder_path
            / hydra_model_name.integrated_model_name(head)
            / (hydra_model_name.integrated_model_name(head) + ".pt")
        )
        for head in hydra_model_name.heads
    }


def export_onnx(
    wrapper: nn.Module,
    dummy_input: Tensor,
    export_path: Path,
    output_names: Iterable[str],
    opset: int,
    *,
    with_nv12: bool,
    static_shapes: bool = False,
    verify: bool = True,
) -> None:
    set_export_mode(wrapper)
    wrapper.eval()
    input_name = "images"
    dynamic_axes: dict[str, dict[int, str]]
    if with_nv12:
        input_name = "raw_bytes_input"
        dynamic_axes = {
            input_name: {0: "half_height", 1: "half_width"},
        }
    else:
        dynamic_axes = {
            input_name: {0: "batch_size", 2: "height", 3: "width"},
        }

    output_names = list(output_names)
    for name in output_names:
        dynamic_axes[name] = {0: "batch_size", 1: "num_predictions"}

    mha_fastpath = torch.backends.mha.get_fastpath_enabled()
    torch.backends.mha.set_fastpath_enabled(False)
    try:
        torch.onnx.export(
            wrapper,
            (dummy_input,),
            export_path,
            input_names=[input_name],
            output_names=output_names,
            dynamic_axes=None if static_shapes else dynamic_axes,
            opset_version=opset,
            external_data=False,
            dynamo=False,
        )
    finally:
        torch.backends.mha.set_fastpath_enabled(mha_fastpath)
    if verify:
        verify_onnx(wrapper, dummy_input, export_path, input_name)


@torch.no_grad()
def verify_onnx(
    wrapper: nn.Module,
    example: Tensor,
    export_path: Path,
    input_name: str,
) -> None:
    """Compare every fixed output without tolerating query reordering."""
    session = onnxruntime.InferenceSession(
        str(export_path),
        providers=["CPUExecutionProvider"],
    )
    runtime_outputs = session.run(
        None,
        {input_name: example.detach().cpu().numpy()},
    )
    expected = wrapper(example)
    expected_outputs = (expected,) if isinstance(expected, Tensor) else expected
    if len(runtime_outputs) != len(expected_outputs):
        raise RuntimeError("ONNX output count differs from PyTorch")
    names = [output.name for output in session.get_outputs()]
    expected_arrays = tuple(
        output.detach().cpu().numpy() for output in expected_outputs
    )
    if names == [
        "object_output",
        "person_pose_output",
        "robot_pose_output",
        "field_feature_output",
    ]:
        comparisons = _aligned_multitask_outputs(
            [cast(np.ndarray, output) for output in runtime_outputs],
            expected_arrays,
        )
    else:
        comparisons = list(zip(runtime_outputs, expected_arrays, strict=True))
    for index, (runtime, pytorch) in enumerate(comparisons):
        np.testing.assert_allclose(
            cast(np.ndarray, runtime),
            pytorch,
            rtol=2e-2,
            atol=2e-1,
            err_msg=f"ONNX output {index} differs from PyTorch",
        )


def _aligned_rows(
    runtime: np.ndarray,
    expected: np.ndarray,
    *,
    coordinate_count: int,
    score_index: int,
    class_index: int,
    minimum_stable: int = 20,
    maximum_coordinate_delta: float = 5.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    aligned_runtime = []
    aligned_expected = []
    runtime_indices = []
    expected_indices = []
    for runtime_batch, expected_batch in zip(
        runtime,
        expected,
        strict=True,
    ):
        count = min(
            max(
                int((runtime_batch[:, score_index] >= 0.1).sum()),
                int((expected_batch[:, score_index] >= 0.1).sum()),
                20,
            ),
            50,
        )
        runtime_prefix = runtime_batch[:count]
        expected_prefix = expected_batch[:count]
        coordinate_cost = np.abs(
            runtime_prefix[:, None, :coordinate_count]
            - expected_prefix[None, :, :coordinate_count]
        ).sum(axis=-1)
        score_cost = np.abs(
            runtime_prefix[:, None, score_index]
            - expected_prefix[None, :, score_index]
        )
        class_cost = (
            runtime_prefix[:, None, class_index]
            != expected_prefix[None, :, class_index]
        ) * 1_000
        runtime_order, expected_order = linear_sum_assignment(
            coordinate_cost + score_cost + class_cost
        )
        order = np.argsort(expected_order)
        runtime_order = runtime_order[order]
        expected_order = expected_order[order]
        compatible = (
            runtime_prefix[runtime_order, class_index]
            == expected_prefix[expected_order, class_index]
        ) & (
            np.abs(
                runtime_prefix[runtime_order, :coordinate_count]
                - expected_prefix[expected_order, :coordinate_count]
            ).sum(axis=1)
            < maximum_coordinate_delta
        )
        if int(compatible.sum()) < minimum_stable:
            raise RuntimeError("Too few stable ONNX predictions for parity")
        runtime_order = runtime_order[compatible]
        expected_order = expected_order[compatible]
        aligned_runtime.append(runtime_prefix[runtime_order])
        aligned_expected.append(expected_prefix[expected_order])
        runtime_indices.append(runtime_order)
        expected_indices.append(expected_order)
    return (
        np.stack(aligned_runtime),
        np.stack(aligned_expected),
        np.stack(runtime_indices),
        np.stack(expected_indices),
    )


def _aligned_multitask_outputs(
    runtime: list[np.ndarray],
    expected: tuple[np.ndarray, ...],
) -> list[tuple[np.ndarray, np.ndarray]]:
    runtime_objects, expected_objects, runtime_indices, expected_indices = (
        _aligned_rows(
            runtime[0],
            expected[0],
            coordinate_count=4,
            score_index=4,
            class_index=5,
        )
    )
    runtime_person, expected_person, _, _ = _aligned_rows(
        runtime[1],
        expected[1],
        coordinate_count=4,
        score_index=4,
        class_index=5,
        minimum_stable=5,
        maximum_coordinate_delta=20,
    )
    aligned_runtime_robot = np.stack(
        [
            batch[indices]
            for batch, indices in zip(
                runtime[2],
                runtime_indices,
                strict=True,
            )
        ]
    )
    aligned_expected_robot = np.stack(
        [
            batch[indices]
            for batch, indices in zip(
                expected[2],
                expected_indices,
                strict=True,
            )
        ]
    )
    return [
        (runtime_objects, expected_objects),
        (runtime_person, expected_person),
        (aligned_runtime_robot, aligned_expected_robot),
        (runtime[3], expected[3]),
    ]


def export_torchscript(
    wrapper: nn.Module,
    dummy_input: Tensor,
    export_path: Path,
) -> None:
    traced = torch.jit.trace(
        wrapper,
        (dummy_input,),
        strict=False,
        check_trace=False,
    )
    if isinstance(traced, tuple):
        raise TypeError("Unexpected trace return type")
    cast(torch.jit.ScriptModule, traced).save(str(export_path))


@click.command(
    context_settings={"help_option_names": ["-h", "--help"]},
    help=(
        "Export one or more Hydra models to ONNX or TorchScript format.\n\n"
        "Arguments:\n\n"
        "  HYDRA_MODEL_NAME  One or more Hydra model names to export\n\n"
        "  EXPORT_FOLDER     Destination folder for the exported model(s)"
    ),
)
@click.argument(
    "hydra-model-names",
    nargs=-1,
    type=HYDRA_MODEL_NAME_TYPE,
)
@click.argument(
    "export-folder",
    nargs=1,
    type=click.Path(path_type=Path),
)
@click.option(
    "--runs_dir",
    type=Path,
    default=Path("runs"),
    help="Directory to save training runs.",
)
@click.option(
    "--val_dir",
    type=Path,
    default=Path("val"),
    help="Directory to save validation runs. Relative to `--runs_dir`.",
)
@click.option(
    "--train_dir",
    type=Path,
    default=Path("train"),
    help="Directory to save validation runs. Relative to `--runs_dir`.",
)
@click.option(
    "--imgsz",
    type=int,
    default=640,
    show_default=True,
    help="Default square input size used when width or height is omitted.",
)
@click.option(
    "--width",
    type=int,
    help="Input image width. Defaults to --imgsz.",
)
@click.option(
    "--height",
    type=int,
    help="Input image height. Defaults to --imgsz.",
)
@click.option(
    "--opset",
    type=int,
    default=20,
    show_default=True,
    help="ONNX opset version.",
)
@click.option(
    "--format",
    "export_format",
    type=click.Choice(["onnx", "pt"], case_sensitive=False),
    default="onnx",
    show_default=True,
    help="Export format: ONNX or TorchScript .pt.",
)
@click.option(
    "--device",
    default="cpu",
    show_default=True,
    help="Torch device for export, e.g. cpu or cuda:0.",
)
@click.option(
    "--with-nv12-layer",
    is_flag=True,
    default=False,
    help="Add NV12 preprocessing layer before Hydra model.",
)
def main(  # noqa: C901
    hydra_model_names: list[HydraModelName],
    export_folder: Path,
    *,
    runs_dir: Path,
    val_dir: Path,
    train_dir: Path,
    imgsz: int,
    width: int | None,
    height: int | None,
    opset: int,
    export_format: str,
    device: str,
    with_nv12_layer: bool,
) -> None:
    if imgsz <= 0:
        raise click.BadParameter("--imgsz must be > 0")
    input_width = imgsz if width is None else width
    input_height = imgsz if height is None else height
    if input_width <= 0:
        raise click.BadParameter("--width must be > 0")
    if input_height <= 0:
        raise click.BadParameter("--height must be > 0")

    train_folder_path = runs_dir / train_dir
    val_folder_path = runs_dir / val_dir

    for hydra_model_name in hydra_model_names:
        backbone = hydra_model_name.backbone

        task_dict = build_task_dict(
            hydra_model_name=hydra_model_name,
            train_folder_path=train_folder_path,
            val_folder_path=val_folder_path,
        )

        hydra_model = Hydra(
            backbone_path=str(backbone),
            task_dict=task_dict,
            number_of_frozen_modules=(
                hydra_model_name.number_of_frozen_modules
            ),
            family=hydra_model_name.family(),
        ).to(device)
        hydra_model.eval()
        set_export_mode(hydra_model)

        base_wrapper = HydraWrapper(hydra_model, task_dict=task_dict).to(device)
        wrapper: nn.Module = base_wrapper
        if with_nv12_layer:
            wrapper = HydraNv12Wrapper(base_wrapper).to(device)
        wrapper.eval()

        export_folder.mkdir(parents=True, exist_ok=True)

        if with_nv12_layer:
            if input_width % 2 != 0 or input_height % 2 != 0:
                raise click.BadParameter(
                    "--width and --height must be even for NV12"
                )
            dummy_input = torch.zeros(
                (input_height // 2, input_width // 2, 6),
                dtype=torch.uint8,
                device=device,
            )
        else:
            dummy_input = torch.zeros(
                (1, 3, input_height, input_width),
                dtype=torch.float32,
                device=device,
            )

        if export_format == "onnx":
            export_path = export_folder / (str(hydra_model_name) + ".onnx")
            export_onnx(
                wrapper=wrapper,
                dummy_input=dummy_input,
                export_path=export_path,
                output_names=hydra_model.deployment_output_names,
                opset=opset,
                with_nv12=with_nv12_layer,
                static_shapes=(hydra_model_name.family() == ModelFamily.DFINE),
            )
            if hasattr(hydra_model, "dfine_multitask"):
                manifest = hydra_model.dfine_multitask.checkpoint_payload()[
                    "manifest"
                ]
                preprocessing = manifest["preprocessing"]
                preprocessing["input_size"] = [input_height, input_width]
                if with_nv12_layer:
                    preprocessing.update(
                        {
                            "input_dtype": "uint8",
                            "input_shape": [
                                input_height // 2,
                                input_width // 2,
                                6,
                            ],
                            "input_layout": "packed_nv12_h2_w2_6",
                        }
                    )
                else:
                    preprocessing.update(
                        {
                            "input_dtype": "float32",
                            "input_shape": [1, 3, input_height, input_width],
                            "input_layout": "nchw_rgb",
                        }
                    )
                refresh_manifest_config_hash(manifest)
                with export_path.with_suffix(".manifest.yaml").open(
                    "w",
                    encoding="utf-8",
                ) as file:
                    yaml.safe_dump(manifest, file, sort_keys=False)
            click.echo(
                "Exported Hydra ONNX model to: "
                f"{os.path.abspath(export_folder)}"
            )
            continue

        export_torchscript(
            wrapper=wrapper,
            dummy_input=dummy_input,
            export_path=export_folder / (str(hydra_model_name) + ".pt"),
        )
        click.echo(
            "Exported Hydra TorchScript model to: "
            f"{os.path.abspath(export_folder)}"
        )


if __name__ == "__main__":
    main()
