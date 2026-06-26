from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from itertools import pairwise
from pathlib import Path
from typing import Any, cast

import click
import torch
from torch import nn
from ultralytics.models.yolo.model import YOLO
from ultralytics.nn.tasks import DetectionModel
from ultralytics.utils.torch_utils import get_flops

from model.hydra import get_backbone, get_backbone_length, get_head
from utils.export_hydra import (
    export_torchscript,
    set_export_mode,
)
from utils.model_naming import (
    HYDRA_MODEL_NAME_TYPE,
    HydraModelName,
    TaskType,
    resolve_model_path,
)

GIGA = 1_000_000_000
MEGA = 1_000_000
ResolvedModelPath = Path | str


@dataclass(frozen=True)
class ComplexityResult:
    path: str
    input_size: int
    file_size_bytes: int
    file_size_mb: float
    layers: int | None
    parameters: int | None
    macs: int | None
    gmacs: float | None
    flops: int | None
    gflops: float | None
    exported_model_path: str | None = None
    report_path: str | None = None
    error: str | None = None


@dataclass(frozen=True)
class HydraHeadProfileSpec:
    task_type: TaskType
    path: ResolvedModelPath


class HydraComplexityModel(nn.Module):
    def __init__(
        self,
        backbone_path: ResolvedModelPath,
        heads: list[HydraHeadProfileSpec],
        number_of_frozen_modules: int | None,
    ) -> None:
        super().__init__()

        backbone_yolo = YOLO(backbone_path)
        backbone_root = cast(DetectionModel, backbone_yolo.model)
        self.backbone_length = (
            number_of_frozen_modules
            if number_of_frozen_modules is not None
            else get_backbone_length(cast(dict, backbone_root.yaml))
        )
        self.shared_backbone = get_backbone(
            backbone_root,
            number_of_frozen_modules,
        )
        self.save_backbone = cast(list[int], backbone_root.save)

        self.heads = nn.ModuleList()
        self.branch_saves: list[list[int]] = []
        for head in heads:
            task_yolo = YOLO(head.path)
            task_root = cast(DetectionModel, task_yolo.model)
            self.heads.append(get_head(task_root, number_of_frozen_modules))
            self.branch_saves.append(cast(list[int], task_root.save))

    def forward(
        self, x: torch.Tensor
    ) -> torch.Tensor | tuple[torch.Tensor, ...]:
        y_backbone: list[torch.Tensor | None] = []
        backbone_activations: Any = x

        for index, module in enumerate(self.shared_backbone):
            from_index = cast(Any, module.f)
            if from_index != -1:
                backbone_activations = (
                    y_backbone[from_index]
                    if isinstance(from_index, int)
                    else [
                        backbone_activations if idx == -1 else y_backbone[idx]
                        for idx in cast(list[int], from_index)
                    ]
                )
            backbone_activations = module(backbone_activations)
            y_backbone.append(
                backbone_activations if index in self.save_backbone else None
            )

        outputs: list[torch.Tensor] = []
        for head_module, branch_save in zip(
            self.heads, self.branch_saves, strict=True
        ):
            head = cast(nn.ModuleList, head_module)
            y_head = list(y_backbone)
            head_activations: Any = backbone_activations

            for index, module in enumerate(head):
                module_index = index + self.backbone_length
                from_index = cast(Any, module.f)
                if from_index != -1:
                    head_activations = (
                        cast(torch.Tensor, y_head[from_index])
                        if isinstance(from_index, int)
                        else [
                            cast(torch.Tensor, head_activations)
                            if idx == -1
                            else cast(torch.Tensor, y_head[idx])
                            for idx in cast(list[int], from_index)
                        ]
                    )

                head_activations = module(head_activations)
                y_head.append(
                    cast(torch.Tensor, head_activations)
                    if module_index in branch_save
                    else None
                )

            if isinstance(head_activations, torch.Tensor):
                outputs.append(head_activations)
            elif isinstance(head_activations, tuple) and all(
                isinstance(output, torch.Tensor) for output in head_activations
            ):
                outputs.extend(head_activations)
            else:
                raise TypeError(  # noqa: TRY003
                    "Hydra head output must be a tensor or tuple of tensors, "
                    f"got {type(head_activations)}"
                )

        if len(outputs) == 1:
            return outputs[0]
        return tuple(outputs)


def display_path(path: Path) -> str:
    try:
        return str(path.relative_to(Path.cwd()))
    except ValueError:
        return str(path)


def is_generated_complexity_artifact(path: Path) -> bool:
    parts = path.parts
    return any(
        first == "runs" and second == "complexity"
        for first, second in pairwise(parts)
    )


def discover_weight_paths(
    paths: tuple[Path, ...],
    checkpoint_names: tuple[str, ...],
) -> list[Path]:
    checkpoint_name_set = set(checkpoint_names)
    weights: set[Path] = set()

    for path in paths:
        if is_generated_complexity_artifact(path):
            continue

        if path.is_file():
            if path.suffix == ".pt":
                weights.add(path)
            continue

        for weight_path in path.rglob("*.pt"):
            if is_generated_complexity_artifact(weight_path):
                continue
            if (
                checkpoint_name_set
                and weight_path.name not in checkpoint_name_set
            ):
                continue
            weights.add(weight_path)

    return sorted(weights)


def count_parameters(model: nn.Module) -> int:
    return sum(parameter.numel() for parameter in model.parameters())


def count_leaf_modules(model: nn.Module) -> int:
    return sum(1 for module in model.modules() if not list(module.children()))


def model_file_size(path: Path) -> tuple[int, float]:
    file_size_bytes = path.stat().st_size
    return file_size_bytes, file_size_bytes / MEGA


def hydra_output_dir(runs_dir: Path, hydra_model_name: HydraModelName) -> Path:
    return runs_dir / "complexity" / str(hydra_model_name)


def hydra_export_path(runs_dir: Path, hydra_model_name: HydraModelName) -> Path:
    return hydra_output_dir(runs_dir, hydra_model_name) / (
        f"{hydra_model_name}.pt"
    )


def hydra_report_path(runs_dir: Path, hydra_model_name: HydraModelName) -> Path:
    return hydra_output_dir(runs_dir, hydra_model_name) / "report.json"


def checkpoint_report_path(runs_dir: Path, path: Path) -> Path:
    return runs_dir / "complexity" / path.stem / "report.json"


def model_path_exists(path: ResolvedModelPath) -> bool:
    return Path(path).exists()


def resolve_asset_path(model_name: str, assets_dir: Path) -> ResolvedModelPath:
    return resolve_model_path(model_name, assets_dir)


def resolve_hydra_head_specs(
    hydra_model_name: HydraModelName,
    *,
    assets_dir: Path,
    train_folder_path: Path,
    val_folder_path: Path,
) -> list[HydraHeadProfileSpec]:
    head_specs: list[HydraHeadProfileSpec] = []

    for head in hydra_model_name.heads:
        task_type = head.task_type()
        integrated_model_name = hydra_model_name.integrated_model_name(head)
        if head.is_finetuned_model():
            path = train_folder_path / integrated_model_name / "weights/best.pt"
        else:
            path = (
                val_folder_path
                / integrated_model_name
                / f"{integrated_model_name}.pt"
            )

        if not model_path_exists(path):
            path = resolve_asset_path(head.name, assets_dir)
        head_specs.append(HydraHeadProfileSpec(task_type=task_type, path=path))

    return head_specs


def resolve_hydra_backbone_path(
    hydra_model_name: HydraModelName,
    assets_dir: Path,
    head_specs: list[HydraHeadProfileSpec],
) -> ResolvedModelPath:
    asset_path = resolve_asset_path(hydra_model_name.backbone.name, assets_dir)
    if model_path_exists(asset_path):
        return asset_path

    for head in head_specs:
        if model_path_exists(head.path):
            return head.path

    return asset_path


def profile_checkpoint(
    path: Path,
    *,
    imgsz: int,
    device: str,
    report_path: Path | None = None,
) -> ComplexityResult:
    file_size_bytes, file_size_mb = model_file_size(path)

    with torch.inference_mode():
        yolo_model = YOLO(path)
        model = yolo_model.model.to(device)
        model.eval()

        # Ultralytics reports FLOPs as two floating point ops per MAC.
        gflops = float(get_flops(model, imgsz=imgsz))
        flops = round(gflops * GIGA)
        macs = flops // 2

    result = ComplexityResult(
        path=display_path(path),
        input_size=imgsz,
        file_size_bytes=file_size_bytes,
        file_size_mb=file_size_mb,
        layers=count_leaf_modules(model),
        parameters=count_parameters(model),
        macs=macs,
        gmacs=macs / GIGA,
        flops=flops,
        gflops=gflops,
        report_path=(
            display_path(report_path) if report_path is not None else None
        ),
    )
    if report_path is not None:
        write_report(report_path, result)

    return result


def profile_hydra_model(
    hydra_model_name: HydraModelName,
    *,
    imgsz: int,
    device: str,
    runs_dir: Path,
    assets_dir: Path,
    train_folder_path: Path,
    val_folder_path: Path,
) -> ComplexityResult:
    head_specs = resolve_hydra_head_specs(
        hydra_model_name=hydra_model_name,
        assets_dir=assets_dir,
        train_folder_path=train_folder_path,
        val_folder_path=val_folder_path,
    )
    backbone_path = resolve_hydra_backbone_path(
        hydra_model_name,
        assets_dir,
        head_specs,
    )
    output_dir = hydra_output_dir(runs_dir, hydra_model_name)
    output_dir.mkdir(parents=True, exist_ok=True)
    export_path = hydra_export_path(runs_dir, hydra_model_name)
    report_path = hydra_report_path(runs_dir, hydra_model_name)

    with torch.inference_mode():
        model = HydraComplexityModel(
            backbone_path=backbone_path,
            heads=head_specs,
            number_of_frozen_modules=(
                hydra_model_name.number_of_frozen_modules
            ),
        ).to(device)
        model.eval()
        set_export_mode(model)

        # Ultralytics reports FLOPs as two floating point ops per MAC.
        gflops = float(get_flops(model, imgsz=imgsz))
        flops = round(gflops * GIGA)
        macs = flops // 2

        dummy_input = torch.zeros(
            (1, 3, imgsz, imgsz),
            dtype=torch.float32,
            device=device,
        )
        export_torchscript(model, dummy_input, export_path)

    file_size_bytes, file_size_mb = model_file_size(export_path)
    result = ComplexityResult(
        path=str(hydra_model_name),
        input_size=imgsz,
        file_size_bytes=file_size_bytes,
        file_size_mb=file_size_mb,
        layers=count_leaf_modules(model),
        parameters=count_parameters(model),
        macs=macs,
        gmacs=macs / GIGA,
        flops=flops,
        gflops=gflops,
        exported_model_path=display_path(export_path),
        report_path=display_path(report_path),
    )
    write_report(report_path, result)
    return result


def error_result(
    path: Path,
    imgsz: int,
    exc: Exception,
    report_path: Path | None = None,
) -> ComplexityResult:
    file_size_bytes, file_size_mb = model_file_size(path)
    return ComplexityResult(
        path=display_path(path),
        input_size=imgsz,
        file_size_bytes=file_size_bytes,
        file_size_mb=file_size_mb,
        layers=None,
        parameters=None,
        macs=None,
        gmacs=None,
        flops=None,
        gflops=None,
        report_path=(
            display_path(report_path) if report_path is not None else None
        ),
        error=f"{type(exc).__name__}: {exc}",
    )


def hydra_error_result(
    hydra_model_name: HydraModelName,
    imgsz: int,
    runs_dir: Path,
    exc: Exception,
) -> ComplexityResult:
    report_path = hydra_report_path(runs_dir, hydra_model_name)
    export_path = hydra_export_path(runs_dir, hydra_model_name)
    return ComplexityResult(
        path=str(hydra_model_name),
        input_size=imgsz,
        file_size_bytes=0,
        file_size_mb=0,
        layers=None,
        parameters=None,
        macs=None,
        gmacs=None,
        flops=None,
        gflops=None,
        exported_model_path=display_path(export_path),
        report_path=display_path(report_path),
        error=f"{type(exc).__name__}: {exc}",
    )


def format_int(value: int | None) -> str:
    if value is None:
        return "-"
    return f"{value:,}"


def format_float(value: float | None, digits: int = 3) -> str:
    if value is None:
        return "-"
    return f"{value:.{digits}f}"


def format_error(error: str | None) -> str:
    if error is None:
        return ""
    if len(error) <= 80:
        return error
    return f"{error[:77]}..."


def format_table(results: list[ComplexityResult]) -> str:
    if not results:
        return "No .pt checkpoints found."

    headers = (
        "model",
        "params",
        "GMACs",
        "GFLOPs",
        "size MB",
        "error",
    )
    rows = [
        (
            result.path,
            format_int(result.parameters),
            format_float(result.gmacs),
            format_float(result.gflops),
            format_float(result.file_size_mb, digits=1),
            format_error(result.error),
        )
        for result in results
    ]
    widths = [
        max(len(header), *(len(row[index]) for row in rows))
        for index, header in enumerate(headers)
    ]

    lines = [
        "  ".join(
            header.ljust(width)
            for header, width in zip(headers, widths, strict=True)
        ),
        "  ".join("-" * width for width in widths),
    ]
    lines.extend(
        "  ".join(
            value.ljust(width) for value, width in zip(row, widths, strict=True)
        )
        for row in rows
    )
    return "\n".join(lines)


def write_json(path: Path, results: list[ComplexityResult]) -> None:
    payload: list[dict[str, Any]] = [asdict(result) for result in results]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as output_file:
        json.dump(payload, output_file, indent=2)
        output_file.write("\n")


def write_report(path: Path, result: ComplexityResult) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as output_file:
        json.dump(asdict(result), output_file, indent=2)
        output_file.write("\n")


@click.command(
    context_settings={"help_option_names": ["-h", "--help"]},
    help=(
        "Report parameters, MACs, FLOPs, and size for YOLO checkpoints "
        "and Hydra model names. Hydra models are exported under "
        "runs/complexity/<model-name>/ before size is measured.\n\n"
        "FLOPs use the Ultralytics convention: 1 MAC = 2 FLOPs."
    ),
)
@click.argument(
    "paths",
    nargs=-1,
    type=click.Path(path_type=Path, exists=True),
)
@click.option(
    "--imgsz",
    default=640,
    type=int,
    show_default=True,
    help="Square input image size used for FLOPs estimation.",
)
@click.option(
    "--device",
    default="cpu",
    show_default=True,
    help="Torch device for loading and profiling, e.g. cpu or cuda:0.",
)
@click.option(
    "--checkpoint-name",
    multiple=True,
    help="Only include a checkpoint file name such as best.pt. May repeat.",
)
@click.option(
    "--hydra-model-name",
    "--hydra_model_name",
    "hydra_model_names",
    multiple=True,
    type=HYDRA_MODEL_NAME_TYPE,
    help=(
        "Hydra model name to assemble and profile. "
        "Example: yolo26s=f11+yolo26s+yolo26s-pose. May repeat."
    ),
)
@click.option(
    "--assets-dir",
    "--assets_dir",
    type=click.Path(path_type=Path),
    default=Path("assets"),
    show_default=True,
    help="Directory containing base model assets.",
)
@click.option(
    "--runs-dir",
    "--runs_dir",
    type=click.Path(path_type=Path),
    default=Path("runs"),
    show_default=True,
    help="Directory containing run outputs.",
)
@click.option(
    "--val-dir",
    "--val_dir",
    type=click.Path(path_type=Path),
    default=Path("val"),
    show_default=True,
    help="Validation run directory relative to --runs-dir.",
)
@click.option(
    "--train-dir",
    "--train_dir",
    type=click.Path(path_type=Path),
    default=Path("train"),
    show_default=True,
    help="Training run directory relative to --runs-dir.",
)
@click.option(
    "--json-output",
    type=click.Path(path_type=Path),
    help="Optional JSON output path for full machine-readable results.",
)
@click.option(
    "--strict",
    is_flag=True,
    default=False,
    help="Fail immediately when a checkpoint cannot be profiled.",
)
def main(
    paths: tuple[Path, ...],
    *,
    imgsz: int,
    device: str,
    checkpoint_name: tuple[str, ...],
    hydra_model_names: tuple[HydraModelName, ...],
    assets_dir: Path,
    runs_dir: Path,
    val_dir: Path,
    train_dir: Path,
    json_output: Path | None,
    strict: bool,
) -> None:
    if imgsz <= 0:
        raise click.BadParameter("--imgsz must be > 0")  # noqa: TRY003

    search_paths = paths
    if not search_paths and not hydra_model_names:
        search_paths = (runs_dir,)

    results: list[ComplexityResult] = []
    for weight_path in discover_weight_paths(search_paths, checkpoint_name):
        report_path = checkpoint_report_path(runs_dir, weight_path)
        try:
            results.append(
                profile_checkpoint(
                    weight_path,
                    imgsz=imgsz,
                    device=device,
                    report_path=report_path,
                )
            )
        except Exception as exc:
            if strict:
                raise click.ClickException(str(exc)) from exc
            result = error_result(weight_path, imgsz, exc, report_path)
            write_report(report_path, result)
            results.append(result)

    train_folder_path = runs_dir / train_dir
    val_folder_path = runs_dir / val_dir
    for hydra_model_name in hydra_model_names:
        try:
            results.append(
                profile_hydra_model(
                    hydra_model_name,
                    imgsz=imgsz,
                    device=device,
                    runs_dir=runs_dir,
                    assets_dir=assets_dir,
                    train_folder_path=train_folder_path,
                    val_folder_path=val_folder_path,
                )
            )
        except Exception as exc:
            if strict:
                raise click.ClickException(str(exc)) from exc
            result = hydra_error_result(
                hydra_model_name,
                imgsz,
                runs_dir,
                exc,
            )
            write_report(hydra_report_path(runs_dir, hydra_model_name), result)
            results.append(result)

    click.echo(format_table(results))

    if json_output is not None:
        write_json(json_output, results)
        click.echo(f"\nWrote JSON results to {json_output}")


if __name__ == "__main__":
    main()
