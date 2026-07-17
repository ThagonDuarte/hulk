from __future__ import annotations

import json
import pickle
from dataclasses import asdict, dataclass
from itertools import pairwise
from pathlib import Path
from typing import Any, Literal

import click
import torch
from torch import nn
from ultralytics.models.yolo.model import YOLO
from ultralytics.utils.torch_utils import get_flops

from model.hydra import Hydra
from ultralytics_dfine.nn import DFINEDetectionModel, DFINEMultiTaskModel
from utils.export_hydra import (
    HydraWrapper,
    build_task_dict,
    export_torchscript,
    set_export_mode,
)
from utils.model_naming import HYDRA_MODEL_NAME_TYPE, HydraModelName

GIGA = 1_000_000_000
MEGA = 1_000_000

ImageSize = tuple[int, int]
CheckpointKind = Literal["dfine", "dfine-multitask", "yolo"]


@dataclass(frozen=True)
class ComplexityResult:
    path: str
    input_size: int | ImageSize
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


def normalize_image_size(imgsz: int | ImageSize) -> ImageSize:
    """Return an explicit ``(height, width)`` pair."""
    if isinstance(imgsz, int):
        return imgsz, imgsz
    return imgsz


def report_image_size(image_size: ImageSize) -> int | ImageSize:
    """Preserve the historical scalar schema for square reports."""
    height, width = image_size
    return height if height == width else image_size


def resolve_cli_image_size(
    imgsz: int,
    width: int | None,
    height: int | None,
) -> ImageSize:
    """Resolve rectangular overrides and validate every explicit dimension."""
    if imgsz <= 0:
        raise click.BadParameter("--imgsz must be > 0")  # noqa: TRY003
    input_width = imgsz if width is None else width
    input_height = imgsz if height is None else height
    if input_width <= 0:
        raise click.BadParameter("--width must be > 0")  # noqa: TRY003
    if input_height <= 0:
        raise click.BadParameter("--height must be > 0")  # noqa: TRY003
    return input_height, input_width


def checkpoint_kind(path: Path) -> CheckpointKind:
    """Select the loader from safe, explicit checkpoint metadata."""
    try:
        checkpoint = torch.load(
            path,
            map_location="cpu",
            weights_only=True,
        )
    except pickle.UnpicklingError:
        # Ultralytics YOLO checkpoints contain allowlisted module objects and
        # are intentionally loaded through YOLO's own trusted loader below.
        return "yolo"
    if not isinstance(checkpoint, dict):
        return "yolo"
    architecture = checkpoint.get("architecture")
    if architecture == "dfine-multitask":
        return "dfine-multitask"
    if architecture == "dfine":
        return "dfine"
    return "yolo"


def load_checkpoint_model(path: Path, device: str) -> nn.Module:
    """Load a supported checkpoint without relying on trial-and-error."""
    kind = checkpoint_kind(path)
    if kind == "dfine-multitask":
        model = DFINEMultiTaskModel.from_checkpoint(path)
    elif kind == "dfine":
        model = DFINEDetectionModel.from_checkpoint(path)
    else:
        model = YOLO(path).model
    return model.to(device).eval()


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


def resolve_asset_path(model_name: str, assets_dir: Path) -> Path:
    path = assets_dir / model_name
    if path.exists() or path.suffix:
        return path

    for suffix in (".pt", ".yaml"):
        suffixed_path = path.with_suffix(suffix)
        if suffixed_path.exists():
            return suffixed_path

    return path


def resolve_hydra_backbone_path(
    hydra_model_name: HydraModelName,
    assets_dir: Path,
    task_paths: list[Path],
) -> Path:
    asset_path = resolve_asset_path(hydra_model_name.backbone.name, assets_dir)
    if asset_path.exists():
        return asset_path

    for task_path in task_paths:
        if task_path.exists():
            return task_path

    checked_paths = ", ".join(str(path) for path in [asset_path, *task_paths])
    raise FileNotFoundError(  # noqa: TRY003
        f"No local backbone source found for {hydra_model_name}. "
        f"Checked: {checked_paths}"
    )


def profile_checkpoint(
    path: Path,
    *,
    imgsz: int | ImageSize,
    device: str,
    report_path: Path | None = None,
) -> ComplexityResult:
    file_size_bytes, file_size_mb = model_file_size(path)
    image_size = normalize_image_size(imgsz)

    with torch.inference_mode():
        model = load_checkpoint_model(path, device)

        # Ultralytics reports FLOPs as two floating point ops per MAC.
        gflops = float(get_flops(model, imgsz=list(image_size)))
        flops = round(gflops * GIGA)
        macs = flops // 2

    result = ComplexityResult(
        path=display_path(path),
        input_size=report_image_size(image_size),
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
    imgsz: int | ImageSize,
    device: str,
    runs_dir: Path,
    assets_dir: Path,
    train_folder_path: Path,
    val_folder_path: Path,
) -> ComplexityResult:
    image_size = normalize_image_size(imgsz)
    task_dict = build_task_dict(
        hydra_model_name=hydra_model_name,
        train_folder_path=train_folder_path,
        val_folder_path=val_folder_path,
    )
    task_paths = list(task_dict.values())
    backbone_path = resolve_hydra_backbone_path(
        hydra_model_name,
        assets_dir,
        task_paths,
    )
    output_dir = hydra_output_dir(runs_dir, hydra_model_name)
    output_dir.mkdir(parents=True, exist_ok=True)
    export_path = hydra_export_path(runs_dir, hydra_model_name)
    report_path = hydra_report_path(runs_dir, hydra_model_name)

    with torch.inference_mode():
        hydra_model = Hydra(
            backbone_path=str(backbone_path),
            task_dict=task_dict,
            number_of_frozen_modules=(
                hydra_model_name.number_of_frozen_modules
            ),
            family=hydra_model_name.family(),
        ).to(device)
        hydra_model.eval()
        set_export_mode(hydra_model)

        model = HydraWrapper(hydra_model, task_dict=task_dict).to(device)
        model.eval()

        # Ultralytics reports FLOPs as two floating point ops per MAC.
        gflops = float(get_flops(model, imgsz=list(image_size)))
        flops = round(gflops * GIGA)
        macs = flops // 2

        dummy_input = torch.zeros(
            (1, 3, *image_size),
            dtype=torch.float32,
            device=device,
        )
        export_torchscript(model, dummy_input, export_path)

    file_size_bytes, file_size_mb = model_file_size(export_path)
    result = ComplexityResult(
        path=str(hydra_model_name),
        input_size=report_image_size(image_size),
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
    imgsz: int | ImageSize,
    exc: Exception,
    report_path: Path | None = None,
) -> ComplexityResult:
    file_size_bytes, file_size_mb = model_file_size(path)
    return ComplexityResult(
        path=display_path(path),
        input_size=report_image_size(normalize_image_size(imgsz)),
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
    imgsz: int | ImageSize,
    runs_dir: Path,
    exc: Exception,
) -> ComplexityResult:
    report_path = hydra_report_path(runs_dir, hydra_model_name)
    export_path = hydra_export_path(runs_dir, hydra_model_name)
    return ComplexityResult(
        path=str(hydra_model_name),
        input_size=report_image_size(normalize_image_size(imgsz)),
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
    "--width",
    type=int,
    help="Input width. Defaults to --imgsz.",
)
@click.option(
    "--height",
    type=int,
    help="Input height. Defaults to --imgsz.",
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
        "Example: yolo26m=f11+yolo26m+yolo26m-pose. May repeat."
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
    width: int | None,
    height: int | None,
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
    image_size = resolve_cli_image_size(imgsz, width, height)

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
                    imgsz=image_size,
                    device=device,
                    report_path=report_path,
                )
            )
        except Exception as exc:
            if strict:
                raise click.ClickException(str(exc)) from exc
            result = error_result(weight_path, image_size, exc, report_path)
            write_report(report_path, result)
            results.append(result)

    train_folder_path = runs_dir / train_dir
    val_folder_path = runs_dir / val_dir
    for hydra_model_name in hydra_model_names:
        try:
            results.append(
                profile_hydra_model(
                    hydra_model_name,
                    imgsz=image_size,
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
                image_size,
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
