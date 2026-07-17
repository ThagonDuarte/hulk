"""Assert exact tensor invariants between multi-task checkpoints."""

from __future__ import annotations

import json
from collections.abc import Mapping, Sequence
from enum import StrEnum
from pathlib import Path
from typing import Any, cast

import click
import torch
from torch import Tensor

REPORT_FORMAT_VERSION = 1


class CheckpointBranch(StrEnum):
    """Tensor-bearing branches in a multi-task training checkpoint."""

    MODEL = "model"
    EMA = "ema"
    INFERENCE_MODEL = "inference_model"

    @property
    def report_name(self) -> str:
        """Return the unambiguous checkpoint path used in reports."""
        if self is CheckpointBranch.EMA:
            return "ema.module"
        return self.value


class ParentSource(StrEnum):
    """Policy used to select the parent state for each comparison."""

    CORRESPONDING = "corresponding"
    MODEL = CheckpointBranch.MODEL
    EMA = CheckpointBranch.EMA
    INFERENCE_MODEL = CheckpointBranch.INFERENCE_MODEL


CHILD_BRANCHES = (
    CheckpointBranch.MODEL,
    CheckpointBranch.EMA,
    CheckpointBranch.INFERENCE_MODEL,
)


def _as_mapping(value: object, *, description: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        msg = f"{description} must be a mapping"
        raise TypeError(msg)
    if not all(isinstance(key, str) for key in value):
        msg = f"{description} must have string keys"
        raise TypeError(msg)
    return cast("Mapping[str, object]", value)


def checkpoint_branch(
    checkpoint: Mapping[str, object],
    branch: CheckpointBranch,
) -> dict[str, Tensor]:
    """Extract and validate one checkpoint tensor state dictionary."""
    if branch is CheckpointBranch.EMA:
        ema = _as_mapping(
            checkpoint.get("ema"),
            description="checkpoint branch 'ema'",
        )
        raw_state = ema.get("module")
    else:
        raw_state = checkpoint.get(branch.value)

    state = _as_mapping(
        raw_state,
        description=f"checkpoint branch '{branch.report_name}'",
    )
    non_tensor_keys = sorted(
        key for key, value in state.items() if not isinstance(value, Tensor)
    )
    if non_tensor_keys:
        preview = ", ".join(non_tensor_keys[:5])
        suffix = " ..." if len(non_tensor_keys) > 5 else ""
        msg = (
            f"checkpoint branch '{branch.report_name}' contains non-tensor "
            f"values: {preview}{suffix}"
        )
        raise TypeError(msg)
    return cast("dict[str, Tensor]", dict(state))


def load_checkpoint_branches(
    path: Path,
    branches: Sequence[CheckpointBranch],
) -> dict[CheckpointBranch, dict[str, Tensor]]:
    """Load only the requested logical branches onto CPU."""
    raw_checkpoint = torch.load(path, map_location="cpu", weights_only=True)
    checkpoint = _as_mapping(raw_checkpoint, description="checkpoint")
    return {
        branch: checkpoint_branch(checkpoint, branch)
        for branch in dict.fromkeys(branches)
    }


def _is_allowed(name: str, allowed_prefixes: Sequence[str]) -> bool:
    return any(name.startswith(prefix) for prefix in allowed_prefixes)


def _value_mismatch(
    name: str,
    parent: Tensor,
    child: Tensor,
) -> dict[str, object]:
    unequal = torch.ne(parent, child)
    return {
        "name": name,
        "different_elements": int(torch.count_nonzero(unequal).item()),
        "total_elements": parent.numel(),
    }


def compare_state_dicts(
    parent: Mapping[str, Tensor],
    child: Mapping[str, Tensor],
    *,
    allowed_prefixes: Sequence[str] = (),
) -> dict[str, object]:
    """Compare all tensors outside the allowed prefixes with ``torch.equal``."""
    parent_names = set(parent)
    child_names = set(child)
    ignored_parent = sorted(
        name for name in parent_names if _is_allowed(name, allowed_prefixes)
    )
    ignored_child = sorted(
        name for name in child_names if _is_allowed(name, allowed_prefixes)
    )
    checked_parent = parent_names - set(ignored_parent)
    checked_child = child_names - set(ignored_child)
    missing = sorted(checked_parent - checked_child)
    unexpected = sorted(checked_child - checked_parent)

    dtype_mismatches: list[dict[str, str]] = []
    shape_mismatches: list[dict[str, object]] = []
    value_mismatches: list[dict[str, object]] = []
    exact_names: list[str] = []
    for name in sorted(checked_parent & checked_child):
        parent_tensor = parent[name]
        child_tensor = child[name]
        same_dtype = parent_tensor.dtype == child_tensor.dtype
        same_shape = parent_tensor.shape == child_tensor.shape
        if not same_dtype:
            dtype_mismatches.append(
                {
                    "name": name,
                    "parent": str(parent_tensor.dtype),
                    "child": str(child_tensor.dtype),
                }
            )
        if not same_shape:
            shape_mismatches.append(
                {
                    "name": name,
                    "parent": list(parent_tensor.shape),
                    "child": list(child_tensor.shape),
                }
            )
        if not same_dtype or not same_shape:
            continue
        if torch.equal(parent_tensor, child_tensor):
            exact_names.append(name)
        else:
            value_mismatches.append(
                _value_mismatch(name, parent_tensor, child_tensor)
            )

    violating_names = sorted(
        {
            *missing,
            *unexpected,
            *(entry["name"] for entry in dtype_mismatches),
            *(entry["name"] for entry in shape_mismatches),
            *(entry["name"] for entry in value_mismatches),
        }
    )
    return {
        "passed": not violating_names,
        "counts": {
            "parent_tensors": len(parent),
            "child_tensors": len(child),
            "ignored_parent_tensors": len(ignored_parent),
            "ignored_child_tensors": len(ignored_child),
            "shared_checked_tensors": len(checked_parent & checked_child),
            "exact_tensors": len(exact_names),
            "violating_tensor_names": len(violating_names),
        },
        "ignored_parent_tensors": ignored_parent,
        "ignored_child_tensors": ignored_child,
        "missing_tensors": missing,
        "unexpected_tensors": unexpected,
        "dtype_mismatches": dtype_mismatches,
        "shape_mismatches": shape_mismatches,
        "value_mismatches": value_mismatches,
        "violating_tensor_names": violating_names,
    }


def _parent_branch(
    source: ParentSource,
    child_branch: CheckpointBranch,
) -> CheckpointBranch:
    if source is ParentSource.CORRESPONDING:
        return child_branch
    return CheckpointBranch(source.value)


def compare_checkpoints(
    parent_path: Path,
    child_path: Path,
    *,
    parent_source: ParentSource = ParentSource.CORRESPONDING,
    allowed_prefixes: Sequence[str] = (),
) -> dict[str, object]:
    """Build a JSON-safe exact-invariance report for two checkpoints."""
    normalized_prefixes = tuple(dict.fromkeys(allowed_prefixes))
    if any(not prefix for prefix in normalized_prefixes):
        msg = "Allowed tensor prefixes must not be empty"
        raise ValueError(msg)

    parent_branches = tuple(
        _parent_branch(parent_source, child_branch)
        for child_branch in CHILD_BRANCHES
    )
    parent_states = load_checkpoint_branches(parent_path, parent_branches)
    child_states = load_checkpoint_branches(child_path, CHILD_BRANCHES)

    comparisons = []
    for child_branch in CHILD_BRANCHES:
        parent_branch = _parent_branch(parent_source, child_branch)
        comparison = compare_state_dicts(
            parent_states[parent_branch],
            child_states[child_branch],
            allowed_prefixes=normalized_prefixes,
        )
        comparison.update(
            {
                "name": child_branch.report_name,
                "parent_branch": parent_branch.report_name,
                "child_branch": child_branch.report_name,
            }
        )
        comparisons.append(comparison)

    return {
        "format_version": REPORT_FORMAT_VERSION,
        "criterion": "torch.equal",
        "passed": all(comparison["passed"] for comparison in comparisons),
        "parent_checkpoint": str(parent_path.absolute()),
        "child_checkpoint": str(child_path.absolute()),
        "parent_source": parent_source.value,
        "allowed_prefixes": list(normalized_prefixes),
        "comparisons": comparisons,
    }


def write_report(path: Path, report: Mapping[str, Any]) -> None:
    """Write an indented, deterministic JSON report."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as output:
        json.dump(report, output, indent=2, sort_keys=True)
        output.write("\n")


@click.command(context_settings={"help_option_names": ["-h", "--help"]})
@click.argument(
    "parent_checkpoint",
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
)
@click.argument(
    "child_checkpoint",
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
)
@click.option(
    "--parent-source",
    type=click.Choice([source.value for source in ParentSource]),
    default=ParentSource.CORRESPONDING.value,
    show_default=True,
    help=(
        "Parent branch used for each child branch. 'corresponding' checks "
        "matching branches; another value uses that one parent branch for "
        "all three child comparisons."
    ),
)
@click.option(
    "--allow-prefix",
    "allowed_prefixes",
    multiple=True,
    help=(
        "Tensor-name prefix allowed to change or be added/removed. "
        "May be repeated."
    ),
)
@click.option(
    "--output",
    type=click.Path(dir_okay=False, path_type=Path),
    help="Optional path that receives the same JSON emitted on stdout.",
)
def main(
    parent_checkpoint: Path,
    child_checkpoint: Path,
    *,
    parent_source: str,
    allowed_prefixes: tuple[str, ...],
    output: Path | None,
) -> None:
    """Compare parent and child raw, EMA, and inference tensors exactly."""
    try:
        report = compare_checkpoints(
            parent_checkpoint,
            child_checkpoint,
            parent_source=ParentSource(parent_source),
            allowed_prefixes=allowed_prefixes,
        )
    except (OSError, RuntimeError, TypeError, ValueError) as error:
        raise click.ClickException(str(error)) from error

    rendered = json.dumps(report, indent=2, sort_keys=True)
    click.echo(rendered)
    if output is not None:
        write_report(output, report)
    if not report["passed"]:
        raise click.exceptions.Exit(1)


if __name__ == "__main__":
    main()
