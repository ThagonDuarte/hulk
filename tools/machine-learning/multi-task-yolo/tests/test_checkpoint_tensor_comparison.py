"""CPU tests for exact multi-task checkpoint tensor comparisons."""

import json
import tempfile
import unittest
from pathlib import Path
from typing import Any

import torch
from click.testing import CliRunner

from utils.compare_checkpoint_tensors import (
    ParentSource,
    compare_checkpoints,
    compare_state_dicts,
    main,
)


def _checkpoint(
    model: dict[str, torch.Tensor],
    *,
    ema: dict[str, torch.Tensor] | None = None,
    inference_model: dict[str, torch.Tensor] | None = None,
) -> dict[str, Any]:
    return {
        "model": model,
        "ema": {"module": ema if ema is not None else model},
        "inference_model": (
            inference_model if inference_model is not None else model
        ),
    }


class CheckpointTensorComparisonTest(unittest.TestCase):
    def test_reports_every_mismatch_category(self) -> None:
        parent = {
            "exact": torch.tensor([1.0]),
            "missing": torch.tensor([2.0]),
            "dtype": torch.tensor([3.0]),
            "shape": torch.tensor([4.0, 5.0]),
            "value": torch.tensor([6.0, 7.0]),
        }
        child = {
            "exact": torch.tensor([1.0]),
            "unexpected": torch.tensor([2.0]),
            "dtype": torch.tensor([3], dtype=torch.int64),
            "shape": torch.tensor([[4.0, 5.0]]),
            "value": torch.tensor([6.0, 8.0]),
        }

        report = compare_state_dicts(parent, child)

        self.assertFalse(report["passed"])
        self.assertEqual(report["missing_tensors"], ["missing"])
        self.assertEqual(report["unexpected_tensors"], ["unexpected"])
        self.assertEqual(
            [entry["name"] for entry in report["dtype_mismatches"]],
            ["dtype"],
        )
        self.assertEqual(
            [entry["name"] for entry in report["shape_mismatches"]],
            ["shape"],
        )
        self.assertEqual(
            report["value_mismatches"],
            [
                {
                    "name": "value",
                    "different_elements": 1,
                    "total_elements": 2,
                }
            ],
        )

    def test_allowed_prefix_covers_values_and_topology(self) -> None:
        report = compare_state_dicts(
            {
                "frozen.weight": torch.tensor([1.0]),
                "field.removed": torch.tensor([2.0]),
                "field.changed": torch.tensor([3.0]),
            },
            {
                "frozen.weight": torch.tensor([1.0]),
                "field.added": torch.tensor([4.0]),
                "field.changed": torch.tensor([5.0]),
            },
            allowed_prefixes=("field.",),
        )

        self.assertTrue(report["passed"])
        self.assertEqual(
            report["ignored_parent_tensors"],
            ["field.changed", "field.removed"],
        )
        self.assertEqual(
            report["ignored_child_tensors"],
            ["field.added", "field.changed"],
        )

    def test_inference_model_can_explicitly_seed_all_child_branches(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            parent_path = root / "parent.pt"
            child_path = root / "child.pt"
            seed = {"frozen.weight": torch.tensor([7.0])}
            torch.save(
                _checkpoint(
                    {"frozen.weight": torch.tensor([1.0])},
                    ema={"frozen.weight": torch.tensor([2.0])},
                    inference_model=seed,
                ),
                parent_path,
            )
            torch.save(_checkpoint(seed), child_path)

            corresponding = compare_checkpoints(parent_path, child_path)
            inference = compare_checkpoints(
                parent_path,
                child_path,
                parent_source=ParentSource.INFERENCE_MODEL,
            )

        self.assertFalse(corresponding["passed"])
        self.assertTrue(inference["passed"])
        self.assertEqual(
            [item["parent_branch"] for item in inference["comparisons"]],
            ["inference_model"] * 3,
        )
        self.assertEqual(
            [item["child_branch"] for item in inference["comparisons"]],
            ["model", "ema.module", "inference_model"],
        )

    def test_cli_writes_json_and_fails_on_violation(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            parent_path = root / "parent.pt"
            child_path = root / "child.pt"
            output_path = root / "report.json"
            torch.save(
                _checkpoint({"frozen.weight": torch.tensor([1.0])}),
                parent_path,
            )
            torch.save(
                _checkpoint({"frozen.weight": torch.tensor([2.0])}),
                child_path,
            )

            result = CliRunner().invoke(
                main,
                [
                    str(parent_path),
                    str(child_path),
                    "--output",
                    str(output_path),
                ],
            )

            self.assertEqual(result.exit_code, 1, result.output)
            stdout_report = json.loads(result.output)
            file_report = json.loads(output_path.read_text(encoding="utf-8"))
            self.assertEqual(stdout_report, file_report)
            self.assertFalse(stdout_report["passed"])
            self.assertEqual(
                stdout_report["comparisons"][0]["value_mismatches"][0]["name"],
                "frozen.weight",
            )

    def test_cli_passes_with_repeated_allowed_prefixes(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            parent_path = root / "parent.pt"
            child_path = root / "child.pt"
            torch.save(
                _checkpoint(
                    {
                        "field.weight": torch.tensor([1.0]),
                        "pose.weight": torch.tensor([2.0]),
                    }
                ),
                parent_path,
            )
            torch.save(
                _checkpoint(
                    {
                        "field.weight": torch.tensor([3.0]),
                        "pose.weight": torch.tensor([4.0]),
                    }
                ),
                child_path,
            )

            result = CliRunner().invoke(
                main,
                [
                    str(parent_path),
                    str(child_path),
                    "--allow-prefix",
                    "field.",
                    "--allow-prefix",
                    "pose.",
                ],
            )

        self.assertEqual(result.exit_code, 0, result.output)
        report = json.loads(result.output)
        self.assertTrue(report["passed"])
        self.assertEqual(report["allowed_prefixes"], ["field.", "pose."])


if __name__ == "__main__":
    unittest.main()
