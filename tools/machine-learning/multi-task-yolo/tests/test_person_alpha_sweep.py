"""Focused CPU/static tests for the exact Person visibility-alpha sweep."""

# ruff: noqa: S607

from __future__ import annotations

import gzip
import importlib.util
import json
import subprocess
import tempfile
import unittest
from pathlib import Path
from types import ModuleType


def load_gate() -> ModuleType:
    path = (
        Path(__file__).parents[1]
        / "runs/reports/dfine-mt-object-field-7d-v1"
        / "person_alpha_sweep_gate.py"
    )
    spec = importlib.util.spec_from_file_location("person_alpha_gate", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class PersonAlphaSweepTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.gate = load_gate()
        cls.runner = (
            Path(__file__).parents[1]
            / "runs/reports/dfine-mt-object-field-7d-v1"
            / "run_person_alpha_sweep.sh"
        )

    def test_runner_is_exact_runtime_only_and_immutable(self) -> None:
        result = subprocess.run(
            ["bash", "-n", str(self.runner)],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        source = self.runner.read_text(encoding="utf-8")
        for value in (
            "PERSON_ALPHA_SWEEP_LOCKED=YES",
            "--backend pytorch-rgb",
            "--task person_pose",
            "--height 448 --width 544",
            "--batch-size 32 --workers 4",
            "--confidence 0.001",
            "--person-visibility-alpha 1.25",
            "--person-visibility-alpha 1.5",
            "--person-visibility-alpha 1.75",
            "--person-visibility-alpha 2",
            "--predictions",
            'require_absent "$VAL_OUTPUT"',
            'require_absent "$SELECTION_REPORT"',
        ):
            self.assertIn(value, source)
        for forbidden in (
            "torchrun",
            "--task object",
            "--task field_features",
            "--resume",
        ):
            self.assertNotIn(forbidden, source)

    def test_runner_rejects_unlocked_gpu_work(self) -> None:
        result = subprocess.run(
            ["bash", str(self.runner), "validate", "2"],
            check=False,
            capture_output=True,
            env={"PERSON_ALPHA_SWEEP_LOCKED": "NO"},
            text=True,
        )
        self.assertEqual(result.returncode, 3)
        self.assertIn("Refusing GPU work", result.stderr)

    def test_locked_identity_and_pose_contract_are_exact(self) -> None:
        self.assertEqual(
            self.gate.PARENT_SHA256,
            "3160e1ee8ef81dd416bf2d7480cabde5fc8a108053071845a8814f6ce02b57de",
        )
        self.assertEqual(self.gate.SOURCE_SHA256, self.gate.source_sha256())
        self.assertEqual(
            self.gate.EXPECTED_FINGERPRINT["digest"],
            "a1dfe01c0ed1ae09d50d5a1e47bde7a820c3e51213b84af07dbd4e4e1c6c9b18",
        )
        issues, contract = self.gate.pose_contract()
        self.assertEqual(issues, [])
        self.assertEqual(contract["effective_pose_max_detections"], 20)
        self.assertFalse(contract["cli_override"])

    def test_selection_requires_all_four_metrics_to_improve(self) -> None:
        metrics: dict[str, float] = {}
        baseline = {"map": 0.1, "map50": 0.2, "map75": 0.08, "mar": 0.3}
        for name, value in baseline.items():
            metrics[f"person/{name}"] = value
        rows = {
            1.0: baseline,
            1.25: {name: value + 0.01 for name, value in baseline.items()},
            1.5: {
                **{name: value + 0.02 for name, value in baseline.items()},
                "mar": baseline["mar"],
            },
            1.75: {name: value + 0.005 for name, value in baseline.items()},
            2.0: {name: value - 0.01 for name, value in baseline.items()},
        }
        for alpha, row in rows.items():
            for name, value in row.items():
                metrics[self.gate.metric_key(alpha, name)] = value

        issues, selection = self.gate.select_alpha(metrics)

        self.assertEqual(issues, [])
        self.assertEqual(selection["eligible_alphas"], [1.25, 1.75])
        self.assertEqual(selection["selected_alpha"], 1.25)
        self.assertEqual(selection["decision"], "promote_visibility_alpha")

    def test_no_eligible_candidate_keeps_alpha_one(self) -> None:
        metrics: dict[str, float] = {}
        for alpha in self.gate.ALPHAS:
            for name in self.gate.METRIC_NAMES:
                value = 0.5 if alpha == 1 else 0.49
                metrics[self.gate.metric_key(alpha, name)] = value
        for name in self.gate.METRIC_NAMES:
            metrics[f"person/{name}"] = 0.5

        issues, selection = self.gate.select_alpha(metrics)

        self.assertEqual(issues, [])
        self.assertEqual(selection["selected_alpha"], 1.0)
        self.assertEqual(selection["decision"], "keep_alpha_1")

    def test_predictions_require_exact_count_unique_person_records(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "predictions.jsonl.gz"
            with gzip.open(path, "wt", encoding="utf-8") as file:
                for index in range(2346):
                    record = {
                        "version": 1,
                        "task": "person_pose",
                        "image_key": f"person_pose:image-{index}",
                        "evaluation_size": [448, 544],
                        "predictions": {
                            "visibility_alpha": 1.0,
                            "boxes_xyxy": [[]] * 20,
                            "scores": [0.1] * 20,
                            "mean_visibility": [0.2] * 20,
                            "keypoints": [[]] * 20,
                        },
                    }
                    file.write(json.dumps(record) + "\n")
            issues, details = self.gate.inspect_predictions(path)
            self.assertEqual(issues, [])
            self.assertEqual(details["records"], 2346)
            self.assertEqual(details["maximum_predictions_per_image"], 20)

    def test_reports_never_overwrite(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "report.json"
            self.gate.write_report(path, {"status": "pass"})
            with self.assertRaises(FileExistsError):
                self.gate.write_report(path, {"status": "pass"})


if __name__ == "__main__":
    unittest.main()
