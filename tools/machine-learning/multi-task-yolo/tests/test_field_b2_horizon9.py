"""Focused CPU/static tests for the fresh B2-derived Field 9k probe."""

# ruff: noqa: S607

from __future__ import annotations

import importlib.util
import json
import subprocess
import tempfile
import unittest
from pathlib import Path
from types import ModuleType

import torch


def load_gate() -> ModuleType:
    path = (
        Path(__file__).parents[1]
        / "runs/reports/dfine-mt-object-field-7d-v1"
        / "field_b2_horizon9_gate.py"
    )
    spec = importlib.util.spec_from_file_location("field_b2_h9_gate", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FieldB2Horizon9Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.gate = load_gate()
        cls.runner = (
            Path(__file__).parents[1]
            / "runs/reports/dfine-mt-object-field-7d-v1"
            / "run_field_b2_horizon9.sh"
        )

    def test_runner_is_fresh_fixed_horizon_and_fail_closed(self) -> None:
        result = subprocess.run(
            ["bash", "-n", str(self.runner)],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        source = self.runner.read_text(encoding="utf-8")
        for value in (
            "b2-freeze-all-bn-s17/best_object.pt",
            "--epochs 9 --steps-per-epoch 1000 --validation-interval 9",
            "--trainable-profile field_head_only",
            "--field-head-learning-rate 0.0005",
            "--field-head-variant spatial_refine",
            "--field-augmentation-profile basic",
            "--cross-pose-visibility-negative-weight 0",
            "--cross-pose-detector-negative-weight 0",
            "--object-weight 0 --person-weight 0 --robot-weight 0",
            "FIELD_B2_HORIZON9_LOCKED=YES",
            "comparison-archive-order-9k-vs-7k-2000.json",
            "--iterations 2000 --seed 20260716",
        ):
            self.assertIn(value, source)
        self.assertNotIn("--resume", source)
        self.assertIn('require_absent "$TRAIN_OUTPUT"', source)
        self.assertIn('require_absent "$VAL_ROOT"', source)

    def test_runner_uses_counterbalanced_same_device_pairs(self) -> None:
        source = self.runner.read_text(encoding="utf-8")
        for invocation in (
            'run_validation_pair "0" "canonical" "gpu0-baseline-first"',
            'run_validation_pair "1" "canonical" "gpu1-candidate-first"',
            'run_validation_pair "2" "unique" "gpu2-baseline-first"',
            'run_validation_pair "3" "unique" "gpu3-candidate-first"',
            '"baseline_then_candidate" "$CANONICAL_OBJECT"',
            '"candidate_then_baseline" "$CANONICAL_OBJECT"',
            '"baseline_then_candidate" "$UNIQUE_OBJECT"',
            '"candidate_then_baseline" "$UNIQUE_OBJECT"',
            'if [[ "$gpu_list" != "0,1,2,3" ]]',
            'uv run "$GATE" pairing --output "$VAL_ROOT/pairing.json"',
        ):
            self.assertIn(invocation, source)
        self.assertEqual(source.count('run_comparison "'), 4)
        self.assertNotIn("$VAL_ROOT/canonical/baseline-7k", source)

    def test_pairing_manifest_is_exact_and_counterbalanced(self) -> None:
        manifest = self.gate.pairing_manifest()
        self.assertEqual(manifest["status"], "pass")
        self.assertEqual(len(manifest["repeats"]), 4)
        self.assertEqual(
            [row["gpu"] for row in manifest["repeats"]],
            [0, 1, 2, 3],
        )
        self.assertEqual(
            [row["execution_order"] for row in manifest["repeats"]],
            [
                "baseline_then_candidate",
                "candidate_then_baseline",
                "baseline_then_candidate",
                "candidate_then_baseline",
            ],
        )
        self.assertEqual(
            manifest["comparison"]["launch_mode"],
            "four_comparisons_in_parallel",
        )
        self.assertEqual(
            manifest["validation_execution"],
            {
                "same_device_within_pair": True,
                "sequential_within_pair": True,
                "four_pairs_in_parallel": True,
                "environment": {
                    "CUBLAS_WORKSPACE_CONFIG": ":4096:8",
                    "PYTHONHASHSEED": "17",
                },
            },
        )

    def test_runner_rejects_noncanonical_validation_gpu_mapping(self) -> None:
        result = subprocess.run(
            ["bash", str(self.runner), "validate", "3,2,1,0"],
            check=False,
            capture_output=True,
            env={"FIELD_B2_HORIZON9_LOCKED": "YES"},
            text=True,
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn("exact GPU mapping 0,1,2,3", result.stderr)

    def test_archived_step7k_reproduction_is_exact(self) -> None:
        issues, details = self.gate.inspect_step7k_reproduction()
        self.assertEqual(issues, [])
        self.assertEqual(
            details["sha256"],
            "6c5b8bd396df396de619f5f980b559863cbe67d39d696661ec3de6abd0596d3c",
        )
        self.assertEqual(details["observed_at_global_step"], 7000)
        self.assertEqual(
            {
                name: row["exact_tensors"]
                for name, row in details["branches"].items()
            },
            {"model": 852, "ema.module": 852, "inference_model": 852},
        )

    def test_runner_rejects_unlocked_gpu_work_before_preflight(self) -> None:
        result = subprocess.run(
            ["bash", str(self.runner), "train", "0,1"],
            check=False,
            capture_output=True,
            env={"FIELD_B2_HORIZON9_LOCKED": "NO"},
            text=True,
        )
        self.assertEqual(result.returncode, 3)
        self.assertIn("Refusing GPU work", result.stderr)

    def test_locked_identity_and_recipe_are_exact(self) -> None:
        self.assertEqual(
            self.gate.B2_PARENT_SHA256,
            "cf498c1c04a213866f7ef0b2bfb13d278609cf887b5bce10c5b7265e4ceca889",
        )
        self.assertEqual(
            self.gate.BASELINE_7K_SHA256,
            "3160e1ee8ef81dd416bf2d7480cabde5fc8a108053071845a8814f6ce02b57de",
        )
        self.assertEqual(
            self.gate.SOURCE_SHA256,
            self.gate.source_sha256(),
        )
        config = self.gate.expected_training_config()
        self.assertEqual(config["epochs"], 9)
        self.assertEqual(config["validation_interval"], 9)
        self.assertEqual(config["sampling_weights"], {"field_features": 1.0})
        self.assertEqual(config["trainable_roles"], ("field_head",))
        self.assertEqual(config["strict_deterministic"], True)
        self.assertEqual(config["ddp_find_unused_parameters"], False)

    def test_non_field_integrity_is_exact_across_all_branches(self) -> None:
        non_field = {
            f"detector.tensor.{index}": torch.tensor(index)
            for index in range(self.gate.EXPECTED_NON_FIELD_TENSORS)
        }
        parent_field = {
            f"field_feature_head.tensor.{index}": torch.tensor(0)
            for index in range(self.gate.EXPECTED_PARENT_FIELD_TENSORS)
        }
        child_field = {
            f"field_feature_head.tensor.{index}": torch.tensor(1)
            for index in range(self.gate.EXPECTED_CHILD_FIELD_TENSORS)
        }
        parent_state = {**non_field, **parent_field}
        child_state = {**non_field, **child_field}
        parent = {
            branch: dict(parent_state)
            for branch in ("model", "ema", "inference_model")
        }
        child = {
            branch: dict(child_state)
            for branch in ("model", "ema", "inference_model")
        }

        issues, details = self.gate.non_field_integrity(parent, child)

        self.assertEqual(issues, [])
        self.assertEqual(
            details["inference_model"]["checked_non_field_tensors"],
            796,
        )
        child["inference_model"]["detector.tensor.0"] = torch.tensor(-1)
        issues, _ = self.gate.non_field_integrity(parent, child)
        self.assertIn(
            "inference_model: non-Field tensor values",
            issues,
        )

    def test_field_regression_and_identity_drift_are_rejected(self) -> None:
        baseline = {
            "object/map": 0.5,
            "object/map50": 0.7,
            "object/map75": 0.4,
            "object/mar300": 0.6,
            "object/max_score_mean": 0.9,
            "field/map": 0.7,
            "field/map50": 0.8,
            "field/map75": 0.7,
            "field/mar": 0.8,
            "field/strict_map": 0.2,
            "field/strict_map50": 0.3,
            "field/localization/pck_5px": 0.7,
            "field/localization/pck_10px": 0.8,
            "field/localization/pck_20px": 0.9,
            "field/localization/matched_fraction": 0.85,
            "field/localization/median_error_px": 1.5,
        }
        for class_name in self.gate.EXPECTED_NAMES[:-1]:
            baseline[f"object/class/{class_name}/map"] = 0.4
        for class_name in (
            "GoalPost",
            "LSpot",
            "PenaltySpot",
            "TSpot",
            "XSpot",
        ):
            baseline[f"field/class/{class_name}/map"] = 0.5
            baseline[f"field/class/{class_name}/map50"] = 0.6
            baseline[f"field/class/{class_name}/strict_map"] = 0.1
        candidate = dict(baseline)

        def report(fingerprints: dict[str, str]) -> dict[str, object]:
            return {
                "bootstrap": {"iterations": 2000, "seed": 20260716},
                "compatibility": {
                    "compatible": True,
                    "dataset_fingerprints": fingerprints,
                },
                "metrics": [
                    {
                        "metric": name,
                        "baseline": value,
                        "candidate": candidate[name],
                    }
                    for name, value in baseline.items()
                ],
            }

        identities = {
            task: value["digest"]
            for task, value in self.gate.VALIDATION_IDENTITIES[
                "canonical"
            ].items()
        }
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / "comparison.json"
            path.write_text(json.dumps(report(identities)), encoding="utf-8")
            issues, _ = self.gate.inspect_comparison(
                path,
                tier="canonical",
                baseline=baseline,
                candidate=candidate,
            )
            self.assertEqual(issues, [])

            candidate["object/max_score_mean"] = 0.9000000000000001
            path.write_text(json.dumps(report(identities)), encoding="utf-8")
            issues, details = self.gate.inspect_comparison(
                path,
                tier="canonical",
                baseline=baseline,
                candidate=candidate,
            )
            self.assertIn(
                "same-device Object metrics are not exactly equal: "
                "object/max_score_mean",
                issues,
            )
            self.assertEqual(details["exact_object_metric_count"], 12)
            candidate["object/max_score_mean"] = 0.9

            candidate["field/class/TSpot/map"] = 0.49
            path.write_text(json.dumps(report(identities)), encoding="utf-8")
            issues, _ = self.gate.inspect_comparison(
                path,
                tier="canonical",
                baseline=baseline,
                candidate=candidate,
            )
            self.assertTrue(
                any(
                    "field/class/TSpot/map violates" in issue
                    for issue in issues
                )
            )

            candidate["field/class/TSpot/map"] = 0.5
            drifted = dict(identities)
            drifted["field_features"] = "f" * 64
            path.write_text(json.dumps(report(drifted)), encoding="utf-8")
            issues, _ = self.gate.inspect_comparison(
                path,
                tier="canonical",
                baseline=baseline,
                candidate=candidate,
            )
            self.assertIn("comparison compatibility mismatch", issues)

    def test_repeat_variance_reports_device_and_order_spread(self) -> None:
        repeats = {
            "gpu0-baseline-first": {
                "baseline": {"object/map": 0.5, "field/map": 0.7},
                "candidate": {"object/map": 0.5, "field/map": 0.72},
            },
            "gpu1-candidate-first": {
                "baseline": {"object/map": 0.51, "field/map": 0.71},
                "candidate": {"object/map": 0.51, "field/map": 0.735},
            },
        }
        report = self.gate.repeat_variance("canonical", repeats)
        object_row = report["metrics"]["object/map"]
        field_row = report["metrics"]["field/map"]
        self.assertAlmostEqual(
            object_row["baseline_between_device_abs_diff"],
            0.01,
        )
        self.assertAlmostEqual(
            object_row["effect_between_repeat_device_order_abs_diff"],
            0.0,
        )
        self.assertAlmostEqual(
            field_row["effect_between_repeat_device_order_abs_diff"],
            0.005,
        )

    def test_class_gates_do_not_require_unavailable_bootstrap_rows(
        self,
    ) -> None:
        object_names, field_higher, field_lower = (
            self.gate.protected_metric_names()
        )
        baseline = dict.fromkeys(
            [*object_names, *field_higher, *field_lower],
            0.5,
        )
        candidate = dict(baseline)
        raw_only = {
            name
            for name in baseline
            if name in {"object/mar300", "field/mar"}
            or name.startswith(("object/class/", "field/class/"))
        }
        report = {
            "bootstrap": {"iterations": 2_000, "seed": 20_260_716},
            "compatibility": {
                "compatible": True,
                "dataset_fingerprints": {
                    task: value["digest"]
                    for task, value in self.gate.VALIDATION_IDENTITIES[
                        "canonical"
                    ].items()
                },
            },
            "metrics": [
                {
                    "metric": name,
                    "baseline": value,
                    "candidate": candidate[name],
                }
                for name, value in baseline.items()
                if name not in raw_only
            ],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "comparison.json"
            path.write_text(json.dumps(report), encoding="utf-8")
            issues, details = self.gate.inspect_comparison(
                path,
                tier="canonical",
                baseline=baseline,
                candidate=candidate,
            )
            self.assertEqual(issues, [])
            self.assertIn(
                "field/class/XSpot/strict_map",
                details["raw_only_protected_metrics"],
            )

            candidate["field/class/XSpot/strict_map"] = 0.49
            issues, _ = self.gate.inspect_comparison(
                path,
                tier="canonical",
                baseline=baseline,
                candidate=candidate,
            )
            self.assertTrue(
                any(
                    "field/class/XSpot/strict_map violates" in issue
                    for issue in issues
                )
            )

    def test_reports_never_overwrite(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "report.json"
            self.gate.write_report(path, {"status": "pass"})
            with self.assertRaises(FileExistsError):
                self.gate.write_report(path, {"status": "pass"})


if __name__ == "__main__":
    unittest.main()
