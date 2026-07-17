"""CPU/static tests for the deterministic fresh-B2 Field horizon-8 run."""

# ruff: noqa: S607

from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import tempfile
import unittest
from contextlib import ExitStack
from pathlib import Path
from types import ModuleType
from unittest import mock

import torch


def load_gate() -> ModuleType:
    path = (
        Path(__file__).parents[1]
        / "runs/reports/dfine-mt-object-field-7d-v1"
        / "field_b2_horizon8_gate.py"
    )
    spec = importlib.util.spec_from_file_location("field_b2_h8_gate", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FieldB2Horizon8Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.gate = load_gate()
        cls.runner = (
            Path(__file__).parents[1]
            / "runs/reports/dfine-mt-object-field-7d-v1"
            / "run_field_b2_horizon8.sh"
        )

    def test_runner_is_fresh_exact_horizon8_and_fail_closed(self) -> None:
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
            "--epochs 8 --steps-per-epoch 1000 --validation-interval 8",
            "--trainable-profile field_head_only",
            "--field-head-learning-rate 0.0005",
            "--field-head-variant spatial_refine",
            "--field-augmentation-profile basic",
            "--cross-pose-visibility-negative-weight 0",
            "--cross-pose-detector-negative-weight 0",
            "--object-weight 0 --person-weight 0 --robot-weight 0",
            "FIELD_B2_HORIZON8_LOCKED=YES",
            "watch-step7k",
            "step7k-exact-reproduction.json",
            "comparison-archive-order-8k-vs-7k-2000.json",
            "--iterations 2000 --seed 20260716",
        ):
            self.assertIn(value, source)
        self.assertNotIn("--resume", source)
        self.assertLess(source.index("watch-step7k"), source.index("torchrun"))
        self.assertIn('require_absent "$TRAIN_OUTPUT"', source)
        self.assertIn('require_absent "$VAL_ROOT"', source)

    def test_sealed_preflight_is_verified_before_gpu_launch(self) -> None:
        source = self.runner.read_text(encoding="utf-8")
        train_body = source.split("run_train() {", maxsplit=1)[1]
        train_body = train_body.split("\n}", maxsplit=1)[0]
        self.assertIn("verify_preflight >/dev/null", train_body)
        self.assertNotIn("run_preflight >/dev/null", train_body)
        self.assertLess(
            train_body.index("verify_preflight >/dev/null"),
            train_body.index('require_absent "$TRAIN_OUTPUT"'),
        )

    def test_sealed_preflight_accepts_exact_and_rejects_drift(self) -> None:
        expected = {
            "status": "pass",
            "source_sha256": "a" * 64,
            "issues": [],
        }
        rendered = json.dumps(expected, indent=2, sort_keys=True) + "\n"
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "preflight.json"
            path.write_text(rendered, encoding="utf-8")
            with (
                mock.patch.object(self.gate, "PREFLIGHT_REPORT", path),
                mock.patch.object(
                    self.gate,
                    "SEALED_PREFLIGHT_SHA256",
                    self.gate.sha256(path),
                ),
                mock.patch.object(self.gate, "relative", side_effect=str),
                mock.patch.object(
                    self.gate,
                    "preflight_report",
                    return_value=expected,
                ) as recompute,
            ):
                exact = self.gate.verify_sealed_preflight()
                recompute.assert_called_once_with(
                    check_outputs=True,
                    allow_existing_preflight=True,
                )
                self.assertEqual(exact["status"], "pass")
                self.assertTrue(exact["byte_exact"])
                self.assertTrue(exact["json_exact"])

                path.write_text(rendered + " ", encoding="utf-8")
                altered_report = self.gate.verify_sealed_preflight()
                self.assertEqual(altered_report["status"], "fail")
                self.assertIn(
                    "sealed preflight SHA-256 mismatch",
                    altered_report["issues"],
                )

                path.write_text(rendered, encoding="utf-8")
                recompute.return_value = {
                    **expected,
                    "status": "fail",
                    "issues": ["current source hash mismatch"],
                }
                altered_input = self.gate.verify_sealed_preflight()
                self.assertEqual(altered_input["status"], "fail")
                self.assertIn(
                    "recomputed preflight does not pass",
                    altered_input["issues"],
                )

    def test_live_sealed_preflight_command_passes_exactly(self) -> None:
        root = Path(__file__).parents[1]
        with tempfile.TemporaryDirectory() as runtime:
            result = subprocess.run(
                [
                    "uv",
                    "run",
                    str(Path(self.gate.__file__)),
                    "verify-preflight",
                ],
                cwd=root,
                check=False,
                capture_output=True,
                env={
                    **os.environ,
                    "MPLCONFIGDIR": str(Path(runtime) / "matplotlib"),
                    "UV_CACHE_DIR": str(Path(runtime) / "uv-cache"),
                },
                text=True,
                timeout=120,
            )
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads(result.stdout)
        self.assertEqual(report["status"], "pass")
        self.assertEqual(report["issues"], [])
        self.assertTrue(report["byte_exact"])
        self.assertTrue(report["json_exact"])
        self.assertEqual(
            report["sealed_preflight"]["sha256"],
            self.gate.SEALED_PREFLIGHT_SHA256,
        )

    def test_runner_uses_counterbalanced_same_device_pairs(self) -> None:
        source = self.runner.read_text(encoding="utf-8")
        for value in (
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
            self.assertIn(value, source)
        self.assertEqual(source.count('run_comparison "'), 4)

    def test_gpu_actions_reject_missing_lock_and_wrong_mapping(self) -> None:
        train = subprocess.run(
            ["bash", str(self.runner), "train", "0,1"],
            check=False,
            capture_output=True,
            env={"FIELD_B2_HORIZON8_LOCKED": "NO"},
            text=True,
        )
        self.assertEqual(train.returncode, 3)
        self.assertIn("Refusing GPU work", train.stderr)
        validation = subprocess.run(
            ["bash", str(self.runner), "validate", "3,2,1,0"],
            check=False,
            capture_output=True,
            env={"FIELD_B2_HORIZON8_LOCKED": "YES"},
            text=True,
        )
        self.assertEqual(validation.returncode, 2)
        self.assertIn("exact GPU mapping 0,1,2,3", validation.stderr)

    def test_pairing_manifest_and_bootstrap_contract_are_exact(self) -> None:
        manifest = self.gate.pairing_manifest()
        self.assertEqual(manifest["status"], "pass")
        self.assertEqual(len(manifest["repeats"]), 4)
        self.assertEqual(
            [value["gpu"] for value in manifest["repeats"]],
            [0, 1, 2, 3],
        )
        self.assertEqual(
            [value["execution_order"] for value in manifest["repeats"]],
            [
                "baseline_then_candidate",
                "candidate_then_baseline",
                "baseline_then_candidate",
                "candidate_then_baseline",
            ],
        )
        comparison = manifest["comparison"]
        self.assertEqual(comparison["iterations"], 2_000)
        self.assertEqual(comparison["seed"], 20_260_716)
        self.assertEqual(
            comparison["implementation_sha256"],
            self.gate.COMPARE_SHA256,
        )

    def test_training_recipe_changes_only_horizon_identity(self) -> None:
        expected = self.gate.expected_training_config()
        self.assertEqual(expected["epochs"], 8)
        self.assertEqual(expected["validation_interval"], 8)
        self.assertEqual(expected["sampling_weights"], {"field_features": 1.0})
        self.assertEqual(expected["trainable_roles"], ("field_head",))
        self.assertEqual(expected["strict_deterministic"], True)
        self.assertEqual(expected["sdpa_backend"], "math")
        self.assertEqual(expected["ddp_find_unused_parameters"], False)
        self.assertEqual(
            self.gate.BASE.source_sha256(),
            self.gate.BASE.SOURCE_SHA256,
        )

    def test_final_schema_is_explicit_for_downstream_locks(self) -> None:
        contract = self.gate.final_schema_contract()
        self.assertEqual(contract["version"], 1)
        self.assertEqual(
            contract["decision_values"],
            {"pass": "promote_8k", "fail": "keep_7k"},
        )
        self.assertEqual(
            contract["candidate_checkpoint"]["fixed_global_step"],
            8_000,
        )
        self.assertEqual(
            contract["rejected_9k_decision"]["required_decision"],
            "keep_7k",
        )
        self.assertEqual(
            contract["validation_evidence"]["tiers"],
            {
                "canonical": [
                    "gpu0-baseline-first",
                    "gpu1-candidate-first",
                ],
                "unique": [
                    "gpu2-baseline-first",
                    "gpu3-candidate-first",
                ],
            },
        )

    def test_all_four_9k_repeats_quantify_every_protected_delta(self) -> None:
        expected = {
            ("canonical", "gpu0-baseline-first"): {
                "field/class/XSpot/strict_map"
            },
            ("canonical", "gpu1-candidate-first"): {
                "field/class/XSpot/strict_map"
            },
            ("unique", "gpu2-baseline-first"): {
                "field/class/XSpot/map50",
                "field/class/XSpot/strict_map",
            },
            ("unique", "gpu3-candidate-first"): {
                "field/class/XSpot/map50",
                "field/class/XSpot/strict_map",
            },
        }
        expected_count = sum(
            len(group) for group in self.gate.BASE.protected_metric_names()
        )
        self.assertEqual(expected_count, 37)
        for (tier, repeat), expected_regressions in expected.items():
            pair = self.gate.HORIZON9_VAL / tier / repeat
            baseline = json.loads(
                (pair / "baseline-7k/metrics.json").read_text()
            )
            candidate = json.loads(
                (pair / "candidate-9k/metrics.json").read_text()
            )
            regressions, rows = self.gate.protected_delta_rows(
                baseline,
                candidate,
            )
            self.assertEqual(len(rows), expected_count)
            self.assertEqual(set(regressions), expected_regressions)
            object_rows = {
                name: row
                for name, row in rows.items()
                if name.startswith("object/")
            }
            self.assertTrue(object_rows)
            self.assertTrue(
                all(
                    row["passed"] and row["delta"] == 0
                    for row in object_rows.values()
                )
            )

    def test_class_gates_are_raw_exact_when_comparator_has_no_class_rows(
        self,
    ) -> None:
        object_names, field_higher, field_lower = (
            self.gate.BASE.protected_metric_names()
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
        rows = [
            {
                "metric": name,
                "baseline": value,
                "candidate": candidate[name],
                "delta": 0.0,
            }
            for name, value in baseline.items()
            if name not in raw_only
        ]
        fingerprints = {
            task: value["digest"]
            for task, value in self.gate.BASE.VALIDATION_IDENTITIES[
                "canonical"
            ].items()
        }
        report = {
            "bootstrap": {"iterations": 2_000, "seed": 20_260_716},
            "compatibility": {
                "compatible": True,
                "dataset_fingerprints": fingerprints,
            },
            "metrics": rows,
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

    def test_checkpoint_inventory_proves_step8_is_not_recoverable(self) -> None:
        issues, inventory = self.gate.horizon9_checkpoint_inventory()
        self.assertEqual(issues, [])
        self.assertEqual(inventory["checkpoint_file_count"], 6)
        self.assertEqual(inventory["validation_steps"], [9_000])
        self.assertEqual(inventory["step8_scalar_train_events"], 2)
        self.assertEqual(inventory["step8_checkpoint_paths"], [])
        self.assertFalse(inventory["recoverable_step8_checkpoint"])
        self.assertTrue(
            all(
                value["metadata"]["global_step"] == 9_000
                for value in inventory["checkpoint_files"].values()
            )
        )

    def test_rejected_9k_reference_requires_exact_failure(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "decision.json"
            report = {
                "status": "fail",
                "decision": "keep_7k",
                "candidate": {
                    "candidate": {
                        "sha256": self.gate.HORIZON9_CHECKPOINT_SHA256
                    }
                },
            }
            path.write_text(json.dumps(report), encoding="utf-8")
            with (
                mock.patch.object(self.gate, "HORIZON9_DECISION", path),
                mock.patch.object(
                    self.gate,
                    "HORIZON9_DECISION_SHA256",
                    self.gate.sha256(path),
                ),
                mock.patch.object(self.gate, "relative", side_effect=str),
            ):
                issues, details = self.gate.rejected_9k_decision()
            self.assertEqual(issues, [])
            self.assertEqual(details["status"], "fail")
            self.assertEqual(details["decision"], "keep_7k")

    def test_watcher_preserves_complete_step7_and_exact_reproduction(
        self,
    ) -> None:
        state = {
            f"tensor.{index}": torch.tensor(index)
            for index in range(self.gate.EXPECTED_STEP7K_TENSORS)
        }
        checkpoint = {
            "epoch": 6,
            "global_step": 7_000,
            "scheduler_step": 7_000,
            "pending_validation_epoch": None,
            "stage": "field_head_only",
            "world_size": 2,
            "model": dict(state),
            "ema": {"module": dict(state)},
            "inference_model": dict(state),
        }
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory) / "run"
            run.mkdir()
            source = run / "last.pt"
            destination = run / "snapshots/step-07000.pt"
            torch.save(checkpoint, source)
            with mock.patch.object(self.gate, "relative", side_effect=str):
                details = self.gate.preserve_step7k_snapshot(
                    run,
                    destination,
                    timeout_seconds=2,
                )
                with mock.patch.object(
                    self.gate.BASE,
                    "inspect_fixed_checkpoint",
                    return_value=(
                        checkpoint,
                        [],
                        {"path": "parent.pt", "sha256": "a" * 64},
                    ),
                ):
                    report = self.gate.exact_reproduction_report(destination)
            self.assertEqual(details["sha256"], self.gate.sha256(destination))
            self.assertEqual(report["status"], "pass")
            self.assertEqual(
                {
                    row["name"]: row["counts"]["exact_tensors"]
                    for row in report["comparisons"]
                },
                {
                    "model": 852,
                    "ema.module": 852,
                    "inference_model": 852,
                },
            )

    def test_final_report_selects_only_fixed8_on_complete_pass(self) -> None:
        object_names, field_higher, field_lower = (
            self.gate.BASE.protected_metric_names()
        )
        metrics = dict.fromkeys(
            [*object_names, *field_higher, *field_lower],
            0.5,
        )
        metrics["object/max_score_mean"] = 0.9
        candidate_identity = {
            "path": str(self.gate.TRAIN_OUTPUT / "last.pt"),
            "sha256": "c" * 64,
            "metadata": {"epoch": 7, "global_step": 8_000},
        }
        fixed = {
            "kind": "final-last-pt",
            "global_step": 8_000,
            "path": candidate_identity["path"],
            "sha256": candidate_identity["sha256"],
        }
        child = {
            "status": "pass",
            "candidate": candidate_identity,
            "fixed_checkpoint": fixed,
            "issues": [],
        }
        rejected = {
            "path": "rejected.json",
            "sha256": "9" * 64,
            "status": "fail",
            "decision": "keep_7k",
        }
        audit = {
            "path": "audit.json",
            "sha256": "8" * 64,
            "status": "pass",
            "decision": "keep_7k",
        }
        reproduction = {
            "path": "step7.json",
            "sha256": "7" * 64,
            "snapshot": {},
            "criterion": "torch.equal",
            "comparisons": [],
        }
        pairing = {"status": "pass", "repeats": []}
        with tempfile.TemporaryDirectory() as directory:
            validation_root = Path(directory)
            (validation_root / "pairing.json").write_text(
                json.dumps(pairing),
                encoding="utf-8",
            )
            with ExitStack() as stack:
                for patcher in (
                    mock.patch.object(
                        self.gate,
                        "VAL_ROOT",
                        validation_root,
                    ),
                    mock.patch.object(
                        self.gate,
                        "preflight_report",
                        return_value={"issues": []},
                    ),
                    mock.patch.object(
                        self.gate,
                        "rejected_9k_decision",
                        return_value=([], rejected),
                    ),
                    mock.patch.object(
                        self.gate,
                        "rejected_9k_audit_reference",
                        return_value=([], audit),
                    ),
                    mock.patch.object(
                        self.gate,
                        "inspect_candidate",
                        return_value=child,
                    ),
                    mock.patch.object(
                        self.gate,
                        "inspect_reproduction_report",
                        return_value=([], reproduction),
                    ),
                    mock.patch.object(
                        self.gate,
                        "pairing_manifest",
                        return_value=pairing,
                    ),
                    mock.patch.object(
                        self.gate.BASE,
                        "inspect_validation",
                        return_value=([], metrics, {"status": "pass"}),
                    ),
                    mock.patch.object(
                        self.gate,
                        "inspect_comparison",
                        return_value=([], {"status": "pass"}),
                    ),
                ):
                    stack.enter_context(patcher)
                report = self.gate.final_report(Path("candidate.pt"))
            self.assertEqual(set(report), set(self.gate.FINAL_REPORT_KEYS))
            self.assertEqual(report["status"], "pass")
            self.assertEqual(report["decision"], "promote_8k")
            self.assertEqual(report["selected_checkpoint"], fixed)
            self.assertEqual(
                report["candidate_checkpoint"],
                candidate_identity,
            )
            self.assertEqual(report["rejected_9k_decision"], rejected)


if __name__ == "__main__":
    unittest.main()
