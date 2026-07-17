"""Focused tests for the Object/Field-first Person alpha tradeoff gate."""

from __future__ import annotations

import importlib.util
import tempfile
import unittest
from copy import deepcopy
from pathlib import Path
from types import ModuleType


def load_gate() -> ModuleType:
    path = (
        Path(__file__).parents[1]
        / "runs/reports/dfine-mt-object-field-7d-v1"
        / "person_alpha_tradeoff_gate.py"
    )
    spec = importlib.util.spec_from_file_location("person_alpha_tradeoff", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class PersonAlphaTradeoffGateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.gate = load_gate()
        cls.metrics = cls.gate.load_json(cls.gate.METRICS)
        cls.offline = cls.gate.load_json(cls.gate.OFFLINE_AUDIT)

    def test_real_locked_evidence_selects_alpha_1_75(self) -> None:
        report = self.gate.build_decision()
        self.assertEqual(report["status"], "pass")
        self.assertEqual(report["decision"]["selected_alpha"], 1.75)
        self.assertEqual(
            report["decision"]["strict_pose_improvement_gate_decision"],
            "keep_alpha_1",
        )
        rows = {
            row["alpha"]: row
            for row in report["policy_evaluation"]["candidates"]
        }
        self.assertFalse(rows[1.5]["eligible"])
        self.assertTrue(rows[1.75]["eligible"])
        self.assertFalse(rows[2.0]["eligible"])

    def test_policy_thresholds_are_exactly_predeclared(self) -> None:
        self.assertEqual(
            self.gate.POLICY,
            {
                "primary_suppression_point_min": 0.15,
                "primary_suppression_ci95_lower_min": 0.15,
                "broad_suppression_min": 0.0,
                "holdout_suppression_min": 0.0,
                "person_map_loss_max": 0.002,
                "alpha2_map_loss_recovery_min_fraction": 0.25,
                "person_map50_loss_max": 0.015,
                "person_map75_loss_max": 0.002,
                "person_mar_loss_max": 0.002,
                "selection": "smallest_eligible_alpha",
            },
        )

    def test_ci_lower_bound_is_required_independently_of_point(self) -> None:
        replay = deepcopy(self.offline["negative_replay"])
        replay["alpha_1_75"]["primary_suppression"] = 0.25
        replay["alpha_1_75"]["primary_suppression_ci95"][0] = 0.149
        issues, result = self.gate.evaluate_policy(self.metrics, replay)
        self.assertEqual(issues, [])
        row = next(row for row in result["candidates"] if row["alpha"] == 1.75)
        self.assertTrue(row["gates"]["primary_suppression_point"])
        self.assertFalse(row["gates"]["primary_suppression_ci95_lower"])
        self.assertNotEqual(result["selected_alpha"], 1.75)

    def test_broad_and_holdout_false_detection_nonincrease_are_required(
        self,
    ) -> None:
        for key in ("broad_suppression", "holdout_suppression"):
            with self.subTest(key=key):
                replay = deepcopy(self.offline["negative_replay"])
                replay["alpha_1_75"][key] = -1e-6
                issues, result = self.gate.evaluate_policy(
                    self.metrics,
                    replay,
                )
                self.assertEqual(issues, [])
                row = next(
                    row for row in result["candidates"] if row["alpha"] == 1.75
                )
                gate_name = (
                    "broad_false_detections_nonincrease_vs_alpha_1"
                    if key == "broad_suppression"
                    else "holdout_false_detections_nonincrease_vs_alpha_1"
                )
                self.assertFalse(row["gates"][gate_name])

    def test_alpha2_loss_recovery_is_required(self) -> None:
        metrics = dict(self.metrics)
        baseline = metrics[self.gate.metric_key(1.0, "map")]
        alpha2 = metrics[self.gate.metric_key(2.0, "map")]
        alpha2_loss = baseline - alpha2
        metrics[self.gate.metric_key(1.75, "map")] = (
            baseline - 0.758 * alpha2_loss
        )
        issues, result = self.gate.evaluate_policy(
            metrics,
            self.offline["negative_replay"],
        )
        self.assertEqual(issues, [])
        row = next(row for row in result["candidates"] if row["alpha"] == 1.75)
        self.assertTrue(row["gates"]["person_map_loss"])
        self.assertFalse(row["gates"]["alpha2_map_loss_recovery"])

    def test_report_is_immutable(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "decision.json"
            self.gate.write_report(path, {"status": "pass"})
            with self.assertRaises(FileExistsError):
                self.gate.write_report(path, {"status": "pass"})


if __name__ == "__main__":
    unittest.main()
