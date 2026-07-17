"""Static safety checks for the directional cross-negative runner."""

# ruff: noqa: S607

from __future__ import annotations

import subprocess
import unittest
from pathlib import Path


class DetectorCrossNegativeRunnerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.path = (
            Path(__file__).parents[1]
            / "runs/reports/dfine-mt-object-field-7d-v1"
            / "run_detector_cross_negative.sh"
        )
        cls.source = cls.path.read_text(encoding="utf-8")

    def test_shell_syntax(self) -> None:
        result = subprocess.run(
            ["bash", "-n", str(self.path)],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_aggregate_modes_reenter_non_executable_runner_via_bash(
        self,
    ) -> None:
        self.assertIn('"$BASH" "$0" "$1" "$2"', self.source)
        self.assertIn(
            'run_pair_child "$child_mode" "$child_pair"',
            self.source,
        )

    def test_compare_all_attempts_every_pair_and_returns_failure(self) -> None:
        start = self.source.index("run_all_pairs() {")
        end = self.source.index("\n}\n", start) + 3
        function = self.source[start:end]
        harness = f"""
set +e
{function}
run_pair_child() {{
  printf '%s\\n' "$2"
  if [[ "$2" == "bundle" ]]; then
    return 7
  fi
  return 0
}}
run_all_pairs compare
"""
        result = subprocess.run(
            ["bash", "-c", harness],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(
            result.stdout.splitlines(),
            ["bundle", "forward", "reverse"],
        )
        self.assertEqual(result.returncode, 7)
        self.assertIn("Pair bundle failed with status 7", result.stderr)

    def test_screen_floor_is_frozen_below_promotion_floor(self) -> None:
        self.assertIn("screen_minimum_iterations=200", self.source)
        self.assertIn("promotion_minimum_iterations=2000", self.source)
        self.assertIn(
            '--minimum-iterations "$screen_minimum_iterations"',
            self.source,
        )
        self.assertNotIn('--minimum-iterations "$iterations"', self.source)

    def test_three_matched_directional_pairs_are_declared(self) -> None:
        for value in (
            'candidate_arm="a11-bundle"',
            'candidate_arm="a10-forward"',
            'candidate_arm="a01-reverse"',
            'control_arm="a00-${pair}-control"',
            "--person-batch-robot-detector-negative-weight",
            "--robot-batch-person-detector-negative-weight",
        ):
            self.assertIn(value, self.source)
        self.assertIn("--cross-pose-detector-negative-weight 0", self.source)
        for mode in (
            "smoke-all",
            "train-all",
            "validate-campaign-all",
            "compare-all",
        ):
            self.assertIn(mode, self.source)

    def test_both_training_directions_require_locked_manifests(self) -> None:
        for value in (
            "--coco-robot-negative-manifest",
            "--dhrp-person-negative-manifest",
            "COCO_ROBOT_NEGATIVE_MANIFEST_SHA256",
            "COCO_ROBOT_NEGATIVE_TRAIN_REVIEWED_RECORDS",
            "COCO_ROBOT_NEGATIVE_TRAIN_EXCLUDED_RECORDS",
            "COCO_ROBOT_NEGATIVE_TRAIN_TARGET_FINGERPRINT",
            "COCO_ROBOT_NEGATIVE_VAL_IMAGE_FINGERPRINT",
            "--coco-train-reviewed-records",
            "--coco-train-excluded-records",
            "--coco-val-image-fingerprint",
        ):
            self.assertIn(value, self.source)

    def test_validation_populations_are_not_mixed(self) -> None:
        for value in (
            "validate-canonical-unique",
            "canonical-clean-unique.yaml",
            "field-clean-unique.yaml",
            "validate-reverse-primary",
            "dhrp-person-negative-primary-eval-v1.json",
            "primary_evaluation",
            "validate-reverse-broad",
            "stress_evaluation",
            "validate-loss-holdout",
            "loss_holdout_validation",
            '--coco-robot-negative-manifest "$COCO_ROBOT_NEGATIVE_MANIFEST"',
        ):
            self.assertIn(value, self.source)
        self.assertNotIn(
            "coco-pose-robot-negative-reviewed-val-v1.json",
            self.source,
        )

    def test_campaign_is_the_explicit_allowlist_revision(self) -> None:
        self.assertIn(
            'campaign_name="safe-directional-cross-negative-v4-classifier-only"',
            self.source,
        )

    def test_paths_bind_run_attempt_seed_mode_and_pair(self) -> None:
        self.assertIn(
            'campaign_root="$train_root/$run_id/$attempt_id/s${seed}"',
            self.source,
        )
        self.assertIn(
            'local root="$campaign_root/$expected_mode/$pair"',
            self.source,
        )
        self.assertIn("Refusing reused output/log path", self.source)
        self.assertIn("--expected-mode", self.source)
        self.assertIn("ATTEMPT_ID", self.source)
        self.assertIn("--attempt-id", self.source)
        self.assertIn("--determinism-mode", self.source)
        self.assertIn("-${determinism_mode}-", self.source)

    def test_classifier_only_optimization_contract_is_explicit(self) -> None:
        for value in (
            "--trainable-profile cross_negative_classifier_only",
            "--learning-rate 3e-5",
            "--classifier-learning-rate 3e-5",
            "--weight-decay 0",
            "--warmup-steps 250",
            "--learning-rate-schedule constant",
            "--ema-decay 0",
            "--ema-warmups 0",
            "--object-weight 2",
            "--person-weight 1",
            "--robot-weight 1",
            "--field-weight 0",
            "run_pair train 1 1000",
        ):
            self.assertIn(value, self.source)
        self.assertNotIn("run_pair train 3 1000", self.source)

    def test_source_and_all_dataset_yamls_are_sha_locked(self) -> None:
        for value in (
            "LOCKED_SOURCE_CODE_SHA256",
            "LOCKED_OBJECT_DATA_SHA256",
            "LOCKED_PERSON_DATA_SHA256",
            "LOCKED_FIELD_DATA_SHA256",
            "canonical_object_data_sha256",
            "unique_object_data_sha256",
            "canonical_field_data_sha256",
            "unique_field_data_sha256",
            "assert_source_sha",
            "--source-code-sha256",
            "--object-data-sha256",
            "--person-data-sha256",
            "--field-data-sha256",
        ):
            self.assertIn(value, self.source)

    def test_unique_tier_is_compared_and_passed_to_gate(self) -> None:
        for value in (
            "compare_tier canonical-unique",
            "--parent-unique-val",
            "--control-unique-val",
            "--candidate-unique-val",
            "--absolute-unique-comparison",
            "--causal-unique-comparison",
        ):
            self.assertIn(value, self.source)


if __name__ == "__main__":
    unittest.main()
