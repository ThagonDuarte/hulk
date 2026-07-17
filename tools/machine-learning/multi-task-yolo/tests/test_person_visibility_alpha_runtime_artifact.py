"""Pre/post-application checks for the Person visibility-alpha patch."""

from __future__ import annotations

import hashlib
import json
import re
import subprocess
import unittest
from pathlib import Path


class PersonVisibilityAlphaArtifactTests(unittest.TestCase):
    """Bind the patch to its reviewed base and required safety contract."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.project = Path(__file__).resolve().parents[1]
        cls.repo = Path(__file__).resolve().parents[4]
        cls.patch = (
            cls.project
            / "runs/reports/dfine-mt-object-field-7d-v1"
            / "person-visibility-alpha-runtime-v1.incremental.apply_patch"
        )
        cls.text = cls.patch.read_text(encoding="utf-8")
        cls.artifact = json.loads(
            cls.patch.with_name(
                "person-visibility-alpha-runtime-v1.artifact.json"
            ).read_text(encoding="utf-8")
        )

    def _application_state(self) -> str:
        states: set[str] = set()
        for relative, base_digest in self.artifact[
            "reviewed_base_sha256"
        ].items():
            applied_digest = self.artifact["proposed_sha256"][relative]
            actual = hashlib.sha256(
                (self.repo / relative).read_bytes()
            ).hexdigest()
            if actual == base_digest:
                states.add("prepared")
            elif actual == applied_digest:
                states.add("applied")
            else:
                self.fail(
                    f"{relative} is neither the reviewed base nor the "
                    "applied state: "
                    f"{actual}"
                )
        self.assertEqual(
            len(states), 1, f"mixed patch application state: {states}"
        )
        return states.pop()

    def test_patch_matches_exact_current_worktree_state(self) -> None:
        state = self._application_state()
        if state == "applied":
            return
        command = ["/usr/bin/git", "apply"]
        command.extend(("--check", str(self.patch)))
        result = subprocess.run(
            command,
            cwd=self.repo,
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_reviewed_files_are_all_prepared_or_all_applied(self) -> None:
        state = self._application_state()
        expected = self.artifact[
            "reviewed_base_sha256" if state == "prepared" else "proposed_sha256"
        ]
        for relative, digest in expected.items():
            with self.subTest(path=relative):
                actual = hashlib.sha256(
                    (self.repo / relative).read_bytes()
                ).hexdigest()
                self.assertEqual(actual, digest)

    def test_patch_scope_excludes_model_and_training_source(self) -> None:
        paths = re.findall(r"^diff --git a/(\S+) b/\S+$", self.text, re.M)
        self.assertEqual(len(paths), 9)
        self.assertFalse(
            any(
                path.startswith("tools/machine-learning/multi-task-yolo/src/")
                for path in paths
            )
        )
        self.assertFalse(any(path.endswith((".pt", ".onnx")) for path in paths))

    def test_selected_alphas_are_explicit_and_independent(self) -> None:
        for required in (
            "+    visibility_score_alpha: 1.75,",
            "+    visibility_score_alpha: 2.0,",
            "+        person_visibility_alpha: f32,",
            "robot_visibility_score_alpha: f32,",
            "+    pub person_visibility_score_alpha: f32,",
            "pub robot_visibility_score_alpha: f32,",
            '--person-visibility-alpha "$PERSON_ALPHA"',
            "PERSON_ALPHA ROBOT_ALPHA",
        ):
            self.assertIn(required, self.text)

    def test_runtime_calibration_is_post_onnx_and_cache_bound(self) -> None:
        for required in (
            "base_confidence",
            "visibility_sum / keypoint_count as f32",
            "pose_values[4] = confidence",
            "MODEL_RUN_PAYLOAD_VERSION: u32 = 6",
            "person_visibility_score_alpha.to_bits()",
            "validate_person_visibility_score_alpha",
            "v5_manifest_migrates_to_person_alpha_zero_v6_identity",
            "person_pose_visibility_alpha_calibrates_only_pose_confidence",
        ):
            self.assertIn(required, self.text)
        self.assertNotIn("src/ultralytics_dfine/nn/multitask.py", self.text)
        self.assertNotIn("src/utils/export_hydra.py", self.text)

    def test_deployment_guards_against_double_calibration(self) -> None:
        self.assertGreaterEqual(
            self.text.count('visibility_score_alpha") == 0.0'),
            2,
        )
        self.assertIn(
            "manifest {task} visibility alpha must remain zero", self.text
        )
        self.assertIn(
            "checkpoint {task} visibility alpha must remain zero", self.text
        )
        self.assertIn("replay person alpha mismatch", self.text)


if __name__ == "__main__":
    unittest.main()
