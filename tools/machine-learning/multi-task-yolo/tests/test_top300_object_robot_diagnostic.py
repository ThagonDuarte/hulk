"""Adversarial tests for the immutable Object/Robot top-300 diagnostic."""

from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from copy import deepcopy
from pathlib import Path
from types import ModuleType
from typing import Any


def load_tool() -> ModuleType:
    """Load the report-local utility without making reports a package."""
    path = (
        Path(__file__).parents[1]
        / "runs/reports/dfine-mt-object-field-7d-v1"
        / "top300_object_robot_diagnostic.py"
    )
    spec = importlib.util.spec_from_file_location("top300_diagnostic", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class Top300DiagnosticTests(unittest.TestCase):
    """Exercise provenance, selection, matching, and replay gates."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.tool = load_tool()
        cls.manifest = cls.tool.load_json(cls.tool.DEFAULT_MANIFEST)

    def test_real_manifest_is_exactly_300_with_exact_quotas(self) -> None:
        entries = self.manifest["entries"]
        self.assertEqual(len(entries), 300)
        identities = {(entry["task"], entry["image_key"]) for entry in entries}
        self.assertEqual(len(identities), 300)
        counts = {
            bucket: sum(entry["assigned_bucket"] == bucket for entry in entries)
            for bucket in self.tool.BUCKET_ORDER
        }
        self.assertEqual(counts, self.tool.BUCKET_QUOTAS)
        other = [
            entry
            for entry in entries
            if entry["assigned_bucket"] == "object_other_class_fp_fn"
        ]
        class_counts = {
            class_id: sum(
                entry["focus"]["class_id"] == class_id for entry in other
            )
            for class_id in self.tool.OTHER_CLASS_IDS
        }
        self.assertEqual(class_counts, {0: 7, 1: 7, 2: 7, 3: 7, 5: 6, 6: 6})

    def test_real_self_replay_is_all_ties(self) -> None:
        path = self.tool.REPORT_ROOT / (
            "top300-object-robot-parent-self-replay-v1.json"
        )
        replay = self.tool.load_json(path)
        self.assertEqual(replay["status"], "pass")
        self.assertEqual(replay["summary"]["records"], 300)
        self.assertEqual(replay["summary"]["wins"], 0)
        self.assertEqual(replay["summary"]["losses"], 0)
        self.assertEqual(replay["summary"]["ties"], 300)

    def test_manifest_selection_tamper_is_rejected(self) -> None:
        tampered = deepcopy(self.manifest)
        tampered["entries"][0]["image_key"] += ":candidate-picked"
        with self.assertRaisesRegex(
            ValueError, "exact deterministic selection"
        ):
            self.tool.assert_exact_manifest(tampered, self.manifest)

    def test_manifest_duplicate_attack_is_rejected(self) -> None:
        tampered = deepcopy(self.manifest)
        tampered["entries"][1] = deepcopy(tampered["entries"][0])
        with self.assertRaisesRegex(
            ValueError, "exact deterministic selection"
        ):
            self.tool.assert_exact_manifest(tampered, self.manifest)

    def test_manifest_provenance_hash_tamper_is_rejected(self) -> None:
        tampered = deepcopy(self.manifest)
        tampered["parent"]["predictions"]["sha256"] = "0" * 64
        with self.assertRaisesRegex(
            ValueError, "exact deterministic selection"
        ):
            self.tool.assert_exact_manifest(tampered, self.manifest)

    def test_equal_score_matching_uses_stable_prediction_index(self) -> None:
        similarities = self.tool.np.asarray([[0.9], [0.95]])
        scores = self.tool.np.asarray([0.7, 0.7])
        matches, false_positives = self.tool.greedy_matches(
            similarities,
            scores,
            0.5,
        )
        self.assertEqual(matches, {0: 0})
        self.assertEqual(false_positives, {1})

    def test_family_backfill_never_crosses_task_family(self) -> None:
        pools = {bucket: [] for bucket in self.tool.BUCKET_ORDER}
        for index in range(300):
            event = self.tool.Event(
                source_bucket="object_robot_false_positive",
                task="object",
                image_key=f"object:{index}",
                path=f"/object/{index}.png",
                image_id=index,
                record_index=index,
                severity=float(300 - index),
                focus={"kind": "false_positive", "class_id": 4},
            )
            pools["object_robot_false_positive"].append(event)
        with self.assertRaisesRegex(ValueError, "robot_pose_missed_detection"):
            self.tool.allocate_events(pools)

    def test_candidate_target_stream_tamper_is_rejected(self) -> None:
        parent_record = self._record(target_label=4)
        candidate_record = self._record(target_label=0)
        with tempfile.TemporaryDirectory() as directory:
            parent_dir = Path(directory) / "parent"
            candidate_dir = Path(directory) / "candidate"
            self._write_run_sidecars(parent_dir, model="parent.pt")
            self._write_run_sidecars(candidate_dir, model="candidate.pt")
            with self.assertRaisesRegex(ValueError, "target stream differs"):
                self.tool.validate_candidate_archive(
                    parent_dir,
                    candidate_dir,
                    [parent_record],
                    [candidate_record],
                )

    def test_candidate_threshold_change_is_rejected(self) -> None:
        record = self._record(target_label=4)
        with tempfile.TemporaryDirectory() as directory:
            parent_dir = Path(directory) / "parent"
            candidate_dir = Path(directory) / "candidate"
            self._write_run_sidecars(parent_dir, model="parent.pt")
            self._write_run_sidecars(
                candidate_dir,
                model="candidate.pt",
                confidence=0.25,
            )
            with self.assertRaisesRegex(ValueError, "config differs"):
                self.tool.validate_candidate_archive(
                    parent_dir,
                    candidate_dir,
                    [record],
                    [record],
                )

    def test_pose_high_error_bucket_marks_box_iou_unavailable(self) -> None:
        rows = [
            entry
            for entry in self.manifest["entries"]
            if entry["assigned_bucket"]
            == "robot_pose_high_keypoint_error_oks_matched_substitute"
        ]
        self.assertEqual(len(rows), 40)
        self.assertTrue(
            all(entry["focus"]["predicted_box_iou"] is None for entry in rows)
        )
        schema = self.manifest["schemas"]["robot_pose"]
        self.assertFalse(schema["predicted_box_support"])
        self.assertIn(
            "no predicted detector box", schema["predicted_box_limitation"]
        )

    def test_immutable_writer_refuses_even_identical_overwrite(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "artifact.json"
            self.tool.write_new_json(path, {"status": "pass"})
            with self.assertRaises(FileExistsError):
                self.tool.write_new_json(path, {"status": "pass"})

    def _record(self, *, target_label: int) -> Any:
        value = {
            "version": 1,
            "task": "object",
            "image_key": "object:/image.png",
            "image_id": 0,
            "path": "/image.png",
            "evaluation_size": [448, 544],
            "original_size": [448, 544],
            "predictions": {
                "boxes_xyxy": [],
                "scores": [],
                "labels": [],
            },
            "targets": {
                "boxes_xyxy": [[0, 0, 1, 1]],
                "labels": [target_label],
            },
        }
        return self.tool.ArchiveRecord(
            index=0,
            raw_line_sha256=self.tool.value_sha256(value),
            value=value,
        )

    def _write_run_sidecars(
        self,
        directory: Path,
        *,
        model: str,
        confidence: float = 0.001,
    ) -> None:
        directory.mkdir(parents=True)
        model_path = directory / model
        model_path.write_bytes(model.encode("utf-8"))
        config = dict.fromkeys(self.tool.CONFIG_EXCEPT_MODEL)
        config.update(
            {
                "model": str(model_path),
                "confidence": confidence,
                "tasks": ["object", "robot_pose"],
                "robot_visibility_alphas": [2.0],
            }
        )
        metadata = {
            "architecture": "dfine-multitask",
            "backend": {"backend": "pytorch-rgb"},
            "checkpoint_sha256": self.tool.sha256(model_path),
            "dataset_fingerprints": {
                "object": {"digest": "a" * 64},
                "robot_pose": {"digest": "b" * 64},
            },
        }
        (directory / "config.json").write_text(
            json.dumps(config),
            encoding="utf-8",
        )
        (directory / "metadata.json").write_text(
            json.dumps(metadata),
            encoding="utf-8",
        )


if __name__ == "__main__":
    unittest.main()
