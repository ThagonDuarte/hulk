"""Tests for the campaign-specific directional cross-negative gate."""

# ruff: noqa: E501, TRY003

from __future__ import annotations

import gzip
import hashlib
import importlib.util
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from types import ModuleType

import torch


def _gate() -> ModuleType:
    path = (
        Path(__file__).parents[1]
        / "runs/reports/dfine-mt-object-field-7d-v1"
        / "detector_cross_negative_gate.py"
    )
    spec = importlib.util.spec_from_file_location(
        "detector_negative_gate",
        path,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("Could not load detector-negative gate")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class DetectorCrossNegativeGateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.gate = _gate()

    @staticmethod
    def _config(
        forward: float,
        reverse: float,
        *,
        point_weight: float = 5.0,
        legacy: float = 0.0,
    ) -> dict:
        loss = {
            "field_features": {"point_weight": point_weight},
            "cross_pose_visibility_negative_weight": 0.0,
            "cross_pose_detector_negative_weight": legacy,
            "person_batch_robot_visibility_negative_weight": 0.0,
            "robot_batch_person_visibility_negative_weight": 0.0,
            "person_batch_robot_detector_negative_weight": forward,
            "robot_batch_person_detector_negative_weight": reverse,
        }
        return {
            "output_dir": f"run-{forward}-{reverse}",
            "run_name": f"arm-{forward}-{reverse}",
            "seed": 17,
            "provenance": {"loss_config": repr(loss)},
        }

    def _write_canonical_group(
        self,
        root: Path,
        tier: str,
        *,
        overrides: dict[str, dict[str, object]] | None = None,
    ) -> tuple[
        tuple[tuple[Path, Path, str], ...],
        Path,
        Path,
        Path,
    ]:
        object_data = root / f"{tier}-object.yaml"
        person_data = root / "person.yaml"
        field_data = root / f"{tier}-field.yaml"
        for path, content in (
            (object_data, "object\n"),
            (person_data, "person\n"),
            (field_data, "field\n"),
        ):
            path.write_text(content, encoding="utf-8")
        entries = []
        for name in ("parent", "control", "candidate"):
            checkpoint = root / f"{tier}-{name}.pt"
            checkpoint.write_bytes(name.encode())
            validation = root / f"{tier}-{name}"
            validation.mkdir()
            (validation / "config.json").write_text(
                json.dumps(
                    {
                        "object_data": str(object_data.resolve()),
                        "person_data": str(person_data.resolve()),
                        "field_data": str(field_data.resolve()),
                    }
                ),
                encoding="utf-8",
            )
            fingerprints = {
                task: {
                    "digest": self.gate._CANONICAL_EXACT_DIGESTS[tier][task],
                    "samples": samples,
                }
                for task, samples in self.gate._CANONICAL_TIERS[tier].items()
            }
            for task, values in (overrides or {}).items():
                fingerprints[task].update(values)
            (validation / "metadata.json").write_text(
                json.dumps(
                    {
                        "checkpoint_sha256": hashlib.sha256(
                            checkpoint.read_bytes()
                        ).hexdigest(),
                        "dataset_fingerprints": fingerprints,
                    }
                ),
                encoding="utf-8",
            )
            entries.append((checkpoint, validation, name))
        return tuple(entries), object_data, person_data, field_data

    def _run_canonical_group(
        self,
        root: Path,
        tier: str,
        *,
        overrides: dict[str, dict[str, object]] | None = None,
    ) -> object:
        entries, object_data, person_data, field_data = (
            self._write_canonical_group(root, tier, overrides=overrides)
        )
        decision = self.gate.Decision()
        self.gate._canonical_validation_group_gates(
            decision,
            tier=tier,
            entries=entries,
            object_data=object_data,
            object_data_sha256=hashlib.sha256(
                object_data.read_bytes()
            ).hexdigest(),
            person_data=person_data,
            person_data_sha256=hashlib.sha256(
                person_data.read_bytes()
            ).hexdigest(),
            field_data=field_data,
            field_data_sha256=hashlib.sha256(
                field_data.read_bytes()
            ).hexdigest(),
        )
        return decision

    def _cross_comparison(
        self,
        direction: str,
        *,
        ratio: float,
    ) -> dict[str, object]:
        rows = []
        for alpha in ("0", self.gate._DEPLOYED_ALPHA[direction]):
            for threshold in self.gate._THRESHOLDS:
                rows.append(
                    {
                        "metric": self.gate._cross_metric(
                            direction,
                            alpha,
                            f"detections_at_{threshold}_per_image",
                        ),
                        "baseline": 1.0,
                        "candidate": ratio,
                        "ci95_upper": -0.01,
                    }
                )
            rows.append(
                {
                    "metric": self.gate._cross_metric(
                        direction,
                        alpha,
                        "max_score_mean",
                    ),
                    "baseline": 1.0,
                    "candidate": ratio,
                    "ci95_upper": -0.01,
                }
            )
        return {"metrics": rows}

    def test_both_directional_detector_factors_are_normalized(self) -> None:
        control = self._config(0.0, 0.0)
        candidate = self._config(0.25, 0.25)
        self.assertEqual(
            self.gate._normalized_training_config(control),
            self.gate._normalized_training_config(candidate),
        )

    def test_legacy_or_protected_loss_difference_remains_visible(self) -> None:
        control = self._config(0.0, 0.0)
        legacy = self._config(0.25, 0.25, legacy=0.25)
        field = self._config(0.25, 0.25, point_weight=6.0)
        self.assertNotEqual(
            self.gate._normalized_training_config(control),
            self.gate._normalized_training_config(legacy),
        )
        self.assertNotEqual(
            self.gate._normalized_training_config(control),
            self.gate._normalized_training_config(field),
        )

    def test_pair_arm_mapping_is_explicit(self) -> None:
        self.assertEqual(
            self.gate._expected_arm_names("bundle"),
            ("a00-bundle-control", "a11-bundle"),
        )
        self.assertEqual(
            self.gate._expected_arm_names("forward"),
            ("a00-forward-control", "a10-forward"),
        )
        self.assertEqual(
            self.gate._expected_arm_names("reverse"),
            ("a00-reverse-control", "a01-reverse"),
        )

    def test_training_log_gate_checks_masked_applied_counts(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            checkpoint = Path(directory) / "last.pt"
            records = [
                {
                    "train/step/task": "person_pose",
                    "system/gpu_memory_reserved_bytes": 10 * 1024**3,
                },
                {"train/step/task": "robot_pose"},
                {
                    "train/epoch/cross_negative/person_records_seen": 32,
                    "train/epoch/cross_negative/robot_records_seen": 32,
                    "train/epoch/cross_negative/person_robot_reviewed_seen": 30,
                    "train/epoch/cross_negative/person_robot_verified_seen": 28,
                    "train/epoch/cross_negative/person_robot_eligible_seen": 28,
                    "train/epoch/cross_negative/person_robot_excluded_seen": 2,
                    "train/epoch/cross_negative/person_robot_unreviewed_seen": 2,
                    "train/epoch/cross_negative/robot_person_verified_seen": 10,
                    "train/epoch/cross_negative/robot_person_eligible_seen": 8,
                    "train/epoch/cross_negative/person_robot_detector_applied_records": 28,
                    "train/epoch/cross_negative/robot_person_detector_applied_records": 0,
                    "train/epoch/cross_negative/person_robot_visibility_applied_records": 0,
                    "train/epoch/cross_negative/robot_person_visibility_applied_records": 0,
                },
            ]
            (checkpoint.parent / "metrics.jsonl").write_text(
                "".join(json.dumps(record) + "\n" for record in records),
                encoding="utf-8",
            )
            decision = self.gate.Decision()

            self.gate._training_log_gates(
                decision,
                "forward",
                checkpoint,
                forward_weight=0.25,
                reverse_weight=0.0,
                maximum_memory_gib=22.0,
                expected_person_records=32,
                expected_robot_records=32,
            )

        self.assertTrue(all(decision.gates.values()), decision.details)

    def test_tensor_invariance_uses_parent_inference_for_all_child_branches(
        self,
    ) -> None:
        parent_state = {
            "frozen.weight": torch.tensor([1.0, 2.0]),
            "detector.core.model.decoder.class_embed.0.weight": (
                torch.tensor([3.0])
            ),
            "detector.core.class_embed.0.weight": torch.tensor([3.0]),
        }
        changed_classifier = {
            **parent_state,
            "detector.core.model.decoder.class_embed.0.weight": (
                torch.tensor([4.0])
            ),
            "detector.core.class_embed.0.weight": torch.tensor([4.0]),
        }
        parent = {"inference_model": parent_state}
        child = {
            "model": changed_classifier,
            "ema": {"module": changed_classifier},
            "inference_model": changed_classifier,
        }
        decision = self.gate.Decision()

        self.gate._tensor_invariance_gates(
            decision,
            name="candidate",
            parent_checkpoint=parent,
            child_checkpoint=child,
        )

        self.assertTrue(all(decision.gates.values()), decision.details)
        child["inference_model"] = {
            **changed_classifier,
            "frozen.weight": torch.tensor([9.0, 2.0]),
        }
        decision = self.gate.Decision()
        self.gate._tensor_invariance_gates(
            decision,
            name="candidate",
            parent_checkpoint=parent,
            child_checkpoint=child,
        )
        self.assertFalse(
            decision.gates[
                "training/candidate/non_classifier_tensor_invariance/"
                "inference_model"
            ]
        )

    def test_v4_train_horizon_and_task_counts_are_surgical(self) -> None:
        self.assertEqual(
            self.gate._MODE_HORIZONS["train"],
            {"epochs": 1, "steps_per_epoch": 1000, "global_step": 1000},
        )
        self.assertEqual(
            self.gate._task_batch_counts("train"),
            {
                "object": 500,
                "person_pose": 250,
                "robot_pose": 250,
                "field_features": 0,
            },
        )

    def test_unique_validation_group_binds_yaml_and_effective_population(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            object_data = root / "object-unique.yaml"
            person_data = root / "person.yaml"
            field_data = root / "field-unique.yaml"
            for path, content in (
                (object_data, "object\n"),
                (person_data, "person\n"),
                (field_data, "field\n"),
            ):
                path.write_text(content, encoding="utf-8")
            checkpoints = []
            entries = []
            for name in ("parent", "control", "candidate"):
                checkpoint = root / f"{name}.pt"
                checkpoint.write_bytes(name.encode())
                validation = root / name
                validation.mkdir()
                (validation / "config.json").write_text(
                    json.dumps(
                        {
                            "object_data": str(object_data.resolve()),
                            "person_data": str(person_data.resolve()),
                            "field_data": str(field_data.resolve()),
                        }
                    ),
                    encoding="utf-8",
                )
                (validation / "metadata.json").write_text(
                    json.dumps(
                        {
                            "checkpoint_sha256": hashlib.sha256(
                                checkpoint.read_bytes()
                            ).hexdigest(),
                            "dataset_fingerprints": {
                                "object": {
                                    "digest": self.gate._CANONICAL_EXACT_DIGESTS[
                                        "canonical-unique"
                                    ]["object"],
                                    "samples": 4690,
                                },
                                "field_features": {
                                    "digest": self.gate._CANONICAL_EXACT_DIGESTS[
                                        "canonical-unique"
                                    ]["field_features"],
                                    "samples": 3464,
                                },
                                "robot_pose": {
                                    "digest": self.gate._CANONICAL_EXACT_DIGESTS[
                                        "canonical-unique"
                                    ]["robot_pose"],
                                    "samples": 1454,
                                },
                            },
                        }
                    ),
                    encoding="utf-8",
                )
                checkpoints.append(checkpoint)
                entries.append((checkpoint, validation, name))
            decision = self.gate.Decision()

            self.gate._canonical_validation_group_gates(
                decision,
                tier="canonical-unique",
                entries=tuple(entries),
                object_data=object_data,
                object_data_sha256=hashlib.sha256(
                    object_data.read_bytes()
                ).hexdigest(),
                person_data=person_data,
                person_data_sha256=hashlib.sha256(
                    person_data.read_bytes()
                ).hexdigest(),
                field_data=field_data,
                field_data_sha256=hashlib.sha256(
                    field_data.read_bytes()
                ).hexdigest(),
            )

        self.assertTrue(all(decision.gates.values()), decision.details)

    def test_canonical_group_rejects_identical_content_drift_all_arms(
        self,
    ) -> None:
        cases = (
            ("canonical", "object"),
            ("canonical", "field_features"),
            ("canonical", "robot_pose"),
            ("canonical-unique", "object"),
            ("canonical-unique", "field_features"),
        )
        for tier, task in cases:
            with self.subTest(tier=tier, task=task):
                with tempfile.TemporaryDirectory() as directory:
                    decision = self._run_canonical_group(
                        Path(directory),
                        tier,
                        overrides={task: {"digest": "f" * 64}},
                    )
                for name in ("parent", "control", "candidate"):
                    self.assertFalse(
                        decision.gates[
                            f"validation/{tier}/{name}/{task}_identity"
                        ]
                    )
                self.assertTrue(
                    decision.gates[
                        f"validation/{tier}/{task}_same_population_all_arms"
                    ]
                )

    def test_canonical_group_rejects_robot_subset_substitution(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            decision = self._run_canonical_group(
                Path(directory),
                "canonical",
                overrides={
                    "robot_pose": {
                        "samples": 268,
                        "digest": self.gate._VALIDATION_IDENTITIES[
                            "reverse-primary"
                        ]["digest"],
                    }
                },
            )

        for name in ("parent", "control", "candidate"):
            gate = f"validation/canonical/{name}/robot_pose_identity"
            self.assertFalse(decision.gates[gate])
            self.assertEqual(
                decision.details[gate]["requirement"],
                str(
                    {
                        "digest": self.gate._CANONICAL_EXACT_DIGESTS[
                            "canonical"
                        ]["robot_pose"],
                        "samples": 1454,
                    }
                ),
            )

    def test_bootstrap_screen_cannot_emit_promotion(self) -> None:
        comparisons = {
            name: {"bootstrap": {"iterations": 200}}
            for name in (
                "absolute",
                "causal",
                "unique_absolute",
                "unique_causal",
                "reverse_absolute",
                "reverse_causal",
            )
        }
        eligible, observed = self.gate._promotion_iteration_status(comparisons)
        self.assertFalse(eligible)
        self.assertEqual(set(observed.values()), {200})
        self.assertEqual(
            self.gate._decision_label(
                training_only=False,
                expected_mode="train",
                pair="forward",
                passed=True,
                promotion_iteration_eligible=eligible,
            ),
            "pass_screen_forward",
        )
        for comparison in comparisons.values():
            comparison["bootstrap"]["iterations"] = 2000
        eligible, _ = self.gate._promotion_iteration_status(comparisons)
        self.assertTrue(eligible)
        self.assertEqual(
            self.gate._decision_label(
                training_only=False,
                expected_mode="train",
                pair="forward",
                passed=True,
                promotion_iteration_eligible=eligible,
            ),
            "promote_forward",
        )

    def test_disabled_direction_is_noninferior_without_efficacy_gate(
        self,
    ) -> None:
        forward_good = self._cross_comparison(
            self.gate._FORWARD_DIRECTION,
            ratio=0.8,
        )
        reverse_worse = self._cross_comparison(
            self.gate._REVERSE_DIRECTION,
            ratio=1.01,
        )
        decision = self.gate.Decision()
        self.gate._cross_direction_gates(
            decision,
            candidate_weights=(0.25, 0.0),
            forward_absolute=forward_good,
            forward_causal=forward_good,
            reverse_absolute=reverse_worse,
            reverse_causal=reverse_worse,
        )

        self.assertFalse(
            decision.gates[
                "reverse_primary_absolute_vs_parent/"
                "cross_pose/person_on_robot/alpha_0/detections_at_25"
            ]
        )
        self.assertFalse(
            any(
                name.startswith("reverse_primary_")
                and name.endswith("suppression_efficacy")
                for name in decision.gates
            )
        )

    def test_enabled_direction_requires_absolute_and_causal_efficacy(
        self,
    ) -> None:
        forward_absolute = self._cross_comparison(
            self.gate._FORWARD_DIRECTION,
            ratio=0.9,
        )
        forward_causal = self._cross_comparison(
            self.gate._FORWARD_DIRECTION,
            ratio=0.8,
        )
        reverse_equal = self._cross_comparison(
            self.gate._REVERSE_DIRECTION,
            ratio=1.0,
        )
        decision = self.gate.Decision()
        self.gate._cross_direction_gates(
            decision,
            candidate_weights=(0.25, 0.0),
            forward_absolute=forward_absolute,
            forward_causal=forward_causal,
            reverse_absolute=reverse_equal,
            reverse_causal=reverse_equal,
        )

        suffix = (
            "cross_pose/robot_on_person/alpha_0/detections_at_25/"
            "suppression_efficacy"
        )
        self.assertFalse(decision.gates[f"forward_absolute_vs_parent/{suffix}"])
        self.assertTrue(
            decision.gates[f"forward_causal_vs_matched_control/{suffix}"]
        )

    def test_sidecar_summary_preserves_policy_and_sources(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "cross_pose_predictions.jsonl.gz"
            with gzip.open(path, "wt", encoding="utf-8") as file:
                for index in range(2):
                    file.write(
                        json.dumps(
                            {
                                "task": "cross_pose/person_on_robot",
                                "image_key": f"image-{index}",
                                "negative_target_eligibility": {
                                    "policy": "verified-person-free-dhrp-v1",
                                    "annotation_file": (
                                        "eval_set_TargetHumanoidRobots_EVE.json"
                                    ),
                                    "verified": True,
                                    "eligible": True,
                                },
                            }
                        )
                        + "\n"
                    )

            summary = self.gate._cross_sidecar_summary(path)

        self.assertEqual(summary[0], 2)
        self.assertEqual(summary[1], {"cross_pose/person_on_robot"})
        self.assertEqual(len(summary[2]), 2)
        self.assertEqual(
            summary[3],
            {"eval_set_TargetHumanoidRobots_EVE.json"},
        )
        self.assertEqual(summary[4], {"verified-person-free-dhrp-v1"})
        self.assertTrue(summary[5])

    def test_reverse_broad_requires_exact_six_manifest_sources(self) -> None:
        expected_sources = self.gate._VALIDATION_IDENTITIES["reverse-broad"][
            "annotation_files"
        ]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            checkpoint = root / "checkpoint.pt"
            checkpoint.write_bytes(b"checkpoint")
            validation = root / "validation"
            validation.mkdir()
            (validation / "metadata.json").write_text(
                json.dumps(
                    {
                        "checkpoint_sha256": hashlib.sha256(
                            checkpoint.read_bytes()
                        ).hexdigest(),
                        "dataset_fingerprints": {
                            "robot_pose": {
                                "digest": self.gate._VALIDATION_IDENTITIES[
                                    "reverse-broad"
                                ]["digest"],
                                "samples": 825,
                            }
                        },
                    }
                ),
                encoding="utf-8",
            )
            (validation / "metrics.json").write_text(
                json.dumps({"cross_pose/person_on_robot/images": 825}),
                encoding="utf-8",
            )

            def write_sidecar(sources: tuple[str, ...]) -> None:
                with gzip.open(
                    validation / "cross_pose_predictions.jsonl.gz",
                    "wt",
                    encoding="utf-8",
                ) as file:
                    for index in range(825):
                        file.write(
                            json.dumps(
                                {
                                    "task": "cross_pose/person_on_robot",
                                    "image_key": f"image-{index}",
                                    "negative_target_eligibility": {
                                        "policy": (
                                            "verified-person-free-dhrp-v1"
                                        ),
                                        "annotation_file": sources[
                                            index % len(sources)
                                        ],
                                        "verified": True,
                                        "eligible": True,
                                    },
                                }
                            )
                            + "\n"
                        )

            write_sidecar(tuple(sorted(expected_sources)))
            approved = self.gate.Decision()
            self.gate._validation_identity_gates(
                approved,
                name="reverse-broad/approved",
                validation_dir=validation,
                checkpoint_path=checkpoint,
                identity_name="reverse-broad",
            )
            write_sidecar(
                (*tuple(sorted(expected_sources)), "unexpected-source.json")
            )
            unexpected = self.gate.Decision()
            self.gate._validation_identity_gates(
                unexpected,
                name="reverse-broad/unexpected",
                validation_dir=validation,
                checkpoint_path=checkpoint,
                identity_name="reverse-broad",
            )

        self.assertTrue(all(approved.gates.values()), approved.details)
        self.assertFalse(
            unexpected.gates[
                "validation/reverse-broad/unexpected/sidecar_population"
            ]
        )

    def test_forward_sidecar_requires_explicit_approved_decision(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "cross_pose_predictions.jsonl.gz"
            with gzip.open(path, "wt", encoding="utf-8") as file:
                file.write(
                    json.dumps(
                        {
                            "task": "cross_pose/robot_on_person",
                            "image_key": "approved",
                            "negative_target_eligibility": {
                                "policy": (
                                    "explicit-verified-robot-free-keypoint-v2"
                                ),
                                "reviewed": True,
                                "verified": True,
                                "eligible": True,
                                "excluded": False,
                            },
                        }
                    )
                    + "\n"
                )

            approved = self.gate._cross_sidecar_summary(path)

            with gzip.open(path, "wt", encoding="utf-8") as file:
                file.write(
                    json.dumps(
                        {
                            "task": "cross_pose/robot_on_person",
                            "image_key": "unreviewed",
                            "negative_target_eligibility": {
                                "policy": (
                                    "explicit-verified-robot-free-keypoint-v2"
                                ),
                                "reviewed": False,
                                "verified": True,
                                "eligible": True,
                                "excluded": False,
                            },
                        }
                    )
                    + "\n"
                )

            unreviewed = self.gate._cross_sidecar_summary(path)

        self.assertTrue(approved[5])
        self.assertFalse(unreviewed[5])

    def test_training_only_cli_accepts_exact_forward_smoke(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            parent = root / "parent.pt"
            parent_state = {
                "frozen.weight": torch.tensor([1.0, 2.0]),
                "detector.core.model.decoder.class_embed.0.weight": (
                    torch.tensor([3.0])
                ),
                "detector.core.class_embed.0.weight": torch.tensor([3.0]),
            }
            torch.save({"inference_model": parent_state}, parent)
            parent_sha = hashlib.sha256(parent.read_bytes()).hexdigest()
            coco_manifest_sha = "a" * 64
            coco_fingerprint = "b" * 64
            coco_val_fingerprint = "c" * 64
            source_sha = "d" * 64
            object_data_sha = "e" * 64
            person_data_sha = "f" * 64
            field_data_sha = "1" * 64

            def write_arm(
                arm: str,
                forward: float,
                applied: float,
            ) -> Path:
                output = (
                    root
                    / "rid"
                    / "attempt-strict-01"
                    / "s17"
                    / "smoke"
                    / "forward"
                    / arm
                )
                output.mkdir(parents=True)
                loss = {
                    **self.gate._EXPECTED_PROTECTED_LOSS_CONFIG,
                    "cross_pose_detector_negative_weight": 0.0,
                    "cross_pose_visibility_negative_weight": 0.0,
                    "person_batch_robot_detector_negative_weight": forward,
                    "robot_batch_person_detector_negative_weight": 0.0,
                    "person_batch_robot_visibility_negative_weight": 0.0,
                    "robot_batch_person_visibility_negative_weight": 0.0,
                }
                config = {
                    "output_dir": str(output),
                    "run_name": (
                        f"rid-attempt-strict-01-smoke-s17-forward-{arm}-"
                        "cross_negative_classifier_only"
                    ),
                    "seed": 17,
                    "epochs": 1,
                    "steps_per_epoch": 20,
                    "max_validation_batches": 1,
                    "stage_name": "cross_negative_classifier_only",
                    "trainable_roles": ("classifiers",),
                    "sampling_weights": {
                        "object": 2.0,
                        "person_pose": 1.0,
                        "robot_pose": 1.0,
                    },
                    "batch_size_per_rank": 16,
                    "validation_batch_size": 32,
                    "learning_rate": 3e-5,
                    "role_learning_rates": dict(
                        self.gate._EXPECTED_ROLE_LEARNING_RATES
                    ),
                    "weight_decay": 0.0,
                    "warmup_steps": 250,
                    "learning_rate_schedule": "constant",
                    "minimum_learning_rate_ratio": 0.1,
                    "clip_max_norm": 0.1,
                    "ema_decay": 0.0,
                    "ema_warmups": 0,
                    "amp": False,
                    "device": "cuda",
                    "validation_interval": 1,
                    "freeze_frozen_bn_stats": True,
                    "freeze_all_bn_stats": True,
                    "sync_batch_norm": False,
                    "deterministic": True,
                    "strict_deterministic": True,
                    "sdpa_backend": "math",
                    "ddp_find_unused_parameters": True,
                    "allow_existing_output": False,
                    "wandb_project": "multi-task-yolo-dfine",
                    "wandb_group": (
                        "safe-directional-cross-negative-v4-classifier-only-"
                        "rid-attempt-strict-01-s17-forward"
                    ),
                    "wandb_mode": "disabled",
                    "wandb_log_interval": 1,
                    "wandb_log_checkpoints": False,
                    "save_named_best_checkpoints": False,
                    "dataset_audits": {
                        "train/person_pose": {
                            "images": 56599,
                            "robot_negative_reviewed_records": 4927,
                            "robot_negative_verified_records": 4900,
                            "robot_negative_eligible_records": 4900,
                            "robot_negative_excluded_records": 27,
                            "robot_negative_unreviewed_records": 51672,
                        },
                        "train/robot_pose": {
                            "images": 14597,
                            "person_negative_verified_records": 5852,
                            "person_negative_eligible_records": 4927,
                        },
                    },
                    "dataset_fingerprints": {
                        "train/person_pose": {
                            "digest": coco_fingerprint,
                            "robot_negative_manifest_sha256": (
                                coco_manifest_sha
                            ),
                            "robot_negative_reviewed_records": 4927,
                            "robot_negative_eligible_records": 4900,
                            "robot_negative_excluded_records": 27,
                            "robot_negative_unreviewed_records": 51672,
                        },
                        "train/robot_pose": {
                            "digest": self.gate._DHRP_TRAIN_TARGET_FINGERPRINT
                        },
                    },
                    "provenance": {
                        "loss_config": repr(loss),
                        "head_config": repr(self.gate._EXPECTED_HEAD_CONFIG),
                        "train_image_size": "(448, 544)",
                        "validation_image_size": "(448, 544)",
                        "field_augmentation_profile": "basic",
                        "ddp_find_unused_parameters": "True",
                        "strict_deterministic": "True",
                        "sdpa_backend": "math",
                        "model_checkpoint_sha256": parent_sha,
                        "source_code_sha256": source_sha,
                        "object_data_sha256": object_data_sha,
                        "person_data_sha256": person_data_sha,
                        "field_data_sha256": field_data_sha,
                        "coco_robot_negative_manifest_sha256": (
                            coco_manifest_sha
                        ),
                        "dhrp_person_negative_manifest_sha256": (
                            self.gate._DHRP_TRAIN_MANIFEST_SHA256
                        ),
                    },
                }
                checkpoint = output / "last.pt"
                classifier_value = 3.0 + forward
                child_state = {
                    **parent_state,
                    "detector.core.model.decoder.class_embed.0.weight": (
                        torch.tensor([classifier_value])
                    ),
                    "detector.core.class_embed.0.weight": torch.tensor(
                        [classifier_value]
                    ),
                }
                torch.save(
                    {
                        "stage": "cross_negative_classifier_only",
                        "global_step": 20,
                        "world_size": 2,
                        "loss_config": loss,
                        "training_config": config,
                        "model": child_state,
                        "ema": {"module": child_state},
                        "inference_model": child_state,
                    },
                    checkpoint,
                )
                (output / "run_config.json").write_text(
                    json.dumps(config),
                    encoding="utf-8",
                )
                epoch = {
                    "train/epoch/cross_negative/person_records_seen": 160,
                    "train/epoch/cross_negative/robot_records_seen": 160,
                    "train/epoch/cross_negative/person_robot_reviewed_seen": 150,
                    "train/epoch/cross_negative/person_robot_verified_seen": 140,
                    "train/epoch/cross_negative/person_robot_eligible_seen": 140,
                    "train/epoch/cross_negative/person_robot_excluded_seen": 10,
                    "train/epoch/cross_negative/person_robot_unreviewed_seen": 10,
                    "train/epoch/cross_negative/robot_person_verified_seen": 50,
                    "train/epoch/cross_negative/robot_person_eligible_seen": 40,
                    "train/epoch/cross_negative/person_robot_detector_applied_records": applied,
                    "train/epoch/cross_negative/robot_person_detector_applied_records": 0,
                    "train/epoch/cross_negative/person_robot_visibility_applied_records": 0,
                    "train/epoch/cross_negative/robot_person_visibility_applied_records": 0,
                }
                records = [
                    {
                        "train/step/task": "person_pose",
                        "system/gpu_memory_reserved_bytes": 10 * 1024**3,
                    },
                    {"train/step/task": "robot_pose"},
                    epoch,
                ]
                (output / "metrics.jsonl").write_text(
                    "".join(json.dumps(record) + "\n" for record in records),
                    encoding="utf-8",
                )
                return checkpoint

            control = write_arm("a00-forward-control", 0.0, 0.0)
            candidate = write_arm("a10-forward", 0.25, 140.0)
            decision = root / "decision.json"
            result = subprocess.run(
                [
                    sys.executable,
                    str(
                        Path(__file__).parents[1]
                        / "runs/reports/dfine-mt-object-field-7d-v1"
                        / "detector_cross_negative_gate.py"
                    ),
                    "--training-only",
                    "--expected-mode",
                    "smoke",
                    "--pair",
                    "forward",
                    "--run-id",
                    "rid",
                    "--attempt-id",
                    "attempt-strict-01",
                    "--determinism-mode",
                    "strict",
                    "--seed",
                    "17",
                    "--parent-checkpoint",
                    str(parent),
                    "--parent-sha256",
                    parent_sha,
                    "--control-checkpoint",
                    str(control),
                    "--candidate-checkpoint",
                    str(candidate),
                    "--coco-train-manifest-sha256",
                    coco_manifest_sha,
                    "--coco-train-records",
                    "56599",
                    "--coco-train-reviewed-records",
                    "4927",
                    "--coco-train-eligible-records",
                    "4900",
                    "--coco-train-excluded-records",
                    "27",
                    "--coco-train-target-fingerprint",
                    coco_fingerprint,
                    "--coco-val-image-fingerprint",
                    coco_val_fingerprint,
                    "--source-code-sha256",
                    source_sha,
                    "--object-data-sha256",
                    object_data_sha,
                    "--person-data-sha256",
                    person_data_sha,
                    "--field-data-sha256",
                    field_data_sha,
                    "--output",
                    str(decision),
                ],
                check=False,
                capture_output=True,
                text=True,
            )

            report = json.loads(decision.read_text())

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(report["passes_all_gates"], report["failed_gates"])
        self.assertEqual(report["decision"], "pass_smoke_forward")


if __name__ == "__main__":
    unittest.main()
