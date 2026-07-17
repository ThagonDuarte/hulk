import hashlib
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import torch
import yaml
from PIL import Image

from ultralytics_dfine.data import YOLOKeypointDataset
from ultralytics_dfine.schemas import COCO_FLIP_IDX, HeadId


class YOLOKeypointDatasetTests(unittest.TestCase):
    @staticmethod
    def _fixture(
        root: Path,
        *,
        names: list[str],
        label: str,
    ) -> Path:
        (root / "images" / "train").mkdir(parents=True)
        (root / "labels" / "train").mkdir(parents=True)
        Image.new("RGB", (80, 40), color="white").save(
            root / "images" / "train" / "image.jpg"
        )
        (root / "labels" / "train" / "image.txt").write_text(
            label,
            encoding="utf-8",
        )
        data = root / "data.yaml"
        data.write_text(
            yaml.safe_dump(
                {
                    "path": str(root),
                    "names": names,
                    "train": "images/train",
                    "val": "images/train",
                }
            ),
            encoding="utf-8",
        )
        return data

    @staticmethod
    def _robot_negative_manifest(
        root: Path,
        data: Path,
        *,
        role: str = "train_negative",
        eligible: tuple[str, ...] = ("images/train/image.jpg",),
        excluded: tuple[str, ...] = (),
    ) -> Path:
        digest = hashlib.sha256()
        digest.update(b"keypoint-robot-negative-population-v1\0")
        digest.update(b"train\0")
        images = sorted((root / "images" / "train").glob("*.jpg"))
        image_sha = {}
        for image in images:
            relative_path = image.relative_to(root).as_posix()
            value = hashlib.sha256(image.read_bytes()).hexdigest()
            image_sha[relative_path] = value
            digest.update(f"{relative_path}\0{value}\n".encode())
        eligible_path = root / "eligible.jsonl"
        eligible_path.write_text(
            "".join(
                json.dumps(
                    {
                        "relative_path": path,
                        "image_sha256": image_sha[path],
                        "review": "manual fixture approval",
                    }
                )
                + "\n"
                for path in eligible
            ),
            encoding="utf-8",
        )
        excluded_reference = None
        if excluded:
            excluded_path = root / "excluded.jsonl"
            excluded_path.write_text(
                "".join(
                    json.dumps(
                        {
                            "relative_path": path,
                            "image_sha256": image_sha[path],
                            "reason": "fixture robot or ambiguity",
                        }
                    )
                    + "\n"
                    for path in excluded
                ),
                encoding="utf-8",
            )
            excluded_reference = {
                "path": excluded_path.name,
                "sha256": hashlib.sha256(
                    excluded_path.read_bytes()
                ).hexdigest(),
                "records": len(excluded),
            }
        split = {
            "role": role,
            "records": len(images),
            "population_sha256": digest.hexdigest(),
            "reviewed_records": len(eligible) + len(excluded),
            "eligible_records": len(eligible),
            "excluded_records": len(excluded),
            "eligible": {
                "path": eligible_path.name,
                "sha256": hashlib.sha256(
                    eligible_path.read_bytes()
                ).hexdigest(),
                "records": len(eligible),
            },
        }
        if excluded_reference is not None:
            split["excluded"] = excluded_reference
        manifest = root / "robot-negative.json"
        manifest.write_text(
            json.dumps(
                {
                    "version": 2,
                    "policy": "explicit-verified-robot-free-keypoint-v2",
                    "population_digest_algorithm": (
                        "sha256(keypoint-robot-negative-population-v1\\0 + "
                        "split + \\0 + ordered(relative_path + \\0 + "
                        "image_sha256 + \\n))"
                    ),
                    "dataset_yaml_sha256": hashlib.sha256(
                        data.read_bytes()
                    ).hexdigest(),
                    "splits": {"train": split},
                }
            ),
            encoding="utf-8",
        )
        return manifest

    def test_person_rows_map_to_global_person_class(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            points = " ".join("0.25 0.5 2" for _ in range(17))
            data = self._fixture(
                root,
                names=["person"],
                label=f"0 0.5 0.5 0.4 0.8 {points}\n",
            )
            dataset = YOLOKeypointDataset(
                data,
                "train",
                schema_id=HeadId.PERSON_POSE,
                keypoint_count=17,
                flip_idx=COCO_FLIP_IDX,
                global_class_ids=(7,),
                horizontal_flip_probability=0,
            )

            image, target = dataset[0]

        self.assertEqual(image.shape, (3, 640, 640))
        self.assertEqual(target["labels"].tolist(), [7])
        keypoints = target.get("keypoints")
        valid_classes = target.get("valid_detection_classes")
        self.assertIsInstance(keypoints, torch.Tensor)
        self.assertIsInstance(valid_classes, torch.Tensor)
        assert isinstance(keypoints, torch.Tensor)
        assert isinstance(valid_classes, torch.Tensor)
        self.assertEqual(keypoints.shape, (1, 17, 3))
        self.assertEqual(valid_classes.sum().item(), 1)
        self.assertFalse(target["robot_negative_verified"])
        self.assertFalse(target["robot_negative_eligible"])
        self.assertFalse(target["robot_negative_reviewed"])
        self.assertFalse(target["robot_negative_excluded"])

    def test_robot_negative_manifest_marks_exact_population(self) -> None:
        points = " ".join("0.25 0.5 2" for _ in range(17))
        for role, excluded, reviewed, verified, eligible in (
            ("train_negative", (), True, True, True),
            ("primary_evaluation", (), True, True, False),
            (
                "train_negative",
                ("images/train/image.jpg",),
                True,
                False,
                False,
            ),
        ):
            with (
                self.subTest(role=role, excluded=excluded),
                tempfile.TemporaryDirectory() as directory,
            ):
                root = Path(directory)
                data = self._fixture(
                    root,
                    names=["person"],
                    label=f"0 0.5 0.5 0.4 0.8 {points}\n",
                )
                manifest = self._robot_negative_manifest(
                    root,
                    data,
                    role=role,
                    eligible=() if excluded else ("images/train/image.jpg",),
                    excluded=excluded,
                )
                dataset = YOLOKeypointDataset(
                    data,
                    "train",
                    schema_id=HeadId.PERSON_POSE,
                    keypoint_count=17,
                    flip_idx=COCO_FLIP_IDX,
                    global_class_ids=(7,),
                    horizontal_flip_probability=0,
                    robot_negative_manifest=manifest,
                )
                _, target = dataset[0]

                self.assertEqual(target["robot_negative_reviewed"], reviewed)
                self.assertEqual(target["robot_negative_verified"], verified)
                self.assertEqual(target["robot_negative_eligible"], eligible)
                self.assertEqual(
                    target["robot_negative_excluded"], bool(excluded)
                )
                self.assertEqual(dataset.robot_negative_reviewed_records, 1)
                self.assertEqual(
                    dataset.robot_negative_verified_records,
                    int(verified),
                )
                self.assertEqual(
                    dataset.robot_negative_eligible_records,
                    int(eligible),
                )
                self.assertEqual(
                    dataset.robot_negative_excluded_records,
                    int(bool(excluded)),
                )

    def test_unlisted_manifest_record_remains_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            points = " ".join("0.25 0.5 2" for _ in range(17))
            label = f"0 0.5 0.5 0.4 0.8 {points}\n"
            data = self._fixture(root, names=["person"], label=label)
            Image.new("RGB", (80, 40), color="gray").save(
                root / "images" / "train" / "unreviewed.jpg"
            )
            (root / "labels" / "train" / "unreviewed.txt").write_text(
                label,
                encoding="utf-8",
            )
            manifest = self._robot_negative_manifest(root, data)
            dataset = YOLOKeypointDataset(
                data,
                "train",
                schema_id=HeadId.PERSON_POSE,
                keypoint_count=17,
                flip_idx=COCO_FLIP_IDX,
                global_class_ids=(7,),
                horizontal_flip_probability=0,
                robot_negative_manifest=manifest,
            )
            index = next(
                index
                for index, path in enumerate(dataset.images)
                if path.name == "unreviewed.jpg"
            )
            _, target = dataset[index]

        self.assertFalse(target["robot_negative_reviewed"])
        self.assertFalse(target["robot_negative_verified"])
        self.assertFalse(target["robot_negative_eligible"])
        self.assertFalse(target["robot_negative_excluded"])
        self.assertEqual(dataset.robot_negative_reviewed_records, 1)
        self.assertEqual(dataset.robot_negative_eligible_records, 1)
        self.assertEqual(dataset.robot_negative_unreviewed_records, 1)

    def test_robot_negative_manifest_rejects_changed_image(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            points = " ".join("0.25 0.5 2" for _ in range(17))
            data = self._fixture(
                root,
                names=["person"],
                label=f"0 0.5 0.5 0.4 0.8 {points}\n",
            )
            manifest = self._robot_negative_manifest(root, data)
            Image.new("RGB", (80, 40), color="black").save(
                root / "images" / "train" / "image.jpg"
            )

            with self.assertRaisesRegex(ValueError, "population SHA-256"):
                YOLOKeypointDataset(
                    data,
                    "train",
                    schema_id=HeadId.PERSON_POSE,
                    keypoint_count=17,
                    flip_idx=COCO_FLIP_IDX,
                    global_class_ids=(7,),
                    robot_negative_manifest=manifest,
                )

    def test_robot_negative_manifest_rejects_changed_yaml(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            points = " ".join("0.25 0.5 2" for _ in range(17))
            data = self._fixture(
                root,
                names=["person"],
                label=f"0 0.5 0.5 0.4 0.8 {points}\n",
            )
            manifest = self._robot_negative_manifest(root, data)
            data.write_text(data.read_text() + "# changed\n", encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "YAML SHA-256"):
                YOLOKeypointDataset(
                    data,
                    "train",
                    schema_id=HeadId.PERSON_POSE,
                    keypoint_count=17,
                    flip_idx=COCO_FLIP_IDX,
                    global_class_ids=(7,),
                    robot_negative_manifest=manifest,
                )

    def test_legacy_complement_manifest_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            points = " ".join("0.25 0.5 2" for _ in range(17))
            data = self._fixture(
                root,
                names=["person"],
                label=f"0 0.5 0.5 0.4 0.8 {points}\n",
            )
            manifest = root / "legacy-complement.json"
            manifest.write_text(
                json.dumps(
                    {
                        "version": 1,
                        "policy": "verified-robot-free-keypoint-v1",
                        "dataset_yaml_sha256": hashlib.sha256(
                            data.read_bytes()
                        ).hexdigest(),
                        "splits": {
                            "train": {
                                "role": "train_negative",
                                "records": 1,
                                "eligible_records": 1,
                                "excluded": [],
                            }
                        },
                    }
                ),
                encoding="utf-8",
            )

            with self.assertRaisesRegex(ValueError, "manifest version"):
                YOLOKeypointDataset(
                    data,
                    "train",
                    schema_id=HeadId.PERSON_POSE,
                    keypoint_count=17,
                    flip_idx=COCO_FLIP_IDX,
                    global_class_ids=(7,),
                    robot_negative_manifest=manifest,
                )

    def test_robot_negative_manifest_requires_eligible_allowlist(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            points = " ".join("0.25 0.5 2" for _ in range(17))
            data = self._fixture(
                root,
                names=["person"],
                label=f"0 0.5 0.5 0.4 0.8 {points}\n",
            )
            manifest = self._robot_negative_manifest(root, data)
            values = json.loads(manifest.read_text(encoding="utf-8"))
            del values["splits"]["train"]["eligible"]
            manifest.write_text(json.dumps(values), encoding="utf-8")

            with self.assertRaisesRegex(
                TypeError, "eligible must be an object"
            ):
                YOLOKeypointDataset(
                    data,
                    "train",
                    schema_id=HeadId.PERSON_POSE,
                    keypoint_count=17,
                    flip_idx=COCO_FLIP_IDX,
                    global_class_ids=(7,),
                    robot_negative_manifest=manifest,
                )

    def test_robot_negative_manifest_rejects_tampered_allowlist(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            points = " ".join("0.25 0.5 2" for _ in range(17))
            data = self._fixture(
                root,
                names=["person"],
                label=f"0 0.5 0.5 0.4 0.8 {points}\n",
            )
            manifest = self._robot_negative_manifest(root, data)
            (root / "eligible.jsonl").write_text(
                '{"relative_path":"images/train/image.jpg"}\n',
                encoding="utf-8",
            )

            with self.assertRaisesRegex(ValueError, "list SHA-256 mismatch"):
                YOLOKeypointDataset(
                    data,
                    "train",
                    schema_id=HeadId.PERSON_POSE,
                    keypoint_count=17,
                    flip_idx=COCO_FLIP_IDX,
                    global_class_ids=(7,),
                    robot_negative_manifest=manifest,
                )

    def test_robot_negative_manifest_rejects_overlapping_decisions(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            points = " ".join("0.25 0.5 2" for _ in range(17))
            data = self._fixture(
                root,
                names=["person"],
                label=f"0 0.5 0.5 0.4 0.8 {points}\n",
            )
            manifest = self._robot_negative_manifest(
                root,
                data,
                eligible=("images/train/image.jpg",),
                excluded=("images/train/image.jpg",),
            )

            with self.assertRaisesRegex(ValueError, "lists overlap"):
                YOLOKeypointDataset(
                    data,
                    "train",
                    schema_id=HeadId.PERSON_POSE,
                    keypoint_count=17,
                    flip_idx=COCO_FLIP_IDX,
                    global_class_ids=(7,),
                    robot_negative_manifest=manifest,
                )

    def test_robot_negative_manifest_binds_reviewed_image_bytes(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            points = " ".join("0.25 0.5 2" for _ in range(17))
            data = self._fixture(
                root,
                names=["person"],
                label=f"0 0.5 0.5 0.4 0.8 {points}\n",
            )
            manifest = self._robot_negative_manifest(root, data)
            eligible = root / "eligible.jsonl"
            entry = json.loads(eligible.read_text(encoding="utf-8"))
            entry["image_sha256"] = "0" * 64
            eligible.write_text(json.dumps(entry) + "\n", encoding="utf-8")
            values = json.loads(manifest.read_text(encoding="utf-8"))
            values["splits"]["train"]["eligible"]["sha256"] = hashlib.sha256(
                eligible.read_bytes()
            ).hexdigest()
            manifest.write_text(json.dumps(values), encoding="utf-8")

            with self.assertRaisesRegex(
                ValueError,
                "reviewed image SHA-256 mismatch",
            ):
                YOLOKeypointDataset(
                    data,
                    "train",
                    schema_id=HeadId.PERSON_POSE,
                    keypoint_count=17,
                    flip_idx=COCO_FLIP_IDX,
                    global_class_ids=(7,),
                    robot_negative_manifest=manifest,
                )

    def test_field_rows_map_two_dimensional_points_to_schema(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            data = self._fixture(
                root,
                names=["GoalPost", "LSpot", "PenaltySpot", "TSpot", "XSpot"],
                label="2 0.5 0.5 0.2 0.2 0.6 0.7\n",
            )
            dataset = YOLOKeypointDataset(
                data,
                "train",
                schema_id=HeadId.FIELD_FEATURES,
                keypoint_count=1,
                keypoint_dimensions=2,
                flip_idx=(0,),
                global_class_ids=(1, 2, 3, 5, 6),
                horizontal_flip_probability=0,
                point_set=True,
                point_label_ids=(0, 1, 3, 2, 4),
            )

            _, target = dataset[0]

        self.assertEqual(target["labels"].tolist(), [3])
        point_labels = target.get("point_labels")
        points = target.get("points")
        self.assertIsInstance(point_labels, torch.Tensor)
        self.assertIsInstance(points, torch.Tensor)
        assert isinstance(point_labels, torch.Tensor)
        assert isinstance(points, torch.Tensor)
        self.assertEqual(point_labels.tolist(), [3])
        torch.testing.assert_close(
            points,
            torch.tensor([[0.6, 0.7]]),
        )

    def test_rectangular_resize_preserves_normalized_annotations(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            data = self._fixture(
                root,
                names=["GoalPost"],
                label="0 0.5 0.5 0.2 0.4 0.6 0.7\n",
            )
            dataset = YOLOKeypointDataset(
                data,
                "val",
                schema_id=HeadId.FIELD_FEATURES,
                keypoint_count=1,
                keypoint_dimensions=2,
                flip_idx=(0,),
                global_class_ids=(1,),
                image_size=(32, 64),
                point_set=True,
            )

            image, target = dataset[0]

        self.assertEqual(image.shape, (3, 32, 64))
        torch.testing.assert_close(
            target["boxes"],
            torch.tensor([[0.5, 0.5, 0.2, 0.4]]),
        )
        torch.testing.assert_close(
            target["points"],
            torch.tensor([[0.6, 0.7]]),
        )

    def test_field_affine_removes_points_transformed_out_of_frame(self) -> None:
        image = torch.full((3, 40, 80), 255, dtype=torch.uint8)
        labels = torch.tensor([0])
        boxes = torch.tensor([[0.95, 0.5, 0.1, 0.2]])
        keypoints = torch.tensor([[[0.99, 0.5, 1.0]]])
        parameters = [0, 0.08, 0, 1, 0, 0]

        with patch(
            "ultralytics_dfine.data.keypoints.random.uniform",
            side_effect=parameters,
        ):
            _, labels, boxes, keypoints = YOLOKeypointDataset._field_affine(
                image,
                labels,
                boxes,
                keypoints,
            )

        self.assertEqual(labels.numel(), 0)
        self.assertEqual(boxes.shape, (0, 4))
        self.assertEqual(keypoints.shape, (0, 1, 3))


if __name__ == "__main__":
    unittest.main()
