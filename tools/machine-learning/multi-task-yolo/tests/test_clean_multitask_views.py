"""Tests for immutable exact-content-clean dataset wrappers."""

import shutil
import tempfile
import unittest
from pathlib import Path
from typing import cast
from unittest.mock import patch

import yaml
from PIL import Image

from ultralytics_dfine.data import DFINEDataset
from utils.prepare_clean_multitask_views import prepare_clean_views
from validation.multitask_data import sha256_file


def _write_image(path: Path, color: tuple[int, int, int]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    Image.new("RGB", (8, 8), color).save(path)


def _write_label(path: Path, row: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(row + "\n", encoding="utf-8")


def _write_yaml(path: Path, values: object) -> None:
    path.write_text(yaml.safe_dump(values), encoding="utf-8")


class CleanMultiTaskViewsTest(unittest.TestCase):
    def test_filters_training_hashes_and_preserves_field_links(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            objects = root / "objects"
            object_train = objects / "images/train/leak.png"
            object_leak = objects / "images/val/leak-copy.png"
            object_keep_a = objects / "images/val/keep-a.png"
            object_keep_b = objects / "images/val/keep-b.png"
            _write_image(object_train, (255, 0, 0))
            object_leak.parent.mkdir(parents=True)
            shutil.copy2(object_train, object_leak)
            _write_image(object_keep_a, (0, 255, 0))
            shutil.copy2(object_keep_a, object_keep_b)
            for image, row in (
                (object_train, "0 0.5 0.5 0.2 0.2"),
                (object_leak, "0 0.5 0.5 0.2 0.2"),
                (object_keep_a, "0 0.4 0.5 0.2 0.2"),
                (object_keep_b, "0 0.6 0.5 0.2 0.2"),
            ):
                _write_label(
                    Path(
                        str(image).replace("/images/", "/labels/")
                    ).with_suffix(".txt"),
                    row,
                )
            canonical = root / "canonical.yaml"
            refresh = root / "refresh.yaml"
            object_values = {
                "path": str(objects),
                "train": "images/train",
                "val": "images/val",
                "names": ["Ball"],
            }
            _write_yaml(canonical, object_values)
            _write_yaml(refresh, object_values)

            person = root / "person"
            _write_image(person / "images/train/person.png", (0, 0, 255))
            _write_image(person / "images/val/person.png", (0, 0, 0))
            person_data = root / "person.yaml"
            _write_yaml(
                person_data,
                {
                    "path": str(person),
                    "train": "images/train",
                    "val": "images/val",
                    "names": ["person"],
                },
            )

            field = root / "field"
            field_train = field / "images/train/leak-field.png"
            field_leak = field / "images/val/leak-field.png"
            field_keep = field / "images/val/keep-field.png"
            for link, target in (
                (field_train, object_train),
                (field_leak, object_leak),
            ):
                link.parent.mkdir(parents=True, exist_ok=True)
                link.symlink_to(target)
            _write_image(root / "field-source.png", (255, 255, 0))
            field_keep.symlink_to(root / "field-source.png")
            for image in (field_train, field_leak, field_keep):
                _write_label(
                    Path(
                        str(image).replace("/images/", "/labels/")
                    ).with_suffix(".txt"),
                    "0 0.5 0.5 0.2 0.2 0.5 0.6",
                )
            field_data = root / "field.yaml"
            _write_yaml(
                field_data,
                {
                    "path": str(field),
                    "train": "images/train",
                    "val": "images/val",
                    "kpt_shape": [1, 2],
                    "names": ["GoalPost"],
                },
            )

            destination = root / "clean"
            manifest = prepare_clean_views(
                destination,
                canonical,
                refresh,
                person_data,
                field_data,
                None,
                workers=2,
                robot_train_images=(),
                robot_validation_images=(),
            )
            self.assertEqual(manifest["version"], 2)

            object_clean = (
                (destination / "object-val-clean.txt")
                .read_text(encoding="utf-8")
                .splitlines()
            )
            object_unique = (
                (destination / "object-val-clean-unique.txt")
                .read_text(encoding="utf-8")
                .splitlines()
            )
            field_clean = (
                (destination / "field-val-clean.txt")
                .read_text(encoding="utf-8")
                .splitlines()
            )
            self.assertEqual(
                object_clean,
                [str(object_keep_a), str(object_keep_b)],
            )
            self.assertEqual(object_unique, [str(object_keep_a)])
            self.assertEqual(field_clean, [str(field_keep)])
            self.assertTrue(Path(field_clean[0]).is_symlink())
            self.assertEqual(
                len(DFINEDataset(destination / "canonical-clean.yaml", "val")),
                2,
            )
            self.assertEqual(
                len(
                    DFINEDataset(
                        destination / "canonical-clean-unique.yaml",
                        "val",
                    )
                ),
                1,
            )
            wrapper_validation_lists = {
                "canonical-clean.yaml": "object-val-clean.txt",
                "canonical-clean-unique.yaml": ("object-val-clean-unique.txt"),
                "refresh-clean.yaml": "object-val-clean.txt",
                "refresh-clean-unique.yaml": "object-val-clean-unique.txt",
                "field-clean.yaml": "field-val-clean.txt",
                "field-clean-unique.yaml": "field-val-clean-unique.txt",
            }
            for (
                wrapper_name,
                validation_list,
            ) in wrapper_validation_lists.items():
                with self.subTest(wrapper=wrapper_name):
                    wrapper = yaml.safe_load(
                        (destination / wrapper_name).read_text(encoding="utf-8")
                    )
                    self.assertEqual(
                        wrapper["val"],
                        str(destination / validation_list),
                    )

            views = cast(dict[str, object], manifest["views"])
            object_view = cast(dict[str, object], views["object"])
            field_view = cast(dict[str, object], views["field"])
            self.assertEqual(object_view["excluded_unique_hashes"], 1)
            self.assertEqual(field_view["excluded_unique_hashes"], 1)
            duplicate_groups = cast(
                list[dict[str, object]],
                object_view["clean_duplicate_groups"],
            )
            self.assertEqual(len(duplicate_groups), 1)
            self.assertTrue(duplicate_groups[0]["label_conflict"])

            artifacts = cast(
                dict[str, dict[str, object]],
                manifest["artifacts"],
            )
            self.assertEqual(
                artifacts["object-val-clean.txt"]["sha256"],
                sha256_file(destination / "object-val-clean.txt"),
            )
            for wrapper_name in wrapper_validation_lists:
                with self.subTest(artifact=wrapper_name):
                    self.assertEqual(
                        artifacts[wrapper_name]["sha256"],
                        sha256_file(destination / wrapper_name),
                    )

            source_hashes = {
                path: sha256_file(path)
                for path in (
                    canonical,
                    refresh,
                    person_data,
                    field_data,
                )
            }
            failed_destination = root / "failed-clean"
            with (
                patch(
                    "utils.prepare_clean_multitask_views._write_yaml",
                    side_effect=RuntimeError("injected wrapper write failure"),
                ),
                self.assertRaisesRegex(
                    RuntimeError,
                    "injected wrapper write failure",
                ),
            ):
                prepare_clean_views(
                    failed_destination,
                    canonical,
                    refresh,
                    person_data,
                    field_data,
                    None,
                    workers=2,
                    robot_train_images=(),
                    robot_validation_images=(),
                )
            self.assertFalse(failed_destination.exists())
            self.assertEqual(
                source_hashes,
                {path: sha256_file(path) for path in source_hashes},
            )


if __name__ == "__main__":
    unittest.main()
