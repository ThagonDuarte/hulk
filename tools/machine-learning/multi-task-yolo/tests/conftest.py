"""Shared pytest fixtures for the multi-task YOLO test suite."""
from __future__ import annotations

from pathlib import Path

import pytest

@pytest.fixture
def tmp_run_dir(tmp_path: Path) -> Path:
    """A temp directory shaped like a joint-train run output."""
    run_dir = tmp_path / "joint_train" / "fake-model~placeholder"
    run_dir.mkdir(parents=True)
    return run_dir
