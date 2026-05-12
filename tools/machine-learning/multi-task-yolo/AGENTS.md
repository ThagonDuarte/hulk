# AGENTS.md

## Scope

- This file is for `tools/machine-learning/multi-task-yolo` only.
- The git root is `../../../hulk`, but commands here should usually run from this directory.

## Environment and Tooling

- Python is pinned to `3.13` (`.python-version`) and `pyproject.toml` requires `>=3.13`.
- Dependencies are managed with `uv` (`uv.lock` is present). Prefer `uv run ...` over bare `python`/`pip`.
- Lint config is in `ruff.toml` (line length `80`, broad strict rule set, tests get limited `S101`/`S603` ignores).
- No `pytest`/`pyrightconfig`/`pre-commit`/CI config exists in this project directory, but `pyright` is available via the `dev` dependency group in `pyproject.toml`.

## Reliable Commands

- Lint: `uv run ruff check src`
- Format: `uv run ruff format src`
- Validation CLI help: `uv run -m validation.validator --help`
- Compare validation runs help: `uv run -m validation.compare_results --help`
- ONNX export help: `uv run -m utils.export_yolo_to_onnx --help`
- Hydra ONNX/TorchScript export help: `uv run -m utils.export_hydra --help`
- Single-task training CLI help: `uv run python src/model/train.py --help`
- Backbone-swap CLI help: `uv run python src/model/cross.py --help`
- Joint training CLI help: `uv run python src/model/joint_train.py --help`
- Run the joint-training test suite: `uv run pytest tests/`

## Code Layout (what actually runs)

- `src/model/hydra.py`: core multi-head model assembly (shared backbone + task heads).
- `src/model/cross.py`: argparse CLI that swaps the backbone of one YOLO checkpoint into a head model from another YOLO checkpoint and saves the combined model.
- `src/validation/validator.py`: main validation pipeline; can validate original models and Hydra heads; writes `metrics.json`, `metadata.json`, and `config.json` under `runs/val/...`.
- `src/validation/compare_results.py`: compares two saved validation run directories and emits a comparison report JSON.
- `src/utils/export_yolo_to_onnx.py`: Click CLI that wraps NV12 preprocessing and exports a single YOLO model to ONNX.
- `src/utils/export_hydra.py`: Click CLI that wraps a `Hydra` model in `HydraWrapper` / `HydraNv12Wrapper` (optionally prepending NV12 preprocessing) and exports it to ONNX or TorchScript.
- `src/utils/model_naming.py`: defines `TaskType` (object/pose/segmentation) and `ModelName` helpers, including ONNX `output_specs()` / `output_names()` used by exports.
- `src/utils/nv12_to_rgb.py`: `NV12ToRgb` `nn.Module` that converts NV12 byte tensors to RGB, with optional chroma subsample mode.
- `src/model/joint_train.py`: Click CLI for joint multi-task finetuning of a Hydra model. Drives the engine in `src/model/joint_loop/`.
- `src/model/joint_loop/`: package hosting the joint training engine (`loop.py`, `dataloaders.py`, `criteria.py`, `weighting.py`, `optim.py`, `validation.py`, `checkpoints.py`).

## Repo-Specific Gotchas

- Use `uv run -m ...` for Python module entrypoints in this repo.
- `src/model/train.py` is a Click CLI for finetuning a single task model with configurable paths/flags.
- `src/model/train.py` `--device` expects a comma-separated list (for example `--device 0,1`) and is parsed to `list[int]`.
- `src/model/train.py` defaults to full training settings; dev settings run only when `--dev-mode` is provided.
- `src/model/train.py` tuning runs only when `--do-tuning` is provided.
- `src/model/cross.py` is argparse-based (unlike most other entrypoints, which use Click) and defaults to `assets/yolo26m.pt` for the backbone and `assets/yolo26m-pose.pt` for both the head and the output, so it will overwrite the head checkpoint in-place if `--output` is left as default.
- `src/utils/export_hydra.py` infers task selection from filename prefixes via `utils.model_naming.ModelName.task_type()` (`yolo26m-pose` → pose, `yolo26m-seg` → segmentation, otherwise object detection), so the model filename matters for export wiring.
- Large/generated artifacts are intentionally ignored (`runs/`, `assets/datasets/`, `assets/output/`, most `*.pt` weights).
- `src/model/joint_train.py` is a separate engine from `src/model/train.py`; it owns a custom PyTorch loop with synchronized cross-task gradient accumulation. It deliberately does not use `YOLO().train()`.
- `Hydra.forward()` flattens head outputs into a `<task>_output` dict for export/inference; for training, use `Hydra.run_backbone()` + `Hydra.run_head()` (raw, non-flattened) which `E2ELoss.parse_output()` expects.
- The MuSGD cv3/proto LR boost regex in `model.joint_loop.optim.build_param_groups` is parameterized by head-last-layer index, not hardcoded to `23` — non-yolo26m scales work transparently.
- `tests/` is run via `uv run pytest tests/`; `pyproject.toml` includes a `[tool.pytest.ini_options]` block with `pythonpath = ["src"]`.
