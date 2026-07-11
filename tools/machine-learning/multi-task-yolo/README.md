# multi-task-yolo

Utilities for building, validating, and exporting Hydra-style YOLO and D-FINE
models. A Hydra model shares one family-compatible backbone across task heads.

## Requirements

- Python `3.13`
- `uv` for dependency management and command execution

## Setup

From this directory (`tools/machine-learning/multi-task-yolo`):

```bash
uv sync
```

This project uses `uv run ...` for all commands.

## Project layout

- `src/model/hydra.py`: Hydra model assembly (shared backbone + per-task heads).
- `src/model/train.py`: Click CLI for single-task YOLO tuning/training.
- `src/model/train_dfine.py`: distributed D-FINE-S training CLI.
- `src/ultralytics_dfine/`: external D-FINE model-family plugin.
- `src/validation/validator.py`: validation pipeline for original models and Hydra heads.
- `src/validation/compare_results.py`: compare two saved validation runs.
- `src/validation/predictor.py`: local smoke predictor/visualizer for detection + pose.
- `src/utils/export_yolo_to_onnx.py`: export a single YOLO checkpoint with NV12 preprocessing.
- `src/utils/export_hydra.py`: export Hydra models to ONNX or TorchScript (optional NV12 layer).
- `src/utils/model_complexity.py`: report checkpoint parameters, MACs, FLOPs, and file size.
- `src/utils/nv12_to_rgb.py`: NV12-to-RGB layer used by export wrappers.

## Common commands

```bash
# Lint
uv run ruff check src

# Format
uv run ruff format src

# Training CLI help
uv run python src/model/train.py --help

# D-FINE training CLI help
uv run -m model.train_dfine --help

# Validation CLI help
uv run -m validation.validator --help

# Compare two validation runs
uv run -m validation.compare_results --help

# Single-task ONNX export help
uv run -m utils.export_yolo_to_onnx --help

# Hydra export help
uv run -m utils.export_hydra --help

# Model complexity help
uv run -m utils.model_complexity --help
```

## Hydra model names

Hydra names use `BACKBONE=fN+HEAD[+HEAD...]`.

- YOLO example: `yolo26m=f11+yolo26m+yolo26m-pose`
- D-FINE example: `dfine-s=f1+dfine-s`
- Fine-tuned D-FINE example:
  `dfine-s=f1+dfine-s~hslvision-132e`

Family compatibility is validated while parsing. YOLO backbones accept only
YOLO heads. D-FINE backbones currently accept exactly one D-FINE detection
head and use `f1` as the semantic HGNetv2 split. D-FINE pose/segmentation heads
and cross-family head transplantation are intentionally unsupported.

## D-FINE plugin

The plugin uses the pinned converted official D-FINE-S HGNetv2-B0 checkpoint
and replaces the Transformers training loss assembly with the official D-FINE
criterion contract. It includes GO-LSD union matching, FDR/FGL, DDF,
preliminary, encoder, decoder-auxiliary, and contrastive-denoising losses.
Inference uses NMS-free flattened query/class top-k selection.

The normal plugin output is `[B, 300, 6]` normalized
`[cx, cy, width, height, score, class]`. Hydra deployment converts this to the
existing `object_output` `[B, 300, 6]` pixel
`[x_min, y_min, x_max, y_max, score, class]` contract.

```python
from ultralytics_dfine import DFINE

model = DFINE("dfine-s")
model.train(
    data="assets/datasets/hslvision_yolo/data.yaml",
    output_dir="runs/train/dfine-s=f1+dfine-s~experiment",
)
metrics = model.val(data="assets/datasets/hslvision_yolo/data.yaml")
model.predict("images", output_dir="runs/predict/dfine-s")
model.export("runs/export/dfine-s.onnx")
```

Official 132-epoch distributed training:

```bash
CUDA_VISIBLE_DEVICES=0,1 torchrun --standalone --nproc_per_node=2 \
  -m model.train_dfine \
  --model dfine-s \
  --data assets/datasets/hslvision_yolo/data.yaml \
  --output-dir runs/train/dfine-s=f1+dfine-s~hslvision-132e \
  --epochs 132 \
  --transition-epoch 120 \
  --batch 32 \
  --no-amp
```

`--no-amp` is the safe setting on hardware where FP16 Hungarian/FGL training
produces non-finite gradients. Finite output, loss, and gradient checks abort
rather than silently skipping corrupt steps. Training writes resumable current
and EMA checkpoints, pure state dicts, manifests, complete loss JSONL, rendered
validation predictions, and W&B media every epoch.

Validate and export through the Hydra schema:

```bash
uv run -m validation.validator \
  --hydra_model_name dfine-s=f1+dfine-s \
  --object_dataset_name hslvision_yolo/data.yaml \
  --device cuda

uv run -m utils.export_hydra \
  dfine-s=f1+dfine-s~hslvision-132e \
  runs/export/hydra \
  --runs_dir runs \
  --imgsz 640 \
  --opset 17
```

## Single-task training (`src/model/train.py`)

Finetunes one YOLO model and can optionally run hyperparameter tuning first.

- `--project-tune-dir` and `--project-train-dir` override output roots.
- If omitted, output roots resolve from `--repo-root` to
  `<repo-root>/runs/tune` and `<repo-root>/runs/train`.
- `--device` accepts a comma-separated list and is parsed to `list[int]`
  (example: `--device 0,1`).
- `--dev-mode` is opt-in and switches to short development settings.
- `--do-tuning` is opt-in and runs `model.tune()` before `model.train()`.
- `--use-tuned-hyperparameters` loads
  `runs/tune/<tuning-folder-name>/best_hyperparameters.yaml`.

Examples:

```bash
# Default training
uv run python src/model/train.py

# Fast development run
uv run python src/model/train.py --dev-mode

# Multi-GPU training
uv run python src/model/train.py --device 0,1

# Tune then train
uv run python src/model/train.py --do-tuning
```

## Validation (`src/validation/validator.py`)

Runs Ultralytics validation for Hydra heads and can optionally validate the
original source checkpoints first.

- Default checkpoints:
  - `--backbone assets/yolo26m.pt`
  - `--detection-model assets/yolo26m.pt`
  - `--pose-model assets/yolo26m-pose.pt`
- Default datasets:
  - `--detection-data assets/datasets/coco.yaml`
  - `--pose-data assets/datasets/coco-pose.yaml`
- `--validate-original` enables baseline validation of the original task
  models before multi-task validation.

Example:

```bash
uv run -m validation.validator \
  --backbone assets/yolo26m.pt \
  --detection-model assets/yolo26m.pt \
  --pose-model assets/yolo26m-pose.pt \
  --detection-data assets/datasets/coco.yaml \
  --pose-data assets/datasets/coco-pose.yaml \
  --validate-original
```

Validation outputs are saved under `runs/val/...` and include:

- `metrics.json`
- `metadata.json`
- `config.json`

## Compare validation runs (`src/validation/compare_results.py`)

Compares two saved validation run directories of the same task type and writes
a JSON report.

- Required inputs: `--baseline <run_dir>` and `--candidate <run_dir>`.
- By default, writes output to `<candidate>/comparison.json`.
- Supports `--task auto|detect|pose`, strict config checks via
  `--strict-config`, custom primary metric, and regression threshold.

Example:

```bash
uv run -m validation.compare_results \
  --baseline runs/val/yolo26m \
  --candidate runs/val/yolo26m-pose_yolo26m \
  --task auto
```

## Model complexity (`src/utils/model_complexity.py`)

Reports checkpoint file size, parameter counts, MACs, and FLOPs for YOLO
`.pt` files and assembled Hydra model names. FLOPs use the Ultralytics
convention: `1 MAC = 2 FLOPs`. Checkpoint reports are saved under
`runs/complexity/<checkpoint-name>/report.json`. For Hydra model names, the
assembled model is exported first, then `size MB` is measured from that
exported file. Hydra exports and per-model reports are saved under
`runs/complexity/<model-name>/`.

Examples:

```bash
# Profile one checkpoint
uv run -m utils.model_complexity \
  runs/val/yolo26m=f11+yolo26m/yolo26m=f11+yolo26m.pt

# Profile specific asset checkpoints
uv run -m utils.model_complexity assets \
  --checkpoint-name yolo26m.pt \
  --checkpoint-name yolo26m-pose.pt \
  --checkpoint-name yolo26m-seg.pt

# Scan every checkpoint under runs and save JSON
uv run -m utils.model_complexity runs \
  --imgsz 640 \
  --json-output runs/model_complexity.json

# Only scan training best checkpoints
uv run -m utils.model_complexity runs/train \
  --checkpoint-name best.pt

# Profile an assembled Hydra model by name
uv run -m utils.model_complexity \
  --hydra-model-name yolo26m=f11+yolo26m+yolo26m-pose
```

## Export utilities

### Export single YOLO to ONNX (`src/utils/export_yolo_to_onnx.py`)

Wraps a YOLO checkpoint with an NV12 preprocessing layer and exports ONNX.

```bash
uv run -m utils.export_yolo_to_onnx \
  assets/yolo26m.pt \
  assets/output/yolo26m-nv12.onnx
```

Use `--subsample` to enable chroma subsampling behavior in the wrapper.

### Export Hydra model (`src/utils/export_hydra.py`)

Builds a Hydra model from a backbone checkpoint plus one or more heads,
then exports ONNX (`--format onnx`) or TorchScript (`--format pt`).

- Repeat `--head NAME=MODEL_PATH` for each task head.
- Optional `--with-nv12-layer` prepends NV12 preprocessing before export.
- When `--with-nv12-layer` is enabled, `--imgsz` must be even.

Examples:

```bash
# ONNX export
uv run -m utils.export_hydra \
  assets/yolo26m.pt \
  --head detection=assets/yolo26m.pt \
  --head pose=assets/yolo26m-pose.pt \
  assets/output/hydra.onnx

# TorchScript export with NV12 input wrapper
uv run -m utils.export_hydra \
  assets/yolo26m.pt \
  --head detection=assets/yolo26m.pt \
  --head pose=assets/yolo26m-pose.pt \
  assets/output/hydra-nv12.pt \
  --format pt \
  --with-nv12-layer
```

## Local predictor note

`src/validation/predictor.py` contains a local smoke workflow. Its `main()`
uses a hardcoded example image path under `assets/datasets/...` and is not a
general-purpose CLI entrypoint.
