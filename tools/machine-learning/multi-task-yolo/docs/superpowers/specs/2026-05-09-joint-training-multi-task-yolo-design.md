# Joint Training of Multi-Task YOLO Hydra Models — Design

**Status:** Approved design, pending implementation.
**Date:** 2026-05-09
**Scope:** `tools/machine-learning/multi-task-yolo` only.

## 1. Goal & Non-Goals

### Goal

Add a custom-loop joint trainer that finetunes a single `Hydra` instance (shared
backbone + N task heads) with multiple datasets simultaneously, mirroring
Ultralytics' YOLO26 training recipe (E2ELoss + MuSGD + 4-group param split with
cv3/proto LR boost) per module group. The trainer must integrate with existing
tooling (`validator.py`, `export_hydra.py`, `cross.py`) without breaking any
public API.

### Non-Goals (v1)

- Hyperparameter tuning (stays in `train.py`).
- DDP / multi-node training (single-host with `nn.DataParallel` for multi-GPU).
- Gradient checkpointing.
- Mid-epoch resume (epoch-boundary resume only).
- Per-task batch size / image size knobs (shared globally).
- AdamW-path benchmarking (kept as a CLI knob but not validated in v1).

## 2. Background

`src/model/train.py` uses `YOLO().train()` (Ultralytics' `BaseTrainer`) and
strictly assumes a single dataset YAML, single loss, single optimizer. Hacking
that trainer to support disjoint multi-task datasets would require fragile
overrides. We instead build a parallel `joint_train.py` engine with a custom
PyTorch loop, leaving `train.py` untouched for single-task finetuning and
hyperparameter searches.

## 3. Locked Design Decisions

| # | Decision | Choice |
|---|----------|--------|
| Q1 | CLI shape | Single `--hydra_model_name` (existing `HydraModelName` syntax) + per-task dataset flags (`--object_dataset_name`, `--pose_dataset_name`, `--segmentation_dataset_name`). Active tasks inferred from heads in the HydraModelName. |
| Q2 | Validation cadence + best metric | Validate every epoch via existing `validate_hydra_model()`, one call per task. "Best" = configurable weighted sum of per-task primary metrics (`--task_weight task=value`, default 1.0 each). |
| Q3 | Checkpoint format | Per-task `.pt` files, Ultralytics-compatible (backbone + that head, `cross.py`-style assembly). Existing `validator.py` and `export_hydra.py` consume them unchanged. |
| Q4 | Per-task hyperparameters | Shared globally: one `--batch`, one `--imgsz`, one `--workers`. |
| Q5 | LR strategy | Three explicit CLI knobs: `--lr_backbone`, `--lr_heads`, `--lr_logvar`. Cosine schedule per optimizer with shared linear warmup. |
| Q6 | Recipe faithfulness | Full mirror of YOLO26: `E2ELoss(adapter, inner_cls)` when head is end2end (with `PoseLoss26` for pose, `v8SegmentationLoss` for seg, `v8DetectionLoss` for detect); v8XxxLoss otherwise. `MuSGD` optimizer with 4-group param split + cv3/proto x3 boost replicated per module. |
| Q7 | Run output path | `runs/joint_train/<hydra_model>~<wordlet>/<task>/{best,last}.pt`. Wordlet from `wonderwords.RandomWord()` (already a dependency); same string used as the W&B run name. |

## 4. Architecture

### 4.1 New files

- `src/model/joint_train.py` — Click CLI shell. Parses args, builds `Hydra`,
  builds dataloaders, calls into the engine, saves per-task checkpoints. Owns
  no math.
- `src/model/joint_loop/__init__.py`
- `src/model/joint_loop/dataloaders.py` — `InterleavedTaskDataloader` plus
  per-task dataloader builders that wrap `ultralytics.data.build.build_yolo_dataset`
  + `ultralytics.data.build.build_dataloader`.
- `src/model/joint_loop/criteria.py` — `_HydraHeadAdapter`, `build_criterion`
  dispatcher (E2ELoss vs v8XxxLoss based on `hydra.head_end2end[task]`),
  `epoch_update(criteria)` for E2ELoss decay.
- `src/model/joint_loop/weighting.py` — `UncertaintyWeighter` `nn.Module` with
  one learnable log-variance per task.
- `src/model/joint_loop/optim.py` — `build_param_groups(...)` (port of
  `BaseTrainer.build_optimizer` semantics: 4-group split + cv3/proto regex),
  `build_joint_optimizers(...)`, `build_schedulers(...)`, `EMAHydra`,
  `make_amp_scaler()`, `clip_backbone_grads()`.
- `src/model/joint_loop/validation.py` — Per-epoch hook calling existing
  `validate_hydra_model()` per task on EMA weights, returning
  `(weighted_score, per_task_metrics)`.
- `src/model/joint_loop/checkpoints.py` — Per-task `.pt` writer
  (cross-style assembly), `joint_state.json` writer/reader, `config.json`
  writer.
- `src/model/joint_loop/loop.py` — `train_joint(...)` function holding the
  synchronized accumulation, AMP, gradient clipping, EMA update, scheduler
  step, and validation orchestration.
- `tests/joint_loop/test_dataloader.py`
- `tests/joint_loop/test_weighting.py`
- `tests/joint_loop/test_optim.py`
- `tests/joint_loop/test_hydra_split.py`
- `tests/joint_loop/test_loop_smoke.py`

### 4.2 Modified files

- `src/model/hydra.py` — Add `run_backbone(x) -> tuple[Tensor, list[Tensor | None]]`
  and `run_head(task, backbone_activations, y_backbone) -> Any` (returning the
  raw, non-flattened head output). Existing `forward()` is rewritten to call
  the two new methods and re-apply the current dict-flattening behaviour, so
  all existing callers (export, inference) keep behaving identically. Public
  API (constructor signature, attribute names, `forward()` return shape) is
  unchanged.
- `pyproject.toml` (`dev` group) — Add `pytest>=8.3` for the new test surface.

### 4.3 Files explicitly NOT touched

`src/model/train.py`, `src/model/cross.py`, `src/validation/validator.py`,
`src/validation/compare_results.py`, `src/utils/export_hydra.py`,
`src/utils/export_yolo_to_onnx.py`, `src/utils/model_naming.py`,
`src/utils/nv12_to_rgb.py`.

## 5. Data Pipeline

### 5.1 Per-task dataloader

`build_task_dataloader(task, dataset_yaml, imgsz, batch, workers, device, mode)`:

- Loads YAML via `ultralytics.cfg.get_cfg` to produce the standard `args`
  namespace (so augmentation hyperparameters follow Ultralytics defaults).
- Calls `ultralytics.data.build.build_yolo_dataset(args, img_path, batch, data,
  mode, rect, stride)` — Ultralytics picks the right dataset class based on
  `data["task"]`.
- Wraps with `ultralytics.data.build.build_dataloader(dataset, batch, workers,
  shuffle=True, rank=-1)` (`InfiniteDataLoader`).
- Returns `(loader, dataset)` so the caller can read `len(dataset)`.

### 5.2 Interleaved iteration

`InterleavedTaskDataloader(loaders: dict[TaskType, DataLoader], steps_per_epoch: int)`:

- `steps_per_epoch = max(len(loader) for loader in loaders.values())`.
- Iteration yields `steps_per_epoch * len(loaders)` `(task, batch)` pairs per
  epoch in fixed task order (sorted alphabetically by task name for
  determinism). Each per-task iterator is a `cycle()`-like wrapper that
  reseeds and restarts when exhausted, with a debug-level log on each restart.
- `__len__` returns `steps_per_epoch` — one step = one round across all tasks.
- `set_epoch(epoch)` propagates to underlying samplers for shuffle determinism.

### 5.3 Determinism

`--seed` seeds Python/NumPy/Torch and is forwarded to `build_dataloader`.
Restart events are logged at INFO level so they show up in W&B run logs.

## 6. Loss & Cross-Task Weighting

### 6.1 `_HydraHeadAdapter`

A lightweight wrapper around a `Hydra` instance and a `TaskType` exposing
exactly the attributes Ultralytics' loss code reads:

- `.model = hydra.heads[task]` (so `model.model[-1]` resolves to the actual
  task head module — required because `PoseLoss26` reads
  `model.model[-1].flow_model`).
- `.stride = hydra.head_strides[task]`
- `.nc = len(hydra.head_class_names[task])`
- `.kpt_shape = hydra.head_kpt_shapes[task]` (for pose; `None` otherwise)
- `.end2end = hydra.head_end2end[task]`
- `.device` (filled at criterion-build time)
- `.args` and `.hyp`: a `SimpleNamespace` populated from a single source of
  truth — the joint-train config — including `box`, `cls`, `dfl`, `pose`,
  `kobj`, `rle`, `epochs`, plus task-specific gains. Defaults pulled from
  `ultralytics.cfg.default.yaml` and overridable via CLI.

### 6.2 Criterion factory

`build_criterion(hydra, task, hyp) -> Callable`:

- If `hydra.head_end2end[task]`: `E2ELoss(adapter, inner_cls)` where
  `inner_cls` is `PoseLoss26` for pose, `v8SegmentationLoss` for seg,
  `v8DetectionLoss` for detect.
- Else: `v8PoseLoss(adapter)` / `v8SegmentationLoss(adapter)` /
  `v8DetectionLoss(adapter)`.

`epoch_update(criteria: dict[TaskType, Callable])` calls `.update()` on each
criterion that has the method (E2ELoss does; v8XxxLoss doesn't), driving the
one-to-many → one-to-one decay schedule.

### 6.3 `UncertaintyWeighter`

```python
class UncertaintyWeighter(nn.Module):
    def __init__(self, tasks: list[TaskType]) -> None: ...
    log_var: nn.Parameter  # shape (len(tasks),), initial value 0.0 (variance 1.0)
    def weight_single(self, task: TaskType, loss: Tensor) -> Tensor: ...
    def forward(self, losses: dict[TaskType, Tensor]) -> Tensor: ...
```

Implements Kendall–Gal 2018 homoscedastic uncertainty weighting:

```
total = sum_t( exp(-log_var_t) * loss_t + log_var_t )
```

`weight_single` exposes the per-task contribution so the loop can
`backward()` per task without holding all forward graphs simultaneously.

Initial values are zero by default; `--init_log_var task=value` (repeatable)
overrides per task.

### 6.4 Per-step gradient flow

For each `(task, batch)` pair within a synchronized step:

1. Forward backbone → forward task head → call task criterion → scalar
   `loss_t_total` (`result[0]` for E2ELoss; `result[0]` is also the scalar
   total for v8 losses).
2. Stash `loss_t_total.detach()` for logging.
3. `weighted = weighter.weight_single(task, loss_t_total)`.
4. `scaler.scale(weighted).backward()` — grads land in `shared_backbone`,
   `heads[task]`, and `weighter.log_var[task_idx]`. Backbone grads accumulate
   naturally because we do not zero them between tasks within a step.

### 6.5 Logging

Per-step (gated by `--log_interval`, default 50): per-task raw loss, per-task
weighted loss, per-task `log_var`, per-optimizer LR. Per-epoch: per-task
primary metric, weighted aggregate score, EMA decay value.

## 7. Optimizer & Training Dynamics

### 7.1 Param-group construction

`build_param_groups(module, name="MuSGD", lr=0.01, momentum=0.9, decay=1e-5)`
ports `BaseTrainer.build_optimizer`'s 4-group split:

- g0: weights with decay (`ndim==1` non-bias non-norm)
- g1: norm / `logit_scale` (no decay)
- g2: bias (no decay)
- g3: muon params (`ndim≥2`, with decay) — only when `name == "MuSGD"`

Then applies the cv3/proto regex to split each group into `lr*3` and `lr*1`
sub-groups. Critically, the regex `(?=.*23)(?=.*cv3)|proto\.semseg` is
hardcoded for YOLO26m's layer index 23. We compute the matching head index
dynamically from the head module structure rather than hardcoding `23`, so
yolo26s/l/x scales work identically. Concretely: the regex becomes
`(?=.*<head_last_layer_index>)(?=.*cv3)|proto\.semseg`, parameterized per
head at construction time.

### 7.2 Optimizer instantiation

One optimizer per module, dispatched on `--optimizer`:

- `opt_backbone` over `hydra.shared_backbone`, base LR `--lr_backbone`
  (default `0.001`).
- `opt_head_<task>` per `hydra.heads[task]`, base LR `--lr_heads`
  (default `0.01`).

When `--optimizer MuSGD` (default), both `opt_backbone` and `opt_head_<task>`
use `MuSGD` with the 4-group split + cv3/proto x3 boost.

When `--optimizer AdamW`, both use `AdamW` with a 3-group split (g0/g1/g2 only;
g3 muon group is collapsed back into g0). The cv3/proto x3 LR boost still
applies. AdamW path is left as a debug knob and is not validated in v1.

`opt_logvar` over `weighter.log_var` is **always AdamW** regardless of
`--optimizer`, because the log-variance tensor is 1-D and Newton-Schulz
orthogonalization requires `ndim≥2`. Base LR `--lr_logvar` (default `0.001`).

Momentum / decay defaults: `momentum=0.9`, `weight_decay=1e-5` (Ultralytics
defaults), overridable via `--momentum` / `--weight_decay`.

### 7.3 Schedulers

One `CosineAnnealingLR` per optimizer with shared `T_max=epochs`, all stepped
once per epoch after the validation hook. A `LambdaLR` linear warmup (default
3 epochs, `--warmup_epochs`) is chained before the cosine on each optimizer,
matching Ultralytics' default warmup shape.

### 7.4 Synchronized task accumulation

Pseudocode for one training step:

```python
for opt in all_optimizers:
    opt.zero_grad(set_to_none=True)

per_task_losses: dict[TaskType, Tensor] = {}

for task in sorted_tasks:
    batch = next_batch(task)
    batch = move_to_device(batch, device)

    with torch.amp.autocast(device_type="cuda", dtype=torch.float16, enabled=use_amp):
        feat, y_backbone = hydra.run_backbone(batch.imgs)
        pred = hydra.run_head(task, feat, y_backbone)
        loss_t_total, loss_t_components = criteria[task](pred, batch)
        weighted = weighter.weight_single(task, loss_t_total)

    scaler.scale(weighted).backward()
    per_task_losses[task] = loss_t_total.detach()

# Once all tasks have contributed:
scaler.unscale_(opt_backbone)
clip_grad_norm_(hydra.shared_backbone.parameters(), max_norm=args.max_grad_norm)

def params_of(opt):
    return [p for group in opt.param_groups for p in group["params"]]

for opt in all_optimizers:
    if opt is not opt_backbone:
        scaler.unscale_(opt)
    if args.clip_heads and opt is not opt_backbone and opt is not opt_logvar:
        clip_grad_norm_(params_of(opt), max_norm=args.max_grad_norm)
    scaler.step(opt)

scaler.update()
ema.update(hydra, weighter)
```

Backbone gradient accumulation is correct because `opt_backbone.zero_grad()`
runs once per *step* (across all tasks), `opt_head_<task>.zero_grad()` runs
once per task within a step (only that head receives grads from its own
forward), and the backbone's `.grad` tensor is the natural sum of per-task
backward contributions.

### 7.5 AMP

`torch.amp.autocast(device_type="cuda", dtype=torch.float16)` and
`torch.amp.GradScaler("cuda")` (the non-deprecated API). `--no_amp` disables.

### 7.6 EMA

`EMAHydra` wraps a deep-copied, no-grad `Hydra` instance plus a copy of the
`UncertaintyWeighter`. Decay schedule:

```
decay(step) = 0.9999 * (1 - exp(-step / 2000))
```

(matching Ultralytics' `ModelEMA`). EMA weights are what the validation hook
sees and what gets written as `best.pt` / `last.pt`. `--no_ema` disables.

### 7.7 Multi-GPU

If `--device 0,1,2` is provided, wrap `hydra` in `nn.DataParallel`. DDP is
deferred (would require `mp.spawn` entry-point, larger scope than v1).

## 8. Validation & Checkpointing

### 8.1 Per-epoch validation flow

After step loop completes for epoch `e`:

1. Call `epoch_update(criteria)` — advances E2ELoss decay state.
2. Step every scheduler.
3. If `e % val_interval != 0` and `e != epochs - 1`, skip validation/checkpointing
   for this epoch (the patience counter is **not** advanced on skipped epochs).
4. `run_validation(ema_hydra, hydra_model_name, datasets_per_task, run_dir, weights)`:
   - For each task, materialize a temp per-task `.pt` (cross-style: EMA
     backbone swapped into the task head model).
   - Call `validate_hydra_model(hydra_model_name_for_task, ValidationConfig(...), assets_dir)`
     from existing `validator.py`, **unmodified**.
   - Read back `metrics.json` from each per-task validation run, extract the
     primary metric (`metrics/mAP50-95(B)` for detect/seg, `metrics/mAP50-95(P)`
     for pose), build `{task: primary_metric}`.
   - `score = sum(weights[task] * metric[task] for task in tasks)`.
5. Log `(score, per_task_metrics)` to W&B.

### 8.2 Checkpoint outputs

Each epoch writes:

- `runs/joint_train/<hydra_model>~<wordlet>/<task>/last.pt` — current EMA
  snapshot, per-task assembled.

When `score > best_score`, atomically renames a fresh `last.pt`-equivalent into:

- `runs/joint_train/<hydra_model>~<wordlet>/<task>/best.pt`

Other artifacts (single per run, sibling to per-task subdirs):

- `runs/joint_train/<hydra_model>~<wordlet>/joint_state.json` — optimizer
  states, scheduler states, EMA decay step counter, log-variance values,
  best_score, current epoch, RNG state. The resume artifact.
- `runs/joint_train/<hydra_model>~<wordlet>/config.json` — effective CLI args
  and resolved hyperparameters, written once at start.

### 8.3 Early stopping

A `Patience(patience: int)` helper in `loop.py`: if `score` hasn't improved
for `--patience` epochs (default `30`, mirroring `train.py`'s default), halt
early. `last.pt` and `best.pt` already on disk.

### 8.4 Resume

`--resume` reads `joint_state.json` and rehydrates everything. Mid-epoch
resume is not supported in v1: the current epoch restarts from step 0 with
restored optimizer/EMA/RNG state.

### 8.5 W&B integration

Same pattern as `train.py`:

```python
wandb.init(project="multi-task-yolo", name="<hydra_model>~<wordlet>",
           config=<run config dict>)
```

Per-step (gated by `--log_interval`): per-task raw losses, weighted total,
log-variances, per-optimizer LRs.

Per-epoch: per-task metrics, weighted score, EMA decay, scheduler LRs.

## 9. CLI Surface

`uv run python src/model/joint_train.py [OPTIONS]`. Click CLI, mirroring the
conventions in `train.py` / `validator.py`.

### 9.1 Required

- `--hydra_model_name TEXT` — e.g. `yolo26m=f11+yolo26m-pose+yolo26m-seg`.
  Uses existing `HYDRA_MODEL_NAME_TYPE`.

### 9.2 Per-task dataset overrides

- `--object_dataset_name PATH` (default `coco.yaml`)
- `--pose_dataset_name PATH` (default `coco-pose.yaml`)
- `--segmentation_dataset_name PATH` (default `coco.yaml`)

### 9.3 Paths

- `--assets_dir PATH` (default `assets`)
- `--runs_dir PATH` (default `runs`)
- `--joint_train_dir PATH` (default `joint_train` — subdir under `runs_dir`)

### 9.4 Compute

- `--device TEXT` (default `"0"`, comma-separated list parsed like `train.py`)
- `--workers INT` (default `8`)
- `--seed INT` (default `0`)

### 9.5 Training schedule

- `--epochs INT` (default `100`)
- `--patience INT` (default `30`)
- `--warmup_epochs INT` (default `3`)
- `--val_interval INT` (default `1` — every Nth epoch runs validation)
- `--batch INT` (default `16`)
- `--imgsz INT` (default `640`)

### 9.6 Optimizer

- `--lr_backbone FLOAT` (default `0.001`)
- `--lr_heads FLOAT` (default `0.01`)
- `--lr_logvar FLOAT` (default `0.001`)
- `--momentum FLOAT` (default `0.9`)
- `--weight_decay FLOAT` (default `1e-5`)
- `--max_grad_norm FLOAT` (default `10.0`)
- `--optimizer [MuSGD|AdamW]` (default `MuSGD`; MuSGD is the only path
  validated in v1)

### 9.7 Loss / weighting

- `--init_log_var TEXT` (repeatable, e.g. `--init_log_var pose=0.5`)
- `--task_weight TEXT` (repeatable, e.g. `--task_weight pose=1.5
  --task_weight object=1.0`) — used only for the validation aggregate score.

### 9.8 Toggles

- `--amp / --no_amp` (default on)
- `--ema / --no_ema` (default on)
- `--clip_heads` (default off)
- `--resume` (rehydrate from `runs_dir/joint_train_dir/<hydra_model>~<wordlet>/joint_state.json`)

### 9.9 Logging

- `--log_interval INT` (default `50`)
- `--wandb_project TEXT` (default `multi-task-yolo`)

## 10. Risks & Mitigations

1. **`Hydra.run_backbone` / `run_head` parity with current `forward()`.**
   *Mitigation:* parity test (`test_hydra_split.py`) — random input, assert
   `dict_equal(old_forward, new_forward)` across all task combos in
   `task_dict`. Ship parity test before refactoring `forward()`.

2. **`E2ELoss.parse_output(preds)` shape requirements.** The end2end head's
   raw output structure varies by task and may not match what `parse_output`
   expects.
   *Mitigation:* before wiring `criteria.py`, inspect
   `ultralytics.nn.modules.head.{Detect,Segment,Pose}` end2end forward and
   confirm `run_head` returns exactly what `parse_output` consumes. If
   mismatched, do the conversion inside `_HydraHeadAdapter` rather than
   modifying `Hydra`.

3. **`PoseLoss26` `flow_model` access.** Reads
   `model.model[-1].flow_model`.
   *Mitigation:* `_HydraHeadAdapter.model = hydra.heads[task]` makes
   `model.model[-1]` resolve to the head's last `nn.Module`. Verify via
   `test_hydra_split.py` that the indexing yields the module owning
   `flow_model`.

4. **MuSGD param-group regex hardcoded for layer 23.** Yolo26s/l/x have
   different head indices.
   *Mitigation:* compute the head's last-layer index from
   `hydra.backbone_length` + head module length and substitute into the
   regex at param-group construction time.

5. **Backbone gradient accumulation correctness.**
   *Mitigation:* unit test (`test_optim.py`) with two trivial fake tasks
   asserting `shared_backbone.weight.grad` after a synchronized step matches
   the manual sum of single-task grads.

6. **Validation hook overhead.** Calling `validate_hydra_model()` per task per
   epoch may dominate runtime on small datasets.
   *Mitigation:* expose `--val_interval N` (default `1`) so users can validate
   every Nth epoch. Default still validates every epoch per Q2.

## 11. Test Surface

No live unit-test infra exists in this project today (`AGENTS.md` notes no
`pytest`/CI). We add `pytest>=8.3` to the `dev` dependency group and a
`tests/` directory; `uv run pytest tests/` becomes the test entry-point.

- `tests/joint_loop/test_dataloader.py` — `InterleavedTaskDataloader`
  round-robin order; restart-on-exhaustion behaviour; `set_epoch` propagation.
- `tests/joint_loop/test_weighting.py` — `UncertaintyWeighter` math; gradient
  flow into `log_var`; `weight_single` equivalent to `forward` summed across
  tasks.
- `tests/joint_loop/test_optim.py` — `build_param_groups` 4-group split for
  backbone/head/logvar; cv3/proto x3 LR boost applied; backbone gradient
  accumulation invariant under synchronized steps.
- `tests/joint_loop/test_hydra_split.py` — Parity between old `forward()` and
  `run_backbone+run_head`-composed `forward()` across all task combos.
- `tests/joint_loop/test_loop_smoke.py` — End-to-end smoke test on tiny
  synthetic dataset, two tasks, two epochs. Asserts: no NaN, score is
  non-decreasing on average, expected checkpoint files exist.

## 12. Out-of-Scope Items (future work)

- DDP / multi-node.
- Gradient checkpointing.
- Mid-epoch resume.
- AdamW-path validation.
- Tuning integration.
- Per-task batch / imgsz knobs.
- Per-task EMA decay schedules.
