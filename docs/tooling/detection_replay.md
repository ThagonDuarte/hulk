# Detection Replay

`detection-replay` prerenders object detections from ROS-Z MCAP recordings and compares models on a synchronized image feed. Inference uses the production detection implementation through `run_boxed_with_model_info`; viewing only reads cached predictions. The repository wrapper builds and runs the tool in release mode.

## Recording index

Build the random-access JPEG proxy and import any recorded `detected_objects` messages:

```bash
./detection-replay index /path/to/recording.mcap
```

The source MCAP is opened read-only. A recording that ends in an incomplete final chunk remains usable: complete chunks are indexed and the damaged tail is reported. The tool supports `inputs/left_image` and `inputs/stereo_image_pair` recordings. Prerender resumes seek through the MCAP chunk index when a summary is available; damaged or unindexed recordings keep a direct-seek CDR copy of original images in the cache.

By default the cache is written next to the recording as `recording.detection-replay-cache`. Use `--cache-dir` to put it elsewhere.

All commands accept an inclusive frame-ID range through `--start-frame` and `--end-frame`. The index still scans the MCAP once to establish stable global frame IDs, but prerendering and viewing are restricted to the selected range.

## Prerendering

Run models sequentially so their GPU timings and predictions do not interfere:

```bash
./detection-replay prerender /path/to/recording.mcap \
  --model baseline=/path/to/baseline.onnx \
  --model candidate=/path/to/candidate.onnx \
  --provider cuda --gpu 1 \
  --start-frame 5000 --end-frame 5500
```

Each input frame is published only after the preceding result has been received. Predictions are matched by the image header timestamp and written in atomic chunks. Repeating the command resumes at the first uncached frame.

Use `--limit 10` for a short provider/model compatibility check. CUDA is the default provider. Select `automatic`, `tensorrt`, `cuda`, or `cpu` with `--provider`; `--gpu` selects the physical device index for NVIDIA providers. Replay exposes that physical GPU as logical device zero through `CUDA_VISIBLE_DEVICES` before ONNX Runtime or worker threads start, avoiding cross-thread CUDA context mismatches on nonzero devices. `tensorrt` retains CUDA as an operator fallback, while `automatic` follows the available production order TensorRT, CUDA, then CPU in this NVIDIA build. The default confidence threshold is `0.05` and the default NMS IoU is `0.4`; use `--confidence` and `--iou` to change them. Thresholds, model hash, provider, and device are part of the cache identity, so switching provider cannot silently mix results or timing measurements in one run.

Models must accept `raw_bytes_input` as a rank-three `uint8` tensor with shape `[height / 2, width / 2, 6]`. Its contiguous bytes are the full-resolution NV12 Y plane followed by the half-height interleaved UV plane; image width and height must be multiples of 32.

`object_output` is mandatory and must be a `float32` tensor with shape `[1, 300, 6]`. Each row is `[x_min, y_min, x_max, y_max, confidence, class_index]`; coordinates are full input-image pixel coordinates. Class indices must be integral values from 0 through 6: Ball, GoalPost, LSpot, PenaltySpot, Robot, TSpot, and XSpot.

`person_pose_output` is optional and must be `float32 [1, 300, 57]`: six pose-object values followed by 17 `(x, y, confidence)` triples in COCO order: nose, left eye, right eye, left ear, right ear, left shoulder, right shoulder, left elbow, right elbow, left hand, right hand, left hip, right hip, left knee, right knee, left foot, right foot. Pose class indices use the COCO `YOLOObjectLabel` mapping (`0` is Person), not the RoboCup object mapping. Coordinates use the same full-image pixel space. For older human-pose models, `pose_output [1, 300, 57]` remains supported when the model has no `branches` metadata.

`pose_output` may instead carry field poses as `float32 [1, 300, 9]`: six pose-object values followed by one `(x, y, confidence)` keypoint. Detection recognizes this form through the Hydra `branches` metadata and maps the metadata's class names to GoalPost, LSpot, TSpot, PenaltySpot, and XSpot rather than assuming a numeric class order. It publishes the keypoint through `detected_field_features`, so replay and robotics use the same field-feature message.

`robot_pose_output` is optional and must be `float32 [1, 300, 48]`: six pose-object values followed by 14 `(x, y, confidence)` triples in this DHRP order: nose, neck, right shoulder, right elbow, right wrist, left shoulder, left elbow, left wrist, right hip, right knee, right ankle, left hip, left knee, left ankle. The DHRP model has one class (`0`, Robot); the runtime maps it to the RoboCup Robot label. Replay caches these timestamp-matched robot skeletons separately and distinguishes unavailable output from a valid empty result.

`field_feature_output` is also optional and must be `float32 [1, 300, 4]`. Each row is `[x, y, confidence, class_index]` in full-image pixel coordinates. Class indices are GoalPost, LSpot, TSpot, PenaltySpot, and XSpot. Replay stores the timestamp-matched field-feature points separately from the object and pose chunks so caches created before field-feature capture remain readable.

The desktop replay build uses the x86-64 ONNX Runtime 1.22 CUDA distribution. The wrapper prevents a CPU-only system runtime discovered through `pkg-config` from overriding it and adds CUDA 12 user-space libraries from the multi-task YOLO virtual environment to the loader path when they are available. Operators unsupported by the selected NVIDIA provider may still use ORT's CPU fallback. Robot builds dynamically load their runtime image and retain the default provider order compiled into that deployment. Only the generic provider choice crosses into the detection node; replay-specific CLI parsing, physical-device remapping, CUDA library discovery, and defaults remain confined to this tool.

## Viewing

```bash
./detection-replay view /path/to/recording.mcap --start-frame 5000 --end-frame 5500
```

The viewer initially opens model controls and model viewports as tabs in the upper dock, with the timeline in a lower dock. Tabs can be moved between dock nodes, reordered, detached, closed, and reopened from the top-bar `View` menu. Model viewports remain synchronized and share pan and zoom.

The timestamp-proportional timeline shows source capture gaps and prediction availability for every run. Drag to scrub, scroll to zoom around the pointer, Shift+scroll to pan, and double-click to reset to the selected CLI frame range. Press `B` to toggle a bookmark and Page Up/Page Down to visit bookmarks; bookmarks are persisted in the recording-specific cache and persistence failures are shown in Models.

The Models tab manages cached runs. Model runs can be renamed, hidden, or permanently deleted after confirming the `Are you sure?` dialog. Renames and hidden state persist in recording-specific cache metadata. Hidden runs are removed from the timeline, viewport tabs, and `View` menu but remain in Models for unhiding. The source-derived Recorded baseline can be hidden but cannot be renamed or deleted.

Model-run directories may be synced between machines. The recording can live at a different absolute path on the destination, but the sync must preserve its size and modification time so the viewer can safely associate the runs with it.

`Show field features` overlays cached field-feature points and is enabled by default. `Show robot poses` is also enabled by default; `Show person poses` controls the human-pose overlay. Pose bounding boxes use the display-confidence threshold, while skeleton lines and keypoints use the separate keypoint-confidence threshold. The viewport distinguishes unavailable optional outputs from valid empty results in its status line. Older runs remain usable but report uncaptured outputs as unavailable; prerender them again to populate the new overlays.

Useful controls:

- `Space`: play or pause
- Left/right arrow: step one frame
- Drag: pan every viewport
- Scroll: zoom every viewport
- Double-click: reset pan and zoom

An unavailable prediction is shown separately from a valid empty detection result.

## Validation

Use a short range before prerendering an entire recording:

```bash
./detection-replay index /path/to/recording.mcap --start-frame 100 --end-frame 109
./detection-replay prerender /path/to/recording.mcap \
  --model test=etc/neural_networks/model.onnx \
  --provider cuda --gpu 0 \
  --start-frame 100 --end-frame 109 --limit 10
```

The prerender must report 10 of 10 frames without a provider or shape error. Repeating it should immediately report the cached 10 frames. Open the same range with `view`; the timeline should show ten source frames, the run should be `complete`, empty detections should differ from unavailable predictions, and any damaged MCAP tail should appear as a warning.
