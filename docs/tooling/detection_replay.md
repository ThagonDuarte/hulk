# Detection Replay

`detection-replay` prerenders object detections from ROS-Z MCAP recordings and compares models on a synchronized image feed. Inference uses the production `detection::run_boxed` node; viewing only reads cached predictions.

## Recording index

Build the random-access JPEG proxy and import any recorded `detected_objects` messages:

```bash
./detection-replay index /path/to/recording.mcap
```

The source MCAP is opened read-only. A recording that ends in an incomplete final chunk remains usable: complete chunks are indexed and the damaged tail is reported. The tool supports `inputs/left_image` and `inputs/stereo_image_pair` recordings.

By default the cache is written next to the recording as `recording.detection-replay-cache`. Use `--cache-dir` to put it elsewhere.

## Prerendering

Run models sequentially so their GPU timings and predictions do not interfere:

```bash
./detection-replay prerender /path/to/recording.mcap \
  --model baseline=/path/to/baseline.onnx \
  --model candidate=/path/to/candidate.onnx
```

Each input frame is published only after the preceding result has been received. Predictions are matched by the image header timestamp and written in atomic chunks. Repeating the command resumes at the first uncached frame.

Use `--limit 10` for a short provider/model compatibility check. The default confidence threshold is `0.05` and the default NMS IoU is `0.4`; use `--confidence` and `--iou` to change them. Thresholds and the model hash are part of the cache identity.

Models must accept the production `raw_bytes_input` NV12 tensor and expose `object_output` with shape `[1, 300, 6]`. `pose_output` is optional; object-only models publish an empty timestamp-matched pose result.

The desktop build requires successful ONNX Runtime WebGPU registration. The wrapper prevents a CPU-only system ONNX Runtime discovered through `pkg-config` from overriding the WebGPU runtime. Operators unsupported by WebGPU may still use ORT's CPU fallback. Robot builds retain the provider order TensorRT, CUDA, WebGPU, CPU.

## Viewing

```bash
./detection-replay view /path/to/recording.mcap
```

The viewer shows one synchronized viewport per selected model. It supports source-time playback, speed changes, looping, frame stepping, scrubbing, shared pan/zoom, confidence filtering, inference timing, and a sparse `Recorded` baseline when the MCAP contains detections.

Useful controls:

- `Space`: play or pause
- Left/right arrow: step one frame
- Drag: pan every viewport
- Scroll: zoom every viewport
- Double-click: reset pan and zoom

An unavailable prediction is shown separately from a valid empty detection result.
