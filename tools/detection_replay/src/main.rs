use std::{fs, io::Write, path::PathBuf, str::FromStr, time::Duration};

use clap::{Parser, Subcommand};
use color_eyre::{
    Result,
    eyre::{Context, ContextCompat, bail},
};
use detection_replay::{
    DetectionThresholds, LoadedPredictionRun, ModelRunConfig, PerHeadDetectionThresholds,
    PredictionSource, Recording, run_model,
};
use serde::Serialize;
use tracing_subscriber::EnvFilter;

mod app;
mod timeline;

#[derive(Debug, Parser)]
#[command(about = "Prerender and compare detection models on a ROS-Z MCAP recording")]
struct Arguments {
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// Build the random-access frame proxy and import recorded detections.
    Index {
        recording: PathBuf,
        #[arg(long)]
        cache_dir: Option<PathBuf>,
        #[command(flatten)]
        frames: FrameRangeArguments,
    },
    /// Run one or more models through the actual ROS-Z detection node.
    Prerender {
        recording: PathBuf,
        #[arg(long, value_name = "LABEL=MODEL", required = true)]
        model: Vec<ModelArgument>,
        #[arg(long)]
        cache_dir: Option<PathBuf>,
        #[arg(long, default_value_t = 0.05)]
        confidence: f32,
        /// Override object confidence; defaults to --confidence.
        #[arg(long)]
        object_confidence: Option<f32>,
        /// Override person-pose confidence; defaults to --confidence.
        #[arg(long)]
        person_confidence: Option<f32>,
        /// Override robot-pose confidence; defaults to --confidence.
        #[arg(long)]
        robot_confidence: Option<f32>,
        /// Override field-feature confidence; defaults to --confidence.
        #[arg(long)]
        field_confidence: Option<f32>,
        #[arg(long, default_value_t = 0.4)]
        iou: f32,
        /// Exponent applied to mean person-keypoint visibility when scoring person poses.
        #[arg(
            long,
            default_value_t = 0.0,
            allow_hyphen_values = true,
            value_parser = parse_person_visibility_alpha
        )]
        person_visibility_alpha: f32,
        /// Exponent applied to mean robot-keypoint visibility when scoring robot poses.
        #[arg(
            long,
            default_value_t = 0.0,
            allow_hyphen_values = true,
            value_parser = parse_robot_visibility_alpha
        )]
        robot_visibility_alpha: f32,
        /// Stop after this many frames in the selected range. Useful for provider smoke tests.
        #[arg(long)]
        limit: Option<usize>,
        #[command(flatten)]
        frames: FrameRangeArguments,
    },
    /// Summarize cached inference, postprocessing, and NMS latency as JSON.
    Stats {
        recording: PathBuf,
        #[arg(long)]
        cache_dir: Option<PathBuf>,
        /// Exact run key or an unambiguous run label.
        #[arg(long, required = true)]
        run: String,
        /// Number of available frames to exclude before computing statistics.
        #[arg(long, default_value_t = 32)]
        warmup_frames: usize,
        #[arg(long)]
        output: Option<PathBuf>,
        #[command(flatten)]
        frames: FrameRangeArguments,
    },
    /// Open the synchronized multi-model viewer.
    View {
        recording: PathBuf,
        #[arg(long)]
        cache_dir: Option<PathBuf>,
        #[command(flatten)]
        frames: FrameRangeArguments,
    },
}

#[derive(Clone, Copy, Debug, clap::Args)]
struct FrameRangeArguments {
    /// First frame ID to process or display.
    #[arg(long, default_value_t = 0)]
    start_frame: usize,
    /// Last frame ID to process or display, inclusive.
    #[arg(long)]
    end_frame: Option<usize>,
}

impl FrameRangeArguments {
    fn resolve(self, frame_count: usize) -> Result<(usize, usize)> {
        let end = self
            .end_frame
            .unwrap_or_else(|| frame_count.saturating_sub(1));
        if self.start_frame > end {
            color_eyre::eyre::bail!(
                "start frame {} is greater than end frame {end}",
                self.start_frame
            );
        }
        if end >= frame_count {
            color_eyre::eyre::bail!("end frame {end} is out of range for {frame_count} frames");
        }
        Ok((self.start_frame, end))
    }
}

#[derive(Debug, Serialize)]
struct LatencyDistribution {
    count: usize,
    mean_ms: f64,
    p50_ms: f64,
    p90_ms: f64,
    p95_ms: f64,
    p99_ms: f64,
    max_ms: f64,
}

#[derive(Debug, Serialize)]
struct LatencyComponents {
    inference: LatencyDistribution,
    postprocessing: LatencyDistribution,
    non_maximum_suppression: LatencyDistribution,
    total: LatencyDistribution,
}

#[derive(Debug, Serialize)]
struct LatencyReport {
    schema_version: u32,
    run_key: String,
    label: String,
    state: String,
    canonical_model_path: PathBuf,
    model_blake3: String,
    provider_note: Option<String>,
    recording_path: PathBuf,
    recording_size_bytes: u64,
    recording_modified_unix_nanos: u128,
    per_head_thresholds: PerHeadDetectionThresholds,
    person_visibility_score_alpha: f32,
    robot_visibility_score_alpha: f32,
    requested_frame_start: usize,
    requested_frame_end: usize,
    run_frame_start: usize,
    run_frame_end: usize,
    available_frame_count: usize,
    warmup_policy: &'static str,
    warmup_requested_frames: usize,
    warmup_excluded_frames: usize,
    missing_timing_frames: usize,
    percentile_method: &'static str,
    latency: LatencyComponents,
}

fn effective_per_head_thresholds(
    fallback: DetectionThresholds,
    object_confidence: Option<f32>,
    person_confidence: Option<f32>,
    robot_confidence: Option<f32>,
    field_confidence: Option<f32>,
) -> Result<PerHeadDetectionThresholds> {
    PerHeadDetectionThresholds {
        object_minimum_candidate_confidence: object_confidence
            .unwrap_or(fallback.minimum_candidate_confidence),
        person_pose_minimum_candidate_confidence: person_confidence
            .unwrap_or(fallback.minimum_candidate_confidence),
        robot_pose_minimum_candidate_confidence: robot_confidence
            .unwrap_or(fallback.minimum_candidate_confidence),
        field_feature_minimum_candidate_confidence: field_confidence
            .unwrap_or(fallback.minimum_candidate_confidence),
        maximum_intersection_over_union: fallback.maximum_intersection_over_union,
    }
    .validate()
}

fn select_model_run<'a>(
    runs: &'a [LoadedPredictionRun],
    selector: &str,
) -> Result<&'a LoadedPredictionRun> {
    if let Some(run) = runs
        .iter()
        .find(|run| run.source == PredictionSource::Model && run.key == selector)
    {
        return Ok(run);
    }
    let matches = runs
        .iter()
        .filter(|run| run.source == PredictionSource::Model && run.label == selector)
        .collect::<Vec<_>>();
    match matches.as_slice() {
        [] => bail!("no cached model run matches `{selector}`"),
        [selected] => Ok(*selected),
        _ => {
            let keys = matches
                .iter()
                .map(|run| run.key.as_str())
                .collect::<Vec<_>>()
                .join(", ");
            bail!("cached run label `{selector}` is ambiguous; use one of: {keys}")
        }
    }
}

fn latency_distribution(values_nanos: &[u64]) -> Result<LatencyDistribution> {
    if values_nanos.is_empty() {
        bail!("no complete cached timing samples remain after warmup exclusion");
    }
    let mut values = values_nanos
        .iter()
        .map(|value| *value as f64 / 1.0e6)
        .collect::<Vec<_>>();
    values.sort_by(f64::total_cmp);
    let mean_ms = values.iter().sum::<f64>() / values.len() as f64;
    Ok(LatencyDistribution {
        count: values.len(),
        mean_ms,
        p50_ms: interpolated_percentile(&values, 0.50),
        p90_ms: interpolated_percentile(&values, 0.90),
        p95_ms: interpolated_percentile(&values, 0.95),
        p99_ms: interpolated_percentile(&values, 0.99),
        max_ms: values[values.len() - 1],
    })
}

fn interpolated_percentile(sorted: &[f64], probability: f64) -> f64 {
    debug_assert!(!sorted.is_empty());
    debug_assert!((0.0..=1.0).contains(&probability));
    let position = (sorted.len() - 1) as f64 * probability;
    let lower = position.floor() as usize;
    let upper = position.ceil() as usize;
    let fraction = position - lower as f64;
    sorted[lower] + (sorted[upper] - sorted[lower]) * fraction
}

fn latency_report(
    recording_path: &std::path::Path,
    run: &LoadedPredictionRun,
    start_frame: usize,
    end_frame: usize,
    warmup_frames: usize,
) -> Result<LatencyReport> {
    let manifest = run
        .manifest
        .as_ref()
        .wrap_err("cached model run has no manifest")?;
    let mut available_frame_count = 0_usize;
    let mut warmup_excluded_frames = 0_usize;
    let mut missing_timing_frames = 0_usize;
    let mut inference = Vec::new();
    let mut postprocessing = Vec::new();
    let mut non_maximum_suppression = Vec::new();
    let mut total = Vec::new();
    for frame in start_frame..=end_frame {
        if !run.is_available(frame) {
            continue;
        }
        available_frame_count += 1;
        if warmup_excluded_frames < warmup_frames {
            warmup_excluded_frames += 1;
            continue;
        }
        let Some(prediction) = run.prediction(frame)? else {
            missing_timing_frames += 1;
            continue;
        };
        let (Some(inference_nanos), Some(postprocessing_nanos), Some(nms_nanos)) = (
            prediction.inference_duration_nanos,
            prediction.postprocessing_duration_nanos,
            prediction.non_maximum_suppression_duration_nanos,
        ) else {
            missing_timing_frames += 1;
            continue;
        };
        inference.push(inference_nanos);
        postprocessing.push(postprocessing_nanos);
        non_maximum_suppression.push(nms_nanos);
        total.push(
            inference_nanos
                .saturating_add(postprocessing_nanos)
                .saturating_add(nms_nanos),
        );
    }
    let model_blake3 = manifest
        .model_hash
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect::<String>();
    let canonical_recording_path = recording_path.canonicalize().wrap_err_with(|| {
        format!(
            "failed to canonicalize recording path {}",
            recording_path.display()
        )
    })?;
    Ok(LatencyReport {
        schema_version: 2,
        run_key: manifest.run_key.clone(),
        label: manifest.label.clone(),
        state: format!("{:?}", manifest.state),
        canonical_model_path: manifest.canonical_model_path.clone(),
        model_blake3,
        provider_note: manifest.provider_note.clone(),
        recording_path: canonical_recording_path,
        recording_size_bytes: manifest.recording_fingerprint.size,
        recording_modified_unix_nanos: manifest.recording_fingerprint.modified_unix_nanos,
        per_head_thresholds: manifest.per_head_thresholds,
        person_visibility_score_alpha: manifest.person_visibility_score_alpha,
        robot_visibility_score_alpha: manifest.robot_visibility_score_alpha,
        requested_frame_start: start_frame,
        requested_frame_end: end_frame,
        run_frame_start: manifest.frame_start,
        run_frame_end: manifest.frame_end,
        available_frame_count,
        warmup_policy: "first N available frames in the requested range",
        warmup_requested_frames: warmup_frames,
        warmup_excluded_frames,
        missing_timing_frames,
        percentile_method: "linear interpolation at (N - 1) * p",
        latency: LatencyComponents {
            inference: latency_distribution(&inference)?,
            postprocessing: latency_distribution(&postprocessing)?,
            non_maximum_suppression: latency_distribution(&non_maximum_suppression)?,
            total: latency_distribution(&total)?,
        },
    })
}

#[derive(Clone, Debug)]
struct ModelArgument {
    label: String,
    path: PathBuf,
}

impl FromStr for ModelArgument {
    type Err = String;

    fn from_str(value: &str) -> Result<Self, Self::Err> {
        let (label, path) = value
            .split_once('=')
            .ok_or_else(|| "model must use LABEL=MODEL syntax".to_string())?;
        if label.trim().is_empty() || path.trim().is_empty() {
            return Err("model label and path must not be empty".to_string());
        }
        Ok(Self {
            label: label.trim().to_string(),
            path: PathBuf::from(path.trim()),
        })
    }
}

fn main() -> Result<()> {
    color_eyre::install()?;
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env())
        .init();

    match Arguments::parse().command {
        Command::Index {
            recording,
            cache_dir,
            frames,
        } => {
            let recording_path = resolve_recording(recording);
            let cache_dir = cache_dir.unwrap_or_else(|| default_cache_dir(&recording_path));
            let recording = Recording::open(&recording_path, &cache_dir)?;
            report_recording_warning(&recording);
            let (start_frame, end_frame) = frames.resolve(recording.frame_count())?;
            let recorded_predictions = recording
                .load_runs()?
                .into_iter()
                .find(|run| run.key == "recorded")
                .map(|run| run.available_count(start_frame..=end_frame))
                .unwrap_or_default();
            println!(
                "indexed frames {}..={} ({} of {}) and {} aligned recorded prediction frames from {} into {}",
                start_frame,
                end_frame,
                end_frame - start_frame + 1,
                recording.frame_count(),
                recorded_predictions,
                recording_path.display(),
                cache_dir.display()
            );
        }
        Command::Prerender {
            recording,
            model,
            cache_dir,
            confidence,
            object_confidence,
            person_confidence,
            robot_confidence,
            field_confidence,
            iou,
            person_visibility_alpha,
            robot_visibility_alpha,
            limit,
            frames,
        } => {
            let recording_path = resolve_recording(recording);
            let cache_dir = cache_dir.unwrap_or_else(|| default_cache_dir(&recording_path));
            let recording = Recording::open(&recording_path, &cache_dir)?;
            report_recording_warning(&recording);
            let (start_frame, end_frame) = frames.resolve(recording.frame_count())?;
            let thresholds = DetectionThresholds {
                minimum_candidate_confidence: confidence,
                maximum_intersection_over_union: iou,
            }
            .validate()?;
            let per_head_thresholds = effective_per_head_thresholds(
                thresholds,
                object_confidence,
                person_confidence,
                robot_confidence,
                field_confidence,
            )?;
            let ort_result = ort::init()
                .commit()
                .wrap_err("failed to initialize ONNX Runtime");
            let runtime = tokio::runtime::Builder::new_multi_thread()
                .enable_all()
                .build()
                .wrap_err("failed to build replay runtime")?;

            let prerender_result = ort_result.and_then(|_| {
                for model in model {
                    let mut config = ModelRunConfig::new(model.label.clone(), model.path);
                    config.thresholds = thresholds;
                    config.per_head_thresholds = Some(per_head_thresholds);
                    config.person_visibility_score_alpha = person_visibility_alpha;
                    config.robot_visibility_score_alpha = robot_visibility_alpha;
                    config.frame_limit = limit;
                    config.start_frame = start_frame;
                    config.end_frame = Some(end_frame);
                    config.output_timeout = Duration::from_secs(120);
                    let mut last_reported = usize::MAX;
                    let manifest = runtime.block_on(run_model(&recording, config, |progress| {
                        if progress.completed_frames == progress.target_frames
                            || progress.completed_frames / 100 != last_reported / 100
                        {
                            println!(
                                "{}: {}/{} frames",
                                progress.label, progress.completed_frames, progress.target_frames
                            );
                            last_reported = progress.completed_frames;
                        }
                    }))?;
                    println!(
                        "{} [{}]: {:?} ({}/{} frames)",
                        manifest.label,
                        manifest.run_key,
                        manifest.state,
                        manifest.completed_frame_count,
                        end_frame - start_frame + 1
                    );
                }
                Ok(())
            });
            runtime.shutdown_timeout(Duration::from_secs(2));
            exit_after_webgpu(prerender_result)?;
        }
        Command::Stats {
            recording,
            cache_dir,
            run,
            warmup_frames,
            output,
            frames,
        } => {
            let recording_path = resolve_recording(recording);
            let cache_dir = cache_dir.unwrap_or_else(|| default_cache_dir(&recording_path));
            let recording = Recording::open(&recording_path, &cache_dir)?;
            report_recording_warning(&recording);
            let (start_frame, end_frame) = frames.resolve(recording.frame_count())?;
            let runs = recording.load_runs()?;
            let selected = select_model_run(&runs, &run)?;
            let report = latency_report(
                &recording_path,
                selected,
                start_frame,
                end_frame,
                warmup_frames,
            )?;
            let encoded = serde_json::to_string_pretty(&report)? + "\n";
            if let Some(output) = output {
                if let Some(parent) = output
                    .parent()
                    .filter(|parent| !parent.as_os_str().is_empty())
                {
                    fs::create_dir_all(parent).wrap_err_with(|| {
                        format!(
                            "failed to create latency report directory {}",
                            parent.display()
                        )
                    })?;
                }
                fs::write(&output, &encoded).wrap_err_with(|| {
                    format!("failed to write latency report {}", output.display())
                })?;
                println!("latency report: {}", output.display());
            } else {
                print!("{encoded}");
            }
        }
        Command::View {
            recording,
            cache_dir,
            frames,
        } => {
            let recording_path = resolve_recording(recording);
            let cache_dir = cache_dir.unwrap_or_else(|| default_cache_dir(&recording_path));
            let recording = Recording::open(recording_path, cache_dir)?;
            let (start_frame, end_frame) = frames.resolve(recording.frame_count())?;
            app::run(recording, start_frame, end_frame)?;
        }
    }

    Ok(())
}

fn report_recording_warning(recording: &Recording) {
    if let Some(warning) = &recording.index().tail_warning {
        eprintln!("warning: {warning}");
    }
}

fn resolve_recording(path: PathBuf) -> PathBuf {
    if path.is_dir() {
        path.join("recording.mcap")
    } else {
        path
    }
}

fn default_cache_dir(recording: &std::path::Path) -> PathBuf {
    recording.with_extension("detection-replay-cache")
}

fn parse_visibility_alpha(value: &str, subject: &str) -> std::result::Result<f32, String> {
    let alpha = value
        .parse::<f32>()
        .map_err(|error| format!("invalid {subject} visibility alpha `{value}`: {error}"))?;
    if !alpha.is_finite() || alpha < 0.0 {
        return Err(format!(
            "{subject} visibility alpha must be finite and non-negative, got {value}"
        ));
    }
    Ok(if alpha == 0.0 { 0.0 } else { alpha })
}

fn parse_person_visibility_alpha(value: &str) -> std::result::Result<f32, String> {
    parse_visibility_alpha(value, "person")
}

fn parse_robot_visibility_alpha(value: &str) -> std::result::Result<f32, String> {
    parse_visibility_alpha(value, "robot")
}

fn exit_after_webgpu(result: Result<()>) -> Result<()> {
    #[cfg(target_os = "linux")]
    {
        let status = match result {
            Ok(()) => 0,
            Err(error) => {
                eprintln!("Error: {error:?}");
                1
            }
        };
        // ORT 1.22's WebGPU artifact double-releases its Dawn instance from an
        // atexit handler. All cache files are synced before reaching this point.
        let _ = std::io::stdout().flush();
        let _ = std::io::stderr().flush();
        // SAFETY: all application-owned output and cache files were flushed above.
        unsafe { libc::_exit(status) }
    }

    #[cfg(not(target_os = "linux"))]
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse_prerender_alpha(value: &str) -> std::result::Result<f32, clap::Error> {
        let arguments = Arguments::try_parse_from([
            "detection-replay",
            "prerender",
            "recording.mcap",
            "--model",
            "candidate=model.onnx",
            "--robot-visibility-alpha",
            value,
        ])?;
        let Command::Prerender {
            robot_visibility_alpha,
            ..
        } = arguments.command
        else {
            unreachable!();
        };
        Ok(robot_visibility_alpha)
    }

    #[test]
    fn prerender_accepts_non_negative_robot_visibility_alpha() {
        assert_eq!(parse_prerender_alpha("1.25").unwrap(), 1.25);
        assert_eq!(
            parse_prerender_alpha("-0").unwrap().to_bits(),
            0.0_f32.to_bits()
        );
    }

    #[test]
    fn prerender_rejects_invalid_robot_visibility_alpha() {
        for invalid in ["-1", "NaN", "inf", "-inf"] {
            let error = parse_prerender_alpha(invalid).unwrap_err();
            assert!(error.to_string().contains("finite and non-negative"));
        }
    }

    #[test]
    fn prerender_accepts_independent_person_and_robot_visibility_alphas() {
        let arguments = Arguments::try_parse_from([
            "detection-replay",
            "prerender",
            "recording.mcap",
            "--model",
            "candidate=model.onnx",
            "--person-visibility-alpha",
            "1.75",
            "--robot-visibility-alpha",
            "2",
        ])
        .unwrap();
        let Command::Prerender {
            person_visibility_alpha,
            robot_visibility_alpha,
            ..
        } = arguments.command
        else {
            unreachable!();
        };
        assert_eq!(person_visibility_alpha, 1.75);
        assert_eq!(robot_visibility_alpha, 2.0);
    }

    #[test]
    fn prerender_rejects_invalid_person_visibility_alpha() {
        for invalid in ["-1", "NaN", "inf", "-inf"] {
            let arguments = Arguments::try_parse_from([
                "detection-replay",
                "prerender",
                "recording.mcap",
                "--model",
                "candidate=model.onnx",
                "--person-visibility-alpha",
                invalid,
            ]);
            let error = arguments.unwrap_err();
            assert!(error.to_string().contains("finite and non-negative"));
        }
    }

    #[test]
    fn prerender_confidence_shorthand_and_overrides_are_distinct() {
        let arguments = Arguments::try_parse_from([
            "detection-replay",
            "prerender",
            "recording.mcap",
            "--model",
            "candidate=model.onnx",
            "--confidence",
            "0.1",
            "--object-confidence",
            "0.25",
            "--person-confidence",
            "0.5",
            "--robot-confidence",
            "0.45",
            "--field-confidence",
            "0.35",
            "--iou",
            "0.5",
        ])
        .unwrap();
        let Command::Prerender {
            confidence,
            object_confidence,
            person_confidence,
            robot_confidence,
            field_confidence,
            iou,
            ..
        } = arguments.command
        else {
            unreachable!();
        };
        let thresholds = effective_per_head_thresholds(
            DetectionThresholds {
                minimum_candidate_confidence: confidence,
                maximum_intersection_over_union: iou,
            }
            .validate()
            .unwrap(),
            object_confidence,
            person_confidence,
            robot_confidence,
            field_confidence,
        )
        .unwrap();
        assert_eq!(thresholds.object_minimum_candidate_confidence, 0.25);
        assert_eq!(thresholds.person_pose_minimum_candidate_confidence, 0.5);
        assert_eq!(thresholds.robot_pose_minimum_candidate_confidence, 0.45);
        assert_eq!(thresholds.field_feature_minimum_candidate_confidence, 0.35);
        assert_eq!(thresholds.maximum_intersection_over_union, 0.5);

        let uniform = effective_per_head_thresholds(
            DetectionThresholds {
                minimum_candidate_confidence: 0.1,
                maximum_intersection_over_union: 0.4,
            },
            None,
            None,
            None,
            None,
        )
        .unwrap();
        assert_eq!(
            uniform,
            PerHeadDetectionThresholds::uniform(DetectionThresholds {
                minimum_candidate_confidence: 0.1,
                maximum_intersection_over_union: 0.4,
            })
            .unwrap()
        );
    }

    #[test]
    fn latency_distribution_uses_declared_linear_percentiles() {
        let values = [1_000_000, 2_000_000, 3_000_000, 4_000_000, 5_000_000];
        let summary = latency_distribution(&values).unwrap();

        assert_eq!(summary.count, 5);
        assert_eq!(summary.mean_ms, 3.0);
        assert_eq!(summary.p50_ms, 3.0);
        assert!((summary.p90_ms - 4.6).abs() < 1.0e-12);
        assert!((summary.p95_ms - 4.8).abs() < 1.0e-12);
        assert!((summary.p99_ms - 4.96).abs() < 1.0e-12);
        assert_eq!(summary.max_ms, 5.0);
    }

    #[test]
    fn stats_defaults_to_recommended_warmup() {
        let arguments = Arguments::try_parse_from([
            "detection-replay",
            "stats",
            "recording.mcap",
            "--run",
            "model-0123456789abcdef",
        ])
        .unwrap();
        let Command::Stats { warmup_frames, .. } = arguments.command else {
            unreachable!();
        };

        assert_eq!(warmup_frames, 32);
    }
}
