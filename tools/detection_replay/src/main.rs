use std::{io::Write, path::PathBuf, str::FromStr, time::Duration};

use clap::{Parser, Subcommand};
use color_eyre::{Result, eyre::Context};
use detection_replay::{DetectionThresholds, ModelRunConfig, Recording, run_model};
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
        #[arg(long, default_value_t = 0.4)]
        iou: f32,
        /// Stop after this many frames in the selected range. Useful for provider smoke tests.
        #[arg(long)]
        limit: Option<usize>,
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
            let (start_frame, end_frame) = frames.resolve(recording.frame_count())?;
            let recorded_predictions = recording
                .load_runs()?
                .into_iter()
                .find(|run| run.key == "recorded")
                .map(|run| {
                    run.predictions[start_frame..=end_frame]
                        .iter()
                        .filter(|prediction| prediction.is_some())
                        .count()
                })
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
            if let Some(warning) = &recording.index().tail_warning {
                println!("warning: {warning}");
            }
        }
        Command::Prerender {
            recording,
            model,
            cache_dir,
            confidence,
            iou,
            limit,
            frames,
        } => {
            let recording_path = resolve_recording(recording);
            let cache_dir = cache_dir.unwrap_or_else(|| default_cache_dir(&recording_path));
            let recording = Recording::open(&recording_path, &cache_dir)?;
            let (start_frame, end_frame) = frames.resolve(recording.frame_count())?;
            let thresholds = DetectionThresholds {
                minimum_candidate_confidence: confidence,
                maximum_intersection_over_union: iou,
            }
            .validate()?;
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
                    config.frame_limit = limit;
                    config.start_frame = start_frame;
                    config.end_frame = Some(end_frame);
                    config.output_timeout = Duration::from_secs(120);
                    let mut last_reported = usize::MAX;
                    let manifest = runtime.block_on(run_model(
                        &recording,
                        &cache_dir,
                        config,
                        |progress| {
                            if progress.completed_frames == progress.target_frames
                                || progress.completed_frames / 100 != last_reported / 100
                            {
                                println!(
                                    "{}: {}/{} frames",
                                    progress.label,
                                    progress.completed_frames,
                                    progress.target_frames
                                );
                                last_reported = progress.completed_frames;
                            }
                        },
                    ))?;
                    println!(
                        "{}: {:?} ({}/{} frames)",
                        manifest.label,
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
