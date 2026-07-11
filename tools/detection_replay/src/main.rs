use std::{io::Write, path::PathBuf, str::FromStr, time::Duration};

use clap::{Parser, Subcommand};
use color_eyre::{Result, eyre::Context};
use detection_replay::{DetectionThresholds, ModelRunConfig, Recording, run_model};
use tracing_subscriber::EnvFilter;

mod app;

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
        /// Stop after this many total frames. Useful for provider smoke tests.
        #[arg(long)]
        limit: Option<usize>,
    },
    /// Open the synchronized multi-model viewer.
    View {
        recording: PathBuf,
        #[arg(long)]
        cache_dir: Option<PathBuf>,
    },
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
        } => {
            let recording_path = resolve_recording(recording);
            let cache_dir = cache_dir.unwrap_or_else(|| default_cache_dir(&recording_path));
            let recording = Recording::open(&recording_path, &cache_dir)?;
            let recorded_predictions = recording
                .load_runs()?
                .into_iter()
                .find(|run| run.key == "recorded")
                .map(|run| run.predictions.into_iter().filter(Option::is_some).count())
                .unwrap_or_default();
            println!(
                "indexed {} frames and {} aligned recorded prediction frames from {} into {}",
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
        } => {
            let recording_path = resolve_recording(recording);
            let cache_dir = cache_dir.unwrap_or_else(|| default_cache_dir(&recording_path));
            let recording = Recording::open(&recording_path, &cache_dir)?;
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
                        manifest.total_frame_count
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
        } => {
            let recording_path = resolve_recording(recording);
            let cache_dir = cache_dir.unwrap_or_else(|| default_cache_dir(&recording_path));
            app::run(Recording::open(recording_path, cache_dir)?)?;
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
