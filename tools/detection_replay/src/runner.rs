use std::{
    fs::File,
    path::{Path, PathBuf},
    sync::{
        Arc,
        atomic::{AtomicU64, Ordering},
    },
    time::{Duration, Instant},
};

use color_eyre::{
    Result,
    eyre::{Context as _, ContextCompat, Report, bail, eyre},
};
use ros_z::prelude::*;
use ros2::sensor_msgs::image::Image;
use tokio::task::JoinHandle;
use types::{
    object_detection::{Object, RobocupObjectLabel, YOLOObjectLabel},
    parameters::{DetectionParameters, ObjectDetectionParameters, PoseDetectionParameters},
    pose_detection::Pose,
    time_wrapper::TimeWrapper,
};

use crate::{
    cache::{DetectionThresholds, ModelRunManifest, Prediction, PredictionStore, hash_model},
    recording::{OriginalFrame, Recording},
};

const PROVIDER_NOTE: &str =
    "detection provider order: WebGPU, then ONNX Runtime implicit CPU fallback";
static NEXT_NAMESPACE: AtomicU64 = AtomicU64::new(0);

#[derive(Clone, Debug)]
pub struct ModelRunConfig {
    pub label: String,
    pub model_path: PathBuf,
    pub thresholds: DetectionThresholds,
    pub frame_limit: Option<usize>,
    pub start_frame: usize,
    pub end_frame: Option<usize>,
    pub startup_timeout: Duration,
    pub output_timeout: Duration,
    /// Cleanup is reported as failed after this duration, but native inference is still joined.
    pub cleanup_warning_after: Duration,
}

impl ModelRunConfig {
    pub fn new(label: impl Into<String>, model_path: impl Into<PathBuf>) -> Self {
        Self {
            label: label.into(),
            model_path: model_path.into(),
            thresholds: DetectionThresholds::default(),
            frame_limit: None,
            start_frame: 0,
            end_frame: None,
            startup_timeout: Duration::from_secs(30),
            output_timeout: Duration::from_secs(30),
            cleanup_warning_after: Duration::from_secs(5),
        }
    }
}

#[derive(Clone, Debug)]
pub struct RunProgress {
    pub label: String,
    pub completed_frames: usize,
    pub target_frames: usize,
    pub total_frames: usize,
}

pub async fn run_model<F>(
    recording: &Recording,
    config: ModelRunConfig,
    mut progress: F,
) -> Result<ModelRunManifest>
where
    F: FnMut(RunProgress),
{
    let cache_directory = recording.cache_directory();
    let (canonical_model_path, model_hash) = hash_model(&config.model_path)?;
    let frame_end = config
        .end_frame
        .unwrap_or_else(|| recording.frame_count().saturating_sub(1));
    validate_frame_range(config.start_frame, frame_end, recording.frame_count())?;
    let frame_count = frame_end - config.start_frame + 1;
    let proposed = ModelRunManifest::new(
        config.label.clone(),
        canonical_model_path,
        model_hash,
        recording.fingerprint().clone(),
        config.thresholds,
        recording.frame_count(),
        config.start_frame..=frame_end,
    )?;
    let mut store = PredictionStore::open(cache_directory, proposed)?;

    let target_frames =
        target_frame_count(frame_count, config.frame_limit, store.completed_count());
    progress(RunProgress {
        label: config.label.clone(),
        completed_frames: store.completed_count(),
        target_frames,
        total_frames: recording.frame_count(),
    });

    if store.completed_count() < target_frames {
        store.begin(PROVIDER_NOTE)?;
        if let Err(error) = execute_run(
            recording,
            cache_directory,
            &config,
            target_frames,
            &mut store,
            &mut progress,
        )
        .await
        {
            return match store.fail(format!("{error:#}")) {
                Ok(()) => Err(error),
                Err(cache_error) => Err(error.wrap_err(format!(
                    "failed to persist model run failure state: {cache_error:#}"
                ))),
            };
        }
    }

    store.finish()?;
    Ok(store.manifest().clone())
}

pub async fn run_model_traced(
    recording: &Recording,
    config: ModelRunConfig,
) -> Result<ModelRunManifest> {
    run_model(recording, config, |progress| {
        tracing::info!(
            label = progress.label,
            completed = progress.completed_frames,
            target = progress.target_frames,
            total = progress.total_frames,
            "detection replay progress"
        );
    })
    .await
}

async fn execute_run<F>(
    recording: &Recording,
    cache_directory: &Path,
    config: &ModelRunConfig,
    target_frames: usize,
    store: &mut PredictionStore,
    progress: &mut F,
) -> Result<()>
where
    F: FnMut(RunProgress),
{
    let parameter_directory = tempfile::tempdir_in(cache_directory)
        .wrap_err("failed to create temporary detection parameter layer")?;
    write_detection_parameters(
        parameter_directory.path(),
        store.manifest(),
        config.thresholds,
    )?;

    let namespace_id = NEXT_NAMESPACE.fetch_add(1, Ordering::Relaxed);
    let namespace = format!("/detection_replay/{}_{namespace_id}", std::process::id());
    let context = Arc::new(
        ContextBuilder::default()
            .with_namespace(namespace)
            .with_mode("peer")
            .disable_multicast_scouting()
            .with_connect_endpoints(std::iter::empty::<String>())
            .with_listen_endpoints(std::iter::empty::<String>())
            .with_parameter_layers([parameter_directory.path()])
            .build()
            .await
            .wrap_err("failed to build isolated detection replay context")?,
    );
    let (model_info_sender, model_info_receiver) = tokio::sync::oneshot::channel();
    let mut detector_task =
        AbortOnDropTask::new(tokio::spawn(detection::run_boxed_with_model_info(
            Arc::clone(&context),
            detection::ExecutionProviderPolicy::WebGpuRequired,
            model_info_sender,
        )));

    let result = async {
        let model_info = tokio::select! {
            result = tokio::time::timeout(config.startup_timeout, model_info_receiver) => {
                result
                    .map_err(|_| eyre!("timed out after {:?} waiting for model information", config.startup_timeout))?
                    .wrap_err("detection node stopped before reporting model information")?
            }
            result = detector_task.join() => return Err(detector_stopped(result)),
        };

        drive_detector(
            recording,
            config,
            target_frames,
            store,
            progress,
            Arc::clone(&context),
            &mut detector_task,
            model_info,
        )
        .await
    }
    .await;

    let cleanup_started = Instant::now();
    detector_task.abort();
    let shutdown_result = context
        .shutdown()
        .wrap_err("failed to shut down detection replay context");
    let remaining_warning_duration = config
        .cleanup_warning_after
        .saturating_sub(cleanup_started.elapsed());
    let task_result = detector_task.stop(remaining_warning_duration).await;
    let mut cleanup_result = match (shutdown_result, task_result) {
        (Ok(()), Ok(())) => Ok(()),
        (Err(error), Ok(())) | (Ok(()), Err(error)) => Err(error),
        (Err(shutdown_error), Err(task_error)) => Err(shutdown_error.wrap_err(format!(
            "detection task cleanup also failed: {task_error:#}"
        ))),
    };
    if cleanup_started.elapsed() > config.cleanup_warning_after {
        cleanup_result = match cleanup_result {
            Ok(()) => Err(eyre!("detection cleanup exceeded its warning threshold")),
            Err(error) => Err(error.wrap_err("detection cleanup exceeded its warning threshold")),
        };
    }
    match (result, cleanup_result) {
        (Ok(()), Ok(())) => Ok(()),
        (Ok(()), Err(error)) => Err(error),
        (Err(error), Ok(())) => Err(error),
        (Err(error), Err(cleanup_error)) => {
            Err(error.wrap_err(format!("detection cleanup also failed: {cleanup_error:#}")))
        }
    }
}

#[allow(clippy::too_many_arguments)]
async fn drive_detector<F>(
    recording: &Recording,
    config: &ModelRunConfig,
    target_frames: usize,
    store: &mut PredictionStore,
    progress: &mut F,
    context: Arc<ros_z::context::Context>,
    detector_task: &mut AbortOnDropTask,
    model_info: detection::DetectionModelInfo,
) -> Result<()>
where
    F: FnMut(RunProgress),
{
    let node = context
        .create_node("detection_replay_driver")
        .build()
        .await
        .wrap_err("failed to create detection replay driver node")?;
    let image_publisher = node
        .publisher::<Image>("inputs/left_image")
        .build()
        .await
        .wrap_err("failed to create replay image publisher")?;
    let object_subscriber = node
        .subscriber::<TimeWrapper<Vec<Object<RobocupObjectLabel>>>>("detected_objects")
        .build()
        .await
        .wrap_err("failed to subscribe to detected_objects")?;
    let pose_subscriber = node
        .subscriber::<TimeWrapper<Vec<Pose<YOLOObjectLabel>>>>("detected_poses")
        .build()
        .await
        .wrap_err("failed to subscribe to detected_poses")?;
    let inference_subscriber = node
        .subscriber::<Duration>("inference_duration")
        .build()
        .await
        .wrap_err("failed to subscribe to inference_duration")?;
    let postprocessing_subscriber = node
        .subscriber::<Duration>("post_processing_duration")
        .build()
        .await
        .wrap_err("failed to subscribe to post_processing_duration")?;
    let nms_subscriber = node
        .subscriber::<Duration>("non_maximum_suppression_duration")
        .build()
        .await
        .wrap_err("failed to subscribe to non_maximum_suppression_duration")?;

    let matched = tokio::select! {
        matched = image_publisher.wait_for_subscribers(1, config.startup_timeout) => matched,
        result = detector_task.join() => return Err(detector_stopped(result)),
    };
    if !matched {
        bail!(
            "detection node did not subscribe to images within {:?}",
            config.startup_timeout
        );
    }

    let resume_frame = config.start_frame + store.completed_count();
    let mut images = recording.original_images(
        resume_frame,
        Some(target_frames.saturating_sub(store.completed_count())),
    )?;
    while store.completed_count() < target_frames {
        let frame = tokio::select! {
            frame = images.next() => frame,
            result = detector_task.join() => return Err(detector_stopped(result)),
        }
        .wrap_err("original image stream ended before the run target")??;

        tokio::select! {
            result = image_publisher.publish(&frame.image) => {
                result.wrap_err_with(|| format!("failed to publish frame {}", frame.frame_index))?;
            }
            result = detector_task.join() => return Err(detector_stopped(result)),
        }

        let (objects, poses, inference, postprocessing, nms) = tokio::select! {
            result = receive_outputs(
                &object_subscriber,
                &pose_subscriber,
                &inference_subscriber,
                &postprocessing_subscriber,
                &nms_subscriber,
                config.output_timeout,
            ) => result?,
            result = detector_task.join() => return Err(detector_stopped(result)),
        };
        verify_output_timestamps(&frame, &objects, &poses)?;
        store.append(Prediction {
            frame_index: frame.frame_index,
            timestamp_nanos: frame.timestamp_nanos,
            objects: objects.inner,
            poses: model_info.has_pose_output.then_some(poses.inner),
            inference_duration_nanos: Some(duration_nanos(inference)),
            postprocessing_duration_nanos: Some(duration_nanos(postprocessing)),
            non_maximum_suppression_duration_nanos: Some(duration_nanos(nms)),
        })?;
        progress(RunProgress {
            label: config.label.clone(),
            completed_frames: store.completed_count(),
            target_frames,
            total_frames: recording.frame_count(),
        });
    }
    Ok(())
}

struct AbortOnDropTask {
    handle: JoinHandle<Result<()>>,
    joined: bool,
}

impl AbortOnDropTask {
    fn new(handle: JoinHandle<Result<()>>) -> Self {
        Self {
            handle,
            joined: false,
        }
    }

    async fn join(&mut self) -> std::result::Result<Result<()>, tokio::task::JoinError> {
        let result = (&mut self.handle).await;
        self.joined = true;
        result
    }

    fn abort(&self) {
        self.handle.abort();
    }

    async fn stop(mut self, warning_after: Duration) -> Result<()> {
        if self.joined {
            return Ok(());
        }
        self.handle.abort();
        match tokio::time::timeout(warning_after, &mut self.handle).await {
            Ok(result) => classify_cleanup_join(result, false),
            Err(_) => classify_cleanup_join((&mut self.handle).await, true),
        }
    }
}

impl Drop for AbortOnDropTask {
    fn drop(&mut self) {
        self.handle.abort();
    }
}

fn classify_cleanup_join(
    result: std::result::Result<Result<()>, tokio::task::JoinError>,
    timed_out: bool,
) -> Result<()> {
    let result = match result {
        Err(error) if error.is_cancelled() => Ok(()),
        Err(error) => Err(eyre!(error).wrap_err("detection task failed during cleanup")),
        Ok(result) => result.wrap_err("detection node failed during cleanup"),
    };
    if timed_out {
        match result {
            Ok(()) => bail!("detection cleanup exceeded its warning threshold"),
            Err(error) => Err(error.wrap_err("detection cleanup exceeded its warning threshold")),
        }
    } else {
        result
    }
}

async fn receive_outputs(
    object_subscriber: &Subscriber<TimeWrapper<Vec<Object<RobocupObjectLabel>>>>,
    pose_subscriber: &Subscriber<TimeWrapper<Vec<Pose<YOLOObjectLabel>>>>,
    inference_subscriber: &Subscriber<Duration>,
    postprocessing_subscriber: &Subscriber<Duration>,
    nms_subscriber: &Subscriber<Duration>,
    timeout: Duration,
) -> Result<(
    TimeWrapper<Vec<Object<RobocupObjectLabel>>>,
    TimeWrapper<Vec<Pose<YOLOObjectLabel>>>,
    Duration,
    Duration,
    Duration,
)> {
    tokio::time::timeout(timeout, async {
        tokio::try_join!(
            object_subscriber.recv(),
            pose_subscriber.recv(),
            inference_subscriber.recv(),
            postprocessing_subscriber.recv(),
            nms_subscriber.recv(),
        )
    })
    .await
    .map_err(|_| eyre!("timed out after {timeout:?} waiting for detection outputs"))?
    .wrap_err("failed to receive detection outputs")
}

fn verify_output_timestamps(
    frame: &OriginalFrame,
    objects: &TimeWrapper<Vec<Object<RobocupObjectLabel>>>,
    poses: &TimeWrapper<Vec<Pose<YOLOObjectLabel>>>,
) -> Result<()> {
    let object_time = objects.time.as_nanos();
    let pose_time = poses.time.as_nanos();
    if object_time != frame.timestamp_nanos || pose_time != frame.timestamp_nanos {
        bail!(
            "frame {} output timestamps do not match image timestamp {} (objects {}, poses {})",
            frame.frame_index,
            frame.timestamp_nanos,
            object_time,
            pose_time
        );
    }
    Ok(())
}

fn write_detection_parameters(
    directory: &Path,
    manifest: &ModelRunManifest,
    thresholds: DetectionThresholds,
) -> Result<()> {
    let parent = manifest
        .canonical_model_path
        .parent()
        .wrap_err("canonical model path has no parent")?;
    let model_name = manifest
        .canonical_model_path
        .file_name()
        .and_then(|name| name.to_str())
        .wrap_err("model file name is not valid UTF-8")?;
    let parameters = DetectionParameters {
        enable: true,
        neural_networks_folder: parent.to_path_buf(),
        model_name: model_name.to_string(),
        object_detection_parameters: ObjectDetectionParameters {
            maximum_intersection_over_union: thresholds.maximum_intersection_over_union,
            minimum_candidate_confidence: thresholds.minimum_candidate_confidence,
        },
        pose_detection_parameters: PoseDetectionParameters {
            maximum_intersection_over_union: thresholds.maximum_intersection_over_union,
            minimum_candidate_confidence: thresholds.minimum_candidate_confidence,
        },
    };
    let path = directory.join("detection.json5");
    let mut file =
        File::create(&path).wrap_err_with(|| format!("failed to create {}", path.display()))?;
    serde_json::to_writer_pretty(&mut file, &parameters)
        .wrap_err_with(|| format!("failed to write {}", path.display()))?;
    file.sync_all()
        .wrap_err_with(|| format!("failed to sync {}", path.display()))?;
    Ok(())
}

fn validate_frame_range(start: usize, end: usize, frame_count: usize) -> Result<()> {
    if start > end {
        bail!("start frame {start} is greater than end frame {end}");
    }
    if end >= frame_count {
        bail!("end frame {end} is out of range for {frame_count} frames");
    }
    Ok(())
}

fn target_frame_count(frame_count: usize, limit: Option<usize>, completed: usize) -> usize {
    limit.unwrap_or(frame_count).min(frame_count).max(completed)
}

fn detector_stopped(result: std::result::Result<Result<()>, tokio::task::JoinError>) -> Report {
    match result {
        Ok(Ok(())) => eyre!("detection node stopped unexpectedly"),
        Ok(Err(error)) => error.wrap_err("detection node failed"),
        Err(error) => eyre!(error).wrap_err("detection node task failed"),
    }
}

fn duration_nanos(duration: Duration) -> u64 {
    duration.as_nanos().min(u128::from(u64::MAX)) as u64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_match_replay_requirements() {
        let config = ModelRunConfig::new("model", "model.onnx");
        assert_eq!(config.thresholds.minimum_candidate_confidence, 0.05);
        assert_eq!(config.thresholds.maximum_intersection_over_union, 0.4);
        assert_eq!(config.cleanup_warning_after, Duration::from_secs(5));
    }

    #[test]
    fn progress_target_never_precedes_cached_frames() {
        assert_eq!(target_frame_count(100, Some(10), 80), 80);
        assert_eq!(target_frame_count(100, Some(10), 5), 10);
        assert_eq!(target_frame_count(100, None, 5), 100);
    }

    #[test]
    fn detector_task_cleanup_waits_after_timeout() {
        let runtime = tokio::runtime::Builder::new_multi_thread()
            .enable_all()
            .build()
            .unwrap();
        runtime.block_on(async {
            let (started_sender, started_receiver) = tokio::sync::oneshot::channel();
            let task = AbortOnDropTask::new(tokio::spawn(async move {
                tokio::task::block_in_place(|| {
                    let _ = started_sender.send(());
                    std::thread::sleep(Duration::from_millis(30));
                });
                Ok(())
            }));
            started_receiver.await.unwrap();
            let start = std::time::Instant::now();
            let error = task.stop(Duration::from_millis(1)).await.unwrap_err();
            assert!(start.elapsed() >= Duration::from_millis(20));
            assert!(error.to_string().contains("exceeded its warning threshold"));
        });
    }
}
