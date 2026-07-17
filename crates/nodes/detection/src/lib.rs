use std::{boxed::Box, future::Future, path::Path, pin::Pin, sync::Arc, time::Duration};

use color_eyre::{Result, eyre::bail, eyre::eyre};
use ndarray::{ArrayView2, ArrayView3, ArrayViewD, Axis};
#[cfg(feature = "webgpu-provider")]
use ort::execution_providers::WebGPUExecutionProvider;
#[cfg(feature = "nvidia")]
use ort::execution_providers::{CUDAExecutionProvider, TensorRTExecutionProvider};
use ort::{
    execution_providers::{CPUExecutionProvider, ExecutionProvider, ExecutionProviderDispatch},
    inputs,
    session::{Session, SessionOutputs, builder::GraphOptimizationLevel},
    value::TensorRef,
};
use ros_z_streams::CreateAnnouncingPublisher;
use ros2::sensor_msgs::image::Image;

use ros_z::prelude::*;
use tokio::{sync::oneshot, task::block_in_place, time::Instant};
use types::{
    bounding_box::BoundingBox,
    object_detection::{NUMBER_OF_VALUES_PER_OBJECT, Object, RobocupObjectLabel, YOLOObjectLabel},
    parameters::DetectionParameters,
    pose_detection::{
        FieldFeatureDetection, FieldFeatureLabel, NUMBER_OF_VALUES_PER_POSE, Pose, RobotKeypoints,
        RobotPoseDetection,
    },
    time_wrapper::TimeWrapper,
};

pub const NUMBER_OF_DETECTIONS: usize = 300;

#[derive(Clone, Copy, Debug, Default)]
/// Selects the ordered ONNX Runtime providers registered for a detection session.
pub enum ExecutionProviderPolicy {
    /// Try TensorRT, CUDA, WebGPU, then ORT's implicit CPU fallback when compiled in.
    #[default]
    Automatic,
    /// Require WebGPU registration while still permitting per-operator CPU fallback.
    WebGpuRequired,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// Capabilities reported after the ONNX session has been created.
pub struct DetectionModelInfo {
    pub has_pose_output: bool,
    pub has_robot_pose_output: bool,
    pub has_field_feature_output: bool,
}

#[derive(Clone, Copy, Debug)]
enum TaskHead {
    ObjectDetection,
    LegacyPose,
    PersonPose,
    RobotPose,
    FieldFeature,
}

struct DetectionOutput {
    inference_duration: Duration,
    post_processing_duration: Duration,
    non_maximum_suppression_duration: Duration,
    detected_objects: Vec<Object<RobocupObjectLabel>>,
    detected_poses: Vec<Pose<YOLOObjectLabel>>,
    detected_robot_poses: Vec<RobotPoseDetection>,
    detected_field_features: Vec<FieldFeatureDetection>,
}

impl TaskHead {
    fn output_name(self) -> &'static str {
        match self {
            TaskHead::ObjectDetection => "object_output",
            TaskHead::LegacyPose => "pose_output",
            TaskHead::PersonPose => "person_pose_output",
            TaskHead::RobotPose => "robot_pose_output",
            TaskHead::FieldFeature => "field_feature_output",
        }
    }

    fn expected_shape(self) -> &'static [usize] {
        match self {
            Self::ObjectDetection => &[1, NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_OBJECT],
            Self::LegacyPose | Self::PersonPose => {
                &[1, NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_POSE]
            }
            Self::RobotPose => &[1, NUMBER_OF_DETECTIONS, 14, 3],
            Self::FieldFeature => &[1, NUMBER_OF_DETECTIONS, 4],
        }
    }
}

#[derive(Debug)]
struct ModelOutputs<'a> {
    objects: ArrayView2<'a, f32>,
    poses: Option<ArrayView2<'a, f32>>,
    robot_poses: Option<ArrayView3<'a, f32>>,
    field_features: Option<ArrayView2<'a, f32>>,
}

pub fn run_boxed(ctx: Arc<Context>) -> Pin<Box<dyn Future<Output = Result<()>> + Send>> {
    run_boxed_with_provider_policy(ctx, ExecutionProviderPolicy::Automatic)
}

/// Runs detection with an explicit provider-registration policy.
pub fn run_boxed_with_provider_policy(
    ctx: Arc<Context>,
    provider_policy: ExecutionProviderPolicy,
) -> Pin<Box<dyn Future<Output = Result<()>> + Send>> {
    Box::pin(run(ctx, provider_policy, None))
}

/// Runs detection and reports model capabilities once session creation succeeds.
///
/// The one-shot notification does not indicate that ROS-Z publishers and subscribers are ready.
pub fn run_boxed_with_model_info(
    ctx: Arc<Context>,
    provider_policy: ExecutionProviderPolicy,
    model_info_sender: oneshot::Sender<DetectionModelInfo>,
) -> Pin<Box<dyn Future<Output = Result<()>> + Send>> {
    Box::pin(run(ctx, provider_policy, Some(model_info_sender)))
}

async fn run(
    ctx: Arc<Context>,
    provider_policy: ExecutionProviderPolicy,
    model_info_sender: Option<oneshot::Sender<DetectionModelInfo>>,
) -> Result<()> {
    let node = ctx.create_node("detection").build().await?;

    let node_parameters = node.bind_parameter_as::<DetectionParameters>("detection")?;
    let mut parameter_receiver = node_parameters.subscribe();

    let image_sub = node
        .subscriber::<Image>("inputs/left_image")
        .build()
        .await?;
    let inference_duration_pub = node
        .publisher::<Duration>("inference_duration")
        .build()
        .await?;
    let post_processing_duration_pub = node
        .publisher::<Duration>("post_processing_duration")
        .build()
        .await?;
    let non_maximum_suppression_duration_pub = node
        .publisher::<Duration>("non_maximum_suppression_duration")
        .build()
        .await?;
    let detected_objects_pub = node
        .announcing_publisher::<TimeWrapper<Vec<Object<RobocupObjectLabel>>>>("detected_objects")
        .await?;
    let detected_poses_pub = node
        .announcing_publisher::<TimeWrapper<Vec<Pose<YOLOObjectLabel>>>>("detected_poses")
        .await?;
    let detected_robot_poses_pub = node
        .announcing_publisher::<TimeWrapper<Vec<RobotPoseDetection>>>("detected_robot_poses")
        .await?;
    let detected_field_features_pub = node
        .announcing_publisher::<TimeWrapper<Vec<FieldFeatureDetection>>>("detected_field_features")
        .await?;

    let initial_parameters_snapshot = node_parameters.snapshot();
    let parameters = initial_parameters_snapshot.typed();
    let model_path = parameters
        .neural_networks_folder
        .join(&parameters.model_name);

    let execution_providers =
        execution_providers(&parameters.neural_networks_folder, provider_policy)?;

    let mut session = block_in_place(|| {
        Session::builder()?
            .with_execution_providers(execution_providers)?
            .with_optimization_level(GraphOptimizationLevel::Level3)?
            .with_intra_threads(2)?
            .commit_from_file(model_path)
    })?;
    let model_info =
        model_info_from_output_names(session.outputs.iter().map(|output| &*output.name));
    if let Some(sender) = model_info_sender {
        let _ = sender.send(model_info);
    }

    loop {
        parameter_receiver
            .wait_for(|parameters| parameters.typed().enable)
            .await?;

        let image = image_sub.recv().await?;

        let parameter_snapshot = node_parameters.snapshot();
        let parameters = parameter_snapshot.typed();
        if !parameters.enable {
            continue;
        }

        let image_time = image.header.stamp.into();
        let detected_objects_pending = detected_objects_pub.announce(image_time).await?;
        let detected_poses_pending = detected_poses_pub.announce(image_time).await?;
        let detected_robot_poses_pending = detected_robot_poses_pub.announce(image_time).await?;
        let detected_field_features_pending =
            detected_field_features_pub.announce(image_time).await?;

        check_image(&image)?;

        let output = block_in_place(|| {
            let inference_start = Instant::now();

            let nv12_data = ArrayView3::from_shape(
                [image.height as usize / 2, image.width as usize / 2, 6],
                &image.data,
            )?;
            let outputs: SessionOutputs = session
                .run(inputs!["raw_bytes_input" => TensorRef::from_array_view(nv12_data)?])?;

            let inference_duration = inference_start.elapsed();

            let post_processing_start = Instant::now();

            let outputs = extract_outputs(&outputs)?;
            let candidate_detections = extract_candidate_object_detections(
                &outputs,
                parameters
                    .object_detection_parameters
                    .minimum_candidate_confidence,
            )?;
            let candidate_human_poses = extract_candidate_pose_detections(
                &outputs,
                parameters
                    .pose_detection_parameters
                    .minimum_candidate_confidence,
                parameters.pose_detection_parameters.visibility_score_alpha,
            )?;
            let detected_robot_poses = extract_robot_pose_detections(
                &outputs,
                parameters
                    .robot_pose_detection_parameters
                    .minimum_candidate_confidence,
                parameters
                    .robot_pose_detection_parameters
                    .visibility_score_alpha,
            )?;
            let detected_field_features = extract_field_feature_detections(
                &outputs,
                parameters
                    .field_feature_detection_parameters
                    .minimum_candidate_confidence,
            );
            let post_processing_duration = post_processing_start.elapsed();
            let non_maximum_suppression_start = Instant::now();
            let detected_objects = non_maximum_suppression(
                candidate_detections,
                parameters
                    .object_detection_parameters
                    .maximum_intersection_over_union,
            );
            let detected_poses = non_maximum_suppression(
                candidate_human_poses,
                parameters
                    .pose_detection_parameters
                    .maximum_intersection_over_union,
            );
            let detected_robot_poses = non_maximum_suppression(
                detected_robot_poses,
                parameters
                    .robot_pose_detection_parameters
                    .maximum_intersection_over_union,
            );
            let detected_field_features = suppress_field_features(
                detected_field_features,
                parameters
                    .field_feature_detection_parameters
                    .maximum_suppression_distance_in_pixels,
            );
            let non_maximum_suppression_duration = non_maximum_suppression_start.elapsed();

            Ok::<_, color_eyre::eyre::Error>(DetectionOutput {
                inference_duration,
                post_processing_duration,
                non_maximum_suppression_duration,
                detected_objects,
                detected_poses,
                detected_robot_poses,
                detected_field_features,
            })
        })?;

        inference_duration_pub
            .publish(&output.inference_duration)
            .await?;
        post_processing_duration_pub
            .publish(&output.post_processing_duration)
            .await?;
        non_maximum_suppression_duration_pub
            .publish(&output.non_maximum_suppression_duration)
            .await?;

        detected_objects_pending
            .publish(&TimeWrapper {
                time: image_time,
                inner: output.detected_objects,
            })
            .await?;
        detected_poses_pending
            .publish(&TimeWrapper {
                time: image_time,
                inner: output.detected_poses,
            })
            .await?;
        detected_robot_poses_pending
            .publish(&TimeWrapper {
                time: image_time,
                inner: output.detected_robot_poses,
            })
            .await?;
        detected_field_features_pending
            .publish(&TimeWrapper {
                time: image_time,
                inner: output.detected_field_features,
            })
            .await?;
    }
}

fn model_info_from_output_names<'a>(
    names: impl IntoIterator<Item = &'a str>,
) -> DetectionModelInfo {
    let mut model_info = DetectionModelInfo {
        has_pose_output: false,
        has_robot_pose_output: false,
        has_field_feature_output: false,
    };
    for name in names {
        model_info.has_pose_output |= name == TaskHead::LegacyPose.output_name()
            || name == TaskHead::PersonPose.output_name();
        model_info.has_robot_pose_output |= name == TaskHead::RobotPose.output_name();
        model_info.has_field_feature_output |= name == TaskHead::FieldFeature.output_name();
    }
    model_info
}

fn execution_providers(
    _neural_networks_folder: &Path,
    policy: ExecutionProviderPolicy,
) -> Result<Vec<ExecutionProviderDispatch>> {
    #[allow(unused_mut)]
    let mut providers = Vec::new();

    #[cfg(feature = "nvidia")]
    if matches!(policy, ExecutionProviderPolicy::Automatic) {
        let tensor_rt = TensorRTExecutionProvider::default()
            .with_device_id(0)
            .with_fp16(true)
            .with_engine_cache(true)
            .with_engine_cache_path(_neural_networks_folder.display());
        log_provider_availability(&tensor_rt);
        providers.push(tensor_rt.build());

        let cuda = CUDAExecutionProvider::default();
        log_provider_availability(&cuda);
        providers.push(cuda.build());
    }

    #[cfg(feature = "webgpu-provider")]
    {
        let webgpu = WebGPUExecutionProvider::default();
        log_provider_availability(&webgpu);
        let webgpu = webgpu.build();
        let webgpu = if matches!(policy, ExecutionProviderPolicy::WebGpuRequired) {
            webgpu.error_on_failure()
        } else {
            webgpu
        };
        providers.push(webgpu);
    }

    #[cfg(not(feature = "webgpu-provider"))]
    if matches!(policy, ExecutionProviderPolicy::WebGpuRequired) {
        bail!("WebGPU was required but detection was built without its WebGPU feature");
    }

    // CPU is ORT's implicit final fallback and must not be explicitly registered.
    log_provider_availability(&CPUExecutionProvider::default());

    Ok(providers)
}

fn log_provider_availability(provider: &impl ExecutionProvider) {
    match provider.is_available() {
        Ok(available) => tracing::info!(
            provider = provider.name(),
            available,
            "ONNX Runtime execution provider availability"
        ),
        Err(error) => tracing::warn!(
            provider = provider.name(),
            ?error,
            "failed to query ONNX Runtime execution provider availability"
        ),
    }
}

fn check_image(image: &Image) -> Result<()> {
    if image.encoding != "nv12" {
        bail!("unsupported image encoding: {}", image.encoding);
    }

    if !image.width.is_multiple_of(32) || !image.height.is_multiple_of(32) {
        bail!(
            "image dimensions must be multiples of 32 (got {}x{})",
            image.width,
            image.height
        );
    }

    Ok(())
}

fn extract_outputs<'a>(outputs: &'a SessionOutputs<'a>) -> Result<ModelOutputs<'a>> {
    model_outputs_from_arrays(
        extract_output(outputs, TaskHead::ObjectDetection)?,
        extract_output(outputs, TaskHead::LegacyPose)?,
        extract_output(outputs, TaskHead::PersonPose)?,
        extract_output(outputs, TaskHead::RobotPose)?,
        extract_output(outputs, TaskHead::FieldFeature)?,
    )
}

fn extract_output<'a>(
    outputs: &'a SessionOutputs<'a>,
    task_head: TaskHead,
) -> Result<Option<ArrayViewD<'a, f32>>> {
    outputs
        .get(task_head.output_name())
        .map(|output| {
            output.try_extract_array::<f32>().map_err(|error| {
                eyre!(error).wrap_err(format!(
                    "failed to extract model output `{}`",
                    task_head.output_name()
                ))
            })
        })
        .transpose()
}

fn model_outputs_from_arrays<'a>(
    objects_output: Option<ArrayViewD<'a, f32>>,
    legacy_poses_output: Option<ArrayViewD<'a, f32>>,
    person_poses_output: Option<ArrayViewD<'a, f32>>,
    robot_poses_output: Option<ArrayViewD<'a, f32>>,
    field_features_output: Option<ArrayViewD<'a, f32>>,
) -> Result<ModelOutputs<'a>> {
    let objects_output = objects_output.ok_or_else(|| {
        eyre!(
            "mandatory model output `{}` is missing",
            TaskHead::ObjectDetection.output_name()
        )
    })?;
    let objects = validate_and_reshape_output(TaskHead::ObjectDetection, objects_output)?;
    let poses = if let Some(output) = person_poses_output {
        Some(validate_and_reshape_output(TaskHead::PersonPose, output)?)
    } else {
        legacy_poses_output
            .map(|output| validate_and_reshape_output(TaskHead::LegacyPose, output))
            .transpose()?
    };
    let robot_poses = robot_poses_output
        .map(validate_and_reshape_robot_output)
        .transpose()?;
    let field_features = field_features_output
        .map(|output| validate_and_reshape_output(TaskHead::FieldFeature, output))
        .transpose()?;

    Ok(ModelOutputs {
        objects,
        poses,
        robot_poses,
        field_features,
    })
}

fn validate_and_reshape_output<'a>(
    task_head: TaskHead,
    output: ArrayViewD<'a, f32>,
) -> Result<ArrayView2<'a, f32>> {
    if output.shape() != task_head.expected_shape() {
        bail!(
            "{} not of expected shape. Expected: {:?}, got: {:?}",
            task_head.output_name(),
            task_head.expected_shape(),
            output.shape()
        )
    }

    Ok(output.squeeze().into_dimensionality()?)
}

fn validate_and_reshape_robot_output(output: ArrayViewD<'_, f32>) -> Result<ArrayView3<'_, f32>> {
    let task_head = TaskHead::RobotPose;
    if output.shape() != task_head.expected_shape() {
        bail!(
            "{} not of expected shape. Expected: {:?}, got: {:?}",
            task_head.output_name(),
            task_head.expected_shape(),
            output.shape()
        )
    }

    Ok(output.squeeze().into_dimensionality()?)
}

fn extract_candidate_object_detections(
    outputs: &ModelOutputs,
    confidence_threshold: f32,
) -> Result<Vec<Object<RobocupObjectLabel>>> {
    Ok(outputs
        .objects
        .axis_iter(Axis(0))
        .filter_map(|row| {
            let confidence = row[4usize];
            if !confidence.is_finite() || confidence < confidence_threshold {
                return None;
            }

            let object_values: [f32; NUMBER_OF_VALUES_PER_OBJECT] = row
                .as_slice()
                .expect("slice is not contiguous")
                .try_into()
                .unwrap_or_else(|_| {
                    panic!("slice is not of length {}", NUMBER_OF_VALUES_PER_OBJECT)
                });

            Some(Object::from(object_values))
        })
        .collect())
}

fn extract_candidate_pose_detections(
    outputs: &ModelOutputs,
    confidence_threshold: f32,
    visibility_score_alpha: f32,
) -> Result<Vec<Pose<YOLOObjectLabel>>> {
    if !visibility_score_alpha.is_finite() || visibility_score_alpha < 0.0 {
        bail!("person pose visibility score alpha must be finite and non-negative");
    }
    let Some(poses) = &outputs.poses else {
        return Ok(Vec::new());
    };

    poses
        .axis_iter(Axis(0))
        .filter_map(|row| {
            let base_confidence = row[4usize];
            if !base_confidence.is_finite() {
                return None;
            }
            let confidence = if visibility_score_alpha == 0.0 {
                base_confidence
            } else {
                let values = row.as_slice().expect("slice is not contiguous");
                let keypoints = values[NUMBER_OF_VALUES_PER_OBJECT..].chunks_exact(3);
                let keypoint_count = keypoints.len();
                if keypoint_count == 0 {
                    return None;
                }
                let mut visibility_sum = 0.0;
                for keypoint in keypoints {
                    let visibility = keypoint[2];
                    if !visibility.is_finite() || !(0.0..=1.0).contains(&visibility) {
                        return None;
                    }
                    visibility_sum += visibility;
                }
                base_confidence
                    * (visibility_sum / keypoint_count as f32).powf(visibility_score_alpha)
            };
            if !confidence.is_finite() || confidence < confidence_threshold {
                return None;
            }

            let mut pose_values: [f32; NUMBER_OF_VALUES_PER_POSE] = row
                .as_slice()
                .expect("slice is not contiguous")
                .try_into()
                .unwrap_or_else(|_| panic!("slice is not of length {}", NUMBER_OF_VALUES_PER_POSE));
            pose_values[4] = confidence;

            Some(Ok(Pose::from(&pose_values)))
        })
        .collect()
}

fn extract_robot_pose_detections(
    outputs: &ModelOutputs,
    confidence_threshold: f32,
    visibility_score_alpha: f32,
) -> Result<Vec<RobotPoseDetection>> {
    if !visibility_score_alpha.is_finite() || visibility_score_alpha < 0.0 {
        bail!("robot pose visibility score alpha must be finite and non-negative");
    }
    let Some(robot_poses) = &outputs.robot_poses else {
        return Ok(Vec::new());
    };

    fn calibrated_confidence(
        object_confidence: f32,
        keypoint_rows: &ArrayView2<f32>,
        visibility_score_alpha: f32,
    ) -> Option<f32> {
        if !object_confidence.is_finite() {
            return None;
        }
        if visibility_score_alpha == 0.0 {
            return Some(object_confidence);
        }
        let mut visibility_sum = 0.0;
        for keypoint in keypoint_rows.axis_iter(Axis(0)) {
            let visibility = keypoint[2];
            if !visibility.is_finite() || !(0.0..=1.0).contains(&visibility) {
                return None;
            }
            visibility_sum += visibility;
        }
        let keypoint_count = keypoint_rows.len_of(Axis(0));
        if keypoint_count == 0 {
            return None;
        }
        let mean_visibility = visibility_sum / keypoint_count as f32;
        let confidence = object_confidence * mean_visibility.powf(visibility_score_alpha);
        confidence.is_finite().then_some(confidence)
    }

    outputs
        .objects
        .axis_iter(Axis(0))
        .zip(robot_poses.axis_iter(Axis(0)))
        .filter_map(|(object_row, keypoint_rows)| {
            let class_index = object_row[5] as usize;
            if class_index != RobocupObjectLabel::Robot as usize {
                return None;
            }
            let confidence =
                calibrated_confidence(object_row[4], &keypoint_rows, visibility_score_alpha)?;
            (confidence >= confidence_threshold).then_some((object_row, keypoint_rows, confidence))
        })
        .map(|(object_row, keypoint_rows, confidence)| {
            let mut object_values: [f32; NUMBER_OF_VALUES_PER_OBJECT] = object_row
                .as_slice()
                .expect("slice is not contiguous")
                .try_into()
                .expect("object row has invalid length");
            object_values[4] = confidence;
            let keypoint_values: [f32; 42] = keypoint_rows
                .as_slice()
                .expect("slice is not contiguous")
                .try_into()
                .expect("robot keypoints have invalid length");
            Ok(RobotPoseDetection {
                object: Object::from(object_values),
                keypoints: RobotKeypoints::from(&keypoint_values),
            })
        })
        .collect()
}

fn extract_field_feature_detections(
    outputs: &ModelOutputs,
    confidence_threshold: f32,
) -> Vec<FieldFeatureDetection> {
    let Some(field_features) = &outputs.field_features else {
        return Vec::new();
    };

    field_features
        .axis_iter(Axis(0))
        .filter_map(|row| {
            let confidence = row[2];
            (confidence.is_finite() && confidence >= confidence_threshold).then(|| {
                FieldFeatureDetection {
                    point: linear_algebra::point![row[0], row[1]],
                    confidence,
                    label: FieldFeatureLabel::from_index(row[3] as usize),
                }
            })
        })
        .collect()
}

trait HasBoundingBox {
    fn bounding_box(&self) -> &BoundingBox;
}

impl<T> HasBoundingBox for Object<T> {
    fn bounding_box(&self) -> &BoundingBox {
        &self.bounding_box
    }
}

impl<T> HasBoundingBox for Pose<T> {
    fn bounding_box(&self) -> &BoundingBox {
        &self.object.bounding_box
    }
}

impl HasBoundingBox for RobotPoseDetection {
    fn bounding_box(&self) -> &BoundingBox {
        &self.object.bounding_box
    }
}

fn non_maximum_suppression<T: HasBoundingBox>(
    mut sorted_candidate_detections: Vec<T>,
    maximum_intersection_over_union: f32,
) -> Vec<T> {
    sorted_candidate_detections.sort_by(|detection1, detection2| {
        detection1
            .bounding_box()
            .confidence
            .total_cmp(&detection2.bounding_box().confidence)
    });

    let mut remaining_detections = Vec::new();

    while let Some(detection) = sorted_candidate_detections.pop() {
        sorted_candidate_detections.retain(|detection_candidate| {
            detection
                .bounding_box()
                .intersection_over_union(detection_candidate.bounding_box())
                < maximum_intersection_over_union
        });

        remaining_detections.push(detection)
    }

    remaining_detections
}

fn suppress_field_features(
    mut candidates: Vec<FieldFeatureDetection>,
    maximum_distance: f32,
) -> Vec<FieldFeatureDetection> {
    candidates.sort_by(|left, right| left.confidence.total_cmp(&right.confidence));
    let mut detections = Vec::new();
    while let Some(detection) = candidates.pop() {
        candidates.retain(|candidate| {
            candidate.label != detection.label
                || (candidate.point - detection.point).norm() > maximum_distance
        });
        detections.push(detection);
    }
    detections
}

#[cfg(test)]
mod tests {
    use ndarray::{Array2, Array3, Array4};

    use super::*;

    #[test]
    fn missing_pose_output_produces_no_pose_candidates() {
        let objects = Array3::zeros((1, NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_OBJECT));
        let outputs =
            model_outputs_from_arrays(Some(objects.view().into_dyn()), None, None, None, None)
                .unwrap();

        let poses = extract_candidate_pose_detections(&outputs, 0.0, 0.0).unwrap();

        assert!(poses.is_empty());
        assert!(
            extract_robot_pose_detections(&outputs, 0.0, 0.0)
                .unwrap()
                .is_empty()
        );
        assert!(extract_field_feature_detections(&outputs, 0.0).is_empty());
    }

    #[test]
    fn model_info_distinguishes_pose_capability() {
        assert_eq!(
            model_info_from_output_names(["object_output"]),
            DetectionModelInfo {
                has_pose_output: false,
                has_robot_pose_output: false,
                has_field_feature_output: false,
            }
        );
        assert_eq!(
            model_info_from_output_names(["object_output", "pose_output"]),
            DetectionModelInfo {
                has_pose_output: true,
                has_robot_pose_output: false,
                has_field_feature_output: false,
            }
        );
        assert_eq!(
            model_info_from_output_names(["object_output", "person_pose_output"]),
            DetectionModelInfo {
                has_pose_output: true,
                has_robot_pose_output: false,
                has_field_feature_output: false,
            }
        );
        assert_eq!(
            model_info_from_output_names(["object_output", "field_feature_output"]),
            DetectionModelInfo {
                has_pose_output: false,
                has_robot_pose_output: false,
                has_field_feature_output: true,
            }
        );
        assert_eq!(
            model_info_from_output_names(["object_output", "robot_pose_output"]),
            DetectionModelInfo {
                has_pose_output: false,
                has_robot_pose_output: true,
                has_field_feature_output: false,
            }
        );
    }

    #[test]
    fn object_output_is_mandatory() {
        let error = model_outputs_from_arrays(None, None, None, None, None).unwrap_err();

        assert!(error.to_string().contains("`object_output` is missing"));
    }

    #[test]
    fn object_output_must_have_expected_shape() {
        let objects =
            Array3::<f32>::zeros((1, NUMBER_OF_DETECTIONS - 1, NUMBER_OF_VALUES_PER_OBJECT));

        let error =
            model_outputs_from_arrays(Some(objects.view().into_dyn()), None, None, None, None)
                .unwrap_err();

        assert!(
            error
                .to_string()
                .contains("object_output not of expected shape")
        );
    }

    #[test]
    fn legacy_pose_output_is_validated_and_reshaped() {
        let objects = Array3::zeros((1, NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_OBJECT));
        let poses = Array3::zeros((1, NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_POSE));

        let outputs = model_outputs_from_arrays(
            Some(objects.view().into_dyn()),
            Some(poses.view().into_dyn()),
            None,
            None,
            None,
        )
        .unwrap();

        assert_eq!(
            outputs.objects.shape(),
            [NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_OBJECT]
        );
        assert_eq!(
            outputs.poses.as_ref().unwrap().shape(),
            [NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_POSE]
        );
    }

    #[test]
    fn multitask_outputs_are_validated_and_reshaped() {
        let objects = Array3::zeros((1, NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_OBJECT));
        let poses = Array3::zeros((1, NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_POSE));
        let robot_poses = Array4::zeros((1, NUMBER_OF_DETECTIONS, 14, 3));
        let field_features = Array3::zeros((1, NUMBER_OF_DETECTIONS, 4));

        let outputs = model_outputs_from_arrays(
            Some(objects.view().into_dyn()),
            None,
            Some(poses.view().into_dyn()),
            Some(robot_poses.view().into_dyn()),
            Some(field_features.view().into_dyn()),
        )
        .unwrap();

        assert_eq!(
            outputs.poses.as_ref().unwrap().shape(),
            [NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_POSE]
        );
        assert_eq!(
            outputs.robot_poses.as_ref().unwrap().shape(),
            [NUMBER_OF_DETECTIONS, 14, 3]
        );
        assert_eq!(
            outputs.field_features.as_ref().unwrap().shape(),
            [NUMBER_OF_DETECTIONS, 4]
        );
    }

    #[test]
    fn present_legacy_pose_output_must_have_expected_shape() {
        let objects = Array3::zeros((1, NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_OBJECT));
        let poses = Array3::<f32>::zeros((1, NUMBER_OF_DETECTIONS - 1, NUMBER_OF_VALUES_PER_POSE));

        let error = model_outputs_from_arrays(
            Some(objects.view().into_dyn()),
            Some(poses.view().into_dyn()),
            None,
            None,
            None,
        )
        .unwrap_err();

        assert!(
            error
                .to_string()
                .contains("pose_output not of expected shape")
        );
    }

    #[test]
    fn deployment_output_contract_has_four_named_shapes() {
        assert_eq!(TaskHead::ObjectDetection.output_name(), "object_output");
        assert_eq!(TaskHead::LegacyPose.output_name(), "pose_output");
        assert_eq!(TaskHead::PersonPose.output_name(), "person_pose_output");
        assert_eq!(TaskHead::RobotPose.output_name(), "robot_pose_output");
        assert_eq!(TaskHead::FieldFeature.output_name(), "field_feature_output");
        assert_eq!(TaskHead::ObjectDetection.expected_shape(), &[1, 300, 6]);
        assert_eq!(TaskHead::PersonPose.expected_shape(), &[1, 300, 57]);
        assert_eq!(TaskHead::RobotPose.expected_shape(), &[1, 300, 14, 3]);
        assert_eq!(TaskHead::FieldFeature.expected_shape(), &[1, 300, 4]);
    }

    #[test]
    fn field_feature_output_routes_class_and_confidence() {
        let objects = Array2::zeros((NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_OBJECT));
        let poses = Array2::zeros((NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_POSE));
        let robot_poses = Array3::zeros((NUMBER_OF_DETECTIONS, 14, 3));
        let mut field_features = Array2::zeros((NUMBER_OF_DETECTIONS, 4));
        field_features[[0, 0]] = 12.0;
        field_features[[0, 1]] = 34.0;
        field_features[[0, 2]] = 0.9;
        field_features[[0, 3]] = 2.0;
        let outputs = ModelOutputs {
            objects: objects.view(),
            poses: Some(poses.view()),
            robot_poses: Some(robot_poses.view()),
            field_features: Some(field_features.view()),
        };

        let detections = extract_field_feature_detections(&outputs, 0.5);

        assert_eq!(detections.len(), 1);
        assert_eq!(detections[0].confidence, 0.9);
        assert_eq!(detections[0].label, FieldFeatureLabel::TSpot);
    }

    #[test]
    fn robot_pose_alpha_zero_preserves_legacy_confidence() {
        let mut objects = Array2::zeros((NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_OBJECT));
        objects[[0, 4]] = 0.8;
        objects[[0, 5]] = RobocupObjectLabel::Robot as usize as f32;
        let robot_poses = Array3::zeros((NUMBER_OF_DETECTIONS, 14, 3));
        let outputs = ModelOutputs {
            objects: objects.view(),
            poses: None,
            robot_poses: Some(robot_poses.view()),
            field_features: None,
        };

        let legacy = extract_robot_pose_detections(&outputs, 0.5, 0.0).unwrap();
        let calibrated = extract_robot_pose_detections(&outputs, 0.5, 1.0).unwrap();

        assert_eq!(legacy.len(), 1);
        assert_eq!(legacy[0].object.bounding_box.confidence, 0.8);
        assert!(calibrated.is_empty());
    }

    #[test]
    fn person_pose_visibility_alpha_calibrates_only_pose_confidence() {
        let objects = Array2::zeros((NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_OBJECT));
        let mut poses = Array2::zeros((NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_POSE));
        poses[[0, 4]] = 0.8;
        for keypoint in 0..17 {
            poses[[0, NUMBER_OF_VALUES_PER_OBJECT + keypoint * 3 + 2]] = 0.5;
        }
        let outputs = ModelOutputs {
            objects: objects.view(),
            poses: Some(poses.view()),
            robot_poses: None,
            field_features: None,
        };

        let legacy = extract_candidate_pose_detections(&outputs, 0.0, 0.0).unwrap();
        let calibrated = extract_candidate_pose_detections(&outputs, 0.0, 1.75).unwrap();

        assert_eq!(legacy[0].object.bounding_box.confidence, 0.8);
        assert_eq!(
            calibrated[0].object.bounding_box.confidence,
            0.8 * 0.5_f32.powf(1.75)
        );
        assert_eq!(
            legacy[0].object.bounding_box.area,
            calibrated[0].object.bounding_box.area
        );
        assert_eq!(
            legacy[0].keypoints.nose.confidence,
            calibrated[0].keypoints.nose.confidence
        );
    }

    #[test]
    fn person_pose_visibility_alpha_must_be_valid() {
        let objects = Array2::zeros((NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_OBJECT));
        let poses = Array2::zeros((NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_POSE));
        let outputs = ModelOutputs {
            objects: objects.view(),
            poses: Some(poses.view()),
            robot_poses: None,
            field_features: None,
        };

        for alpha in [-1.0, f32::NAN, f32::INFINITY] {
            let error = extract_candidate_pose_detections(&outputs, 0.0, alpha).unwrap_err();
            assert!(error.to_string().contains("visibility score alpha"));
        }
    }

    #[test]
    fn robot_pose_calibration_changes_only_pose_threshold_and_nms() {
        let mut objects = Array2::zeros((NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_OBJECT));
        for index in 0..2 {
            objects[[index, 0]] = 0.0;
            objects[[index, 1]] = 0.0;
            objects[[index, 2]] = 10.0;
            objects[[index, 3]] = 10.0;
            objects[[index, 5]] = RobocupObjectLabel::Robot as usize as f32;
        }
        objects[[0, 4]] = 0.9;
        objects[[1, 4]] = 0.8;
        let mut robot_poses = Array3::zeros((NUMBER_OF_DETECTIONS, 14, 3));
        for keypoint in 0..14 {
            robot_poses[[0, keypoint, 2]] = 0.1;
            robot_poses[[1, keypoint, 2]] = 1.0;
        }
        let outputs = ModelOutputs {
            objects: objects.view(),
            poses: None,
            robot_poses: Some(robot_poses.view()),
            field_features: None,
        };

        let detected_objects = non_maximum_suppression(
            extract_candidate_object_detections(&outputs, 0.5).unwrap(),
            0.5,
        );
        let detected_robot_poses = non_maximum_suppression(
            extract_robot_pose_detections(&outputs, 0.0, 1.0).unwrap(),
            0.5,
        );

        assert_eq!(detected_objects.len(), 1);
        assert_eq!(detected_robot_poses.len(), 1);
        assert_eq!(detected_objects[0].bounding_box.confidence, 0.9);
        assert_eq!(detected_robot_poses[0].object.bounding_box.confidence, 0.8);
    }

    #[test]
    fn robot_pose_visibility_alpha_must_be_valid() {
        let objects = Array2::zeros((NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_OBJECT));
        let robot_poses = Array3::zeros((NUMBER_OF_DETECTIONS, 14, 3));
        let outputs = ModelOutputs {
            objects: objects.view(),
            poses: None,
            robot_poses: Some(robot_poses.view()),
            field_features: None,
        };

        for alpha in [-1.0, f32::NAN, f32::INFINITY] {
            let error = extract_robot_pose_detections(&outputs, 0.0, alpha).unwrap_err();
            assert!(error.to_string().contains("visibility score alpha"));
        }
    }

    #[test]
    fn field_feature_suppression_keeps_distinct_classes_and_best_duplicate() {
        let detections = vec![
            FieldFeatureDetection {
                point: linear_algebra::point![10.0, 10.0],
                confidence: 0.8,
                label: FieldFeatureLabel::LSpot,
            },
            FieldFeatureDetection {
                point: linear_algebra::point![12.0, 10.0],
                confidence: 0.9,
                label: FieldFeatureLabel::LSpot,
            },
            FieldFeatureDetection {
                point: linear_algebra::point![12.0, 10.0],
                confidence: 0.7,
                label: FieldFeatureLabel::TSpot,
            },
        ];

        let filtered = suppress_field_features(detections, 8.0);

        assert_eq!(filtered.len(), 2);
        assert_eq!(filtered[0].confidence, 0.9);
        assert_eq!(filtered[1].label, FieldFeatureLabel::TSpot);
    }

    #[test]
    fn non_finite_field_confidence_is_rejected() {
        let objects = Array2::zeros((NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_OBJECT));
        let poses = Array2::zeros((NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_POSE));
        let robot_poses = Array3::zeros((NUMBER_OF_DETECTIONS, 14, 3));
        let mut field_features = Array2::zeros((NUMBER_OF_DETECTIONS, 4));
        field_features[[0, 2]] = f32::NAN;
        let outputs = ModelOutputs {
            objects: objects.view(),
            poses: Some(poses.view()),
            robot_poses: Some(robot_poses.view()),
            field_features: Some(field_features.view()),
        };

        let detections = extract_field_feature_detections(&outputs, 0.0);

        assert_eq!(detections.len(), NUMBER_OF_DETECTIONS - 1);
    }
}
