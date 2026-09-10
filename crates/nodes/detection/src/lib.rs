use std::{
    boxed::Box, collections::BTreeMap, future::Future, path::Path, pin::Pin, sync::Arc,
    time::Duration,
};

use color_eyre::{Result, eyre::bail, eyre::eyre};
use ndarray::{ArrayView2, ArrayView3, ArrayViewD, Axis};
#[cfg(feature = "webgpu-provider")]
use ort::execution_providers::WebGPUExecutionProvider;
#[cfg(feature = "nvidia")]
use ort::execution_providers::{CUDAExecutionProvider, TensorRTExecutionProvider};
use ort::{
    execution_providers::{
        CPUExecutionProvider, ExecutionProvider as OrtExecutionProvider, ExecutionProviderDispatch,
    },
    inputs,
    session::{Session, SessionOutputs, builder::GraphOptimizationLevel},
    value::TensorRef,
};
use ros_z_streams::CreateAnnouncingPublisher;
use ros2::sensor_msgs::image::Image;
use serde::Deserialize;

use ros_z::prelude::*;
use tokio::{sync::oneshot, task::block_in_place, time::Instant};
use types::{
    bounding_box::BoundingBox,
    object_detection::{NUMBER_OF_VALUES_PER_OBJECT, Object, RobocupObjectLabel, YOLOObjectLabel},
    parameters::DetectionParameters,
    pose_detection::{
        FieldFeatureDetection, FieldFeatureLabel, NUMBER_OF_VALUES_PER_POSE,
        NUMBER_OF_VALUES_PER_ROBOT_POSE, Pose, RobotPoseDetection,
    },
    time_wrapper::TimeWrapper,
};

pub const NUMBER_OF_DETECTIONS: usize = 300;
const NUMBER_OF_VALUES_PER_FIELD_FEATURE_POSE: usize = 9;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
/// Selects the ONNX Runtime execution provider used by detection.
pub enum ExecutionProvider {
    /// Try TensorRT, CUDA, WebGPU, then ORT's implicit CPU fallback when compiled in.
    #[default]
    Automatic,
    /// Prefer TensorRT, with CUDA and ORT's CPU provider available for unsupported operators.
    TensorRt,
    /// Use CUDA, with ORT's CPU provider available for unsupported operators.
    Cuda,
    /// Use WebGPU, with ORT's CPU provider available for unsupported operators.
    WebGpu,
    /// Use ORT's CPU provider only.
    Cpu,
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

#[derive(Clone, Debug)]
enum LegacyPoseOutput {
    Person,
    FieldFeature { labels: [FieldFeatureLabel; 5] },
}

#[derive(Clone, Debug)]
struct ModelOutputContract {
    legacy_pose: Option<LegacyPoseOutput>,
    has_person_pose: bool,
    has_robot_pose: bool,
    has_direct_field_features: bool,
}

#[derive(Debug, Deserialize)]
struct HydraBranchesMetadata {
    #[serde(default)]
    pose_output: Option<HydraBranchMetadata>,
}

#[derive(Debug, Deserialize)]
struct HydraBranchMetadata {
    task: String,
    #[serde(default)]
    names: BTreeMap<usize, String>,
    #[serde(default)]
    kpt_shape: Option<Vec<usize>>,
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
            Self::RobotPose => &[1, NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_ROBOT_POSE],
            Self::FieldFeature => &[1, NUMBER_OF_DETECTIONS, 4],
        }
    }
}

#[derive(Debug)]
struct ModelOutputs<'a> {
    objects: ArrayView2<'a, f32>,
    poses: Option<ArrayView2<'a, f32>>,
    robot_poses: Option<ArrayView2<'a, f32>>,
    field_features: Option<ArrayView2<'a, f32>>,
    field_feature_poses: Option<ArrayView2<'a, f32>>,
}

pub fn run_boxed(ctx: Arc<Context>) -> Pin<Box<dyn Future<Output = Result<()>> + Send>> {
    Box::pin(run(ctx, ExecutionProvider::Automatic, None))
}

/// Runs detection and reports model capabilities once session creation succeeds.
///
/// The one-shot notification does not indicate that ROS-Z publishers and subscribers are ready.
pub fn run_boxed_with_model_info(
    ctx: Arc<Context>,
    provider: ExecutionProvider,
    model_info_sender: oneshot::Sender<DetectionModelInfo>,
) -> Pin<Box<dyn Future<Output = Result<()>> + Send>> {
    Box::pin(run(ctx, provider, Some(model_info_sender)))
}

async fn run(
    ctx: Arc<Context>,
    provider: ExecutionProvider,
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

    let execution_providers = execution_providers(&parameters.neural_networks_folder, provider)?;

    let mut session = block_in_place(|| {
        Session::builder()?
            .with_execution_providers(execution_providers)?
            .with_optimization_level(GraphOptimizationLevel::Level3)?
            .with_intra_threads(2)?
            .commit_from_file(model_path)
    })?;
    let model_contract = model_output_contract(&session)?;
    let model_info = model_contract.model_info();
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

            let outputs = extract_outputs(&outputs, &model_contract)?;
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
            )?;
            let detected_robot_poses = extract_robot_pose_detections(
                &outputs,
                parameters
                    .robot_pose_detection_parameters
                    .minimum_candidate_confidence,
            )?;
            let detected_field_features = extract_field_feature_detections(
                &outputs,
                &model_contract,
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

impl ModelOutputContract {
    fn model_info(&self) -> DetectionModelInfo {
        DetectionModelInfo {
            has_pose_output: self.has_person_pose
                || matches!(self.legacy_pose.as_ref(), Some(LegacyPoseOutput::Person)),
            has_robot_pose_output: self.has_robot_pose,
            has_field_feature_output: self.has_direct_field_features
                || matches!(
                    self.legacy_pose.as_ref(),
                    Some(LegacyPoseOutput::FieldFeature { .. })
                ),
        }
    }
}

fn model_output_contract(session: &Session) -> Result<ModelOutputContract> {
    let branches = session.metadata()?.custom("branches")?;
    model_output_contract_from_metadata(
        session.outputs.iter().map(|output| output.name.as_str()),
        branches.as_deref(),
    )
}

fn model_output_contract_from_metadata<'a>(
    names: impl IntoIterator<Item = &'a str>,
    branches: Option<&str>,
) -> Result<ModelOutputContract> {
    let names = names.into_iter().collect::<Vec<_>>();
    let has_output = |name: &str| names.contains(&name);
    let legacy_pose = if has_output(TaskHead::LegacyPose.output_name()) {
        legacy_pose_output_from_metadata(branches)?
    } else {
        None
    };

    Ok(ModelOutputContract {
        legacy_pose,
        has_person_pose: has_output(TaskHead::PersonPose.output_name()),
        has_robot_pose: has_output(TaskHead::RobotPose.output_name()),
        has_direct_field_features: has_output(TaskHead::FieldFeature.output_name()),
    })
}

fn legacy_pose_output_from_metadata(branches: Option<&str>) -> Result<Option<LegacyPoseOutput>> {
    let Some(branches) = branches else {
        return Ok(Some(LegacyPoseOutput::Person));
    };
    let branches: HydraBranchesMetadata = serde_json::from_str(branches)
        .map_err(|error| eyre!(error).wrap_err("failed to parse ONNX `branches` model metadata"))?;
    let Some(branch) = branches.pose_output else {
        return Ok(Some(LegacyPoseOutput::Person));
    };
    if branch.task != "pose" {
        bail!(
            "`pose_output` metadata has task `{}`, expected `pose`",
            branch.task
        );
    }

    match branch.kpt_shape.as_deref() {
        Some([1, 3]) => Ok(Some(LegacyPoseOutput::FieldFeature {
            labels: field_feature_labels(&branch.names)?,
        })),
        Some([17, 3]) | None => Ok(Some(LegacyPoseOutput::Person)),
        Some(shape) => bail!("unsupported `pose_output` keypoint shape {shape:?}"),
    }
}

fn field_feature_labels(names: &BTreeMap<usize, String>) -> Result<[FieldFeatureLabel; 5]> {
    let labels = names
        .iter()
        .enumerate()
        .map(|(expected_index, (index, name))| {
            if *index != expected_index {
                bail!("field-pose class indices must be contiguous from zero");
            }
            match name.as_str() {
                "GoalPost" => Ok(FieldFeatureLabel::GoalPost),
                "LSpot" => Ok(FieldFeatureLabel::LSpot),
                "PenaltySpot" => Ok(FieldFeatureLabel::PenaltySpot),
                "TSpot" => Ok(FieldFeatureLabel::TSpot),
                "XSpot" => Ok(FieldFeatureLabel::XSpot),
                _ => bail!("unknown field-pose class `{name}`"),
            }
        })
        .collect::<Result<Vec<_>>>()?;
    for (index, label) in labels.iter().enumerate() {
        if labels[..index].contains(label) {
            bail!("field-pose metadata contains duplicate class `{label:?}`");
        }
    }
    labels.try_into().map_err(|labels: Vec<_>| {
        eyre!(
            "field-pose metadata has {} classes, expected 5",
            labels.len()
        )
    })
}

fn execution_providers(
    _neural_networks_folder: &Path,
    provider: ExecutionProvider,
) -> Result<Vec<ExecutionProviderDispatch>> {
    #[allow(unused_mut)]
    let mut providers = Vec::new();

    #[cfg(feature = "nvidia")]
    if matches!(
        provider,
        ExecutionProvider::Automatic | ExecutionProvider::TensorRt
    ) {
        let tensor_rt = TensorRTExecutionProvider::default()
            .with_device_id(0)
            .with_fp16(true)
            .with_engine_cache(true)
            .with_engine_cache_path(_neural_networks_folder.display());
        log_provider_availability(&tensor_rt);
        let tensor_rt = tensor_rt.build();
        providers.push(if provider == ExecutionProvider::TensorRt {
            tensor_rt.error_on_failure()
        } else {
            tensor_rt
        });
    }

    #[cfg(feature = "nvidia")]
    if matches!(
        provider,
        ExecutionProvider::Automatic | ExecutionProvider::TensorRt | ExecutionProvider::Cuda
    ) {
        let cuda = CUDAExecutionProvider::default().with_device_id(0);
        log_provider_availability(&cuda);
        let cuda = cuda.build();
        providers.push(if provider == ExecutionProvider::Cuda {
            cuda.error_on_failure()
        } else {
            cuda
        });
    }

    #[cfg(feature = "webgpu-provider")]
    if matches!(
        provider,
        ExecutionProvider::Automatic | ExecutionProvider::WebGpu
    ) {
        let webgpu = WebGPUExecutionProvider::default().with_device_id(0);
        log_provider_availability(&webgpu);
        let webgpu = webgpu.build();
        let webgpu = if provider == ExecutionProvider::WebGpu {
            webgpu.error_on_failure()
        } else {
            webgpu
        };
        providers.push(webgpu);
    }

    #[cfg(not(feature = "nvidia"))]
    if matches!(
        provider,
        ExecutionProvider::TensorRt | ExecutionProvider::Cuda
    ) {
        bail!("the requested NVIDIA provider is not compiled into detection");
    }

    #[cfg(not(feature = "webgpu-provider"))]
    if provider == ExecutionProvider::WebGpu {
        bail!("WebGPU was requested but detection was built without its WebGPU feature");
    }

    // CPU is ORT's implicit final fallback and must not be explicitly registered.
    log_provider_availability(&CPUExecutionProvider::default());

    Ok(providers)
}

fn log_provider_availability(provider: &impl OrtExecutionProvider) {
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

fn extract_outputs<'a>(
    outputs: &'a SessionOutputs<'a>,
    contract: &ModelOutputContract,
) -> Result<ModelOutputs<'a>> {
    model_outputs_from_arrays(
        extract_output(outputs, TaskHead::ObjectDetection)?,
        extract_output(outputs, TaskHead::LegacyPose)?,
        extract_output(outputs, TaskHead::PersonPose)?,
        extract_output(outputs, TaskHead::RobotPose)?,
        extract_output(outputs, TaskHead::FieldFeature)?,
        contract,
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
    contract: &ModelOutputContract,
) -> Result<ModelOutputs<'a>> {
    let objects_output = objects_output.ok_or_else(|| {
        eyre!(
            "mandatory model output `{}` is missing",
            TaskHead::ObjectDetection.output_name()
        )
    })?;
    let objects = validate_and_reshape_output(TaskHead::ObjectDetection, objects_output)?;
    let mut poses = person_poses_output
        .map(|output| validate_and_reshape_output(TaskHead::PersonPose, output))
        .transpose()?;
    let mut field_feature_poses = None;
    match (&contract.legacy_pose, legacy_poses_output) {
        (Some(LegacyPoseOutput::Person), Some(output)) if poses.is_none() => {
            poses = Some(validate_and_reshape_output(TaskHead::LegacyPose, output)?);
        }
        (Some(LegacyPoseOutput::FieldFeature { .. }), Some(output)) => {
            field_feature_poses = Some(validate_and_reshape_named_output(
                TaskHead::LegacyPose.output_name(),
                &[
                    1,
                    NUMBER_OF_DETECTIONS,
                    NUMBER_OF_VALUES_PER_FIELD_FEATURE_POSE,
                ],
                output,
            )?);
        }
        (None, None) | (Some(LegacyPoseOutput::Person), Some(_)) => {}
        (None, Some(_)) | (Some(_), None) => {
            bail!("model output contract does not match `pose_output` availability")
        }
    };
    let robot_poses = robot_poses_output
        .map(|output| validate_and_reshape_output(TaskHead::RobotPose, output))
        .transpose()?;
    let field_features = field_features_output
        .map(|output| validate_and_reshape_output(TaskHead::FieldFeature, output))
        .transpose()?;

    Ok(ModelOutputs {
        objects,
        poses,
        robot_poses,
        field_features,
        field_feature_poses,
    })
}

fn validate_and_reshape_output<'a>(
    task_head: TaskHead,
    output: ArrayViewD<'a, f32>,
) -> Result<ArrayView2<'a, f32>> {
    validate_and_reshape_named_output(task_head.output_name(), task_head.expected_shape(), output)
}

fn validate_and_reshape_named_output<'a>(
    output_name: &str,
    expected_shape: &[usize],
    output: ArrayViewD<'a, f32>,
) -> Result<ArrayView2<'a, f32>> {
    if output.shape() != expected_shape {
        bail!(
            "{} not of expected shape. Expected: {:?}, got: {:?}",
            output_name,
            expected_shape,
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
) -> Result<Vec<Pose<YOLOObjectLabel>>> {
    let Some(poses) = &outputs.poses else {
        return Ok(Vec::new());
    };

    Ok(poses
        .axis_iter(Axis(0))
        .filter_map(|row| {
            let confidence = row[4usize];
            if !confidence.is_finite() || confidence < confidence_threshold {
                return None;
            }

            let pose_values: [f32; NUMBER_OF_VALUES_PER_POSE] = row
                .as_slice()
                .expect("slice is not contiguous")
                .try_into()
                .unwrap_or_else(|_| panic!("slice is not of length {}", NUMBER_OF_VALUES_PER_POSE));

            Some(Pose::from(&pose_values))
        })
        .collect())
}

fn extract_robot_pose_detections(
    outputs: &ModelOutputs,
    confidence_threshold: f32,
) -> Result<Vec<RobotPoseDetection>> {
    let Some(robot_poses) = &outputs.robot_poses else {
        return Ok(Vec::new());
    };

    Ok(robot_poses
        .axis_iter(Axis(0))
        .filter_map(|row| {
            let confidence = row[4];
            if !confidence.is_finite() || confidence < confidence_threshold {
                return None;
            }

            let pose_values: [f32; NUMBER_OF_VALUES_PER_ROBOT_POSE] = row
                .as_slice()
                .expect("slice is not contiguous")
                .try_into()
                .expect("robot pose row has invalid length");
            Some(RobotPoseDetection::from(&pose_values))
        })
        .collect())
}

fn extract_field_feature_detections(
    outputs: &ModelOutputs,
    contract: &ModelOutputContract,
    confidence_threshold: f32,
) -> Vec<FieldFeatureDetection> {
    let mut detections = outputs
        .field_features
        .iter()
        .flat_map(|field_features| field_features.axis_iter(Axis(0)))
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
        .collect::<Vec<_>>();

    let Some(field_feature_poses) = &outputs.field_feature_poses else {
        return detections;
    };
    let Some(LegacyPoseOutput::FieldFeature { labels }) = &contract.legacy_pose else {
        return detections;
    };
    detections.extend(field_feature_poses.axis_iter(Axis(0)).filter_map(|row| {
        let confidence = row[4];
        let class = row[5];
        if !confidence.is_finite()
            || confidence < confidence_threshold
            || !class.is_finite()
            || class < 0.0
            || class.fract() != 0.0
        {
            return None;
        }
        let label = labels.get(class as usize).copied()?;
        Some(FieldFeatureDetection {
            point: linear_algebra::point![row[6], row[7]],
            confidence,
            label,
        })
    }));
    detections
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
    use ndarray::{Array2, Array3};

    use super::*;

    fn contract<'a>(names: impl IntoIterator<Item = &'a str>) -> ModelOutputContract {
        model_output_contract_from_metadata(names, None).unwrap()
    }

    const FIELD_POSE_BRANCHES: &str = r#"{
        "object_output": {"task": "detect", "names": {"0": "Ball"}},
        "pose_output": {
            "task": "pose",
            "names": {
                "0": "GoalPost",
                "1": "LSpot",
                "2": "PenaltySpot",
                "3": "TSpot",
                "4": "XSpot"
            },
            "kpt_shape": [1, 3]
        }
    }"#;

    #[test]
    fn default_provider_preserves_production_behavior() {
        assert_eq!(ExecutionProvider::default(), ExecutionProvider::Automatic);
    }

    #[test]
    fn missing_pose_output_produces_no_pose_candidates() {
        let objects = Array3::zeros((1, NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_OBJECT));
        let contract = contract(["object_output"]);
        let outputs = model_outputs_from_arrays(
            Some(objects.view().into_dyn()),
            None,
            None,
            None,
            None,
            &contract,
        )
        .unwrap();

        let poses = extract_candidate_pose_detections(&outputs, 0.0).unwrap();

        assert!(poses.is_empty());
        assert!(
            extract_robot_pose_detections(&outputs, 0.0)
                .unwrap()
                .is_empty()
        );
        assert!(extract_field_feature_detections(&outputs, &contract, 0.0).is_empty());
    }

    #[test]
    fn model_info_distinguishes_pose_capability() {
        assert_eq!(
            contract(["object_output"]).model_info(),
            DetectionModelInfo {
                has_pose_output: false,
                has_robot_pose_output: false,
                has_field_feature_output: false,
            }
        );
        assert_eq!(
            contract(["object_output", "pose_output"]).model_info(),
            DetectionModelInfo {
                has_pose_output: true,
                has_robot_pose_output: false,
                has_field_feature_output: false,
            }
        );
        assert_eq!(
            contract(["object_output", "person_pose_output"]).model_info(),
            DetectionModelInfo {
                has_pose_output: true,
                has_robot_pose_output: false,
                has_field_feature_output: false,
            }
        );
        assert_eq!(
            contract(["object_output", "field_feature_output"]).model_info(),
            DetectionModelInfo {
                has_pose_output: false,
                has_robot_pose_output: false,
                has_field_feature_output: true,
            }
        );
        assert_eq!(
            contract(["object_output", "robot_pose_output"]).model_info(),
            DetectionModelInfo {
                has_pose_output: false,
                has_robot_pose_output: true,
                has_field_feature_output: false,
            }
        );
        assert_eq!(
            model_output_contract_from_metadata(
                ["object_output", "pose_output", "robot_pose_output"],
                Some(FIELD_POSE_BRANCHES),
            )
            .unwrap()
            .model_info(),
            DetectionModelInfo {
                has_pose_output: false,
                has_robot_pose_output: true,
                has_field_feature_output: true,
            }
        );
    }

    #[test]
    fn object_output_is_mandatory() {
        let error =
            model_outputs_from_arrays(None, None, None, None, None, &contract(["object_output"]))
                .unwrap_err();

        assert!(error.to_string().contains("`object_output` is missing"));
    }

    #[test]
    fn object_output_must_have_expected_shape() {
        let objects =
            Array3::<f32>::zeros((1, NUMBER_OF_DETECTIONS - 1, NUMBER_OF_VALUES_PER_OBJECT));

        let error = model_outputs_from_arrays(
            Some(objects.view().into_dyn()),
            None,
            None,
            None,
            None,
            &contract(["object_output"]),
        )
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
            &contract(["object_output", "pose_output"]),
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
        let robot_poses = Array3::zeros((1, NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_ROBOT_POSE));
        let field_features = Array3::zeros((1, NUMBER_OF_DETECTIONS, 4));

        let outputs = model_outputs_from_arrays(
            Some(objects.view().into_dyn()),
            None,
            Some(poses.view().into_dyn()),
            Some(robot_poses.view().into_dyn()),
            Some(field_features.view().into_dyn()),
            &contract([
                "object_output",
                "person_pose_output",
                "robot_pose_output",
                "field_feature_output",
            ]),
        )
        .unwrap();

        assert_eq!(
            outputs.poses.as_ref().unwrap().shape(),
            [NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_POSE]
        );
        assert_eq!(
            outputs.robot_poses.as_ref().unwrap().shape(),
            [NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_ROBOT_POSE]
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
            &contract(["object_output", "pose_output"]),
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
        assert_eq!(TaskHead::RobotPose.expected_shape(), &[1, 300, 48]);
        assert_eq!(TaskHead::FieldFeature.expected_shape(), &[1, 300, 4]);
        assert_eq!(NUMBER_OF_VALUES_PER_FIELD_FEATURE_POSE, 9);
    }

    #[test]
    fn robot_pose_output_is_self_contained() {
        let objects = Array2::zeros((NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_OBJECT));
        let mut robot_poses =
            Array2::zeros((NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_ROBOT_POSE));
        robot_poses[[0, 0]] = 10.0;
        robot_poses[[0, 1]] = 20.0;
        robot_poses[[0, 2]] = 30.0;
        robot_poses[[0, 3]] = 40.0;
        robot_poses[[0, 4]] = 0.9;
        let outputs = ModelOutputs {
            objects: objects.view(),
            poses: None,
            robot_poses: Some(robot_poses.view()),
            field_features: None,
            field_feature_poses: None,
        };

        let detections = extract_robot_pose_detections(&outputs, 0.5).unwrap();

        assert_eq!(detections.len(), 1);
        assert_eq!(detections[0].object.label, RobocupObjectLabel::Robot);
        assert_eq!(detections[0].object.bounding_box.confidence, 0.9);
    }

    #[test]
    fn field_feature_output_routes_class_and_confidence() {
        let objects = Array2::zeros((NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_OBJECT));
        let poses = Array2::zeros((NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_POSE));
        let robot_poses = Array2::zeros((NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_ROBOT_POSE));
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
            field_feature_poses: None,
        };

        let detections = extract_field_feature_detections(
            &outputs,
            &contract(["object_output", "field_feature_output"]),
            0.5,
        );

        assert_eq!(detections.len(), 1);
        assert_eq!(detections[0].confidence, 0.9);
        assert_eq!(detections[0].label, FieldFeatureLabel::TSpot);
    }

    #[test]
    fn field_pose_output_routes_keypoint_and_metadata_class_order() {
        let contract = model_output_contract_from_metadata(
            ["object_output", "pose_output"],
            Some(FIELD_POSE_BRANCHES),
        )
        .unwrap();
        let objects = Array3::zeros((1, NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_OBJECT));
        let mut field_poses = Array3::zeros((
            1,
            NUMBER_OF_DETECTIONS,
            NUMBER_OF_VALUES_PER_FIELD_FEATURE_POSE,
        ));
        field_poses[[0, 0, 4]] = 0.9;
        field_poses[[0, 0, 5]] = 2.0;
        field_poses[[0, 0, 6]] = 12.0;
        field_poses[[0, 0, 7]] = 34.0;
        field_poses[[0, 0, 8]] = 0.8;
        let outputs = model_outputs_from_arrays(
            Some(objects.view().into_dyn()),
            Some(field_poses.view().into_dyn()),
            None,
            None,
            None,
            &contract,
        )
        .unwrap();

        let detections = extract_field_feature_detections(&outputs, &contract, 0.5);

        assert!(outputs.poses.is_none());
        assert_eq!(detections.len(), 1);
        assert_eq!(detections[0].point, linear_algebra::point![12.0, 34.0]);
        assert_eq!(detections[0].confidence, 0.9);
        assert_eq!(detections[0].label, FieldFeatureLabel::PenaltySpot);
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
        let robot_poses = Array2::zeros((NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_ROBOT_POSE));
        let mut field_features = Array2::zeros((NUMBER_OF_DETECTIONS, 4));
        field_features[[0, 2]] = f32::NAN;
        let outputs = ModelOutputs {
            objects: objects.view(),
            poses: Some(poses.view()),
            robot_poses: Some(robot_poses.view()),
            field_features: Some(field_features.view()),
            field_feature_poses: None,
        };

        let detections = extract_field_feature_detections(
            &outputs,
            &contract(["object_output", "field_feature_output"]),
            0.0,
        );

        assert_eq!(detections.len(), NUMBER_OF_DETECTIONS - 1);
    }
}
