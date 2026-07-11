use std::{boxed::Box, future::Future, path::Path, pin::Pin, sync::Arc, time::Duration};

use color_eyre::{Result, eyre::bail, eyre::eyre};
use ndarray::{ArrayView2, ArrayView3, ArrayViewD, Axis};
#[cfg(feature = "webgpu")]
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
use tokio::{task::block_in_place, time::Instant};
use types::{
    bounding_box::BoundingBox,
    object_detection::{NUMBER_OF_VALUES_PER_OBJECT, Object, RobocupObjectLabel, YOLOObjectLabel},
    parameters::DetectionParameters,
    pose_detection::{NUMBER_OF_VALUES_PER_POSE, Pose},
    time_wrapper::TimeWrapper,
};

pub const NUMBER_OF_DETECTIONS: usize = 300;

#[derive(Clone, Copy, Debug, Default)]
pub enum ExecutionProviderPolicy {
    #[default]
    Automatic,
    WebGpuRequired,
}

#[derive(Clone, Copy, Debug)]
enum TaskHead {
    ObjectDetection,
    PoseDetection,
}

struct DetectionOutput {
    inference_duration: Duration,
    post_processing_duration: Duration,
    non_maximum_suppression_duration: Duration,
    detected_objects: Vec<Object<RobocupObjectLabel>>,
    detected_poses: Vec<Pose<YOLOObjectLabel>>,
}

impl TaskHead {
    fn output_name(self) -> &'static str {
        match self {
            TaskHead::ObjectDetection => "object_output",
            TaskHead::PoseDetection => "pose_output",
        }
    }

    fn expected_shape(self) -> [usize; 3] {
        match self {
            Self::ObjectDetection => [1, NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_OBJECT],
            Self::PoseDetection => [1, NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_POSE],
        }
    }
}

#[derive(Debug)]
struct ModelOutputs<'a> {
    objects: ArrayView2<'a, f32>,
    poses: Option<ArrayView2<'a, f32>>,
}

pub fn run_boxed(ctx: Arc<Context>) -> Pin<Box<dyn Future<Output = Result<()>> + Send>> {
    run_boxed_with_provider_policy(ctx, ExecutionProviderPolicy::Automatic)
}

pub fn run_boxed_with_provider_policy(
    ctx: Arc<Context>,
    provider_policy: ExecutionProviderPolicy,
) -> Pin<Box<dyn Future<Output = Result<()>> + Send>> {
    Box::pin(run(ctx, provider_policy))
}

async fn run(ctx: Arc<Context>, provider_policy: ExecutionProviderPolicy) -> Result<()> {
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
            )?;
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
            let non_maximum_suppression_duration = non_maximum_suppression_start.elapsed();

            Ok::<_, color_eyre::eyre::Error>(DetectionOutput {
                inference_duration,
                post_processing_duration,
                non_maximum_suppression_duration,
                detected_objects,
                detected_poses,
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
    }
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

    #[cfg(feature = "webgpu")]
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

    #[cfg(not(feature = "webgpu"))]
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
    let objects_output = outputs
        .get(TaskHead::ObjectDetection.output_name())
        .map(|output| output.try_extract_array::<f32>())
        .transpose()?;
    let poses_output = outputs
        .get(TaskHead::PoseDetection.output_name())
        .map(|output| output.try_extract_array::<f32>())
        .transpose()?;

    model_outputs_from_arrays(objects_output, poses_output)
}

fn model_outputs_from_arrays<'a>(
    objects_output: Option<ArrayViewD<'a, f32>>,
    poses_output: Option<ArrayViewD<'a, f32>>,
) -> Result<ModelOutputs<'a>> {
    let objects_output = objects_output.ok_or_else(|| {
        eyre!(
            "mandatory model output `{}` is missing",
            TaskHead::ObjectDetection.output_name()
        )
    })?;
    let objects = validate_and_reshape_output(TaskHead::ObjectDetection, objects_output)?;
    let poses = poses_output
        .map(|output| validate_and_reshape_output(TaskHead::PoseDetection, output))
        .transpose()?;

    Ok(ModelOutputs { objects, poses })
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

fn extract_candidate_object_detections(
    outputs: &ModelOutputs,
    confidence_threshold: f32,
) -> Result<Vec<Object<RobocupObjectLabel>>> {
    Ok(outputs
        .objects
        .axis_iter(Axis(0))
        .filter_map(|row| {
            let confidence = row[4usize];
            if confidence < confidence_threshold {
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
            if confidence < confidence_threshold {
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

#[cfg(test)]
mod tests {
    use ndarray::Array3;

    use super::*;

    #[test]
    fn missing_pose_output_produces_no_pose_candidates() {
        let objects = Array3::zeros(TaskHead::ObjectDetection.expected_shape());
        let outputs = model_outputs_from_arrays(Some(objects.view().into_dyn()), None).unwrap();

        let poses = extract_candidate_pose_detections(&outputs, 0.0).unwrap();

        assert!(poses.is_empty());
    }

    #[test]
    fn object_output_is_mandatory() {
        let error = model_outputs_from_arrays(None, None).unwrap_err();

        assert!(error.to_string().contains("`object_output` is missing"));
    }

    #[test]
    fn object_output_must_have_expected_shape() {
        let objects =
            Array3::<f32>::zeros((1, NUMBER_OF_DETECTIONS - 1, NUMBER_OF_VALUES_PER_OBJECT));

        let error = model_outputs_from_arrays(Some(objects.view().into_dyn()), None).unwrap_err();

        assert!(
            error
                .to_string()
                .contains("object_output not of expected shape")
        );
    }

    #[test]
    fn present_outputs_are_shape_validated_and_reshaped() {
        let objects = Array3::zeros(TaskHead::ObjectDetection.expected_shape());
        let poses = Array3::zeros(TaskHead::PoseDetection.expected_shape());

        let outputs = model_outputs_from_arrays(
            Some(objects.view().into_dyn()),
            Some(poses.view().into_dyn()),
        )
        .unwrap();

        assert_eq!(
            outputs.objects.shape(),
            [NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_OBJECT]
        );
        assert_eq!(
            outputs.poses.unwrap().shape(),
            [NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_POSE]
        );
    }

    #[test]
    fn present_pose_output_must_have_expected_shape() {
        let objects = Array3::zeros(TaskHead::ObjectDetection.expected_shape());
        let poses = Array3::<f32>::zeros((1, NUMBER_OF_DETECTIONS - 1, NUMBER_OF_VALUES_PER_POSE));

        let error = model_outputs_from_arrays(
            Some(objects.view().into_dyn()),
            Some(poses.view().into_dyn()),
        )
        .unwrap_err();

        assert!(
            error
                .to_string()
                .contains("pose_output not of expected shape")
        );
    }
}
