use std::{boxed::Box, future::Future, pin::Pin, sync::Arc, time::Duration};

use color_eyre::{Result, eyre::bail, eyre::eyre};
use ndarray::{ArrayView2, ArrayView3, ArrayViewD, Axis};
use ort::{
    execution_providers::{CUDAExecutionProvider, TensorRTExecutionProvider},
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
    pose_detection::{
        NUMBER_OF_VALUES_PER_POSE, NUMBER_OF_VALUES_PER_ROBOT_POSE, Pose, RobotPoseDetection,
    },
    time_wrapper::TimeWrapper,
};

pub const NUMBER_OF_DETECTIONS: usize = 300;

#[derive(Clone, Copy, Debug)]
enum TaskHead {
    ObjectDetection,
    LegacyPose,
    PersonPose,
    RobotPose,
}

struct DetectionOutput {
    inference_duration: Duration,
    post_processing_duration: Duration,
    non_maximum_suppression_duration: Duration,
    detected_objects: Vec<Object<RobocupObjectLabel>>,
    detected_poses: Vec<Pose<YOLOObjectLabel>>,
    detected_robot_poses: Vec<RobotPoseDetection>,
}

impl TaskHead {
    fn output_name(self) -> &'static str {
        match self {
            TaskHead::ObjectDetection => "object_output",
            TaskHead::LegacyPose => "pose_output",
            TaskHead::PersonPose => "person_pose_output",
            TaskHead::RobotPose => "robot_pose_output",
        }
    }

    fn expected_shape(self) -> [usize; 3] {
        match self {
            Self::ObjectDetection => [1, NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_OBJECT],
            Self::LegacyPose | Self::PersonPose => {
                [1, NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_POSE]
            }
            Self::RobotPose => [1, NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_ROBOT_POSE],
        }
    }
}

#[derive(Debug)]
struct ModelOutputs<'a> {
    objects: ArrayView2<'a, f32>,
    poses: Option<ArrayView2<'a, f32>>,
    robot_poses: Option<ArrayView2<'a, f32>>,
}

pub fn run_boxed(ctx: Arc<Context>) -> Pin<Box<dyn Future<Output = Result<()>> + Send>> {
    Box::pin(run(ctx))
}

async fn run(ctx: Arc<Context>) -> Result<()> {
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

    let initial_parameters_snapshot = node_parameters.snapshot();
    let parameters = initial_parameters_snapshot.typed();
    let model_path = parameters
        .neural_networks_folder
        .join(&parameters.model_name);

    let tensor_rt = TensorRTExecutionProvider::default()
        .with_device_id(0)
        .with_fp16(true)
        .with_engine_cache(true)
        .with_engine_cache_path(parameters.neural_networks_folder.display())
        .build();
    let cuda = CUDAExecutionProvider::default().build();

    let mut session = block_in_place(|| {
        Session::builder()?
            .with_execution_providers([tensor_rt, cuda])?
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
        let detected_robot_poses_pending = detected_robot_poses_pub.announce(image_time).await?;

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
            let candidate_robot_poses = extract_robot_pose_detections(
                &outputs,
                parameters
                    .robot_pose_detection_parameters
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
            let detected_robot_poses = non_maximum_suppression(
                candidate_robot_poses,
                parameters
                    .robot_pose_detection_parameters
                    .maximum_intersection_over_union,
            );
            let non_maximum_suppression_duration = non_maximum_suppression_start.elapsed();

            Ok::<_, color_eyre::eyre::Error>(DetectionOutput {
                inference_duration,
                post_processing_duration,
                non_maximum_suppression_duration,
                detected_objects,
                detected_poses,
                detected_robot_poses,
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
                eyre!(
                    "failed to extract model output `{}`: {error}",
                    task_head.output_name()
                )
            })
        })
        .transpose()
}

fn model_outputs_from_arrays<'a>(
    objects_output: Option<ArrayViewD<'a, f32>>,
    legacy_poses_output: Option<ArrayViewD<'a, f32>>,
    person_poses_output: Option<ArrayViewD<'a, f32>>,
    robot_poses_output: Option<ArrayViewD<'a, f32>>,
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
        .map(|output| validate_and_reshape_output(TaskHead::RobotPose, output))
        .transpose()?;

    Ok(ModelOutputs {
        objects,
        poses,
        robot_poses,
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

#[cfg(test)]
mod tests {
    use ndarray::{Array2, Array3};

    use super::*;

    #[test]
    fn missing_optional_pose_outputs_produce_no_candidates() {
        let objects = Array3::zeros((1, NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_OBJECT));
        let outputs =
            model_outputs_from_arrays(Some(objects.view().into_dyn()), None, None, None).unwrap();

        assert!(
            extract_candidate_pose_detections(&outputs, 0.0)
                .unwrap()
                .is_empty()
        );
        assert!(
            extract_robot_pose_detections(&outputs, 0.0)
                .unwrap()
                .is_empty()
        );
    }

    #[test]
    fn object_output_is_mandatory() {
        let error = model_outputs_from_arrays(None, None, None, None).unwrap_err();

        assert!(error.to_string().contains("`object_output` is missing"));
    }

    #[test]
    fn three_head_outputs_are_validated_and_reshaped() {
        let objects = Array3::zeros((1, NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_OBJECT));
        let poses = Array3::zeros((1, NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_POSE));
        let robot_poses = Array3::zeros((1, NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_ROBOT_POSE));

        let outputs = model_outputs_from_arrays(
            Some(objects.view().into_dyn()),
            None,
            Some(poses.view().into_dyn()),
            Some(robot_poses.view().into_dyn()),
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
        assert_eq!(
            outputs.robot_poses.as_ref().unwrap().shape(),
            [NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_ROBOT_POSE]
        );
    }

    #[test]
    fn legacy_pose_output_remains_supported() {
        let objects = Array3::zeros((1, NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_OBJECT));
        let poses = Array3::zeros((1, NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_POSE));

        let outputs = model_outputs_from_arrays(
            Some(objects.view().into_dyn()),
            Some(poses.view().into_dyn()),
            None,
            None,
        )
        .unwrap();

        assert_eq!(
            outputs.poses.as_ref().unwrap().shape(),
            [NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_POSE]
        );
    }

    #[test]
    fn deployment_output_contract_has_three_named_shapes() {
        assert_eq!(TaskHead::ObjectDetection.output_name(), "object_output");
        assert_eq!(TaskHead::LegacyPose.output_name(), "pose_output");
        assert_eq!(TaskHead::PersonPose.output_name(), "person_pose_output");
        assert_eq!(TaskHead::RobotPose.output_name(), "robot_pose_output");
        assert_eq!(TaskHead::ObjectDetection.expected_shape(), [1, 300, 6]);
        assert_eq!(TaskHead::PersonPose.expected_shape(), [1, 300, 57]);
        assert_eq!(TaskHead::RobotPose.expected_shape(), [1, 300, 48]);
    }

    #[test]
    fn robot_pose_output_is_self_contained() {
        let objects = Array2::zeros((NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_OBJECT));
        let mut robot_poses =
            Array2::zeros((NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_ROBOT_POSE));
        robot_poses[[0, 4]] = 0.9;
        let outputs = ModelOutputs {
            objects: objects.view(),
            poses: None,
            robot_poses: Some(robot_poses.view()),
        };

        let detections = extract_robot_pose_detections(&outputs, 0.5).unwrap();

        assert_eq!(detections.len(), 1);
        assert_eq!(detections[0].object.label, RobocupObjectLabel::Robot);
        assert_eq!(detections[0].object.bounding_box.confidence, 0.9);
    }
}
