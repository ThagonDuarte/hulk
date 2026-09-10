use std::{boxed::Box, future::Future, pin::Pin, sync::Arc, time::Duration};

use color_eyre::{
    Result,
    eyre::{bail, eyre},
};
use linear_algebra::point;
use ndarray::{ArrayView2, ArrayView3, Axis};
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
        FieldPose, Keypoint, NUMBER_OF_VALUES_PER_FIELD_POSE, NUMBER_OF_VALUES_PER_ROBOT_POSE,
        Pose, RobocupPose, RobotPose,
    },
    time_wrapper::TimeWrapper,
};

pub const NUMBER_OF_DETECTIONS: usize = 300;

#[derive(Clone, Copy, Debug)]
enum TaskHead {
    Ball,
    FieldPose,
    RobotPose,
}

struct DetectionOutput {
    inference_duration: Duration,
    post_processing_duration: Duration,
    non_maximum_suppression_duration: Duration,
    detected_objects: Vec<Object<RobocupObjectLabel>>,
    detected_field_poses: Vec<FieldPose>,
    detected_robot_poses: Vec<RobotPose>,
}

impl TaskHead {
    fn output_name(self) -> &'static str {
        match self {
            TaskHead::Ball => "object_output",
            TaskHead::FieldPose => "pose_output",
            TaskHead::RobotPose => "robot_pose_output",
        }
    }

    fn expected_shape(self) -> [usize; 3] {
        match self {
            Self::Ball => [1, NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_OBJECT],
            Self::FieldPose => [1, NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_FIELD_POSE],
            Self::RobotPose => [1, NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_ROBOT_POSE],
        }
    }
}

#[derive(Debug)]
struct ModelOutputs<'a> {
    objects: ArrayView2<'a, f32>,
    poses: ArrayView2<'a, f32>,
    robot_poses: ArrayView2<'a, f32>,
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

    let detected_field_poses_pub = node
        .announcing_publisher::<TimeWrapper<Vec<FieldPose>>>("detected_field_poses")
        .await?;
    let detected_robot_poses_pub = node
        .announcing_publisher::<TimeWrapper<Vec<RobotPose>>>("detected_robot_poses")
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

        let detected_field_poses_pending = detected_field_poses_pub.announce(image_time).await?;
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
            let candidate_field_poses = extract_candidate_pose_detections::<1>(
                outputs.poses,
                TaskHead::FieldPose,
                parameters
                    .pose_detection_parameters
                    .minimum_candidate_confidence,
            )?;
            let candidate_robot_poses = extract_candidate_pose_detections::<14>(
                outputs.robot_poses,
                TaskHead::RobotPose,
                parameters
                    .robot_pose_detection_parameters
                    .minimum_candidate_confidence,
            )?;
            let post_processing_duration = post_processing_start.elapsed();
            let non_maximum_suppression_start = Instant::now();
            let mut detected_objects = non_maximum_suppression(
                candidate_detections,
                parameters
                    .object_detection_parameters
                    .maximum_intersection_over_union,
            );
            let detected_field_poses = non_maximum_suppression(
                candidate_field_poses,
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
            // Preserve the object stream consumed by ball, field-mark and obstacle filters.
            detected_objects.extend(detected_field_poses.iter().map(|pose| pose.object));
            detected_objects.extend(detected_robot_poses.iter().map(|pose| pose.object));
            let non_maximum_suppression_duration = non_maximum_suppression_start.elapsed();

            Ok::<_, color_eyre::eyre::Error>(DetectionOutput {
                inference_duration,
                post_processing_duration,
                non_maximum_suppression_duration,
                detected_objects,
                detected_field_poses,
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
                // This model has no human-pose head. Complete announcements for
                // existing consumers without interpreting robot joints as human joints.
                inner: Vec::new(),
            })
            .await?;
        detected_field_poses_pending
            .publish(&TimeWrapper {
                time: image_time,
                inner: output.detected_field_poses,
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

    if image.step != image.width
        || image.data.len() != image.width as usize * image.height as usize * 3 / 2
    {
        bail!("model input requires tightly packed NV12 data");
    }
    if image.width == 0 || image.height == 0 {
        bail!("image dimensions must be nonzero");
    }

    Ok(())
}

fn extract_outputs<'a>(outputs: &'a SessionOutputs<'a>) -> Result<ModelOutputs<'a>> {
    Ok(ModelOutputs {
        objects: extract_output(outputs, TaskHead::Ball)?,
        poses: extract_output(outputs, TaskHead::FieldPose)?,
        robot_poses: extract_output(outputs, TaskHead::RobotPose)?,
    })
}

fn extract_output<'a>(
    outputs: &'a SessionOutputs<'a>,
    head: TaskHead,
) -> Result<ArrayView2<'a, f32>> {
    let output = outputs
        .get(head.output_name())
        .ok_or_else(|| eyre!("missing model output {}", head.output_name()))?
        .try_extract_array::<f32>()?;
    if output.shape() != head.expected_shape() {
        bail!(
            "{} output shape: expected {:?}, got {:?}",
            head.output_name(),
            head.expected_shape(),
            output.shape()
        );
    }
    Ok(output.squeeze().into_dimensionality()?)
}

fn extract_candidate_object_detections(
    outputs: &ModelOutputs,
    confidence_threshold: f32,
) -> Result<Vec<Object<RobocupObjectLabel>>> {
    extract_candidate_pose_detections::<0>(outputs.objects, TaskHead::Ball, confidence_threshold)
        .map(|poses| poses.into_iter().map(|pose| pose.object).collect())
}

fn extract_candidate_pose_detections<const N: usize>(
    output: ArrayView2<'_, f32>,
    head: TaskHead,
    confidence_threshold: f32,
) -> Result<Vec<RobocupPose<[Keypoint; N]>>> {
    output
        .axis_iter(Axis(0))
        .filter(|row| row[4].is_finite() && row[4] >= confidence_threshold)
        .map(|row| {
            let label = match (head, row[5]) {
                (TaskHead::Ball, 0.0) => RobocupObjectLabel::Ball,
                (TaskHead::FieldPose, 0.0) => RobocupObjectLabel::GoalPost,
                (TaskHead::FieldPose, 1.0) => RobocupObjectLabel::LSpot,
                (TaskHead::FieldPose, 2.0) => RobocupObjectLabel::PenaltySpot,
                (TaskHead::FieldPose, 3.0) => RobocupObjectLabel::TSpot,
                (TaskHead::FieldPose, 4.0) => RobocupObjectLabel::XSpot,
                (TaskHead::RobotPose, 0.0) => RobocupObjectLabel::Robot,
                _ => bail!("invalid {} class index {}", head.output_name(), row[5]),
            };
            let object = Object::from([row[0], row[1], row[2], row[3], row[4], label as u8 as f32]);
            let keypoints = std::array::from_fn(|index| {
                let offset = NUMBER_OF_VALUES_PER_OBJECT + index * 3;
                Keypoint {
                    point: point![row[offset], row[offset + 1]],
                    confidence: row[offset + 2],
                }
            });
            Ok(RobocupPose { object, keypoints })
        })
        .collect()
}

trait HasBoundingBox {
    fn label(&self) -> RobocupObjectLabel;
    fn bounding_box(&self) -> &BoundingBox;
}

impl HasBoundingBox for Object<RobocupObjectLabel> {
    fn label(&self) -> RobocupObjectLabel {
        self.label
    }
    fn bounding_box(&self) -> &BoundingBox {
        &self.bounding_box
    }
}

impl<K> HasBoundingBox for RobocupPose<K> {
    fn label(&self) -> RobocupObjectLabel {
        self.object.label
    }
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
            detection.label() != detection_candidate.label()
                || detection
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
    use ndarray::{Array2, array};

    use super::*;

    #[test]
    #[ignore = "requires the Git LFS three-branch ONNX model"]
    fn three_branch_model_contract() {
        let model = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(
            "../../../etc/neural_networks/yolo26m-pose=f17+ball-detection+field-pose+robot-pose~synthetic-all-venues.onnx",
        );
        let mut session = Session::builder()
            .unwrap()
            .with_intra_threads(2)
            .unwrap()
            .commit_from_file(model)
            .unwrap();
        let input = ndarray::Array3::from_elem((224, 272, 6), 128_u8);
        let raw_outputs = session
            .run(inputs!["raw_bytes_input" => TensorRef::from_array_view(input.view()).unwrap()])
            .unwrap();
        let outputs = extract_outputs(&raw_outputs).unwrap();
        assert!(
            outputs
                .objects
                .iter()
                .chain(outputs.poses.iter())
                .chain(outputs.robot_poses.iter())
                .all(|value| value.is_finite())
        );
        assert_eq!(
            extract_candidate_object_detections(&outputs, 0.0)
                .unwrap()
                .len(),
            NUMBER_OF_DETECTIONS
        );
        assert_eq!(
            extract_candidate_pose_detections::<1>(outputs.poses, TaskHead::FieldPose, 0.0)
                .unwrap()
                .len(),
            NUMBER_OF_DETECTIONS
        );
        assert_eq!(
            extract_candidate_pose_detections::<14>(outputs.robot_poses, TaskHead::RobotPose, 0.0)
                .unwrap()
                .len(),
            NUMBER_OF_DETECTIONS
        );
    }

    #[test]
    fn field_classes_map_to_robocup_labels_and_preserve_keypoints() {
        let rows = Array2::from_shape_fn((5, 9), |(row, column)| match column {
            0 | 1 => 10.0,
            2 | 3 => 20.0,
            4 => 0.9,
            5 => row as f32,
            6 => 12.0,
            7 => 18.0,
            8 => 0.7,
            _ => unreachable!(),
        });
        let poses =
            extract_candidate_pose_detections::<1>(rows.view(), TaskHead::FieldPose, 0.5).unwrap();
        assert_eq!(
            poses
                .iter()
                .map(|pose| pose.object.label)
                .collect::<Vec<_>>(),
            [
                RobocupObjectLabel::GoalPost,
                RobocupObjectLabel::LSpot,
                RobocupObjectLabel::PenaltySpot,
                RobocupObjectLabel::TSpot,
                RobocupObjectLabel::XSpot,
            ]
        );
        for pose in poses {
            assert_eq!(pose.object.bounding_box.area.min, point![10.0, 10.0]);
            assert_eq!(pose.object.bounding_box.area.max, point![20.0, 20.0]);
            assert_eq!(pose.keypoints[0].point, point![12.0, 18.0]);
            assert_eq!(pose.keypoints[0].confidence, 0.7);
        }
    }

    #[test]
    fn robot_head_preserves_all_fourteen_joints_in_export_order() {
        let mut rows = Array2::zeros((1, NUMBER_OF_VALUES_PER_ROBOT_POSE));
        rows[[0, 4]] = 0.9;
        for index in 0..14 {
            rows[[0, 6 + index * 3]] = index as f32;
            rows[[0, 7 + index * 3]] = index as f32 + 20.0;
            rows[[0, 8 + index * 3]] = 0.8;
        }
        let poses =
            extract_candidate_pose_detections::<14>(rows.view(), TaskHead::RobotPose, 0.5).unwrap();
        assert_eq!(poses.len(), 1);
        assert_eq!(poses[0].object.label, RobocupObjectLabel::Robot);
        for (index, keypoint) in poses[0].keypoints.iter().enumerate() {
            assert_eq!(keypoint.point, point![index as f32, index as f32 + 20.0]);
            assert_eq!(keypoint.confidence, 0.8);
        }
    }

    #[test]
    fn confidence_filter_keeps_boundary_and_rejects_nan() {
        let rows = array![
            [0.0, 0.0, 10.0, 10.0, 0.49, 0.0],
            [0.0, 0.0, 10.0, 10.0, 0.5, 0.0],
            [0.0, 0.0, 10.0, 10.0, f32::NAN, 0.0],
        ];
        let poses =
            extract_candidate_pose_detections::<0>(rows.view(), TaskHead::Ball, 0.5).unwrap();
        assert_eq!(poses.len(), 1);
        assert_eq!(poses[0].object.label, RobocupObjectLabel::Ball);
    }

    #[test]
    fn invalid_branch_class_returns_error() {
        for class in [1.0, -1.0, 0.5, f32::NAN] {
            let rows = array![[0.0, 0.0, 10.0, 10.0, 0.9, class]];
            assert!(
                extract_candidate_pose_detections::<0>(rows.view(), TaskHead::Ball, 0.5,).is_err()
            );
        }
    }

    #[test]
    fn suppression_keeps_overlapping_field_marks_of_different_classes() {
        let rows = array![
            [0.0, 0.0, 10.0, 10.0, 0.7, 0.0, 5.0, 5.0, 0.9],
            [0.0, 0.0, 10.0, 10.0, 0.9, 0.0, 5.0, 5.0, 0.9],
            [0.0, 0.0, 10.0, 10.0, 0.8, 1.0, 5.0, 5.0, 0.9],
        ];
        let poses =
            extract_candidate_pose_detections::<1>(rows.view(), TaskHead::FieldPose, 0.5).unwrap();
        let poses = non_maximum_suppression(poses, 0.4);
        assert_eq!(poses.len(), 2);
        assert_eq!(poses[0].object.label, RobocupObjectLabel::GoalPost);
        assert_eq!(poses[0].object.bounding_box.confidence, 0.9);
        assert_eq!(poses[1].object.label, RobocupObjectLabel::LSpot);
    }

    #[test]
    fn input_requires_packed_nv12() {
        let mut image = Image {
            encoding: "nv12".to_owned(),
            width: 544,
            height: 448,
            step: 544,
            data: vec![128; 544 * 448 * 3 / 2].into(),
            ..Default::default()
        };
        assert!(check_image(&image).is_ok());
        image.step += 32;
        assert!(check_image(&image).is_err());
        image.step = 544;
        image.data = vec![128; 544 * 448].into();
        assert!(check_image(&image).is_err());
    }
}
