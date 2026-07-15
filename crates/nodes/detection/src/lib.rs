use std::{boxed::Box, future::Future, pin::Pin, sync::Arc, time::Duration};

use color_eyre::{Result, eyre::bail};
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
        FieldFeatureDetection, FieldFeatureLabel, NUMBER_OF_VALUES_PER_POSE, Pose, RobotKeypoints,
        RobotPoseDetection,
    },
    time_wrapper::TimeWrapper,
};

pub const NUMBER_OF_DETECTIONS: usize = 300;

#[derive(Clone, Copy, Debug)]
enum TaskHead {
    ObjectDetection,
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
            TaskHead::PersonPose => "person_pose_output",
            TaskHead::RobotPose => "robot_pose_output",
            TaskHead::FieldFeature => "field_feature_output",
        }
    }

    fn expected_shape(self) -> &'static [usize] {
        match self {
            Self::ObjectDetection => &[1, NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_OBJECT],
            Self::PersonPose => &[1, NUMBER_OF_DETECTIONS, NUMBER_OF_VALUES_PER_POSE],
            Self::RobotPose => &[1, NUMBER_OF_DETECTIONS, 14, 3],
            Self::FieldFeature => &[1, NUMBER_OF_DETECTIONS, 4],
        }
    }
}

#[derive(Debug)]
struct ModelOutputs<'a> {
    objects: ArrayView2<'a, f32>,
    poses: ArrayView2<'a, f32>,
    robot_poses: ArrayView3<'a, f32>,
    field_features: ArrayView2<'a, f32>,
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
    let detected_field_features_pub = node
        .announcing_publisher::<TimeWrapper<Vec<FieldFeatureDetection>>>("detected_field_features")
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
            )?;
            let detected_robot_poses = extract_robot_pose_detections(
                &outputs,
                parameters
                    .robot_pose_detection_parameters
                    .minimum_candidate_confidence,
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
    let objects_output =
        outputs[TaskHead::ObjectDetection.output_name()].try_extract_array::<f32>()?;
    if objects_output.shape() != TaskHead::ObjectDetection.expected_shape() {
        bail!(
            "object detection output not of expected shape. Expected: {:?}, got: {:?}",
            TaskHead::ObjectDetection.expected_shape(),
            objects_output.shape()
        )
    }
    let reshaped_objects_output = objects_output.squeeze().into_dimensionality()?;

    let poses_output = outputs[TaskHead::PersonPose.output_name()].try_extract_array::<f32>()?;
    if poses_output.shape() != TaskHead::PersonPose.expected_shape() {
        bail!(
            "pose detection output not of expected shape. Expected: {:?}, got: {:?}",
            TaskHead::PersonPose.expected_shape(),
            poses_output.shape()
        )
    }
    let reshaped_pose_output = poses_output.squeeze().into_dimensionality()?;

    let robot_poses_output =
        outputs[TaskHead::RobotPose.output_name()].try_extract_array::<f32>()?;
    if robot_poses_output.shape() != TaskHead::RobotPose.expected_shape() {
        bail!(
            "robot pose output not of expected shape. Expected: {:?}, got: {:?}",
            TaskHead::RobotPose.expected_shape(),
            robot_poses_output.shape()
        )
    }
    let reshaped_robot_pose_output = robot_poses_output.squeeze().into_dimensionality()?;

    let field_features_output =
        outputs[TaskHead::FieldFeature.output_name()].try_extract_array::<f32>()?;
    if field_features_output.shape() != TaskHead::FieldFeature.expected_shape() {
        bail!(
            "field feature output not of expected shape. Expected: {:?}, got: {:?}",
            TaskHead::FieldFeature.expected_shape(),
            field_features_output.shape()
        )
    }
    let reshaped_field_feature_output = field_features_output.squeeze().into_dimensionality()?;

    Ok(ModelOutputs {
        objects: reshaped_objects_output,
        poses: reshaped_pose_output,
        robot_poses: reshaped_robot_pose_output,
        field_features: reshaped_field_feature_output,
    })
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
    Ok(outputs
        .poses
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
    outputs
        .objects
        .axis_iter(Axis(0))
        .zip(outputs.robot_poses.axis_iter(Axis(0)))
        .filter_map(|(object_row, keypoint_rows)| {
            let confidence = object_row[4];
            let class_index = object_row[5] as usize;
            if !confidence.is_finite()
                || confidence < confidence_threshold
                || class_index != RobocupObjectLabel::Robot as usize
            {
                return None;
            }
            Some((object_row, keypoint_rows))
        })
        .map(|(object_row, keypoint_rows)| {
            let object_values: [f32; NUMBER_OF_VALUES_PER_OBJECT] = object_row
                .as_slice()
                .expect("slice is not contiguous")
                .try_into()
                .expect("object row has invalid length");
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
    outputs
        .field_features
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
    use ndarray::{Array2, Array3};

    use super::*;

    #[test]
    fn deployment_output_contract_has_four_named_shapes() {
        assert_eq!(TaskHead::ObjectDetection.output_name(), "object_output");
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
            poses: poses.view(),
            robot_poses: robot_poses.view(),
            field_features: field_features.view(),
        };

        let detections = extract_field_feature_detections(&outputs, 0.5);

        assert_eq!(detections.len(), 1);
        assert_eq!(detections[0].confidence, 0.9);
        assert_eq!(detections[0].label, FieldFeatureLabel::TSpot);
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
            poses: poses.view(),
            robot_poses: robot_poses.view(),
            field_features: field_features.view(),
        };

        let detections = extract_field_feature_detections(&outputs, 0.0);

        assert_eq!(detections.len(), NUMBER_OF_DETECTIONS - 1);
    }
}
