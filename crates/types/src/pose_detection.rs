use std::{ops::Index, time::SystemTime};

use color_eyre::Result;
use path_serde::{PathDeserialize, PathIntrospect, PathSerialize};
use serde::{Deserialize, Serialize};

use coordinate_systems::Pixel;
use hsl_network_messages::Team;
use linear_algebra::{Point2, point};

use crate::object_detection::{
    FieldFeatureLabel, LabelIndex, NUMBER_OF_VALUES_PER_OBJECT, Object, YOLOObjectLabel,
};

#[derive(
    Debug,
    Clone,
    Copy,
    Serialize,
    Deserialize,
    PathSerialize,
    PathDeserialize,
    PathIntrospect,
    ros_z::Message,
)]
pub enum DetectionRegion {
    Narrow,
    Full,
}

pub const OVERALL_KEYPOINT_INDEX_MASK: [usize; 15] =
    [0, 1, 2, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];
pub const VISUAL_REFEREE_KEYPOINT_INDEX_MASK: [usize; 8] = [5, 6, 7, 8, 9, 10, 15, 16];
pub const NUMBER_OF_KEYPOINTS_PER_HUMANOID_POSE: usize = 17;
pub const NUMBER_OF_KEYPOINTS_PER_FIELD_FEATURE_POSE: usize = 1;
pub const NUMBER_OF_VALUES_PER_KEYPOINTS: usize = 3;
pub const NUMBER_OF_VALUES_PER_HUMANOID_KEYPOINTS: usize =
    NUMBER_OF_VALUES_PER_KEYPOINTS * NUMBER_OF_KEYPOINTS_PER_HUMANOID_POSE;
pub const NUMBER_OF_VALUES_PER_FIELD_FEATURE_KEYPOINTS: usize =
    NUMBER_OF_VALUES_PER_KEYPOINTS * NUMBER_OF_KEYPOINTS_PER_FIELD_FEATURE_POSE;
pub const NUMBER_OF_VALUES_PER_HUMANOID_POSE: usize = NUMBER_OF_VALUES_PER_OBJECT
    + NUMBER_OF_VALUES_PER_KEYPOINTS * NUMBER_OF_KEYPOINTS_PER_HUMANOID_POSE;
pub const NUMBER_OF_VALUES_PER_FIELD_FEATURE_POSE: usize = NUMBER_OF_VALUES_PER_OBJECT
    + NUMBER_OF_VALUES_PER_KEYPOINTS * NUMBER_OF_KEYPOINTS_PER_FIELD_FEATURE_POSE;
pub const NUMBER_OF_VALUES_PER_POSE: usize = NUMBER_OF_VALUES_PER_HUMANOID_POSE;
pub const POSE_KEYPOINT_OFFSET: usize = NUMBER_OF_VALUES_PER_OBJECT;

#[derive(
    Debug,
    Clone,
    Copy,
    Serialize,
    Deserialize,
    PathSerialize,
    PathDeserialize,
    PathIntrospect,
    ros_z::Message,
)]
pub struct Keypoint {
    pub point: Point2<Pixel>,
    pub confidence: f32,
}

#[derive(
    Debug,
    Clone,
    Copy,
    Serialize,
    Deserialize,
    PathSerialize,
    PathDeserialize,
    PathIntrospect,
    ros_z::Message,
)]
pub struct HumanoidKeypoints {
    pub left_eye: Keypoint,
    pub right_eye: Keypoint,
    pub nose: Keypoint,
    pub left_ear: Keypoint,
    pub right_ear: Keypoint,
    pub left_shoulder: Keypoint,
    pub right_shoulder: Keypoint,
    pub left_elbow: Keypoint,
    pub right_elbow: Keypoint,
    pub left_hand: Keypoint,
    pub right_hand: Keypoint,
    pub left_hip: Keypoint,
    pub right_hip: Keypoint,
    pub left_knee: Keypoint,
    pub right_knee: Keypoint,
    pub left_foot: Keypoint,
    pub right_foot: Keypoint,
}

impl HumanoidKeypoints {
    pub fn as_array(self) -> [Keypoint; NUMBER_OF_KEYPOINTS_PER_HUMANOID_POSE] {
        Into::<[Keypoint; NUMBER_OF_KEYPOINTS_PER_HUMANOID_POSE]>::into(self)
    }
}

impl From<&[f32; 51]> for HumanoidKeypoints {
    fn from(keypoints_slice: &[f32; 51]) -> Self {
        let mut keypoints_iter = keypoints_slice
            .chunks_exact(3)
            .map(|keypoint_chunk| Keypoint {
                point: point![keypoint_chunk[0], keypoint_chunk[1]],
                confidence: keypoint_chunk[2],
            });

        Self {
            left_eye: keypoints_iter.next().unwrap(),
            right_eye: keypoints_iter.next().unwrap(),
            nose: keypoints_iter.next().unwrap(),
            left_ear: keypoints_iter.next().unwrap(),
            right_ear: keypoints_iter.next().unwrap(),
            left_shoulder: keypoints_iter.next().unwrap(),
            right_shoulder: keypoints_iter.next().unwrap(),
            left_elbow: keypoints_iter.next().unwrap(),
            right_elbow: keypoints_iter.next().unwrap(),
            left_hand: keypoints_iter.next().unwrap(),
            right_hand: keypoints_iter.next().unwrap(),
            left_hip: keypoints_iter.next().unwrap(),
            right_hip: keypoints_iter.next().unwrap(),
            left_knee: keypoints_iter.next().unwrap(),
            right_knee: keypoints_iter.next().unwrap(),
            left_foot: keypoints_iter.next().unwrap(),
            right_foot: keypoints_iter.next().unwrap(),
        }
    }
}

impl Index<usize> for HumanoidKeypoints {
    fn index(&self, index: usize) -> &Keypoint {
        match index {
            0 => &self.left_eye,
            1 => &self.right_eye,
            2 => &self.nose,
            3 => &self.left_ear,
            4 => &self.right_ear,
            5 => &self.left_shoulder,
            6 => &self.right_shoulder,
            7 => &self.left_elbow,
            8 => &self.right_elbow,
            9 => &self.left_hand,
            10 => &self.right_hand,
            11 => &self.left_hip,
            12 => &self.right_hip,
            13 => &self.left_knee,
            14 => &self.right_knee,
            15 => &self.left_foot,
            16 => &self.right_foot,
            _ => panic!("out of bounds: {index}"),
        }
    }
    type Output = Keypoint;
}

impl From<HumanoidKeypoints> for [Keypoint; NUMBER_OF_KEYPOINTS_PER_HUMANOID_POSE] {
    fn from(keypoints: HumanoidKeypoints) -> Self {
        [
            keypoints.left_eye,
            keypoints.right_eye,
            keypoints.nose,
            keypoints.left_ear,
            keypoints.right_ear,
            keypoints.left_shoulder,
            keypoints.right_shoulder,
            keypoints.left_elbow,
            keypoints.right_elbow,
            keypoints.left_hand,
            keypoints.right_hand,
            keypoints.left_hip,
            keypoints.right_hip,
            keypoints.left_knee,
            keypoints.right_knee,
            keypoints.left_foot,
            keypoints.right_foot,
        ]
    }
}

#[derive(
    Debug,
    Clone,
    Copy,
    Serialize,
    Deserialize,
    PathSerialize,
    PathDeserialize,
    PathIntrospect,
    ros_z::Message,
)]
pub struct FieldFeatureKeypoints {
    pub feature: Keypoint,
}

impl FieldFeatureKeypoints {
    pub fn as_array(self) -> [Keypoint; NUMBER_OF_KEYPOINTS_PER_FIELD_FEATURE_POSE] {
        [self.feature]
    }
}

impl From<&[f32; NUMBER_OF_VALUES_PER_FIELD_FEATURE_KEYPOINTS]> for FieldFeatureKeypoints {
    fn from(keypoints_slice: &[f32; NUMBER_OF_VALUES_PER_FIELD_FEATURE_KEYPOINTS]) -> Self {
        Self {
            feature: Keypoint {
                point: point![keypoints_slice[0], keypoints_slice[1]],
                confidence: keypoints_slice[2],
            },
        }
    }
}

impl Index<usize> for FieldFeatureKeypoints {
    fn index(&self, index: usize) -> &Keypoint {
        match index {
            0 => &self.feature,
            _ => panic!("out of bounds: {index}"),
        }
    }
    type Output = Keypoint;
}

impl From<FieldFeatureKeypoints> for [Keypoint; NUMBER_OF_KEYPOINTS_PER_FIELD_FEATURE_POSE] {
    fn from(keypoints: FieldFeatureKeypoints) -> Self {
        keypoints.as_array()
    }
}

#[derive(
    Debug,
    Clone,
    Copy,
    Serialize,
    Deserialize,
    PathSerialize,
    PathDeserialize,
    PathIntrospect,
    ros_z::Message,
)]
pub struct Pose<T, K = HumanoidKeypoints> {
    pub object: Object<T>,
    pub keypoints: K,
}

impl<T, K> Pose<T, K> {
    pub fn new(object: Object<T>, keypoints: K) -> Pose<T, K> {
        Self { object, keypoints }
    }
}

pub type HumanoidPose<T> = Pose<T, HumanoidKeypoints>;
pub type FieldFeaturePose = Pose<FieldFeatureLabel, FieldFeatureKeypoints>;

impl<T> From<&[f32; NUMBER_OF_VALUES_PER_HUMANOID_POSE]> for Pose<T, HumanoidKeypoints>
where
    T: LabelIndex,
{
    fn from(values: &[f32; NUMBER_OF_VALUES_PER_HUMANOID_POSE]) -> Self {
        let object_detection_values: [f32; NUMBER_OF_VALUES_PER_OBJECT] = values
            [..POSE_KEYPOINT_OFFSET]
            .try_into()
            .unwrap_or_else(|_| {
                panic!(
                    "slice does not contain at least {} values",
                    POSE_KEYPOINT_OFFSET
                )
            });

        let keypoint_values: [f32; NUMBER_OF_VALUES_PER_HUMANOID_KEYPOINTS] = values
            [POSE_KEYPOINT_OFFSET..]
            .try_into()
            .unwrap_or_else(|_| {
                panic!(
                    "slice does not contain at least {} values",
                    NUMBER_OF_VALUES_PER_HUMANOID_KEYPOINTS
                )
            });

        Pose {
            object: Object::from(object_detection_values),
            keypoints: HumanoidKeypoints::from(&keypoint_values),
        }
    }
}

impl<T> From<&[f32; NUMBER_OF_VALUES_PER_FIELD_FEATURE_POSE]> for Pose<T, FieldFeatureKeypoints>
where
    T: LabelIndex,
{
    fn from(values: &[f32; NUMBER_OF_VALUES_PER_FIELD_FEATURE_POSE]) -> Self {
        let object_detection_values: [f32; NUMBER_OF_VALUES_PER_OBJECT] = values
            [..POSE_KEYPOINT_OFFSET]
            .try_into()
            .unwrap_or_else(|_| {
                panic!(
                    "slice does not contain at least {} values",
                    POSE_KEYPOINT_OFFSET
                )
            });

        let keypoint_values: [f32; NUMBER_OF_VALUES_PER_FIELD_FEATURE_KEYPOINTS] = values
            [POSE_KEYPOINT_OFFSET..]
            .try_into()
            .unwrap_or_else(|_| {
                panic!(
                    "slice does not contain at least {} values",
                    NUMBER_OF_VALUES_PER_FIELD_FEATURE_KEYPOINTS
                )
            });

        Pose {
            object: Object::from(object_detection_values),
            keypoints: FieldFeatureKeypoints::from(&keypoint_values),
        }
    }
}

#[derive(
    Debug,
    Clone,
    Copy,
    Serialize,
    Deserialize,
    PathSerialize,
    PathDeserialize,
    PathIntrospect,
    ros_z::Message,
)]
pub struct RefereePoseCandidate {
    pub pose: HumanoidPose<YOLOObjectLabel>,
    pub distance_to_referee_position: f32,
}

#[derive(
    Debug,
    Clone,
    Copy,
    Default,
    Serialize,
    Deserialize,
    PathSerialize,
    PathDeserialize,
    PathIntrospect,
)]
pub struct ReadySignalDetectionResult {
    pub detected_own_ready_signal: bool,
    pub did_detect_any_ready_pose_this_cycle: bool,
}

#[derive(
    Debug,
    Clone,
    Copy,
    Default,
    Serialize,
    Deserialize,
    PathSerialize,
    PathDeserialize,
    PathIntrospect,
)]
pub struct FreeKickSignalDetectionResult {
    pub own_detected_kicking_team: Option<Team>,
    pub did_detect_any_free_kick_pose_this_cycle: bool,
}

#[derive(
    Debug, Clone, Copy, Serialize, Deserialize, PathSerialize, PathDeserialize, PathIntrospect,
)]
pub struct TimeTaggedKickingTeamDetections {
    pub time: SystemTime,
    pub detected_kicking_team: Option<Team>,
}

#[derive(
    Debug,
    Default,
    Clone,
    Copy,
    Serialize,
    Deserialize,
    PartialEq,
    PathSerialize,
    PathDeserialize,
    PathIntrospect,
)]
pub enum ReadySignalState {
    #[default]
    WaitingForDetections,
    WaitingForOpponentPenalties {
        active_since: SystemTime,
    },
    WaitingForOwnPenalties {
        active_since: SystemTime,
    },
    GoToReady,
}
