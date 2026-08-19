use std::{ops::Index, time::SystemTime};

use serde::{Deserialize, Serialize};

use coordinate_systems::Pixel;
use hsl_network_messages::Team;
use linear_algebra::{Point2, point};

use crate::object_detection::{LabelIndex, NUMBER_OF_VALUES_PER_OBJECT, Object, YOLOObjectLabel};

#[derive(Debug, Clone, Copy, Serialize, Deserialize, ros_z::Message)]
pub enum DetectionRegion {
    Narrow,
    Full,
}

pub const OVERALL_KEYPOINT_INDEX_MASK: [usize; 15] =
    [0, 1, 2, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];
pub const VISUAL_REFEREE_KEYPOINT_INDEX_MASK: [usize; 8] = [5, 6, 7, 8, 9, 10, 15, 16];
pub const NUMBER_OF_VALUES_PER_POSE: usize = 57;
pub const NUMBER_OF_VALUES_PER_ROBOT_POSE: usize = 48;
pub const POSE_KEYPOINT_OFFSET: usize = NUMBER_OF_VALUES_PER_OBJECT;
pub const POSE_SKELETON_EDGES: [(usize, usize); 16] = [
    (2, 0),
    (2, 1),
    (0, 3),
    (1, 4),
    (5, 6),
    (5, 11),
    (6, 12),
    (11, 12),
    (5, 7),
    (6, 8),
    (7, 9),
    (8, 10),
    (11, 13),
    (12, 14),
    (13, 15),
    (14, 16),
];
pub const ROBOT_POSE_SKELETON_EDGES: [(usize, usize); 13] = [
    (1, 0),
    (1, 2),
    (2, 3),
    (3, 4),
    (1, 5),
    (5, 6),
    (6, 7),
    (1, 8),
    (8, 9),
    (9, 10),
    (1, 11),
    (11, 12),
    (12, 13),
];

#[derive(Debug, Clone, Copy, Serialize, Deserialize, ros_z::Message)]
pub struct Keypoint {
    pub point: Point2<Pixel>,
    pub confidence: f32,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, ros_z::Message)]
pub struct Keypoints {
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

impl Keypoints {
    pub fn as_array(self) -> [Keypoint; 17] {
        Into::<[Keypoint; 17]>::into(self)
    }
}

impl From<&[f32; 51]> for Keypoints {
    fn from(keypoints_slice: &[f32; 51]) -> Self {
        let keypoints = keypoints_slice
            .chunks_exact(3)
            .map(|keypoint_chunk| Keypoint {
                point: point![keypoint_chunk[0], keypoint_chunk[1]],
                confidence: keypoint_chunk[2],
            })
            .collect::<Vec<_>>();

        Self {
            nose: keypoints[0],
            left_eye: keypoints[1],
            right_eye: keypoints[2],
            left_ear: keypoints[3],
            right_ear: keypoints[4],
            left_shoulder: keypoints[5],
            right_shoulder: keypoints[6],
            left_elbow: keypoints[7],
            right_elbow: keypoints[8],
            left_hand: keypoints[9],
            right_hand: keypoints[10],
            left_hip: keypoints[11],
            right_hip: keypoints[12],
            left_knee: keypoints[13],
            right_knee: keypoints[14],
            left_foot: keypoints[15],
            right_foot: keypoints[16],
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, ros_z::Message)]
pub struct RobotKeypoints {
    pub nose: Keypoint,
    pub neck: Keypoint,
    pub right_shoulder: Keypoint,
    pub right_elbow: Keypoint,
    pub right_wrist: Keypoint,
    pub left_shoulder: Keypoint,
    pub left_elbow: Keypoint,
    pub left_wrist: Keypoint,
    pub right_hip: Keypoint,
    pub right_knee: Keypoint,
    pub right_ankle: Keypoint,
    pub left_hip: Keypoint,
    pub left_knee: Keypoint,
    pub left_ankle: Keypoint,
}

impl RobotKeypoints {
    pub fn as_array(self) -> [Keypoint; 14] {
        [
            self.nose,
            self.neck,
            self.right_shoulder,
            self.right_elbow,
            self.right_wrist,
            self.left_shoulder,
            self.left_elbow,
            self.left_wrist,
            self.right_hip,
            self.right_knee,
            self.right_ankle,
            self.left_hip,
            self.left_knee,
            self.left_ankle,
        ]
    }
}

impl From<&[f32; 42]> for RobotKeypoints {
    fn from(values: &[f32; 42]) -> Self {
        let keypoints = values
            .chunks_exact(3)
            .map(|chunk| Keypoint {
                point: point![chunk[0], chunk[1]],
                confidence: chunk[2],
            })
            .collect::<Vec<_>>();
        Self {
            nose: keypoints[0],
            neck: keypoints[1],
            right_shoulder: keypoints[2],
            right_elbow: keypoints[3],
            right_wrist: keypoints[4],
            left_shoulder: keypoints[5],
            left_elbow: keypoints[6],
            left_wrist: keypoints[7],
            right_hip: keypoints[8],
            right_knee: keypoints[9],
            right_ankle: keypoints[10],
            left_hip: keypoints[11],
            left_knee: keypoints[12],
            left_ankle: keypoints[13],
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, ros_z::Message)]
pub struct RobotPoseDetection {
    pub object: Object<crate::object_detection::RobocupObjectLabel>,
    pub keypoints: RobotKeypoints,
}

impl From<&[f32; NUMBER_OF_VALUES_PER_ROBOT_POSE]> for RobotPoseDetection {
    fn from(values: &[f32; NUMBER_OF_VALUES_PER_ROBOT_POSE]) -> Self {
        let mut object_values: [f32; NUMBER_OF_VALUES_PER_OBJECT] = values[..POSE_KEYPOINT_OFFSET]
            .try_into()
            .expect("robot pose must contain an object detection");
        object_values[5] = crate::object_detection::RobocupObjectLabel::Robot as usize as f32;
        let keypoint_values: &[f32; 42] = values[POSE_KEYPOINT_OFFSET..]
            .try_into()
            .expect("robot pose must contain 14 keypoints");

        Self {
            object: Object::from(object_values),
            keypoints: RobotKeypoints::from(keypoint_values),
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, ros_z::Message)]
pub enum FieldFeatureLabel {
    GoalPost = 0,
    LSpot = 1,
    TSpot = 2,
    PenaltySpot = 3,
    XSpot = 4,
}

impl FieldFeatureLabel {
    pub fn from_index(index: usize) -> Self {
        match index {
            0 => Self::GoalPost,
            1 => Self::LSpot,
            2 => Self::TSpot,
            3 => Self::PenaltySpot,
            4 => Self::XSpot,
            _ => panic!("invalid field-feature class index: {index}"),
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, ros_z::Message)]
pub struct FieldFeatureDetection {
    pub point: Point2<Pixel>,
    pub confidence: f32,
    pub label: FieldFeatureLabel,
}

impl Index<usize> for Keypoints {
    fn index(&self, index: usize) -> &Keypoint {
        match index {
            0 => &self.nose,
            1 => &self.left_eye,
            2 => &self.right_eye,
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

impl From<Keypoints> for [Keypoint; 17] {
    fn from(keypoints: Keypoints) -> Self {
        [
            keypoints.nose,
            keypoints.left_eye,
            keypoints.right_eye,
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

#[derive(Debug, Clone, Copy, Serialize, Deserialize, ros_z::Message)]
pub struct Pose<T> {
    pub object: Object<T>,
    pub keypoints: Keypoints,
}

impl<T> Pose<T> {
    pub fn new(object: Object<T>, keypoints: Keypoints) -> Pose<T> {
        Self { object, keypoints }
    }
}

impl<T> From<&[f32; 57]> for Pose<T>
where
    T: LabelIndex,
{
    fn from(values: &[f32; 57]) -> Self {
        let object_detection_values: [f32; 6] = values[..POSE_KEYPOINT_OFFSET]
            .try_into()
            .unwrap_or_else(|_| {
                panic!(
                    "slice does not contain atleast {} values",
                    POSE_KEYPOINT_OFFSET
                )
            });

        let keypoint_values = &values[POSE_KEYPOINT_OFFSET..]
            .try_into()
            .unwrap_or_else(|_| {
                panic!(
                    "slice does not contain atleast {} values",
                    NUMBER_OF_VALUES_PER_POSE - POSE_KEYPOINT_OFFSET
                )
            });

        Pose {
            object: Object::from(object_detection_values),
            keypoints: Keypoints::from(keypoint_values),
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, ros_z::Message)]
pub struct RefereePoseCandidate {
    pub pose: Pose<YOLOObjectLabel>,
    pub distance_to_referee_position: f32,
}

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct ReadySignalDetectionResult {
    pub detected_own_ready_signal: bool,
    pub did_detect_any_ready_pose_this_cycle: bool,
}

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct FreeKickSignalDetectionResult {
    pub own_detected_kicking_team: Option<Team>,
    pub did_detect_any_free_kick_pose_this_cycle: bool,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct TimeTaggedKickingTeamDetections {
    pub time: SystemTime,
    pub detected_kicking_team: Option<Team>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn person_keypoints_use_standard_coco_order() {
        let mut values = [0.0; 51];
        for (index, keypoint) in values.chunks_exact_mut(3).enumerate() {
            keypoint[0] = index as f32;
            keypoint[1] = index as f32 + 0.5;
            keypoint[2] = 1.0;
        }

        let keypoints = Keypoints::from(&values);

        assert_eq!(keypoints.nose.point.x(), 0.0);
        assert_eq!(keypoints.left_eye.point.x(), 1.0);
        assert_eq!(keypoints.right_foot.point.x(), 16.0);
        assert_eq!(keypoints.as_array()[0].point.x(), 0.0);
    }

    #[test]
    fn robot_keypoints_use_dhrp_order() {
        let mut values = [0.0; 42];
        for (index, keypoint) in values.chunks_exact_mut(3).enumerate() {
            keypoint[0] = index as f32;
            keypoint[2] = 1.0;
        }

        let keypoints = RobotKeypoints::from(&values);

        assert_eq!(keypoints.nose.point.x(), 0.0);
        assert_eq!(keypoints.neck.point.x(), 1.0);
        assert_eq!(keypoints.left_ankle.point.x(), 13.0);
        assert_eq!(keypoints.as_array()[13].point.x(), 13.0);
    }

    #[test]
    fn robot_pose_maps_the_single_dhrp_class_to_robot() {
        let mut values = [0.0; NUMBER_OF_VALUES_PER_ROBOT_POSE];
        values[..6].copy_from_slice(&[1.0, 2.0, 3.0, 4.0, 0.9, 0.0]);
        for (index, keypoint) in values[POSE_KEYPOINT_OFFSET..]
            .chunks_exact_mut(3)
            .enumerate()
        {
            keypoint[0] = index as f32;
            keypoint[2] = 1.0;
        }

        let pose = RobotPoseDetection::from(&values);

        assert_eq!(
            pose.object.label,
            crate::object_detection::RobocupObjectLabel::Robot
        );
        assert_eq!(pose.object.bounding_box.confidence, 0.9);
        assert_eq!(pose.keypoints.left_ankle.point.x(), 13.0);
    }
}

#[derive(Debug, Default, Clone, Copy, Serialize, Deserialize, PartialEq)]
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
