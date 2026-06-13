use path_serde::{PathDeserialize, PathIntrospect, PathSerialize};
use ros_z::Message;
use serde::{Deserialize, Serialize};

#[derive(
    Clone,
    Copy,
    Debug,
    Default,
    Deserialize,
    Eq,
    Hash,
    PartialEq,
    Serialize,
    PathSerialize,
    PathDeserialize,
    PathIntrospect,
    Message,
)]
pub enum RobotMode {
    #[default]
    Unknown,
    Damping,
    Prepare,
    Walking,
}

#[derive(
    Clone,
    Copy,
    Debug,
    Default,
    Deserialize,
    Eq,
    Hash,
    PartialEq,
    Serialize,
    PathSerialize,
    PathDeserialize,
    PathIntrospect,
    Message,
)]
pub struct SequencedRobotMode {
    pub mode: RobotMode,
    pub sequence_number: u64,
}
