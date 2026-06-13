use path_serde::{PathDeserialize, PathIntrospect, PathSerialize};
use ros_z::{
    Message, SerdeCdrCodec,
    schema::{MessageSchema, SchemaBuilder},
};
use ros_z_schema::{SchemaError, TypeDef};
use serde::{Deserialize, Serialize};

pub use booster_sdk::types::RobotMode;

#[derive(
    Clone,
    Copy,
    Debug,
    Deserialize,
    Eq,
    Hash,
    PartialEq,
    Serialize,
    PathSerialize,
    PathDeserialize,
    PathIntrospect,
)]
pub struct SequencedRobotMode {
    pub mode: RobotMode,
    pub sequence_number: u64,
}

impl MessageSchema for SequencedRobotMode {
    fn build_schema(builder: &mut SchemaBuilder) -> Result<TypeDef, SchemaError> {
        builder.define_message_struct::<Self>(|fields| {
            fields.field::<i32>("mode")?;
            fields.field::<u64>("sequence_number")?;
            Ok(())
        })
    }
}

impl Message for SequencedRobotMode {
    type Codec = SerdeCdrCodec<Self>;

    fn type_name() -> String {
        "types::robot_mode::SequencedRobotMode".to_string()
    }
}

impl Default for SequencedRobotMode {
    fn default() -> Self {
        Self {
            mode: RobotMode::Unknown,
            sequence_number: 0,
        }
    }
}
