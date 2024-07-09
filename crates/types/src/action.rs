use path_serde::{PathDeserialize, PathIntrospect, PathSerialize};
use serde::{Deserialize, Serialize};

#[derive(
    Debug, Clone, Copy, PathSerialize, PathDeserialize, PathIntrospect, Serialize, Deserialize,
)]
pub enum Action {
    Calibrate,
    DefendGoal,
    DefendKickOff,
    DefendLeft,
    DefendPenaltyKick,
    DefendRight,
    Dribble,
    FallSafely,
    Initial,
    InterceptBall,
    Jump,
    LookAround,
    NoGroundContact,
    Penalize,
    PrepareJump,
    Search,
    SearchForLostBall,
    SitDown,
    Stand,
    StandUp,
    WideStance,
    SupportLeft,
    SupportRight,
    SupportStriker,
    Unstiff,
    WalkToKickOff,
    WalkToPenaltyKick,
}
