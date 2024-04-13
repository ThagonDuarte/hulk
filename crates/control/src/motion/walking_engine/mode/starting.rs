use crate::motion::walking_engine::{
    kicking::KickState, step_plan::StepPlan, step_state::StepState, stiffness::Stiffness as _,
};

use super::{
    super::CycleContext, kicking::Kicking, standing::Standing, Mode, WalkTransition, Walking,
};
use serde::{Deserialize, Serialize};
use serialize_hierarchy::SerializeHierarchy;
use types::{
    joints::body::BodyJoints, motion_command::KickVariant, motor_commands::MotorCommands,
    step_plan::Step, support_foot::Side, walking_engine::WalkingEngineParameters,
};

#[derive(Clone, Copy, Debug, Serialize, Deserialize, SerializeHierarchy)]
pub struct Starting {
    pub step: StepState,
}

impl Starting {
    pub fn new(context: &CycleContext, support_side: Side, joints: &BodyJoints) -> Self {
        let plan = StepPlan::new_from_request(context.parameters, Step::ZERO, support_side, joints);
        let step = StepState::new(plan);
        Self { step }
    }
}

impl WalkTransition for Starting {
    fn stand(self, context: &CycleContext, _joints: &BodyJoints) -> Mode {
        let current_step = self.step;
        if current_step.is_support_switched(context)
            || current_step.is_timeouted(context.parameters)
        {
            return Mode::Standing(Standing {});
        }

        Mode::Starting(self)
    }

    fn walk(self, context: &CycleContext, joints: &BodyJoints, requested_step: Step) -> Mode {
        let current_step = self.step;

        if current_step.is_support_switched(context)
            || current_step.is_timeouted(context.parameters)
        {
            return Mode::Walking(Walking::new(
                context,
                requested_step,
                current_step.plan.support_side.opposite(),
                joints,
                Step::ZERO,
            ));
        }

        Mode::Starting(self)
    }

    fn kick(
        self,
        context: &CycleContext,
        joints: &BodyJoints,
        variant: KickVariant,
        kick_side: Side,
        strength: f32,
    ) -> Mode {
        let current_step = self.step;

        if current_step.is_timeouted(context.parameters) {
            return Mode::Walking(Walking::new(
                context,
                Step::ZERO,
                current_step.plan.support_side.opposite(),
                joints,
                Step::ZERO,
            ));
        }

        if current_step.is_support_switched(context) {
            let next_support_side = current_step.plan.support_side.opposite();
            if next_support_side == kick_side {
                return Mode::Walking(Walking::new(
                    context,
                    Step::ZERO,
                    current_step.plan.support_side.opposite(),
                    joints,
                    Step::ZERO,
                ));
            }

            let kick = KickState::new(variant, kick_side, strength);
            return Mode::Kicking(Kicking::new(
                context,
                kick,
                current_step.plan.support_side.opposite(),
                joints,
            ));
        }
        Mode::Starting(self)
    }
}

impl Starting {
    pub fn compute_commands(
        &self,
        parameters: &WalkingEngineParameters,
    ) -> MotorCommands<BodyJoints> {
        self.step.compute_joints(parameters).apply_stiffness(
            parameters.stiffnesses.leg_stiffness_walk,
            parameters.stiffnesses.arm_stiffness,
        )
    }

    pub fn tick(&mut self, context: &CycleContext, gyro: nalgebra::Vector3<f32>) {
        self.step.tick(context, gyro);
    }
}
