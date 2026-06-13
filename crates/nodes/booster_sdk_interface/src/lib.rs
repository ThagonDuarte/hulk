use std::{boxed::Box, fmt::Display, future::Future, pin::Pin, sync::Arc, time::Duration};

use booster_sdk::{
    client::{BoosterClient, light_control::LightControlClient},
    types::RobotMode,
};
use color_eyre::{Result, eyre::WrapErr};
use kinematics::joints::head::HeadJoints;
use ros_z::{prelude::*, qos::QosDurability, time::Time};
use serde::{Deserialize, Serialize};
use tokio::sync::watch;
use types::{
    buttons::{ButtonPressType, Buttons},
    motion_command::{MotionCommand, SequencedMotionCommand},
    robot_mode::SequencedRobotMode,
};

mod control;
mod kick_transport;

#[derive(Debug, Clone, Serialize, Deserialize, Message)]
#[serde(deny_unknown_fields)]
pub struct WalkingParameters {
    pub hybrid_align_distance: f32,
    pub max_alignment_rate: f32,
    pub deceleration_distance: f32,
}

#[derive(Serialize, Deserialize, Message)]
pub enum LedCommand {
    SetParam { r: u8, g: u8, b: u8 },
    Stop,
}

#[derive(Debug, Clone, Serialize, Deserialize, Message)]
#[serde(deny_unknown_fields)]
pub struct Parameters {
    pub walking: WalkingParameters,
    pub move_robot_message_interval: std::time::Duration,
    pub kicking: types::parameters::BoosterKickingParameters,
    pub rotate_head_message_interval: std::time::Duration,
    pub sdk_request_timeout: std::time::Duration,
    pub mode_poll_interval: std::time::Duration,
    pub mode_retry_interval: std::time::Duration,
    pub remote_stop_toggle: bool,
}

#[derive(Clone)]
struct EffectInputs {
    motion_command: Option<SequencedMotionCommand>,
    head_joints: Option<HeadJoints<f32>>,
    emergency_damping: bool,
    now: Time,
    parameters: Parameters,
}

struct InterfaceState {
    confirmed_mode: Option<RobotMode>,
    robot_mode: SequencedRobotMode,
    desired_mode: Option<control::DesiredMode>,
    last_mode_request: Time,
    last_mode_poll: Option<Time>,
    last_move_robot: Time,
    last_rotate_head: Time,
    last_kick: Time,
    last_visual_kick_attempt: Option<Time>,
    last_motion_command: SequencedMotionCommand,
    visual_kick: control::VisualKickState,
    stand_up_request: StandUpRequestState,
}

impl Default for InterfaceState {
    fn default() -> Self {
        Self::new(Time::zero())
    }
}

impl InterfaceState {
    fn new(now: Time) -> Self {
        Self {
            confirmed_mode: None,
            robot_mode: SequencedRobotMode::default(),
            desired_mode: None,
            last_mode_request: now,
            last_mode_poll: None,
            last_move_robot: now,
            last_rotate_head: now,
            last_kick: now,
            last_visual_kick_attempt: None,
            last_motion_command: SequencedMotionCommand::default(),
            visual_kick: control::VisualKickState::default(),
            stand_up_request: StandUpRequestState::default(),
        }
    }
}

impl InterfaceState {
    fn update_confirmed_mode(
        &mut self,
        confirmed_mode: Option<RobotMode>,
    ) -> Option<SequencedRobotMode> {
        let mode = confirmed_mode?;
        if self.confirmed_mode == Some(mode) && self.robot_mode.mode == mode {
            return None;
        }

        self.confirmed_mode = Some(mode);

        self.robot_mode = SequencedRobotMode {
            mode,
            sequence_number: self.robot_mode.sequence_number + 1,
        };
        Some(self.robot_mode)
    }

    fn should_poll_mode(&self, now: Time, poll_interval: std::time::Duration) -> bool {
        self.last_mode_poll
            .is_none_or(|last_mode_poll| now.duration_since(last_mode_poll) >= poll_interval)
    }

    fn update_motion_command(
        &mut self,
        motion_command: Option<SequencedMotionCommand>,
    ) -> Option<&MotionCommand> {
        if let Some(motion_command) = motion_command
            && self.confirmed_mode.is_some()
            && motion_command.robot_mode_sequence_number == self.robot_mode.sequence_number
        {
            self.last_motion_command = motion_command;
        }

        if self.confirmed_mode.is_none()
            || self.last_motion_command.robot_mode_sequence_number
                != self.robot_mode.sequence_number
        {
            return None;
        }

        Some(&self.last_motion_command.motion_command)
    }
}

#[derive(Debug, Default)]
struct StandUpRequestState {
    was_stand_up: bool,
    pending: bool,
    last_attempt: Option<Time>,
}

impl StandUpRequestState {
    fn update_command(&mut self, command: &MotionCommand) {
        let is_stand_up = matches!(command, MotionCommand::StandUp);
        if is_stand_up && !self.was_stand_up {
            self.pending = true;
            self.last_attempt = None;
        } else if !is_stand_up {
            self.pending = false;
            self.last_attempt = None;
        }
        self.was_stand_up = is_stand_up;
    }

    #[cfg(test)]
    fn is_pending(&self) -> bool {
        self.pending
    }

    fn should_request(
        &self,
        confirmed_mode: Option<RobotMode>,
        now: Time,
        retry_interval: std::time::Duration,
        allow_stand_up: bool,
    ) -> bool {
        self.pending
            && allow_stand_up
            && matches!(
                confirmed_mode,
                Some(RobotMode::Prepare | RobotMode::Damping)
            )
            && self
                .last_attempt
                .is_none_or(|last_attempt| now.duration_since(last_attempt) >= retry_interval)
    }

    fn record_attempt(&mut self, now: Time) {
        self.last_attempt = Some(now);
    }

    fn record_success(&mut self) {
        self.pending = false;
        self.last_attempt = None;
    }
}

pub fn run_boxed(ctx: Arc<Context>) -> Pin<Box<dyn Future<Output = Result<()>> + Send>> {
    Box::pin(run(ctx))
}

async fn run(ctx: Arc<Context>) -> Result<()> {
    let node = ctx.create_node("booster_interface").build().await?;
    let parameters = node
        .bind_parameter_as::<Parameters>("booster_interface")
        .wrap_err("failed to bind booster_interface parameters")?;
    let light_control_client =
        Arc::new(LightControlClient::new().wrap_err("failed to create LightControlClient")?);
    let booster_client = BoosterClient::new().wrap_err("failed to create BoosterClient")?;
    let kick_ball_publisher = kick_transport::KickBallPublisher::new(ctx.session())
        .await
        .wrap_err("failed to create kick ball publisher")?;

    let motion_command_sub = node
        .subscriber::<SequencedMotionCommand>("behavior/motion_command")?
        .build()
        .await?;
    let head_joints_sub = node
        .subscriber::<HeadJoints<f32>>("head_joints_command")?
        .build()
        .await?;
    let led_command_sub = node
        .subscriber::<LedCommand>("commands/led_command")?
        .build()
        .await?;
    let buttons_sub = node
        .subscriber::<Buttons<Option<ButtonPressType>>>("buttons")?
        .build()
        .await?;
    let robot_mode_pub = node
        .publisher::<SequencedRobotMode>("robot_mode")?
        .qos(QosProfile {
            durability: QosDurability::TransientLocal,
            ..Default::default()
        })
        .build()
        .await?;

    let clock = node.clock().clone();
    let initial_parameters = parameters.snapshot().typed().clone();
    let (effect_inputs_tx, effect_inputs_rx) = watch::channel(EffectInputs {
        motion_command: None,
        head_joints: None,
        emergency_damping: false,
        now: Time::zero(),
        parameters: initial_parameters,
    });
    robot_mode_pub
        .publish(&SequencedRobotMode::default())
        .await
        .wrap_err("failed to publish initial robot_mode")?;
    tokio::spawn(run_effect_worker(
        effect_inputs_rx,
        booster_client,
        kick_ball_publisher,
        robot_mode_pub,
    ));

    let mut latest_motion_command: Option<SequencedMotionCommand> = None;
    let mut local_stop_toggle = false;
    let mut tick = tokio::time::interval(std::time::Duration::from_millis(10));
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    let mut latest_head_joints: Option<HeadJoints<f32>> = None;

    loop {
        let parameters_snapshot = parameters.snapshot();
        let parameters = parameters_snapshot.typed();

        tokio::select! {
            motion_command = motion_command_sub.recv() => {
                latest_motion_command = Some(motion_command?);
            }
            head_joints = head_joints_sub.recv() => {
                latest_head_joints = Some(head_joints?);
            }
            led_command = led_command_sub.recv() => {
                let light_control_client = light_control_client.clone();
                tokio::spawn(handle_led_command(light_control_client, led_command?));
            }
            buttons = buttons_sub.recv() => {
                let buttons = buttons?;
                local_stop_toggle = update_local_stop_toggle(
                    local_stop_toggle,
                    parameters.remote_stop_toggle,
                    &buttons,
                );
            }
            _ = tick.tick() => {
                let now = clock.now();
                let emergency_damping = local_stop_toggle != parameters.remote_stop_toggle;
                effect_inputs_tx.send_replace(EffectInputs {
                    motion_command: latest_motion_command.clone(),
                    head_joints: latest_head_joints,
                    emergency_damping,
                    now,
                    parameters: parameters.clone(),
                });
            }
        }
    }
}

async fn run_effect_worker(
    mut effect_inputs_rx: watch::Receiver<EffectInputs>,
    booster_client: BoosterClient,
    kick_ball_publisher: kick_transport::KickBallPublisher,
    robot_mode_pub: Publisher<SequencedRobotMode>,
) {
    let mut state = InterfaceState::default();

    loop {
        if effect_inputs_rx.changed().await.is_err() {
            break;
        }
        let inputs = effect_inputs_rx.borrow_and_update().clone();

        drive_booster_effects(
            &mut state,
            &booster_client,
            &kick_ball_publisher,
            &robot_mode_pub,
            inputs.head_joints,
            inputs.emergency_damping,
            inputs.motion_command,
            inputs.now,
            &inputs.parameters,
        )
        .await;
    }
}

async fn handle_led_command(
    light_control_client: Arc<LightControlClient>,
    led_command: LedCommand,
) -> Result<()> {
    match led_command {
        LedCommand::SetParam { r, g, b } => {
            if let Err(err) = light_control_client.set_led_light_color(r, g, b).await {
                log::error!("failed to set leds: {err}");
            }
        }
        LedCommand::Stop => {
            if let Err(err) = light_control_client.stop_led_light_control().await {
                log::error!("failed to stop led control: {err}");
            }
        }
    };

    Ok(())
}

fn sdk_mode_for(desired_mode: control::DesiredMode) -> RobotMode {
    match desired_mode {
        control::DesiredMode::Damping => RobotMode::Damping,
        control::DesiredMode::Walking => RobotMode::Walking,
    }
}

fn button_requests_local_stop_toggle(buttons: &Buttons<Option<ButtonPressType>>) -> bool {
    matches!(buttons.f1, Some(ButtonPressType::Short))
}

fn button_clears_local_stop_toggle(buttons: &Buttons<Option<ButtonPressType>>) -> bool {
    buttons.stand.is_some()
}

fn update_local_stop_toggle(
    local_stop_toggle: bool,
    remote_stop_toggle: bool,
    buttons: &Buttons<Option<ButtonPressType>>,
) -> bool {
    if button_requests_local_stop_toggle(buttons) {
        !local_stop_toggle
    } else if button_clears_local_stop_toggle(buttons) {
        remote_stop_toggle
    } else {
        local_stop_toggle
    }
}

fn visual_kick_transition_for(
    state: control::VisualKickState,
    should_be_active: bool,
) -> control::VisualKickTransition {
    match (state.is_active(), should_be_active) {
        (false, true) => control::VisualKickTransition::Start,
        (true, false) => control::VisualKickTransition::Stop,
        _ => control::VisualKickTransition::None,
    }
}

fn visual_kick_retry_due(
    last_attempt: Option<Time>,
    now: Time,
    retry_interval: std::time::Duration,
) -> bool {
    last_attempt.is_none_or(|last_attempt| now.duration_since(last_attempt) >= retry_interval)
}

async fn await_sdk_call<T, E>(
    future: impl Future<Output = std::result::Result<T, E>>,
    timeout: Duration,
    operation: impl Into<String>,
) -> Option<T>
where
    E: Display,
{
    let operation = operation.into();
    match tokio::time::timeout(timeout, future).await {
        Ok(Ok(result)) => Some(result),
        Ok(Err(error)) => {
            log::error!("failed to {operation}: {error}");
            None
        }
        Err(_) => {
            log::error!("timed out while trying to {operation} after {timeout:?}");
            None
        }
    }
}

async fn poll_mode(client: &BoosterClient, timeout: Duration) -> Option<RobotMode> {
    await_sdk_call(client.get_mode(), timeout, "poll booster mode")
        .await
        .and_then(|mode| mode.mode_enum())
}

async fn request_mode(
    client: &BoosterClient,
    desired_mode: control::DesiredMode,
    timeout: Duration,
) {
    let mode = sdk_mode_for(desired_mode);
    let _ = await_sdk_call(
        client.change_mode(mode),
        timeout,
        format!("request booster mode {mode:?}"),
    )
    .await;
}

async fn drive_booster_effects(
    state: &mut InterfaceState,
    booster_client: &BoosterClient,
    kick_ball_publisher: &kick_transport::KickBallPublisher,
    robot_mode_pub: &Publisher<SequencedRobotMode>,
    latest_head_joints: Option<HeadJoints<f32>>,
    emergency_damping: bool,
    motion_command: Option<SequencedMotionCommand>,
    now: Time,
    parameters: &Parameters,
) {
    if state.should_poll_mode(now, parameters.mode_poll_interval) {
        let confirmed_mode = poll_mode(booster_client, parameters.sdk_request_timeout).await;
        state.last_mode_poll = Some(now);
        if let Some(robot_mode) = state.update_confirmed_mode(confirmed_mode)
            && let Err(error) = robot_mode_pub.publish(&robot_mode).await
        {
            log::error!("failed to publish robot_mode: {error}");
        }
    }

    let accepted_motion_command = state.update_motion_command(motion_command).cloned();
    let Some(desired_mode) = control::desired_mode_for(&accepted_motion_command, emergency_damping)
    else {
        return;
    };
    let confirmed_desired_mode = state.confirmed_mode == Some(sdk_mode_for(desired_mode));
    if !confirmed_desired_mode
        && (state.desired_mode != Some(desired_mode)
            || now.duration_since(state.last_mode_request) >= parameters.mode_retry_interval)
    {
        request_mode(booster_client, desired_mode, parameters.sdk_request_timeout).await;
        state.desired_mode = Some(desired_mode);
        state.last_mode_request = now;
    }

    let walking_allowed =
        control::confirmed_mode_allows_walking(state.confirmed_mode) && !emergency_damping;

    if let Some(motion_command) = accepted_motion_command.as_ref() {
        state.stand_up_request.update_command(motion_command);
    }
    if state.stand_up_request.should_request(
        state.confirmed_mode,
        now,
        parameters.mode_retry_interval,
        !emergency_damping,
    ) {
        match await_sdk_call(
            booster_client.get_up(),
            parameters.sdk_request_timeout,
            "request get_up",
        )
        .await
        {
            Some(()) => {
                state.stand_up_request.record_success();
            }
            None => {
                state.stand_up_request.record_attempt(now);
            }
        }
    }

    if !walking_allowed {
        let transition = visual_kick_transition_for(state.visual_kick, false);
        if transition == control::VisualKickTransition::Stop
            && visual_kick_retry_due(
                state.last_visual_kick_attempt,
                now,
                parameters.mode_retry_interval,
            )
        {
            match await_sdk_call(
                booster_client.visual_kick(false),
                parameters.sdk_request_timeout,
                "stop visual kick",
            )
            .await
            {
                Some(()) => {
                    state.visual_kick.update(false);
                    state.last_visual_kick_attempt = None;
                }
                None => {
                    state.last_visual_kick_attempt = Some(now);
                }
            }
        } else if transition == control::VisualKickTransition::None {
            state.last_visual_kick_attempt = None;
        }
        return;
    }

    let Some(current_motion_command) = accepted_motion_command.as_ref() else {
        return;
    };

    if now.duration_since(state.last_move_robot) >= parameters.move_robot_message_interval {
        let step = control::step_from_motion_command(current_motion_command, &parameters.walking);
        let _ = await_sdk_call(
            booster_client.move_robot(step.forward, step.left, step.turn),
            parameters.sdk_request_timeout,
            "send move_robot",
        )
        .await;
        state.last_move_robot = now;
    }

    if now.duration_since(state.last_rotate_head) >= parameters.rotate_head_message_interval
        && let Some(head_joints) = latest_head_joints
    {
        let _ = await_sdk_call(
            booster_client.rotate_head(head_joints.pitch, head_joints.yaw),
            parameters.sdk_request_timeout,
            "rotate head",
        )
        .await;
        state.last_rotate_head = now;
    }

    let should_visual_kick = matches!(current_motion_command, MotionCommand::VisualKick { .. });
    match visual_kick_transition_for(state.visual_kick, should_visual_kick) {
        control::VisualKickTransition::Start
            if visual_kick_retry_due(
                state.last_visual_kick_attempt,
                now,
                parameters.mode_retry_interval,
            ) =>
        {
            match await_sdk_call(
                booster_client.visual_kick(true),
                parameters.sdk_request_timeout,
                "start visual kick",
            )
            .await
            {
                Some(()) => {
                    state.visual_kick.update(true);
                    state.last_visual_kick_attempt = None;
                }
                None => {
                    state.last_visual_kick_attempt = Some(now);
                }
            }
        }
        control::VisualKickTransition::Stop
            if visual_kick_retry_due(
                state.last_visual_kick_attempt,
                now,
                parameters.mode_retry_interval,
            ) =>
        {
            match await_sdk_call(
                booster_client.visual_kick(false),
                parameters.sdk_request_timeout,
                "stop visual kick",
            )
            .await
            {
                Some(()) => {
                    state.visual_kick.update(false);
                    state.last_visual_kick_attempt = None;
                }
                None => {
                    state.last_visual_kick_attempt = Some(now);
                }
            }
        }
        control::VisualKickTransition::Start | control::VisualKickTransition::Stop => {}
        control::VisualKickTransition::None => {
            state.last_visual_kick_attempt = None;
        }
    }

    if should_visual_kick
        && now.duration_since(state.last_kick) >= parameters.kicking.kick_message_interval
        && let Some(kick) = control::kick_from_motion_command(
            current_motion_command,
            now.to_wallclock(),
            &parameters.kicking,
        )
    {
        let _ = await_sdk_call(
            kick_ball_publisher.publish(&kick),
            parameters.sdk_request_timeout,
            "publish visual kick command",
        )
        .await;
        state.last_kick = now;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ros_z::time::Time;
    use std::time::Duration;
    use types::{
        motion_command::SequencedMotionCommand,
        robot_mode::{RobotMode, SequencedRobotMode},
    };

    #[test]
    fn interface_maps_desired_modes_to_sdk_modes() {
        assert_eq!(
            sdk_mode_for(control::DesiredMode::Damping),
            RobotMode::Damping
        );
        assert_eq!(
            sdk_mode_for(control::DesiredMode::Walking),
            RobotMode::Walking
        );
    }

    #[test]
    fn interface_detects_short_f1_local_stop_requests() {
        assert!(!button_requests_local_stop_toggle(&Buttons {
            f1: None,
            stand: None,
            walking: None,
        }));
        assert!(button_requests_local_stop_toggle(&Buttons {
            f1: Some(ButtonPressType::Short),
            stand: None,
            walking: None,
        }));
        assert!(!button_requests_local_stop_toggle(&Buttons {
            f1: None,
            stand: Some(ButtonPressType::Short),
            walking: None,
        }));
        assert!(!button_requests_local_stop_toggle(&Buttons {
            f1: Some(ButtonPressType::Long),
            stand: None,
            walking: None,
        }));
        assert!(!button_requests_local_stop_toggle(&Buttons {
            f1: None,
            stand: None,
            walking: Some(ButtonPressType::Short),
        }));
    }

    #[test]
    fn stand_button_clears_f1_local_stop_latch() {
        assert!(update_local_stop_toggle(
            false,
            false,
            &Buttons {
                f1: Some(ButtonPressType::Short),
                stand: None,
                walking: None,
            },
        ));
        assert!(!update_local_stop_toggle(
            true,
            false,
            &Buttons {
                f1: None,
                stand: Some(ButtonPressType::Long),
                walking: None,
            },
        ));
        assert!(!update_local_stop_toggle(
            true,
            false,
            &Buttons {
                f1: None,
                stand: Some(ButtonPressType::Short),
                walking: None,
            },
        ));
    }

    #[test]
    fn stand_button_clears_to_remote_stop_toggle_value() {
        assert!(update_local_stop_toggle(
            false,
            true,
            &Buttons {
                f1: None,
                stand: Some(ButtonPressType::Long),
                walking: None,
            },
        ));
    }

    #[test]
    fn robot_mode_sequence_increments_when_confirmed_mode_changes() {
        let mut state = InterfaceState::default();

        assert_eq!(
            state.update_confirmed_mode(Some(RobotMode::Damping)),
            Some(SequencedRobotMode {
                mode: RobotMode::Damping,
                sequence_number: 1,
            })
        );
        assert_eq!(state.update_confirmed_mode(Some(RobotMode::Damping)), None);
        assert_eq!(
            state.update_confirmed_mode(Some(RobotMode::Prepare)),
            Some(SequencedRobotMode {
                mode: RobotMode::Prepare,
                sequence_number: 2,
            })
        );
    }

    #[test]
    fn interface_ignores_motion_commands_from_stale_robot_mode_sequence() {
        let mut state = InterfaceState::default();
        state.update_confirmed_mode(Some(RobotMode::Prepare));

        assert_eq!(
            state.update_motion_command(Some(SequencedMotionCommand {
                motion_command: MotionCommand::Damping,
                robot_mode_sequence_number: 0,
            })),
            None
        );

        assert_eq!(
            state.update_motion_command(Some(SequencedMotionCommand {
                motion_command: MotionCommand::Prepare,
                robot_mode_sequence_number: 1,
            })),
            Some(&MotionCommand::Prepare)
        );
        assert_eq!(
            state.update_motion_command(Some(SequencedMotionCommand {
                motion_command: MotionCommand::Damping,
                robot_mode_sequence_number: 0,
            })),
            Some(&MotionCommand::Prepare)
        );

        state.update_confirmed_mode(Some(RobotMode::Walking));

        assert_eq!(state.update_motion_command(None), None);
    }

    #[test]
    fn interface_rejects_motion_commands_before_confirmed_robot_mode() {
        let mut state = InterfaceState::default();

        assert_eq!(state.update_motion_command(None), None);
        assert_eq!(
            state.update_motion_command(Some(SequencedMotionCommand {
                motion_command: MotionCommand::Prepare,
                robot_mode_sequence_number: 0,
            })),
            None
        );
        assert_eq!(state.update_motion_command(None), None);
    }

    #[test]
    fn interface_polls_robot_mode_on_first_tick() {
        let poll_interval = Duration::from_millis(100);
        let now = Time::from_nanos(1_000_000_000);
        let mut state = InterfaceState::new(now);

        assert!(state.should_poll_mode(now, poll_interval));

        state.last_mode_poll = Some(now);

        assert!(!state.should_poll_mode(now + Duration::from_millis(99), poll_interval,));
        assert!(state.should_poll_mode(now + poll_interval, poll_interval));
    }

    #[test]
    fn interface_keeps_last_confirmed_mode_when_poll_fails() {
        let mut state = InterfaceState::default();

        state.update_confirmed_mode(Some(RobotMode::Prepare));

        assert_eq!(state.update_confirmed_mode(None), None);
        assert_eq!(state.confirmed_mode, Some(RobotMode::Prepare));
        assert_eq!(
            state.robot_mode,
            SequencedRobotMode {
                mode: RobotMode::Prepare,
                sequence_number: 1,
            }
        );
    }

    #[test]
    fn stand_up_retry_state_keeps_request_pending_until_success() {
        let retry_interval = Duration::from_millis(100);
        let now = Time::from_nanos(1_000_000_000);
        let mut state = StandUpRequestState::default();

        state.update_command(&MotionCommand::StandUp);

        assert!(state.is_pending());
        assert!(!state.should_request(Some(RobotMode::Walking), now, retry_interval, true,));
        assert!(state.should_request(Some(RobotMode::Prepare), now, retry_interval, true,));

        state.record_attempt(now);

        assert!(state.is_pending());
        assert!(!state.should_request(
            Some(RobotMode::Prepare),
            now + Duration::from_millis(99),
            retry_interval,
            true,
        ));
        assert!(state.should_request(
            Some(RobotMode::Damping),
            now + retry_interval,
            retry_interval,
            true,
        ));

        state.record_success();
        state.update_command(&MotionCommand::StandUp);

        assert!(!state.is_pending());

        state.update_command(&MotionCommand::Prepare);
        state.update_command(&MotionCommand::StandUp);

        assert!(state.is_pending());
    }

    #[test]
    fn stand_up_retry_state_is_suppressed_during_emergency_damping() {
        let retry_interval = Duration::from_millis(100);
        let now = Time::from_nanos(1_000_000_000);
        let mut state = StandUpRequestState::default();

        state.update_command(&MotionCommand::StandUp);

        assert!(!state.should_request(Some(RobotMode::Damping), now, retry_interval, false,));
        assert!(state.is_pending());
        assert!(state.should_request(Some(RobotMode::Damping), now, retry_interval, true,));
    }

    #[test]
    fn visual_kick_retry_state_does_not_change_before_success() {
        let mut state = control::VisualKickState::default();

        assert_eq!(
            visual_kick_transition_for(state, true),
            control::VisualKickTransition::Start
        );
        assert!(!state.is_active());

        state.update(true);
        assert_eq!(
            visual_kick_transition_for(state, true),
            control::VisualKickTransition::None
        );
        assert_eq!(
            visual_kick_transition_for(state, false),
            control::VisualKickTransition::Stop
        );
        assert!(state.is_active());

        state.update(false);
        assert_eq!(
            visual_kick_transition_for(state, false),
            control::VisualKickTransition::None
        );
    }

    #[test]
    fn visual_kick_transition_retry_waits_for_retry_interval() {
        let retry_interval = Duration::from_millis(100);
        let now = Time::from_nanos(1_000_000_000);

        assert!(visual_kick_retry_due(None, now, retry_interval));
        assert!(!visual_kick_retry_due(
            Some(now),
            now + Duration::from_millis(99),
            retry_interval,
        ));
        assert!(visual_kick_retry_due(
            Some(now),
            now + retry_interval,
            retry_interval,
        ));
    }

    #[tokio::test]
    async fn sdk_call_returns_none_on_timeout() {
        let result = await_sdk_call(
            std::future::pending::<std::result::Result<(), std::convert::Infallible>>(),
            Duration::from_millis(1),
            "pending test operation",
        )
        .await;

        assert!(result.is_none());
    }
}
