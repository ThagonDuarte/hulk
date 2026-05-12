use std::{
    net::{IpAddr, Ipv4Addr, SocketAddr},
    sync::Arc,
    time::Duration,
};

use booster::{ButtonEventMsg, FallDownState, LowState, Odometer, RemoteControllerState};
use booster_sdk::{
    client::{BoosterClient, light_control::LightControlClient},
    types::RobotMode as RobotModeSdk,
};
use color_eyre::{
    Result,
    eyre::{Context as _, eyre},
};
use ros_z::{
    Message, SerdeCdrCodec, Service, ServiceTypeInfo, TypeInfo, context::Context,
    dynamic::SchemaError, node::Node, pubsub::Publisher, service::ServiceServer,
};
use ros2::sensor_msgs::{camera_info::CameraInfo, image::Image};
use serde::{Deserialize, Serialize};
use x5_receiver::receiver::X5Receiver;

use crate::IntoEyreResultExt;

const X5_ADDRESS: SocketAddr = SocketAddr::new(IpAddr::V4(Ipv4Addr::new(192, 168, 127, 10)), 7654);
const ZENOH_LOCALHOST_ENDPOINT: &str = "tcp/127.0.0.1:7447";

const LOW_STATE_ZENOH_TOPIC: &str = "rt/low_state";
const ODOMETER_STATE_ZENOH_TOPIC: &str = "rt/odometer_state";
const FALL_DOWN_ZENOH_TOPIC: &str = "rt/fall_down";
const BUTTON_EVENT_ZENOH_TOPIC: &str = "rt/button_event";
const REMOTE_CONTROLLER_STATE_ZENOH_TOPIC: &str = "rt/remote_controller_state";

const LOW_STATE_ROSZ_TOPIC: &str = "sensors/low_state";
const ODOMETER_STATE_ROSZ_TOPIC: &str = "sensors/odometer";
const FALL_DOWN_ROSZ_TOPIC: &str = "sensors/fall_down_state";
const BUTTON_EVENT_ROSZ_TOPIC: &str = "sensors/button_event_message";
const REMOTE_CONTROLLER_STATE_ROSZ_TOPIC: &str = "sensors/remote_controller_state";

#[derive(Serialize, Deserialize, Message)]
pub enum LedCommand {
    SetParam { r: u8, g: u8, b: u8 },
    Stop,
}

#[derive(Debug, Clone, Serialize, Deserialize, Message)]
pub enum RobotMode {
    /// Unknown mode, typically used for error handling.
    Unknown = -1,

    /// Damping mode, motors are compliant
    Damping = 0,

    /// Prepare mode, standing pose
    Prepare = 1,

    /// Walking mode, active locomotion
    Walking = 2,

    /// Custom mode, user-defined behavior
    Custom = 3,

    /// Soccer mode
    Soccer = 4,
}

impl From<RobotModeSdk> for RobotMode {
    fn from(sdk_mode: RobotModeSdk) -> Self {
        match sdk_mode {
            RobotModeSdk::Unknown => Self::Unknown,
            RobotModeSdk::Damping => Self::Damping,
            RobotModeSdk::Prepare => Self::Prepare,
            RobotModeSdk::Walking => Self::Walking,
            RobotModeSdk::Custom => Self::Custom,
            RobotModeSdk::Soccer => Self::Soccer,
            _ => Self::Unknown,
        }
    }
}

impl From<Option<RobotModeSdk>> for RobotMode {
    fn from(sdk_mode: Option<RobotModeSdk>) -> Self {
        match sdk_mode {
            Some(RobotModeSdk::Unknown) => Self::Unknown,
            Some(RobotModeSdk::Damping) => Self::Damping,
            Some(RobotModeSdk::Prepare) => Self::Prepare,
            Some(RobotModeSdk::Walking) => Self::Walking,
            Some(RobotModeSdk::Custom) => Self::Custom,
            Some(RobotModeSdk::Soccer) => Self::Soccer,
            _ => Self::Unknown,
        }
    }
}

impl From<RobotMode> for RobotModeSdk {
    fn from(mode: RobotMode) -> Self {
        match mode {
            RobotMode::Unknown => Self::Unknown,
            RobotMode::Damping => Self::Damping,
            RobotMode::Prepare => Self::Prepare,
            RobotMode::Walking => Self::Walking,
            RobotMode::Custom => Self::Custom,
            RobotMode::Soccer => Self::Soccer,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, Message)]
struct GetRobotModeRequest {}

#[derive(Debug, Clone, Serialize, Deserialize, Message)]
struct GetRobotModeResponse {
    robot_mode: RobotMode,
}

struct GetRobotMode;

impl ServiceTypeInfo for GetRobotMode {
    fn service_type_info() -> std::prelude::v1::Result<ros_z::prelude::TypeInfo, SchemaError> {
        Ok(TypeInfo::new("hardware_interface::GetRobotMode", None))
    }
}

impl Service for GetRobotMode {
    type Request = GetRobotModeRequest;

    type Response = GetRobotModeResponse;
}

#[derive(Serialize, Deserialize, Message)]
pub enum HighLevelCommand {
    ChangeMode { mode: RobotMode },
    MoveRobot { forward: f32, left: f32, turn: f32 },
    RotateHead { pitch: f32, yaw: f32 },
    RotateHeadWithDirection { pitch: i32, yaw: i32 },
    LieDown,
    GetUp,
    GetUpWithMode { mode: RobotMode },
    EnterWbcGait,
    ExitWbcGait,
    VisualKick { start: bool },
    ResetOdometer,
}

pub async fn run(ctx: Arc<Context>) -> Result<()> {
    let node = Arc::new(
        ctx.create_node("hardware_interface")
            .build()
            .await
            .into_eyre()?,
    );

    let left_image_pub = node
        .publisher::<Image>("left_image")
        .build()
        .await
        .into_eyre()?;
    let right_image_pub = node
        .publisher::<Image>("right_image")
        .build()
        .await
        .into_eyre()?;
    let camera_info_pub = node
        .publisher::<CameraInfo>("camera_info")
        .build()
        .await
        .into_eyre()?;
    tokio::spawn(image_publisher_task(
        node.clone(),
        left_image_pub,
        right_image_pub,
        camera_info_pub,
    ));

    let zenoh_session = Arc::new(
        zenoh::open(localhost_zenoh_config()?)
            .await
            .map_err(|error| eyre!("failed to create Zenoh session: {error}"))?,
    );

    spawn_zenoh_rosz_bridges(&node, zenoh_session);

    let led_command_sub = node
        .subscriber::<LedCommand>("led_command")
        .build()
        .await
        .into_eyre()?;
    let high_level_command_sub = node
        .subscriber::<HighLevelCommand>("high_level_command")
        .build()
        .await
        .into_eyre()?;
    let mut robot_mode_service: ServiceServer<GetRobotMode> = node
        .create_service_server::<GetRobotMode>("robot_mode")
        .build()
        .await
        .into_eyre()?;

    let high_level_interface_client = Arc::new(BoosterClient::new()?);
    let light_control_client = Arc::new(LightControlClient::new()?);

    loop {
        tokio::select! {
            led_command = led_command_sub.recv() => {
                let led_command = led_command.into_eyre()?;

                tokio::spawn({
                    let light_control_client = light_control_client.clone();

                    handle_led_command(light_control_client, led_command)
                });
            },
            high_level_command = high_level_command_sub.recv() => {
                let high_level_command = high_level_command.into_eyre()?;

                tokio::spawn({
                    let high_level_interface_client = high_level_interface_client.clone();

                    handle_high_level_command(high_level_interface_client, high_level_command)
                });
            },
            robot_mode_request = robot_mode_service.take_request_async() => {
                let robot_mode_request = robot_mode_request.into_eyre()?;

                let client = high_level_interface_client.clone();
                tokio::spawn(async move {
                    handle_robot_mode_request(client, robot_mode_request).await;
                });
            }
        }
    }
}

fn spawn_zenoh_rosz_bridges(node: &Arc<Node>, zenoh_session: Arc<zenoh::Session>) {
    tokio::spawn(zenoh_to_rosz_forwarder::<LowState>(
        zenoh_session.clone(),
        node.clone(),
        LOW_STATE_ZENOH_TOPIC,
        LOW_STATE_ROSZ_TOPIC,
    ));
    tokio::spawn(zenoh_to_rosz_forwarder::<Odometer>(
        zenoh_session.clone(),
        node.clone(),
        ODOMETER_STATE_ZENOH_TOPIC,
        ODOMETER_STATE_ROSZ_TOPIC,
    ));
    tokio::spawn(zenoh_to_rosz_forwarder::<FallDownState>(
        zenoh_session.clone(),
        node.clone(),
        FALL_DOWN_ZENOH_TOPIC,
        FALL_DOWN_ROSZ_TOPIC,
    ));
    tokio::spawn(zenoh_to_rosz_forwarder::<ButtonEventMsg>(
        zenoh_session.clone(),
        node.clone(),
        BUTTON_EVENT_ZENOH_TOPIC,
        BUTTON_EVENT_ROSZ_TOPIC,
    ));
    tokio::spawn(zenoh_to_rosz_forwarder::<RemoteControllerState>(
        zenoh_session.clone(),
        node.clone(),
        REMOTE_CONTROLLER_STATE_ZENOH_TOPIC,
        REMOTE_CONTROLLER_STATE_ROSZ_TOPIC,
    ));
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

async fn handle_high_level_command(
    high_level_interface_client: Arc<BoosterClient>,
    high_level_command: HighLevelCommand,
) -> Result<()> {
    match high_level_command {
        HighLevelCommand::ChangeMode { mode } => high_level_interface_client
            .change_mode(mode.into())
            .await
            .into_eyre(),
        HighLevelCommand::MoveRobot {
            forward,
            left,
            turn,
        } => high_level_interface_client
            .move_robot(forward, left, turn)
            .await
            .into_eyre(),
        HighLevelCommand::RotateHead { pitch, yaw } => high_level_interface_client
            .rotate_head(pitch, yaw)
            .await
            .into_eyre(),
        HighLevelCommand::RotateHeadWithDirection { pitch, yaw } => high_level_interface_client
            .rotate_head_with_direction(pitch, yaw)
            .await
            .into_eyre(),
        HighLevelCommand::LieDown => high_level_interface_client.lie_down().await.into_eyre(),
        HighLevelCommand::GetUp => high_level_interface_client.get_up().await.into_eyre(),
        HighLevelCommand::GetUpWithMode { mode } => high_level_interface_client
            .get_up_with_mode(mode.into())
            .await
            .into_eyre(),
        HighLevelCommand::EnterWbcGait => high_level_interface_client
            .enter_wbc_gait()
            .await
            .into_eyre(),
        HighLevelCommand::ExitWbcGait => high_level_interface_client
            .exit_wbc_gait()
            .await
            .into_eyre(),
        HighLevelCommand::VisualKick { start } => high_level_interface_client
            .visual_kick(start)
            .await
            .into_eyre(),
        HighLevelCommand::ResetOdometer => high_level_interface_client
            .reset_odometry()
            .await
            .into_eyre(),
    }
}

async fn handle_robot_mode_request(
    high_level_interface_client: Arc<BoosterClient>,
    robot_mode_request: ros_z::service::ServiceRequest<GetRobotMode>,
) {
    match high_level_interface_client.get_mode().await {
        Ok(mode) => {
            let robot_mode: RobotMode = mode.mode_enum().into();
            let get_robot_mode_response = GetRobotModeResponse { robot_mode };
            if let Err(e) = robot_mode_request
                .reply_async(&get_robot_mode_response)
                .await
            {
                log::error!("failed to reply to robot mode request: {e}");
            }
        }
        Err(e) => {
            log::error!("failed to get robot mode from booster client: {e}");
            let get_robot_mode_response = GetRobotModeResponse {
                robot_mode: RobotMode::Unknown,
            };
            if let Err(e) = robot_mode_request
                .reply_async(&get_robot_mode_response)
                .await
            {
                log::error!("failed to reply to robot mode request after error: {e}");
            }
        }
    }
}

async fn image_publisher_task(
    node: Arc<Node>,
    left_image_pub: Publisher<Image, SerdeCdrCodec<Image>>,
    right_image_pub: Publisher<Image, SerdeCdrCodec<Image>>,
    camera_info_pub: Publisher<CameraInfo, SerdeCdrCodec<CameraInfo>>,
) -> Result<()> {
    let x5_receiver = X5Receiver::new(X5_ADDRESS);
    let left_camera_info = x5_receiver.last_camera_info().await.left_camera_info();
    let mut camera_info_timer = node.clock().timer(Duration::from_secs(1));

    loop {
        tokio::select! {
            left_frame = x5_receiver.next_left_frame() => {
                left_image_pub.publish(&left_frame.into()).await.into_eyre()?;
            }
            right_frame = x5_receiver.next_right_frame() => {
                right_image_pub.publish(&right_frame.into()).await.into_eyre()?;
            }
            _ = camera_info_timer.tick() => {
                camera_info_pub
                    .publish(&left_camera_info)
                    .await
                    .into_eyre()?;
            }
        }
    }
}

fn localhost_zenoh_config() -> Result<zenoh::Config> {
    let mut config = zenoh::Config::default();
    config
        .insert_json5("mode", r#""client""#)
        .map_err(|error| eyre!("failed to set Zenoh mode: {error}"))?;
    config
        .insert_json5(
            "connect/endpoints",
            &format!(r#"["{ZENOH_LOCALHOST_ENDPOINT}"]"#),
        )
        .map_err(|error| eyre!("failed to set Zenoh connect endpoint: {error}"))?;
    Ok(config)
}

async fn zenoh_to_rosz_forwarder<'de, T: Message + Serialize + Deserialize<'de>>(
    zenoh_session: Arc<zenoh::Session>,
    rosz_node: Arc<Node>,
    zenoh_topic: &str,
    rosz_topic: &str,
) -> Result<()> {
    let zenoh_subscriber = zenoh_session
        .declare_subscriber(zenoh_topic)
        .await
        .into_eyre()?;

    let rosz_publisher = rosz_node
        .publisher::<T>(rosz_topic)
        .build()
        .await
        .into_eyre()?;

    loop {
        let zenoh_sample = zenoh_subscriber.recv_async().await.into_eyre()?;
        let deserialized_sample = cdr::deserialize(&zenoh_sample.payload().to_bytes())
            .wrap_err("deserialization failed")?;
        rosz_publisher
            .publish(&deserialized_sample)
            .await
            .into_eyre()?;
    }
}

async fn rosz_to_zenoh_forwarder<'de, T: Message + Serialize + Deserialize<'de>>(
    zenoh_session: Arc<zenoh::Session>,
    rosz_node: Arc<Node>,
    zenoh_topic: &str,
    rosz_topic: &str,
) -> Result<()> {
    let zenoh_subscriber = zenoh_session
        .declare_subscriber(zenoh_topic)
        .await
        .into_eyre()?;

    let rosz_publisher = rosz_node
        .publisher::<T>(rosz_topic)
        .build()
        .await
        .into_eyre()?;

    loop {
        let zenoh_sample = zenoh_subscriber.recv_async().await.into_eyre()?;
        let deserialized_sample = cdr::deserialize(&zenoh_sample.payload().to_bytes())
            .wrap_err("deserialization failed")?;
        rosz_publisher
            .publish(&deserialized_sample)
            .await
            .into_eyre()?;
    }
}
