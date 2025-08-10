use anyhow::Result;
use rclrs;
use std::time::{Duration, Instant};

// use geometry_msgs::msg::{PoseStamped, PoseArray};          // enable when message crates exist
// use nav2_msgs::action::NavigateToPose;                      // enable when action crate exists
// use rclrs_action::Client as ActionClient;                   // if using rclrs_action

struct Params {
    standoff: f32,
    reissue_period: f32,
    frame_id: String,
}

fn main() -> Result<()> {
    let ctx = rclrs::Context::new([])?;
    let node = rclrs::create_node(&ctx, "tracker_fuser_rust")?;

    let params = Params { standoff: 1.2, reissue_period: 1.5, frame_id: "base_link".to_string() };

    // TODO: Uncomment once messages are generated in workspace
    // let target_pub = node.create_publisher::<PoseStamped>("/target_pose", rclrs::QOS_PROFILE_DEFAULT)?;
    // let _legs_sub = node.create_subscription::<PoseArray>("/legs", rclrs::QOS_PROFILE_DEFAULT, move |msg| {
    //     // Pick closest in-front leg and compute target pose standoff meters away
    //     // Publish PoseStamped to /target_pose
    // })?;

    // TODO: Set up NavigateToPose action client in Rust once rclrs_action + nav2_msgs are wired.
    // let action_client: ActionClient<NavigateToPose> = ActionClient::new(&node, "navigate_to_pose");
    let _timer = node.create_wall_timer(Duration::from_millis(200), || {
        // Periodically (re)send a NavigateToPose goal using the last target pose.
        // Keep this as a skeleton until action bindings are enabled.
        rclrs::log_info!("tracker_fuser_rust", "alive (wire PoseArray + action client when rust messages ready)");
    })?;

    rclrs::spin(&node)?;
    Ok(())
}
