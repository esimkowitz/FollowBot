use anyhow::Result;
use std::time::Duration;
use rclrs;
fn main() -> Result<()> {
    let context = rclrs::Context::new([])?;
    let node = rclrs::create_node(&context, "rvr_ros")?;
    let _timer = node.create_wall_timer(Duration::from_secs(1), || {
        rclrs::log_info!("rvr_ros", "alive (wire cmd_vel + serial when ready)");
    })?;
    rclrs::spin(&node)?;
    Ok(())
}
