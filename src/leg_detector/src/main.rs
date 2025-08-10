use anyhow::Result;
use rclrs;
use std::time::Duration;
fn main() -> Result<()> {
    let context = rclrs::Context::new([])?;
    let node = rclrs::create_node(&context, "leg_detector")?;
    let _timer = node.create_wall_timer(Duration::from_secs(1), || {
        rclrs::log_info!("leg_detector", "alive (subscribe /scan; publish /legs when ready)");
    })?;
    rclrs::spin(&node)?;
    Ok(())
}
