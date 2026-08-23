//! Command-line probe for a connected RVR.
//!
//! Useful for confirming the link before bringing ROS into the picture:
//!
//! ```text
//! rvr_probe /dev/ttyAMA0          # wake, read version and battery
//! rvr_probe /dev/ttyAMA0 stream   # also stream odometry for 5 seconds
//! rvr_probe /dev/ttyAMA0 drive    # brief nudge forward, then stop
//! ```

use rvr_protocol::client::Notification;
use rvr_protocol::ids::Target;
use rvr_protocol::streaming::{ServiceId, SlotConfig, StreamSample};
use rvr_protocol::{NotificationChannel, RvrClient, SerialTransport};

use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let port = args.next().unwrap_or_else(|| "/dev/ttyAMA0".to_string());
    let mode = args.next().unwrap_or_else(|| "info".to_string());

    println!("opening {port} at 115200...");
    let transport = SerialTransport::open(&port, 115_200)?;
    let client = RvrClient::new(Box::new(transport))?;

    let (handler, channel) = NotificationChannel::new();
    client.on_notification(handler);

    println!("waking robot...");
    client.wake()?;
    // The Nordic side needs a moment before the ST processor answers.
    std::thread::sleep(Duration::from_millis(500));

    match client.firmware_version(Target::Secondary) {
        Ok(version) => println!("  ST firmware:     {version}"),
        Err(e) => println!("  ST firmware:     unavailable ({e})"),
    }
    match client.firmware_version(Target::Primary) {
        Ok(version) => println!("  Nordic firmware: {version}"),
        Err(e) => println!("  Nordic firmware: unavailable ({e})"),
    }
    match client.battery_percentage() {
        Ok(percent) => println!("  battery:         {percent}%"),
        Err(e) => println!("  battery:         unavailable ({e})"),
    }
    match client.battery_voltage() {
        Ok(volts) => println!("  voltage:         {volts:.2} V"),
        Err(e) => println!("  voltage:         unavailable ({e})"),
    }

    match mode.as_str() {
        "stream" => stream_odometry(&client, &channel)?,
        "drive" => nudge(&client)?,
        _ => {}
    }

    println!("done");
    Ok(())
}

/// Stream locator and velocity for a few seconds, printing each sample.
fn stream_odometry(
    client: &RvrClient,
    channel: &NotificationChannel,
) -> Result<(), Box<dyn std::error::Error>> {
    println!("\nconfiguring odometry stream...");

    // Clear first: a stale slot definition on the robot would produce blobs that
    // no longer match what we expect.
    let _ = client.clear_streaming(Target::Secondary);

    let slot = SlotConfig::new(Target::Secondary, 0x02)
        .with(ServiceId::Locator)?
        .with(ServiceId::Velocity)?;
    client.configure_streaming(&slot)?;
    client.start_streaming(Target::Secondary, 100)?;

    println!("streaming for 5s (push the robot to see values change)...");
    let deadline = Instant::now() + Duration::from_secs(5);
    while Instant::now() < deadline {
        match channel.receiver.recv_timeout(Duration::from_millis(500)) {
            Ok(Notification::Stream { samples, .. }) => {
                for sample in samples {
                    match sample {
                        // Reported in the RVR's own X-right / Y-forward frame;
                        // the ROS node is what converts to REP-103.
                        StreamSample::Locator { x, y } => print!("  locator ({x:+.3}, {y:+.3}) m"),
                        StreamSample::Velocity { x, y } => println!("  vel ({x:+.3}, {y:+.3}) m/s"),
                        other => println!("  {other:?}"),
                    }
                }
            }
            Ok(Notification::StreamInvalid { token, .. }) => {
                println!("  (slot {token} reported an invalid sample)")
            }
            Ok(other) => println!("  {other:?}"),
            Err(_) => println!("  (no data)"),
        }
    }

    client.clear_streaming(Target::Secondary)?;
    Ok(())
}

/// Drive forward briefly, then stop.
fn nudge(client: &RvrClient) -> Result<(), Box<dyn std::error::Error>> {
    println!("\ndriving forward at 0.2 m/s for 1s...");
    let deadline = Instant::now() + Duration::from_secs(1);
    while Instant::now() < deadline {
        // Re-issue faster than the robot's control-system timeout.
        client.drive_tank_si(0.2, 0.2)?;
        std::thread::sleep(Duration::from_millis(50));
    }
    client.drive_stop()?;
    println!("stopped");
    Ok(())
}
