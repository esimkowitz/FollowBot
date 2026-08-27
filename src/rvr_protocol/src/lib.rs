//! Sphero RVR UART protocol.
//!
//! A hardware-independent port of the RVR's serial API: packet framing, the
//! command set, and sensor streaming. Nothing here depends on ROS, so the whole
//! crate builds and tests on a development machine with `cargo test`.
//!
//! ```ignore
//! # use rvr_protocol::{RvrClient, SerialTransport};
//! let transport = SerialTransport::open("/dev/rvr", 115_200)?;
//! let rvr = RvrClient::new(transport);
//! rvr.wake()?;
//! rvr.drive_tank_si(0.5, 0.5)?;
//! # Ok::<(), rvr_protocol::Error>(())
//! ```

pub mod client;
pub mod devices;
pub mod error;
pub mod ids;
pub mod packet;

pub use client::{Notification, NotificationChannel, RvrClient};
pub use error::{Error, Result};
pub use ids::{DeviceId, ErrorCode, Target};
pub use packet::{Packet, PacketReader};

pub mod streaming;
pub mod transport;

pub use streaming::{ServiceId, SlotConfig, StreamNotification, StreamSample};
#[cfg(feature = "serial")]
pub use transport::SerialTransport;
pub use transport::{MockTransport, Transport};
