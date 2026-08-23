//! Error types for the RVR protocol layer.

use crate::ids::ErrorCode;

pub type Result<T> = std::result::Result<T, Error>;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("packet body too short to decode ({len} bytes)")]
    Truncated { len: usize },

    #[error("checksum mismatch: expected {expected:#04X}, received {received:#04X}")]
    Checksum { expected: u8, received: u8 },

    #[error("invalid escape sequence: ESC followed by {0:#04X}")]
    BadEscape(u8),

    #[error("frame ended on an escape byte")]
    DanglingEscape,

    #[error("extended flags (0x80) are not supported by RVR firmware")]
    ExtendedFlagsUnsupported,

    /// The robot answered, but reported a failure.
    #[error("robot returned error: {0:?}")]
    Robot(ErrorCode),

    #[error("response payload too short: expected {expected} bytes, got {actual}")]
    ShortPayload { expected: usize, actual: usize },

    #[error("timed out waiting for a response to {device_id:#04X}/{command_id:#04X}")]
    Timeout { device_id: u8, command_id: u8 },

    #[error("the RVR connection is closed")]
    Disconnected,

    #[error("streaming slot is full (max {max} services)")]
    SlotFull { max: usize },

    /// Underlying I/O failure. Serial-port errors arrive here too, funnelled
    /// through `io::Error` so this enum has the same shape with or without the
    /// `serial` feature.
    #[error(transparent)]
    Io(#[from] std::io::Error),
}
