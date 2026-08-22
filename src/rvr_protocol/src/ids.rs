//! Device, target and error identifiers.

/// Processor addresses on the RVR's internal bus.
///
/// The wire encoding is `(port << 4) | node`; both processors sit on port 0, so
/// the enum values double as the address byte.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum Target {
    /// Nordic: power, IO/LEDs, ambient light, color detection.
    Primary = 0x01,
    /// ST: drive, IMU, locator, encoders.
    Secondary = 0x02,
}

impl Target {
    pub fn from_byte(byte: u8) -> Option<Self> {
        match byte & 0x0F {
            0x01 => Some(Self::Primary),
            0x02 => Some(Self::Secondary),
            _ => None,
        }
    }

    pub fn port(self) -> u8 {
        (self as u8) >> 4
    }

    pub fn node(self) -> u8 {
        (self as u8) & 0x0F
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DeviceId {
    ApiAndShell = 0x10,
    SystemInfo = 0x11,
    Power = 0x13,
    Drive = 0x16,
    Sensor = 0x18,
    Connection = 0x19,
    Io = 0x1A,
}

/// Error codes returned in a response's `err` byte.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ErrorCode {
    Success = 0x00,
    BadDeviceId = 0x01,
    BadCommandId = 0x02,
    NotYetImplemented = 0x03,
    Restricted = 0x04,
    BadDataLength = 0x05,
    Failed = 0x06,
    BadDataValue = 0x07,
    Busy = 0x08,
    BadTargetId = 0x09,
    TargetUnavailable = 0x0A,
    /// Anything the firmware returns that predates this enum.
    Unknown = 0xFF,
}

impl ErrorCode {
    pub fn from_byte(byte: u8) -> Self {
        match byte {
            0x00 => Self::Success,
            0x01 => Self::BadDeviceId,
            0x02 => Self::BadCommandId,
            0x03 => Self::NotYetImplemented,
            0x04 => Self::Restricted,
            0x05 => Self::BadDataLength,
            0x06 => Self::Failed,
            0x07 => Self::BadDataValue,
            0x08 => Self::Busy,
            0x09 => Self::BadTargetId,
            0x0A => Self::TargetUnavailable,
            _ => Self::Unknown,
        }
    }

    pub fn is_success(self) -> bool {
        matches!(self, Self::Success)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn targets_encode_as_port_and_node_nibbles() {
        assert_eq!(Target::Primary as u8, 0x01);
        assert_eq!(Target::Secondary as u8, 0x02);
        assert_eq!(Target::Secondary.port(), 0);
        assert_eq!(Target::Secondary.node(), 2);
    }

    #[test]
    fn target_round_trips_through_a_source_byte() {
        assert_eq!(Target::from_byte(0x01), Some(Target::Primary));
        assert_eq!(Target::from_byte(0x02), Some(Target::Secondary));
        assert_eq!(Target::from_byte(0x07), None);
    }

    #[test]
    fn unrecognized_error_codes_do_not_panic() {
        assert_eq!(ErrorCode::from_byte(0x05), ErrorCode::BadDataLength);
        assert_eq!(ErrorCode::from_byte(0x7E), ErrorCode::Unknown);
        assert!(ErrorCode::from_byte(0x00).is_success());
    }
}
