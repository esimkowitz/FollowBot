//! System info (device `0x11`) and version reporting.

use crate::error::Result;
use crate::ids::DeviceId;

pub const DEVICE: DeviceId = DeviceId::SystemInfo;

pub mod cid {
    pub const GET_MAIN_APPLICATION_VERSION: u8 = 0x00;
    pub const GET_BOOTLOADER_VERSION: u8 = 0x01;
    pub const GET_BOARD_REVISION: u8 = 0x03;
    pub const GET_MAC_ADDRESS: u8 = 0x06;
    pub const GET_STATS_ID: u8 = 0x13;
    pub const GET_PROCESSOR_NAME: u8 = 0x1F;
    pub const GET_SKU: u8 = 0x38;
    pub const GET_CORE_UP_TIME_IN_MILLISECONDS: u8 = 0x39;
}

/// A firmware version triple.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Version {
    pub major: u16,
    pub minor: u16,
    pub revision: u16,
}

impl std::fmt::Display for Version {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}.{}.{}", self.major, self.minor, self.revision)
    }
}

pub fn get_main_application_version() -> Vec<u8> {
    Vec::new()
}

pub fn get_bootloader_version() -> Vec<u8> {
    Vec::new()
}

pub fn get_board_revision() -> Vec<u8> {
    Vec::new()
}

pub fn get_mac_address() -> Vec<u8> {
    Vec::new()
}

pub fn get_processor_name() -> Vec<u8> {
    Vec::new()
}

pub fn get_sku() -> Vec<u8> {
    Vec::new()
}

pub fn get_core_up_time_in_milliseconds() -> Vec<u8> {
    Vec::new()
}

/// Parse three big-endian `u16`s.
pub fn parse_version(payload: &[u8]) -> Result<Version> {
    Ok(Version {
        major: u16::from_be_bytes(crate::devices::be_bytes(payload, 0)?),
        minor: u16::from_be_bytes(crate::devices::be_bytes(payload, 2)?),
        revision: u16::from_be_bytes(crate::devices::be_bytes(payload, 4)?),
    })
}

/// Parse a NUL-padded ASCII string response.
pub fn parse_string(payload: &[u8]) -> String {
    let end = payload
        .iter()
        .position(|&b| b == 0)
        .unwrap_or(payload.len());
    String::from_utf8_lossy(&payload[..end]).into_owned()
}

/// Parse a big-endian `u64` uptime.
pub fn parse_uptime_ms(payload: &[u8]) -> Result<u64> {
    crate::devices::be_bytes(payload, 0).map(u64::from_be_bytes)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn version_parses_three_big_endian_u16s() {
        let payload = [0x00, 0x03, 0x00, 0x01, 0x00, 0x2A];
        let version = parse_version(&payload).unwrap();
        assert_eq!(
            version,
            Version {
                major: 3,
                minor: 1,
                revision: 42
            }
        );
        assert_eq!(version.to_string(), "3.1.42");
        assert!(parse_version(&payload[..5]).is_err(), "truncated");
    }

    #[test]
    fn strings_stop_at_the_nul_terminator() {
        assert_eq!(parse_string(b"ST\0\0\0"), "ST");
        assert_eq!(parse_string(b"Nordic"), "Nordic");
        assert_eq!(parse_string(b""), "");
    }

    #[test]
    fn uptime_parses_from_big_endian_u64() {
        assert_eq!(
            parse_uptime_ms(&[0, 0, 0, 0, 0, 0, 0x03, 0xE8]).unwrap(),
            1000
        );
        assert!(parse_uptime_ms(&[0, 0]).is_err());
    }
}
