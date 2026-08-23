//! API and shell (device `0x10`) — echo, used as a link check and keepalive.

use crate::ids::DeviceId;

pub const DEVICE: DeviceId = DeviceId::ApiAndShell;

pub mod cid {
    pub const ECHO: u8 = 0x00;
    pub const GENERATE_API_ERROR: u8 = 0x09;
}

/// Echo `data` back. An empty payload works as a cheap ping.
pub fn echo(data: &[u8]) -> Vec<u8> {
    data.to_vec()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn echo_passes_its_payload_through() {
        assert_eq!(echo(&[1, 2, 3]), vec![1, 2, 3]);
        assert!(echo(&[]).is_empty(), "empty echo is a valid ping");
    }
}
