//! Power commands (device `0x13`), sent to the Nordic processor.

use crate::ids::{DeviceId, Target};

pub const DEVICE: DeviceId = DeviceId::Power;
pub const TARGET: Target = Target::Primary;

pub mod cid {
    pub const SLEEP: u8 = 0x01;
    pub const WAKE: u8 = 0x0D;
    pub const GET_BATTERY_PERCENTAGE: u8 = 0x10;
    pub const GET_BATTERY_VOLTAGE_STATE: u8 = 0x17;
    pub const WILL_SLEEP_NOTIFY: u8 = 0x19;
    pub const DID_SLEEP_NOTIFY: u8 = 0x1A;
    pub const ENABLE_BATTERY_VOLTAGE_STATE_CHANGE_NOTIFY: u8 = 0x1B;
    pub const BATTERY_VOLTAGE_STATE_CHANGE_NOTIFY: u8 = 0x1C;
    pub const GET_BATTERY_VOLTAGE_IN_VOLTS: u8 = 0x25;
    pub const GET_BATTERY_VOLTAGE_STATE_THRESHOLDS: u8 = 0x26;
    pub const GET_CURRENT_SENSE_AMPLIFIER_CURRENT: u8 = 0x27;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum BatteryVoltageState {
    Unknown = 0,
    Ok = 1,
    Low = 2,
    Critical = 3,
}

impl BatteryVoltageState {
    pub fn from_byte(byte: u8) -> Self {
        match byte {
            1 => Self::Ok,
            2 => Self::Low,
            3 => Self::Critical,
            _ => Self::Unknown,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum VoltageReadingType {
    CalibratedAndFiltered = 0,
    CalibratedAndUnfiltered = 1,
    UncalibratedAndUnfiltered = 2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AmplifierId {
    LeftMotor = 0,
    RightMotor = 1,
}

pub fn wake() -> Vec<u8> {
    Vec::new()
}

/// Soft sleep: driving, LEDs and sensors stop until the next [`wake`].
pub fn sleep() -> Vec<u8> {
    Vec::new()
}

pub fn get_battery_percentage() -> Vec<u8> {
    Vec::new()
}

pub fn get_battery_voltage_state() -> Vec<u8> {
    Vec::new()
}

pub fn get_battery_voltage_in_volts(reading_type: VoltageReadingType) -> Vec<u8> {
    vec![reading_type as u8]
}

pub fn enable_battery_voltage_state_change_notify(enabled: bool) -> Vec<u8> {
    vec![enabled as u8]
}

pub fn get_battery_voltage_state_thresholds() -> Vec<u8> {
    Vec::new()
}

pub fn get_current_sense_amplifier_current(amplifier: AmplifierId) -> Vec<u8> {
    vec![amplifier as u8]
}

/// Parse the `u8` percentage response.
pub fn parse_battery_percentage(payload: &[u8]) -> Option<u8> {
    payload.first().copied()
}

/// Parse a big-endian `f32` volts response.
pub fn parse_voltage(payload: &[u8]) -> Option<f32> {
    payload
        .get(..4)
        .map(|b| f32::from_be_bytes(b.try_into().expect("4 bytes")))
}

pub fn parse_battery_voltage_state(payload: &[u8]) -> Option<BatteryVoltageState> {
    payload.first().copied().map(BatteryVoltageState::from_byte)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn voltage_parses_from_big_endian_f32() {
        // 12.5f32 == 0x41480000
        assert_eq!(parse_voltage(&[0x41, 0x48, 0x00, 0x00]), Some(12.5));
        assert_eq!(parse_voltage(&[0x41, 0x48]), None, "truncated payload");
    }

    #[test]
    fn battery_states_map_to_the_documented_codes() {
        assert_eq!(BatteryVoltageState::from_byte(1), BatteryVoltageState::Ok);
        assert_eq!(
            BatteryVoltageState::from_byte(3),
            BatteryVoltageState::Critical
        );
        assert_eq!(
            BatteryVoltageState::from_byte(99),
            BatteryVoltageState::Unknown
        );
    }

    #[test]
    fn percentage_parses_from_a_single_byte() {
        assert_eq!(parse_battery_percentage(&[87]), Some(87));
        assert_eq!(parse_battery_percentage(&[]), None);
    }
}
