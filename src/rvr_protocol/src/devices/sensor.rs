//! Sensor commands (device `0x18`).
//!
//! Streaming configuration lives in [`crate::streaming`]; this module carries the
//! command ids and the one-shot sensor reads.

use crate::error::Result;
use crate::ids::{DeviceId, Target};
use crate::streaming::{SlotConfig, MIN_STREAM_PERIOD_MS};

pub const DEVICE: DeviceId = DeviceId::Sensor;

pub mod cid {
    pub const ENABLE_GYRO_MAX_NOTIFY: u8 = 0x0F;
    pub const GYRO_MAX_NOTIFY: u8 = 0x10;
    pub const RESET_LOCATOR_X_AND_Y: u8 = 0x13;
    pub const SET_LOCATOR_FLAGS: u8 = 0x17;
    pub const GET_BOT_TO_BOT_INFRARED_READINGS: u8 = 0x22;
    pub const GET_RGBC_SENSOR_VALUES: u8 = 0x23;
    pub const MAGNETOMETER_CALIBRATE_TO_NORTH: u8 = 0x25;
    pub const GET_AMBIENT_LIGHT_SENSOR_VALUE: u8 = 0x30;
    pub const ENABLE_COLOR_DETECTION_NOTIFY: u8 = 0x35;
    pub const COLOR_DETECTION_NOTIFY: u8 = 0x36;
    pub const GET_CURRENT_DETECTED_COLOR_READING: u8 = 0x37;
    pub const ENABLE_COLOR_DETECTION: u8 = 0x38;
    pub const CONFIGURE_STREAMING_SERVICE: u8 = 0x39;
    pub const START_STREAMING_SERVICE: u8 = 0x3A;
    pub const STOP_STREAMING_SERVICE: u8 = 0x3B;
    pub const CLEAR_STREAMING_SERVICE: u8 = 0x3C;
    pub const STREAMING_SERVICE_DATA_NOTIFY: u8 = 0x3D;
    pub const ENABLE_ROBOT_INFRARED_MESSAGE_NOTIFY: u8 = 0x3E;
    pub const SEND_INFRARED_MESSAGE: u8 = 0x3F;
    pub const GET_TEMPERATURE: u8 = 0x4A;
    pub const GET_MOTOR_THERMAL_PROTECTION_STATUS: u8 = 0x4B;
    pub const ENABLE_MOTOR_THERMAL_PROTECTION_STATUS_NOTIFY: u8 = 0x4C;
    pub const MOTOR_THERMAL_PROTECTION_STATUS_NOTIFY: u8 = 0x4D;
    pub const MAGNETOMETER_CALIBRATION_COMPLETE_NOTIFY: u8 = 0x51;
    pub const GET_MAGNETOMETER_READING: u8 = 0x52;
    pub const GET_ENCODER_COUNTS: u8 = 0x53;
    pub const DISABLE_NOTIFICATIONS_AND_ACTIVE_COMMANDS: u8 = 0x54;
}

/// Locator behaviour bits for [`set_locator_flags`].
pub mod locator_flags {
    /// Keep the startup X-Y axes across a driving yaw reset.
    pub const AUTO_CALIBRATE: u8 = 0x01;
}

/// Payload for `configure_streaming_service`.
pub fn configure_streaming_service(slot: &SlotConfig) -> Vec<u8> {
    slot.to_config_payload()
}

/// Payload for `start_streaming_service`. The period is clamped to the firmware
/// minimum, since a smaller value is silently rejected.
pub fn start_streaming_service(period_ms: u16) -> Vec<u8> {
    period_ms.max(MIN_STREAM_PERIOD_MS).to_be_bytes().to_vec()
}

pub fn stop_streaming_service() -> Vec<u8> {
    Vec::new()
}

pub fn clear_streaming_service() -> Vec<u8> {
    Vec::new()
}

/// Zero the locator's X and Y estimate.
pub fn reset_locator_x_and_y() -> Vec<u8> {
    Vec::new()
}

pub fn set_locator_flags(flags: u8) -> Vec<u8> {
    vec![flags]
}

pub fn get_ambient_light_sensor_value() -> Vec<u8> {
    Vec::new()
}

pub fn get_rgbc_sensor_values() -> Vec<u8> {
    Vec::new()
}

pub fn enable_color_detection(enabled: bool) -> Vec<u8> {
    vec![enabled as u8]
}

pub fn enable_color_detection_notify(
    enabled: bool,
    interval_ms: u16,
    confidence_threshold: u8,
) -> Vec<u8> {
    let mut payload = Vec::with_capacity(4);
    payload.push(enabled as u8);
    payload.extend_from_slice(&interval_ms.to_be_bytes());
    payload.push(confidence_threshold);
    payload
}

pub fn get_encoder_counts() -> Vec<u8> {
    Vec::new()
}

pub fn get_temperature() -> Vec<u8> {
    Vec::new()
}

pub fn disable_notifications_and_active_commands() -> Vec<u8> {
    Vec::new()
}

/// Which processor a sensor command addresses.
///
/// Ambient light and color live on the Nordic side; motion sensing on the ST side.
pub const fn target_for(command_id: u8) -> Target {
    match command_id {
        cid::GET_AMBIENT_LIGHT_SENSOR_VALUE
        | cid::GET_RGBC_SENSOR_VALUES
        | cid::ENABLE_COLOR_DETECTION
        | cid::ENABLE_COLOR_DETECTION_NOTIFY
        | cid::GET_CURRENT_DETECTED_COLOR_READING => Target::Primary,
        _ => Target::Secondary,
    }
}

/// Parse the two big-endian `i32` counts from `get_encoder_counts`.
pub fn parse_encoder_counts(payload: &[u8]) -> Result<(i32, i32)> {
    Ok((
        i32::from_be_bytes(crate::devices::be_bytes(payload, 0)?),
        i32::from_be_bytes(crate::devices::be_bytes(payload, 4)?),
    ))
}

/// Parse a big-endian `f32` ambient light reading, in lux.
pub fn parse_ambient_light(payload: &[u8]) -> Result<f32> {
    crate::devices::be_bytes(payload, 0).map(f32::from_be_bytes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::streaming::ServiceId;

    #[test]
    fn start_streaming_clamps_below_the_firmware_minimum() {
        assert_eq!(
            start_streaming_service(10),
            vec![0x00, 33],
            "clamped up to 33 ms"
        );
        assert_eq!(start_streaming_service(100), vec![0x00, 100]);
    }

    #[test]
    fn configure_delegates_to_the_slot_layout() {
        let slot = SlotConfig::new(Target::Secondary, 0x02)
            .with(ServiceId::Locator)
            .unwrap();
        assert_eq!(configure_streaming_service(&slot), slot.to_config_payload());
    }

    #[test]
    fn sensor_commands_route_to_the_owning_processor() {
        assert_eq!(
            target_for(cid::GET_AMBIENT_LIGHT_SENSOR_VALUE),
            Target::Primary
        );
        assert_eq!(target_for(cid::ENABLE_COLOR_DETECTION), Target::Primary);
        assert_eq!(target_for(cid::GET_ENCODER_COUNTS), Target::Secondary);
        assert_eq!(target_for(cid::RESET_LOCATOR_X_AND_Y), Target::Secondary);
    }

    #[test]
    fn encoder_counts_parse_as_signed_big_endian() {
        // get_encoder_counts returns i32s directly, unlike the streamed form.
        let payload = [0x00, 0x00, 0x01, 0x00, 0xFF, 0xFF, 0xFF, 0xFF];
        assert_eq!(parse_encoder_counts(&payload).unwrap(), (256, -1));
        assert!(parse_encoder_counts(&payload[..7]).is_err());
    }

    #[test]
    fn ambient_light_parses_from_big_endian_f32() {
        assert_eq!(
            parse_ambient_light(&[0x43, 0xFA, 0x00, 0x00]).unwrap(),
            500.0
        );
    }
}
