//! Drive commands (device `0x16`), sent to the ST processor.
//!
//! The RVR closes the velocity loop onboard, so the SI commands take real m/s
//! and the host never deals in PWM duty cycles.
//!
//! One convention trap lives here: [`drive_with_heading`] measures heading
//! *clockwise*-positive, while every SI command below is counter-clockwise
//! positive per the right-hand rule. The base driver uses only the SI commands
//! and so sidesteps it entirely.

use crate::ids::{DeviceId, Target};

pub const DEVICE: DeviceId = DeviceId::Drive;
pub const TARGET: Target = Target::Secondary;

pub mod cid {
    pub const RAW_MOTORS: u8 = 0x01;
    pub const RESET_YAW: u8 = 0x06;
    pub const DRIVE_WITH_HEADING: u8 = 0x07;
    pub const SET_DEFAULT_CONTROL_SYSTEM_FOR_TYPE: u8 = 0x0E;
    pub const SET_CUSTOM_CONTROL_SYSTEM_TIMEOUT: u8 = 0x22;
    pub const ENABLE_MOTOR_STALL_NOTIFY: u8 = 0x25;
    pub const MOTOR_STALL_NOTIFY: u8 = 0x26;
    pub const ENABLE_MOTOR_FAULT_NOTIFY: u8 = 0x27;
    pub const MOTOR_FAULT_NOTIFY: u8 = 0x28;
    pub const GET_MOTOR_FAULT_STATE: u8 = 0x29;
    pub const DRIVE_TANK_SI_UNITS: u8 = 0x32;
    pub const DRIVE_TANK_NORMALIZED: u8 = 0x33;
    pub const DRIVE_RC_SI_UNITS: u8 = 0x34;
    pub const DRIVE_RC_NORMALIZED: u8 = 0x35;
    pub const DRIVE_WITH_YAW_SI: u8 = 0x36;
    pub const DRIVE_WITH_YAW_NORMALIZED: u8 = 0x37;
    pub const DRIVE_TO_POSITION_SI: u8 = 0x38;
    pub const DRIVE_TO_POSITION_NORMALIZED: u8 = 0x39;
    pub const XY_POSITION_DRIVE_RESULT_NOTIFY: u8 = 0x3A;
    pub const SET_DRIVE_TARGET_SLEW_PARAMETERS: u8 = 0x3C;
    pub const GET_DRIVE_TARGET_SLEW_PARAMETERS: u8 = 0x3D;
    pub const DRIVE_STOP_CUSTOM_DECEL: u8 = 0x3E;
    pub const ROBOT_HAS_STOPPED_NOTIFY: u8 = 0x3F;
    pub const RESTORE_DEFAULT_DRIVE_TARGET_SLEW_PARAMETERS: u8 = 0x40;
    pub const GET_STOP_CONTROLLER_STATE: u8 = 0x41;
    pub const DRIVE_STOP: u8 = 0x42;
    pub const RESTORE_DEFAULT_CONTROL_SYSTEM_TIMEOUT: u8 = 0x43;
    pub const GET_ACTIVE_CONTROL_SYSTEM_ID: u8 = 0x44;
    pub const RESTORE_INITIAL_DEFAULT_CONTROL_SYSTEMS: u8 = 0x45;
    pub const GET_DEFAULT_CONTROL_SYSTEM_FOR_TYPE: u8 = 0x46;
}

/// Motor direction for [`raw_motors`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RawMotorMode {
    Off = 0,
    Forward = 1,
    Reverse = 2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MotorIndex {
    Left = 0,
    Right = 1,
}

/// Flag bits for the heading- and RC-style drive commands.
pub mod drive_flags {
    pub const NONE: u8 = 0;
    pub const DRIVE_REVERSE: u8 = 1;
    pub const BOOST: u8 = 2;
    pub const FAST_TURN: u8 = 4;
    pub const LEFT_DIRECTION: u8 = 8;
    pub const RIGHT_DIRECTION: u8 = 16;
    pub const ENABLE_DRIFT: u8 = 32;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ControlSystemType {
    Stop = 0,
    RawMotor = 1,
    TankDrive = 2,
    DriveWithYaw = 3,
    RcDrive = 4,
    XyPositionDrive = 5,
    InfraredDrive = 6,
    MagnetometerDrive = 7,
}

/// Per-wheel closed-loop velocity in m/s. The primary command for a
/// differential-drive base.
pub fn drive_tank_si_units(left_mps: f32, right_mps: f32) -> Vec<u8> {
    let mut payload = Vec::with_capacity(8);
    payload.extend_from_slice(&left_mps.to_be_bytes());
    payload.extend_from_slice(&right_mps.to_be_bytes());
    payload
}

/// Per-wheel velocity as a signed fraction of full scale (-127..=127).
pub fn drive_tank_normalized(left: i8, right: i8) -> Vec<u8> {
    vec![left as u8, right as u8]
}

/// Yaw rate (deg/s, CCW positive) plus forward velocity (m/s).
pub fn drive_rc_si_units(
    yaw_angular_velocity_dps: f32,
    linear_velocity_mps: f32,
    flags: u8,
) -> Vec<u8> {
    let mut payload = Vec::with_capacity(9);
    payload.extend_from_slice(&yaw_angular_velocity_dps.to_be_bytes());
    payload.extend_from_slice(&linear_velocity_mps.to_be_bytes());
    payload.push(flags);
    payload
}

pub fn drive_rc_normalized(yaw_angular_velocity: i8, linear_velocity: i8, flags: u8) -> Vec<u8> {
    vec![yaw_angular_velocity as u8, linear_velocity as u8, flags]
}

/// Hold an absolute yaw angle (degrees, CCW positive) while driving forward.
pub fn drive_with_yaw_si(yaw_angle_deg: f32, linear_velocity_mps: f32) -> Vec<u8> {
    let mut payload = Vec::with_capacity(8);
    payload.extend_from_slice(&yaw_angle_deg.to_be_bytes());
    payload.extend_from_slice(&linear_velocity_mps.to_be_bytes());
    payload
}

pub fn drive_with_yaw_normalized(yaw_angle: i16, linear_velocity: i8) -> Vec<u8> {
    let mut payload = Vec::with_capacity(3);
    payload.extend_from_slice(&yaw_angle.to_be_bytes());
    payload.push(linear_velocity as u8);
    payload
}

/// Legacy heading drive. Heading is **clockwise**-positive (0 forward, 90 right),
/// the opposite of every SI command here.
pub fn drive_with_heading(speed: u8, heading_deg: u16, flags: u8) -> Vec<u8> {
    let mut payload = Vec::with_capacity(4);
    payload.push(speed);
    payload.extend_from_slice(&heading_deg.to_be_bytes());
    payload.push(flags);
    payload
}

/// Open-loop motor duty cycles (0..=255). Bypasses the onboard controller.
pub fn raw_motors(
    left_mode: RawMotorMode,
    left_duty: u8,
    right_mode: RawMotorMode,
    right_duty: u8,
) -> Vec<u8> {
    vec![left_mode as u8, left_duty, right_mode as u8, right_duty]
}

/// Zero the yaw reference.
pub fn reset_yaw() -> Vec<u8> {
    Vec::new()
}

/// Decelerate to a stop using the active control system.
pub fn drive_stop() -> Vec<u8> {
    Vec::new()
}

pub fn drive_stop_custom_decel(deceleration_rate: f32) -> Vec<u8> {
    deceleration_rate.to_be_bytes().to_vec()
}

/// How long the robot keeps driving without a fresh command before stopping.
///
/// Defaults to roughly 2 s. A base driver publishing at a steady rate should
/// either re-issue faster than this or raise it deliberately.
pub fn set_custom_control_system_timeout(timeout_ms: u16) -> Vec<u8> {
    timeout_ms.to_be_bytes().to_vec()
}

pub fn restore_default_control_system_timeout() -> Vec<u8> {
    Vec::new()
}

pub fn enable_motor_stall_notify(enabled: bool) -> Vec<u8> {
    vec![enabled as u8]
}

pub fn enable_motor_fault_notify(enabled: bool) -> Vec<u8> {
    vec![enabled as u8]
}

pub fn get_motor_fault_state() -> Vec<u8> {
    Vec::new()
}

pub fn set_default_control_system_for_type(
    system: ControlSystemType,
    controller_id: u8,
) -> Vec<u8> {
    vec![system as u8, controller_id]
}

/// Decoded `motor_stall_notify` payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotorStall {
    pub motor: MotorIndex,
    pub stalled: bool,
}

pub fn parse_motor_stall_notify(payload: &[u8]) -> Option<MotorStall> {
    let (&index, &triggered) = (payload.first()?, payload.get(1)?);
    let motor = match index {
        0 => MotorIndex::Left,
        1 => MotorIndex::Right,
        _ => return None,
    };
    Some(MotorStall {
        motor,
        stalled: triggered != 0,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tank_si_encodes_two_big_endian_f32s() {
        // 1.0f32 is 0x3F800000; -1.0f32 is 0xBF800000. Big-endian on the wire.
        assert_eq!(
            drive_tank_si_units(1.0, -1.0),
            vec![0x3F, 0x80, 0x00, 0x00, 0xBF, 0x80, 0x00, 0x00]
        );
    }

    #[test]
    fn tank_si_round_trips_through_the_wire_format() {
        let bytes = drive_tank_si_units(0.35, -0.125);
        let left = f32::from_be_bytes(bytes[0..4].try_into().unwrap());
        let right = f32::from_be_bytes(bytes[4..8].try_into().unwrap());
        assert_eq!((left, right), (0.35, -0.125));
    }

    #[test]
    fn normalized_commands_encode_signed_bytes_as_twos_complement() {
        assert_eq!(drive_tank_normalized(127, -127), vec![0x7F, 0x81]);
        assert_eq!(drive_tank_normalized(0, -1), vec![0x00, 0xFF]);
    }

    #[test]
    fn rc_si_appends_the_flags_byte_after_both_floats() {
        let payload = drive_rc_si_units(90.0, 0.5, drive_flags::NONE);
        assert_eq!(payload.len(), 9);
        assert_eq!(payload[8], 0);
        assert_eq!(f32::from_be_bytes(payload[0..4].try_into().unwrap()), 90.0);
    }

    #[test]
    fn heading_and_timeout_use_big_endian_u16() {
        assert_eq!(drive_with_heading(64, 270, 0), vec![64, 0x01, 0x0E, 0]);
        assert_eq!(set_custom_control_system_timeout(2000), vec![0x07, 0xD0]);
    }

    #[test]
    fn commands_without_arguments_send_empty_payloads() {
        assert!(reset_yaw().is_empty());
        assert!(drive_stop().is_empty());
    }

    #[test]
    fn parses_motor_stall_notifications() {
        assert_eq!(
            parse_motor_stall_notify(&[1, 1]),
            Some(MotorStall {
                motor: MotorIndex::Right,
                stalled: true
            })
        );
        assert_eq!(
            parse_motor_stall_notify(&[0, 0]),
            Some(MotorStall {
                motor: MotorIndex::Left,
                stalled: false
            })
        );
        assert_eq!(
            parse_motor_stall_notify(&[9, 0]),
            None,
            "unknown motor index"
        );
        assert_eq!(parse_motor_stall_notify(&[0]), None, "truncated");
    }
}
