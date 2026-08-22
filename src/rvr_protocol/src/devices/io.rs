//! LED control (device `0x1A`), sent to the Nordic processor.
//!
//! [`set_all_leds`] takes a 32-bit mask of channels plus one brightness byte per
//! set bit, ordered from the least-significant bit upward. Each RGB group is
//! three consecutive bits, so a group's bytes are always `[r, g, b]`.

use crate::ids::{DeviceId, Target};

pub const DEVICE: DeviceId = DeviceId::Io;
pub const TARGET: Target = Target::Primary;

pub mod cid {
    pub const SET_ALL_LEDS: u8 = 0x1A;
    pub const GET_ACTIVE_COLOR_PALETTE: u8 = 0x44;
    pub const SET_ACTIVE_COLOR_PALETTE: u8 = 0x45;
    pub const GET_COLOR_IDENTIFICATION_REPORT: u8 = 0x46;
    pub const LOAD_COLOR_PALETTE: u8 = 0x47;
    pub const SAVE_COLOR_PALETTE: u8 = 0x48;
    pub const RELEASE_LED_REQUESTS: u8 = 0x4E;
}

/// Individual LED channel bits.
pub mod led_bits {
    pub const RIGHT_HEADLIGHT_RED: u32 = 0x0000_0001;
    pub const RIGHT_HEADLIGHT_GREEN: u32 = 0x0000_0002;
    pub const RIGHT_HEADLIGHT_BLUE: u32 = 0x0000_0004;
    pub const LEFT_HEADLIGHT_RED: u32 = 0x0000_0008;
    pub const LEFT_HEADLIGHT_GREEN: u32 = 0x0000_0010;
    pub const LEFT_HEADLIGHT_BLUE: u32 = 0x0000_0020;
    pub const LEFT_STATUS_RED: u32 = 0x0000_0040;
    pub const LEFT_STATUS_GREEN: u32 = 0x0000_0080;
    pub const LEFT_STATUS_BLUE: u32 = 0x0000_0100;
    pub const RIGHT_STATUS_RED: u32 = 0x0000_0200;
    pub const RIGHT_STATUS_GREEN: u32 = 0x0000_0400;
    pub const RIGHT_STATUS_BLUE: u32 = 0x0000_0800;
    pub const BATTERY_DOOR_REAR_RED: u32 = 0x0000_1000;
    pub const BATTERY_DOOR_REAR_GREEN: u32 = 0x0000_2000;
    pub const BATTERY_DOOR_REAR_BLUE: u32 = 0x0000_4000;
    pub const BATTERY_DOOR_FRONT_RED: u32 = 0x0000_8000;
    pub const BATTERY_DOOR_FRONT_GREEN: u32 = 0x0001_0000;
    pub const BATTERY_DOOR_FRONT_BLUE: u32 = 0x0002_0000;
    pub const POWER_BUTTON_FRONT_RED: u32 = 0x0004_0000;
    pub const POWER_BUTTON_FRONT_GREEN: u32 = 0x0008_0000;
    pub const POWER_BUTTON_FRONT_BLUE: u32 = 0x0010_0000;
    pub const POWER_BUTTON_REAR_RED: u32 = 0x0020_0000;
    pub const POWER_BUTTON_REAR_GREEN: u32 = 0x0040_0000;
    pub const POWER_BUTTON_REAR_BLUE: u32 = 0x0080_0000;
    pub const LEFT_BRAKELIGHT_RED: u32 = 0x0100_0000;
    pub const LEFT_BRAKELIGHT_GREEN: u32 = 0x0200_0000;
    pub const LEFT_BRAKELIGHT_BLUE: u32 = 0x0400_0000;
    pub const RIGHT_BRAKELIGHT_RED: u32 = 0x0800_0000;
    pub const RIGHT_BRAKELIGHT_GREEN: u32 = 0x1000_0000;
    pub const RIGHT_BRAKELIGHT_BLUE: u32 = 0x2000_0000;
    pub const UNDERCARRIAGE_WHITE: u32 = 0x4000_0000;
}

/// An addressable RGB group, named by its lowest (red) bit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum LedGroup {
    RightHeadlight = led_bits::RIGHT_HEADLIGHT_RED,
    LeftHeadlight = led_bits::LEFT_HEADLIGHT_RED,
    LeftStatus = led_bits::LEFT_STATUS_RED,
    RightStatus = led_bits::RIGHT_STATUS_RED,
    BatteryDoorRear = led_bits::BATTERY_DOOR_REAR_RED,
    BatteryDoorFront = led_bits::BATTERY_DOOR_FRONT_RED,
    PowerButtonFront = led_bits::POWER_BUTTON_FRONT_RED,
    PowerButtonRear = led_bits::POWER_BUTTON_REAR_RED,
    LeftBrakelight = led_bits::LEFT_BRAKELIGHT_RED,
    RightBrakelight = led_bits::RIGHT_BRAKELIGHT_RED,
}

impl LedGroup {
    /// Mask covering this group's red, green and blue bits.
    pub const fn mask(self) -> u32 {
        let red = self as u32;
        red | (red << 1) | (red << 2)
    }

    /// Every RGB group, excluding the single-channel undercarriage light.
    pub const ALL: [LedGroup; 10] = [
        Self::RightHeadlight,
        Self::LeftHeadlight,
        Self::LeftStatus,
        Self::RightStatus,
        Self::BatteryDoorRear,
        Self::BatteryDoorFront,
        Self::PowerButtonFront,
        Self::PowerButtonRear,
        Self::LeftBrakelight,
        Self::RightBrakelight,
    ];
}

/// Mask covering all ten RGB groups.
pub fn all_lights_mask() -> u32 {
    LedGroup::ALL.iter().fold(0, |acc, g| acc | g.mask())
}

/// Raw form: a channel mask plus one brightness byte per set bit, LSB first.
pub fn set_all_leds(mask: u32, brightnesses: &[u8]) -> Vec<u8> {
    debug_assert_eq!(
        mask.count_ones() as usize,
        brightnesses.len(),
        "one brightness byte is required per set mask bit"
    );
    let mut payload = Vec::with_capacity(4 + brightnesses.len());
    payload.extend_from_slice(&mask.to_be_bytes());
    payload.extend_from_slice(brightnesses);
    payload
}

/// Set one RGB group to a color.
pub fn set_group_color(group: LedGroup, rgb: [u8; 3]) -> Vec<u8> {
    set_all_leds(group.mask(), &rgb)
}

/// Set every RGB group to the same color.
pub fn set_all_color(rgb: [u8; 3]) -> Vec<u8> {
    let brightnesses: Vec<u8> = LedGroup::ALL.iter().flat_map(|_| rgb).collect();
    set_all_leds(all_lights_mask(), &brightnesses)
}

/// Set the single-channel undercarriage light.
pub fn set_undercarriage(brightness: u8) -> Vec<u8> {
    set_all_leds(led_bits::UNDERCARRIAGE_WHITE, &[brightness])
}

pub fn release_led_requests() -> Vec<u8> {
    Vec::new()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn group_masks_cover_three_consecutive_bits() {
        assert_eq!(LedGroup::RightHeadlight.mask(), 0b111);
        assert_eq!(LedGroup::LeftHeadlight.mask(), 0b111_000);
        assert_eq!(LedGroup::RightHeadlight.mask().count_ones(), 3);
    }

    #[test]
    fn all_lights_covers_ten_rgb_groups_but_not_the_undercarriage() {
        let mask = all_lights_mask();
        assert_eq!(mask.count_ones(), 30, "ten groups of three channels");
        assert_eq!(mask & led_bits::UNDERCARRIAGE_WHITE, 0);
    }

    #[test]
    fn set_group_color_emits_mask_then_rgb() {
        assert_eq!(
            set_group_color(LedGroup::RightHeadlight, [0x11, 0x22, 0x33]),
            vec![0x00, 0x00, 0x00, 0x07, 0x11, 0x22, 0x33]
        );
    }

    #[test]
    fn set_all_color_emits_one_rgb_triple_per_group() {
        let payload = set_all_color([0xAA, 0xBB, 0xCC]);
        assert_eq!(payload.len(), 4 + 30);
        assert_eq!(
            u32::from_be_bytes(payload[..4].try_into().unwrap()),
            all_lights_mask()
        );
        // Every group repeats the same triple.
        for chunk in payload[4..].chunks_exact(3) {
            assert_eq!(chunk, &[0xAA, 0xBB, 0xCC]);
        }
    }

    #[test]
    fn undercarriage_is_a_single_channel() {
        let payload = set_undercarriage(0x80);
        assert_eq!(payload.len(), 5);
        assert_eq!(payload[4], 0x80);
    }
}
