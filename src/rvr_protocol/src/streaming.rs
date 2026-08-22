//! Sensor streaming: service table, slot configuration, and sample decoding.
//!
//! The RVR pushes sensor data on its own schedule once configured. Crucially it
//! does *not* tag samples with what they are — each notification carries one
//! opaque blob per slot, and the receiver must know the layout because it chose
//! it. A config/decode mismatch therefore produces plausible-looking numbers
//! rather than an error, so both sides are derived from [`SERVICES`] here rather
//! than from hand-written offsets.

use crate::error::{Error, Result};
use crate::ids::Target;

/// Slot tokens available per processor.
pub const SLOT_TOKENS: [u8; 4] = [0x01, 0x02, 0x03, 0x04];

/// Firmware limit on services sharing one slot.
pub const MAX_SERVICES_PER_SLOT: usize = 6;

/// Fastest streaming interval the firmware accepts.
pub const MIN_STREAM_PERIOD_MS: u16 = 33;

/// Upper nibble of the token byte flags a bad sample.
const STATUS_MASK: u8 = 0xF0;
const STATUS_OK: u8 = 0x00;
const TOKEN_MASK: u8 = 0x0F;

/// Width of each streamed attribute.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DataSize {
    Eight = 0x00,
    Sixteen = 0x01,
    ThirtyTwo = 0x02,
}

impl DataSize {
    /// Bytes per attribute: `1 << code`.
    pub const fn byte_count(self) -> usize {
        1 << (self as u8)
    }

    /// Largest raw value, the denominator when rescaling to real units.
    pub const fn max_raw(self) -> f64 {
        match self {
            Self::Eight => u8::MAX as f64,
            Self::Sixteen => u16::MAX as f64,
            Self::ThirtyTwo => u32::MAX as f64,
        }
    }
}

/// One streamed quantity and the range its raw value maps onto.
#[derive(Debug, Clone, Copy)]
pub struct Attribute {
    pub name: &'static str,
    pub min: f64,
    pub max: f64,
}

impl Attribute {
    const fn new(name: &'static str, min: f64, max: f64) -> Self {
        Self { name, min, max }
    }
}

/// A streaming service: an addressable group of attributes on one processor.
#[derive(Debug, Clone, Copy)]
pub struct ServiceDef {
    pub id: u16,
    pub name: &'static str,
    pub processor: Target,
    pub data_size: DataSize,
    pub attributes: &'static [Attribute],
}

impl ServiceDef {
    /// Bytes this service contributes to a streaming notification.
    pub const fn frame_len(&self) -> usize {
        self.attributes.len() * self.data_size.byte_count()
    }
}

/// Identifies a service without carrying its whole definition.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ServiceId {
    Quaternion = 0x0000,
    Imu = 0x0001,
    Accelerometer = 0x0002,
    ColorDetection = 0x0003,
    Gyroscope = 0x0004,
    Locator = 0x0006,
    Velocity = 0x0007,
    Speed = 0x0008,
    CoreTime = 0x0009,
    AmbientLight = 0x000A,
    Encoders = 0x000B,
}

impl ServiceId {
    pub fn def(self) -> &'static ServiceDef {
        SERVICES
            .iter()
            .find(|s| s.id == self as u16)
            .expect("every ServiceId variant has a SERVICES entry")
    }
}

/// The full service table. Note there is no service `0x0005`.
///
/// Ranges are the firmware's, and the decoder maps raw unsigned integers onto
/// them linearly — see [`decode_attribute`].
pub static SERVICES: &[ServiceDef] = &[
    ServiceDef {
        id: 0x0000,
        name: "Quaternion",
        processor: Target::Secondary,
        data_size: DataSize::ThirtyTwo,
        attributes: &[
            Attribute::new("W", -1.0, 1.0),
            Attribute::new("X", -1.0, 1.0),
            Attribute::new("Y", -1.0, 1.0),
            Attribute::new("Z", -1.0, 1.0),
        ],
    },
    ServiceDef {
        id: 0x0001,
        name: "IMU",
        processor: Target::Secondary,
        data_size: DataSize::ThirtyTwo,
        attributes: &[
            Attribute::new("Pitch", -180.0, 180.0),
            Attribute::new("Roll", -90.0, 90.0),
            Attribute::new("Yaw", -180.0, 180.0),
        ],
    },
    ServiceDef {
        id: 0x0002,
        name: "Accelerometer",
        processor: Target::Secondary,
        data_size: DataSize::ThirtyTwo,
        attributes: &[
            Attribute::new("X", -16.0, 16.0),
            Attribute::new("Y", -16.0, 16.0),
            Attribute::new("Z", -16.0, 16.0),
        ],
    },
    ServiceDef {
        id: 0x0003,
        name: "ColorDetection",
        processor: Target::Primary,
        data_size: DataSize::Eight,
        attributes: &[
            Attribute::new("R", 0.0, 255.0),
            Attribute::new("G", 0.0, 255.0),
            Attribute::new("B", 0.0, 255.0),
            Attribute::new("Index", 0.0, 255.0),
            Attribute::new("Confidence", 0.0, 1.0),
        ],
    },
    ServiceDef {
        id: 0x0004,
        name: "Gyroscope",
        processor: Target::Secondary,
        data_size: DataSize::ThirtyTwo,
        attributes: &[
            Attribute::new("X", -2000.0, 2000.0),
            Attribute::new("Y", -2000.0, 2000.0),
            Attribute::new("Z", -2000.0, 2000.0),
        ],
    },
    ServiceDef {
        id: 0x0006,
        name: "Locator",
        processor: Target::Secondary,
        data_size: DataSize::ThirtyTwo,
        attributes: &[
            Attribute::new("X", -16000.0, 16000.0),
            Attribute::new("Y", -16000.0, 16000.0),
        ],
    },
    ServiceDef {
        id: 0x0007,
        name: "Velocity",
        processor: Target::Secondary,
        data_size: DataSize::ThirtyTwo,
        attributes: &[
            Attribute::new("X", -5.0, 5.0),
            Attribute::new("Y", -5.0, 5.0),
        ],
    },
    ServiceDef {
        id: 0x0008,
        name: "Speed",
        processor: Target::Secondary,
        data_size: DataSize::ThirtyTwo,
        attributes: &[Attribute::new("Speed", 0.0, 5.0)],
    },
    ServiceDef {
        id: 0x0009,
        name: "CoreTime",
        processor: Target::Secondary,
        data_size: DataSize::ThirtyTwo,
        attributes: &[
            Attribute::new("TimeUpper", 0.0, u32::MAX as f64),
            Attribute::new("TimeLower", 0.0, u32::MAX as f64),
        ],
    },
    ServiceDef {
        id: 0x000A,
        name: "AmbientLight",
        processor: Target::Primary,
        data_size: DataSize::ThirtyTwo,
        attributes: &[Attribute::new("Light", 0.0, 120_000.0)],
    },
    ServiceDef {
        id: 0x000B,
        name: "Encoders",
        processor: Target::Secondary,
        data_size: DataSize::ThirtyTwo,
        attributes: &[
            Attribute::new("LeftTicks", 0.0, u32::MAX as f64),
            Attribute::new("RightTicks", 0.0, u32::MAX as f64),
        ],
    },
];

/// One slot's contents on one processor.
///
/// Service order is preserved because it defines the decode order of the blob
/// the robot sends back.
#[derive(Debug, Clone)]
pub struct SlotConfig {
    pub processor: Target,
    pub token: u8,
    services: Vec<ServiceId>,
}

impl SlotConfig {
    pub fn new(processor: Target, token: u8) -> Self {
        Self {
            processor,
            token,
            services: Vec::new(),
        }
    }

    /// Add a service. Fails if the slot is full or the service lives on another
    /// processor — either would silently corrupt decoding.
    pub fn add(&mut self, service: ServiceId) -> Result<()> {
        if self.services.len() >= MAX_SERVICES_PER_SLOT {
            return Err(Error::SlotFull {
                max: MAX_SERVICES_PER_SLOT,
            });
        }
        debug_assert_eq!(
            service.def().processor,
            self.processor,
            "service {} lives on the other processor",
            service.def().name
        );
        self.services.push(service);
        Ok(())
    }

    pub fn with(mut self, service: ServiceId) -> Result<Self> {
        self.add(service)?;
        Ok(self)
    }

    pub fn services(&self) -> &[ServiceId] {
        &self.services
    }

    pub fn is_empty(&self) -> bool {
        self.services.is_empty()
    }

    /// Payload for `configure_streaming_service`: the token, then three bytes per
    /// service (big-endian id, then the data-size code). An empty slot sends a
    /// single zero byte.
    pub fn to_config_payload(&self) -> Vec<u8> {
        let mut payload = vec![self.token];
        if self.services.is_empty() {
            payload.push(0);
            return payload;
        }
        for service in &self.services {
            let def = service.def();
            payload.extend_from_slice(&def.id.to_be_bytes());
            payload.push(def.data_size as u8);
        }
        payload
    }

    /// Total attribute bytes the robot will send for this slot.
    pub fn frame_len(&self) -> usize {
        self.services.iter().map(|s| s.def().frame_len()).sum()
    }
}

/// A decoded reading from one streaming service.
///
/// A typed enum rather than a string-keyed map, so consumers match exhaustively
/// and the compiler flags any service they forget to handle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StreamSample {
    Quaternion {
        w: f32,
        x: f32,
        y: f32,
        z: f32,
    },
    /// Degrees.
    Imu {
        pitch: f32,
        roll: f32,
        yaw: f32,
    },
    /// g.
    Accelerometer {
        x: f32,
        y: f32,
        z: f32,
    },
    ColorDetection {
        r: u8,
        g: u8,
        b: u8,
        index: u8,
        confidence: f32,
    },
    /// Degrees/second.
    Gyroscope {
        x: f32,
        y: f32,
        z: f32,
    },
    /// Meters, in the RVR's X-right / Y-forward frame (not ROS's).
    Locator {
        x: f32,
        y: f32,
    },
    /// Meters/second, same frame as [`StreamSample::Locator`].
    Velocity {
        x: f32,
        y: f32,
    },
    /// Meters/second, unsigned magnitude.
    Speed {
        speed: f32,
    },
    CoreTime {
        upper: u32,
        lower: u32,
    },
    /// Lux.
    AmbientLight {
        lux: f32,
    },
    Encoders {
        left: u32,
        right: u32,
    },
}

/// Map a raw unsigned integer onto the attribute's range.
///
/// Streamed values are *not* two's-complement: they arrive as unsigned integers
/// spanning `[0, max_raw]`, linearly rescaled onto `[min, max]`. Casting the raw
/// bytes to a signed integer instead yields garbage for every signed quantity.
pub fn decode_attribute(raw: u32, size: DataSize, attr: &Attribute) -> f64 {
    (raw as f64 / size.max_raw()) * (attr.max - attr.min) + attr.min
}

/// Decode one service's slice of a streaming blob.
fn decode_service(def: &ServiceDef, bytes: &[u8]) -> StreamSample {
    let size = def.data_size;
    let width = size.byte_count();

    let raws: Vec<u32> = bytes
        .chunks_exact(width)
        .map(|chunk| chunk.iter().fold(0u32, |acc, &b| (acc << 8) | b as u32))
        .collect();

    let value = |i: usize| decode_attribute(raws[i], size, &def.attributes[i]) as f32;
    // Attributes with integral bounds map identically, so the raw value is exact.
    let integral = |i: usize| raws[i];

    match def.id {
        0x0000 => StreamSample::Quaternion {
            w: value(0),
            x: value(1),
            y: value(2),
            z: value(3),
        },
        0x0001 => StreamSample::Imu {
            pitch: value(0),
            roll: value(1),
            yaw: value(2),
        },
        0x0002 => StreamSample::Accelerometer {
            x: value(0),
            y: value(1),
            z: value(2),
        },
        0x0003 => StreamSample::ColorDetection {
            r: raws[0] as u8,
            g: raws[1] as u8,
            b: raws[2] as u8,
            index: raws[3] as u8,
            confidence: value(4),
        },
        0x0004 => StreamSample::Gyroscope {
            x: value(0),
            y: value(1),
            z: value(2),
        },
        0x0006 => StreamSample::Locator {
            x: value(0),
            y: value(1),
        },
        0x0007 => StreamSample::Velocity {
            x: value(0),
            y: value(1),
        },
        0x0008 => StreamSample::Speed { speed: value(0) },
        0x0009 => StreamSample::CoreTime {
            upper: integral(0),
            lower: integral(1),
        },
        0x000A => StreamSample::AmbientLight { lux: value(0) },
        0x000B => StreamSample::Encoders {
            left: integral(0),
            right: integral(1),
        },
        other => unreachable!("service {other:#06X} is not in SERVICES"),
    }
}

/// Outcome of decoding one `streaming_service_data_notify` payload.
#[derive(Debug, Clone, PartialEq)]
pub enum StreamNotification {
    /// Samples in the order the slot was configured.
    Samples(Vec<StreamSample>),
    /// The robot flagged the sample as invalid; callers should discard it rather
    /// than publish a bogus reading.
    Invalid { token: u8 },
}

/// Decode a streaming notification against the slot that produced it.
///
/// `slot` must be the configuration sent for `(processor, token)` — the blob
/// carries no type information of its own.
pub fn decode_notification(slot: &SlotConfig, payload: &[u8]) -> Result<StreamNotification> {
    let &token_byte = payload.first().ok_or(Error::ShortPayload {
        expected: 1,
        actual: 0,
    })?;

    if token_byte & STATUS_MASK != STATUS_OK {
        return Ok(StreamNotification::Invalid {
            token: token_byte & TOKEN_MASK,
        });
    }

    let data = &payload[1..];
    let expected = slot.frame_len();
    if data.len() < expected {
        return Err(Error::ShortPayload {
            expected,
            actual: data.len(),
        });
    }

    let mut samples = Vec::with_capacity(slot.services().len());
    let mut offset = 0;
    for service in slot.services() {
        let def = service.def();
        let end = offset + def.frame_len();
        samples.push(decode_service(def, &data[offset..end]));
        offset = end;
    }
    Ok(StreamNotification::Samples(samples))
}

/// Extract the slot token from a notification payload.
pub fn notification_token(payload: &[u8]) -> Option<u8> {
    payload.first().map(|b| b & TOKEN_MASK)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Raw value that decodes to `target` for the given attribute.
    fn raw_for(target: f64, size: DataSize, attr: &Attribute) -> u32 {
        (((target - attr.min) / (attr.max - attr.min)) * size.max_raw()).round() as u32
    }

    fn push_be32(out: &mut Vec<u8>, value: u32) {
        out.extend_from_slice(&value.to_be_bytes());
    }

    #[test]
    fn service_table_is_internally_consistent() {
        for def in SERVICES {
            assert!(!def.attributes.is_empty(), "{} has no attributes", def.name);
            assert_eq!(
                def.frame_len(),
                def.attributes.len() * def.data_size.byte_count()
            );
        }
        // 0x0005 is deliberately absent from the firmware's table.
        assert!(SERVICES.iter().all(|s| s.id != 0x0005));
    }

    #[test]
    fn service_ids_resolve_to_their_definitions() {
        assert_eq!(ServiceId::Locator.def().name, "Locator");
        assert_eq!(ServiceId::ColorDetection.def().data_size, DataSize::Eight);
        assert_eq!(ServiceId::AmbientLight.def().processor, Target::Primary);
        assert_eq!(ServiceId::Imu.def().processor, Target::Secondary);
    }

    #[test]
    fn data_sizes_follow_the_power_of_two_rule() {
        assert_eq!(DataSize::Eight.byte_count(), 1);
        assert_eq!(DataSize::Sixteen.byte_count(), 2);
        assert_eq!(DataSize::ThirtyTwo.byte_count(), 4);
    }

    #[test]
    fn config_payload_is_token_then_three_bytes_per_service() {
        let slot = SlotConfig::new(Target::Secondary, 0x02)
            .with(ServiceId::Locator)
            .unwrap()
            .with(ServiceId::Velocity)
            .unwrap();

        assert_eq!(
            slot.to_config_payload(),
            vec![
                0x02, // token
                0x00,
                0x06,
                DataSize::ThirtyTwo as u8, // Locator
                0x00,
                0x07,
                DataSize::ThirtyTwo as u8, // Velocity
            ]
        );
    }

    #[test]
    fn empty_slot_configures_as_a_single_zero() {
        let slot = SlotConfig::new(Target::Secondary, 0x03);
        assert_eq!(slot.to_config_payload(), vec![0x03, 0x00]);
    }

    #[test]
    fn slot_rejects_more_than_six_services() {
        let mut slot = SlotConfig::new(Target::Secondary, 0x01);
        for _ in 0..MAX_SERVICES_PER_SLOT {
            slot.add(ServiceId::Speed).unwrap();
        }
        assert!(matches!(
            slot.add(ServiceId::Speed),
            Err(Error::SlotFull { .. })
        ));
    }

    #[test]
    fn decodes_negative_values_via_range_mapping_not_twos_complement() {
        // The trap: values are rescaled across the whole unsigned range, so zero
        // sits at mid-scale and the negative half occupies [0, 0x7FFFFFFF] --
        // bytes that look like a perfectly valid *positive* two's-complement int.
        // There is no sign bit to give the mistake away.
        let def = ServiceId::Locator.def();
        let attr = &def.attributes[0];

        assert_eq!(
            raw_for(0.0, DataSize::ThirtyTwo, attr),
            0x8000_0000,
            "zero should land at mid-scale"
        );

        let raw = raw_for(-1.5, DataSize::ThirtyTwo, attr);
        let decoded = decode_attribute(raw, DataSize::ThirtyTwo, attr);
        assert!((decoded - -1.5).abs() < 1e-3, "decoded {decoded}");

        // What the naive cast would have produced instead: a large positive
        // number rather than a small negative one.
        assert!(raw as i32 > 2_000_000_000, "raw as i32 = {}", raw as i32);
    }

    #[test]
    fn decodes_a_locator_and_velocity_slot() {
        let slot = SlotConfig::new(Target::Secondary, 0x02)
            .with(ServiceId::Locator)
            .unwrap()
            .with(ServiceId::Velocity)
            .unwrap();

        let locator = ServiceId::Locator.def();
        let velocity = ServiceId::Velocity.def();

        let mut payload = vec![0x02]; // status OK, token 2
        push_be32(
            &mut payload,
            raw_for(1.25, DataSize::ThirtyTwo, &locator.attributes[0]),
        );
        push_be32(
            &mut payload,
            raw_for(-0.75, DataSize::ThirtyTwo, &locator.attributes[1]),
        );
        push_be32(
            &mut payload,
            raw_for(0.5, DataSize::ThirtyTwo, &velocity.attributes[0]),
        );
        push_be32(
            &mut payload,
            raw_for(-0.25, DataSize::ThirtyTwo, &velocity.attributes[1]),
        );

        let StreamNotification::Samples(samples) = decode_notification(&slot, &payload).unwrap()
        else {
            panic!("expected samples");
        };
        assert_eq!(samples.len(), 2);

        match samples[0] {
            StreamSample::Locator { x, y } => {
                assert!((x - 1.25).abs() < 1e-2, "x = {x}");
                assert!((y - -0.75).abs() < 1e-2, "y = {y}");
            }
            other => panic!("expected Locator, got {other:?}"),
        }
        match samples[1] {
            StreamSample::Velocity { x, y } => {
                assert!((x - 0.5).abs() < 1e-3, "x = {x}");
                assert!((y - -0.25).abs() < 1e-3, "y = {y}");
            }
            other => panic!("expected Velocity, got {other:?}"),
        }
    }

    #[test]
    fn advances_the_offset_across_services_of_differing_widths() {
        // Ambient light is 32-bit, color detection 8-bit: if the offset advanced by
        // a fixed stride, the second service would decode from the wrong bytes.
        let slot = SlotConfig::new(Target::Primary, 0x01)
            .with(ServiceId::AmbientLight)
            .unwrap()
            .with(ServiceId::ColorDetection)
            .unwrap();
        assert_eq!(slot.frame_len(), 4 + 5);

        let light = ServiceId::AmbientLight.def();
        let mut payload = vec![0x01];
        push_be32(
            &mut payload,
            raw_for(500.0, DataSize::ThirtyTwo, &light.attributes[0]),
        );
        payload.extend_from_slice(&[0x10, 0x20, 0x30, 0x04, 0xFF]); // r,g,b,index,confidence

        let StreamNotification::Samples(samples) = decode_notification(&slot, &payload).unwrap()
        else {
            panic!("expected samples");
        };

        match samples[0] {
            StreamSample::AmbientLight { lux } => assert!((lux - 500.0).abs() < 1.0, "lux = {lux}"),
            other => panic!("expected AmbientLight, got {other:?}"),
        }
        match samples[1] {
            StreamSample::ColorDetection {
                r,
                g,
                b,
                index,
                confidence,
            } => {
                assert_eq!((r, g, b, index), (0x10, 0x20, 0x30, 0x04));
                assert!((confidence - 1.0).abs() < 1e-6);
            }
            other => panic!("expected ColorDetection, got {other:?}"),
        }
    }

    #[test]
    fn integral_attributes_decode_exactly() {
        let slot = SlotConfig::new(Target::Secondary, 0x04)
            .with(ServiceId::Encoders)
            .unwrap();

        let mut payload = vec![0x04];
        push_be32(&mut payload, 123_456);
        push_be32(&mut payload, 654_321);

        let StreamNotification::Samples(samples) = decode_notification(&slot, &payload).unwrap()
        else {
            panic!("expected samples");
        };
        assert_eq!(
            samples[0],
            StreamSample::Encoders {
                left: 123_456,
                right: 654_321
            }
        );
    }

    #[test]
    fn flags_invalid_samples_instead_of_decoding_them() {
        let slot = SlotConfig::new(Target::Secondary, 0x02)
            .with(ServiceId::Locator)
            .unwrap();

        // 0x12: status nibble 0x10 (invalid), token 2.
        let payload = vec![0x12, 0, 0, 0, 0, 0, 0, 0, 0];
        assert_eq!(
            decode_notification(&slot, &payload).unwrap(),
            StreamNotification::Invalid { token: 2 }
        );
    }

    #[test]
    fn short_payloads_error_rather_than_panic() {
        let slot = SlotConfig::new(Target::Secondary, 0x02)
            .with(ServiceId::Locator)
            .unwrap();

        assert!(matches!(
            decode_notification(&slot, &[0x02, 0x00, 0x00]),
            Err(Error::ShortPayload {
                expected: 8,
                actual: 2
            })
        ));
        assert!(matches!(
            decode_notification(&slot, &[]),
            Err(Error::ShortPayload {
                expected: 1,
                actual: 0
            })
        ));
    }

    #[test]
    fn reads_the_token_from_a_notification() {
        assert_eq!(notification_token(&[0x02, 0xAA]), Some(2));
        assert_eq!(notification_token(&[0x13]), Some(3)); // invalid status, token 3
        assert_eq!(notification_token(&[]), None);
    }
}
