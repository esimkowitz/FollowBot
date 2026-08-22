//! Packet framing for the Sphero RVR API.
//!
//! Wire format, everything between SOP and EOP escaped:
//!
//! ```text
//! SOP | flags | [target] | [source] | did | cid | seq | [err] | payload | checksum | EOP
//! ```
//!
//! `target`/`source` are present only when the corresponding flag bit is set, and
//! `err` only on responses (`Flags::IS_RESPONSE`). Getting that last one wrong
//! shifts every subsequent field by one byte, so decoding always inspects the
//! flags before touching the payload.

use crate::error::{Error, Result};
use crate::ids::ErrorCode;

pub const SOP: u8 = 0x8D;
pub const EOP: u8 = 0xD8;
pub const ESC: u8 = 0xAB;

pub const ESCAPED_SOP: u8 = 0x05;
pub const ESCAPED_EOP: u8 = 0x50;
pub const ESCAPED_ESC: u8 = 0x23;

/// Header flag bits.
///
/// Kept as plain constants rather than a bitflags dependency; the set is small
/// and fixed by the protocol.
pub mod flags {
    /// Packet is a response to a request. Implies an `err` byte after `seq`.
    pub const IS_RESPONSE: u8 = 0x01;
    /// Ask the robot to respond, success or failure.
    pub const REQUESTS_RESPONSE: u8 = 0x02;
    /// Ask the robot to respond only on error.
    pub const REQUESTS_RESPONSE_IF_ERROR: u8 = 0x04;
    /// Marks link activity, deferring the robot's inactivity sleep.
    pub const IS_ACTIVITY: u8 = 0x08;
    pub const HAS_TARGET: u8 = 0x10;
    pub const HAS_SOURCE: u8 = 0x20;
    pub const HAS_MORE_FLAGS: u8 = 0x80;
}

/// Flags for a normal outgoing command that expects a reply.
pub const DEFAULT_REQUEST_FLAGS: u8 =
    flags::REQUESTS_RESPONSE | flags::IS_ACTIVITY | flags::HAS_TARGET | flags::HAS_SOURCE;

/// Flags for fire-and-forget commands (drive commands at rate, for instance).
pub const NO_RESPONSE_FLAGS: u8 = flags::IS_ACTIVITY | flags::HAS_TARGET | flags::HAS_SOURCE;

/// The host's address on the bus.
pub const HOST_SOURCE: u8 = 0x00;

/// A decoded (or to-be-encoded) API packet.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Packet {
    pub flags: u8,
    pub target: Option<u8>,
    pub source: Option<u8>,
    pub device_id: u8,
    pub command_id: u8,
    pub sequence: u8,
    /// Present only on responses.
    pub error: Option<ErrorCode>,
    pub payload: Vec<u8>,
}

impl Packet {
    /// Build a command addressed to `target`, expecting a response.
    pub fn command(
        target: u8,
        device_id: u8,
        command_id: u8,
        sequence: u8,
        payload: Vec<u8>,
    ) -> Self {
        Self {
            flags: DEFAULT_REQUEST_FLAGS,
            target: Some(target),
            source: Some(HOST_SOURCE),
            device_id,
            command_id,
            sequence,
            error: None,
            payload,
        }
    }

    /// Build a command that suppresses the response packet.
    pub fn command_no_response(
        target: u8,
        device_id: u8,
        command_id: u8,
        sequence: u8,
        payload: Vec<u8>,
    ) -> Self {
        Self {
            flags: NO_RESPONSE_FLAGS,
            ..Self::command(target, device_id, command_id, sequence, payload)
        }
    }

    pub fn is_response(&self) -> bool {
        self.flags & flags::IS_RESPONSE != 0
    }

    /// True when this arrived unsolicited — streaming data or an async notification.
    pub fn is_notification(&self) -> bool {
        !self.is_response()
    }

    /// Header and payload without framing, in wire order. This is the byte range
    /// the checksum covers.
    fn body(&self) -> Vec<u8> {
        let mut body = Vec::with_capacity(8 + self.payload.len());
        body.push(self.flags);
        if let Some(target) = self.target {
            body.push(target);
        }
        if let Some(source) = self.source {
            body.push(source);
        }
        body.push(self.device_id);
        body.push(self.command_id);
        body.push(self.sequence);
        if let Some(error) = self.error {
            body.push(error as u8);
        }
        body.extend_from_slice(&self.payload);
        body
    }

    /// Serialize to an escaped, framed byte string ready for the wire.
    pub fn encode(&self) -> Vec<u8> {
        let body = self.body();
        let checksum = checksum(&body);

        let mut out = Vec::with_capacity(body.len() + 8);
        out.push(SOP);
        for &byte in body.iter().chain(std::iter::once(&checksum)) {
            escape_into(&mut out, byte);
        }
        out.push(EOP);
        out
    }

    /// Parse an unescaped body (everything between SOP and EOP, checksum included).
    pub fn decode_body(body: &[u8]) -> Result<Self> {
        // Shortest legal packet: flags, did, cid, seq, checksum.
        if body.len() < 5 {
            return Err(Error::Truncated { len: body.len() });
        }

        let (content, &[received]) = body.split_at(body.len() - 1) else {
            unreachable!("split_at leaves exactly one trailing byte")
        };
        let expected = checksum(content);
        if received != expected {
            return Err(Error::Checksum { expected, received });
        }

        let mut cur = Cursor::new(content);
        let flags = cur.u8()?;
        if flags & flags::HAS_MORE_FLAGS != 0 {
            // The extended-flags byte is defined but unused by RVR firmware; the
            // official SDK refuses these too rather than guessing the layout.
            return Err(Error::ExtendedFlagsUnsupported);
        }

        let target = (flags & flags::HAS_TARGET != 0)
            .then(|| cur.u8())
            .transpose()?;
        let source = (flags & flags::HAS_SOURCE != 0)
            .then(|| cur.u8())
            .transpose()?;
        let device_id = cur.u8()?;
        let command_id = cur.u8()?;
        let sequence = cur.u8()?;
        let error = (flags & flags::IS_RESPONSE != 0)
            .then(|| cur.u8().map(ErrorCode::from_byte))
            .transpose()?;

        Ok(Self {
            flags,
            target,
            source,
            device_id,
            command_id,
            sequence,
            error,
            payload: cur.rest().to_vec(),
        })
    }
}

/// Sum every byte, then invert: `!(sum & 0xFF)`.
pub fn checksum(bytes: &[u8]) -> u8 {
    !bytes.iter().fold(0u8, |acc, &b| acc.wrapping_add(b))
}

fn escape_into(out: &mut Vec<u8>, byte: u8) {
    match byte {
        SOP => out.extend_from_slice(&[ESC, ESCAPED_SOP]),
        EOP => out.extend_from_slice(&[ESC, ESCAPED_EOP]),
        ESC => out.extend_from_slice(&[ESC, ESCAPED_ESC]),
        _ => out.push(byte),
    }
}

/// Reverse the escaping applied by [`escape_into`].
pub fn unescape(bytes: &[u8]) -> Result<Vec<u8>> {
    let mut out = Vec::with_capacity(bytes.len());
    let mut iter = bytes.iter().copied();
    while let Some(byte) = iter.next() {
        if byte != ESC {
            out.push(byte);
            continue;
        }
        match iter.next() {
            Some(ESCAPED_SOP) => out.push(SOP),
            Some(ESCAPED_EOP) => out.push(EOP),
            Some(ESCAPED_ESC) => out.push(ESC),
            Some(other) => return Err(Error::BadEscape(other)),
            None => return Err(Error::DanglingEscape),
        }
    }
    Ok(out)
}

/// Incremental framer: feed it bytes off the serial port, take whole packets out.
///
/// Bytes before the first SOP are discarded, which lets the reader recover from a
/// mid-packet start or line noise without special handling by the caller.
#[derive(Debug, Default)]
pub struct PacketReader {
    buf: Vec<u8>,
    in_packet: bool,
}

impl PacketReader {
    pub fn new() -> Self {
        Self::default()
    }

    /// Feed raw bytes; returns every packet completed by this chunk.
    ///
    /// A packet that fails to decode (bad checksum, truncated) yields an `Err`
    /// entry rather than aborting: one corrupt frame should not desynchronize the
    /// stream or hide the frames that follow it.
    pub fn push(&mut self, bytes: &[u8]) -> Vec<Result<Packet>> {
        let mut packets = Vec::new();
        for &byte in bytes {
            match byte {
                SOP => {
                    // A second SOP means the previous frame never terminated; drop it.
                    self.buf.clear();
                    self.in_packet = true;
                }
                EOP if self.in_packet => {
                    self.in_packet = false;
                    let frame = std::mem::take(&mut self.buf);
                    packets.push(unescape(&frame).and_then(|body| Packet::decode_body(&body)));
                }
                _ if self.in_packet => self.buf.push(byte),
                // Outside a frame: line noise, ignore.
                _ => {}
            }
        }
        packets
    }
}

/// Bounds-checked reader over a packet body.
struct Cursor<'a> {
    bytes: &'a [u8],
    pos: usize,
}

impl<'a> Cursor<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, pos: 0 }
    }

    fn u8(&mut self) -> Result<u8> {
        let byte = *self.bytes.get(self.pos).ok_or(Error::Truncated {
            len: self.bytes.len(),
        })?;
        self.pos += 1;
        Ok(byte)
    }

    fn rest(&self) -> &'a [u8] {
        &self.bytes[self.pos.min(self.bytes.len())..]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::{DeviceId, Target};

    #[test]
    fn checksum_matches_sdk_definition() {
        // Sum 0x01+0x02+0x03 = 0x06, inverted = 0xF9.
        assert_eq!(checksum(&[0x01, 0x02, 0x03]), 0xF9);
        assert_eq!(checksum(&[]), 0xFF);
        // Wrapping: 0xFF + 0x01 = 0x00, inverted = 0xFF.
        assert_eq!(checksum(&[0xFF, 0x01]), 0xFF);
    }

    #[test]
    fn checksum_validates_by_summing_to_ff() {
        let body = [0x3A, 0x02, 0x00, 0x16, 0x32, 0x01];
        let sum = body
            .iter()
            .chain(std::iter::once(&checksum(&body)))
            .fold(0u8, |a, &b| a.wrapping_add(b));
        assert_eq!(sum, 0xFF);
    }

    #[test]
    fn frames_are_delimited_by_sop_and_eop() {
        let packet = Packet::command(
            Target::Secondary as u8,
            DeviceId::Drive as u8,
            0x42,
            1,
            vec![],
        );
        let encoded = packet.encode();
        assert_eq!(encoded[0], SOP);
        assert_eq!(*encoded.last().unwrap(), EOP);
    }

    #[test]
    fn round_trips_through_encode_and_decode() {
        let original = Packet::command(
            Target::Secondary as u8,
            DeviceId::Drive as u8,
            0x32,
            7,
            vec![0x3F, 0x80, 0x00, 0x00, 0xBF, 0x80, 0x00, 0x00],
        );
        let encoded = original.encode();
        let body = unescape(&encoded[1..encoded.len() - 1]).unwrap();
        assert_eq!(Packet::decode_body(&body).unwrap(), original);
    }

    #[test]
    fn escapes_delimiter_bytes_appearing_in_payload() {
        // A payload containing all three special bytes must survive the round trip,
        // and the framed form must not contain a bare SOP/EOP after the delimiters.
        let packet = Packet::command(
            Target::Primary as u8,
            DeviceId::Io as u8,
            0x1A,
            3,
            vec![SOP, EOP, ESC, 0x00],
        );
        let encoded = packet.encode();

        let interior = &encoded[1..encoded.len() - 1];
        assert!(!interior.contains(&SOP), "raw SOP leaked into frame body");
        assert!(!interior.contains(&EOP), "raw EOP leaked into frame body");

        let body = unescape(interior).unwrap();
        assert_eq!(
            Packet::decode_body(&body).unwrap().payload,
            vec![SOP, EOP, ESC, 0x00]
        );
    }

    #[test]
    fn escape_sequences_use_the_documented_substitutes() {
        assert_eq!(unescape(&[ESC, ESCAPED_SOP]).unwrap(), vec![SOP]);
        assert_eq!(unescape(&[ESC, ESCAPED_EOP]).unwrap(), vec![EOP]);
        assert_eq!(unescape(&[ESC, ESCAPED_ESC]).unwrap(), vec![ESC]);
    }

    #[test]
    fn rejects_invalid_escape_sequences() {
        assert!(matches!(
            unescape(&[ESC, 0x99]),
            Err(Error::BadEscape(0x99))
        ));
        assert!(matches!(unescape(&[ESC]), Err(Error::DanglingEscape)));
    }

    #[test]
    fn responses_carry_an_error_byte_notifications_do_not() {
        // Identical bytes, differing only in the response flag. That one bit moves
        // the payload boundary by a byte, which is the trap this guards against:
        // did=0x11, cid=0x00, seq=0x01, then either [err=0x05, payload=0xAA]
        // or [payload=0x05, 0xAA].
        let tail = [0x11u8, 0x00, 0x01, 0x05, 0xAA];

        let response_body: Vec<u8> = std::iter::once(flags::IS_RESPONSE).chain(tail).collect();
        let response = Packet::decode_body(&with_checksum(&response_body)).unwrap();
        assert!(response.is_response());
        assert_eq!(response.error, Some(ErrorCode::BadDataLength));
        assert_eq!(response.payload, vec![0xAA]);

        let notify_body: Vec<u8> = std::iter::once(0x00u8).chain(tail).collect();
        let notify = Packet::decode_body(&with_checksum(&notify_body)).unwrap();
        assert_eq!(notify.error, None);
        assert_eq!(notify.payload, vec![0x05, 0xAA]);
        assert!(notify.is_notification());
    }

    #[test]
    fn rejects_a_corrupted_checksum() {
        let packet = Packet::command(
            Target::Secondary as u8,
            DeviceId::Drive as u8,
            0x42,
            1,
            vec![],
        );
        let encoded = packet.encode();
        let mut body = unescape(&encoded[1..encoded.len() - 1]).unwrap();
        *body.last_mut().unwrap() ^= 0xFF;
        assert!(matches!(
            Packet::decode_body(&body),
            Err(Error::Checksum { .. })
        ));
    }

    #[test]
    fn rejects_truncated_bodies_without_panicking() {
        assert!(matches!(
            Packet::decode_body(&[]),
            Err(Error::Truncated { .. })
        ));
        assert!(matches!(
            Packet::decode_body(&[0x3A, 0x02]),
            Err(Error::Truncated { .. })
        ));
        // Flags claim target+source but the body ends before the ids arrive.
        let body = [DEFAULT_REQUEST_FLAGS, 0x02, 0x00, 0x16];
        assert!(matches!(
            Packet::decode_body(&with_checksum(&body)),
            Err(Error::Truncated { .. })
        ));
    }

    #[test]
    fn rejects_extended_flags() {
        let body = [flags::HAS_MORE_FLAGS, 0x00, 0x11, 0x00, 0x01];
        assert!(matches!(
            Packet::decode_body(&with_checksum(&body)),
            Err(Error::ExtendedFlagsUnsupported)
        ));
    }

    #[test]
    fn reader_extracts_packets_split_across_chunks() {
        let packet = Packet::command(
            Target::Secondary as u8,
            DeviceId::Drive as u8,
            0x32,
            9,
            vec![1, 2, 3],
        );
        let encoded = packet.encode();
        let (head, tail) = encoded.split_at(4);

        let mut reader = PacketReader::new();
        assert!(reader.push(head).is_empty(), "no packet before EOP arrives");

        let packets = reader.push(tail);
        assert_eq!(packets.len(), 1);
        assert_eq!(packets.into_iter().next().unwrap().unwrap(), packet);
    }

    #[test]
    fn reader_skips_noise_and_recovers_after_a_bad_frame() {
        let good = Packet::command(
            Target::Secondary as u8,
            DeviceId::Drive as u8,
            0x42,
            2,
            vec![],
        );

        let mut stream = vec![0x11, 0x22]; // leading noise, before any SOP
        stream.extend_from_slice(&[SOP, 0x01, 0x02, EOP]); // too short to decode
        stream.extend_from_slice(&good.encode());

        let results = PacketReader::new().push(&stream);
        assert_eq!(results.len(), 2);
        assert!(
            results[0].is_err(),
            "short frame should surface as an error"
        );
        assert_eq!(*results[1].as_ref().unwrap(), good, "reader recovered");
    }

    #[test]
    fn reader_abandons_a_frame_interrupted_by_a_new_sop() {
        let good = Packet::command(
            Target::Primary as u8,
            DeviceId::Power as u8,
            0x10,
            4,
            vec![],
        );

        let mut stream = vec![SOP, 0x3A, 0x01]; // truncated frame, no EOP
        stream.extend_from_slice(&good.encode());

        let results = PacketReader::new().push(&stream);
        assert_eq!(results.len(), 1);
        assert_eq!(*results[0].as_ref().unwrap(), good);
    }

    /// Append the checksum so a hand-written body decodes.
    fn with_checksum(body: &[u8]) -> Vec<u8> {
        let mut out = body.to_vec();
        out.push(checksum(body));
        out
    }
}
