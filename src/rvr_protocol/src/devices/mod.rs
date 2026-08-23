//! Command builders, one module per device ID.
//!
//! Each module exposes its command ids under `cid`, builders returning the
//! command's payload bytes, and parsers for the responses it can produce.
//! Modules whose commands all reach the same processor also expose a `TARGET`
//! const; `sensor` routes per command (see [`sensor::target_for`]) and
//! `system_info`/`api_shell` answer on either processor, so the caller chooses.
//!
//! Keeping these free of transport concerns makes the encoding directly testable.

pub mod api_shell;
pub mod drive;
pub mod io;
pub mod power;
pub mod sensor;
pub mod system_info;

/// Take a fixed-width big-endian field at `offset`, or report how short the
/// payload was.
///
/// Parsers use this so the expected width is stated once, next to the type it
/// decodes into, rather than repeated at each call site.
pub(crate) fn be_bytes<const N: usize>(
    payload: &[u8],
    offset: usize,
) -> crate::error::Result<[u8; N]> {
    payload
        .get(offset..offset + N)
        .and_then(|slice| slice.try_into().ok())
        .ok_or(crate::error::Error::ShortPayload {
            expected: offset + N,
            actual: payload.len(),
        })
}
