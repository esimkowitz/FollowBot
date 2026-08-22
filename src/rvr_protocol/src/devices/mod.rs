//! Command builders, one module per device ID.
//!
//! Each function returns a payload plus the target processor and command id; the
//! client wraps them in packets. Keeping them free of transport concerns makes
//! the encoding directly testable.

pub mod api_shell;
pub mod drive;
pub mod io;
pub mod power;
pub mod sensor;
pub mod system_info;
