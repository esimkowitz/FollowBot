//! Byte transports for the RVR link.
//!
//! Abstracting the serial port lets the whole protocol stack run against an
//! in-memory fake, so the codec and client logic are testable on a development
//! machine with no robot attached.

use crate::error::Result;
use std::sync::{Arc, Mutex};

/// A bidirectional byte stream to the robot.
///
/// `read` should block for at most the transport's timeout and return `Ok(0)` on
/// timeout rather than erroring — the RX loop polls continuously and a quiet line
/// is normal.
pub trait Transport: Send {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize>;
    fn write_all(&mut self, bytes: &[u8]) -> Result<()>;
    /// Clone a handle to the same underlying stream, so the RX loop can read on
    /// one thread while commands are written from another.
    fn try_clone(&self) -> Result<Box<dyn Transport>>;
}

#[cfg(feature = "serial")]
mod serial {
    use super::*;
    use std::io::{Read, Write};
    use std::time::Duration;

    /// A real UART connection.
    pub struct SerialTransport {
        port: Box<dyn serialport::SerialPort>,
    }

    impl SerialTransport {
        /// Open `path` at `baud`, 8N1, no flow control — the RVR's expected setup.
        pub fn open(path: &str, baud: u32) -> Result<Self> {
            let port = serialport::new(path, baud)
                .data_bits(serialport::DataBits::Eight)
                .parity(serialport::Parity::None)
                .stop_bits(serialport::StopBits::One)
                .flow_control(serialport::FlowControl::None)
                // Short enough that the RX loop stays responsive to shutdown.
                .timeout(Duration::from_millis(50))
                .open()
                .map_err(std::io::Error::other)?;
            Ok(Self { port })
        }
    }

    impl Transport for SerialTransport {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize> {
            match Read::read(&mut self.port, buf) {
                Ok(n) => Ok(n),
                // A quiet line is normal, not an error.
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(0),
                Err(e) => Err(e.into()),
            }
        }

        fn write_all(&mut self, bytes: &[u8]) -> Result<()> {
            Write::write_all(&mut self.port, bytes)?;
            Write::flush(&mut self.port)?;
            Ok(())
        }

        fn try_clone(&self) -> Result<Box<dyn Transport>> {
            let port = self.port.try_clone().map_err(std::io::Error::other)?;
            Ok(Box::new(Self { port }))
        }
    }
}

#[cfg(feature = "serial")]
pub use serial::SerialTransport;

/// Buffers behind a [`MockTransport`].
#[derive(Debug, Default)]
struct MockState {
    /// Every byte the client has written.
    written: Vec<u8>,
    /// Bytes queued for the client to read.
    to_read: std::collections::VecDeque<u8>,
}

/// An in-memory transport for tests: records what was written and replays what
/// you queue.
#[derive(Clone, Default)]
pub struct MockTransport {
    state: Arc<Mutex<MockState>>,
}

impl MockTransport {
    pub fn new() -> Self {
        Self::default()
    }

    /// Queue bytes for the client to read, as though the robot had sent them.
    pub fn push_to_read(&self, bytes: &[u8]) {
        self.state
            .lock()
            .unwrap()
            .to_read
            .extend(bytes.iter().copied());
    }

    /// Everything written so far.
    pub fn written(&self) -> Vec<u8> {
        self.state.lock().unwrap().written.clone()
    }

    /// Take everything written so far, clearing the buffer.
    pub fn take_written(&self) -> Vec<u8> {
        std::mem::take(&mut self.state.lock().unwrap().written)
    }
}

impl Transport for MockTransport {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize> {
        let mut state = self.state.lock().unwrap();
        let n = buf.len().min(state.to_read.len());
        for slot in buf.iter_mut().take(n) {
            *slot = state.to_read.pop_front().expect("checked length above");
        }
        Ok(n)
    }

    fn write_all(&mut self, bytes: &[u8]) -> Result<()> {
        self.state.lock().unwrap().written.extend_from_slice(bytes);
        Ok(())
    }

    fn try_clone(&self) -> Result<Box<dyn Transport>> {
        Ok(Box::new(self.clone()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mock_records_writes() {
        let mut transport = MockTransport::new();
        transport.write_all(&[1, 2, 3]).unwrap();
        transport.write_all(&[4]).unwrap();
        assert_eq!(transport.written(), vec![1, 2, 3, 4]);
        assert_eq!(transport.take_written(), vec![1, 2, 3, 4]);
        assert!(transport.written().is_empty());
    }

    #[test]
    fn mock_replays_queued_reads_and_reports_empty_without_erroring() {
        let mut transport = MockTransport::new();
        transport.push_to_read(&[0xAA, 0xBB]);

        let mut buf = [0u8; 8];
        assert_eq!(transport.read(&mut buf).unwrap(), 2);
        assert_eq!(&buf[..2], &[0xAA, 0xBB]);
        // Drained: a quiet line reads zero bytes rather than failing.
        assert_eq!(transport.read(&mut buf).unwrap(), 0);
    }

    #[test]
    fn clones_share_one_underlying_stream() {
        let transport = MockTransport::new();
        let mut clone = transport.try_clone().unwrap();
        clone.write_all(&[0x42]).unwrap();
        assert_eq!(
            transport.written(),
            vec![0x42],
            "clone writes to the same buffer"
        );
    }
}
