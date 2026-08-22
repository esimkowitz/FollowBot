//! The RVR connection: sequencing, response matching, and notification dispatch.
//!
//! One serial line carries three interleaved conversations — command responses,
//! async notifications (motor stall, battery state), and streaming sensor data at
//! up to 30 Hz. A background thread owns the port and sorts arriving packets into
//! those categories, so sending a drive command never blocks on sensor traffic.

use crate::devices::{drive, power, sensor, system_info};
use crate::error::{Error, Result};
use crate::ids::{DeviceId, Target};
use crate::packet::{Packet, PacketReader};
use crate::streaming::{
    decode_notification, notification_token, SlotConfig, StreamNotification, StreamSample,
};
use crate::transport::Transport;

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::mpsc::{channel, Receiver, RecvTimeoutError, Sender};
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// How long to wait for a command response before giving up.
pub const DEFAULT_TIMEOUT: Duration = Duration::from_secs(2);

/// An unsolicited message from the robot.
#[derive(Debug, Clone, PartialEq)]
pub enum Notification {
    /// Decoded sensor samples, in the order their slot was configured.
    Stream {
        processor: Target,
        token: u8,
        samples: Vec<StreamSample>,
    },
    /// The robot flagged a streaming sample as invalid; it carries no data.
    StreamInvalid {
        processor: Target,
        token: u8,
    },
    MotorStall(drive::MotorStall),
    MotorFault {
        faulted: bool,
    },
    BatteryVoltageState(power::BatteryVoltageState),
    WillSleep,
    DidSleep,
    /// Anything not recognized above, surfaced rather than dropped.
    Other(Packet),
}

/// Callback invoked on the RX thread for each notification.
pub type NotificationHandler = Box<dyn FnMut(Notification) + Send>;

/// Identifies an in-flight request: `(device_id, command_id, sequence)`.
type RequestKey = (u8, u8, u8);

/// Requests awaiting a response.
type PendingMap = HashMap<RequestKey, Sender<Result<Packet>>>;

/// Shared between the client handle and its RX thread.
struct Shared {
    pending: Mutex<PendingMap>,
    /// Slot layouts, needed to decode streaming blobs.
    slots: Mutex<HashMap<(Target, u8), SlotConfig>>,
    handler: Mutex<Option<NotificationHandler>>,
    running: AtomicBool,
}

/// A connection to the robot.
pub struct RvrClient {
    shared: Arc<Shared>,
    writer: Mutex<Box<dyn Transport>>,
    sequence: AtomicU8,
    timeout: Duration,
    rx_thread: Option<std::thread::JoinHandle<()>>,
}

impl RvrClient {
    /// Take ownership of `transport` and start the receive loop.
    pub fn new(transport: Box<dyn Transport>) -> Result<Self> {
        let reader = transport.try_clone()?;
        let shared = Arc::new(Shared {
            pending: Mutex::new(HashMap::new()),
            slots: Mutex::new(HashMap::new()),
            handler: Mutex::new(None),
            running: AtomicBool::new(true),
        });

        let rx_shared = Arc::clone(&shared);
        let rx_thread = std::thread::Builder::new()
            .name("rvr-rx".into())
            .spawn(move || receive_loop(reader, rx_shared))?;

        Ok(Self {
            shared,
            writer: Mutex::new(transport),
            // The firmware's generator pre-increments, so the first id sent is 1.
            sequence: AtomicU8::new(0),
            timeout: DEFAULT_TIMEOUT,
            rx_thread: Some(rx_thread),
        })
    }

    pub fn with_timeout(mut self, timeout: Duration) -> Self {
        self.timeout = timeout;
        self
    }

    /// Register a callback for notifications. Runs on the RX thread, so it should
    /// hand work off rather than block.
    pub fn on_notification(&self, handler: NotificationHandler) {
        *self.shared.handler.lock().unwrap() = Some(handler);
    }

    fn next_sequence(&self) -> u8 {
        self.sequence
            .fetch_add(1, Ordering::Relaxed)
            .wrapping_add(1)
    }

    /// Send a command and wait for its response.
    pub fn request(
        &self,
        target: Target,
        device: DeviceId,
        command: u8,
        payload: Vec<u8>,
    ) -> Result<Packet> {
        let seq = self.next_sequence();
        let packet = Packet::command(target as u8, device as u8, command, seq, payload);

        let (tx, rx) = channel();
        let key = (device as u8, command, seq);
        self.shared.pending.lock().unwrap().insert(key, tx);

        // Drop the pending entry if the write itself fails, so it can't leak.
        if let Err(e) = self.write_packet(&packet) {
            self.shared.pending.lock().unwrap().remove(&key);
            return Err(e);
        }

        let result = match rx.recv_timeout(self.timeout) {
            Ok(response) => response,
            Err(RecvTimeoutError::Timeout) => {
                self.shared.pending.lock().unwrap().remove(&key);
                Err(Error::Timeout {
                    device_id: device as u8,
                    command_id: command,
                })
            }
            Err(RecvTimeoutError::Disconnected) => Err(Error::Disconnected),
        };

        let response = result?;
        match response.error {
            Some(code) if !code.is_success() => Err(Error::Robot(code)),
            _ => Ok(response),
        }
    }

    /// Send a command without waiting for a response. Used for drive commands
    /// issued at rate, where a reply per command would be pure overhead.
    pub fn send(
        &self,
        target: Target,
        device: DeviceId,
        command: u8,
        payload: Vec<u8>,
    ) -> Result<()> {
        let seq = self.next_sequence();
        self.write_packet(&Packet::command_no_response(
            target as u8,
            device as u8,
            command,
            seq,
            payload,
        ))
    }

    fn write_packet(&self, packet: &Packet) -> Result<()> {
        self.writer.lock().unwrap().write_all(&packet.encode())
    }

    // ---- Power ----------------------------------------------------------

    pub fn wake(&self) -> Result<()> {
        self.request(
            power::TARGET,
            power::DEVICE,
            power::cid::WAKE,
            power::wake(),
        )?;
        Ok(())
    }

    pub fn sleep(&self) -> Result<()> {
        self.request(
            power::TARGET,
            power::DEVICE,
            power::cid::SLEEP,
            power::sleep(),
        )?;
        Ok(())
    }

    pub fn battery_percentage(&self) -> Result<u8> {
        let response = self.request(
            power::TARGET,
            power::DEVICE,
            power::cid::GET_BATTERY_PERCENTAGE,
            power::get_battery_percentage(),
        )?;
        power::parse_battery_percentage(&response.payload).ok_or(Error::ShortPayload {
            expected: 1,
            actual: response.payload.len(),
        })
    }

    pub fn battery_voltage(&self) -> Result<f32> {
        let response = self.request(
            power::TARGET,
            power::DEVICE,
            power::cid::GET_BATTERY_VOLTAGE_IN_VOLTS,
            power::get_battery_voltage_in_volts(power::VoltageReadingType::CalibratedAndFiltered),
        )?;
        power::parse_voltage(&response.payload).ok_or(Error::ShortPayload {
            expected: 4,
            actual: response.payload.len(),
        })
    }

    pub fn enable_battery_notifications(&self, enabled: bool) -> Result<()> {
        self.request(
            power::TARGET,
            power::DEVICE,
            power::cid::ENABLE_BATTERY_VOLTAGE_STATE_CHANGE_NOTIFY,
            power::enable_battery_voltage_state_change_notify(enabled),
        )?;
        Ok(())
    }

    // ---- Drive ----------------------------------------------------------

    /// Closed-loop per-wheel velocity in m/s. Fire-and-forget: at control rate a
    /// response per command would add latency without adding information.
    pub fn drive_tank_si(&self, left_mps: f32, right_mps: f32) -> Result<()> {
        self.send(
            drive::TARGET,
            drive::DEVICE,
            drive::cid::DRIVE_TANK_SI_UNITS,
            drive::drive_tank_si_units(left_mps, right_mps),
        )
    }

    pub fn drive_stop(&self) -> Result<()> {
        self.send(
            drive::TARGET,
            drive::DEVICE,
            drive::cid::DRIVE_STOP,
            drive::drive_stop(),
        )
    }

    pub fn reset_yaw(&self) -> Result<()> {
        self.request(
            drive::TARGET,
            drive::DEVICE,
            drive::cid::RESET_YAW,
            drive::reset_yaw(),
        )?;
        Ok(())
    }

    pub fn set_control_system_timeout(&self, timeout_ms: u16) -> Result<()> {
        self.request(
            drive::TARGET,
            drive::DEVICE,
            drive::cid::SET_CUSTOM_CONTROL_SYSTEM_TIMEOUT,
            drive::set_custom_control_system_timeout(timeout_ms),
        )?;
        Ok(())
    }

    pub fn enable_motor_stall_notifications(&self, enabled: bool) -> Result<()> {
        self.request(
            drive::TARGET,
            drive::DEVICE,
            drive::cid::ENABLE_MOTOR_STALL_NOTIFY,
            drive::enable_motor_stall_notify(enabled),
        )?;
        Ok(())
    }

    // ---- Sensors --------------------------------------------------------

    pub fn reset_locator(&self) -> Result<()> {
        self.request(
            Target::Secondary,
            sensor::DEVICE,
            sensor::cid::RESET_LOCATOR_X_AND_Y,
            sensor::reset_locator_x_and_y(),
        )?;
        Ok(())
    }

    pub fn encoder_counts(&self) -> Result<(i32, i32)> {
        let response = self.request(
            Target::Secondary,
            sensor::DEVICE,
            sensor::cid::GET_ENCODER_COUNTS,
            sensor::get_encoder_counts(),
        )?;
        sensor::parse_encoder_counts(&response.payload).ok_or(Error::ShortPayload {
            expected: 8,
            actual: response.payload.len(),
        })
    }

    /// Configure a streaming slot and remember its layout for decoding.
    ///
    /// The layout must be registered before the robot starts sending, since the
    /// blobs carry no type information.
    pub fn configure_streaming(&self, slot: &SlotConfig) -> Result<()> {
        self.shared
            .slots
            .lock()
            .unwrap()
            .insert((slot.processor, slot.token), slot.clone());

        self.request(
            slot.processor,
            sensor::DEVICE,
            sensor::cid::CONFIGURE_STREAMING_SERVICE,
            sensor::configure_streaming_service(slot),
        )?;
        Ok(())
    }

    /// Begin streaming on `processor` at `period_ms` (clamped to the 33 ms floor).
    pub fn start_streaming(&self, processor: Target, period_ms: u16) -> Result<()> {
        self.request(
            processor,
            sensor::DEVICE,
            sensor::cid::START_STREAMING_SERVICE,
            sensor::start_streaming_service(period_ms),
        )?;
        Ok(())
    }

    pub fn stop_streaming(&self, processor: Target) -> Result<()> {
        self.request(
            processor,
            sensor::DEVICE,
            sensor::cid::STOP_STREAMING_SERVICE,
            sensor::stop_streaming_service(),
        )?;
        Ok(())
    }

    /// Stop and clear streaming, forgetting the slot layouts.
    ///
    /// Always clear before reconfiguring: stale slot definitions on the robot
    /// produce blobs that no longer match what the host expects.
    pub fn clear_streaming(&self, processor: Target) -> Result<()> {
        self.stop_streaming(processor)?;
        self.request(
            processor,
            sensor::DEVICE,
            sensor::cid::CLEAR_STREAMING_SERVICE,
            sensor::clear_streaming_service(),
        )?;
        self.shared
            .slots
            .lock()
            .unwrap()
            .retain(|&(p, _), _| p != processor);
        Ok(())
    }

    // ---- System ---------------------------------------------------------

    /// Read the main application firmware version — a cheap link check.
    pub fn firmware_version(&self, target: Target) -> Result<system_info::Version> {
        let response = self.request(
            target,
            system_info::DEVICE,
            system_info::cid::GET_MAIN_APPLICATION_VERSION,
            system_info::get_main_application_version(),
        )?;
        system_info::parse_version(&response.payload).ok_or(Error::ShortPayload {
            expected: 6,
            actual: response.payload.len(),
        })
    }
}

impl Drop for RvrClient {
    fn drop(&mut self) {
        self.shared.running.store(false, Ordering::Relaxed);
        if let Some(handle) = self.rx_thread.take() {
            let _ = handle.join();
        }
    }
}

/// Read packets until the client is dropped, routing each to its destination.
fn receive_loop(mut reader: Box<dyn Transport>, shared: Arc<Shared>) {
    let mut framer = PacketReader::new();
    let mut buf = [0u8; 512];

    while shared.running.load(Ordering::Relaxed) {
        let n = match reader.read(&mut buf) {
            Ok(0) => {
                // Quiet line: yield rather than spin.
                std::thread::sleep(Duration::from_millis(2));
                continue;
            }
            Ok(n) => n,
            // A read error shouldn't kill the loop; the link may recover.
            Err(_) => {
                std::thread::sleep(Duration::from_millis(10));
                continue;
            }
        };

        for result in framer.push(&buf[..n]) {
            match result {
                Ok(packet) => route(&shared, packet),
                // A corrupt frame is expected occasionally on a noisy line; the
                // framer has already resynchronized.
                Err(_) => continue,
            }
        }
    }
}

/// Deliver a packet to whoever is waiting for it.
fn route(shared: &Arc<Shared>, packet: Packet) {
    if packet.is_response() {
        let key = (packet.device_id, packet.command_id, packet.sequence);
        if let Some(tx) = shared.pending.lock().unwrap().remove(&key) {
            let _ = tx.send(Ok(packet));
            return;
        }
        // Late response after a timeout: nothing to do with it.
        return;
    }

    if let Some(notification) = classify(shared, &packet) {
        if let Some(handler) = shared.handler.lock().unwrap().as_mut() {
            handler(notification);
        }
    }
}

/// Turn an unsolicited packet into a typed notification.
fn classify(shared: &Arc<Shared>, packet: &Packet) -> Option<Notification> {
    let processor = packet.source.and_then(Target::from_byte);

    match (packet.device_id, packet.command_id) {
        (id, cid)
            if id == DeviceId::Sensor as u8
                && cid == sensor::cid::STREAMING_SERVICE_DATA_NOTIFY =>
        {
            // Slots are per-processor: a Nordic slot 1 and an ST slot 1 both
            // stream under token 1, so the source byte is what disambiguates them.
            let processor = processor?;
            let token = notification_token(&packet.payload)?;
            let slot = shared
                .slots
                .lock()
                .unwrap()
                .get(&(processor, token))
                .cloned()?;

            match decode_notification(&slot, &packet.payload) {
                Ok(StreamNotification::Samples(samples)) => Some(Notification::Stream {
                    processor,
                    token,
                    samples,
                }),
                Ok(StreamNotification::Invalid { token }) => {
                    Some(Notification::StreamInvalid { processor, token })
                }
                Err(_) => None,
            }
        }
        (id, cid) if id == DeviceId::Drive as u8 && cid == drive::cid::MOTOR_STALL_NOTIFY => {
            drive::parse_motor_stall_notify(&packet.payload).map(Notification::MotorStall)
        }
        (id, cid) if id == DeviceId::Drive as u8 && cid == drive::cid::MOTOR_FAULT_NOTIFY => {
            Some(Notification::MotorFault {
                faulted: packet.payload.first().is_some_and(|&b| b != 0),
            })
        }
        (id, cid)
            if id == DeviceId::Power as u8
                && cid == power::cid::BATTERY_VOLTAGE_STATE_CHANGE_NOTIFY =>
        {
            power::parse_battery_voltage_state(&packet.payload)
                .map(Notification::BatteryVoltageState)
        }
        (id, cid) if id == DeviceId::Power as u8 && cid == power::cid::WILL_SLEEP_NOTIFY => {
            Some(Notification::WillSleep)
        }
        (id, cid) if id == DeviceId::Power as u8 && cid == power::cid::DID_SLEEP_NOTIFY => {
            Some(Notification::DidSleep)
        }
        _ => Some(Notification::Other(packet.clone())),
    }
}

/// Convenience wrapper for collecting notifications off the RX thread.
pub struct NotificationChannel {
    pub receiver: Receiver<Notification>,
}

impl NotificationChannel {
    /// Build a handler that forwards notifications onto a channel.
    pub fn new() -> (NotificationHandler, Self) {
        let (tx, receiver) = channel();
        let handler: NotificationHandler = Box::new(move |n| {
            let _ = tx.send(n);
        });
        (handler, Self { receiver })
    }
}
