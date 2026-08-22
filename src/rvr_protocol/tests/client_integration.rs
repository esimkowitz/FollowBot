//! End-to-end tests over a fake robot.
//!
//! These drive the real client, framer and codec; only the serial port is
//! replaced. That covers the request/response handshake, notification dispatch
//! and streaming decode without any hardware.

use rvr_protocol::client::Notification;
use rvr_protocol::devices::{drive, power, sensor, system_info};
use rvr_protocol::ids::{DeviceId, Target};
use rvr_protocol::packet::{flags, Packet, PacketReader};
use rvr_protocol::streaming::{DataSize, ServiceId, SlotConfig, StreamSample};
use rvr_protocol::transport::MockTransport;
use rvr_protocol::{NotificationChannel, RvrClient};

use std::time::Duration;

/// Decode whatever the client wrote to the wire.
fn sent_packets(transport: &MockTransport) -> Vec<Packet> {
    PacketReader::new()
        .push(&transport.take_written())
        .into_iter()
        .map(|r| r.expect("client emitted a decodable packet"))
        .collect()
}

/// Build the response the robot would send for a request.
fn response_to(request: &Packet, payload: Vec<u8>) -> Vec<u8> {
    Packet {
        flags: flags::IS_RESPONSE | flags::HAS_TARGET | flags::HAS_SOURCE,
        // Source and target swap on the way back.
        target: Some(0x00),
        source: request.target,
        device_id: request.device_id,
        command_id: request.command_id,
        sequence: request.sequence,
        error: Some(rvr_protocol::ErrorCode::Success),
        payload,
    }
    .encode()
}

/// Build an unsolicited notification packet.
fn notification(source: Target, device: DeviceId, command: u8, payload: Vec<u8>) -> Vec<u8> {
    Packet {
        flags: flags::HAS_TARGET | flags::HAS_SOURCE,
        target: Some(0x00),
        source: Some(source as u8),
        device_id: device as u8,
        command_id: command,
        sequence: 0,
        error: None,
        payload,
    }
    .encode()
}

/// Spawn a thread that answers the next request with `payload`.
///
/// The client blocks awaiting a response, so the reply has to come from
/// elsewhere; this polls the mock until the request appears.
fn answer_next_request(
    transport: &MockTransport,
    payload: Vec<u8>,
) -> std::thread::JoinHandle<Packet> {
    let transport = transport.clone();
    std::thread::spawn(move || {
        for _ in 0..200 {
            let packets = sent_packets(&transport);
            if let Some(request) = packets.into_iter().next() {
                transport.push_to_read(&response_to(&request, payload));
                return request;
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        panic!("client never sent a request");
    })
}

#[test]
fn fire_and_forget_commands_are_written_without_awaiting_a_response() {
    let transport = MockTransport::new();
    let client = RvrClient::new(Box::new(transport.clone())).unwrap();

    client.drive_tank_si(0.5, -0.25).unwrap();

    let packets = sent_packets(&transport);
    assert_eq!(packets.len(), 1);
    let packet = &packets[0];

    assert_eq!(packet.device_id, DeviceId::Drive as u8);
    assert_eq!(packet.command_id, drive::cid::DRIVE_TANK_SI_UNITS);
    assert_eq!(packet.target, Some(Target::Secondary as u8));
    assert_eq!(
        packet.flags & flags::REQUESTS_RESPONSE,
        0,
        "drive commands at rate should not request a response"
    );
    assert_ne!(
        packet.flags & flags::IS_ACTIVITY,
        0,
        "commands must mark activity to defer the inactivity sleep"
    );

    let left = f32::from_be_bytes(packet.payload[0..4].try_into().unwrap());
    let right = f32::from_be_bytes(packet.payload[4..8].try_into().unwrap());
    assert_eq!((left, right), (0.5, -0.25));
}

#[test]
fn request_matches_the_response_to_its_sequence_number() {
    let transport = MockTransport::new();
    let client = RvrClient::new(Box::new(transport.clone())).unwrap();

    // 87% battery.
    let responder = answer_next_request(&transport, vec![87]);
    assert_eq!(client.battery_percentage().unwrap(), 87);

    let request = responder.join().unwrap();
    assert_eq!(request.device_id, DeviceId::Power as u8);
    assert_eq!(request.command_id, power::cid::GET_BATTERY_PERCENTAGE);
    assert_eq!(
        request.target,
        Some(Target::Primary as u8),
        "power lives on the Nordic side"
    );
    assert_ne!(request.flags & flags::REQUESTS_RESPONSE, 0);
}

#[test]
fn firmware_version_parses_from_a_response() {
    let transport = MockTransport::new();
    let client = RvrClient::new(Box::new(transport.clone())).unwrap();

    let responder = answer_next_request(&transport, vec![0x00, 0x03, 0x00, 0x01, 0x00, 0x2A]);
    let version = client.firmware_version(Target::Secondary).unwrap();

    assert_eq!(
        version,
        system_info::Version {
            major: 3,
            minor: 1,
            revision: 42
        }
    );
    responder.join().unwrap();
}

#[test]
fn sequence_numbers_advance_and_start_at_one() {
    let transport = MockTransport::new();
    let client = RvrClient::new(Box::new(transport.clone())).unwrap();

    client.drive_tank_si(0.0, 0.0).unwrap();
    client.drive_stop().unwrap();
    client.drive_tank_si(0.1, 0.1).unwrap();

    let sequences: Vec<u8> = sent_packets(&transport)
        .iter()
        .map(|p| p.sequence)
        .collect();
    assert_eq!(
        sequences,
        vec![1, 2, 3],
        "firmware's generator pre-increments"
    );
}

#[test]
fn robot_errors_surface_as_errors_not_silent_success() {
    let transport = MockTransport::new();
    let client = RvrClient::new(Box::new(transport.clone())).unwrap();

    let poller = {
        let transport = transport.clone();
        std::thread::spawn(move || {
            for _ in 0..200 {
                if let Some(request) = sent_packets(&transport).into_iter().next() {
                    let mut failure = Packet {
                        flags: flags::IS_RESPONSE | flags::HAS_TARGET | flags::HAS_SOURCE,
                        target: Some(0x00),
                        source: request.target,
                        device_id: request.device_id,
                        command_id: request.command_id,
                        sequence: request.sequence,
                        error: Some(rvr_protocol::ErrorCode::Busy),
                        payload: vec![],
                    };
                    failure.error = Some(rvr_protocol::ErrorCode::Busy);
                    transport.push_to_read(&failure.encode());
                    return;
                }
                std::thread::sleep(Duration::from_millis(5));
            }
        })
    };

    let result = client.battery_percentage();
    poller.join().unwrap();

    assert!(
        matches!(
            result,
            Err(rvr_protocol::Error::Robot(rvr_protocol::ErrorCode::Busy))
        ),
        "got {result:?}"
    );
}

#[test]
fn requests_time_out_rather_than_hanging_forever() {
    let transport = MockTransport::new();
    let client = RvrClient::new(Box::new(transport))
        .unwrap()
        .with_timeout(Duration::from_millis(50));

    // Nothing ever answers.
    let result = client.battery_percentage();
    assert!(
        matches!(result, Err(rvr_protocol::Error::Timeout { .. })),
        "got {result:?}"
    );
}

#[test]
fn streaming_notifications_decode_against_the_configured_slot() {
    let transport = MockTransport::new();
    let client = RvrClient::new(Box::new(transport.clone())).unwrap();

    let (handler, channel) = NotificationChannel::new();
    client.on_notification(handler);

    // Configure locator + velocity on the ST processor, slot 2.
    let slot = SlotConfig::new(Target::Secondary, 0x02)
        .with(ServiceId::Locator)
        .unwrap()
        .with(ServiceId::Velocity)
        .unwrap();

    let responder = answer_next_request(&transport, vec![]);
    client.configure_streaming(&slot).unwrap();
    responder.join().unwrap();

    // The robot streams: locator (1.0, -2.0) m, velocity (0.5, 0.0) m/s.
    let mut payload = vec![0x02]; // status OK, token 2
    for (value, attr) in [
        (1.0, &ServiceId::Locator.def().attributes[0]),
        (-2.0, &ServiceId::Locator.def().attributes[1]),
        (0.5, &ServiceId::Velocity.def().attributes[0]),
        (0.0, &ServiceId::Velocity.def().attributes[1]),
    ] {
        let raw = (((value - attr.min) / (attr.max - attr.min)) * DataSize::ThirtyTwo.max_raw())
            .round() as u32;
        payload.extend_from_slice(&raw.to_be_bytes());
    }
    transport.push_to_read(&notification(
        Target::Secondary,
        DeviceId::Sensor,
        sensor::cid::STREAMING_SERVICE_DATA_NOTIFY,
        payload,
    ));

    let notification = channel
        .receiver
        .recv_timeout(Duration::from_secs(2))
        .expect("streaming notification should arrive");

    match notification {
        Notification::Stream {
            processor,
            token,
            samples,
        } => {
            assert_eq!(processor, Target::Secondary);
            assert_eq!(token, 2);
            assert_eq!(samples.len(), 2);

            match samples[0] {
                StreamSample::Locator { x, y } => {
                    assert!((x - 1.0).abs() < 1e-2, "x = {x}");
                    assert!((y - -2.0).abs() < 1e-2, "y = {y}");
                }
                other => panic!("expected Locator, got {other:?}"),
            }
            match samples[1] {
                StreamSample::Velocity { x, y } => {
                    assert!((x - 0.5).abs() < 1e-3, "x = {x}");
                    assert!(y.abs() < 1e-3, "y = {y}");
                }
                other => panic!("expected Velocity, got {other:?}"),
            }
        }
        other => panic!("expected a stream notification, got {other:?}"),
    }
}

#[test]
fn motor_stall_notifications_are_classified() {
    let transport = MockTransport::new();
    let client = RvrClient::new(Box::new(transport.clone())).unwrap();

    let (handler, channel) = NotificationChannel::new();
    client.on_notification(handler);

    transport.push_to_read(&notification(
        Target::Secondary,
        DeviceId::Drive,
        drive::cid::MOTOR_STALL_NOTIFY,
        vec![1, 1], // right motor, stalled
    ));

    let received = channel
        .receiver
        .recv_timeout(Duration::from_secs(2))
        .unwrap();
    assert_eq!(
        received,
        Notification::MotorStall(drive::MotorStall {
            motor: drive::MotorIndex::Right,
            stalled: true
        })
    );
}

#[test]
fn battery_state_notifications_are_classified() {
    let transport = MockTransport::new();
    let client = RvrClient::new(Box::new(transport.clone())).unwrap();

    let (handler, channel) = NotificationChannel::new();
    client.on_notification(handler);

    transport.push_to_read(&notification(
        Target::Primary,
        DeviceId::Power,
        power::cid::BATTERY_VOLTAGE_STATE_CHANGE_NOTIFY,
        vec![2], // low
    ));

    let received = channel
        .receiver
        .recv_timeout(Duration::from_secs(2))
        .unwrap();
    assert_eq!(
        received,
        Notification::BatteryVoltageState(power::BatteryVoltageState::Low)
    );
}

#[test]
fn notifications_do_not_block_concurrent_commands() {
    // The point of the RX thread: sensor traffic must not stall the control path.
    let transport = MockTransport::new();
    let client = RvrClient::new(Box::new(transport.clone())).unwrap();

    let (handler, channel) = NotificationChannel::new();
    client.on_notification(handler);

    for _ in 0..20 {
        transport.push_to_read(&notification(
            Target::Secondary,
            DeviceId::Drive,
            drive::cid::MOTOR_STALL_NOTIFY,
            vec![0, 0],
        ));
    }

    // Writing while the RX thread churns through notifications must not block.
    for _ in 0..10 {
        client.drive_tank_si(0.1, 0.1).unwrap();
    }

    let mut received = 0;
    while channel
        .receiver
        .recv_timeout(Duration::from_millis(200))
        .is_ok()
    {
        received += 1;
    }
    assert_eq!(received, 20, "every notification should be delivered");
    assert_eq!(
        sent_packets(&transport).len(),
        10,
        "every command should be written"
    );
}

#[test]
fn a_corrupt_frame_does_not_stop_later_notifications() {
    let transport = MockTransport::new();
    let client = RvrClient::new(Box::new(transport.clone())).unwrap();

    let (handler, channel) = NotificationChannel::new();
    client.on_notification(handler);

    // Garbage frame, then a good one: the framer should resynchronize.
    transport.push_to_read(&[0x8D, 0x01, 0x02, 0x03, 0xD8]);
    transport.push_to_read(&notification(
        Target::Primary,
        DeviceId::Power,
        power::cid::DID_SLEEP_NOTIFY,
        vec![],
    ));

    let received = channel
        .receiver
        .recv_timeout(Duration::from_secs(2))
        .unwrap();
    assert_eq!(received, Notification::DidSleep);
}

#[test]
fn clearing_streaming_forgets_the_slot_layout() {
    let transport = MockTransport::new();
    let client = RvrClient::new(Box::new(transport.clone())).unwrap();

    let (handler, channel) = NotificationChannel::new();
    client.on_notification(handler);

    let slot = SlotConfig::new(Target::Secondary, 0x02)
        .with(ServiceId::Speed)
        .unwrap();

    // The responder must be armed before the blocking call, not after.
    let responder = answer_next_request(&transport, vec![]);
    client.configure_streaming(&slot).unwrap();
    responder.join().unwrap();

    // clear_streaming issues stop and then clear; answer both.
    let clearing = {
        let transport = transport.clone();
        std::thread::spawn(move || {
            // Answer both requests clear_streaming makes.
            for _ in 0..2 {
                for _ in 0..200 {
                    if let Some(request) = sent_packets(&transport).into_iter().next() {
                        transport.push_to_read(&response_to(&request, vec![]));
                        break;
                    }
                    std::thread::sleep(Duration::from_millis(5));
                }
            }
        })
    };
    client.clear_streaming(Target::Secondary).unwrap();
    clearing.join().unwrap();

    // A stream for the now-forgotten slot must not decode as stale data.
    let mut payload = vec![0x02];
    payload.extend_from_slice(&0u32.to_be_bytes());
    transport.push_to_read(&notification(
        Target::Secondary,
        DeviceId::Sensor,
        sensor::cid::STREAMING_SERVICE_DATA_NOTIFY,
        payload,
    ));

    assert!(
        channel
            .receiver
            .recv_timeout(Duration::from_millis(300))
            .is_err(),
        "an unconfigured slot should be dropped, not decoded"
    );
}
