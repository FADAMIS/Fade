// src/msp.rs

use crate::pid::{Axis, PidControllers, Vector3};
use crate::usb::UsbManager;
use heapless::Vec;

// MSPv2 protocol constants
const MSP_PREAMBLE: u8 = b'$';
const MSP_VERSION: u8 = b'X';
const MSP_REQUEST: u8 = b'<';
const MSP_RESPONSE: u8 = b'>';

// Define the MSP commands we want to handle
const MSP_SET_PID: u16 = 202;
const MSP_PID: u16 = 112;
// I will invent some commands for gyro and pid corrections, as they are not standard
const MSP_GYRO: u16 = 2000;
const MSP_PID_CORR: u16 = 2001;

#[derive(Debug)]
pub struct MspFrame {
    pub command: u16,
    pub payload: Vec<u8, 256>,
}

impl MspFrame {
    pub fn new(command: u16, payload: &[u8]) -> Self {
        let mut vec_payload = Vec::new();
        vec_payload.extend_from_slice(payload).unwrap();
        Self {
            command,
            payload: vec_payload,
        }
    }

    pub fn serialize(&self, buffer: &mut [u8]) -> Result<usize, ()> {
        if buffer.len() < self.payload.len() + 9 {
            return Err(());
        }

        buffer[0] = MSP_PREAMBLE;
        buffer[1] = MSP_VERSION;
        buffer[2] = MSP_RESPONSE;
        buffer[3] = 0; // flag
        buffer[4..6].copy_from_slice(&self.command.to_le_bytes());
        buffer[6..8].copy_from_slice(&(self.payload.len() as u16).to_le_bytes());
        buffer[8..8 + self.payload.len()].copy_from_slice(&self.payload);

        let mut checksum = 0;
        for i in 3..8 + self.payload.len() {
            checksum ^= buffer[i];
        }
        buffer[8 + self.payload.len()] = checksum;

        Ok(9 + self.payload.len())
    }
}

#[derive(Debug, PartialEq)]
enum ParserState {
    Idle,
    Preamble,
    Version,
    Direction,
    Flag,
    Command1,
    #[allow(unused)]
    Command2,
    PayloadSize1,
    PayloadSize2,
    Payload,
    Checksum,
}

pub struct MspParser {
    state: ParserState,
    command: u16,
    payload_size: u16,
    payload: Vec<u8, 256>,
    checksum: u8,
    payload_counter: u16,
}

impl MspParser {
    pub fn new() -> Self {
        Self {
            state: ParserState::Idle,
            command: 0,
            payload_size: 0,
            payload: Vec::new(),
            checksum: 0,
            payload_counter: 0,
        }
    }

    pub fn parse(&mut self, byte: u8) -> Option<MspFrame> {
        match self.state {
            ParserState::Idle => {
                if byte == MSP_PREAMBLE {
                    self.state = ParserState::Preamble;
                }
            }
            ParserState::Preamble => {
                if byte == MSP_VERSION {
                    self.state = ParserState::Version;
                } else {
                    self.state = ParserState::Idle;
                }
            }
            ParserState::Version => {
                if byte == MSP_REQUEST {
                    self.state = ParserState::Direction;
                } else {
                    self.state = ParserState::Idle;
                }
            }
            ParserState::Direction => {
                self.checksum = byte; // This is the flag byte
                self.state = ParserState::Flag;
            }
            ParserState::Flag => {
                self.command = byte as u16;
                self.checksum ^= byte;
                self.state = ParserState::Command1;
            }
            ParserState::Command1 => {
                self.command |= (byte as u16) << 8;
                self.checksum ^= byte;
                self.state = ParserState::PayloadSize1;
            }
            ParserState::PayloadSize1 => {
                self.payload_size = byte as u16;
                self.checksum ^= byte;
                self.state = ParserState::PayloadSize2;
            }
            ParserState::PayloadSize2 => {
                self.payload_size |= (byte as u16) << 8;
                self.checksum ^= byte;
                if self.payload_size == 0 {
                    self.state = ParserState::Checksum;
                } else {
                    self.payload.clear();
                    self.payload_counter = 0;
                    self.state = ParserState::Payload;
                }
            }
            ParserState::Payload => {
                self.payload.push(byte).unwrap();
                self.checksum ^= byte;
                self.payload_counter += 1;
                if self.payload_counter >= self.payload_size {
                    self.state = ParserState::Checksum;
                }
            }
            ParserState::Checksum => {
                if self.checksum == byte {
                    let frame = MspFrame {
                        command: self.command,
                        payload: self.payload.clone(),
                    };
                    self.state = ParserState::Idle;
                    return Some(frame);
                } else {
                    self.state = ParserState::Idle;
                }
            }
            ParserState::Command2 => {
                // This state should not be reached with the corrected logic, but we handle it to satisfy the compiler.
                self.state = ParserState::Idle;
            }
        }
        None
    }
}

pub struct MspManager {
    parser: MspParser,
}

impl MspManager {
    pub fn new() -> Self {
        Self {
            parser: MspParser::new(),
        }
    }

    pub fn handle_msp(
        &mut self,
        usb: &mut UsbManager,
        pids: &mut PidControllers,
        gyro: &Vector3,
        pid_corr: &Vector3,
    ) {
        let mut buffer = [0u8; 256];
        if let Ok(count) = usb.read_bytes(&mut buffer) {
            for i in 0..count {
                if let Some(frame) = self.parser.parse(buffer[i]) {
                    match frame.command {
                        MSP_SET_PID => {
                            if frame.payload.len() >= 4 * 9 {
                                let mut data = [0.0f32; 9];
                                for i in 0..9 {
                                    let bytes = [
                                        frame.payload[i * 4],
                                        frame.payload[i * 4 + 1],
                                        frame.payload[i * 4 + 2],
                                        frame.payload[i * 4 + 3],
                                    ];
                                    data[i] = f32::from_le_bytes(bytes);
                                }
                                pids.set_gains(Axis::Roll, data[0], data[1], data[2]);
                                pids.set_gains(Axis::Pitch, data[3], data[4], data[5]);
                                pids.set_gains(Axis::Yaw, data[6], data[7], data[8]);
                            }
                        }
                        MSP_PID => {
                            let gains = pids.get_gains();
                            let mut payload = [0u8; 4 * 9];
                            for idx in 0..3 {
                                let (p, i, d) = gains[idx];
                                let p_bytes = p.to_le_bytes();
                                let i_bytes = i.to_le_bytes();
                                let d_bytes = d.to_le_bytes();
                                payload[idx * 12..idx * 12 + 4].copy_from_slice(&p_bytes);
                                payload[idx * 12 + 4..idx * 12 + 8].copy_from_slice(&i_bytes);
                                payload[idx * 12 + 8..idx * 12 + 12].copy_from_slice(&d_bytes);
                            }
                            let response = MspFrame::new(MSP_PID, &payload);
                            let mut out_buffer = [0u8; 256];
                            if let Ok(len) = response.serialize(&mut out_buffer) {
                                usb.write_bytes(&out_buffer[..len]).ok();
                            }
                        }
                        MSP_GYRO => {
                            let mut payload = [0u8; 12];
                            let x_bytes = gyro.x.to_le_bytes();
                            let y_bytes = gyro.y.to_le_bytes();
                            let z_bytes = gyro.z.to_le_bytes();
                            payload[0..4].copy_from_slice(&x_bytes);
                            payload[4..8].copy_from_slice(&y_bytes);
                            payload[8..12].copy_from_slice(&z_bytes);
                            let response = MspFrame::new(MSP_GYRO, &payload);
                            let mut out_buffer = [0u8; 256];
                            if let Ok(len) = response.serialize(&mut out_buffer) {
                                usb.write_bytes(&out_buffer[..len]).ok();
                            }
                        }
                        MSP_PID_CORR => {
                            let mut payload = [0u8; 12];
                            let x_bytes = pid_corr.x.to_le_bytes();
                            let y_bytes = pid_corr.y.to_le_bytes();
                            let z_bytes = pid_corr.z.to_le_bytes();
                            payload[0..4].copy_from_slice(&x_bytes);
                            payload[4..8].copy_from_slice(&y_bytes);
                            payload[8..12].copy_from_slice(&z_bytes);
                            let response = MspFrame::new(MSP_PID_CORR, &payload);
                            let mut out_buffer = [0u8; 256];
                            if let Ok(len) = response.serialize(&mut out_buffer) {
                                usb.write_bytes(&out_buffer[..len]).ok();
                            }
                        }
                        _ => {}
                    }
                }
            }
        }
    }
}
