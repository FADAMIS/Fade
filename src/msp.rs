// src/msp.rs

use crate::config::{FlightConfig, CONFIG_FLASH_ADDRESS};
use crate::pid::{Axis, PidControllers, Vector3};
use heapless::Vec;

// Simulated flash storage (4KB)
static mut FLASH_STORAGE: [u8; 4096] = [0xFF; 4096];
static mut FLASH_INITIALIZED: bool = false;

// Configuration storage
static mut CURRENT_CONFIG: Option<FlightConfig> = None;

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
const MSP_SAVE_SETTINGS: u16 = 2002;
const MSP_FLASH_WRITE: u16 = 2003;
const MSP_FLASH_READ: u16 = 2004;

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
    #[allow(dead_code)]
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
            ParserState::Command2 => {
                // This state should not be reached with the corrected logic
                self.state = ParserState::Idle;
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

    fn save_config_to_flash(&self, pids: &PidControllers) -> bool {
        #[allow(static_mut_refs)]
        unsafe {
            if !FLASH_INITIALIZED {
                FLASH_STORAGE.fill(0xFF);
                FLASH_INITIALIZED = true;
            }

            // Get or create current config
            let mut config = CURRENT_CONFIG
                .clone()
                .unwrap_or_else(|| FlightConfig::new());

            // Update PID values in config
            config.update_from_pid_controllers(pids);

            // Serialize config to bytes
            let config_clone = config.clone();
            let config_bytes = config_clone.to_bytes();
            let start_addr = (CONFIG_FLASH_ADDRESS % 4096) as usize;
            let end_addr = start_addr + config_bytes.len();

            if end_addr <= FLASH_STORAGE.len() {
                FLASH_STORAGE[start_addr..end_addr].copy_from_slice(config_bytes);
                CURRENT_CONFIG = Some(config);
                defmt::info!(
                    "Configuration saved to flash (size: {} bytes)",
                    config_bytes.len()
                );
                true
            } else {
                defmt::error!("Config save failed: out of bounds");
                false
            }
        }
    }

    pub fn load_config_from_flash(&self, pids: &mut PidControllers) -> bool {
        #[allow(static_mut_refs)]
        unsafe {
            if !FLASH_INITIALIZED {
                FLASH_STORAGE.fill(0xFF);
                FLASH_INITIALIZED = true;
                return false;
            }

            let start_addr = (CONFIG_FLASH_ADDRESS % 4096) as usize;
            let config_size = FlightConfig::size();
            let end_addr = start_addr + config_size;

            if end_addr > FLASH_STORAGE.len() {
                defmt::error!("Config load failed: out of bounds");
                return false;
            }

            let config_bytes = &FLASH_STORAGE[start_addr..end_addr];

            // Try to deserialize config
            if let Some(config) = FlightConfig::from_bytes(config_bytes) {
                defmt::info!("Valid configuration loaded from flash");

                // Apply config to PID controllers
                config.apply_to_pid_controllers(pids);

                defmt::info!(
                    "Configuration applied: Roll P={} I={} D={}",
                    config.pid_roll.p,
                    config.pid_roll.i,
                    config.pid_roll.d
                );
                defmt::info!(
                    "                     Pitch P={} I={} D={}",
                    config.pid_pitch.p,
                    config.pid_pitch.i,
                    config.pid_pitch.d
                );
                defmt::info!(
                    "                     Yaw P={} I={} D={}",
                    config.pid_yaw.p,
                    config.pid_yaw.i,
                    config.pid_yaw.d
                );

                // Store loaded config
                CURRENT_CONFIG = Some(config);

                true
            } else {
                defmt::info!("No valid configuration found in flash, using defaults");
                CURRENT_CONFIG = Some(FlightConfig::new());
                false
            }
        }
    }

    pub fn handle_msp(
        &mut self,
        data: &[u8],
        pids: &mut PidControllers,
        gyro: &Vector3,
        pid_corr: &Vector3,
    ) -> Vec<MspFrame, 8> {
        let mut responses = Vec::new();

        defmt::debug!("MSP received {} bytes: {:02X}", data.len(), data);

        for &byte in data {
            if let Some(frame) = self.parser.parse(byte) {
                defmt::info!(
                    "MSP frame parsed: command=0x{:04X}, payload_len={}",
                    frame.command,
                    frame.payload.len()
                );
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

                            // Auto-save configuration when PID values are changed
                            self.save_config_to_flash(pids);
                            defmt::info!("PID values updated and configuration saved");
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
                        responses.push(response).ok();
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
                        responses.push(response).ok();
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
                        responses.push(response).ok();
                    }
                    MSP_SAVE_SETTINGS => {
                        // Save current configuration to flash
                        let success = self.save_config_to_flash(pids);
                        let response_data = [if success { 1 } else { 0 }];
                        let response = MspFrame::new(MSP_SAVE_SETTINGS, &response_data);
                        responses.push(response).ok();

                        if success {
                            defmt::info!("Configuration saved successfully");
                        } else {
                            defmt::error!("Configuration save failed");
                        }
                    }
                    MSP_FLASH_WRITE => {
                        defmt::info!(
                            "MSP_FLASH_WRITE received, payload len: {}",
                            frame.payload.len()
                        );
                        // Payload: [address:4][data_length:4][data:N]
                        if frame.payload.len() >= 8 {
                            let address_bytes = [
                                frame.payload[0],
                                frame.payload[1],
                                frame.payload[2],
                                frame.payload[3],
                            ];
                            let length_bytes = [
                                frame.payload[4],
                                frame.payload[5],
                                frame.payload[6],
                                frame.payload[7],
                            ];
                            let address = u32::from_le_bytes(address_bytes);
                            let length = u32::from_le_bytes(length_bytes) as usize;

                            defmt::info!("Flash write: addr=0x{:08X}, expected len={}, actual payload len={}", address, length, frame.payload.len());
                            if frame.payload.len() >= 8 + length {
                                let data = &frame.payload[8..8 + length];
                                #[allow(static_mut_refs)]
                                let success = unsafe {
                                    if !FLASH_INITIALIZED {
                                        // Initialize flash with 0xFF pattern
                                        FLASH_STORAGE.fill(0xFF);
                                        FLASH_INITIALIZED = true;
                                    }

                                    let start_addr = (address % 4096) as usize;
                                    let end_addr = start_addr + length;

                                    if end_addr <= FLASH_STORAGE.len() {
                                        FLASH_STORAGE[start_addr..end_addr].copy_from_slice(data);
                                        defmt::info!(
                                            "Flash write: addr=0x{:08X}, len={} - SUCCESS",
                                            address,
                                            length
                                        );
                                        true
                                    } else {
                                        defmt::error!(
                                            "Flash write: addr=0x{:08X}, len={} - OUT OF BOUNDS",
                                            address,
                                            length
                                        );
                                        false
                                    }
                                };

                                let response_data = [if success { 1 } else { 0 }];
                                let response = MspFrame::new(MSP_FLASH_WRITE, &response_data);
                                defmt::info!(
                                    "Sending flash write response: {}",
                                    if success { "SUCCESS" } else { "FAILED" }
                                );
                                responses.push(response).ok();
                            } else {
                                defmt::error!(
                                    "Flash write payload too short: expected {}, got {}",
                                    8 + length,
                                    frame.payload.len()
                                );
                            }
                        } else {
                            defmt::error!(
                                "Flash write payload too short: expected >=8, got {}",
                                frame.payload.len()
                            );
                        }
                    }
                    MSP_FLASH_READ => {
                        defmt::info!(
                            "MSP_FLASH_READ received, payload len: {}",
                            frame.payload.len()
                        );
                        // Payload: [address:4][length:4]
                        if frame.payload.len() >= 8 {
                            let address_bytes = [
                                frame.payload[0],
                                frame.payload[1],
                                frame.payload[2],
                                frame.payload[3],
                            ];
                            let length_bytes = [
                                frame.payload[4],
                                frame.payload[5],
                                frame.payload[6],
                                frame.payload[7],
                            ];
                            let address = u32::from_le_bytes(address_bytes);
                            let length = u32::from_le_bytes(length_bytes) as usize;

                            if length <= 200 {
                                // Read from simulated flash storage
                                let mut read_data = [0u8; 200];
                                #[allow(static_mut_refs)]
                                let success = unsafe {
                                    if !FLASH_INITIALIZED {
                                        // Initialize flash with 0xFF pattern
                                        FLASH_STORAGE.fill(0xFF);
                                        FLASH_INITIALIZED = true;
                                    }

                                    let start_addr = (address % 4096) as usize;
                                    let end_addr = start_addr + length;
                                    #[allow(static_mut_refs)]
                                    if end_addr <= FLASH_STORAGE.len() {
                                        read_data[..length]
                                            .copy_from_slice(&FLASH_STORAGE[start_addr..end_addr]);
                                        defmt::info!(
                                            "Flash read: addr=0x{:08X}, len={} - SUCCESS",
                                            address,
                                            length
                                        );
                                        true
                                    } else {
                                        defmt::error!(
                                            "Flash read: addr=0x{:08X}, len={} - OUT OF BOUNDS",
                                            address,
                                            length
                                        );
                                        // Fill with test pattern if out of bounds
                                        for i in 0..length {
                                            read_data[i] = (address as u8).wrapping_add(i as u8);
                                        }
                                        false
                                    }
                                };

                                let response = if success {
                                    MspFrame::new(MSP_FLASH_READ, &read_data[..length])
                                } else {
                                    // Return test pattern for out-of-bounds reads
                                    MspFrame::new(MSP_FLASH_READ, &read_data[..length])
                                };
                                responses.push(response).ok();
                            }
                        }
                    }
                    _ => {}
                }
            }
        }
        responses
    }
}
