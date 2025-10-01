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

// Extended configuration commands
const MSP_GET_FILTER_CONFIG: u16 = 3000;
const MSP_SET_FILTER_CONFIG: u16 = 3001;
const MSP_GET_RATE_PROFILE: u16 = 3002;
const MSP_SET_RATE_PROFILE: u16 = 3003;
const MSP_GET_MOTOR_CONFIG: u16 = 3004;
const MSP_SET_MOTOR_CONFIG: u16 = 3005;
const MSP_GET_OSD_CONFIG: u16 = 3006;
const MSP_SET_OSD_CONFIG: u16 = 3007;
const MSP_GET_FLIGHT_MODES: u16 = 3008;
const MSP_SET_FLIGHT_MODES: u16 = 3009;
const MSP_GET_BATTERY_CONFIG: u16 = 3010;
const MSP_SET_BATTERY_CONFIG: u16 = 3011;
const MSP_GET_FAILSAFE_CONFIG: u16 = 3012;
const MSP_SET_FAILSAFE_CONFIG: u16 = 3013;
const MSP_GET_CALIBRATION: u16 = 3014;
const MSP_SET_CALIBRATION: u16 = 3015;
const MSP_GET_ALL_CONFIG: u16 = 3016;
const MSP_SET_ALL_CONFIG: u16 = 3017;

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
    config: FlightConfig,
}

impl MspManager {
    pub fn new() -> Self {
        Self {
            parser: MspParser::new(),
            config: FlightConfig::new(),
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
                    MSP_GET_FILTER_CONFIG => {
                        let config = self.get_current_config();
                        let mut payload = [0u8; 20]; // 4 floats + 1 bool + 3 padding = 20 bytes
                        let p1_bytes = config.filters.gyro_lowpass1_hz.to_le_bytes();
                        let p2_bytes = config.filters.gyro_lowpass2_hz.to_le_bytes();
                        let pid_bytes = config.filters.pid_lowpass_hz.to_le_bytes();
                        let notch_bytes = config.filters.notch_filter_hz.to_le_bytes();

                        payload[0..4].copy_from_slice(&p1_bytes);
                        payload[4..8].copy_from_slice(&p2_bytes);
                        payload[8..12].copy_from_slice(&pid_bytes);
                        payload[12..16].copy_from_slice(&notch_bytes);
                        payload[16] = if config.filters.notch_filter_enabled {
                            1
                        } else {
                            0
                        };
                        // padding bytes remain 0

                        let response = MspFrame::new(MSP_GET_FILTER_CONFIG, &payload);
                        responses.push(response).ok();
                    }
                    MSP_SET_FILTER_CONFIG => {
                        if frame.payload.len() >= 17 {
                            let mut config = self.get_current_config();

                            config.filters.gyro_lowpass1_hz = f32::from_le_bytes([
                                frame.payload[0],
                                frame.payload[1],
                                frame.payload[2],
                                frame.payload[3],
                            ]);
                            config.filters.gyro_lowpass2_hz = f32::from_le_bytes([
                                frame.payload[4],
                                frame.payload[5],
                                frame.payload[6],
                                frame.payload[7],
                            ]);
                            config.filters.pid_lowpass_hz = f32::from_le_bytes([
                                frame.payload[8],
                                frame.payload[9],
                                frame.payload[10],
                                frame.payload[11],
                            ]);
                            config.filters.notch_filter_hz = f32::from_le_bytes([
                                frame.payload[12],
                                frame.payload[13],
                                frame.payload[14],
                                frame.payload[15],
                            ]);
                            config.filters.notch_filter_enabled = frame.payload[16] != 0;

                            config.checksum = config.calculate_checksum();
                            self.set_current_config(config);
                            defmt::info!("Filter configuration updated");
                        }
                    }
                    MSP_GET_RATE_PROFILE => {
                        let config = self.get_current_config();
                        let mut payload = [0u8; 36]; // 9 floats = 36 bytes
                        let rates = [
                            config.rates.max_rate_roll,
                            config.rates.max_rate_pitch,
                            config.rates.max_rate_yaw,
                            config.rates.expo_roll,
                            config.rates.expo_pitch,
                            config.rates.expo_yaw,
                            config.rates.super_rate_roll,
                            config.rates.super_rate_pitch,
                            config.rates.super_rate_yaw,
                        ];

                        for (i, rate) in rates.iter().enumerate() {
                            let bytes = rate.to_le_bytes();
                            payload[i * 4..(i + 1) * 4].copy_from_slice(&bytes);
                        }

                        let response = MspFrame::new(MSP_GET_RATE_PROFILE, &payload);
                        responses.push(response).ok();
                    }
                    MSP_SET_RATE_PROFILE => {
                        if frame.payload.len() >= 36 {
                            let mut config = self.get_current_config();

                            config.rates.max_rate_roll = f32::from_le_bytes([
                                frame.payload[0],
                                frame.payload[1],
                                frame.payload[2],
                                frame.payload[3],
                            ]);
                            config.rates.max_rate_pitch = f32::from_le_bytes([
                                frame.payload[4],
                                frame.payload[5],
                                frame.payload[6],
                                frame.payload[7],
                            ]);
                            config.rates.max_rate_yaw = f32::from_le_bytes([
                                frame.payload[8],
                                frame.payload[9],
                                frame.payload[10],
                                frame.payload[11],
                            ]);
                            config.rates.expo_roll = f32::from_le_bytes([
                                frame.payload[12],
                                frame.payload[13],
                                frame.payload[14],
                                frame.payload[15],
                            ]);
                            config.rates.expo_pitch = f32::from_le_bytes([
                                frame.payload[16],
                                frame.payload[17],
                                frame.payload[18],
                                frame.payload[19],
                            ]);
                            config.rates.expo_yaw = f32::from_le_bytes([
                                frame.payload[20],
                                frame.payload[21],
                                frame.payload[22],
                                frame.payload[23],
                            ]);
                            config.rates.super_rate_roll = f32::from_le_bytes([
                                frame.payload[24],
                                frame.payload[25],
                                frame.payload[26],
                                frame.payload[27],
                            ]);
                            config.rates.super_rate_pitch = f32::from_le_bytes([
                                frame.payload[28],
                                frame.payload[29],
                                frame.payload[30],
                                frame.payload[31],
                            ]);
                            config.rates.super_rate_yaw = f32::from_le_bytes([
                                frame.payload[32],
                                frame.payload[33],
                                frame.payload[34],
                                frame.payload[35],
                            ]);

                            config.checksum = config.calculate_checksum();
                            self.set_current_config(config);
                            defmt::info!("Rate profile updated");
                        }
                    }
                    MSP_GET_MOTOR_CONFIG => {
                        let config = self.get_current_config();
                        let mut payload = [0u8; 8]; // 2+4+1+1 = 8 bytes

                        payload[0..2].copy_from_slice(&config.motors.motor_pwm_rate.to_le_bytes());
                        payload[2..6].copy_from_slice(&config.motors.motor_idle.to_le_bytes());
                        payload[6] = config.motors.motor_protocol;
                        payload[7] = config.motors.motor_poles;

                        let response = MspFrame::new(MSP_GET_MOTOR_CONFIG, &payload);
                        responses.push(response).ok();
                    }
                    MSP_SET_MOTOR_CONFIG => {
                        if frame.payload.len() >= 8 {
                            let mut config = self.get_current_config();

                            config.motors.motor_pwm_rate =
                                u16::from_le_bytes([frame.payload[0], frame.payload[1]]);
                            config.motors.motor_idle = f32::from_le_bytes([
                                frame.payload[2],
                                frame.payload[3],
                                frame.payload[4],
                                frame.payload[5],
                            ]);
                            config.motors.motor_protocol = frame.payload[6];
                            config.motors.motor_poles = frame.payload[7];

                            config.checksum = config.calculate_checksum();
                            self.set_current_config(config);
                            defmt::info!("Motor configuration updated");
                        }
                    }
                    MSP_GET_OSD_CONFIG => {
                        let config = self.get_current_config();
                        let mut payload = [0u8; 12]; // 1+4+1+2+2+1+1 = 12 bytes

                        payload[0] = if config.osd.enabled { 1 } else { 0 };
                        payload[1..5].copy_from_slice(&config.osd.voltage_alarm.to_le_bytes());
                        payload[5] = config.osd.rssi_alarm;
                        payload[6..8].copy_from_slice(&config.osd.capacity_alarm.to_le_bytes());
                        payload[8..10].copy_from_slice(&config.osd.timer_alarm.to_le_bytes());
                        payload[10] = config.osd.units;
                        // padding byte remains 0

                        let response = MspFrame::new(MSP_GET_OSD_CONFIG, &payload);
                        responses.push(response).ok();
                    }
                    MSP_SET_OSD_CONFIG => {
                        if frame.payload.len() >= 11 {
                            let mut config = self.get_current_config();

                            config.osd.enabled = frame.payload[0] != 0;
                            config.osd.voltage_alarm = f32::from_le_bytes([
                                frame.payload[1],
                                frame.payload[2],
                                frame.payload[3],
                                frame.payload[4],
                            ]);
                            config.osd.rssi_alarm = frame.payload[5];
                            config.osd.capacity_alarm =
                                u16::from_le_bytes([frame.payload[6], frame.payload[7]]);
                            config.osd.timer_alarm =
                                u16::from_le_bytes([frame.payload[8], frame.payload[9]]);
                            config.osd.units = frame.payload[10];

                            config.checksum = config.calculate_checksum();
                            self.set_current_config(config);
                            defmt::info!("OSD configuration updated");
                        }
                    }
                    MSP_GET_FLIGHT_MODES => {
                        let config = self.get_current_config();
                        let mut payload = [0u8; 4]; // 4 bools = 4 bytes

                        payload[0] = if config.angle_mode_enabled { 1 } else { 0 };
                        payload[1] = if config.horizon_mode_enabled { 1 } else { 0 };
                        payload[2] = if config.air_mode_enabled { 1 } else { 0 };
                        payload[3] = if config.anti_gravity_enabled { 1 } else { 0 };

                        let response = MspFrame::new(MSP_GET_FLIGHT_MODES, &payload);
                        responses.push(response).ok();
                    }
                    MSP_SET_FLIGHT_MODES => {
                        if frame.payload.len() >= 4 {
                            let mut config = self.get_current_config();

                            config.angle_mode_enabled = frame.payload[0] != 0;
                            config.horizon_mode_enabled = frame.payload[1] != 0;
                            config.air_mode_enabled = frame.payload[2] != 0;
                            config.anti_gravity_enabled = frame.payload[3] != 0;

                            config.checksum = config.calculate_checksum();
                            self.set_current_config(config);
                            defmt::info!("Flight modes updated");
                        }
                    }
                    MSP_GET_BATTERY_CONFIG => {
                        let config = self.get_current_config();
                        let mut payload = [0u8; 11]; // 1+2+4+4 = 11 bytes

                        payload[0] = config.battery_cells;
                        payload[1..3].copy_from_slice(&config.battery_capacity.to_le_bytes());
                        payload[3..7].copy_from_slice(&config.voltage_scale.to_le_bytes());
                        payload[7..11].copy_from_slice(&config.current_scale.to_le_bytes());

                        let response = MspFrame::new(MSP_GET_BATTERY_CONFIG, &payload);
                        responses.push(response).ok();
                    }
                    MSP_SET_BATTERY_CONFIG => {
                        if frame.payload.len() >= 11 {
                            let mut config = self.get_current_config();

                            config.battery_cells = frame.payload[0];
                            config.battery_capacity =
                                u16::from_le_bytes([frame.payload[1], frame.payload[2]]);
                            config.voltage_scale = f32::from_le_bytes([
                                frame.payload[3],
                                frame.payload[4],
                                frame.payload[5],
                                frame.payload[6],
                            ]);
                            config.current_scale = f32::from_le_bytes([
                                frame.payload[7],
                                frame.payload[8],
                                frame.payload[9],
                                frame.payload[10],
                            ]);

                            config.checksum = config.calculate_checksum();
                            self.set_current_config(config);
                            defmt::info!("Battery configuration updated");
                        }
                    }
                    MSP_GET_FAILSAFE_CONFIG => {
                        let config = self.get_current_config();
                        let mut payload = [0u8; 4]; // 2+2 = 4 bytes

                        payload[0..2].copy_from_slice(&config.failsafe_throttle.to_le_bytes());
                        payload[2..4].copy_from_slice(&config.failsafe_delay.to_le_bytes());

                        let response = MspFrame::new(MSP_GET_FAILSAFE_CONFIG, &payload);
                        responses.push(response).ok();
                    }
                    MSP_SET_FAILSAFE_CONFIG => {
                        if frame.payload.len() >= 4 {
                            let mut config = self.get_current_config();

                            config.failsafe_throttle =
                                u16::from_le_bytes([frame.payload[0], frame.payload[1]]);
                            config.failsafe_delay =
                                u16::from_le_bytes([frame.payload[2], frame.payload[3]]);

                            config.checksum = config.calculate_checksum();
                            self.set_current_config(config);
                            defmt::info!("Failsafe configuration updated");
                        }
                    }
                    MSP_GET_CALIBRATION => {
                        let config = self.get_current_config();
                        let mut payload = [0u8; 24]; // 6 floats = 24 bytes
                        let calibration = [
                            config.gyro_bias_x,
                            config.gyro_bias_y,
                            config.gyro_bias_z,
                            config.accel_bias_x,
                            config.accel_bias_y,
                            config.accel_bias_z,
                        ];

                        for (i, bias) in calibration.iter().enumerate() {
                            let bytes = bias.to_le_bytes();
                            payload[i * 4..(i + 1) * 4].copy_from_slice(&bytes);
                        }

                        let response = MspFrame::new(MSP_GET_CALIBRATION, &payload);
                        responses.push(response).ok();
                    }
                    MSP_SET_CALIBRATION => {
                        if frame.payload.len() >= 24 {
                            let mut config = self.get_current_config();

                            config.gyro_bias_x = f32::from_le_bytes([
                                frame.payload[0],
                                frame.payload[1],
                                frame.payload[2],
                                frame.payload[3],
                            ]);
                            config.gyro_bias_y = f32::from_le_bytes([
                                frame.payload[4],
                                frame.payload[5],
                                frame.payload[6],
                                frame.payload[7],
                            ]);
                            config.gyro_bias_z = f32::from_le_bytes([
                                frame.payload[8],
                                frame.payload[9],
                                frame.payload[10],
                                frame.payload[11],
                            ]);
                            config.accel_bias_x = f32::from_le_bytes([
                                frame.payload[12],
                                frame.payload[13],
                                frame.payload[14],
                                frame.payload[15],
                            ]);
                            config.accel_bias_y = f32::from_le_bytes([
                                frame.payload[16],
                                frame.payload[17],
                                frame.payload[18],
                                frame.payload[19],
                            ]);
                            config.accel_bias_z = f32::from_le_bytes([
                                frame.payload[20],
                                frame.payload[21],
                                frame.payload[22],
                                frame.payload[23],
                            ]);

                            config.checksum = config.calculate_checksum();
                            self.set_current_config(config);
                            defmt::info!("Calibration data updated");
                        }
                    }
                    MSP_GET_ALL_CONFIG => {
                        let config = self.get_current_config();
                        let config_bytes = config.to_bytes();
                        // Send configuration in chunks if needed, for now send first 200 bytes
                        let chunk_size = core::cmp::min(200, config_bytes.len());
                        let response =
                            MspFrame::new(MSP_GET_ALL_CONFIG, &config_bytes[..chunk_size]);
                        responses.push(response).ok();
                    }
                    MSP_SET_ALL_CONFIG => {
                        if let Some(config) = FlightConfig::from_bytes(&frame.payload) {
                            self.set_current_config(config);
                            defmt::info!("Full configuration updated");
                        } else {
                            defmt::error!("Invalid configuration data received");
                        }
                    }
                    _ => {}
                }
            }
        }
        responses
    }

    fn get_current_config(&self) -> FlightConfig {
        self.config.clone()
    }

    fn set_current_config(&mut self, config: FlightConfig) {
        self.config = config;
    }
}
