use heapless::Vec;

use crate::config::{FlightConfig, PidGains};

/// FSP command IDs
const CMD_SEND: u8 = 0x69;
const CMD_READ_PID: u8 = 0x50;
const CMD_WRITE_PID: u8 = 0x51;
const CMD_SAVE_CONFIG: u8 = 0x52;

/// FSP response codes
const ACK: u8 = 0x0A;
const NACK: u8 = 0xC4;

/// Response indicating config should be saved to flash
pub const SAVE_FLAG: u8 = 0x53;

pub struct Fsp {
    state: u8,
    checksum: bool,
    cmd: Vec<u32, 3>,
    buf: Vec<u8, 4>,
    send: bool,
}

impl Fsp {
    pub fn new() -> Self {
        Self {
            state: 0,
            checksum: false,
            cmd: Vec::new(),
            buf: Vec::new(),
            send: false,
        }
    }

    /// Parse an incoming byte and optionally return a response.
    ///
    /// `config` is the current flight config; used to read PID gains on `0x69`
    /// and to write PID gains on `0x51`.
    ///
    /// Returns `Some(response_bytes)` when a complete command is handled.
    /// When `response[0] == SAVE_FLAG`, the caller should trigger a flash save.
    pub fn parse(&mut self, input_byte: u8, config: &mut FlightConfig) -> Option<Vec<u8, 41>> {
        if !self.checksum {
            match self.state {
                0 => {
                    if input_byte == CMD_SEND {
                        self.send = true;
                    }
                    self.cmd.push(input_byte as u32).unwrap();
                    self.checksum = true;
                    None
                }
                1 => {
                    self.buf.push(input_byte).unwrap();
                    if self.buf.len() == 2 {
                        self.cmd
                            .push(u16::from_be_bytes([self.buf[0], self.buf[1]]) as u32)
                            .unwrap();
                        self.buf.clear();
                        self.checksum = true;
                    }
                    None
                }
                2 => {
                    self.buf.push(input_byte).unwrap();
                    if self.buf.len() == 4 {
                        self.cmd
                            .push(u32::from_be_bytes(self.buf.clone().into_array().unwrap()))
                            .unwrap();
                        self.buf.clear();
                        self.checksum = true;
                    }
                    None
                }
                _ => None,
            }
        } else {
            if Self::verify_checksum(
                &self.cmd[self.state as usize].to_be_bytes(),
                input_byte,
                self.state,
            ) {
                self.checksum = false;
                let cmd_byte = self.cmd[0] as u8;

                match (self.state, cmd_byte) {
                    // ── READ PID (0x50): after command byte is verified, return all PID gains ──
                    (0, CMD_READ_PID) => {
                        let resp = self.build_pid_read_response(config);
                        self.reset_state();
                        Some(resp)
                    }

                    // ── SAVE CONFIG (0x52): trigger flash save ──
                    (0, CMD_SAVE_CONFIG) => {
                        let mut resp: Vec<u8, 41> = Vec::new();
                        resp.push(SAVE_FLAG).unwrap();
                        self.reset_state();
                        Some(resp)
                    }

                    // ── WRITE PID (0x51): need axis_id (state 1) + pid value (state 2) ──
                    (0, CMD_WRITE_PID) => {
                        self.state = 1;
                        let mut ack: Vec<u8, 41> = Vec::new();
                        ack.push(ACK).unwrap();
                        Some(ack)
                    }

                    // ── State 1 for WRITE PID: axis_id received ──
                    (1, CMD_WRITE_PID) => {
                        self.state = 2;
                        let mut ack: Vec<u8, 41> = Vec::new();
                        ack.push(ACK).unwrap();
                        Some(ack)
                    }

                    // ── State 2 for WRITE PID: pid value received, apply it ──
                    (2, CMD_WRITE_PID) => {
                        self.apply_pid_write(config);
                        let mut ack: Vec<u8, 41> = Vec::new();
                        ack.push(ACK).unwrap();
                        self.reset_state();
                        Some(ack)
                    }

                    // ── Original SEND command (0x69) ──
                    (1, CMD_SEND) => {
                        self.send = false;
                        // Return PID gains for current config
                        let resp = self.build_pid_read_response(config);
                        self.reset_state();
                        Some(resp)
                    }

                    // ── Generic state transitions for other commands ──
                    (0, _) => {
                        let mut ack: Vec<u8, 41> = Vec::new();
                        ack.push(ACK).unwrap();
                        self.state += 1;
                        Some(ack)
                    }
                    (1, _) => {
                        let mut ack: Vec<u8, 41> = Vec::new();
                        ack.push(ACK).unwrap();
                        self.state += 1;
                        Some(ack)
                    }
                    (2, _) => {
                        let mut ack: Vec<u8, 41> = Vec::new();
                        ack.push(ACK).unwrap();
                        self.reset_state();
                        Some(ack)
                    }
                    _ => None,
                }
            } else {
                self.checksum = false;
                self.reset_state();
                let mut nack: Vec<u8, 41> = Vec::new();
                nack.push(NACK).unwrap();
                Some(nack)
            }
        }
    }

    /// Build a response containing all 9 PID gains (roll P/I/D, pitch P/I/D, yaw P/I/D)
    fn build_pid_read_response(&self, config: &FlightConfig) -> Vec<u8, 41> {
        let mut resp: Vec<u8, 41> = Vec::new();
        resp.push(ACK).unwrap();

        // Pack 9 floats: roll(P,I,D), pitch(P,I,D), yaw(P,I,D)
        let gains = [
            config.pid_roll.p,
            config.pid_roll.i,
            config.pid_roll.d,
            config.pid_pitch.p,
            config.pid_pitch.i,
            config.pid_pitch.d,
            config.pid_yaw.p,
            config.pid_yaw.i,
            config.pid_yaw.d,
        ];

        for g in gains.iter() {
            for b in g.to_be_bytes().iter() {
                resp.push(*b).unwrap();
            }
        }

        resp
    }

    /// Apply a PID write command.
    /// cmd[1] = axis_id (0=roll, 1=pitch, 2=yaw) encoded as u16
    /// cmd[2] = packed PID value:
    ///   - P in upper 10 bits (0..1023) → maps to 0.0..200.0
    ///   - I in next 11 bits (0..2047)  → maps to 0.0..200.0
    ///   - D in lower 11 bits (0..2047) → maps to 0.0..200.0
    fn apply_pid_write(&self, config: &mut FlightConfig) {
        let axis_id = self.cmd[1] as u8;
        let packed = self.cmd[2];

        let p_raw = (packed >> 22) & 0x3FF;
        let i_raw = (packed >> 11) & 0x7FF;
        let d_raw = packed & 0x7FF;

        let p = p_raw as f32 * 200.0 / 1023.0;
        let i = i_raw as f32 * 200.0 / 2047.0;
        let d = d_raw as f32 * 200.0 / 2047.0;

        let gains = PidGains::new(p, i, d);

        match axis_id {
            0 => config.pid_roll = gains,
            1 => config.pid_pitch = gains,
            2 => config.pid_yaw = gains,
            _ => {} // ignore invalid axis
        }

        // Recalculate checksum after modifying config
        config.checksum = config.calculate_checksum();
    }

    fn reset_state(&mut self) {
        self.state = 0;
        self.send = false;
        self.cmd.clear();
        self.buf.clear();
    }

    fn verify_checksum(data: &[u8], check: u8, mode: u8) -> bool {
        let acc = data.iter().fold(0u8, |a, b| a ^ b);
        if mode == 0 {
            // In command mode: data must contain exactly 1 command byte
            acc ^ check == 0xFF
        } else {
            // In data mode: checksum must XOR to 0
            acc ^ check == 0x00
        }
    }
}
