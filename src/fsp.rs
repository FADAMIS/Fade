use heapless::Vec;

use crate::config::FlightConfig;

/// FSP command bytes
const CMD_SET: u8 = 0x67;
const CMD_GET: u8 = 0x69;
const CMD_DFU: u8 = 0xDF;
const CMD_SAVE: u8 = 0xFE;

/// Response bytes
const ACK: u8 = 0x0A;
const NACK: u8 = 0xC4;

/// Internal flag: first byte signals "save to flash" to the caller
pub const SAVE_FLAG: u8 = 0x53;

/// Key IDs
const KEY_P: u16 = 0;
const KEY_I: u16 = 1;
const KEY_D: u16 = 2;

pub struct Fsp {
    state: u8, // 0=cmd, 1=key, 2=value
    expect_checksum: bool,
    cmd_byte: u8,
    key: u16,
    buf: Vec<u8, 4>,
}

impl Fsp {
    pub fn new() -> Self {
        Self {
            state: 0,
            expect_checksum: false,
            cmd_byte: 0,
            key: 0,
            buf: Vec::new(),
        }
    }

    /// Parse one incoming byte. Returns Some(response) when a protocol step completes.
    /// When response[0] == SAVE_FLAG, the caller should trigger a flash save.
    pub fn parse(&mut self, byte: u8, config: &mut FlightConfig) -> Option<Vec<u8, 8>> {
        if !self.expect_checksum {
            match self.state {
                0 => {
                    self.cmd_byte = byte;
                    self.expect_checksum = true;
                    None
                }
                1 => {
                    let _ = self.buf.push(byte);
                    if self.buf.len() == 2 {
                        self.expect_checksum = true;
                    }
                    None
                }
                2 => {
                    let _ = self.buf.push(byte);
                    if self.buf.len() == 4 {
                        self.expect_checksum = true;
                    }
                    None
                }
                _ => {
                    self.reset();
                    None
                }
            }
        } else {
            self.expect_checksum = false;

            match self.state {
                // ── Command checksum: cmd ^ check must == 0xFF ──
                0 => {
                    if self.cmd_byte ^ byte == 0xFF {
                        match self.cmd_byte {
                            CMD_SET | CMD_GET => {
                                self.state = 1;
                                self.buf.clear();
                                Some(Self::ack())
                            }
                            CMD_SAVE => {
                                let mut resp: Vec<u8, 8> = Vec::new();
                                let _ = resp.push(SAVE_FLAG);
                                self.reset();
                                Some(resp)
                            }
                            CMD_DFU => {
                                self.reset();
                                Some(Self::ack())
                            }
                            _ => {
                                self.reset();
                                Some(Self::nack())
                            }
                        }
                    } else {
                        self.reset();
                        Some(Self::nack())
                    }
                }

                // ── Key checksum: XOR(key_bytes) ^ check must == 0x00 ──
                1 => {
                    let acc = self.buf.iter().fold(0u8, |a, b| a ^ b);
                    if acc ^ byte == 0x00 {
                        self.key = u16::from_be_bytes([self.buf[0], self.buf[1]]);
                        self.buf.clear();

                        match self.cmd_byte {
                            CMD_SET => {
                                self.state = 2;
                                Some(Self::ack())
                            }
                            CMD_GET => {
                                let resp = self.make_get_response(config);
                                self.reset();
                                Some(resp)
                            }
                            _ => {
                                self.reset();
                                Some(Self::nack())
                            }
                        }
                    } else {
                        self.reset();
                        Some(Self::nack())
                    }
                }

                // ── Value checksum: XOR(value_bytes) ^ check must == 0x00 ──
                2 => {
                    let acc = self.buf.iter().fold(0u8, |a, b| a ^ b);
                    if acc ^ byte == 0x00 {
                        let value = f32::from_be_bytes([
                            self.buf[0],
                            self.buf[1],
                            self.buf[2],
                            self.buf[3],
                        ]);
                        self.apply_value(self.key, value, config);
                        self.reset();
                        Some(Self::ack())
                    } else {
                        self.reset();
                        Some(Self::nack())
                    }
                }

                _ => {
                    self.reset();
                    None
                }
            }
        }
    }

    fn ack() -> Vec<u8, 8> {
        let mut v: Vec<u8, 8> = Vec::new();
        let _ = v.push(ACK);
        v
    }

    fn nack() -> Vec<u8, 8> {
        let mut v: Vec<u8, 8> = Vec::new();
        let _ = v.push(NACK);
        v
    }

    /// Build GET response: ACK + f32 big-endian + XOR checksum (6 bytes total)
    /// Configurator reads: waitForAck(1 byte) + readFull(5 bytes with checksum)
    fn make_get_response(&self, config: &FlightConfig) -> Vec<u8, 8> {
        let mut resp: Vec<u8, 8> = Vec::new();

        if let Some(value) = self.get_value(self.key, config) {
            let _ = resp.push(ACK);
            let bytes = value.to_be_bytes();
            let mut cs: u8 = 0;
            for b in bytes {
                let _ = resp.push(b);
                cs ^= b;
            }
            let _ = resp.push(cs);
        } else {
            let _ = resp.push(NACK);
        }

        resp
    }

    fn get_value(&self, key: u16, config: &FlightConfig) -> Option<f32> {
        match key {
            KEY_P => Some(config.pid_roll.p),
            KEY_I => Some(config.pid_roll.i),
            KEY_D => Some(config.pid_roll.d),
            _ => None,
        }
    }

    fn apply_value(&self, key: u16, value: f32, config: &mut FlightConfig) {
        match key {
            KEY_P => config.pid_roll.p = value,
            KEY_I => config.pid_roll.i = value,
            KEY_D => config.pid_roll.d = value,
            _ => return,
        }
        config.checksum = config.calculate_checksum();
    }

    fn reset(&mut self) {
        self.state = 0;
        self.expect_checksum = false;
        self.cmd_byte = 0;
        self.key = 0;
        self.buf.clear();
    }
}
