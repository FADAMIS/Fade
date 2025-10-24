use core::u8;

use heapless::Vec;

#[derive(Copy, Clone, PartialEq)]
pub enum State {
    Command,
    Key,
    Value,
}

#[repr(u8)]
#[derive(PartialEq)]
pub enum NgChl2Responses {
    Ack = 0x0A,
    Nack = 0xC4,
    None,
}

#[derive(Clone)]
pub struct NgChl2Packet {
    command: u8,
    key: u16,
    value: f32,
}

pub struct NgChl2Result {
    pub command: NgChl2Packet,
    pub ready: bool,
}

pub struct NgChl2Parser {
    state: State,
    checksum: bool,
    last_packet: Vec<u8, 4>,
    commands: [u8; 3],
    command_packet: NgChl2Packet,
    command_ready: bool,
}

impl NgChl2Parser {
    pub fn new() -> Self {
        Self {
            state: State::Command,
            checksum: false,
            last_packet: Vec::new(),
            commands: [0x67, 0x69, 0xDF],
            command_packet: NgChl2Packet {
                command: 0,
                key: 0,
                value: 0.0,
            },
            command_ready: false,
        }
    }
    pub fn process_byte(&mut self, byte: u8) -> NgChl2Responses {
        if !self.checksum {
            match self.state {
                State::Command => {
                    if self.commands.contains(&byte) {
                        self.command_ready = false;
                        self.last_packet.push(byte).unwrap();
                        self.checksum = true;
                        NgChl2Responses::None
                    } else {
                        NgChl2Responses::None
                    }
                }
                State::Key => {
                    self.last_packet.push(byte).unwrap();
                    if self.last_packet.len() == 2 {
                        self.checksum = true;
                        NgChl2Responses::None
                    } else {
                        NgChl2Responses::None
                    }
                }
                State::Value => {
                    self.last_packet.push(byte).unwrap();
                    if self.last_packet.len() == 4 {
                        self.checksum = true;
                        NgChl2Responses::None
                    } else {
                        NgChl2Responses::None
                    }
                }
            }
        } else {
            Self::checksum(self, byte)
        }
    }
    pub fn checksum(&mut self, complement: u8) -> NgChl2Responses {
        match self.state {
            State::Command => {
                if self.last_packet[0] ^ complement == 0xFF {
                    self.command_packet = NgChl2Packet {
                        command: 0,
                        key: 0,
                        value: 0.0,
                    };
                    self.command_packet.command = self.last_packet[0];
                    self.last_packet.clear();
                    self.state = State::Key;
                    self.checksum = false;
                    NgChl2Responses::Ack
                } else {
                    self.last_packet.clear();
                    self.state = State::Command;
                    self.checksum = false;
                    NgChl2Responses::Nack
                }
            }
            State::Key => {
                if Self::xor(2, &self.last_packet, complement) == 0xFF {
                    let array: [u8; 2] = self.last_packet[0..2].try_into().unwrap();
                    self.command_packet.key = u16::from_be_bytes(array);
                    self.last_packet.clear();
                    self.state = State::Value;
                    self.checksum = false;
                    NgChl2Responses::Ack
                } else {
                    self.last_packet.clear();
                    self.state = State::Command;
                    self.checksum = false;
                    NgChl2Responses::Nack
                }
            }
            State::Value => {
                if Self::xor(4, &self.last_packet, complement) == 0xFF {
                    let array: [u8; 4] = self.last_packet.clone().into_array().unwrap();
                    self.command_packet.value = f32::from_be_bytes(array);
                    self.last_packet.clear();
                    self.command_ready = true;
                    self.state = State::Command;
                    self.checksum = false;
                    NgChl2Responses::Ack
                } else {
                    self.last_packet.clear();
                    self.state = State::Command;
                    self.checksum = false;
                    NgChl2Responses::Nack
                }
            }
        }
    }
    fn xor(n: usize, data: &[u8], complement: u8) -> u8 {
        let mut result = 0;
        for i in 0..n - 1 {
            result ^= data[i];
        }
        result ^ complement
    }
    pub fn get_command(&mut self) -> NgChl2Result {
        NgChl2Result {
            command: self.command_packet.clone(),
            ready: self.command_ready,
        }
    }
}
