use core::u8;

use heapless::Vec;

#[derive(Copy, Clone, PartialEq)]
pub enum State {
    Command,
    Key,
    Value,
}

#[repr(u8)]
pub enum NgChl2Responses {
    Ack = 0x0A,
    Nack = 0xC4,
    None,
}

pub struct NgChl2Parser {
    state: State,
    checksum: bool,
    last_packet: Vec<u8, 4>,
    commands: [u8; 3],
}

impl NgChl2Parser {
    pub fn new() -> Self {
        Self {
            state: State::Command,
            checksum: false,
            last_packet: Vec::new(),
            commands: [0x67, 0x69, 0xDF],
        }
    }
    pub fn process_byte(&mut self, byte: u8) -> NgChl2Responses {
        if !self.checksum {
            match self.state {
                State::Command => {
                    if self.commands.contains(&byte) {
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
                if self.last_packet[0] ^ self.last_packet[0] == complement {
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
                if self.last_packet[0] ^ self.last_packet[1] == complement {
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
                if self.last_packet[0]
                    ^ self.last_packet[1]
                    ^ self.last_packet[2]
                    ^ self.last_packet[3]
                    == complement
                {
                    self.last_packet.clear();
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
}
