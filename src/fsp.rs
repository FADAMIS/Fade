use heapless::Vec;

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
    pub fn parse(&mut self, input_byte: u8) -> Option<Vec<u8, 5>> {
        if !self.checksum {
            match self.state {
                0 => {
                    if input_byte == 0x69 {
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
                let mut ack: Vec<u8, 5> = Vec::new();
                ack.push(0x0A).unwrap();
                if self.state == 2 {
                    self.state = 0;
                    self.cmd.clear();
                } else if self.state == 1 && self.send {
                    self.send = false;
                    self.state = 0;
                    self.cmd.clear();
                    let test = 20.0f32.to_be_bytes();
                    for i in test.into_iter() {
                        ack.push(i).unwrap();
                    }
                } else {
                    self.state += 1;
                }
                Some(ack)
            } else {
                self.checksum = false;
                self.state = 0;
                let mut nack: Vec<u8, 5> = Vec::new();
                nack.push(0xC4).unwrap();
                self.cmd.clear();
                Some(nack)
            }
        }
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
