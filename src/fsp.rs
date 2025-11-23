use heapless::Vec;

pub struct Fsp {
    state: u8,
    checksum: bool,
    cmd: Vec<f32, 3>,
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
                    self.cmd[0] = input_byte as f32;
                    self.checksum = true;
                    None
                }
                1 => {
                    self.buf.push(input_byte).unwrap();
                    if self.buf.len() == 2 {
                        self.cmd[1] = u16::from_be_bytes([self.buf[0], self.buf[1]]) as f32;
                        self.buf.clear();
                        self.checksum = true;
                    }
                    None
                }
                2 => {
                    self.buf.push(input_byte).unwrap();
                    if self.buf.len() == 4 {
                        self.cmd[2] = f32::from_be_bytes(self.buf.clone().into_array().unwrap());
                        self.buf.clear();
                        self.checksum = true;
                    }
                    None
                }
                _ => None,
            }
        } else {
            if Self::verify_checksum(&self.cmd[self.state as usize].to_be_bytes(), input_byte) {
                self.checksum = false;
                let mut ack: Vec<u8, 5> = Vec::new();
                ack.push(0x0A).unwrap();
                if self.state == 2 {
                    self.state = 0
                } else if self.state == 1 && self.send {
                    self.send = false;
                    self.state = 0;
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
                 let mut nack = Vec::new();
                 nack.push(0xC4).unwrap();
                 Some(nack)
             }
        }
    }
    fn verify_checksum(data: &[u8], checksum: u8) -> bool {
        let mut buf = 0;
        for i in data.iter() {
            buf ^= i;
        }
        buf ^ checksum == 0xFF
    }
}
