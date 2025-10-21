enum State {
    Start,
    Length,
    Data,
    Checksum,
}

struct NgChl2Parser {
    state: State,
}

impl NgChl2Parser {
    fn parse(&mut self, data: &[u8]) {
        match self.state {
            State::Start => {
                if data[0] == 0x55 {
                    self.state = State::Length;
                }
            }
            State::Length => {
                self.state = State::Data;
            }
            State::Data => {
                self.state = State::Checksum;
            }
            State::Checksum => {
                self.state = State::Start;
            }
        }
    }
}
