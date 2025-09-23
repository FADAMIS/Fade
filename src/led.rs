use stm32f7xx_hal::gpio::{Output, Pin, PushPull};

pub struct LedManager {
    led: Pin<'A', 14, Output<PushPull>>,
    counter: u32,
    on_duration: u32,
    off_duration: u32,
}

impl LedManager {
    /// Create a new LED manager with default blink timing
    /// The LED will be on for 100,000 cycles and off for 100,000 cycles
    pub fn new(led: Pin<'A', 14, Output<PushPull>>) -> Self {
        Self {
            led,
            counter: 0,
            on_duration: 100_000,
            off_duration: 100_000,
        }
    }

    /// Create a new LED manager with custom blink timing
    /// on_duration: number of cycles the LED stays on
    /// off_duration: number of cycles the LED stays off
    pub fn new_with_timing(
        led: Pin<'A', 14, Output<PushPull>>,
        on_duration: u32,
        off_duration: u32,
    ) -> Self {
        Self {
            led,
            counter: 0,
            on_duration,
            off_duration,
        }
    }

    /// Call this in your main loop to handle LED blinking
    pub fn update(&mut self) {
        let cycle_length = self.on_duration + self.off_duration;
        let position_in_cycle = self.counter % cycle_length;

        if position_in_cycle < self.on_duration {
            self.led.set_high();
        } else {
            self.led.set_low();
        }

        self.counter = self.counter.wrapping_add(1);
    }

    /// Turn LED on permanently
    pub fn turn_on(&mut self) {
        self.led.set_high();
    }

    /// Turn LED off permanently
    pub fn turn_off(&mut self) {
        self.led.set_low();
    }

    /// Change the blink timing
    pub fn set_timing(&mut self, on_duration: u32, off_duration: u32) {
        self.on_duration = on_duration;
        self.off_duration = off_duration;
    }
}
