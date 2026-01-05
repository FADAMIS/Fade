// src/dshot.rs

#![allow(dead_code)]

/// Represents the DShot speed and associated timings.
#[derive(Clone, Copy)]
pub enum DShotSpeed {
    DShot300,
    DShot600,
    DShot1200,
}

impl DShotSpeed {
    /// Returns the timer period (number of ticks for one bit) for a given timer frequency.
    fn bit_period_ticks(&self, timer_freq: u32) -> u32 {
        let period_ns = match self {
            DShotSpeed::DShot300 => 3333, // 3.33 µs
            DShotSpeed::DShot600 => 1667, // 1.67 µs
            DShotSpeed::DShot1200 => 833, // 0.83 µs
        };
        (timer_freq as u64 * period_ns as u64 / 1_000_000_000) as u32
    }

    /// Returns the number of timer ticks for a '0' bit's high time.
    fn t0h_ticks(&self, timer_freq: u32) -> u32 {
        // T0H is approx 37.5% of the bit period
        (self.bit_period_ticks(timer_freq) * 3) / 8
    }

    /// Returns the number of timer ticks for a '1' bit's high time.
    fn t1h_ticks(&self, timer_freq: u32) -> u32 {
        // T1H is approx 75% of the bit period
        (self.bit_period_ticks(timer_freq) * 6) / 8
    }
}

/// A structure to manage DShot communication for multiple ESCs.
/// This is a placeholder for the full hardware-specific implementation.
pub struct DShotManager {
    speed: DShotSpeed,
    timer_freq: u32,
    t0h: u32,
    t1h: u32,
    // Here you would have fields for the timer, DMA, and GPIO peripherals.
    // e.g., timer: embassy_stm32::peripherals::TIM1,
    // dma: embassy_stm32::peripherals::DMA1,
    // pins: [Output<'static, AnyPin>; 4]
}

impl DShotManager {
    /// Creates a new DShotManager.
    ///
    /// # Arguments
    ///
    /// * `speed` - The DShot speed to use.
    /// * `timer_freq` - The frequency of the timer used for PWM generation.
    pub fn new(speed: DShotSpeed, timer_freq: u32) -> Self {
        let t0h = speed.t0h_ticks(timer_freq);
        let t1h = speed.t1h_ticks(timer_freq);
        Self {
            speed,
            timer_freq,
            t0h,
            t1h,
        }
    }

    /// Prepares a 16-bit DShot frame from a throttle value.
    ///
    /// The frame is composed of:
    /// - 11 bits for throttle (48-2047)
    /// - 1 bit for telemetry request
    /// - 4 bits for checksum
    fn prepare_frame(throttle: u16, telemetry: bool) -> u16 {
        // Map input throttle (0-2000) to DShot range (48-2047).
        // 0 is reserved for disarmed, 1-47 for commands.
        let dshot_throttle = if throttle > 0 {
            48 + (throttle as u32 * (2047 - 48) / 2000) as u16
        } else {
            0
        };

        let mut frame: u16 = dshot_throttle << 1;
        if telemetry {
            frame |= 1;
        }

        let checksum = {
            let csum_data = (frame >> 4) ^ (frame >> 8) ^ (frame >> 12);
            csum_data & 0x0F
        };

        (frame << 4) | checksum
    }

    /// Encodes a 16-bit DShot frame into an array of timer compare values for DMA transfer.
    /// The buffer must have a length of 18 (16 bits + 2 reset bits).
    pub fn encode_frame(&self, frame: u16, buffer: &mut [u32; 18]) {
        for i in 0..16 {
            let bit = (frame >> (15 - i)) & 1;
            buffer[i] = if bit == 1 { self.t1h } else { self.t0h };
        }
        // Add two "low" bits at the end for separation between frames
        buffer[16] = 0;
        buffer[17] = 0;
    }

    /// Writes throttle values to the ESCs.
    /// In a real implementation, this would configure and start the DMA transfer.
    pub fn write_throttles(&self, throttles: [u16; 4], dma_buffers: &mut [[u32; 18]; 4]) {
        for (i, &throttle_val) in throttles.iter().enumerate() {
            // In a real flight controller, the telemetry bit might be enabled periodically.
            let telemetry_request = false;
            let frame = Self::prepare_frame(throttle_val, telemetry_request);
            self.encode_frame(frame, &mut dma_buffers[i]);
        }

        // TODO: Here would be the hardware-specific logic to start the DMA
        // transfer to the timer's CCR for each motor channel.
        // e.g., dma.start_transfer(timer.ccr(0), &dma_buffers[0]);
        // ... and so on for the other 3 channels.
    }
}

#[cfg(test)]
mod tests {

    #[test]
    fn test_prepare_frame_zero_throttle() {
        let frame = DShotManager::prepare_frame(0, false);
        // Throttle 0, Telemetry 0 -> Checksum 0
        assert_eq!(frame, 0b0000000000000000);
    }

    #[test]
    fn test_prepare_frame_min_throttle() {
        // Throttle 1 should map to 48
        let frame = DShotManager::prepare_frame(1, false);
        // Frame: [48 (0x30), T=0] -> 0x0300
        // Data for checksum: 0011 0000 0000
        // checksum = 0x3 ^ 0x0 ^ 0x0 = 0x3
        // Full frame: 0011 0000 0000 0011 -> 0x3003
        assert_eq!(frame, 0b0011000000000011);
    }

    #[test]
    fn test_prepare_frame_max_throttle() {
        // Throttle 2000 should map to 2047
        let frame = DShotManager::prepare_frame(2000, false);
        // Frame: [2047 (0x7FF), T=0] -> 0x7FF0
        // Data for checksum: 0111 1111 1111
        // checksum = 0x7 ^ 0xF ^ 0xF = 0x7
        // Full frame: 0111 1111 1111 0111 -> 0x7FF7
        assert_eq!(frame, 0b0111111111110111);
    }

    #[test]
    fn test_prepare_frame_mid_throttle_with_telemetry() {
        // Throttle 1000
        // dshot_throttle = 48 + (1000 * (2047-48) / 2000) = 48 + 999 = 1047
        let frame = DShotManager::prepare_frame(1000, true);
        // Frame: [1047 (0x417), T=1] -> 0x417 << 1 | 1 = 0x82E | 1 = 0x82F
        // Packet: 1000 0010 1111
        // Data for checksum: 1000 0010 111
        // checksum = 0x8 ^ 0x2 ^ 0xF = 0x5
        // Full frame: 1000 0010 1111 0101 -> 0x82F5
        assert_eq!(frame, 0b1000001011110101);
    }

    #[test]
    fn test_encode_frame() {
        let manager = DShotManager::new(DShotSpeed::DShot600, 120_000_000);
        let t0h = manager.t0h;
        let t1h = manager.t1h;

        let frame = 0b1010_0000_0000_0000; // A simple pattern
        let mut buffer = [0u32; 18];
        manager.encode_frame(frame, &mut buffer);

        assert_eq!(buffer[0], t1h);
        assert_eq!(buffer[1], t0h);
        assert_eq!(buffer[2], t1h);
        assert_eq!(buffer[3], t0h);

        for i in 4..16 {
            assert_eq!(buffer[i], t0h);
        }

        // trailing zeros
        assert_eq!(buffer[16], 0);
        assert_eq!(buffer[17], 0);
    }
}
