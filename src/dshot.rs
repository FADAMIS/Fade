// src/dshot.rs
//
// Bidirectional DShot600 protocol — hardware output via TIM3 + DMA on Sequire H7 V2.
//
// Transmit: SimplePwm::waveform_up_multi_channel() sends all 4 motor frames
//           simultaneously via DMA burst.
// Receive:  Round-robin telemetry — one motor per frame cycle gets its timer
//           channel switched to input capture via PAC registers. DMA captures
//           edge timestamps, which are decoded from GCR into eRPM values.

#![allow(dead_code)]

use embassy_stm32::peripherals;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::timer::Channel;
use embassy_stm32::Peri;
use embassy_time::{Duration, Timer};

#[cfg(feature = "stm32h7")]
use embassy_stm32::gpio::OutputType;

#[cfg(feature = "stm32h7")]
use embassy_stm32::timer::low_level::CountingMode;

/// Number of bits in a DShot frame
const DSHOT_FRAME_BITS: usize = 16;
/// We send 17 "slots": 16 data bits + 1 reset (low) slot
const DSHOT_BUFFER_LEN: usize = DSHOT_FRAME_BITS + 1;
/// Number of motors
const NUM_MOTORS: usize = 4;
/// Total DMA buffer length: 17 slots × 4 channels (interleaved)
const DMA_BUF_LEN: usize = DSHOT_BUFFER_LEN * NUM_MOTORS;

/// Number of edges to capture for a GCR telemetry frame.
/// 21-bit RLL frame = up to 21 edges + some margin
const CAPTURE_BUF_LEN: usize = 32;

// =================== GCR DECODE TABLE ===================
// Bidirectional DShot telemetry uses GCR (Group Code Recording) encoding.
// Each 4-bit nibble is encoded as a 5-bit GCR symbol for transmission.
// This table maps 5-bit GCR codes back to 4-bit nibble values.
// Invalid codes map to 0xFF.

/// GCR decode: 5-bit code → 4-bit nibble (0xFF = invalid)
const GCR_DECODE: [u8; 32] = [
    0xFF, // 0b00000
    0xFF, // 0b00001
    0xFF, // 0b00010
    0xFF, // 0b00011
    0xFF, // 0b00100
    0xFF, // 0b00101
    0xFF, // 0b00110
    0xFF, // 0b00111
    0xFF, // 0b01000
    0x09, // 0b01001
    0x0A, // 0b01010
    0x0B, // 0b01011
    0xFF, // 0b01100
    0x0D, // 0b01101
    0x0E, // 0b01110
    0x0F, // 0b01111
    0xFF, // 0b10000
    0xFF, // 0b10001
    0x02, // 0b10010
    0x03, // 0b10011
    0xFF, // 0b10100
    0x05, // 0b10101
    0x06, // 0b10110
    0x07, // 0b10111
    0xFF, // 0b11000
    0x00, // 0b11001
    0x08, // 0b11010
    0x01, // 0b11011
    0xFF, // 0b11100
    0x04, // 0b11101
    0x0C, // 0b11110
    0xFF, // 0b11111
];

// =================== FRAME ENCODING ===================

/// DShot speed variants
#[derive(Clone, Copy)]
pub enum DShotSpeed {
    DShot150,
    DShot300,
    DShot600,
    DShot1200,
}

impl DShotSpeed {
    /// Bit rate in bits/second
    pub fn bitrate(&self) -> u32 {
        match self {
            DShotSpeed::DShot150 => 150_000,
            DShotSpeed::DShot300 => 300_000,
            DShotSpeed::DShot600 => 600_000,
            DShotSpeed::DShot1200 => 1_200_000,
        }
    }
}

/// Prepares a 16-bit DShot frame with **normal** checksum (unidirectional mode).
///
/// Frame layout: [11-bit throttle | 1-bit telemetry | 4-bit CRC]
/// CRC = XOR of nibbles: (frame ^ frame>>4 ^ frame>>8) & 0x0F
fn prepare_frame(throttle: u16, telemetry: bool) -> u16 {
    let dshot_throttle = if throttle > 0 {
        48 + (throttle as u32 * (2047 - 48) / 2000) as u16
    } else {
        0
    };

    let mut frame: u16 = dshot_throttle << 1;
    if telemetry {
        frame |= 1;
    }

    let checksum = (frame ^ (frame >> 4) ^ (frame >> 8)) & 0x0F;
    (frame << 4) | checksum
}

/// Prepares a 16-bit DShot frame with **inverted** checksum (bidirectional mode).
///
/// BLHeli_32 ESCs detect bidirectional mode by the inverted CRC. When they see
/// an inverted CRC, they know to send back a GCR telemetry frame after each
/// DShot command.
///
/// CRC = ~(frame ^ frame>>4 ^ frame>>8) & 0x0F
fn prepare_frame_bidir(throttle: u16, telemetry: bool) -> u16 {
    let dshot_throttle = if throttle > 0 {
        48 + (throttle as u32 * (2047 - 48) / 2000) as u16
    } else {
        0
    };

    let mut frame: u16 = dshot_throttle << 1;
    if telemetry {
        frame |= 1;
    }

    // Inverted checksum signals bidirectional mode to the ESC
    let checksum = (!(frame ^ (frame >> 4) ^ (frame >> 8))) & 0x0F;
    (frame << 4) | checksum
}

// =================== GCR TELEMETRY DECODE ===================

/// Decode a 20-bit GCR-encoded value into a 16-bit payload.
///
/// The ESC sends 4 nibbles, each GCR-encoded as 5 bits = 20 bits total.
/// Bit order: MSB first, nibble 3 (upper) → nibble 0 (lower).
///
/// Returns `None` if any GCR code is invalid.
fn decode_gcr_payload(gcr_20bit: u32) -> Option<u16> {
    let n3 = GCR_DECODE[((gcr_20bit >> 15) & 0x1F) as usize];
    let n2 = GCR_DECODE[((gcr_20bit >> 10) & 0x1F) as usize];
    let n1 = GCR_DECODE[((gcr_20bit >> 5) & 0x1F) as usize];
    let n0 = GCR_DECODE[(gcr_20bit & 0x1F) as usize];

    if n3 == 0xFF || n2 == 0xFF || n1 == 0xFF || n0 == 0xFF {
        return None;
    }

    Some(((n3 as u16) << 12) | ((n2 as u16) << 8) | ((n1 as u16) << 4) | (n0 as u16))
}

/// Decode edge timestamps captured via DMA into a 20-bit GCR value.
///
/// The ESC response is RLL (Run-Length Limited) encoded: the bit stream is
/// derived from transitions (edges), not levels. At each bit position,
/// a transition = 1, no transition = 0. The first bit is always 1 (start bit).
///
/// `edges` contains raw timer counter values at each captured edge.
/// `bit_time` is the expected timer ticks per GCR bit (= TIM3 period * 4/5,
/// since GCR telemetry runs at 5/4 × DShot bitrate).
///
/// Returns the 20-bit GCR value (excluding the start bit), or `None` on decode failure.
fn edges_to_gcr(edges: &[u16], num_edges: usize, bit_time: u16) -> Option<u32> {
    if num_edges < 2 {
        return None;
    }

    let half_bit = bit_time / 2;
    let mut gcr_value: u32 = 0;
    let mut bit_count: u32 = 0;

    // First edge is the start bit (always 1) — skip it, start processing from second edge
    for i in 1..num_edges {
        // Delta between consecutive edges
        let delta = edges[i].wrapping_sub(edges[i - 1]);

        // How many bit periods fit in this delta?
        // Round: (delta + half_bit) / bit_time
        let periods = ((delta as u32) + (half_bit as u32)) / (bit_time as u32);

        if periods == 0 || periods > 4 {
            return None; // Invalid timing
        }

        // First period = transition (bit = 1), remaining periods = no transition (bit = 0)
        gcr_value = (gcr_value << 1) | 1; // transition bit
        bit_count += 1;

        for _ in 1..periods {
            gcr_value <<= 1; // no-transition bit = 0
            bit_count += 1;
        }

        if bit_count >= 20 {
            break;
        }
    }

    if bit_count < 20 {
        return None;
    }

    // Take the lower 20 bits
    Some(gcr_value & 0x000F_FFFF)
}

/// Decode a 16-bit telemetry payload into an eRPM period value.
///
/// Payload layout: [12-bit eRPM value | 4-bit CRC]
/// The 12-bit eRPM value is encoded as: [3-bit exponent | 9-bit mantissa]
/// eRPM period = mantissa << exponent  (in µs, electrical period)
///
/// CRC check: XOR of all 4 nibbles should equal 0x0F.
///
/// Returns eRPM period in µs, or `None` if CRC fails.
fn decode_erpm_payload(payload: u16) -> Option<u32> {
    // CRC verification: XOR all 4 nibbles
    let crc_check = (payload ^ (payload >> 4) ^ (payload >> 8) ^ (payload >> 12)) & 0x0F;
    if crc_check != 0x0F {
        return None;
    }

    let erpm_raw = (payload >> 4) & 0x0FFF; // upper 12 bits (without CRC)
    let exponent = (erpm_raw >> 9) & 0x07; // top 3 bits
    let mantissa = erpm_raw & 0x01FF; // bottom 9 bits

    Some((mantissa as u32) << exponent)
}

/// Full decode pipeline: edge timestamps → eRPM period (µs).
///
/// Combines edge-to-GCR, GCR-to-payload, payload-to-eRPM.
/// Returns `None` at any stage if decoding fails (invalid timing, bad GCR, bad CRC).
fn decode_telemetry(edges: &[u16], num_edges: usize, bit_time: u16) -> Option<u32> {
    let gcr = edges_to_gcr(edges, num_edges, bit_time)?;
    let payload = decode_gcr_payload(gcr)?;
    decode_erpm_payload(payload)
}

/// Convert eRPM period (µs) to RPM given motor pole count.
///
/// eRPM = 60_000_000 / period_us  (electrical RPM)
/// RPM  = eRPM * 2 / pole_count   (mechanical RPM)
///
/// Returns 0 if period is 0 (motor stopped / no signal).
pub fn erpm_to_rpm(period_us: u32, pole_count: u32) -> u32 {
    if period_us == 0 || pole_count == 0 {
        return 0;
    }
    (60_000_000 / period_us) * 2 / pole_count
}

// =================== HARDWARE DRIVER (H7) ===================

/// Bidirectional DShot manager — TIM3 + DMA on Sequire H7 V2.
///
/// Sends DShot frames to all 4 motors and reads eRPM telemetry back
/// from one motor per frame cycle (round-robin).
///
///   M1 = PB4 / TIM3_CH1
///   M2 = PB5 / TIM3_CH2
///   M3 = PB0 / TIM3_CH3
///   M4 = PB1 / TIM3_CH4
#[cfg(feature = "stm32h7")]
pub struct DShotManager {
    pwm: SimplePwm<'static, peripherals::TIM3>,
    dma: Peri<'static, peripherals::DMA1_CH4>,
    /// Duty cycle value representing a '0' bit (~37.5% of max)
    t0h: u16,
    /// Duty cycle value representing a '1' bit (~75% of max)
    t1h: u16,
    /// DMA buffer: 17 slots × 4 channels interleaved
    dma_buf: [u16; DMA_BUF_LEN],
    /// Latest eRPM period (µs) for each motor. None = no valid reading yet.
    erpm: [Option<u32>; NUM_MOTORS],
    /// Round-robin index: which motor gets telemetry read this cycle
    telem_motor: usize,
    /// DMA capture buffer for input capture edge timestamps
    capture_buf: [u16; CAPTURE_BUF_LEN],
    /// Expected timer ticks per GCR bit (for telemetry decode)
    gcr_bit_time: u16,
    /// Whether bidirectional mode is enabled
    bidirectional: bool,
}

#[cfg(feature = "stm32h7")]
impl DShotManager {
    /// Create a new bidirectional DShot manager.
    ///
    /// Configures TIM3 for DShot600 (or other speed) PWM output on all 4 channels.
    /// Set `bidirectional` to `true` to enable eRPM telemetry reading.
    pub fn new(
        tim: Peri<'static, peripherals::TIM3>,
        m1: Peri<'static, peripherals::PB4>,
        m2: Peri<'static, peripherals::PB5>,
        m3: Peri<'static, peripherals::PB0>,
        m4: Peri<'static, peripherals::PB1>,
        dma: Peri<'static, peripherals::DMA1_CH4>,
        speed: DShotSpeed,
    ) -> Self {
        let bit_freq = Hertz(speed.bitrate());

        let mut pwm = SimplePwm::new(
            tim,
            Some(PwmPin::new(m1, OutputType::PushPull)),
            Some(PwmPin::new(m2, OutputType::PushPull)),
            Some(PwmPin::new(m3, OutputType::PushPull)),
            Some(PwmPin::new(m4, OutputType::PushPull)),
            bit_freq,
            CountingMode::EdgeAlignedUp,
        );

        let max_duty = pwm.max_duty_cycle();

        // DShot duty cycles:
        // '0' = ~37.5% duty
        // '1' = ~75% duty
        let t0h = (max_duty as u32 * 3 / 8) as u16;
        let t1h = (max_duty as u32 * 3 / 4) as u16;

        // GCR telemetry bit time: telemetry is sent at 5/4 × DShot bitrate.
        // Timer ARR = max_duty (one DShot bit period in ticks).
        // GCR bit period = DShot bit period × 4/5 = max_duty × 4 / 5
        let gcr_bit_time = (max_duty as u32 * 4 / 5) as u16;

        // Enable all channels
        pwm.channel(Channel::Ch1).enable();
        pwm.channel(Channel::Ch2).enable();
        pwm.channel(Channel::Ch3).enable();
        pwm.channel(Channel::Ch4).enable();

        // Set initial duty to 0 (line low)
        pwm.channel(Channel::Ch1).set_duty_cycle(0);
        pwm.channel(Channel::Ch2).set_duty_cycle(0);
        pwm.channel(Channel::Ch3).set_duty_cycle(0);
        pwm.channel(Channel::Ch4).set_duty_cycle(0);

        Self {
            pwm,
            dma,
            t0h,
            t1h,
            dma_buf: [0u16; DMA_BUF_LEN],
            erpm: [None; NUM_MOTORS],
            telem_motor: 0,
            capture_buf: [0u16; CAPTURE_BUF_LEN],
            gcr_bit_time,
            bidirectional: false,
        }
    }

    /// Enable or disable bidirectional DShot mode.
    ///
    /// When enabled, frames use inverted CRC (signals ESC to send telemetry)
    /// and one motor per cycle gets eRPM telemetry read back.
    pub fn set_bidirectional(&mut self, enabled: bool) {
        self.bidirectional = enabled;
    }

    /// Get latest eRPM period readings (µs) for all 4 motors.
    ///
    /// Returns `None` for motors that haven't been read yet or had decode errors.
    pub fn get_erpm(&self) -> [Option<u32>; NUM_MOTORS] {
        self.erpm
    }

    /// Encode a 16-bit DShot frame into duty cycle values for one channel.
    fn encode_frame(&self, frame: u16) -> [u16; DSHOT_BUFFER_LEN] {
        let mut buf = [0u16; DSHOT_BUFFER_LEN];
        for i in 0..DSHOT_FRAME_BITS {
            let bit = (frame >> (15 - i)) & 1;
            buf[i] = if bit == 1 { self.t1h } else { self.t0h };
        }
        buf[DSHOT_BUFFER_LEN - 1] = 0;
        buf
    }

    /// Send throttle values to all 4 ESCs via DMA.
    ///
    /// In bidirectional mode, uses inverted CRC and reads telemetry from
    /// one motor per cycle (round-robin).
    pub async fn write_throttles(&mut self, throttles: [u16; 4]) {
        // Encode frames — use inverted CRC in bidirectional mode
        let frames: [u16; 4] = if self.bidirectional {
            [
                prepare_frame_bidir(throttles[0], true),
                prepare_frame_bidir(throttles[1], true),
                prepare_frame_bidir(throttles[2], true),
                prepare_frame_bidir(throttles[3], true),
            ]
        } else {
            [
                prepare_frame(throttles[0], false),
                prepare_frame(throttles[1], false),
                prepare_frame(throttles[2], false),
                prepare_frame(throttles[3], false),
            ]
        };

        let encoded: [[u16; DSHOT_BUFFER_LEN]; 4] = [
            self.encode_frame(frames[0]),
            self.encode_frame(frames[1]),
            self.encode_frame(frames[2]),
            self.encode_frame(frames[3]),
        ];

        // Interleave into DMA buffer
        for slot in 0..DSHOT_BUFFER_LEN {
            self.dma_buf[slot * NUM_MOTORS + 0] = encoded[0][slot];
            self.dma_buf[slot * NUM_MOTORS + 1] = encoded[1][slot];
            self.dma_buf[slot * NUM_MOTORS + 2] = encoded[2][slot];
            self.dma_buf[slot * NUM_MOTORS + 3] = encoded[3][slot];
        }

        // Send via DMA burst
        self.pwm
            .waveform_up_multi_channel(
                self.dma.reborrow(),
                Channel::Ch1,
                Channel::Ch4,
                &self.dma_buf,
            )
            .await;

        // Pull all lines low after frame transmission
        self.pwm.channel(Channel::Ch1).set_duty_cycle(0);
        self.pwm.channel(Channel::Ch2).set_duty_cycle(0);
        self.pwm.channel(Channel::Ch3).set_duty_cycle(0);
        self.pwm.channel(Channel::Ch4).set_duty_cycle(0);

        // ── Bidirectional telemetry: read response from one motor ──
        if self.bidirectional {
            let motor = self.telem_motor;
            let _channel = match motor {
                0 => Channel::Ch1,
                1 => Channel::Ch2,
                2 => Channel::Ch3,
                _ => Channel::Ch4,
            };

            // Wait ~30µs for ESC to start responding
            Timer::after(Duration::from_micros(30)).await;

            // Capture telemetry via PAC-level input capture + DMA
            let erpm = self.capture_telemetry(motor).await;
            self.erpm[motor] = erpm;

            // Advance round-robin
            self.telem_motor = (self.telem_motor + 1) % NUM_MOTORS;
        }
    }

    /// Capture telemetry from one motor channel using PAC register access.
    ///
    /// Switches the motor's timer channel from PWM output to input capture,
    /// captures edge timestamps via DMA, decodes GCR → eRPM, then restores
    /// the channel to PWM output mode.
    async fn capture_telemetry(&mut self, motor: usize) -> Option<u32> {
        use embassy_stm32::pac;

        let tim = pac::TIM3;
        let channel_idx = motor; // 0=CH1, 1=CH2, 2=CH3, 3=CH4

        // ── Step 1: Switch GPIO pin to input (floating) ──
        // The motor pins are PB4, PB5, PB0, PB1.
        // We need to change MODER from AF (0b10) to Input (0b00).
        let gpiob = pac::GPIOB;
        let pin_num: usize = match motor {
            0 => 4, // PB4
            1 => 5, // PB5
            2 => 0, // PB0
            _ => 1, // PB1
        };

        // Save original MODER value for this pin and switch to input
        let moder_val = gpiob.moder().read().moder(pin_num);
        gpiob
            .moder()
            .modify(|r| r.set_moder(pin_num, embassy_stm32::pac::gpio::vals::Moder::INPUT));

        // Enable pull-up (bidirectional DShot signal idles high)
        let pupdr_val = gpiob.pupdr().read().pupdr(pin_num);
        gpiob
            .pupdr()
            .modify(|r| r.set_pupdr(pin_num, embassy_stm32::pac::gpio::vals::Pupdr::PULL_UP));

        // ── Step 2: Switch timer channel to input capture ──
        // Disable capture/compare for this channel first
        tim.ccer().modify(|r| {
            r.set_cce(channel_idx, false);
        });

        // Configure input capture: CC*S = 01 (IC mapped to TI*), no filter, no prescaler
        match channel_idx {
            0 => {
                tim.ccmr_input(0).modify(|r| {
                    r.set_ccs(0, embassy_stm32::pac::timer::vals::CcmrInputCcs::TI4);
                });
            }
            1 => {
                tim.ccmr_input(0).modify(|r| {
                    r.set_ccs(1, embassy_stm32::pac::timer::vals::CcmrInputCcs::TI4);
                });
            }
            2 => {
                tim.ccmr_input(1).modify(|r| {
                    r.set_ccs(0, embassy_stm32::pac::timer::vals::CcmrInputCcs::TI4);
                });
            }
            _ => {
                tim.ccmr_input(1).modify(|r| {
                    r.set_ccs(1, embassy_stm32::pac::timer::vals::CcmrInputCcs::TI4);
                });
            }
        }

        // Capture on both edges (for edge timing)
        tim.ccer().modify(|r| {
            r.set_cce(channel_idx, true);
            r.set_ccnp(channel_idx, true); // Both edges: CCxP=1, CCxNP=1
            r.set_ccp(channel_idx, true);
        });

        // Clear any pending capture
        tim.sr().modify(|r| r.set_ccif(channel_idx, false));

        // ── Step 3: Poll-capture edges (no DMA for single-channel capture) ──
        // We capture up to CAPTURE_BUF_LEN edge timestamps by polling the CC flag.
        // This runs in a tight loop with a timeout. At 192 MHz timer clock and
        // ~1.3µs per GCR bit, the entire 21-bit response takes ~28µs.
        let mut num_edges: usize = 0;
        let timeout_ticks: u32 = 20_000; // ~100µs at 192 MHz (generous timeout)
        let start_cnt = tim.cnt().read().0;

        while num_edges < CAPTURE_BUF_LEN {
            // Check if we've timed out
            let now = tim.cnt().read().0;
            if now.wrapping_sub(start_cnt) > timeout_ticks {
                break;
            }

            // Check capture flag
            if tim.sr().read().ccif(channel_idx) {
                // Read captured value
                self.capture_buf[num_edges] = tim.ccr(channel_idx).read().0 as u16;
                num_edges += 1;

                // Clear flag
                tim.sr().modify(|r| r.set_ccif(channel_idx, false));
            }
        }

        // ── Step 4: Restore channel to PWM output mode ──
        tim.ccer().modify(|r| {
            r.set_cce(channel_idx, false);
        });

        // Restore CC*S = 00 (output compare mode)
        match channel_idx {
            0 => {
                tim.ccmr_output(0).modify(|r| {
                    r.set_ccs(0, embassy_stm32::pac::timer::vals::CcmrOutputCcs::OUTPUT);
                });
            }
            1 => {
                tim.ccmr_output(0).modify(|r| {
                    r.set_ccs(1, embassy_stm32::pac::timer::vals::CcmrOutputCcs::OUTPUT);
                });
            }
            2 => {
                tim.ccmr_output(1).modify(|r| {
                    r.set_ccs(0, embassy_stm32::pac::timer::vals::CcmrOutputCcs::OUTPUT);
                });
            }
            _ => {
                tim.ccmr_output(1).modify(|r| {
                    r.set_ccs(1, embassy_stm32::pac::timer::vals::CcmrOutputCcs::OUTPUT);
                });
            }
        }

        // Re-enable channel in output mode
        tim.ccer().modify(|r| {
            r.set_cce(channel_idx, true);
            r.set_ccnp(channel_idx, false);
            r.set_ccp(channel_idx, false);
        });

        // Restore GPIO to alternate function (push-pull output)
        gpiob.moder().modify(|r| r.set_moder(pin_num, moder_val));
        gpiob.pupdr().modify(|r| r.set_pupdr(pin_num, pupdr_val));

        // ── Step 5: Decode captured edges ──
        if num_edges < 3 {
            return None; // Not enough edges for a valid frame
        }

        decode_telemetry(&self.capture_buf, num_edges, self.gcr_bit_time)
    }

    /// Initialize ESCs by sending zero-throttle DShot frames for ~600ms.
    ///
    /// Uses normal (non-bidirectional) frames during init so ESCs recognize
    /// the DShot protocol regardless of bidirectional mode setting.
    pub async fn init_escs(&mut self) {
        let was_bidir = self.bidirectional;
        self.bidirectional = false;

        for _ in 0..600 {
            self.write_throttles([0, 0, 0, 0]).await;
            Timer::after(Duration::from_millis(1)).await;
        }

        self.bidirectional = was_bidir;
    }
}

// =================== STUB FOR F722 ===================
#[cfg(feature = "stm32f")]
pub struct DShotManager {
    t0h: u16,
    t1h: u16,
}

#[cfg(feature = "stm32f")]
impl DShotManager {
    pub fn new_stub() -> Self {
        Self { t0h: 0, t1h: 0 }
    }

    pub async fn write_throttles(&mut self, _throttles: [u16; 4]) {}

    pub async fn init_escs(&mut self) {}

    pub fn set_bidirectional(&mut self, _enabled: bool) {}

    pub fn get_erpm(&self) -> [Option<u32>; 4] {
        [None; 4]
    }
}

// =================== TESTS ===================

#[cfg(test)]
mod tests {

    // ── Frame encoding tests ──

    #[test]
    fn test_prepare_frame_zero_throttle() {
        let frame = prepare_frame(0, false);
        assert_eq!(frame, 0b0000000000000000);
    }

    #[test]
    fn test_prepare_frame_min_throttle() {
        let frame = prepare_frame(1, false);
        assert_eq!(frame, 0b0011000000000011);
    }

    #[test]
    fn test_prepare_frame_max_throttle() {
        let frame = prepare_frame(2000, false);
        assert_eq!(frame, 0b0111111111110111);
    }

    #[test]
    fn test_prepare_frame_mid_throttle_with_telemetry() {
        let frame = prepare_frame(1000, true);
        assert_eq!(frame, 0b1000001011110101);
    }

    // ── Bidirectional CRC tests ──

    #[test]
    fn test_bidir_crc_differs_from_normal() {
        let normal = prepare_frame(1000, false);
        let bidir = prepare_frame_bidir(1000, false);
        // Upper 12 bits (throttle + telemetry) should be identical
        assert_eq!(normal >> 4, bidir >> 4);
        // CRC (lower 4 bits) should be inverted
        assert_eq!((normal & 0x0F) ^ (bidir & 0x0F), 0x0F);
    }

    #[test]
    fn test_bidir_zero_throttle() {
        let bidir = prepare_frame_bidir(0, false);
        // Zero throttle + no telemetry = all zero upper bits → CRC = ~0 & 0xF = 0xF
        assert_eq!(bidir & 0x0F, 0x0F);
    }

    // ── GCR decode tests ──

    #[test]
    fn test_gcr_decode_all_valid() {
        // Verify all 16 valid GCR codes decode correctly
        let gcr_encode: [(u8, u8); 16] = [
            (0x0, 0x19), // 0 → 11001
            (0x1, 0x1B), // 1 → 11011
            (0x2, 0x12), // 2 → 10010
            (0x3, 0x13), // 3 → 10011
            (0x4, 0x1D), // 4 → 11101
            (0x5, 0x15), // 5 → 10101
            (0x6, 0x16), // 6 → 10110
            (0x7, 0x17), // 7 → 10111
            (0x8, 0x1A), // 8 → 11010
            (0x9, 0x09), // 9 → 01001
            (0xA, 0x0A), // A → 01010
            (0xB, 0x0B), // B → 01011
            (0xC, 0x1E), // C → 11110
            (0xD, 0x0D), // D → 01101
            (0xE, 0x0E), // E → 01110
            (0xF, 0x0F), // F → 01111
        ];

        for (nibble, code) in gcr_encode {
            assert_eq!(
                GCR_DECODE[code as usize], nibble,
                "GCR code 0x{:02X} should decode to 0x{:X}",
                code, nibble
            );
        }
    }

    #[test]
    fn test_gcr_decode_invalid_codes() {
        // Codes 0-8, 16-17, 20, 24, 28, 31 should be invalid
        let invalid_codes: [u8; 12] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 16, 17, 31];
        for code in invalid_codes {
            assert_eq!(
                GCR_DECODE[code as usize], 0xFF,
                "GCR code {} should be invalid",
                code
            );
        }
    }

    #[test]
    fn test_decode_gcr_payload_known() {
        // Encode 0xABCD as GCR: A=01010, B=01011, C=11110, D=01101
        // 20-bit GCR = 01010_01011_11110_01101 = 0x52F8D... let me compute:
        // A(0xA)=0b01010, B(0xB)=0b01011, C(0xC)=0b11110, D(0xD)=0b01101
        let gcr_20 = (0b01010 << 15) | (0b01011 << 10) | (0b11110 << 5) | 0b01101;
        let decoded = decode_gcr_payload(gcr_20);
        assert_eq!(decoded, Some(0xABCD));
    }

    #[test]
    fn test_decode_gcr_payload_all_zeros() {
        // 0x0000: 0=11001 → 11001_11001_11001_11001
        let gcr_20 = (0b11001 << 15) | (0b11001 << 10) | (0b11001 << 5) | 0b11001;
        let decoded = decode_gcr_payload(gcr_20);
        assert_eq!(decoded, Some(0x0000));
    }

    #[test]
    fn test_decode_gcr_payload_invalid() {
        // Use invalid GCR code 0b00000 in the first nibble
        let gcr_20 = (0b00000 << 15) | (0b11001 << 10) | (0b11001 << 5) | 0b11001;
        let decoded = decode_gcr_payload(gcr_20);
        assert_eq!(decoded, None);
    }

    // ── eRPM decode tests ──

    #[test]
    fn test_decode_erpm_payload_valid() {
        // Construct a valid payload: exponent=0, mantissa=500
        // erpm_raw = (0 << 9) | 500 = 500 = 0x1F4
        // payload_upper = erpm_raw = 0x1F4
        // CRC must make XOR of all nibbles = 0x0F
        // nibbles: 0x1, 0xF, 0x4, CRC
        // XOR: 0x1 ^ 0xF ^ 0x4 ^ CRC = 0x0F → CRC = 0x1 ^ 0xF ^ 0x4 ^ 0x0F = 0x1 ^ 0xF ^ 0x4 ^ 0xF
        //     = 0x1 ^ 0x4 = 0x5? Let me recalc:
        //     0x1 ^ 0xF = 0xE, 0xE ^ 0x4 = 0xA, 0xA ^ 0xF = 0x5
        let crc = 0x5;
        let payload: u16 = (0x1F4 << 4) | crc;
        let result = decode_erpm_payload(payload);
        assert_eq!(result, Some(500)); // mantissa=500, exponent=0 → 500 << 0 = 500
    }

    #[test]
    fn test_decode_erpm_payload_with_exponent() {
        // exponent=2, mantissa=100 = 0x064
        // erpm_raw = (2 << 9) | 100 = 0x400 | 0x64 = 0x464
        // nibbles: 0x4, 0x6, 0x4, CRC
        // 0x4 ^ 0x6 ^ 0x4 ^ CRC = 0x0F → 0x6 ^ CRC = 0x0F → CRC = 0x9
        let crc = 0x9;
        let payload: u16 = (0x464 << 4) | crc;
        let result = decode_erpm_payload(payload);
        assert_eq!(result, Some(100 << 2)); // = 400
    }

    #[test]
    fn test_decode_erpm_payload_bad_crc() {
        // Valid payload but wrong CRC
        let payload: u16 = (0x1F4 << 4) | 0x0; // wrong CRC
        let result = decode_erpm_payload(payload);
        assert_eq!(result, None);
    }

    // ── Edge-to-GCR tests ──

    #[test]
    fn test_edges_to_gcr_simple() {
        // Simulate edges where every bit is a transition (all 1s)
        // For 21 bits (1 start + 20 data), all transitions means 21 edges
        // each separated by exactly bit_time ticks
        let bit_time: u16 = 256; // arbitrary
        let mut edges = [0u16; 22];
        for i in 0..22 {
            edges[i] = (i as u16) * bit_time;
        }
        // All transitions → all 1s → GCR value = 0xFFFFF (20 bits)
        let result = edges_to_gcr(&edges, 21, bit_time);
        assert_eq!(result, Some(0xFFFFF));
    }

    #[test]
    fn test_edges_to_gcr_with_gaps() {
        // Simulate: 1, 0, 1, 1 (4 bits after start)
        // Edge 0 at t=0 (start bit)
        // Edge 1 at t=2*bit_time (1 transition after 2 periods → bit=1, then 0)
        // Edge 2 at t=3*bit_time (1 transition after 1 period → bit=1)
        // Edge 3 at t=4*bit_time (1 transition after 1 period → bit=1)
        let bit_time: u16 = 100;
        let edges = [0, 200, 300, 400];
        let result = edges_to_gcr(&edges, 4, bit_time);
        // After start (edge 0):
        //   edge 1: delta=200, periods=2 → bits: 1, 0
        //   edge 2: delta=100, periods=1 → bits: 1
        //   edge 3: delta=100, periods=1 → bits: 1
        // Total data bits: 1, 0, 1, 1 = 4 bits
        // But we need 20 bits, so this should return None
        assert_eq!(result, None);
    }

    #[test]
    fn test_edges_too_few() {
        let edges = [0u16; 1];
        assert_eq!(edges_to_gcr(&edges, 1, 100), None);
    }

    // ── erpm_to_rpm tests ──

    #[test]
    fn test_erpm_to_rpm_14_pole() {
        // 14-pole motor, eRPM period = 100µs
        // eRPM = 60_000_000 / 100 = 600_000
        // RPM = 600_000 * 2 / 14 = 85_714
        let rpm = erpm_to_rpm(100, 14);
        assert_eq!(rpm, 85714);
    }

    #[test]
    fn test_erpm_to_rpm_zero_period() {
        assert_eq!(erpm_to_rpm(0, 14), 0);
    }

    #[test]
    fn test_erpm_to_rpm_zero_poles() {
        assert_eq!(erpm_to_rpm(100, 0), 0);
    }
}
