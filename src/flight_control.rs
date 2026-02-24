// src/flight_control.rs
//
// Main flight control loop. Consumes gyro and RC data,
// runs the PID controllers, mixes outputs, and writes DShot frames.

use crate::config::FlightConfig;
use crate::dshot::DShotManager;
use crate::mixer::mix_motors;
use crate::pid::{PidControllers, Vector3};

/// CRSF channel range constants
const CRSF_CHANNEL_MIN: u16 = 172;
const CRSF_CHANNEL_MID: u16 = 992;
const CRSF_CHANNEL_MAX: u16 = 1811;

/// Arming constants
const ARM_CHANNEL: usize = 4; // AUX1 (channel index 4, 0-indexed)
const ARM_THRESHOLD: u16 = 1500;

/// Maximum rate in deg/s for stick deflection
const MAX_RATE_ROLL: f32 = 670.0;
const MAX_RATE_PITCH: f32 = 670.0;
const MAX_RATE_YAW: f32 = 400.0;

/// Maximum throttle value for the mixer
const MAX_THROTTLE: f32 = 2000.0;

/// Idle throttle percentage when armed (prevents motors from stopping mid-flight)
const IDLE_THROTTLE: f32 = 0.045; // 4.5%

/// Map a CRSF channel value (172-1811) to a symmetric range (-1.0 to 1.0)
fn map_channel_symmetric(value: u16) -> f32 {
    let clamped = value.max(CRSF_CHANNEL_MIN).min(CRSF_CHANNEL_MAX);
    let normalized = (clamped as f32 - CRSF_CHANNEL_MID as f32)
        / ((CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN) as f32 / 2.0);
    // Clamp to [-1, 1]
    normalized.max(-1.0).min(1.0)
}

/// Map a CRSF channel value (172-1811) to a range (0.0 to 1.0)
fn map_channel_range(value: u16) -> f32 {
    let clamped = value.max(CRSF_CHANNEL_MIN).min(CRSF_CHANNEL_MAX);
    let normalized =
        (clamped as f32 - CRSF_CHANNEL_MIN as f32) / (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN) as f32;
    normalized.max(0.0).min(1.0)
}

/// Check if the quad is armed based on RC channel data
fn is_armed(rc_channels: &[u16; 16]) -> bool {
    rc_channels[ARM_CHANNEL] > ARM_THRESHOLD
}

/// Convert RC stick inputs to rate setpoints (deg/s)
/// Channel order: AETR — [0]=Roll, [1]=Pitch, [2]=Throttle, [3]=Yaw
pub fn rc_to_setpoints(rc_channels: &[u16; 16]) -> (Vector3, f32) {
    let roll_stick = map_channel_symmetric(rc_channels[0]);
    let pitch_stick = map_channel_symmetric(rc_channels[1]);
    let throttle = map_channel_range(rc_channels[2]);
    let yaw_stick = map_channel_symmetric(rc_channels[3]);

    let setpoint = Vector3::new(
        roll_stick * MAX_RATE_ROLL,
        pitch_stick * MAX_RATE_PITCH,
        yaw_stick * MAX_RATE_YAW,
    );

    (setpoint, throttle)
}

/// Flight controller state machine
pub struct FlightController {
    pid: PidControllers,
    dshot: DShotManager,
    armed: bool,
    last_rc: [u16; 16],
}

impl FlightController {
    /// Create a new flight controller with config-derived PID gains
    /// and a pre-configured DShot manager for hardware output.
    pub fn new(config: &FlightConfig, dshot: DShotManager) -> Self {
        // Create PID controllers at gyro sample rate (8kHz)
        let mut pid = PidControllers::new(8000.0);

        // Apply gains from config
        config.apply_to_pid_controllers(&mut pid);

        // Set derivative lowpass filter (reduce D-term noise)
        pid.set_derivative_lpf(100.0, 8000.0);

        // Set output limits
        pid.set_output_limits(500.0);

        Self {
            pid,
            dshot,
            armed: false,
            last_rc: [CRSF_CHANNEL_MID; 16],
        }
    }

    /// Update RC channel data (called whenever new RC data arrives)
    pub fn update_rc(&mut self, rc_channels: [u16; 16]) {
        self.last_rc = rc_channels;
        self.armed = is_armed(&self.last_rc);
    }

    /// Run one iteration of the PID loop and send motor commands via DShot.
    ///
    /// `gyro_data` is [roll_rate, pitch_rate, yaw_rate] in deg/s from the filtered gyro.
    pub async fn update(&mut self, gyro_data: [f32; 3]) {
        if !self.armed {
            // Send zero throttle (disarmed) — DShot value 0 = motor stop
            self.pid.reset();
            self.dshot.write_throttles([0, 0, 0, 0]).await;
            return;
        }

        // Convert RC to setpoints
        let (setpoint, throttle) = rc_to_setpoints(&self.last_rc);

        // Gyro measurement as Vector3
        let measurement = Vector3::new(gyro_data[0], gyro_data[1], gyro_data[2]);

        // Run PID
        let pid_output = self.pid.calculate_rate(&setpoint, &measurement, throttle);

        // Calculate base throttle with idle offset
        let base_throttle = if throttle < 0.01 {
            // Stick at minimum — keep idle spin for airmode
            IDLE_THROTTLE * MAX_THROTTLE
        } else {
            throttle * MAX_THROTTLE
        };

        // Mix into motor outputs
        let motor_outputs = mix_motors(base_throttle, pid_output.x, pid_output.y, pid_output.z);

        // Send to ESCs via DShot DMA
        self.dshot.write_throttles(motor_outputs).await;
    }

    /// Initialize ESCs by sending zero-throttle DShot frames for ~600ms.
    /// Must be called before the main control loop so ESCs recognize the
    /// DShot600 signal and confirm connection with 2 beeps.
    pub async fn init_escs(&mut self) {
        self.dshot.init_escs().await;
    }

    /// Get mutable reference to PID controllers (for config updates)
    pub fn pid_mut(&mut self) -> &mut PidControllers {
        &mut self.pid
    }

    /// Check if armed
    pub fn is_armed(&self) -> bool {
        self.armed
    }

    /// Get latest eRPM period readings (µs) for all 4 motors.
    /// Returns `None` for motors without valid telemetry data.
    pub fn erpm(&self) -> [Option<u32>; 4] {
        self.dshot.get_erpm()
    }
}
