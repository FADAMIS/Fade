// src/mixer.rs

const MAX_MOTOR_OUTPUT: f32 = 2000.0;
const MIN_MOTOR_OUTPUT: f32 = 0.0;

/// Mixes throttle, roll, pitch, and yaw commands to individual motor outputs for a quadcopter.
///
/// # Arguments
///
/// * `throttle` - The base throttle level (e.g., 0.0 to 2000.0).
/// * `roll` - The desired roll rate in degrees per second.
/// * `pitch` - The desired pitch rate in degrees per second.
/// * `yaw` - The desired yaw rate in degrees per second.
///
/// # Returns
///
/// An array of four u16 values representing the power for each motor (front-right, rear-left, front-left, rear-right),
/// scaled to be within the [0, 2000] range.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
pub fn mix_motors(throttle: f32, roll: f32, pitch: f32, yaw: f32) -> [u16; 4] {
    // Standard X-configuration mixing
    let mut motor_fr = throttle - roll + pitch - yaw; // Motor 1 (Front Right)
    let mut motor_rl = throttle + roll - pitch - yaw; // Motor 2 (Rear Left)
    let mut motor_fl = throttle + roll + pitch + yaw; // Motor 3 (Front Left)
    let mut motor_rr = throttle - roll - pitch + yaw; // Motor 4 (Rear Right)

    // Find the maximum motor value
    let mut max_motor = motor_fr;
    if motor_rl > max_motor { max_motor = motor_rl; }
    if motor_fl > max_motor { max_motor = motor_fl; }
    if motor_rr > max_motor { max_motor = motor_rr; }

    // Scaling factor to bring all motor values within the valid range if necessary
    if max_motor > MAX_MOTOR_OUTPUT {
        let scale_factor = MAX_MOTOR_OUTPUT / max_motor;
        motor_fr *= scale_factor;
        motor_rl *= scale_factor;
        motor_fl *= scale_factor;
        motor_rr *= scale_factor;
    }

    // Clamp the values to the allowed range [0, 2000] and convert to u16
    let motor_outputs = [
        motor_fr.max(MIN_MOTOR_OUTPUT).min(MAX_MOTOR_OUTPUT) as u16,
        motor_rl.max(MIN_MOTOR_OUTPUT).min(MAX_MOTOR_OUTPUT) as u16,
        motor_fl.max(MIN_MOTOR_OUTPUT).min(MAX_MOTOR_OUTPUT) as u16,
        motor_rr.max(MIN_MOTOR_OUTPUT).min(MAX_MOTOR_OUTPUT) as u16,
    ];

    motor_outputs
}
