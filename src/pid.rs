#[derive(Debug, Clone, Copy)]
pub enum Axis {
    Roll = 0,
    Pitch = 1,
    Yaw = 2,
}

#[derive(Debug, Clone, Copy)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vector3 {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub fn zero() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }
}

/// Betaflight-style PID controller with advanced features
#[derive(Debug, Clone)]
pub struct PidController {
    // PID gains
    kp: f32,
    ki: f32,
    kd: f32,

    // Output limits
    output_limit: f32,
    integrator_limit: f32,

    // State variables
    integrator: f32,
    last_error: f32,
    last_derivative: f32,

    // Advanced Betaflight features
    derivative_lpf_cutoff: f32,
    derivative_lpf_alpha: f32,

    // Anti-windup and other features
    feed_forward_gain: f32,
    setpoint_weight: f32, // For setpoint weighting (reduces overshoot)

    // TPA (Throttle PID Attenuation) - reduces PID gains at high throttle
    tpa_breakpoint: f32,
    tpa_rate: f32,

    // Dynamic filtering
    error_lpf_cutoff: f32,
    error_lpf_alpha: f32,
    filtered_error: f32,

    // Rate vs Angle mode
    is_angle_mode: bool,
    angle_limit: f32, // Max angle in degrees for angle mode

    // Sample time
    dt: f32,

    // Reset detection
    first_run: bool,
}

impl PidController {
    pub fn new(sample_rate: f32) -> Self {
        let dt = 1.0 / sample_rate;
        Self {
            kp: 0.0,
            ki: 0.0,
            kd: 0.0,
            output_limit: 500.0, // Betaflight default is around 500
            integrator_limit: 100.0,
            integrator: 0.0,
            last_error: 0.0,
            last_derivative: 0.0,
            derivative_lpf_cutoff: 100.0, // 100Hz default
            derivative_lpf_alpha: 0.0,
            feed_forward_gain: 0.0,
            setpoint_weight: 1.0,  // 1.0 = full setpoint weighting
            tpa_breakpoint: 0.7,   // Start TPA at 70% throttle
            tpa_rate: 0.0,         // 0 = no TPA
            error_lpf_cutoff: 0.0, // 0 = disabled
            error_lpf_alpha: 0.0,
            filtered_error: 0.0,
            is_angle_mode: false,
            angle_limit: 55.0, // 55 degrees max angle
            dt,
            first_run: true,
        }
    }

    pub fn set_gains(&mut self, kp: f32, ki: f32, kd: f32) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }

    pub fn set_output_limit(&mut self, limit: f32) {
        self.output_limit = limit;
    }

    pub fn set_integrator_limit(&mut self, limit: f32) {
        self.integrator_limit = limit;
    }

    pub fn set_derivative_lpf(&mut self, cutoff_hz: f32, sample_rate: f32) {
        self.derivative_lpf_cutoff = cutoff_hz;
        if cutoff_hz > 0.0 {
            let omega = 2.0 * core::f32::consts::PI * cutoff_hz / sample_rate;
            self.derivative_lpf_alpha = omega / (omega + 1.0);
        } else {
            self.derivative_lpf_alpha = 1.0; // No filtering
        }
    }

    pub fn set_error_lpf(&mut self, cutoff_hz: f32, sample_rate: f32) {
        self.error_lpf_cutoff = cutoff_hz;
        if cutoff_hz > 0.0 {
            let omega = 2.0 * core::f32::consts::PI * cutoff_hz / sample_rate;
            self.error_lpf_alpha = omega / (omega + 1.0);
        } else {
            self.error_lpf_alpha = 1.0; // No filtering
        }
    }

    pub fn set_feedforward(&mut self, ff_gain: f32) {
        self.feed_forward_gain = ff_gain;
    }

    pub fn set_setpoint_weight(&mut self, weight: f32) {
        self.setpoint_weight = weight.max(0.0).min(1.0);
    }

    pub fn set_tpa(&mut self, breakpoint: f32, rate: f32) {
        self.tpa_breakpoint = breakpoint.max(0.0).min(1.0);
        self.tpa_rate = rate.max(0.0).min(1.0);
    }

    pub fn set_angle_mode(&mut self, enabled: bool, max_angle: f32) {
        self.is_angle_mode = enabled;
        self.angle_limit = max_angle;
    }

    pub fn reset(&mut self) {
        self.integrator = 0.0;
        self.last_error = 0.0;
        self.last_derivative = 0.0;
        self.filtered_error = 0.0;
        self.first_run = true;
    }

    /// Calculate PID output
    /// setpoint: desired value (angle in degrees for angle mode, rate in deg/s for rate mode)
    /// measurement: current value (gyro rate in deg/s)
    /// throttle: current throttle value (0.0 to 1.0) for TPA
    /// current_angle: current angle in degrees (only needed for angle mode)
    pub fn calculate(
        &mut self,
        setpoint: f32,
        measurement: f32,
        throttle: f32,
        current_angle: Option<f32>,
    ) -> f32 {
        // Handle angle mode conversion
        let actual_setpoint = if self.is_angle_mode {
            if let Some(angle) = current_angle {
                // Convert angle setpoint to rate setpoint
                let angle_error = setpoint - angle;
                // Limit angle error to prevent excessive rates
                let limited_angle_error = angle_error.max(-self.angle_limit).min(self.angle_limit);
                // Convert angle error to desired rate (simple P controller for outer loop)
                limited_angle_error * self.kp * 0.1 // Scale factor for angle to rate conversion
            } else {
                0.0 // No angle feedback available
            }
        } else {
            setpoint
        };

        // Calculate error
        let mut error = actual_setpoint - measurement;

        // Apply error low-pass filter if enabled
        if self.error_lpf_cutoff > 0.0 {
            if self.first_run {
                self.filtered_error = error;
            } else {
                self.filtered_error = self.filtered_error * (1.0 - self.error_lpf_alpha)
                    + error * self.error_lpf_alpha;
            }
            error = self.filtered_error;
        }

        // Calculate TPA (Throttle PID Attenuation)
        let tpa_factor = if throttle > self.tpa_breakpoint && self.tpa_rate > 0.0 {
            1.0 - (throttle - self.tpa_breakpoint) * self.tpa_rate / (1.0 - self.tpa_breakpoint)
        } else {
            1.0
        };

        // Apply TPA to gains
        let effective_kp = self.kp * tpa_factor;
        let effective_ki = self.ki * tpa_factor;
        let effective_kd = self.kd * tpa_factor;

        // Proportional term with setpoint weighting
        let proportional = if self.setpoint_weight < 1.0 {
            effective_kp * (self.setpoint_weight * actual_setpoint - measurement)
        } else {
            effective_kp * error
        };

        // Integral term
        if !self.first_run {
            self.integrator += error * self.dt;

            // Anti-windup: limit integrator
            self.integrator = self
                .integrator
                .max(-self.integrator_limit)
                .min(self.integrator_limit);
        }
        let integral = effective_ki * self.integrator;

        // Derivative term (derivative on measurement to avoid derivative kick)
        let derivative_input = -measurement; // Derivative on measurement, not error
        let mut derivative = if self.first_run {
            0.0
        } else {
            (derivative_input - self.last_error) / self.dt
        };

        // Apply derivative low-pass filter
        if self.derivative_lpf_alpha < 1.0 {
            if !self.first_run {
                derivative = self.last_derivative * (1.0 - self.derivative_lpf_alpha)
                    + derivative * self.derivative_lpf_alpha;
            }
            self.last_derivative = derivative;
        }

        let derivative_output = effective_kd * derivative;

        // Feedforward term (based on setpoint rate of change)
        let feedforward = self.feed_forward_gain * actual_setpoint;

        // Combine all terms
        let mut output = proportional + integral + derivative_output + feedforward;

        // Apply output limits
        output = output.max(-self.output_limit).min(self.output_limit);

        // Anti-windup: reduce integrator if output is saturated
        if (output >= self.output_limit || output <= -self.output_limit)
            && (error * self.integrator > 0.0)
        {
            self.integrator *= 0.9; // Reduce integrator when saturated
        }

        // Store values for next iteration
        self.last_error = derivative_input;
        self.first_run = false;

        output
    }
}

/// Three-axis PID controller system
#[derive(Debug, Clone)]
pub struct PidControllers {
    pub roll: PidController,
    pub pitch: PidController,
    pub yaw: PidController,
}

impl PidControllers {
    pub fn new(sample_rate: f32) -> Self {
        Self {
            roll: PidController::new(sample_rate),
            pitch: PidController::new(sample_rate),
            yaw: PidController::new(sample_rate),
        }
    }

    pub fn set_gains(&mut self, axis: Axis, kp: f32, ki: f32, kd: f32) {
        match axis {
            Axis::Roll => self.roll.set_gains(kp, ki, kd),
            Axis::Pitch => self.pitch.set_gains(kp, ki, kd),
            Axis::Yaw => self.yaw.set_gains(kp, ki, kd),
        }
    }

    pub fn set_output_limits(&mut self, limit: f32) {
        self.roll.set_output_limit(limit);
        self.pitch.set_output_limit(limit);
        self.yaw.set_output_limit(limit);
    }

    pub fn set_derivative_lpf(&mut self, cutoff_hz: f32, sample_rate: f32) {
        self.roll.set_derivative_lpf(cutoff_hz, sample_rate);
        self.pitch.set_derivative_lpf(cutoff_hz, sample_rate);
        self.yaw.set_derivative_lpf(cutoff_hz, sample_rate);
    }

    pub fn set_angle_mode(&mut self, enabled: bool, max_angle: f32) {
        self.roll.set_angle_mode(enabled, max_angle);
        self.pitch.set_angle_mode(enabled, max_angle);
        // Yaw typically stays in rate mode
    }

    pub fn reset(&mut self) {
        self.roll.reset();
        self.pitch.reset();
        self.yaw.reset();
    }

    /// Calculate PID outputs for rate mode
    pub fn calculate_rate(
        &mut self,
        setpoint: &Vector3,
        measurement: &Vector3,
        throttle: f32,
    ) -> Vector3 {
        Vector3 {
            x: self
                .roll
                .calculate(setpoint.x, measurement.x, throttle, None),
            y: self
                .pitch
                .calculate(setpoint.y, measurement.y, throttle, None),
            z: self
                .yaw
                .calculate(setpoint.z, measurement.z, throttle, None),
        }
    }

    /// Calculate PID outputs for angle mode (requires current angles)
    pub fn calculate_angle(
        &mut self,
        setpoint: &Vector3,
        measurement: &Vector3,
        current_angles: &Vector3,
        throttle: f32,
    ) -> Vector3 {
        Vector3 {
            x: self
                .roll
                .calculate(setpoint.x, measurement.x, throttle, Some(current_angles.x)),
            y: self
                .pitch
                .calculate(setpoint.y, measurement.y, throttle, Some(current_angles.y)),
            z: self
                .yaw
                .calculate(setpoint.z, measurement.z, throttle, None), // Yaw usually stays in rate mode
        }
    }
}
