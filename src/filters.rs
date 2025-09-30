use core::f32::consts::PI;
use micromath::F32Ext;

/// Biquad filter implementation (similar to Betaflight's biquadFilter_t)
#[derive(Debug, Clone)]
pub struct BiquadFilter {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    x1: f32,
    x2: f32,
    y1: f32,
    y2: f32,
}

impl BiquadFilter {
    pub fn new() -> Self {
        Self {
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
            a1: 0.0,
            a2: 0.0,
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
        }
    }

    /// Configure as lowpass filter (Betaflight style)
    pub fn setup_lowpass(&mut self, sample_rate: f32, cutoff_freq: f32) {
        let omega = 2.0 * PI * cutoff_freq / sample_rate;
        let sin_omega = omega.sin();
        let cos_omega = omega.cos();
        let alpha = sin_omega / (2.0 * 0.7071); // Q = 0.7071 (sqrt(2)/2)

        let a0 = 1.0 + alpha;
        self.b0 = (1.0 - cos_omega) / (2.0 * a0);
        self.b1 = (1.0 - cos_omega) / a0;
        self.b2 = self.b0;
        self.a1 = -2.0 * cos_omega / a0;
        self.a2 = (1.0 - alpha) / a0;

        self.reset();
    }

    /// Configure as notch filter
    pub fn setup_notch(&mut self, sample_rate: f32, center_freq: f32, q_factor: f32) {
        let omega = 2.0 * PI * center_freq / sample_rate;
        let sin_omega = omega.sin();
        let cos_omega = omega.cos();
        let alpha = sin_omega / (2.0 * q_factor);

        let a0 = 1.0 + alpha;
        self.b0 = 1.0 / a0;
        self.b1 = -2.0 * cos_omega / a0;
        self.b2 = 1.0 / a0;
        self.a1 = self.b1; // Same as b1 for notch
        self.a2 = (1.0 - alpha) / a0;

        self.reset();
    }

    /// Configure as pt1 filter (first-order lowpass, similar to Betaflight's pt1Filter)
    pub fn setup_pt1(&mut self, sample_rate: f32, cutoff_freq: f32) {
        let rc = 1.0 / (2.0 * PI * cutoff_freq);
        let dt = 1.0 / sample_rate;
        let alpha = dt / (rc + dt);

        self.b0 = alpha;
        self.b1 = 0.0;
        self.b2 = 0.0;
        self.a1 = -(1.0 - alpha);
        self.a2 = 0.0;

        self.reset();
    }

    pub fn reset(&mut self) {
        self.x1 = 0.0;
        self.x2 = 0.0;
        self.y1 = 0.0;
        self.y2 = 0.0;
    }

    pub fn apply(&mut self, input: f32) -> f32 {
        let output = self.b0 * input + self.b1 * self.x1 + self.b2 * self.x2
            - self.a1 * self.y1
            - self.a2 * self.y2;

        self.x2 = self.x1;
        self.x1 = input;
        self.y2 = self.y1;
        self.y1 = output;

        output
    }
}

/// 3-axis biquad filter for gyro/accel data
#[derive(Debug, Clone)]
pub struct BiquadFilter3 {
    pub x: BiquadFilter,
    pub y: BiquadFilter,
    pub z: BiquadFilter,
}

impl BiquadFilter3 {
    pub fn new() -> Self {
        Self {
            x: BiquadFilter::new(),
            y: BiquadFilter::new(),
            z: BiquadFilter::new(),
        }
    }

    pub fn setup_lowpass(&mut self, sample_rate: f32, cutoff_freq: f32) {
        self.x.setup_lowpass(sample_rate, cutoff_freq);
        self.y.setup_lowpass(sample_rate, cutoff_freq);
        self.z.setup_lowpass(sample_rate, cutoff_freq);
    }

    pub fn setup_notch(&mut self, sample_rate: f32, center_freq: f32, q_factor: f32) {
        self.x.setup_notch(sample_rate, center_freq, q_factor);
        self.y.setup_notch(sample_rate, center_freq, q_factor);
        self.z.setup_notch(sample_rate, center_freq, q_factor);
    }

    pub fn setup_pt1(&mut self, sample_rate: f32, cutoff_freq: f32) {
        self.x.setup_pt1(sample_rate, cutoff_freq);
        self.y.setup_pt1(sample_rate, cutoff_freq);
        self.z.setup_pt1(sample_rate, cutoff_freq);
    }

    pub fn reset(&mut self) {
        self.x.reset();
        self.y.reset();
        self.z.reset();
    }

    pub fn apply(&mut self, input: [f32; 3]) -> [f32; 3] {
        [
            self.x.apply(input[0]),
            self.y.apply(input[1]),
            self.z.apply(input[2]),
        ]
    }
}

/// Betaflight-style gyro filter chain
#[derive(Debug, Clone)]
pub struct GyroFilter {
    // Lowpass filters (typically 2 stages)
    lowpass1: BiquadFilter3,
    lowpass2: BiquadFilter3,

    // Notch filters (up to 2 dynamic notches in modern Betaflight)
    notch1: BiquadFilter3,
    notch2: BiquadFilter3,

    // Static notch filter
    static_notch: BiquadFilter3,

    // Configuration
    lowpass1_enabled: bool,
    lowpass2_enabled: bool,
    notch1_enabled: bool,
    notch2_enabled: bool,
    static_notch_enabled: bool,

    // Robust calibration system
    bias: [f32; 3],
    deadband: f32,
    calibrated: bool,
    calibrating: bool,
    calibration_count: u32,
    calibration_sum: [f32; 3],
    calibration_samples: u32,
    calibration_threshold: f32,

    // Stability detection for robust calibration
    stability_buffer: [[f32; 3]; 10], // Rolling buffer of recent samples
    stability_index: usize,
    stability_buffer_full: bool,
    min_stability_time: u32, // Minimum samples of stability before starting calibration
    stability_counter: u32,
    max_sample_deviation: f32, // Max deviation between samples to consider "stable"

    // Startup delay to ignore initial power-on transients
    startup_delay: u32,
    startup_counter: u32,
}

impl GyroFilter {
    pub fn new(sample_rate: f32) -> Self {
        let mut filter = Self {
            lowpass1: BiquadFilter3::new(),
            lowpass2: BiquadFilter3::new(),
            notch1: BiquadFilter3::new(),
            notch2: BiquadFilter3::new(),
            static_notch: BiquadFilter3::new(),
            lowpass1_enabled: true,
            lowpass2_enabled: true,
            notch1_enabled: false,
            notch2_enabled: false,
            static_notch_enabled: false,
            bias: [0.0; 3],
            deadband: 0.5, // 0.5 deg/s deadband
            calibrated: false,
            calibrating: false,
            calibration_count: 0,
            calibration_sum: [0.0; 3],
            calibration_samples: 200,   // More samples for better accuracy
            calibration_threshold: 1.5, // 1.5 deg/s max motion threshold

            // Stability detection
            stability_buffer: [[0.0; 3]; 10],
            stability_index: 0,
            stability_buffer_full: false,
            min_stability_time: 100, // Must be stable for 100 samples (100ms at 1kHz)
            stability_counter: 0,
            max_sample_deviation: 1.0, // Max 1.0 deg/s deviation between samples (more lenient)

            // Startup delay - ignore first 500ms of data
            startup_delay: (sample_rate * 0.5) as u32, // 500ms delay
            startup_counter: 0,
        };

        // Default Betaflight-like settings
        filter.setup_lowpass1(sample_rate, 250.0); // 250Hz first lowpass
        filter.setup_lowpass2(sample_rate, 125.0); // 125Hz second lowpass

        filter
    }

    pub fn setup_lowpass1(&mut self, sample_rate: f32, cutoff_freq: f32) {
        self.lowpass1.setup_lowpass(sample_rate, cutoff_freq);
        self.lowpass1_enabled = cutoff_freq > 0.0;
    }

    pub fn setup_lowpass2(&mut self, sample_rate: f32, cutoff_freq: f32) {
        self.lowpass2.setup_lowpass(sample_rate, cutoff_freq);
        self.lowpass2_enabled = cutoff_freq > 0.0;
    }

    pub fn setup_notch1(&mut self, sample_rate: f32, center_freq: f32, q_factor: f32) {
        self.notch1.setup_notch(sample_rate, center_freq, q_factor);
        self.notch1_enabled = center_freq > 0.0;
    }

    pub fn setup_notch2(&mut self, sample_rate: f32, center_freq: f32, q_factor: f32) {
        self.notch2.setup_notch(sample_rate, center_freq, q_factor);
        self.notch2_enabled = center_freq > 0.0;
    }

    pub fn setup_static_notch(&mut self, sample_rate: f32, center_freq: f32, q_factor: f32) {
        self.static_notch
            .setup_notch(sample_rate, center_freq, q_factor);
        self.static_notch_enabled = center_freq > 0.0;
    }

    pub fn set_calibration_params(
        &mut self,
        samples: u32,
        threshold: f32,
        deadband: f32,
        stability_time_ms: f32,
        startup_delay_ms: f32,
        sample_rate: f32,
    ) {
        self.calibration_samples = samples;
        self.calibration_threshold = threshold;
        self.deadband = deadband;
        self.min_stability_time = (stability_time_ms * sample_rate / 1000.0) as u32;
        self.startup_delay = (startup_delay_ms * sample_rate / 1000.0) as u32;
    }

    pub fn reset_calibration(&mut self) {
        self.calibrated = false;
        self.calibrating = false;
        self.calibration_count = 0;
        self.calibration_sum = [0.0; 3];
        self.bias = [0.0; 3];
        self.stability_counter = 0;
        self.stability_buffer_full = false;
        self.stability_index = 0;
        self.startup_counter = 0;
        for i in 0..10 {
            self.stability_buffer[i] = [0.0; 3];
        }
    }

    /// Force calibration to start immediately (use when you know quad is still)
    pub fn force_calibration_start(&mut self) {
        if !self.calibrated {
            self.calibrating = true;
            self.calibration_count = 0;
            self.calibration_sum = [0.0; 3];
            self.startup_counter = self.startup_delay; // Skip startup delay
        }
    }

    /// Set manual bias values and mark as calibrated (for testing or known values)
    pub fn set_manual_bias(&mut self, bias: [f32; 3]) {
        self.bias = bias;
        self.calibrated = true;
        self.calibrating = false;

        // Reset all filters
        if self.lowpass1_enabled {
            self.lowpass1.reset();
        }
        if self.lowpass2_enabled {
            self.lowpass2.reset();
        }
        if self.notch1_enabled {
            self.notch1.reset();
        }
        if self.notch2_enabled {
            self.notch2.reset();
        }
        if self.static_notch_enabled {
            self.static_notch.reset();
        }
    }

    /// Skip calibration entirely and use zero bias (for testing)
    pub fn skip_calibration(&mut self) {
        self.set_manual_bias([0.0; 3]);
    }

    /// Check if gyro readings are stable enough for calibration
    fn is_stable(&self, current_sample: [f32; 3]) -> bool {
        if !self.stability_buffer_full && self.stability_index < 5 {
            return false; // Need at least 5 samples
        }

        // Check if current sample and recent samples are within acceptable deviation
        let samples_to_check = if self.stability_buffer_full {
            10
        } else {
            self.stability_index
        };

        for i in 0..samples_to_check {
            let sample = self.stability_buffer[i];
            for axis in 0..3 {
                if (current_sample[axis] - sample[axis]).abs() > self.max_sample_deviation {
                    return false;
                }
            }
        }

        // Also check that the overall magnitude is reasonable
        let magnitude = current_sample
            .iter()
            .map(|v| v.abs())
            .fold(0.0_f32, |a, b| a.max(b));
        magnitude < self.calibration_threshold
    }

    pub fn update(&mut self, raw: [f32; 3]) -> [f32; 3] {
        // Handle startup delay - but still process and return filtered data
        if self.startup_counter < self.startup_delay {
            self.startup_counter += 1;
            // During startup, use zero bias and apply filters
            let mut corrected = raw;

            // Apply basic filtering even during startup
            if self.lowpass1_enabled {
                corrected = self.lowpass1.apply(corrected);
            }
            if self.lowpass2_enabled {
                corrected = self.lowpass2.apply(corrected);
            }

            return corrected;
        }

        // Add current sample to stability buffer
        self.stability_buffer[self.stability_index] = raw;
        self.stability_index = (self.stability_index + 1) % 10;
        if self.stability_index == 0 {
            self.stability_buffer_full = true;
        }

        // Check for calibration conditions
        if !self.calibrated && !self.calibrating {
            let magnitude = raw.iter().map(|v| v.abs()).fold(0.0_f32, |a, b| a.max(b));

            // Basic magnitude check
            if magnitude < self.calibration_threshold {
                if self.is_stable(raw) {
                    self.stability_counter += 1;

                    // Start calibration only after sustained stability
                    if self.stability_counter >= self.min_stability_time {
                        self.calibrating = true;
                        self.calibration_count = 0;
                        self.calibration_sum = [0.0; 3];
                    }
                } else {
                    self.stability_counter = 0; // Reset if not stable
                }
            } else {
                self.stability_counter = 0; // Reset if too much motion
            }
        }

        // Perform calibration
        if self.calibrating {
            // More lenient stability check during calibration
            let magnitude = raw.iter().map(|v| v.abs()).fold(0.0_f32, |a, b| a.max(b));
            if magnitude > self.calibration_threshold * 1.5 {
                // Motion detected during calibration - abort and restart
                self.calibrating = false;
                self.calibration_count = 0;
                self.calibration_sum = [0.0; 3];
                self.stability_counter = 0;
                // Don't return zeros, continue with current processing
            } else {
                // Continue calibration
                for i in 0..3 {
                    self.calibration_sum[i] += raw[i];
                }
                self.calibration_count += 1;

                if self.calibration_count >= self.calibration_samples {
                    for i in 0..3 {
                        self.bias[i] = self.calibration_sum[i] / self.calibration_count as f32;
                    }
                    self.calibrated = true;
                    self.calibrating = false;

                    // Reset all filters
                    if self.lowpass1_enabled {
                        self.lowpass1.reset();
                    }
                    if self.lowpass2_enabled {
                        self.lowpass2.reset();
                    }
                    if self.notch1_enabled {
                        self.notch1.reset();
                    }
                    if self.notch2_enabled {
                        self.notch2.reset();
                    }
                    if self.static_notch_enabled {
                        self.static_notch.reset();
                    }
                }
            }
        }

        // Apply bias correction (use current bias, even if still calibrating)
        let mut corrected = [
            raw[0] - self.bias[0],
            raw[1] - self.bias[1],
            raw[2] - self.bias[2],
        ];

        // Apply Betaflight filter chain
        // Stage 1: Static notch (if enabled)
        if self.static_notch_enabled {
            corrected = self.static_notch.apply(corrected);
        }

        // Stage 2: Dynamic notch 1 (if enabled)
        if self.notch1_enabled {
            corrected = self.notch1.apply(corrected);
        }

        // Stage 3: Dynamic notch 2 (if enabled)
        if self.notch2_enabled {
            corrected = self.notch2.apply(corrected);
        }

        // Stage 4: First lowpass
        if self.lowpass1_enabled {
            corrected = self.lowpass1.apply(corrected);
        }

        // Stage 5: Second lowpass
        if self.lowpass2_enabled {
            corrected = self.lowpass2.apply(corrected);
        }

        // Apply deadband only if fully calibrated
        if self.calibrated {
            for i in 0..3 {
                if corrected[i].abs() < self.deadband {
                    corrected[i] = 0.0;
                }
            }
        }

        corrected
    }

    pub fn is_calibrated(&self) -> bool {
        self.calibrated
    }

    pub fn is_calibrating(&self) -> bool {
        self.calibrating
    }

    pub fn get_bias(&self) -> [f32; 3] {
        self.bias
    }

    pub fn get_calibration_progress(&self) -> f32 {
        if self.calibrating {
            self.calibration_count as f32 / self.calibration_samples as f32
        } else if self.stability_counter > 0 && !self.calibrated {
            self.stability_counter as f32 / self.min_stability_time as f32
        } else {
            0.0
        }
    }

    pub fn is_waiting_for_stability(&self) -> bool {
        !self.calibrated && !self.calibrating && self.startup_counter >= self.startup_delay
    }
}

/// Simple PT1 filter for accelerometer (commonly used in Betaflight)
#[derive(Debug, Clone)]
pub struct AccelFilter3 {
    filters: [BiquadFilter; 3],
}

impl AccelFilter3 {
    pub fn new(sample_rate: f32, cutoff_freq: f32) -> Self {
        let mut filters = [
            BiquadFilter::new(),
            BiquadFilter::new(),
            BiquadFilter::new(),
        ];

        for filter in &mut filters {
            filter.setup_pt1(sample_rate, cutoff_freq);
        }

        Self { filters }
    }

    pub fn update(&mut self, input: [f32; 3]) -> [f32; 3] {
        [
            self.filters[0].apply(input[0]),
            self.filters[1].apply(input[1]),
            self.filters[2].apply(input[2]),
        ]
    }

    pub fn reset(&mut self) {
        for filter in &mut self.filters {
            filter.reset();
        }
    }
}
