// src/config.rs

use crate::pid::{Axis, PidControllers};

/// Magic bytes to identify valid configuration in flash
pub const CONFIG_MAGIC: [u8; 4] = [0xDE, 0xAD, 0xC0, 0xDE];

/// Current configuration version for future compatibility
pub const CONFIG_VERSION: u16 = 1;

/// Flash address where configuration is stored
pub const CONFIG_FLASH_ADDRESS: u32 = 0x200;

/// PID gains for a single axis
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct PidGains {
    pub p: f32,
    pub i: f32,
    pub d: f32,
}

impl PidGains {
    pub fn new(p: f32, i: f32, d: f32) -> Self {
        Self { p, i, d }
    }

    pub fn default_roll() -> Self {
        Self::new(45.0, 85.0, 35.0)
    }

    pub fn default_pitch() -> Self {
        Self::new(58.0, 90.0, 42.0)
    }

    pub fn default_yaw() -> Self {
        Self::new(80.0, 90.0, 0.0)
    }
}

/// Filter configuration
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct FilterConfig {
    pub gyro_lowpass1_hz: f32,
    pub gyro_lowpass2_hz: f32,
    pub pid_lowpass_hz: f32,
    pub notch_filter_hz: f32,
    pub notch_filter_enabled: bool,
    pub _padding: [u8; 3], // Align to 4-byte boundary
}

impl Default for FilterConfig {
    fn default() -> Self {
        Self {
            gyro_lowpass1_hz: 200.0,
            gyro_lowpass2_hz: 100.0,
            pid_lowpass_hz: 150.0,
            notch_filter_hz: 0.0,
            notch_filter_enabled: false,
            _padding: [0; 3],
        }
    }
}

/// Rate profile configuration
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct RateProfile {
    pub max_rate_roll: f32,    // deg/s
    pub max_rate_pitch: f32,   // deg/s
    pub max_rate_yaw: f32,     // deg/s
    pub expo_roll: f32,        // 0.0 - 1.0
    pub expo_pitch: f32,       // 0.0 - 1.0
    pub expo_yaw: f32,         // 0.0 - 1.0
    pub super_rate_roll: f32,  // 0.0 - 1.0
    pub super_rate_pitch: f32, // 0.0 - 1.0
    pub super_rate_yaw: f32,   // 0.0 - 1.0
}

impl Default for RateProfile {
    fn default() -> Self {
        Self {
            max_rate_roll: 670.0,
            max_rate_pitch: 670.0,
            max_rate_yaw: 400.0,
            expo_roll: 0.0,
            expo_pitch: 0.0,
            expo_yaw: 0.0,
            super_rate_roll: 0.7,
            super_rate_pitch: 0.7,
            super_rate_yaw: 0.7,
        }
    }
}

/// Motor configuration
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct MotorConfig {
    pub motor_pwm_rate: u16, // Hz
    pub motor_idle: f32,     // 0.0 - 1.0
    pub motor_protocol: u8,  // 0=PWM, 1=DSHOT150, 2=DSHOT300, 3=DSHOT600
    pub motor_poles: u8,     // Number of poles
    pub _padding: [u8; 4],   // Align to 8-byte boundary
}

impl Default for MotorConfig {
    fn default() -> Self {
        Self {
            motor_pwm_rate: 480,
            motor_idle: 0.05,
            motor_protocol: 3, // DSHOT600
            motor_poles: 14,
            _padding: [0; 4],
        }
    }
}

/// OSD configuration
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct OsdConfig {
    pub enabled: bool,
    pub voltage_alarm: f32,  // Volts
    pub rssi_alarm: u8,      // %
    pub capacity_alarm: u16, // mAh
    pub timer_alarm: u16,    // minutes
    pub units: u8,           // 0=Imperial, 1=Metric
    pub _padding: [u8; 2],   // Align to 4-byte boundary
}

impl Default for OsdConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            voltage_alarm: 3.5,
            rssi_alarm: 20,
            capacity_alarm: 1000,
            timer_alarm: 5,
            units: 1, // Metric
            _padding: [0; 2],
        }
    }
}

/// Main flight configuration structure
/// This contains all persistent settings that can be saved to flash
#[derive(Debug, Clone, PartialEq)]
#[repr(C)]
pub struct FlightConfig {
    // Header
    pub magic: [u8; 4],
    pub version: u16,
    pub size: u16,
    pub checksum: u32,

    // PID Configuration
    pub pid_roll: PidGains,
    pub pid_pitch: PidGains,
    pub pid_yaw: PidGains,

    // Filter Configuration
    pub filters: FilterConfig,

    // Rate Configuration
    pub rates: RateProfile,

    // Motor Configuration
    pub motors: MotorConfig,

    // OSD Configuration
    pub osd: OsdConfig,

    // Flight modes
    pub angle_mode_enabled: bool,
    pub horizon_mode_enabled: bool,
    pub air_mode_enabled: bool,
    pub anti_gravity_enabled: bool,

    // Battery configuration
    pub battery_cells: u8,     // Number of cells
    pub battery_capacity: u16, // mAh
    pub voltage_scale: f32,    // ADC scale factor
    pub current_scale: f32,    // ADC scale factor

    // Failsafe configuration
    pub failsafe_throttle: u16, // Throttle value for failsafe
    pub failsafe_delay: u16,    // ms before activating failsafe

    // Calibration data
    pub gyro_bias_x: f32,
    pub gyro_bias_y: f32,
    pub gyro_bias_z: f32,
    pub accel_bias_x: f32,
    pub accel_bias_y: f32,
    pub accel_bias_z: f32,

    // Reserved space for future use
    pub reserved: [u8; 64],
}

impl FlightConfig {
    /// Create a new configuration with default values
    pub fn new() -> Self {
        let mut config = Self {
            magic: CONFIG_MAGIC,
            version: CONFIG_VERSION,
            size: core::mem::size_of::<FlightConfig>() as u16,
            checksum: 0,

            // PID defaults
            pid_roll: PidGains::default_roll(),
            pid_pitch: PidGains::default_pitch(),
            pid_yaw: PidGains::default_yaw(),

            // Component defaults
            filters: FilterConfig::default(),
            rates: RateProfile::default(),
            motors: MotorConfig::default(),
            osd: OsdConfig::default(),

            // Flight mode defaults
            angle_mode_enabled: false,
            horizon_mode_enabled: false,
            air_mode_enabled: true,
            anti_gravity_enabled: true,

            // Battery defaults
            battery_cells: 4,
            battery_capacity: 1300,
            voltage_scale: 110.0,
            current_scale: 400.0,

            // Failsafe defaults
            failsafe_throttle: 1000,
            failsafe_delay: 1000,

            // Calibration defaults (will be set during calibration)
            gyro_bias_x: 0.0,
            gyro_bias_y: 0.0,
            gyro_bias_z: 0.0,
            accel_bias_x: 0.0,
            accel_bias_y: 0.0,
            accel_bias_z: 0.0,

            reserved: [0; 64],
        };

        // Calculate and set checksum
        config.checksum = config.calculate_checksum();
        config
    }

    /// Calculate checksum for the configuration (excluding the checksum field itself)
    pub fn calculate_checksum(&self) -> u32 {
        let bytes = unsafe {
            core::slice::from_raw_parts(
                (self as *const Self) as *const u8,
                core::mem::size_of::<Self>(),
            )
        };

        let mut checksum: u32 = 0;
        for (i, &byte) in bytes.iter().enumerate() {
            // Skip the checksum field itself (bytes 8-11)
            if i < 8 || i >= 12 {
                checksum = checksum.wrapping_add(byte as u32);
            }
        }
        checksum
    }

    /// Validate the configuration checksum
    pub fn is_valid(&self) -> bool {
        if self.magic != CONFIG_MAGIC {
            return false;
        }
        if self.version != CONFIG_VERSION {
            return false;
        }
        if self.size != core::mem::size_of::<FlightConfig>() as u16 {
            return false;
        }
        self.checksum == self.calculate_checksum()
    }

    /// Apply PID configuration to PID controllers
    pub fn apply_to_pid_controllers(&self, pid_controllers: &mut PidControllers) {
        pid_controllers.set_gains(
            Axis::Roll,
            self.pid_roll.p,
            self.pid_roll.i,
            self.pid_roll.d,
        );
        pid_controllers.set_gains(
            Axis::Pitch,
            self.pid_pitch.p,
            self.pid_pitch.i,
            self.pid_pitch.d,
        );
        pid_controllers.set_gains(Axis::Yaw, self.pid_yaw.p, self.pid_yaw.i, self.pid_yaw.d);
    }

    /// Update PID configuration from PID controllers
    pub fn update_from_pid_controllers(&mut self, pid_controllers: &PidControllers) {
        let gains = pid_controllers.get_gains();
        self.pid_roll = PidGains::new(gains[0].0, gains[0].1, gains[0].2);
        self.pid_pitch = PidGains::new(gains[1].0, gains[1].1, gains[1].2);
        self.pid_yaw = PidGains::new(gains[2].0, gains[2].1, gains[2].2);

        // Update checksum
        self.checksum = self.calculate_checksum();
    }

    /// Convert to bytes for flash storage
    pub fn to_bytes(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(
                (self as *const Self) as *const u8,
                core::mem::size_of::<Self>(),
            )
        }
    }

    /// Create from bytes (for loading from flash)
    pub fn from_bytes(bytes: &[u8]) -> Option<Self> {
        if bytes.len() != core::mem::size_of::<Self>() {
            return None;
        }

        let config = unsafe { core::ptr::read(bytes.as_ptr() as *const Self) };

        if config.is_valid() {
            Some(config)
        } else {
            None
        }
    }

    /// Get configuration size in bytes
    pub fn size() -> usize {
        core::mem::size_of::<Self>()
    }
}

impl Default for FlightConfig {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {

    #[test]
    fn test_config_size() {
        // Ensure the config structure is not too large
        assert!(
            FlightConfig::size() <= 512,
            "Config too large for flash storage"
        );
    }

    #[test]
    fn test_config_checksum() {
        let config = FlightConfig::new();
        assert!(config.is_valid());

        // Test that modifying data invalidates checksum
        let mut config2 = config.clone();
        config2.pid_roll.p = 999.0;
        assert!(!config2.is_valid());

        // Test that updating checksum makes it valid again
        config2.checksum = config2.calculate_checksum();
        assert!(config2.is_valid());
    }

    #[test]
    fn test_config_serialization() {
        let config = FlightConfig::new();
        let bytes = config.to_bytes();
        let config2 = FlightConfig::from_bytes(bytes).unwrap();
        assert_eq!(config, config2);
    }
}
