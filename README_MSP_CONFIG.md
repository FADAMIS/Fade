# MSP Configuration System

A comprehensive MSP (MultiWii Serial Protocol) implementation for configuring all flight controller parameters via serial communication. This system allows external configurator applications to read and modify all flight controller settings.

## Overview

The MSP Configuration System provides complete access to all flight controller parameters through a standardized serial protocol. It supports reading, writing, and persisting configuration data for:

- **PID Controllers** - Flight stabilization parameters
- **Filter Configuration** - Gyro and PID filtering settings
- **Rate Profiles** - Stick response and rate limits
- **Motor Configuration** - ESC protocol and motor settings
- **OSD Configuration** - On-screen display settings
- **Flight Modes** - Angle, horizon, air mode, anti-gravity
- **Battery Configuration** - Cell count, capacity, scaling
- **Failsafe Configuration** - Emergency response settings
- **Calibration Data** - Gyro and accelerometer bias values

## Features

- **Complete Configuration Coverage** - All `FlightConfig` parameters accessible
- **Real-time Updates** - Live parameter modification during flight (where safe)
- **Persistent Storage** - Configuration automatically saved to flash memory
- **Data Integrity** - CRC checksums ensure data validity
- **Standard Protocol** - MSPv2 compatible for configurator integration
- **Lightweight Implementation** - Minimal memory footprint
- **Error Handling** - Robust error detection and recovery

## MSP Command Reference

### Core Commands (Standard MSP)
| Command | ID | Description |
|---------|----|----|
| `MSP_PID` | 112 | Get PID values |
| `MSP_SET_PID` | 202 | Set PID values |
| `MSP_SAVE_SETTINGS` | 2002 | Save configuration to flash |

### Extended Configuration Commands
| Command | ID | Description | Payload Size |
|---------|----|----|------------|
| `MSP_GET_FILTER_CONFIG` | 3000 | Get filter settings | 20 bytes |
| `MSP_SET_FILTER_CONFIG` | 3001 | Set filter settings | 17+ bytes |
| `MSP_GET_RATE_PROFILE` | 3002 | Get rate profile | 36 bytes |
| `MSP_SET_RATE_PROFILE` | 3003 | Set rate profile | 36 bytes |
| `MSP_GET_MOTOR_CONFIG` | 3004 | Get motor settings | 8 bytes |
| `MSP_SET_MOTOR_CONFIG` | 3005 | Set motor settings | 8 bytes |
| `MSP_GET_OSD_CONFIG` | 3006 | Get OSD settings | 12 bytes |
| `MSP_SET_OSD_CONFIG` | 3007 | Set OSD settings | 11+ bytes |
| `MSP_GET_FLIGHT_MODES` | 3008 | Get flight modes | 4 bytes |
| `MSP_SET_FLIGHT_MODES` | 3009 | Set flight modes | 4 bytes |
| `MSP_GET_BATTERY_CONFIG` | 3010 | Get battery settings | 11 bytes |
| `MSP_SET_BATTERY_CONFIG` | 3011 | Set battery settings | 11 bytes |
| `MSP_GET_FAILSAFE_CONFIG` | 3012 | Get failsafe settings | 4 bytes |
| `MSP_SET_FAILSAFE_CONFIG` | 3013 | Set failsafe settings | 4 bytes |
| `MSP_GET_CALIBRATION` | 3014 | Get calibration data | 24 bytes |
| `MSP_SET_CALIBRATION` | 3015 | Set calibration data | 24 bytes |
| `MSP_GET_ALL_CONFIG` | 3016 | Get entire configuration | Variable |
| `MSP_SET_ALL_CONFIG` | 3017 | Set entire configuration | Variable |

## Data Structures

### Filter Configuration (20 bytes)
```rust
struct FilterConfig {
    gyro_lowpass1_hz: f32,     // Primary gyro lowpass filter
    gyro_lowpass2_hz: f32,     // Secondary gyro lowpass filter  
    pid_lowpass_hz: f32,       // PID output lowpass filter
    notch_filter_hz: f32,      // Notch filter frequency
    notch_filter_enabled: bool, // Enable/disable notch filter
    _padding: [u8; 3],         // Alignment padding
}
```

### Rate Profile (36 bytes)
```rust
struct RateProfile {
    max_rate_roll: f32,        // Maximum roll rate (deg/s)
    max_rate_pitch: f32,       // Maximum pitch rate (deg/s)
    max_rate_yaw: f32,         // Maximum yaw rate (deg/s)
    expo_roll: f32,            // Roll exponential curve (0.0-1.0)
    expo_pitch: f32,           // Pitch exponential curve (0.0-1.0)
    expo_yaw: f32,             // Yaw exponential curve (0.0-1.0)
    super_rate_roll: f32,      // Roll super rate (0.0-1.0)
    super_rate_pitch: f32,     // Pitch super rate (0.0-1.0)
    super_rate_yaw: f32,       // Yaw super rate (0.0-1.0)
}
```

### Motor Configuration (8 bytes)
```rust
struct MotorConfig {
    motor_pwm_rate: u16,       // ESC update rate (Hz)
    motor_idle: f32,           // Idle throttle (0.0-1.0)
    motor_protocol: u8,        // 0=PWM, 1=DSHOT150, 2=DSHOT300, 3=DSHOT600
    motor_poles: u8,           // Motor pole count
}
```

### Flight Modes (4 bytes)
```rust
struct FlightModes {
    angle_mode_enabled: bool,     // Self-leveling mode
    horizon_mode_enabled: bool,   // Hybrid self-leveling mode
    air_mode_enabled: bool,       // Always mix motors for control
    anti_gravity_enabled: bool,   // Anti-gravity compensation
}
```

### Battery Configuration (11 bytes)
```rust
struct BatteryConfig {
    battery_cells: u8,         // Number of battery cells
    battery_capacity: u16,     // Battery capacity (mAh)
    voltage_scale: f32,        // ADC voltage scale factor
    current_scale: f32,        // ADC current scale factor
}
```

## Quick Start

### 1. Hardware Connection
Connect your flight controller to computer via USB. The system automatically detects STM32 devices on macOS (`/dev/cu.usbmodemXXXX`) and Linux (`/dev/ttyACMX`).

### 2. Test Configuration Access
```bash
python3 msp_config_test.py
```

This will:
- Auto-detect the flight controller
- Read all configuration sections  
- Display current values
- Test configuration updates
- Verify flash save functionality

### 3. Integration Example
```python
import serial
import struct

# Connect to flight controller
ser = serial.Serial('/dev/cu.usbmodem1234', 115200, timeout=1.0)

# Read current PID values
request = create_msp_request(MSP_PID)
ser.write(request)
cmd, payload = parse_msp_response(ser)
if cmd == MSP_PID:
    pids = struct.unpack('<fffffffff', payload)
    roll_p, roll_i, roll_d = pids[0:3]
    print(f"Roll PID: P={roll_p}, I={roll_i}, D={roll_d}")

# Update filter settings
new_filters = struct.pack('<ffff', 200.0, 100.0, 150.0, 0.0) + b'\x00'
request = create_msp_request(MSP_SET_FILTER_CONFIG, new_filters)
ser.write(request)

# Save to flash
request = create_msp_request(MSP_SAVE_SETTINGS)
ser.write(request)
```

## Protocol Details

### MSPv2 Frame Format
```
| $ | X | < | Flag | Command(2) | Size(2) | Payload(n) | Checksum |
```

- **Preamble**: `$X<` (request) or `$X>` (response)
- **Flag**: Always 0x00
- **Command**: 16-bit command ID (little-endian)
- **Size**: 16-bit payload length (little-endian)
- **Payload**: Variable length data
- **Checksum**: XOR of Flag + Command + Size + Payload bytes

### Error Handling
- Invalid checksums are silently ignored
- Malformed packets are discarded
- Configuration validation ensures data integrity
- Flash save operations include success/failure feedback

### Data Encoding
- **Floats**: IEEE 754 single precision, little-endian
- **Integers**: Little-endian byte order
- **Booleans**: 0x00 = false, non-zero = true
- **Arrays**: Packed sequential encoding

## Integration with Configurator Applications

### Betaflight Configurator Integration
The MSP commands are designed to be compatible with standard configurator applications. Custom commands (3000+) provide extended functionality while maintaining backward compatibility.

### Custom Configurator Development
```python
class FlightControllerConfig:
    def __init__(self, serial_port):
        self.ser = serial.Serial(serial_port, 115200, timeout=1.0)
    
    def get_pid_gains(self):
        """Get current PID gains"""
        # Implementation here
        pass
    
    def set_filter_config(self, gyro_lp1, gyro_lp2, pid_lp, notch_hz, notch_en):
        """Update filter configuration"""  
        # Implementation here
        pass
    
    def save_config(self):
        """Save configuration to flash"""
        # Implementation here
        pass
```

## Safety Considerations

⚠️ **Important Safety Guidelines**:

1. **Ground Testing Only** - Never modify critical parameters during flight
2. **Parameter Validation** - Always validate parameter ranges before applying
3. **Backup Configuration** - Save known-good configurations before experimenting
4. **Progressive Changes** - Make small incremental adjustments
5. **Test Environment** - Use props-off bench testing for initial parameter changes

### Safe Parameter Ranges
| Parameter | Safe Range | Notes |
|-----------|------------|-------|
| PID Gains | P: 0-200, I: 0-500, D: 0-100 | Start with defaults |
| Filter Frequencies | 50-500 Hz | Lower = smoother, higher = more responsive |
| Max Rates | 200-2000 deg/s | Higher values require experience |
| Motor Idle | 0.01-0.15 | Too low may cause motor stop |

## Troubleshooting

### Connection Issues
```bash
# Check available ports
python3 -c "import serial.tools.list_ports; print([p.device for p in serial.tools.list_ports.comports()])"

# Test basic connectivity
python3 msp_config_test.py
```

### Configuration Not Saving
- Ensure flash write operations complete successfully
- Check for sufficient flash memory space
- Verify configuration checksum validation
- Use `MSP_SAVE_SETTINGS` after parameter changes

### Invalid Response Data
- Verify correct command IDs
- Check payload size expectations
- Ensure proper byte ordering (little-endian)
- Validate checksum calculations

### Communication Errors
- Increase timeout values for slow operations
- Add delays between consecutive commands
- Implement retry logic for critical operations
- Check USB cable and connection stability

## Development

### Building the Firmware
```bash
cd fade/
cargo build --release
```

### Running Tests
```bash
# Rust unit tests
cargo test

# Python integration tests  
python3 msp_config_test.py
```

### Adding New Parameters
1. Add fields to appropriate struct in `config.rs`
2. Update struct size calculations
3. Add MSP command constants in `msp.rs`
4. Implement get/set handlers in `handle_msp()`
5. Update test script and documentation

## License

This project is dual-licensed under MIT and Apache 2.0 licenses. See `LICENSE-MIT` and `LICENSE-APACHE` files for details.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Ensure all tests pass
5. Update documentation
6. Submit a pull request

## Support

For issues and questions:
- Open GitHub issues for bugs and feature requests
- Check existing issues for known problems
- Provide hardware details and error logs
- Include minimal reproduction cases

---

**Version**: 1.0.0  
**Last Updated**: 2024  
**Compatibility**: STM32-based flight controllers, MSPv2 protocol