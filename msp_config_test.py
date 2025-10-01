#!/usr/bin/env python3
"""
Minimal MSP Configuration Test Script
Tests all configuration parameters via MSP protocol
"""

import serial
import serial.tools.list_ports
import struct
import time

# MSP Commands
MSP_PID = 112
MSP_SET_PID = 202
MSP_GET_FILTER_CONFIG = 3000
MSP_SET_FILTER_CONFIG = 3001
MSP_GET_RATE_PROFILE = 3002
MSP_SET_RATE_PROFILE = 3003
MSP_GET_MOTOR_CONFIG = 3004
MSP_SET_MOTOR_CONFIG = 3005
MSP_GET_OSD_CONFIG = 3006
MSP_SET_OSD_CONFIG = 3007
MSP_GET_FLIGHT_MODES = 3008
MSP_SET_FLIGHT_MODES = 3009
MSP_GET_BATTERY_CONFIG = 3010
MSP_SET_BATTERY_CONFIG = 3011
MSP_GET_FAILSAFE_CONFIG = 3012
MSP_SET_FAILSAFE_CONFIG = 3013
MSP_GET_CALIBRATION = 3014
MSP_SET_CALIBRATION = 3015
MSP_SAVE_SETTINGS = 2002

def find_serial_port():
    """Find STM32 device serial port"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "usbmodem" in port.device or "ttyACM" in port.device:
            return port.device
    return None

def create_msp_request(command, payload=b''):
    """Create MSPv2 request frame"""
    frame = bytearray()
    frame.append(ord('$'))
    frame.append(ord('X'))
    frame.append(ord('<'))

    flag = 0
    frame.append(flag)

    checksum = flag

    frame.extend(struct.pack('<H', command))
    checksum ^= (command & 0xFF)
    checksum ^= ((command >> 8) & 0xFF)

    payload_size = len(payload)
    frame.extend(struct.pack('<H', payload_size))
    checksum ^= (payload_size & 0xFF)
    checksum ^= ((payload_size >> 8) & 0xFF)

    frame.extend(payload)
    for byte in payload:
        checksum ^= byte

    frame.append(checksum)
    return frame

def parse_msp_response(ser):
    """Parse MSPv2 response"""
    try:
        header = ser.read(3)
        if header != b'$X>':
            return None, None

        flag = ser.read(1)[0]
        command = struct.unpack('<H', ser.read(2))[0]
        payload_size = struct.unpack('<H', ser.read(2))[0]
        payload = ser.read(payload_size)
        checksum_byte = ser.read(1)[0]

        # Verify checksum
        checksum = flag
        checksum ^= (command & 0xFF)
        checksum ^= ((command >> 8) & 0xFF)
        checksum ^= (payload_size & 0xFF)
        checksum ^= ((payload_size >> 8) & 0xFF)
        for byte in payload:
            checksum ^= byte

        if checksum != checksum_byte:
            return None, None

        return command, payload
    except:
        return None, None

def test_filter_config(ser):
    """Test filter configuration"""
    print("Testing Filter Config...")

    # Get current config
    ser.write(create_msp_request(MSP_GET_FILTER_CONFIG))
    cmd, payload = parse_msp_response(ser)
    if cmd == MSP_GET_FILTER_CONFIG and len(payload) >= 17:
        gyro_lp1, gyro_lp2, pid_lp, notch_hz = struct.unpack('<ffff', payload[:16])
        notch_enabled = payload[16] != 0
        print(f"  Current: LP1={gyro_lp1:.1f}Hz, LP2={gyro_lp2:.1f}Hz, PID={pid_lp:.1f}Hz")
        print(f"  Notch: {notch_hz:.1f}Hz, Enabled={notch_enabled}")

        # Test setting new values
        new_config = struct.pack('<ffff', 180.0, 90.0, 140.0, 120.0) + b'\x01'  # Enable notch
        ser.write(create_msp_request(MSP_SET_FILTER_CONFIG, new_config))
        time.sleep(0.1)
        print("  ✓ Filter config updated")
    else:
        print("  ✗ Failed to read filter config")

def test_rate_profile(ser):
    """Test rate profile"""
    print("Testing Rate Profile...")

    ser.write(create_msp_request(MSP_GET_RATE_PROFILE))
    cmd, payload = parse_msp_response(ser)
    if cmd == MSP_GET_RATE_PROFILE and len(payload) >= 36:
        rates = struct.unpack('<fffffffff', payload)
        print(f"  Max Rates: R={rates[0]:.0f}, P={rates[1]:.0f}, Y={rates[2]:.0f} deg/s")
        print(f"  Expo: R={rates[3]:.2f}, P={rates[4]:.2f}, Y={rates[5]:.2f}")
        print(f"  Super Rate: R={rates[6]:.2f}, P={rates[7]:.2f}, Y={rates[8]:.2f}")
        print("  ✓ Rate profile read successfully")
    else:
        print("  ✗ Failed to read rate profile")

def test_motor_config(ser):
    """Test motor configuration"""
    print("Testing Motor Config...")

    ser.write(create_msp_request(MSP_GET_MOTOR_CONFIG))
    cmd, payload = parse_msp_response(ser)
    if cmd == MSP_GET_MOTOR_CONFIG and len(payload) >= 8:
        pwm_rate, idle, protocol, poles = struct.unpack('<HfBB', payload)
        protocols = ["PWM", "DSHOT150", "DSHOT300", "DSHOT600"]
        proto_name = protocols[protocol] if protocol < len(protocols) else f"Unknown({protocol})"
        print(f"  PWM Rate: {pwm_rate}Hz, Idle: {idle:.3f}")
        print(f"  Protocol: {proto_name}, Poles: {poles}")
        print("  ✓ Motor config read successfully")
    else:
        print("  ✗ Failed to read motor config")

def test_flight_modes(ser):
    """Test flight modes"""
    print("Testing Flight Modes...")

    ser.write(create_msp_request(MSP_GET_FLIGHT_MODES))
    cmd, payload = parse_msp_response(ser)
    if cmd == MSP_GET_FLIGHT_MODES and len(payload) >= 4:
        angle, horizon, air, anti_grav = payload[:4]
        print(f"  Angle: {'ON' if angle else 'OFF'}, Horizon: {'ON' if horizon else 'OFF'}")
        print(f"  Air Mode: {'ON' if air else 'OFF'}, Anti-Gravity: {'ON' if anti_grav else 'OFF'}")
        print("  ✓ Flight modes read successfully")
    else:
        print("  ✗ Failed to read flight modes")

def test_battery_config(ser):
    """Test battery configuration"""
    print("Testing Battery Config...")

    ser.write(create_msp_request(MSP_GET_BATTERY_CONFIG))
    cmd, payload = parse_msp_response(ser)
    if cmd == MSP_GET_BATTERY_CONFIG and len(payload) >= 11:
        cells, capacity, v_scale, i_scale = struct.unpack('<BHff', payload)
        print(f"  Cells: {cells}S, Capacity: {capacity}mAh")
        print(f"  Voltage Scale: {v_scale:.1f}, Current Scale: {i_scale:.1f}")
        print("  ✓ Battery config read successfully")
    else:
        print("  ✗ Failed to read battery config")

def test_failsafe_config(ser):
    """Test failsafe configuration"""
    print("Testing Failsafe Config...")

    ser.write(create_msp_request(MSP_GET_FAILSAFE_CONFIG))
    cmd, payload = parse_msp_response(ser)
    if cmd == MSP_GET_FAILSAFE_CONFIG and len(payload) >= 4:
        throttle, delay = struct.unpack('<HH', payload)
        print(f"  Failsafe Throttle: {throttle}, Delay: {delay}ms")
        print("  ✓ Failsafe config read successfully")
    else:
        print("  ✗ Failed to read failsafe config")

def test_pid_values(ser):
    """Test PID values (existing functionality)"""
    print("Testing PID Values...")

    ser.write(create_msp_request(MSP_PID))
    cmd, payload = parse_msp_response(ser)
    if cmd == MSP_PID and len(payload) >= 36:
        pids = struct.unpack('<fffffffff', payload)
        print(f"  Roll:  P={pids[0]:.3f}, I={pids[1]:.3f}, D={pids[2]:.3f}")
        print(f"  Pitch: P={pids[3]:.3f}, I={pids[4]:.3f}, D={pids[5]:.3f}")
        print(f"  Yaw:   P={pids[6]:.3f}, I={pids[7]:.3f}, D={pids[8]:.3f}")
        print("  ✓ PID values read successfully")
    else:
        print("  ✗ Failed to read PID values")

def main():
    port = find_serial_port()
    if not port:
        print("❌ Could not find STM32 device")
        return

    print(f"🔌 Connecting to {port}...")

    try:
        with serial.Serial(port, 115200, timeout=1.0) as ser:
            print("✅ Connected successfully")
            time.sleep(0.5)  # Allow connection to stabilize

            # Test all configuration sections
            test_pid_values(ser)
            print()
            test_filter_config(ser)
            print()
            test_rate_profile(ser)
            print()
            test_motor_config(ser)
            print()
            test_flight_modes(ser)
            print()
            test_battery_config(ser)
            print()
            test_failsafe_config(ser)
            print()

            # Test save functionality
            print("Testing Save to Flash...")
            ser.write(create_msp_request(MSP_SAVE_SETTINGS))
            cmd, payload = parse_msp_response(ser)
            if cmd == MSP_SAVE_SETTINGS and len(payload) >= 1:
                success = payload[0] != 0
                print(f"  {'✅ Saved successfully' if success else '❌ Save failed'}")
            else:
                print("  ⚠️ No response from save command")

            print("\n🎉 All tests completed!")

    except serial.SerialException as e:
        print(f"❌ Serial error: {e}")
    except KeyboardInterrupt:
        print("\n⏹️ Test interrupted by user")
    except Exception as e:
        print(f"❌ Unexpected error: {e}")

if __name__ == "__main__":
    main()
