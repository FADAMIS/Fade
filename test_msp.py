#!/usr/bin/env python3
"""
Simple MSP test script to verify gyro data from flight controller.
This script connects to the flight controller and requests gyro data via MSP protocol.
"""

import serial
import serial.tools.list_ports
import struct
import time

# MSP commands
MSP_GYRO = 2000

def find_serial_port():
    """Finds the correct serial port for the STM32 device."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "usbmodem" in port.device:
            return port.device
    return None

def create_msp_request(command):
    """Creates an MSPv2 request frame."""
    flag = 0
    payload_size = 0

    frame = bytearray()
    frame.append(ord('$'))
    frame.append(ord('X'))
    frame.append(ord('<'))

    checksum = 0

    frame.append(flag)
    checksum ^= flag

    frame.extend(struct.pack('<H', command))
    checksum ^= (command & 0xFF)
    checksum ^= ((command >> 8) & 0xFF)

    frame.extend(struct.pack('<H', payload_size))
    checksum ^= (payload_size & 0xFF)
    checksum ^= ((payload_size >> 8) & 0xFF)

    frame.append(checksum)

    return frame

def parse_msp_response(ser):
    """Parses an MSPv2 response frame."""
    try:
        # Read header
        header = ser.read(3)
        if header != b'$X>':
            print(f"Invalid header: {header}")
            return None, None

        flag = ser.read(1)
        command_bytes = ser.read(2)
        payload_size_bytes = ser.read(2)

        if not flag or not command_bytes or not payload_size_bytes:
            print("Incomplete frame header")
            return None, None

        command = struct.unpack('<H', command_bytes)[0]
        payload_size = struct.unpack('<H', payload_size_bytes)[0]

        payload = ser.read(payload_size)
        checksum_byte = ser.read(1)

        if not payload or not checksum_byte:
            print("Incomplete frame payload")
            return None, None

        # Verify checksum
        checksum = 0
        checksum ^= flag[0]
        checksum ^= command_bytes[0]
        checksum ^= command_bytes[1]
        checksum ^= payload_size_bytes[0]
        checksum ^= payload_size_bytes[1]
        for byte in payload:
            checksum ^= byte

        if checksum != checksum_byte[0]:
            print(f"Checksum mismatch: expected {checksum}, got {checksum_byte[0]}")
            return None, None

        return command, payload
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        return None, None

def main():
    """Main test function."""
    port = find_serial_port()
    if port is None:
        print("Could not find STM32 device. Make sure it's connected.")
        return

    print(f"Connecting to {port}...")

    try:
        with serial.Serial(port, baudrate=115200, timeout=0.5) as ser:
            print("Connected! Testing MSP gyro data...")

            for i in range(10):  # Test 10 times
                # Send gyro request
                request = create_msp_request(MSP_GYRO)
                ser.write(request)

                # Read response
                command, payload = parse_msp_response(ser)

                if command == MSP_GYRO and payload and len(payload) == 12:
                    # Parse gyro data (3 floats)
                    gyro = struct.unpack('<fff', payload)
                    print(f"Gyro {i+1:2d}: X={gyro[0]:8.2f}, Y={gyro[1]:8.2f}, Z={gyro[2]:8.2f} deg/s")
                else:
                    print(f"Failed to read gyro data on attempt {i+1}")

                time.sleep(0.1)  # 10Hz update rate

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Unexpected error: {e}")

if __name__ == "__main__":
    main()
