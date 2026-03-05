#!/usr/bin/env python3
"""Read/write PID values on the flight controller via FSP.

Usage:
    python3 write_pid.py <port> --read
    python3 write_pid.py <port> <P> <I> <D> [--save]

Examples:
    python3 write_pid.py /dev/tty.usbmodem800851 --read
    python3 write_pid.py /dev/tty.usbmodem800851 45.0 85.0 35.0
    python3 write_pid.py /dev/tty.usbmodem800851 45.0 85.0 35.0 --save
"""

import serial
import sys
import struct
import time

CMD_SET  = 0x67
CMD_GET  = 0x69
CMD_SAVE = 0xFE
ACK = 0x0A

def complement(b): return b ^ 0xFF
def xor_cs(data):
    acc = 0
    for b in data: acc ^= b
    return acc

def send_cmd(ser, cmd):
    ser.write(bytes([cmd, complement(cmd)]))
    time.sleep(0.02)
    r = ser.read(1)
    if not r or r[0] != ACK:
        raise Exception(f"No ACK for 0x{cmd:02X}")

def get_value(ser, key):
    ser.reset_input_buffer()
    send_cmd(ser, CMD_GET)
    kd = struct.pack(">H", key)
    ser.write(kd + bytes([xor_cs(kd)]))
    time.sleep(0.02)
    r = ser.read(1)
    if not r or r[0] != ACK:
        raise Exception(f"GET key ACK failed for key {key}")
    r = ser.read(5)
    if len(r) < 5:
        raise Exception(f"GET failed for key {key}")
    received_checksum = r[4]
    calculated_checksum = xor_cs(r[:4])
    if received_checksum != calculated_checksum:
        raise Exception(f"GET checksum mismatch for key {key}")
    return struct.unpack(">f", r[:4])[0]

def set_value(ser, key, value):
    ser.reset_input_buffer()
    send_cmd(ser, CMD_SET)
    kd = struct.pack(">H", key)
    ser.write(kd + bytes([xor_cs(kd)]))
    time.sleep(0.02)
    r = ser.read(1)
    if not r or r[0] != ACK: raise Exception("SET key ACK failed")
    vd = struct.pack(">f", value)
    ser.write(vd + bytes([xor_cs(vd)]))
    time.sleep(0.02)
    r = ser.read(1)
    if not r or r[0] != ACK: raise Exception("SET value ACK failed")

def save(ser):
    ser.reset_input_buffer()
    send_cmd(ser, CMD_SAVE)
    time.sleep(0.1)

def read_pid(ser):
    p, i, d = get_value(ser, 0), get_value(ser, 1), get_value(ser, 2)
    print(f"P={p:.3f}  I={i:.3f}  D={d:.3f}")

if __name__ == "__main__":
    args = sys.argv[1:]
    if len(args) < 2:
        print("Usage: write_pid.py <port> --read")
        print("       write_pid.py <port> <P> <I> <D> [--save]")
        sys.exit(1)

    port = args[0]
    do_read = "--read" in args
    do_save = "--save" in args

    ser = serial.Serial(port, 115200, timeout=0.5)
    ser.flush()
    time.sleep(0.5)

    if do_read:
        read_pid(ser)
    else:
        nums = [a for a in args[1:] if not a.startswith("--")]
        if len(nums) < 3:
            print("Error: provide P I D values or use --read")
            sys.exit(1)
        p, i, d = float(nums[0]), float(nums[1]), float(nums[2])

        cur_p, cur_i, cur_d = get_value(ser, 0), get_value(ser, 1), get_value(ser, 2)
        print(f"Current:  P={cur_p:.3f}  I={cur_i:.3f}  D={cur_d:.3f}")

        set_value(ser, 0, p)
        set_value(ser, 1, i)
        set_value(ser, 2, d)

        new_p, new_i, new_d = get_value(ser, 0), get_value(ser, 1), get_value(ser, 2)
        print(f"Written:  P={new_p:.3f}  I={new_i:.3f}  D={new_d:.3f}")

        if do_save:
            save(ser)
            print("Saved to flash ✅")

    ser.close()
