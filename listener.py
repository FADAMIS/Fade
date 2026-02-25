#!/usr/bin/env python3
"""
FSP Flash PID Test — Tests SET/GET/SAVE for PID values over USB CDC.

Protocol:
  SET (0x67): cmd+cs → ACK → key(u16)+cs → ACK → value(f32)+cs → ACK
  GET (0x69): cmd+cs → ACK → key(u16)+cs → ACK + value(f32)
  SAVE(0xFE): cmd+cs → ACK

Keys: P=0, I=1, D=2

Usage: python3 listener.py <port> [baudrate]
"""

import serial
import sys
import struct
import time

# ── Protocol Constants ──
CMD_SET  = 0x67
CMD_GET  = 0x69
CMD_SAVE = 0xFE
ACK = 0x0A

KEY_P, KEY_I, KEY_D = 0, 1, 2
KEY_NAMES = {0: "P", 1: "I", 2: "D"}


def complement(b):
    return b ^ 0xFF


def xor_cs(data: bytes) -> int:
    acc = 0
    for b in data:
        acc ^= b
    return acc


def send_cmd(ser, cmd):
    """Send command + complement, read 1-byte ACK."""
    ser.write(bytes([cmd, complement(cmd)]))
    time.sleep(0.02)
    r = ser.read(1)
    if not r:
        raise TimeoutError(f"No response for cmd 0x{cmd:02X}")
    if r[0] != ACK:
        raise ValueError(f"Expected ACK, got 0x{r[0]:02X}")


def get_value(ser, key):
    """GET a single f32 value by key ID."""
    ser.reset_input_buffer()
    # 1) Send GET command
    send_cmd(ser, CMD_GET)
    # 2) Send KEY (u16 BE + checksum)
    kd = struct.pack(">H", key)
    ser.write(kd + bytes([xor_cs(kd)]))
    time.sleep(0.02)
    # 3) Read ACK + f32 (5 bytes)
    r = ser.read(5)
    if len(r) < 1:
        raise TimeoutError("No GET response")
    if r[0] != ACK:
        raise ValueError(f"GET key {key}: got 0x{r[0]:02X} instead of ACK")
    if len(r) < 5:
        raise TimeoutError(f"Incomplete GET response: {len(r)} bytes")
    return struct.unpack(">f", r[1:5])[0]


def set_value(ser, key, value):
    """SET a single f32 value by key ID."""
    ser.reset_input_buffer()
    # 1) Send SET command
    send_cmd(ser, CMD_SET)
    # 2) Send KEY (u16 BE + checksum)
    kd = struct.pack(">H", key)
    ser.write(kd + bytes([xor_cs(kd)]))
    time.sleep(0.02)
    r = ser.read(1)
    if not r or r[0] != ACK:
        raise ValueError(f"SET key ACK failed: {r}")
    # 3) Send VALUE (f32 BE + checksum)
    vd = struct.pack(">f", value)
    ser.write(vd + bytes([xor_cs(vd)]))
    time.sleep(0.02)
    r = ser.read(1)
    if not r or r[0] != ACK:
        raise ValueError(f"SET value ACK failed: {r}")


def save(ser):
    """Send SAVE command."""
    ser.reset_input_buffer()
    send_cmd(ser, CMD_SAVE)
    time.sleep(0.1)  # Give flash time


def read_all_pid(ser):
    """Read P, I, D values."""
    p = get_value(ser, KEY_P)
    i = get_value(ser, KEY_I)
    d = get_value(ser, KEY_D)
    return p, i, d


def print_pid(p, i, d):
    print(f"  P = {p:8.3f}")
    print(f"  I = {i:8.3f}")
    print(f"  D = {d:8.3f}")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 listener.py <port> [baudrate]")
        sys.exit(1)

    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    print("╔══════════════════════════════════════╗")
    print("║      FSP Flash PID Test Script       ║")
    print("╚══════════════════════════════════════╝\n")

    ser = serial.Serial(port, baud, timeout=0.5)
    ser.flush()
    time.sleep(0.5)

    try:
        # ── 1. Read current PID values ──
        print("━━━ 1. Reading current PID values ━━━")
        orig_p, orig_i, orig_d = read_all_pid(ser)
        print_pid(orig_p, orig_i, orig_d)
        print()

        # ── 2. Write test values ──
        test_p, test_i, test_d = 12.5, 34.75, 56.125
        print(f"━━━ 2. Writing test PID values ━━━")
        print(f"  P = {test_p}, I = {test_i}, D = {test_d}")
        set_value(ser, KEY_P, test_p)
        print("  ✓ P written")
        set_value(ser, KEY_I, test_i)
        print("  ✓ I written")
        set_value(ser, KEY_D, test_d)
        print("  ✓ D written")
        print()

        # ── 3. Read back (in-memory verify) ──
        print("━━━ 3. Reading back (in-memory) ━━━")
        p, i, d = read_all_pid(ser)
        print_pid(p, i, d)
        ok = abs(p - test_p) < 0.01 and abs(i - test_i) < 0.01 and abs(d - test_d) < 0.01
        print(f"  {'✅ Match!' if ok else '❌ Mismatch!'}\n")

        # ── 4. Save to flash ──
        print("━━━ 4. Saving to flash ━━━")
        save(ser)
        print("  ✅ Save acknowledged!\n")

        # ── 5. Verify after save ──
        print("━━━ 5. Reading after save ━━━")
        time.sleep(0.5)
        ser.reset_input_buffer()
        p, i, d = read_all_pid(ser)
        print_pid(p, i, d)
        ok = abs(p - test_p) < 0.01 and abs(i - test_i) < 0.01 and abs(d - test_d) < 0.01
        print(f"  {'✅ Still correct!' if ok else '❌ Changed after save!'}\n")

        # ── 6. Power cycle test ──
        print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        print("  Power cycle the board, then Enter...")
        print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        ser.close()
        input()

        print(f"  Reconnecting to {port}...")
        time.sleep(1)
        ser = serial.Serial(port, baud, timeout=0.5)
        ser.flush()
        time.sleep(1)

        # ── 7. Read after reboot ──
        print("━━━ 6. Reading after power cycle ━━━")
        p, i, d = read_all_pid(ser)
        print_pid(p, i, d)
        ok = abs(p - test_p) < 0.01 and abs(i - test_i) < 0.01 and abs(d - test_d) < 0.01

        if ok:
            print("\n  ╔════════════════════════════════════╗")
            print("  ║  ✅ FLASH TEST PASSED!              ║")
            print("  ║  PID values persisted!              ║")
            print("  ╚════════════════════════════════════╝")
        else:
            print("\n  ╔════════════════════════════════════╗")
            print("  ║  ❌ FLASH TEST FAILED!              ║")
            print("  ╚════════════════════════════════════╝")
            print(f"  Expected: P={test_p} I={test_i} D={test_d}")
        print()

        # ── Restore originals? ──
        r = input("  Restore original values? [y/N]: ").strip().lower()
        if r == 'y':
            set_value(ser, KEY_P, orig_p)
            set_value(ser, KEY_I, orig_i)
            set_value(ser, KEY_D, orig_d)
            save(ser)
            print("  ✅ Originals restored and saved!")

    except (TimeoutError, ValueError) as e:
        print(f"\n  ❌ Error: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n  Interrupted.")
    finally:
        if ser.is_open:
            ser.close()
        print("\n  Done.")


if __name__ == "__main__":
    main()
