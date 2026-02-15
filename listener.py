import serial
import sys
import struct

if len(sys.argv) < 3:
    print("Usage: python listener.py <port> <baudrate>")
    sys.exit(1)

port = sys.argv[1]
baudrate = int(sys.argv[2])

ser = serial.Serial(port, baudrate, timeout=0.001)
ser.flush()

# Send data byte 0x69, then checksum 0xD4 (for 105.0 f32)
print("Sending Commands")

cmds = [b'\x69', b'\x96', b'\x01', b'\x01', b'\x00']

for cmd in cmds:
    ser.write(cmd)
    print(ser.read())

f32list = [];
for i in range(0, 4):
    f32list.append(ser.read())

print(f32list)

byte_data = bytes([b[0] for b in f32list])

print(struct.unpack(">f", byte_data)[0])

ser.close()
