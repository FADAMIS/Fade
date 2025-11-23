import serial
import sys
import time

if len(sys.argv) < 3:
    print("Usage: python listener.py <port> <baudrate>")
    sys.exit(1)

port = sys.argv[1]
baudrate = int(sys.argv[2])

ser = serial.Serial(port, baudrate, timeout=5)
ser.dtr = True
ser.rts = True
ser.flush()

# Send data byte 0x69, then checksum 0xD4 (for 105.0 f32)
print("Sending 0x69 0xD4")
ser.write(b'\x69\xD4')
ser.flush()
time.sleep(0.1)

# Read response
print("Reading response")
response = ser.read(10)
print("Response:", response.hex() if response else "No response")

ser.close()