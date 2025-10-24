import serial
import serial.tools.list_ports
import struct
import sys


def checksum(data: bytes) -> int:
    result: int = 0
    for byte in data:
        result ^= byte
    return result ^ 0xFF


def encode_cmd(cmd: int, key: int = 0, value: float = 0.0) -> bytes:
    packet: bytearray = bytearray([cmd, checksum(bytes([cmd]))])
    
    if cmd not in [0xDF, 0xFE]:
        key_bytes: bytes = struct.pack('>H', key)
        packet.extend(key_bytes)
        packet.append(checksum(key_bytes))
        
        value_bytes: bytes = struct.pack('>f', value)
        packet.extend(value_bytes)
        packet.append(checksum(value_bytes))
    
    return bytes(packet)


def find_port() -> str:
    ports = list(serial.tools.list_ports.comports())
    
    for i, p in enumerate(ports):
        print(f"[{i}] {p.device}: {p.description}")
    
    if not ports:
        print("No ports found")
        sys.exit(1)
    
    choice: str = input("Port [0]: ").strip() or "0"
    idx: int = int(choice)
    return ports[idx].device

def main() -> None:
    port: str = find_port()
    ser: serial.Serial = serial.Serial(port, 115200, timeout=1)
    ser.reset_input_buffer()  # Clear any stale data
    ser.reset_output_buffer()
    print(f"Connected to {port}\n")
    
    while True:
        cmd_input: str = input("> ").strip()
        if cmd_input in ["q", "quit"]:
            break
        
        parts: list[str] = cmd_input.split()
        cmd: int = int(parts[0], 16 if parts[0].startswith("0x") else 10)
        
        packet: bytes
        acks: int
        
        if cmd in [0xDF, 0xFE]:
            packet = encode_cmd(cmd)
            acks = 1
        else:
            key: int = int(parts[1])
            value: float = float(parts[2])
            packet = encode_cmd(cmd, key, value)
            acks = 3
        
        print(f"TX: {' '.join(f'{b:02X}' for b in packet)}")
        
        # Send byte by byte with small delay (optional, try without first)
        for byte in packet:
            _ = ser.write(bytes([byte]))
            ser.flush()  # Ensure it's sent immediately
        
        # Or just send all at once (should work fine):
        # ser.write(packet)
        # ser.flush()
        
        for i in range(acks):
            resp: bytes = ser.read(1)
            if resp:
                resp_byte: int = resp[0]
                status: str = 'ACK' if resp_byte == 0x0A else 'NACK' if resp_byte == 0xC4 else 'UNK'
                print(f"RX [{i+1}/{acks}]: {status} (0x{resp_byte:02X})")
            else:
                print(f"RX [{i+1}/{acks}]: TIMEOUT")
        print()
    
    ser.close()

if __name__ == "__main__":
    main()
