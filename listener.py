
import serial
import serial.tools.list_ports

def find_serial_port():
    """Finds the correct serial port for the STM32 device."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "usbmodem" in port.device:
            return port.device
    return None

def main():
    """Reads and prints data from the serial port."""
    port = find_serial_port()
    if port is None:
        print("Could not find STM32 device.")
        return

    print(f"Listening on {port}...")
    with serial.Serial(port) as ser:
        while True:
            line = ser.readline().decode("utf-8").strip()
            if line:
                print(line)

if __name__ == "__main__":
    main()
