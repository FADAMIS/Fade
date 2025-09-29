import serial
import serial.tools.list_ports
import time
import struct
import threading
import os

from textual.app import App, ComposeResult
from textual.widgets import Header, Footer, Static, Label, Input, Button
from textual.reactive import reactive
from textual.containers import Vertical, Horizontal

# MSP commands
MSP_PID = 112
MSP_SET_PID = 202
MSP_GYRO = 2000
MSP_SAVE_SETTINGS = 2002

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

def create_msp_set_pid_request(pids):
    """Creates an MSP_SET_PID request frame."""
    flag = 0

    payload = bytearray()
    # The order of axes must match what the flight controller expects
    for axis_pids in [pids["roll"], pids["pitch"], pids["yaw"]]:
        payload.extend(struct.pack('<f', axis_pids['p']))
        payload.extend(struct.pack('<f', axis_pids['i']))
        payload.extend(struct.pack('<f', axis_pids['d']))

    payload_size = len(payload)

    frame = bytearray()
    frame.append(ord('$'))
    frame.append(ord('X'))
    frame.append(ord('<'))

    checksum = 0

    frame.append(flag)
    checksum ^= flag

    frame.extend(struct.pack('<H', MSP_SET_PID))
    checksum ^= (MSP_SET_PID & 0xFF)
    checksum ^= ((MSP_SET_PID >> 8) & 0xFF)

    frame.extend(struct.pack('<H', payload_size))
    checksum ^= (payload_size & 0xFF)
    checksum ^= ((payload_size >> 8) & 0xFF)

    frame.extend(payload)
    for byte in payload:
        checksum ^= byte

    frame.append(checksum)

    return frame

def parse_msp_response(ser, timeout_ms=100):
    """Parses an MSPv2 response frame with timeout."""
    start_time = time.time()
    timeout_s = timeout_ms / 1000.0

    try:
        while (time.time() - start_time) < timeout_s:
            if ser.in_waiting > 0:
                # Try to read header
                header = ser.read(3)
                if len(header) < 3:
                    continue

                if header != b'$X>':
                    continue

                flag = ser.read(1)
                if not flag:
                    continue

                command_bytes = ser.read(2)
                if len(command_bytes) < 2:
                    continue
                command = struct.unpack('<H', command_bytes)[0]

                payload_size_bytes = ser.read(2)
                if len(payload_size_bytes) < 2:
                    continue
                payload_size = struct.unpack('<H', payload_size_bytes)[0]

                payload = ser.read(payload_size)
                if len(payload) < payload_size:
                    continue

                checksum_byte = ser.read(1)
                if not checksum_byte:
                    continue

                # Verify checksum
                checksum = 0
                checksum ^= flag[0]
                checksum ^= command_bytes[0]
                checksum ^= command_bytes[1]
                checksum ^= payload_size_bytes[0]
                checksum ^= payload_size_bytes[1]
                for byte in payload:
                    checksum ^= byte

                if checksum == checksum_byte[0]:
                    return command, payload

            time.sleep(0.001)  # Small delay to prevent busy waiting

        return None, None

    except Exception:
        return None, None

class GyroWidget(Static):
    """A widget to display gyro data."""
    gyro_data = reactive({"x": 0.0, "y": 0.0, "z": 0.0})

    def render(self) -> str:
        return f"Gyro: X={self.gyro_data['x']:>8.2f}, Y={self.gyro_data['y']:>8.2f}, Z={self.gyro_data['z']:>8.2f}"

class PidWidget(Static):
    """A widget to display PID data."""
    pid_data = reactive({})

    def render(self) -> str:
        roll = self.pid_data.get("roll", {})
        pitch = self.pid_data.get("pitch", {})
        yaw = self.pid_data.get("yaw", {})
        return (
            f"Roll:  P={roll.get('p', 0):.2f}, I={roll.get('i', 0):.2f}, D={roll.get('d', 0):.2f}\n"
            f"Pitch: P={pitch.get('p', 0):.2f}, I={pitch.get('i', 0):.2f}, D={pitch.get('d', 0):.2f}\n"
            f"Yaw:   P={yaw.get('p', 0):.2f}, I={yaw.get('i', 0):.2f}, D={yaw.get('d', 0):.2f}"
        )

class MspDashboard(App):
    """A Textual dashboard for MSP data."""

    CSS_PATH = "dashboard.css"
    BINDINGS = [("d", "toggle_dark", "Toggle dark mode"), ("q", "quit", "Quit")]

    def __init__(self):
        super().__init__()
        self.ser = None

    def compose(self) -> ComposeResult:
        """Create child widgets for the app."""
        yield Header()
        with Vertical(id="main-container"):
            yield Label("MSP Dashboard", id="title")
            yield GyroWidget(id="gyro")
            yield PidWidget(id="pid")

            with Horizontal(id="pid-inputs"):
                with Vertical():
                    yield Label("Roll")
                    yield Input(placeholder="P", id="roll_p", classes="pid-input")
                    yield Input(placeholder="I", id="roll_i", classes="pid-input")
                    yield Input(placeholder="D", id="roll_d", classes="pid-input")
                with Vertical():
                    yield Label("Pitch")
                    yield Input(placeholder="P", id="pitch_p", classes="pid-input")
                    yield Input(placeholder="I", id="pitch_i", classes="pid-input")
                    yield Input(placeholder="D", id="pitch_d", classes="pid-input")
                with Vertical():
                    yield Label("Yaw")
                    yield Input(placeholder="P", id="yaw_p", classes="pid-input")
                    yield Input(placeholder="I", id="yaw_i", classes="pid-input")
                    yield Input(placeholder="D", id="yaw_d", classes="pid-input")

            with Horizontal(id="button-row"):
                yield Button("Set PIDs", id="set_pids", variant="primary")
                yield Button("Save to Flash", id="save_settings", variant="success")

            yield Static("Not Connected", id="status")
        yield Footer()

    def on_mount(self) -> None:
        """Called when the app is mounted."""
        # Robust CSS loading
        if self.CSS_PATH and not os.path.exists(self.CSS_PATH):
            self.query_one("#status", Static).update(f"CSS file not found: {self.CSS_PATH}. App will run without custom styles.")

        self.msp_thread = threading.Thread(target=self.run_msp_client, daemon=True)
        self.msp_thread.start()

    def on_button_pressed(self, event: Button.Pressed) -> None:
        """Called when buttons are pressed."""
        if event.button.id == "set_pids":
            self.set_pids()
        elif event.button.id == "save_settings":
            self.save_settings()

    def set_pids(self):
        """Set PID values on the flight controller."""
        try:
            pids = {
                "roll": {
                    "p": float(self.query_one("#roll_p", Input).value or 0),
                    "i": float(self.query_one("#roll_i", Input).value or 0),
                    "d": float(self.query_one("#roll_d", Input).value or 0),
                },
                "pitch": {
                    "p": float(self.query_one("#pitch_p", Input).value or 0),
                    "i": float(self.query_one("#pitch_i", Input).value or 0),
                    "d": float(self.query_one("#pitch_d", Input).value or 0),
                },
                "yaw": {
                    "p": float(self.query_one("#yaw_p", Input).value or 0),
                    "i": float(self.query_one("#yaw_i", Input).value or 0),
                    "d": float(self.query_one("#yaw_d", Input).value or 0),
                },
            }

            if self.ser and self.ser.is_open:
                self.ser.write(create_msp_set_pid_request(pids))
                self.query_one("#status", Static).update("Set PIDs command sent.")
            else:
                self.query_one("#status", Static).update("Not connected. Cannot set PIDs.")

        except ValueError:
            self.query_one("#status", Static).update("Invalid PID values. Please enter numbers only.")

    def save_settings(self):
        """Save current settings to flash memory using separate connection."""
        port = find_serial_port()
        if port is None:
            self.query_one("#status", Static).update("Could not find device for save operation.")
            return

        try:
            self.query_one("#status", Static).update("Saving settings to flash...")

            # Use a separate serial connection for save command
            with serial.Serial(port, baudrate=115200, timeout=0.1) as save_ser:
                # Clear buffer
                save_ser.reset_input_buffer()

                # Send save command
                save_command = create_msp_request(MSP_SAVE_SETTINGS)
                save_ser.write(save_command)
                save_ser.flush()

                # Wait for response using the working approach
                command, payload = parse_msp_response(save_ser, timeout_ms=2000)

                if command == MSP_SAVE_SETTINGS:
                    if payload and len(payload) >= 1:
                        status = payload[0]
                        if status == 1:
                            self.query_one("#status", Static).update("✓ Settings saved to flash successfully!")
                        else:
                            self.query_one("#status", Static).update("✗ Failed to save settings to flash.")
                    else:
                        self.query_one("#status", Static).update("Save command acknowledged.")
                else:
                    self.query_one("#status", Static).update("Save command sent (timeout).")

        except Exception as e:
            self.query_one("#status", Static).update(f"Error saving settings: {e}")

    def action_toggle_dark(self) -> None:
        """An action to toggle dark mode."""
        self.dark = not self.dark

    def action_quit(self) -> None:
        """An action to quit the app."""
        self.exit()

    def run_msp_client(self):
        port = find_serial_port()
        if port is None:
            self.query_one("#status", Static).update("Could not find STM32 device.")
            return

        self.query_one("#status", Static).update(f"Connecting to {port}...")

        try:
            with serial.Serial(port, baudrate=115200, timeout=0.05) as ser:
                self.ser = ser
                self.query_one("#status", Static).update(f"Connected to {port}")
                while True:
                    # Request PID data
                    ser.write(create_msp_request(MSP_PID))
                    command, payload = parse_msp_response(ser, timeout_ms=50)
                    if command == MSP_PID and payload and len(payload) == 36:
                        pids = struct.unpack('<fffffffff', payload)
                        pid_data = {
                            "roll": {"p": pids[0], "i": pids[1], "d": pids[2]},
                            "pitch": {"p": pids[3], "i": pids[4], "d": pids[5]},
                            "yaw": {"p": pids[6], "i": pids[7], "d": pids[8]},
                        }
                        self.query_one(PidWidget).pid_data = pid_data

                        # Auto-populate input fields with current values (only once to avoid conflicts)
                        if not self.query_one("#roll_p", Input).value:
                            self.query_one("#roll_p", Input).value = f"{pids[0]:.3f}"
                            self.query_one("#roll_i", Input).value = f"{pids[1]:.3f}"
                            self.query_one("#roll_d", Input).value = f"{pids[2]:.3f}"
                            self.query_one("#pitch_p", Input).value = f"{pids[3]:.3f}"
                            self.query_one("#pitch_i", Input).value = f"{pids[4]:.3f}"
                            self.query_one("#pitch_d", Input).value = f"{pids[5]:.3f}"
                            self.query_one("#yaw_p", Input).value = f"{pids[6]:.3f}"
                            self.query_one("#yaw_i", Input).value = f"{pids[7]:.3f}"
                            self.query_one("#yaw_d", Input).value = f"{pids[8]:.3f}"

                    # Request Gyro data
                    ser.write(create_msp_request(MSP_GYRO))
                    command, payload = parse_msp_response(ser, timeout_ms=50)
                    if command == MSP_GYRO and payload and len(payload) == 12:
                        gyro = struct.unpack('<fff', payload)
                        gyro_data = {"x": gyro[0], "y": gyro[1], "z": gyro[2]}
                        self.query_one(GyroWidget).gyro_data = gyro_data

                    time.sleep(0.02)
        except serial.SerialException as e:
            self.query_one("#status", Static).update(f"Serial error: {e}")
        except Exception as e:
            self.query_one("#status", Static).update(f"An error occurred: {e}")


if __name__ == "__main__":
    app = MspDashboard()
    app.run()
