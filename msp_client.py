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

def create_msp_save_request():
    """Creates an MSP_SAVE_SETTINGS request frame."""
    flag = 0
    payload_size = 0

    frame = bytearray()
    frame.append(ord('$'))
    frame.append(ord('X'))
    frame.append(ord('<'))

    checksum = 0

    frame.append(flag)
    checksum ^= flag

    frame.extend(struct.pack('<H', MSP_SAVE_SETTINGS))
    checksum ^= (MSP_SAVE_SETTINGS & 0xFF)
    checksum ^= ((MSP_SAVE_SETTINGS >> 8) & 0xFF)

    frame.extend(struct.pack('<H', payload_size))
    checksum ^= (payload_size & 0xFF)
    checksum ^= ((payload_size >> 8) & 0xFF)

    frame.append(checksum)

    return frame

def parse_msp_response(ser):
    """Parses an MSPv2 response frame."""
    try:
        header = ser.read(3)
        if header != b'$X>':
            return None, None

        flag = ser.read(1)
        command_bytes = ser.read(2)
        payload_size_bytes = ser.read(2)

        if not flag or not command_bytes or not payload_size_bytes:
            return None, None

        command = struct.unpack('<H', command_bytes)[0]
        payload_size = struct.unpack('<H', payload_size_bytes)[0]

        payload = ser.read(payload_size)
        checksum_byte = ser.read(1)

        if not payload or not checksum_byte:
            return None, None

        checksum = 0
        checksum ^= flag[0]
        checksum ^= command_bytes[0]
        checksum ^= command_bytes[1]
        checksum ^= payload_size_bytes[0]
        checksum ^= payload_size_bytes[1]
        for byte in payload:
            checksum ^= byte

        if checksum != checksum_byte[0]:
            return None, None

        return command, payload
    except serial.SerialException:
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
            f"Current PID Values:\n"
            f"Roll:  P={roll.get('p', 1.0):.3f}, I={roll.get('i', 0.1):.3f}, D={roll.get('d', 0.01):.3f}\n"
            f"Pitch: P={pitch.get('p', 1.0):.3f}, I={pitch.get('i', 0.1):.3f}, D={pitch.get('d', 0.01):.3f}\n"
            f"Yaw:   P={yaw.get('p', 1.0):.3f}, I={yaw.get('i', 0.1):.3f}, D={yaw.get('d', 0.01):.3f}"
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

            with Horizontal(id="buttons"):
                yield Button("Set PIDs", id="set_pids")
                yield Button("Save to Flash", id="save_pids")
                yield Button("Load Defaults", id="load_defaults")
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
        """Handle button press events."""
        if event.button.id == "set_pids":
            try:
                pids = {
                    "roll": {
                        "p": float(self.query_one("#roll_p", Input).value or 1.0),
                        "i": float(self.query_one("#roll_i", Input).value or 0.1),
                        "d": float(self.query_one("#roll_d", Input).value or 0.01),
                    },
                    "pitch": {
                        "p": float(self.query_one("#pitch_p", Input).value or 1.0),
                        "i": float(self.query_one("#pitch_i", Input).value or 0.1),
                        "d": float(self.query_one("#pitch_d", Input).value or 0.01),
                    },
                    "yaw": {
                        "p": float(self.query_one("#yaw_p", Input).value or 1.0),
                        "i": float(self.query_one("#yaw_i", Input).value or 0.1),
                        "d": float(self.query_one("#yaw_d", Input).value or 0.01),
                    },
                }

                if self.ser and self.ser.is_open:
                    self.ser.write(create_msp_set_pid_request(pids))
                    self.query_one("#status", Static).update("✅ Set PIDs command sent.")
                else:
                    self.query_one("#status", Static).update("❌ Not connected. Cannot set PIDs.")

            except ValueError:
                self.query_one("#status", Static).update("❌ Invalid PID values. Please enter numbers only.")

        elif event.button.id == "save_pids":
            if self.ser and self.ser.is_open:
                self.ser.write(create_msp_save_request())
                self.query_one("#status", Static).update("💾 Save PIDs to flash command sent.")
            else:
                self.query_one("#status", Static).update("❌ Not connected. Cannot save PIDs.")

        elif event.button.id == "load_defaults":
            # Load default values into input fields
            self.query_one("#roll_p", Input).value = "1.0"
            self.query_one("#roll_i", Input).value = "0.1"
            self.query_one("#roll_d", Input).value = "0.01"
            self.query_one("#pitch_p", Input).value = "1.0"
            self.query_one("#pitch_i", Input).value = "0.1"
            self.query_one("#pitch_d", Input).value = "0.01"
            self.query_one("#yaw_p", Input).value = "1.0"
            self.query_one("#yaw_i", Input).value = "0.1"
            self.query_one("#yaw_d", Input).value = "0.01"
            self.query_one("#status", Static).update("🔄 Default PID values loaded into inputs.")

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
            with serial.Serial(port, baudrate=115200, timeout=0.1) as ser:
                self.ser = ser
                self.query_one("#status", Static).update(f"Connected to {port}")
                while True:
                    # Request PID data
                    ser.write(create_msp_request(MSP_PID))
                    command, payload = parse_msp_response(ser)
                    if command == MSP_PID and payload and len(payload) == 36:
                        pids = struct.unpack('<fffffffff', payload)
                        pid_data = {
                            "roll": {"p": pids[0], "i": pids[1], "d": pids[2]},
                            "pitch": {"p": pids[3], "i": pids[4], "d": pids[5]},
                            "yaw": {"p": pids[6], "i": pids[7], "d": pids[8]},
                        }
                        self.query_one(PidWidget).pid_data = pid_data

                        # Update input fields with current values
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
                    command, payload = parse_msp_response(ser)
                    if command == MSP_GYRO and payload and len(payload) == 12:
                        gyro = struct.unpack('<fff', payload)
                        gyro_data = {"x": gyro[0], "y": gyro[1], "z": gyro[2]}
                        self.query_one(GyroWidget).gyro_data = gyro_data

                    time.sleep(0.05)
        except serial.SerialException as e:
            self.query_one("#status", Static).update(f"Serial error: {e}")
        except Exception as e:
            self.query_one("#status", Static).update(f"An error occurred: {e}")


if __name__ == "__main__":
    app = MspDashboard()
    app.run()
