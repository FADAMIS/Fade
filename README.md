# Fade

**Open-source FPV drone firmware written in Rust.**

Fade takes a safety-first approach to flight controller firmware. Unlike traditional C/C++ firmwares where everything is inherently unsafe, Fade builds its application logic in safe Rust on top of minimal unsafe hardware abstraction layers. The project prioritises performance and a lean feature set — only what a freestyle/racing FPV pilot actually needs.

---

## Table of Contents

- [Getting Started](#getting-started)
- [Supported Hardware](#supported-hardware)
- [Architecture Overview](#architecture-overview)
- [Module Reference](#module-reference)
- [Flight Control Loop](#flight-control-loop)
- [FSP Protocol](#fsp-protocol)
- [Configuration & Flash Storage](#configuration--flash-storage)
- [RC Channel Mapping](#rc-channel-mapping)
- [Contributing](#contributing)
- [Project Status](#project-status)

---

## Getting Started

### Prerequisites

| Dependency | Purpose |
|------------|---------|
| [Rust](https://rustup.rs/) (nightly) | Compiler and toolchain |
| `thumbv7em-none-eabihf` target | ARM Cortex-M7 cross-compilation |
| `flip-link` | Stack-overflow protection linker |
| `arm-none-eabi-objcopy` | Binary generation from ELF |
| `dfu-util` | Flashing over USB DFU |
| `probe-rs` *(optional)* | Debug probe support |

```bash
# Install Rust target and tools
rustup target add thumbv7em-none-eabihf
cargo install flip-link
```

### Building

```bash
# Build for H743 (default target)
make build

# Build for a specific target
make build-f722   # STM32F722
make build-h743   # STM32H743
```

All build commands produce a `firm.bin` binary in the project root.

### Flashing

Put the flight controller into DFU mode (hold boot button while powering on), then:

```bash
make flash-h743   # Build + flash H743
make flash-f722   # Build + flash F722
```

### Useful Commands

```bash
make help          # List all available targets
make verify-usb    # Check if the FC's USB device is detected
make monitor-usb   # Continuously poll for USB connection
make monitor       # Start the FSP serial monitor (requires pyserial)
make clean         # Remove all build artifacts
```

---

## Supported Hardware

| Board | MCU | Feature Flag | Status |
|-------|-----|--------------|--------|
| Sequire H7 V2 | STM32H743 | `stm32h7` (default) | ✅ Supported |
| HGLRC F722 | STM32F722 | `stm32f` | ✅ Supported |

Board-specific pin mappings live in [`src/board_config.rs`](src/board_config.rs). Each board is defined as a struct (`SequireH7V2Pins`, `HGLRCF722Pins`) that maps logical functions (gyro SPI, UART, USB, LED) to physical pins and peripherals. Adding a new board is as simple as adding a new struct and a feature flag.

---

## Architecture Overview

Fade runs on the [Embassy](https://embassy.dev/) async runtime — a `no_std`, `no_alloc` executor for embedded Rust. The firmware is structured as a set of cooperatively-scheduled async tasks that communicate through lock-free channels and signals.

```
┌──────────────────────────────────────────────────────────────┐
│                       main.rs (entry point)                  │
│  Initialises hardware, loads config from flash, spawns tasks │
└───────────┬──────────┬──────────┬──────────┬─────────────────┘
            │          │          │          │
   ┌────────▼───┐ ┌────▼────┐ ┌──▼───┐ ┌───▼────────────────┐
   │ Gyro Task  │ │ RX Task │ │ LED  │ │ USB Task + FSP Task│
   │ (8 kHz)    │ │ (CRSF)  │ │      │ │ (config protocol)  │
   └─────┬──────┘ └────┬────┘ └──────┘ └────────┬───────────┘
         │             │                         │
    GYRO_CHANNEL   RC_CHANNEL              SAVE_SIGNAL
         │             │                         │
   ┌─────▼─────────────▼─────┐           ┌──────▼──────┐
   │  Flight Control Task    │           │ Flash Save  │
   │  PID → Mixer → DShot   │           │   Task      │
   └─────────────────────────┘           └─────────────┘
```

### Task Communication

| Channel / Signal | Type | Direction | Purpose |
|-----------------|------|-----------|---------|
| `GYRO_CHANNEL` | `Channel<[f32; 3], 4>` | Gyro → Flight Control | Filtered gyro rates [roll, pitch, yaw] in °/s |
| `RC_CHANNEL` | `Channel<[u16; 16], 16>` | Receiver → Flight Control | 16 CRSF channel values (172–1811) |
| `LINK_STATS_CHANNEL` | `Channel<u8, 4>` | Receiver → (consumer) | CRSF uplink quality percentage |
| `SHUTDOWN_SIGNAL` | `Signal<()>` | Global | Graceful shutdown broadcast |
| `SAVE_SIGNAL` | `Signal<()>` | FSP → Flash Save | Trigger config persistence |
| `CONFIG` (Mutex) | `Mutex<FlightConfig>` | Shared | Runtime-mutable flight configuration |

---

## Module Reference

### `main.rs` — Entry Point

Initialises all hardware peripherals, loads `FlightConfig` from flash (falling back to defaults), and spawns every async task. This is the only file that touches raw peripherals.

### `board_config.rs` — Pin Mappings

Defines per-board structs that map logical peripheral names to physical STM32 pins and DMA channels. To add a new board:

1. Create a new struct (e.g. `MyBoardPins`)
2. Add a feature flag in `Cargo.toml`
3. Add a `#[cfg(feature = "my_board")]` import in `main.rs`

### `gyro.rs` — ICM42688 Gyroscope Driver

Wraps the `icm426xx` crate with an `Icm42688Manager` that handles:

- Sensor initialisation and register configuration
- Gyro reading at up to 8 kHz (`read_gyro() -> [f32; 3]` in °/s)
- Accelerometer reading (`read_accel() -> [f32; 3]` in g)
- Temperature reading

Configurable parameters: full-scale range (250–2000 °/s), output data rate, anti-aliasing filter bandwidth.

### `filters.rs` — Signal Processing

A Betaflight-inspired filter chain applied to raw gyro data before it reaches the PID controller:

| Filter | Type | Default |
|--------|------|---------|
| `BiquadFilter` | Generic biquad (lowpass / notch / PT1) | — |
| `BiquadFilter3` | 3-axis wrapper | — |
| `GyroFilter` | Full chain: 2× lowpass + 2× dynamic notch + static notch | LP1=250 Hz, LP2=125 Hz |
| `AccelFilter3` | PT1 lowpass for accelerometer | configurable |

`GyroFilter` also handles **automatic gyro calibration** — it detects when the quad is stationary, collects bias samples, and subtracts the bias from all future readings. Calibration parameters (stability time, threshold, deadband) are configurable.

### `pid.rs` — PID Controller

A Betaflight-style PID controller with advanced features:

- **Per-axis controllers** (`PidControllers` wraps roll/pitch/yaw)
- **Derivative on measurement** (not error) to avoid derivative kick
- **D-term lowpass filter** to reduce noise
- **Integrator anti-windup** with clamping and saturation backoff
- **TPA** (Throttle PID Attenuation) — reduces gains at high throttle
- **Feedforward** for sharper stick response
- **Setpoint weighting** to reduce overshoot
- **Angle mode** support (outer P-loop converts angle error → rate setpoint)

```rust
// Rate mode (typical freestyle/racing)
let output = pid_controllers.calculate_rate(&setpoint, &gyro, throttle);

// Angle mode (self-levelling)
let output = pid_controllers.calculate_angle(&setpoint, &gyro, &angles, throttle);
```

### `mixer.rs` — Motor Mixing

Standard X-configuration quadcopter mixer:

```
Motor Layout (top view):
  FL(3)──────FR(1)
    \          /
     \   ↑   /    (↑ = front)
      \     /
    RL(2)──────RR(4)
```

```rust
let [fr, rl, fl, rr] = mix_motors(throttle, roll, pitch, yaw);
// Output range: 0–2000 (maps to DShot 48–2047)
```

Includes over-range scaling — if any motor exceeds 2000, all motors are proportionally scaled down to preserve the control differential.

### `dshot.rs` — DShot Protocol

Generates DShot digital protocol frames for ESC communication:

- Supports **DShot300**, **DShot600**, **DShot1200**
- Frame format: 11-bit throttle + 1-bit telemetry + 4-bit CRC
- Encodes frames into timer compare value arrays ready for DMA transfer
- `write_throttles([u16; 4])` handles all 4 motors in one call

> **Note:** The actual DMA transfer to the timer CCRs is hardware-specific and requires pin/timer configuration for your board. See the TODO in `main.rs`.

### `flight_control.rs` — Flight Control Loop

The central closed-loop controller that ties everything together:

1. **Receives** filtered gyro data from `GYRO_CHANNEL` (runs at gyro rate ~8 kHz)
2. **Polls** `RC_CHANNEL` for latest stick inputs (non-blocking)
3. **Maps** CRSF channels to rate setpoints via `rc_to_setpoints()`
4. **Runs** 3-axis PID via `PidControllers::calculate_rate()`
5. **Mixes** PID output into 4 motor values via `mix_motors()`
6. **Encodes** motor values as DShot600 frames via `DShotManager`
7. **Arming gate** — motors are zeroed unless AUX1 > 1500

### `fsp.rs` — Flight Settings Protocol

Custom binary protocol for bidirectional communication with a PC configurator over USB CDC (virtual serial port). Byte-level state machine with XOR checksums.

| Command | Hex | Description |
|---------|-----|-------------|
| Read PID | `0x50` | Returns 9× f32 (roll P/I/D, pitch P/I/D, yaw P/I/D) |
| Write PID | `0x51` | Accepts axis ID + packed P/I/D values |
| Save Config | `0x52` | Writes current config to flash |
| Send (legacy) | `0x69` | Returns PID gains (backwards compat) |

Response codes: `0x0A` = ACK, `0xC4` = NACK.

### `config.rs` — Configuration System

`FlightConfig` is the master configuration struct containing all persistent settings:

- PID gains (per-axis P/I/D)
- Filter settings (lowpass cutoffs, notch)
- Rate profiles (max rates, expo, super rates)
- Motor config (protocol, idle, poles)
- OSD settings
- Flight modes (angle, horizon, airmode, anti-gravity)
- Battery config
- Failsafe settings
- Gyro/accel calibration biases

Features:
- `#[repr(C)]` layout for deterministic serialisation
- Additive checksum validation
- `to_bytes()` / `from_bytes()` for flash read/write
- `apply_to_pid_controllers()` for applying gains at runtime
- `reserved` field for forward-compatible flash layouts

### `flash.rs` — Flash Storage

Handles persistence of `FlightConfig` to STM32 internal flash:

- **Read:** Direct memory-mapped read from flash address, validated through `from_bytes()`
- **Write:** Sector erase + aligned write via `embassy_stm32::flash::Flash`
- Uses the **last 128K sector** (`0x080E0000`) to avoid conflicting with firmware
- Platform-specific alignment: 32-byte writes on H7, 4-byte on F7

---

## Flight Control Loop

The core control loop runs at the gyro sample rate (~8 kHz). Here's the data flow for a single iteration:

```
RC Stick Input (172–1811)
    │
    ▼
map_channel_symmetric() / map_channel_range()
    │
    ▼
Rate Setpoint (°/s)        Gyro Measurement (°/s)
    │                            │
    └──────────┬─────────────────┘
               ▼
        PID Controller
     ┌─────────┼─────────┐
     ▼         ▼         ▼
   P·e      Ki·∫e     Kd·(dM/dt)
     │         │         │
     └────┬────┘─────────┘
          ▼
    PID Output (roll, pitch, yaw)
          │
          ▼
    mix_motors(throttle, r, p, y)
          │
          ▼
    [FR, RL, FL, RR] (0–2000)
          │
          ▼
    DShot600 Frame Encoding
          │
          ▼
    DMA Buffers → Timer CCRs → ESCs
```

---

## FSP Protocol

FSP (Flight Settings Protocol) is a simple, checksummed binary protocol designed for reliable communication over USB CDC. It uses a 3-state machine:

### Frame Structure

```
State 0: Command byte (1 byte) + XOR checksum (1 byte)
State 1: Parameter (2 bytes big-endian u16) + XOR checksum (1 byte)
State 2: Value (4 bytes big-endian u32/f32) + XOR checksum (1 byte)
```

### Checksum Rules

- **State 0 (command):** `data XOR checksum == 0xFF`
- **State 1–2 (data):** `data XOR checksum == 0x00`

### Example: Read PID Gains

```
TX: [0x50] [checksum]
RX: [0x0A] [36 bytes: 9× f32 big-endian]
```

---

## Configuration & Flash Storage

The configuration lifecycle:

1. **Boot** → `load_config_from_flash()` reads from `0x080E0000`
2. **Validate** → Magic bytes + version + checksum are checked
3. **Fallback** → If invalid (first boot, corrupted), uses `FlightConfig::new()` defaults
4. **Runtime** → Config lives in a `Mutex<FlightConfig>` shared between tasks
5. **Modify** → FSP `0x51` (Write PID) updates the config in RAM
6. **Persist** → FSP `0x52` (Save) signals the flash save task to write

---

## RC Channel Mapping

Fade uses **AETR** channel order (standard CRSF):

| Channel Index | Function | Range | Mapped To |
|---------------|----------|-------|-----------|
| 0 | Aileron (Roll) | 172–1811 | ±670 °/s |
| 1 | Elevator (Pitch) | 172–1811 | ±670 °/s |
| 2 | Throttle | 172–1811 | 0–100% |
| 3 | Rudder (Yaw) | 172–1811 | ±400 °/s |
| 4 | AUX1 (Arm) | 172–1811 | Armed if > 1500 |
| 5–15 | AUX2–AUX12 | 172–1811 | Available for modes |

---

## Contributing

### Project Structure

```
fade/
├── .cargo/config.toml     # Cross-compilation & linker config
├── Cargo.toml             # Dependencies and feature flags
├── Makefile               # Build, flash, and utility targets
├── memory.x               # Active linker memory layout
├── memory.h743.x          # H743 memory map (1MB flash, 512K RAM)
├── memory.f722.x          # F722 memory map
└── src/
    ├── main.rs            # Entry point, HW init, task spawning
    ├── lib.rs             # Crate root, module declarations
    ├── board_config.rs    # Per-board pin mappings
    ├── gyro.rs            # ICM42688 gyroscope driver
    ├── filters.rs         # Biquad/PT1 filter chain
    ├── pid.rs             # 3-axis PID controller
    ├── mixer.rs           # Quadcopter motor mixing
    ├── dshot.rs           # DShot protocol encoder
    ├── flight_control.rs  # Closed-loop flight controller
    ├── fsp.rs             # USB configurator protocol
    ├── config.rs          # FlightConfig struct & serialisation
    └── flash.rs           # Flash read/write for config persistence
```

### Adding a New Board

1. Add a feature flag in `Cargo.toml` — e.g. `stm32f4 = ["embassy-stm32/stm32f405rg"]`
2. Create a new struct in `board_config.rs` with pin mappings
3. Add a `#[cfg(feature = "stm32f4")]` use statement in `main.rs`
4. Add a `memory.f405.x` linker script with the correct flash/RAM layout
5. Add build/flash targets to the `Makefile`

### Adding a New Sensor

1. Create a new module in `src/` (e.g. `baro.rs`)
2. Register it in `lib.rs`: `pub mod baro;`
3. Create an async task function for periodic reads
4. Add a static channel for communication
5. Spawn the task from `main.rs`

### Adding a New FSP Command

1. Define a new command constant in `fsp.rs` (e.g. `const CMD_READ_RATES: u8 = 0x60;`)
2. Add a match arm in the `parse()` method's state machine
3. Build the response using `FlightConfig` data

### Code Style

- All hardware access is confined to `main.rs` and board-specific modules
- Business logic is `no_std` pure Rust — no `unsafe` outside of HAL boundaries
- Use `defmt` for debug logging (stripped in release builds)
- Channels for inter-task communication, `Mutex` for shared state
- Prefer `blocking_` flash operations to avoid executor starvation

### Running Tests

The crate has `#[cfg(test)]` unit tests for platform-independent modules:

```bash
# Note: tests run on the host, not the target
cargo test --lib
```

`pid.rs`, `dshot.rs`, `mixer.rs`, and `config.rs` include native unit tests for logic validation.

---

## Project Status

- [x] Init project — LED blinking on commercial BetaFlight flight controller
- [x] PID logic — Betaflight-style PID regulator with TPA, feedforward, anti-windup
- [x] CRSF protocol — Full CRSF receiver support with channel + link stats parsing
- [x] STM32H743 compatibility
- [x] Gyro/Accel ICM42688 compatibility
- [x] PC configuration — FSP protocol for bidirectional USB CDC communication
- [x] DShot protocol — DShot300/600/1200 frame encoding for ESC communication
- [x] Settings persistence — FlightConfig saved to/loaded from internal flash
- [x] STM32F7 compatibility
- [x] STM32F4 compatibility
- [x] Docs — you're reading them 🚀
- [ ] DMA motor output — wire DShot DMA transfer to timer CCRs (board-specific)
- [ ] Blackbox logging
- [ ] OSD integration
- [ ] GPS support

---

## License

*To be determined.*
