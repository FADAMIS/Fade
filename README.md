# Fade

### What is Fade?

Fade is a firmware for FPV drones that apart from other firmware uses Rust for it's safety when programming. We use embedded Rust which in fact will be unsafe inevitably. But the important part is that aside from C/C++ where everything is unsafe, in Rust you build your app logic in safe Rust on top of abstractions which are minority of the code. This project is also focused on speed. We opt to use minimal number of features not necessary for Freestyle/Racing FPV pilot's best experience.

---

### How to use ⚙️

- This is going to build you a firm.bin file for default feature target which is set in **Cargo.toml**

```bash
make build
```

- If you want to build the firmware for specific target without the default feature you can use one of two currently supported targets

```bash
# STM32F722
make build-f722
# STM32H743
make build-h743
```

---

### Current state

- [x] Init project - LED blinking on commercial BetaFlight flight controller
- [x] PID logic - Make a PID regulator for drone stabilization
- [x] CRSF protocol - Implement CRSF protocol to be able to connect and communicate with reciever
- [x] SMT32H743 compatibility
- [x] Gyro/Accel Icm42688 compatibility
- [ ] PC configuration - using custom protocol to communicate with PC and send data bidirectionally. (currently in progress)
- [ ] Dshot protocol - Implement Dshot thus communication with ESCs
- [ ] Settings persistence - Persist settings of the flight controller during power cycle
- [ ] Docs
- [ ] STM32F7 compatibility
- [ ] STM32F4 compatibility
