## Makefile Usage

This project uses a `Makefile` to simplify common tasks.

### `make firmware`

Builds the release version of the firmware and creates a `firm.bin` file.

```
make firmware
```

### `make flash`

Builds and flashes the firmware to the target device.

```
make flash
```

### `make clean`

Cleans the build artifacts.

```
make clean
```

### `make all`

This is the default command, and it's the same as `make firmware`.

```
make all
```

### `make gdb`

Starts a debugging session with GDB and OpenOCD.

```
make gdb
```

### `make clippy`

Runs Clippy, the Rust linter, on the project.

```
make clippy
```

### `make size`

Displays the size of the compiled firmware.

```
make size
```

### `make run`

Runs the firmware on the target device using `probe-run`.

```
make run
```
