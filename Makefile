# Makefile for STM32F7 Rust firmware with USB support

# Default target
.PHONY: all build release flash flash-release clean verify-usb monitor-usb monitor help

# Default build (debug)
all: build

# Build debug version and generate binary
build:
	@echo "Building debug version..."
	@cargo build
	@echo "Generating firm.bin..."
	@arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabihf/debug/main firm.bin
	@echo "✓ Generated firm.bin ($(shell wc -c < firm.bin) bytes)"

# Build release version and generate binary
release:
	@echo "Building release version..."
	@cargo build --release
	@echo "Generating firm.bin..."
	@arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabihf/release/main firm.bin
	@echo "✓ Generated firm.bin ($(shell wc -c < firm.bin) bytes)"

# Flash firmware using dfu-util and verify USB connection
flash: build
	@echo "Flashing firmware..."
	@dfu-util -a 0 -s 0x08000000:leave -D firm.bin
	@echo "Firmware flashed successfully!"


# Flash release firmware and verify USB connection
flash-release: release
	@echo "Flashing release firmware..."
	@dfu-util -a 0 -s 0x08000000 -D firm.bin
	@echo "Release firmware flashed successfully!"

# Verify USB device enumeration on macOS
verify-usb:
	@echo "Checking for STM32F7 USB device..."
	@if system_profiler SPUSBDataType | grep -q "STM32F7 USB CDC"; then \
		echo "✓ USB device detected successfully!"; \
		echo "  Product: STM32F7 USB CDC"; \
		echo "  Manufacturer: STMicroelectronics"; \
		echo "  Location: $(shell system_profiler SPUSBDataType | grep -A 10 "STM32F7 USB CDC" | grep "Location ID" | cut -d ':' -f2 | tr -d ' ')"; \
	elif system_profiler SPUSBDataType | grep -q "STMicroelectronics"; then \
		echo "✓ STMicroelectronics USB device detected!"; \
		echo "  $(shell system_profiler SPUSBDataType | grep -A 2 "STMicroelectronics" | grep "Product" | head -1)"; \
		echo "  Location: $(shell system_profiler SPUSBDataType | grep -A 10 "STMicroelectronics" | grep "Location ID" | head -1 | cut -d ':' -f2 | tr -d ' ')"; \
	else \
		echo "✗ USB device not detected. Please check connections and try again."; \
		echo "  Run 'make monitor-usb' to continuously check for device."; \
		exit 1; \
	fi

# Monitor USB connections until device is detected
monitor-usb:
	@echo "Monitoring for USB device connection..."
	@echo "Press Ctrl+C to stop monitoring."
	@while true; do \
		clear; \
		echo "Checking for STM32F7 USB device... ($(shell date +%H:%M:%S))"; \
		if system_profiler SPUSBDataType | grep -q "STM32F7 USB CDC" || system_profiler SPUSBDataType | grep -q "STMicroelectronics"; then \
			echo "✓ USB device detected!"; \
			system_profiler SPUSBDataType | grep -A 15 -B 2 "STMicroelectronics"; \
			echo "\nDevice successfully enumerated on macOS!"; \
			break; \
		else \
			echo "✗ Device not detected yet. Waiting..."; \
			system_profiler SPUSBDataType | grep -A 2 -B 2 "Product ID"; \
		fi; \
		sleep 2; \
	done

# Clean build artifacts
clean:
	@cargo clean
	@rm -f firm.bin
	@echo "✓ Cleaned build artifacts"

# Show available targets
help:
	@echo "Available targets:"
	@echo "  build          - Build debug version and generate firm.bin"
	@echo "  release        - Build release version and generate firm.bin"
	@echo "  flash          - Build, flash debug firmware via DFU, and verify USB connection"
	@echo "  flash-release  - Build, flash release firmware via DFU, and verify USB connection"
	@echo "  verify-usb     - Check if the STM32F7 USB device is detected on macOS"
	@echo "  monitor-usb    - Continuously monitor for USB device connection"
	@echo "  monitor        - Monitor the USB serial port for messages"
	@echo "  clean          - Clean all build artifacts"
	@echo "  help           - Show this help"

monitor:
	@echo "Checking if pyserial is installed..."
	@python3 -c "import serial" 2>/dev/null || (echo "pyserial not found. Please install it using 'pip3 install pyserial'" && exit 1)
	@echo "Starting USB monitor..."
	@python3 msp_client.py
