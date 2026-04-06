# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Raspberry Pi Pico 2 (RP2350) firmware project implementing a multi-application system controlled by DIP switches. The firmware allows switching between different hardware control applications at boot time based on physical switch positions.

The project includes both the firmware and a Python host library (`picohost`) for controlling devices from a host computer.

## Common Commands

### Building the Firmware

```bash
# Initialize submodules first (if not already done)
git submodule update --init lib/cJSON
git submodule update --init lib/BNO08x_Pico_Library
git submodule update --init pico-sdk
cd pico-sdk && git submodule update --init && cd ..

# Build for Pico 2 (default target)
./build.sh

# Manual build process
mkdir build && cd build
PICO_BOARD=pico2 cmake ..
PICO_BOARD=pico2 make -j$(nproc)

# Clean build artifacts
./clean.sh
```

The build produces `build/pico_multi.uf2` which can be flashed to the Pico by copying it while in BOOTSEL mode.

### Flashing and Development Commands

```bash
# Flash multiple Picos at once
flash-picos --uf2 build/pico_multi.uf2

# Flash with custom parameters
flash-picos --uf2 build/pico_multi.uf2 --baud 115200 --timeout 10

# Monitor serial output (adjust /dev/ttyACM0 as needed)
minicom -D /dev/ttyACM0 -b 115200

# Install Python host library for device control
pip install -e ./picohost

# Install with dev dependencies (pytest, coverage, etc.)
pip install -e ./picohost[dev]

# Run automated tests (emulator-based, no hardware needed)
cd picohost && pytest

# Run hardware test scripts
python3 picohost/scripts/test_motor_pico.py
python3 picohost/scripts/test_rfswitch_pico.py
```

### Setup Requirements

```bash
# Install build dependencies (Ubuntu/Debian)
sudo apt update
sudo apt install build-essential pkg-config libusb-1.0-0-dev cmake

# Install picotool from source
cd ~/
git clone https://github.com/raspberrypi/picotool.git
cd picotool
mkdir build && cd build
cmake .. && make
sudo cp picotool /usr/local/bin/

# Set up udev rules for non-root access
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="2e8a", MODE="0666"' | sudo tee /etc/udev/rules.d/99-pico.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# Install Python dependencies
pip3 install pyserial
```

## Architecture Overview

### Application Framework

The firmware implements a multi-app dispatch system in `src/main.c`:

1. **DIP Switch Selection**: GPIO pins 20, 21, 22 read a 3-bit value (0-7) to select the active application
2. **App Dispatch**: Applications are selected via switch statement with defined app IDs
3. **Execution Model**: Selected app runs with four phases:
   - `app_init()` - One-time initialization
   - `app_server()` - Command processing from JSON input
   - `app_op()` - Continuous operations in main loop
   - `app_status()` - Periodic status reporting (every 200ms)

### Adding New Applications

1. Create app source files following the pattern:
   ```c
   // app_name.h
   void app_name_init(uint8_t app_id);
   void app_name_server(uint8_t app_id, const char *line);
   void app_name_op(uint8_t app_id);
   void app_name_status(uint8_t app_id);
   
   // app_name.c
   void app_name_init(uint8_t app_id) {
       // Initialize hardware for this app
   }
   void app_name_server(uint8_t app_id, const char *line) {
       // Process JSON commands using eigsep_command
   }
   void app_name_op(uint8_t app_id) {
       // Continuous operations
   }
   void app_name_status(uint8_t app_id) {
       // Send status updates using send_json()
   }
   ```

2. Add app ID to `src/pico_multi.h` (e.g., `#define APP_NEWAPP 6`)

3. Add app to dispatch switches in `src/main.c` (init, server, op, status)

4. Update `CMakeLists.txt` to include new source files

5. Create corresponding Python host class in `picohost/src/picohost/` following existing patterns

6. Create a firmware emulator in `picohost/src/picohost/emulators/` (CI enforces every app has one)

### Key Components

- **Main Entry**: `src/main.c` - DIP switch reading and app dispatch
- **Hardware Apps**: 
  - `src/motor.c` - Stepper motor control (APP_MOTOR = 0)
  - `src/tempctrl.c` - Temperature controller (APP_TEMPCTRL = 1)
  - `src/tempmon.c` - Temperature monitoring (APP_TEMPMON = 2)
  - `src/imu.c` - IMU sensor interface (APP_IMU_EL = 3, APP_IMU_AZ = 6)
  - `src/lidar.c` - Lidar sensor interface (APP_LIDAR = 4)
  - `src/rfswitch.c` - RF switch control (APP_RFSWITCH = 5)
- **Shared Utilities**:
  - `src/temp_simple.c` - Reusable DS18B20 temperature sensor helper (used by tempctrl and tempmon)
- **Command Protocol**: JSON-based via `lib/eigsep_command/` using cJSON library
  - **`send_json(count, ...)`**: The first argument is the number of KV entries — it must exactly match the actual entries or fields will be silently dropped. Always verify the count when adding or removing fields.
- **Python Host Library**: `picohost/src/picohost/` - src-layout package with device-specific classes
- **Firmware Emulators**: `picohost/src/picohost/emulators/` - one emulator per app (apps sharing firmware can share an emulator via `// emulator: <name>` annotation in `pico_multi.h`)
- **Tests**: `picohost/tests/` - pytest suite covering emulators, protocol conformance, and integration

### Hardware Configuration

- **DIP Switches**: GPIO 20, 21, 22 (3-bit selection, pull-up enabled)
- **Watchdog**: 8-second timeout enabled
- **USB Serial**: CDC serial for all apps (VID:0x2E8A, PID:0x0009)
- **Target**: RP2350 (Pico 2), also supports RP2040
- **LED**: Default Pico LED for heartbeat/status indication

## Command Protocol

All applications use a JSON-based command/response protocol over USB serial at 115200 baud:

- **Request Format**: `{"cmd": "command_name", "param1": value1, ...}\n`
- **Response Format**: `{"status": "ok/error", "data": {...}}\n`
- **Status Updates**: Automatically sent every 200ms with app-specific data
- **Line-based**: Each JSON message terminated with newline

Example using Python host library:
```python
from picohost import MotorDevice

with MotorDevice() as motor:
    motor.set_azimuth_position(1000)
    motor.set_elevation_position(500)
    print(motor.get_status())
```

## Development Tools

### flash-picos

CLI tool (installed as `flash-picos` entry point from picohost package) for flashing and configuring multiple Picos:
- Uses `picotool` to flash via USB serial number
- Reads JSON device info from each Pico after flashing
- Updates `devices_info.json` with device configurations
- Supports both BOOTSEL and CDC serial mode Picos
- Automatically discovers connected Picos

### Python Host Library (picohost)

Complete Python package (`picohost/src/picohost/`, src-layout) for controlling Pico devices:
- Device-specific classes for each application type
- Automatic USB device discovery
- Context manager support for clean connection handling
- Background status monitoring thread
- Firmware emulators (`picohost/src/picohost/emulators/`) enabling automated testing without hardware
- Dummy device classes (`picohost/src/picohost/testing.py`) that wire emulators to mock serial for tests
- Utility scripts in `picohost/scripts/` for hardware testing and device control

### Libraries and Dependencies

The project includes several libraries:
- **cJSON**: JSON parsing and generation for command protocol
- **eigsep_command**: Custom command handling library built on cJSON
- **onewire**: OneWire protocol library with PIO implementation
- **BNO08x_Pico_Library**: IMU sensor library for BNO08x devices

## CI / Release

- **CI**: GitHub Actions runs pytest across Python 3.9-3.12 on every push/PR, plus a check that every firmware app has a corresponding emulator
- **Release**: release-please automates version bumps and changelogs
- **Conventional commits**: Use conventional commit prefixes — `fix:`, `feat:`, `chore:`, `test:`, `ci:`, `docs:`, `refactor:`, etc. release-please uses these to determine version bumps and generate changelogs

## Important Notes

- The project uses the Pico SDK from the `pico-sdk/` subdirectory
- USB serial devices enumerate as PICO_000, PICO_001, etc. based on unique board ID
- Automated tests use firmware emulators (no hardware needed); hardware testing uses scripts in `picohost/scripts/`
- Apps use a command/response architecture with JSON protocol
- Currently 7 apps are integrated (0-6), with slot 7 reserved for future use
- All firmware apps are in `src/` directory and fully integrated
- Status reporting occurs every 200ms (`STATUS_CADENCE_MS`)
- LED blinks as a heartbeat indicator during operation
- Python code follows standard patterns with type hints and docstrings