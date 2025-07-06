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
python3 flash_picos.py --uf2 build/pico_multi.uf2

# Flash with custom parameters
python3 flash_picos.py --uf2 build/pico_multi.uf2 --baud 115200 --timeout 10

# Monitor serial output (adjust /dev/ttyACM0 as needed)
minicom -D /dev/ttyACM0 -b 115200

# Install Python host library for device control
pip install -e ./picohost

# Run device-specific test scripts
python3 picohost/test_motor.py    # Test motor control
python3 picohost/test_tempmon.py  # Test temperature monitoring
python3 picohost/test_imu.py      # Test IMU sensor
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

1. **DIP Switch Selection**: GPIO pins 2, 3, 4 read a 3-bit value (0-7) to select the active application
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

5. Create corresponding Python host class in `picohost/` following existing patterns

### Key Components

- **Main Entry**: `src/main.c` - DIP switch reading and app dispatch
- **Hardware Apps**: 
  - `src/motor.c` - Stepper motor control (APP_MOTOR = 0)
  - `src/tempctrl.c` - Temperature controller (APP_TEMPCTRL = 1)
  - `src/tempmon.c` - Temperature monitoring (APP_TEMPMON = 2)
  - `src/imu.c` - IMU sensor interface (APP_IMU = 3)
  - `src/lidar.c` - Lidar sensor interface (APP_LIDAR = 4)
  - `src/rfswitch.c` - RF switch control (APP_RFSWITCH = 5)
- **Command Protocol**: JSON-based via `lib/eigsep_command/` using cJSON library
- **Python Host Library**: `picohost/` package with device-specific classes

### Hardware Configuration

- **DIP Switches**: GPIO 2, 3, 4 (3-bit selection, pull-up enabled)
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

### flash_picos.py

Python script for flashing and configuring multiple Picos:
- Uses `picotool` to flash via USB serial number
- Reads JSON device info from each Pico after flashing
- Updates `devices_info.json` with device configurations
- Supports both BOOTSEL and CDC serial mode Picos
- Automatically discovers connected Picos

### Python Host Library (picohost)

Complete Python package for controlling Pico devices:
- Device-specific classes for each application type
- Automatic USB device discovery
- Context manager support for clean connection handling
- Background status monitoring thread
- Comprehensive test scripts for each device type

### Libraries and Dependencies

The project includes several libraries:
- **cJSON**: JSON parsing and generation for command protocol
- **eigsep_command**: Custom command handling library built on cJSON
- **onewire**: OneWire protocol library with PIO implementation
- **BNO08x_Pico_Library**: IMU sensor library for BNO08x devices

## Important Notes

- The project uses the Pico SDK from the `pico-sdk/` subdirectory
- USB serial devices enumerate as PICO_000, PICO_001, etc. based on unique board ID
- No automated firmware tests - testing is hardware-based using Python test scripts
- Apps use a command/response architecture with JSON protocol
- Currently 6 apps are integrated (0-5), with slots 6-7 reserved for future use
- All firmware apps are in `src/` directory and fully integrated
- Status reporting occurs every 200ms (`STATUS_CADENCE_MS`)
- LED blinks as a heartbeat indicator during operation
- Python code follows standard patterns with type hints and docstrings