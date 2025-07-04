# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Raspberry Pi Pico 2 (RP2350) firmware project implementing a multi-application system controlled by DIP switches. The firmware allows switching between different hardware control applications at boot time based on physical switch positions.

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

# Set SDK path if not using the included pico-sdk subdirectory
export PICO_SDK_PATH=/path/to/pico-sdk
```

## Architecture Overview

### Application Framework

The firmware implements a multi-app dispatch system in `src/main.c`:

1. **DIP Switch Selection**: GPIO pins 2, 3, 4 read a 3-bit value (0-7) to select the active application
2. **App Dispatch**: Applications are selected via switch statement with defined app IDs
3. **Execution Model**: Selected app runs with three phases:
   - `app_init()` - One-time initialization
   - `app_server()` - Command processing from JSON input
   - `app_op()` - Continuous operations in main loop
   - `app_status()` - Periodic status reporting

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
       // Process JSON commands
   }
   void app_name_op(uint8_t app_id) {
       // Continuous operations
   }
   void app_name_status(uint8_t app_id) {
       // Send status updates
   }
   ```

2. Add app ID to `src/pico_multi.h` (e.g., `#define APP_NEWAPP 6`)

3. Add app to dispatch switches in `src/main.c` (init, server, op, status)

4. Update `CMakeLists.txt` to include new source files

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

### Hardware Configuration

- **DIP Switches**: GPIO 2, 3, 4 (3-bit selection)
- **Watchdog**: 8-second timeout enabled
- **USB Serial**: CDC serial for all apps
- **Target**: RP2350 (Pico 2), also supports RP2040

## Development Tools

### flash_picos.py

Python script for flashing and configuring multiple Picos:
- Uses `picotool` to flash via USB serial number
- Reads JSON device info from each Pico after flashing
- Saves to `device_info_<unique_id>.json` files
- Supports both BOOTSEL and CDC serial mode Picos

### Libraries and Dependencies

The project includes several libraries:
- **cJSON**: JSON parsing and generation for command protocol
- **eigsep_command**: Custom command handling library built on cJSON
- **onewire**: OneWire protocol library with PIO implementation
- **BNO08x_Pico_Library**: IMU sensor library for BNO08x devices

## Important Notes

- The project uses the Pico SDK from the `pico-sdk/` subdirectory
- USB serial device names use unique board IDs for identification
- No automated tests - testing is hardware-based
- Apps use a command/response architecture with JSON protocol
- Currently 6 apps are integrated in the dispatch system (0-5)
- All apps are in `src/` directory and fully integrated
- Status reporting occurs every 200ms (`STATUS_CADENCE_MS`)
- LED blinks as a heartbeat indicator during operation