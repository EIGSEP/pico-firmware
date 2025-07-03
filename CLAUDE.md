# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Raspberry Pi Pico 2 (RP2350) firmware project implementing a multi-application system controlled by DIP switches. The firmware allows switching between different hardware control applications at boot time based on physical switch positions.

## Common Commands

### Building the Firmware

```bash
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
2. **App Registry**: Applications are registered in the `app_table` array with name and function pointer
3. **Execution Model**: Selected app runs in an infinite loop with USB serial communication

### Adding New Applications

1. Create app source files following the pattern:
   ```c
   // app_name.h
   void app_name(void);
   
   // app_name.c
   void app_name(void) {
       // Initialize hardware
       // Wait for USB if needed
       while (1) {
           // App logic
       }
   }
   ```

2. Add app to `app_table` in `src/main.c:29-33`:
   ```c
   {"app_name", app_name}
   ```

3. Update `CMakeLists.txt` to include new source files

4. Increase `MAX_APPS` in `src/main.c` if needed (currently 2)

### Key Components

- **Main Entry**: `src/main.c` - DIP switch reading and app dispatch
- **Default Apps**: `src/blink_app1.c`, `src/blink_app2.c` - LED blinking examples
- **Hardware Apps**: 
  - `apps/motor/` - Stepper motor control
  - `apps/switches/` - GPIO switch network control

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

### Git Subtree Usage

Individual apps can be managed as git subtrees:
```bash
# Add new app from external repo
git subtree add --prefix=apps/newapp https://github.com/org/repo.git main --squash

# Update existing subtree
git subtree pull --prefix=apps/motor https://github.com/org/motor-repo.git main --squash
```

## Important Notes

- The project uses the Pico SDK from the `pico-sdk/` subdirectory
- USB serial device names planned to be unique (PICO_000, PICO_001, etc.)
- No automated tests - testing is hardware-based
- Apps run in infinite loops - ensure proper resource management
- Currently only 2 apps are registered in the dispatch table (MAX_APPS = 2)
- Apps in `apps/` directory need to be integrated into the dispatch system in `src/main.c`