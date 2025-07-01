# Project Overview

This is a multi-application firmware system for Raspberry Pi Pico boards. The firmware allows a single binary image to run different applications based on DIP switch configuration read at boot time.

## Key Information

- **Main dispatcher**: src/main.c reads DIP switches and calls app functions
- **Build command**: `mkdir build && cd build && cmake .. && make -j4`
- **Output**: pico_multi.uf2
- **DIP switches**: GPIO 2, 3, 4 determine which app runs
- **USB serial numbers**: PICO_000, PICO_001, etc. based on DIP code

## Development Guidelines

- Apps should run in infinite loops and never return
- Use stdio_init_all() for USB serial communication
- Handle commands via stdin/stdout for host communication
- Follow existing code patterns for consistency

## Testing Commands

```bash
# Build the firmware
./build.sh

# Run tests
./build_test.sh

# Clean build artifacts
./clean.sh
```