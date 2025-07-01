# Raspberry Pi Pico C Implementation

This directory contains the C implementation of the switch network functionality for the Raspberry Pi Pico, replacing the MicroPython version.

## Features

- Reads commands from USB serial (stdio)
- Controls 8 GPIO pins based on received commands
- Support for verification commands (commands ending with '!')
- Compatible with the existing Python switch network interface

## GPIO Pin Configuration

The following GPIO pins are used for switches:
- GPIO 6, 5, 11, 3, 15, 0, 8, 16

## Building

### Prerequisites

1. Install the Raspberry Pi Pico SDK
2. Set the `PICO_SDK_PATH` environment variable:
   ```bash
   export PICO_SDK_PATH=/path/to/pico-sdk
   ```

### Build Process

Run the build script:
```bash
./build.sh
```

Or manually:
```bash
mkdir build
cd build
cmake ..
make -j$(nproc)
```

## Flashing to Pico

1. Hold the BOOTSEL button while connecting the Pico to USB
2. Copy `build/switch_network.uf2` to the RPI-RP2 drive that appears
3. The Pico will automatically reboot with the new firmware

## Command Protocol

The C implementation maintains the same protocol as the MicroPython version:

### Basic Commands
- Send a string of 8 characters (0s and 1s) followed by newline
- Example: `10110000\n` sets the corresponding GPIO pins

### Verification Commands
- Append '!' to request verification: `10110000!\n`
- Pico responds with `STATES:xxxxxxxx\n` showing current pin states

## Differences from MicroPython Version

- Uses USB CDC for serial communication instead of UART
- Slightly faster response times
- Lower memory usage
- No runtime Python interpreter overhead