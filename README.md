# Pico Multi-App Firmware

Code running on Raspberry Pi Picos in the EIGSEP box. This firmware system allows a single binary image to run different applications based on DIP switch configuration.

## Overview

The firmware allows each Pico board to read 3 GPIO pins (forming a 3-bit code from 000–101) at boot time and dispatch to the corresponding application function. Each board automatically identifies itself with a unique USB serial number (PICO_000, PICO_001, etc.) based on its DIP switch setting.

## Directory Structure

```
.
├── apps/                    ← Individual application directories
│   ├── motor/              ← Motor controller app (stepper control via JSON commands)
│   │   ├── main.c          ← Standalone motor app main function
│   │   ├── motor.c         ← Motor control implementation
│   │   └── motor.h         ← Motor control header
│   ├── switches/           ← Switch network control app
│   │   └── pico_c/         ← C implementation for switch GPIO control
│   │       ├── CMakeLists.txt
│   │       └── src/main.c  ← Switch control main function
│   └── [future apps]/      ← Additional apps (therm, etc.) to be added
├── src/
│   └── main.c              ← Main dispatcher: reads DIP switches, calls app functions
├── CMakeLists.txt          ← Top-level build configuration
└── README.md               ← This file
```

## Building

### Prerequisites

- Raspberry Pi Pico SDK installed and configured
- CMake 3.13 or later
- ARM cross-compilation toolchain (arm-none-eabi-gcc)
- Build tools (make or ninja)

### Build Steps

```bash
# Ensure Pico SDK is configured
export PICO_SDK_PATH=/path/to/pico-sdk

# Create build directory
mkdir build && cd build

# Configure with CMake
cmake ..

# Build the firmware
make -j4

# Output will be: pico_multi.uf2
```

## Flashing

The same `pico_multi.uf2` file is flashed to all boards:

1. Connect Pico to computer while holding BOOTSEL button
2. Copy `build/pico_multi.uf2` to mounted RPI-RP2 drive
3. Board will reboot and run the appropriate app based on DIP switches

## DIP-Switch Configuration

The firmware reads 3 GPIO pins as DIP switches to determine which app to run:

- **DIP0**: GPIO 2 (bit 0)
- **DIP1**: GPIO 3 (bit 1) 
- **DIP2**: GPIO 4 (bit 2)

All DIP switch inputs use pull-down resistors. Setting a switch to HIGH (3.3V) sets that bit to 1.

### App Mapping

| DIP Code (2,1,0) | Binary | Decimal | Application |
|------------------|--------|---------|-------------|
| 000              | b000   | 0       | therm_app() |
| 001              | b001   | 1       | motor_app() |
| 010              | b010   | 2       | switch_app() |
| 011              | b011   | 3       | [future app] |
| 100              | b100   | 4       | [future app] |
| 101              | b101   | 5       | [future app] |
| 110              | b110   | 6       | Invalid (hangs) |
| 111              | b111   | 7       | Invalid (hangs) |

## USB Enumeration

Each board enumerates with a unique USB serial number based on its DIP switch setting:

- DIP code 0 → Serial: "PICO_000"
- DIP code 1 → Serial: "PICO_001"  
- DIP code 2 → Serial: "PICO_002"
- etc.

This allows host systems to identify specific boards even when multiple are connected simultaneously.

## Host Setup

Use Python to map PICO serial numbers to system device paths:

```python
import serial.tools.list_ports

def find_pico_devices():
    """Find all connected PICO devices and map them by ID"""
    pico_devices = {}
    
    for port in serial.tools.list_ports.comports():
        if port.serial_number and port.serial_number.startswith('PICO_'):
            pico_id = port.serial_number  # e.g., "PICO_001"
            pico_devices[pico_id] = port.device  # e.g., "/dev/ttyACM0"
    
    return pico_devices

# Usage example
devices = find_pico_devices()
for pico_id, device_path in devices.items():
    print(f"{pico_id} -> {device_path}")

# Connect to specific device
if 'PICO_001' in devices:
    motor_port = serial.Serial(devices['PICO_001'], 115200)
    # Send motor commands...
```
