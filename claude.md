# Pico Multi-App Firmware

## Overview

This repository contains a multi-application firmware system for Raspberry Pi Pico boards. The firmware allows a single binary image to run different applications based on DIP switch configuration read at boot time. Each Pico board reads 3 GPIO pins (forming a 3-bit code from 000–101) and dispatches to the corresponding application function.

The system supports up to 6 different applications in a single firmware image, with each board automatically identifying itself with a unique USB serial number (PICO_000, PICO_001, etc.) based on its DIP switch setting.

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
├── CMakeLists.txt          ← Top-level build configuration (to be created)
└── README.md               ← Basic project information
```

## Building

### Prerequisites

- Raspberry Pi Pico SDK installed and configured
- CMake 3.13 or later
- ARM cross-compilation toolchain (arm-none-eabi-gcc)
- Build tools (make or ninja)

### Environment Setup

Ensure the Pico SDK is properly configured:

```bash
export PICO_SDK_PATH=/path/to/pico-sdk
```

### Build Steps

```bash
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

## Updating Apps

### Adding a New App

1. **Create app directory**: `mkdir apps/newapp`

2. **Implement app function**: Create your app's main function with signature:
   ```c
   void newapp_app(void) {
       // Your app implementation
       while (1) {
           // Main app loop
       }
   }
   ```

3. **Create app header**: `apps/newapp/newapp.h` with function declaration

4. **Update main dispatcher**: Edit `src/main.c`:
   ```c
   #include "newapp.h"  // Add include
   
   // Update switch statement in main()
   switch (code) {
       case 0:  therm_app();  break;
       case 1:  motor_app();  break;
       case 2:  switch_app(); break;
       case 3:  newapp_app(); break;  // Add new case
       // ...
   }
   ```

5. **Update CMakeLists.txt**: Add new app subdirectory and source files

6. **Test**: Set DIP switches to unused code and verify app runs correctly

### App Implementation Guidelines

- Apps should run in infinite loops and never return
- Use `stdio_init_all()` for USB serial communication
- Handle commands via stdin/stdout for host communication
- Implement graceful error handling and recovery
- Follow existing code patterns for consistency

## Troubleshooting

### Build Issues

**CMake can't find Pico SDK**:
```bash
export PICO_SDK_PATH=/path/to/pico-sdk
# Or set in CMakeLists.txt: set(PICO_SDK_PATH "/path/to/pico-sdk")
```

**Missing toolchain**:
```bash
# Ubuntu/Debian
sudo apt install gcc-arm-none-eabi

# macOS
brew install arm-none-eabi-gcc
```

### Flash/Boot Issues

**Board doesn't enumerate with expected serial number**:
- Verify DIP switch connections and settings
- Check GPIO pins 2, 3, 4 are properly connected
- Ensure pull-down resistors are working correctly

**App doesn't start**:
- Check DIP switch code matches expected app mapping
- Verify app function is properly implemented and linked
- Use debug output to verify code reading logic

**USB connection issues**:
- Try different USB cable/port
- Check if drivers are installed correctly
- Verify board is recognized in device manager

### Development Issues

**Serial communication not working**:
- Ensure `stdio_init_all()` is called in app
- Check USB vs UART stdio configuration
- Verify baud rate matches host expectations (typically 115200)

**Multiple boards conflicting**:
- Verify each board has unique DIP switch setting
- Check serial numbers are being set correctly
- Use device enumeration code to identify specific boards