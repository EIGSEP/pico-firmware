# EIGSEP Pico Firmware

[![codecov](https://codecov.io/gh/EIGSEP/pico-firmware/graph/badge.svg?token=WNGHYLBF0U)](https://codecov.io/gh/EIGSEP/pico-firmware)

Multi-application firmware for Raspberry Pi Pico 2 (RP2350) that implements hardware control applications selectable via DIP switches. The firmware supports motor control, temperature monitoring/control, IMU sensors, lidar, and RF switches.

---

## Features

- **Multi-app dispatch system** - Select application at boot via DIP switches (GPIO 2,3,4)
- **JSON command protocol** - Control devices via USB serial at 115200 baud
- **6 integrated applications**:
  - Motor control (stepper motors)
  - Temperature controller (Peltier elements)
  - Temperature monitoring (DS18B20 sensors)
  - IMU sensor interface (BNO08x)
  - Lidar sensor interface
  - RF switch control
- **Python host library** - Control devices from host computer
- **Automatic status updates** - Every 200ms
- **Unique device identification** - USB enumeration as PICO_000, PICO_001, etc.

## Prerequisites

- **Operating System:** Ubuntu (or other Debian-based Linux)
- **Hardware:** Raspberry Pi Pico 2 (RP2350) or Pico (RP2040)
- **Permissions:** You will need `sudo` access to install packages and configure udev rules.

---

## 1. Install Dependencies and Tools

### Build Dependencies
```bash
# Install build dependencies
sudo apt update
sudo apt install build-essential pkg-config libusb-1.0-0-dev cmake

# Install Python dependencies
pip3 install pyserial
```

### Install picotool
Picotool is required for flashing firmware to Pico devices.

```bash
# Clone and build picotool
cd ~/
git clone https://github.com/raspberrypi/picotool.git
cd picotool
mkdir build && cd build
cmake ..
sudo make

# Set up udev rules for non-root access
sudo cp udev/99-picotool.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

## 2. Clone and Build Firmware

```bash
# Clone the firmware repository
git clone git@github.com:EIGSEP/pico-firmware.git
cd pico-firmware

# Initialize all submodules
git submodule update --init lib/cJSON
git submodule update --init lib/BNO08x_Pico_Library
git submodule update --init pico-sdk
cd pico-sdk && git submodule update --init && cd ..

# Build the firmware for Pico 2 (default)
./build.sh

# Or build manually
mkdir build && cd build
PICO_BOARD=pico2 cmake ..
PICO_BOARD=pico2 make -j$(nproc)
```

The build produces `build/pico_multi.uf2` ready for flashing.

---

## 3. Flash Pico Devices

### Single Device (Manual)
1. Hold BOOTSEL button while connecting Pico via USB
2. Copy `build/pico_multi.uf2` to the mounted RPI-RP2 drive
3. Device will reboot automatically

### Multiple Devices (Automated)
Use the provided `flash_picos.py` script:

```bash
# Flash all connected Picos
python3 flash_picos.py --uf2 build/pico_multi.uf2

# Flash with custom parameters
python3 flash_picos.py --uf2 build/pico_multi.uf2 --baud 115200 --timeout 10
```

The script will:
- Find all connected Picos (BOOTSEL or CDC mode)
- Flash each device using its USB serial number
- Read device info after flashing
- Update `devices_info.json` with configurations

## 4. Configure DIP Switches

Set GPIO pins 2, 3, 4 to select the application:
- **000** (0): Motor control
- **001** (1): Temperature controller
- **010** (2): Temperature monitor
- **011** (3): IMU sensor
- **100** (4): Lidar sensor
- **101** (5): RF switch
- **110-111** (6-7): Reserved

## 5. Install Python Host Library (Optional)

For controlling devices from a host computer:

```bash
# Install the picohost package
cd pico-firmware
pip install -e ./picohost

# Test your device
python3 picohost/scripts/test_motor_pico_v2.py    # For motor control
python3 picohost/scripts/test_peltier_v2.py       # For temperature control
python3 picohost/scripts/test_rfswitch_pico_v2.py # For RF switch
```

## Monitor Serial Output

```bash
# Find your device
ls /dev/ttyACM*

# Monitor with minicom
minicom -D /dev/ttyACM0 -b 115200

# Or use the Python monitor
python3 picohost/scripts/monitor_picos.py
```

## Project Structure

- `src/` - Firmware source code for all applications
- `lib/` - Libraries (cJSON, BNO08x, eigsep_command, onewire)
- `picohost/` - Python host control library and test scripts
- `build.sh` - Build script for firmware
- `flash_picos.py` - Multi-device flashing tool
- `devices_info.json` - Device configuration database

## Contributing

See [CLAUDE.md](CLAUDE.md) for detailed development guidelines.

## License

MIT License - see LICENSE file for details.
