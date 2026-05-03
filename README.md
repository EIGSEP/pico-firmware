# EIGSEP Pico Firmware

[![codecov](https://codecov.io/gh/EIGSEP/pico-firmware/graph/badge.svg?token=WNGHYLBF0U)](https://codecov.io/gh/EIGSEP/pico-firmware)

Multi-application firmware for Raspberry Pi Pico 2 (RP2350) that implements hardware control applications selectable via DIP switches. The firmware supports motor control, temperature monitoring/control, IMU sensors, lidar, and RF switches.

---

## Features

- **Multi-app dispatch system** - Select application at boot via DIP switches (GPIO 20,21,22)
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
sudo apt install build-essential pkg-config libusb-1.0-0-dev cmake \
gcc-arm-none-eabi libstdc++-arm-none-eabi-newlib

# Install Python dependencies
pip3 install pyserial
```
---

## 2. Get the Firmware

You have two options: download a prebuilt `pico_multi.uf2` from a GitHub Release, or build it yourself from source. Most users only need the prebuilt artifact.

### Option A: Download prebuilt firmware (recommended)

Each tagged release has `pico_multi.uf2` attached as an asset, built from the matching source tree by CI.

- **Browser:** open <https://github.com/EIGSEP/pico-firmware/releases>, pick a release (e.g. `picohost-v3.1.0`), and download `pico_multi.uf2` from the **Assets** section.
- **Command line (pinned, reproducible):**
  ```bash
  gh release download picohost-v3.1.0 --pattern pico_multi.uf2 --repo EIGSEP/pico-firmware
  # or without gh:
  curl -L -o pico_multi.uf2 \
    https://github.com/EIGSEP/pico-firmware/releases/download/picohost-v3.1.0/pico_multi.uf2
  ```
- **Latest non-prerelease:**
  ```bash
  curl -L -o pico_multi.uf2 \
    https://github.com/EIGSEP/pico-firmware/releases/latest/download/pico_multi.uf2
  ```

Skip ahead to [Flash Pico Devices](#5-flash-pico-devices) once you have the `.uf2`.

### Option B: Build from source

Clone with all submodules: `git clone --recurse-submodules git@github.com:EIGSEP/pico-firmware.git`


```bash
# Clone the firmware repository
git clone git@github.com:EIGSEP/pico-firmware.git
cd pico-firmware

# Initialize all submodules
git submodule update --init lib/cJSON
git submodule update --init lib/BNO08x_Pico_Library
git submodule update --init pico-sdk
cd pico-sdk && git submodule update --init && cd ..

#Pico-sdk path must be in bashrc for the picotool make to work.
echo 'export PICO_SDK_PATH=$HOME/pico-firmware/pico-sdk' >> ~/.bashrc
source ~/.bashrc
```

### 3. Install picotool
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
cd ..
sudo cp udev/99-picotool.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```
### 4. Build the firmware for Pico 2 (default)
```bash
./build.sh

# Or build manually
mkdir build && cd build
PICO_BOARD=pico2 cmake ..
PICO_BOARD=pico2 make -j$(nproc)
```

The build produces `build/pico_multi.uf2` ready for flashing.

---

## 5. Flash Pico Devices

In the examples below, `pico_multi.uf2` is either the file you downloaded from a release or the one produced at `build/pico_multi.uf2` by `./build.sh`.

### Single Device (Manual)
1. Hold BOOTSEL button while connecting Pico via USB
2. Copy `pico_multi.uf2` to the mounted RPI-RP2 drive
3. Device will reboot automatically

### Multiple Devices (Automated)
Use the `flash-picos` CLI (installed with the `picohost` package):

```bash
# Flash all connected Picos
flash-picos --uf2 pico_multi.uf2

# Flash with custom parameters
flash-picos --uf2 pico_multi.uf2 --baud 115200 --timeout 10
```

The script will:
- Find all connected Picos (BOOTSEL or CDC mode)
- Flash each device using its USB serial number
- Read device info after flashing
- Update `devices_info.json` with configurations

## Hardware Reference

### App Dispatch (DIP Switches)

GPIO pins 20 (DIP0), 21 (DIP1), 22 (DIP2) select the active application at boot:

| DIP Code | Binary | App ID | Application | Description |
|----------|--------|--------|-------------|-------------|
| 0 | 000 | `APP_MOTOR` | Motor control | Stepper motors for AZ/EL axes |
| 1 | 001 | `APP_TEMPCTRL` | Temperature controller | Peltier control for LNA & LOAD |
| 2 | 010 | `APP_POTMON` | Potentiometer monitor | AZ/EL position feedback |
| 3 | 011 | `APP_IMU_EL` | IMU (elevation) | BNO08x on elevation axis |
| 4 | 100 | `APP_LIDAR` | Lidar | Distance measurement |
| 5 | 101 | `APP_RFSWITCH` | RF switch | Signal path switching |
| 6 | 110 | `APP_IMU_AZ` | IMU (azimuth) | BNO08x on azimuth axis |
| 7 | 111 | — | Reserved | — |

### Temperature Controller Wiring (APP_TEMPCTRL)

Two independent Peltier control channels, each with a DS18B20 temperature sensor and an H-bridge motor driver:

| Channel | Temp Sensor GPIO | PWM GPIO | Dir Pin 1 GPIO | Dir Pin 2 GPIO | PIO |
|---------|-----------------|----------|---------------|---------------|-----|
| **LOAD** | 27 | 8 | 10 | 12 | PIO0 |
| **LNA** | 26 | 9 | 11 | 13 | PIO1 |

JSON protocol keys use `LNA_` and `LOAD_` prefixes (e.g. `LNA_temp_target`, `LOAD_enable`).

### Potentiometer Wiring (APP_POTMON)

Two potentiometers for position feedback, read via the RP2350 ADC:

| Channel | GPIO | ADC Channel | JSON Key |
|---------|------|-------------|----------|
| **EL** (elevation) | 26 | 0 | `pot_el_voltage` |
| **AZ** (azimuth) | 27 | 1 | `pot_az_voltage` |

### IMU Wiring (APP_IMU_EL / APP_IMU_AZ)

Two BNO08x IMUs in UART RVC mode, one per axis. Each runs on a separate Pico with the appropriate DIP switch setting:

| DIP Code | Sensor Name | Axis |
|----------|-------------|------|
| 3 | `imu_el` | Elevation |
| 6 | `imu_az` | Azimuth |

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
- `flash-picos` - Multi-device flashing CLI (installed with `picohost`)
- `devices_info.json` - Device configuration database

## Contributing

See [CLAUDE.md](CLAUDE.md) for detailed development guidelines.

## License

MIT License - see LICENSE file for details.
