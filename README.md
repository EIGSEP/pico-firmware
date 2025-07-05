# EIGSEP Pico Firmware

[![codecov](https://codecov.io/gh/EIGSEP/pico-firmware/graph/badge.svg?token=WNGHYLBF0U)](https://codecov.io/gh/EIGSEP/pico-firmware)

This repository provides instructions and tools for building and flashing firmware onto your Raspberry Pi Pico devices. Follow the steps below to install the required tools, compile your firmware, and flash one or multiple Pico boards.

---

## Prerequisites

- **Operating System:** Ubuntu (or other Debian-based Linux)
- **Hardware:** Raspberry Pi Pico (or compatible RP2040 board)
- **Permissions:** You will need `sudo` access to install packages and configure udev rules.

---

## 1. Install picotool

Picotool is a command-line utility to interact with RP2040 devices over USB. It supports loading firmware, inspecting flash, and more.

```bash
# Clone the picotool repository
git clone https://github.com/raspberrypi/picotool.git

# Install build dependencies
sudo apt update
sudo apt install build-essential pkg-config libusb-1.0-0-dev cmake

# Build picotool
cd picotool
mkdir build && cd build
cmake ..
make

# Install udev rule for non-root access
sudo cp udev/99-picotool.rules /etc/udev/rules.d/

# Reload udev rules (or reboot)
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

## 2. Clone and Build pico-firmware

This project includes your firmware source code and a helper script to build the UF2 file.

```bash
# Clone your firmware repository
git clone git@github.com:EIGSEP/pico-firmware.git
cd pico-firmware

# Initialize the pico-sdk submodule
git submodule update --init lib/cJSON
git submodule update --init lib/BNO08x_Pico_Library
git submodule update --init pico-sdk
cd pico-sdk
git submodule update --init

# Build the firmware
./build.sh
```

- The `build.sh` script will invoke CMake and generate a `build/pico_multi.uf2` file ready for flashing.

---

## 3. Flash Pico Devices

Use the provided Python script `flash_picos.py` to automate flashing one or multiple Pico boards in BOOTSEL mode.

```bash
# Run the flashing script
pip3 install serial
python3 flash_picos.py --uf2 build/pico_multi.uf2
```

### Options
See `python flash_picos.py --help`.
