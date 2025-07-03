# Pico Setup

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
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="000a", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/99-picotool.rules

# Reload udev rules (or reboot)
sudo udevadm control --reload-rules && sudo udevadm trigger
```

> **Note:** Adjust `idVendor` and `idProduct` if you are using a different board.

---

## 2. Clone and Build pico-firmware

This project includes your firmware source code and a helper script to build the UF2 file.

```bash
# Clone your firmware repository
git clone git@github.com:EIGSEP/pico-firmware.git
cd pico-firmware

# Initialize the pico-sdk submodule
git submodule update --init pico-sdk

# Build the firmware
./build.sh
```

- The `build.sh` script will invoke CMake and generate a `build/firmware.uf2` file ready for flashing.

---

## 3. Flash Pico Devices

Use the provided Python script `flash_picos.py` to automate flashing one or multiple Pico boards in BOOTSEL mode.

```bash
# Run the flashing script
python3 flash_picos.py --firmware build/firmware.uf2
```

### Options

- `--firmware <path>`: Path to the UF2 file (default: `build/firmware.uf2`)
- `--port <device>`: Specify a serial port (e.g., `/dev/ttyACM0`) to flash a single board
- `--all`: Flash all connected Pico devices automatically

> **Tip:** If you encounter permission errors, ensure your user is in the `plugdev` group or rerun the udev rules step.

---

## 4. Troubleshooting

- ``** not found:** Make sure the `picotool` binary is in your `PATH` or invoke it via `./build/picotool`.
- **Udev rule not applied:** Check `/etc/udev/rules.d/99-picotool.rules` and reload rules with `sudo udevadm`.
- **Board not detected:** Hold the BOOTSEL button while plugging in the Pico or use the `--all` flag to scan automatically.

---

## License & Author

- **Author:** Christian Bye
- **License:** MIT

---

For any questions or issues, please open an issue on the repository. Happy hacking!

