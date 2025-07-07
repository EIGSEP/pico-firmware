#!/usr/bin/env python3
import argparse
import subprocess
import time
import sys
import json
from pathlib import Path
from serial import Serial
from serial.tools import list_ports

PICO_VID = 0x2E8A  # Raspberry Pi Foundation USB vendor ID
PICO_PID_BOOTSEL = 0x0003  # BOOTSEL-mode PID
PICO_PID_CDC = 0x0009  # CDC serial mode PID


def find_pico_ports():
    """
    Return a dict of device: serial pairs for all ttyACM*/ttyUSB* ports
    whose USB VID/PID matches the Pico in CDC mode.
    """
    ports = {}
    for info in list_ports.comports():
        if info.vid == PICO_VID:
            if info.pid in (PICO_PID_BOOTSEL, PICO_PID_CDC):
                ports[info.device] = info.serial_number
    return ports


def flash_uf2(uf2_path, serial):
    """
    Flash the UF2 onto the Pico whose USB serial number is `serial`,
    using picotool’s --ser selector.
    """
    cmd = f"picotool load -f --ser {serial} -x {uf2_path}".split()
    print(f"Flashing {uf2_path} → serial={serial}")
    res = subprocess.run(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
    )
    if res.returncode != 0:
        print(res.stdout, file=sys.stderr)
        raise RuntimeError(f"picotool failed on serial={serial}")
    print(res.stdout, end="")


def read_json_from_serial(port, baud, timeout):
    """
    Open the serial port, read until a valid JSON line appears or timeout.
    """
    with Serial(port, baudrate=baud, timeout=1) as ser:
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            try:
                return json.loads(line)
            except json.JSONDecodeError:
                continue
    raise RuntimeError(f"[{port}] Timed out waiting for JSON")


def main():
    p = argparse.ArgumentParser(
        description=(
            "Flash all attached Picos, read JSON from each, save to single "
            "file."
        )
    )
    p.add_argument(
        "--port", default=None, help="Serial port of pico, None means all"
    )
    p.add_argument(
        "--uf2",
        default="build/pico_multi.uf2",
        help="Path to your pico_multi.uf2",
    )
    p.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Serial baud rate (default: 115200)",
    )
    p.add_argument(
        "--timeout",
        type=int,
        default=10,
        help="Seconds to wait for each Pico's JSON (default: 10)",
    )
    p.add_argument(
        "--output",
        default="devices_info.json",
        help="Output JSON file (default: devices_info.json)",
    )
    args = p.parse_args()

    # Check if the UF2 file exists
    uf2_path = Path(args.uf2)
    if not uf2_path.is_file():
        print(f"UF2 file not found: {uf2_path}", file=sys.stderr)
        sys.exit(1)

    ports = find_pico_ports()
    if args.port:
        ports = {k: v for k, v in ports.items() if k == args.port}
    if not ports:
        print(
            "No Raspberry Pi Pico serial ports found.",
            file=sys.stderr,
        )
        sys.exit(1)

    print(f"Found Picos on: {ports}")
    all_devices = []

    for port_dev, port_serial in ports.items():
        print("Flashing Pico on port:", port_dev)
        try:
            flash_uf2(uf2_path, port_serial)
        except RuntimeError as e:
            print(e, file=sys.stderr)
            continue

        # give the Pico a moment to reboot into user code
        time.sleep(2)

        try:
            data = read_json_from_serial(port_dev, args.baud, args.timeout)
        except RuntimeError as e:
            print(e, file=sys.stderr)
            continue

        # Add port and serial info to the device data
        data["port"] = port_dev
        data["usb_serial"] = port_serial
        all_devices.append(data)
        print(f"Read device info from {port_dev}")

    # Write all device info to a single file
    with open(args.output, "w") as f:
        json.dump(all_devices, f, indent=2)
    print(
        f"Wrote all device information to {args.output} ({len(all_devices)} "
        "devices)."
    )


if __name__ == "__main__":
    main()
