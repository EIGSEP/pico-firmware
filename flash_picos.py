#!/usr/bin/env python3
import argparse
import subprocess
import time
import sys
import json
from serial import Serial
from serial.tools import list_ports

PICO_VID = 0x2E8A  # Raspberry Pi Foundation USB vendor ID
PICO_PID_BOOTSEL = 0x0003  # BOOTSEL-mode PID
PICO_PID_CDC = 0x0009  # CDC serial mode PID


def find_pico_ports():
    """
    Return a list of (device, serial, is_bootsel) tuples for all
    ttyACM*/ttyUSB* ports whose USB VID/PID matches the Pico in either
    BOOTSEL or CDC mode.
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
        description="Flash all attached Picos, read JSON from each, save to files"
    )
    p.add_argument(
        "--uf2",
        default="build/pico_multi.uf2",
        help="Path to your pico_multi.uf2"
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
        "--out-prefix",
        default="device_info",
        help="Prefix for output JSON files (default: device_info)",
    )
    args = p.parse_args()

    ports = find_pico_ports()
    if not ports:
        print(
            "No Raspberry Pi Pico serial ports found.",
            file=sys.stderr,
        )
        sys.exit(1)

    print(f"Found Picos on: {ports}")
    for port_dev, port_serial in ports.items():
        print("Flashing Pico on port:", port_dev)
        try:
            flash_uf2(args.uf2, port_serial)
        except RuntimeError as e:
            print(e, file=sys.stderr)
            continue

        # give the Pico a moment to reboot into user code
        time.sleep(1)

        try:
            data = read_json_from_serial(port_dev, args.baud, args.timeout)
        except RuntimeError as e:
            print(e, file=sys.stderr)
            continue

        uid = data.get("unique_id") or port_serial
        out_file = f"{args.out_prefix}_{uid}.json"
        with open(out_file, "w") as f:
            json.dump(data, f, indent=2)
        print(f"✔ Wrote {out_file}")


if __name__ == "__main__":
    main()
