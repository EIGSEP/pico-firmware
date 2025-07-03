#!/usr/bin/env python3
import argparse
import subprocess
import time
import sys
import json
from serial import Serial
from serial.tools import list_ports

PICO_VID = 0x2E8A  # Raspberry Pi Foundation USB vendor ID


def find_pico_ports():
    """
    Return a list of device paths for all serial ports whose USB VID
    matches Raspberry Pi.
    """
    pico_ports = []
    for info in list_ports.comports():
        if info.vid == PICO_VID:
            pico_ports.append(info.device)
    return pico_ports


def flash_uf2(uf2_path, port):
    """Flash the UF2 onto a single Pico at `port` using picotool."""
    cmd = ["picotool", "load", "--port", port, "-x", uf2_path, "-f"]
    print(f"Flashing {uf2_path} → {port}")
    res = subprocess.run(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
    )
    if res.returncode != 0:
        print(res.stdout, file=sys.stderr)
        raise RuntimeError(f"picotool failed on {port}")
    print(res.stdout, end="")


def read_json_from_serial(port, baud, timeout):
    """Open the serial port, read until a valid JSON line appears or timeout."""
    with Serial(port, baudrate=baud, timeout=1) as ser:
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
                print(f"[{port}] Received JSON: {obj}")
                return obj
            except json.JSONDecodeError:
                continue
    raise RuntimeError(f"[{port}] Timed out waiting for JSON")


def main():
    p = argparse.ArgumentParser(
        description="Flash all attached Picos, read JSON from each, save to files"
    )
    p.add_argument("--uf2", required=True, help="Path to your pico_multi.uf2")
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
        print("❌ No Raspberry Pi Pico serial ports found.", file=sys.stderr)
        sys.exit(1)

    print(f"Found Picos on: {', '.join(ports)}")
    for port in ports:
        try:
            flash_uf2(args.uf2, port)
        except RuntimeError as e:
            print(e, file=sys.stderr)
            continue
        # give the Pico a moment to reboot into user code
        time.sleep(1)

        try:
            data = read_json_from_serial(port, args.baud, args.timeout)
        except RuntimeError as e:
            print(e, file=sys.stderr)
            continue

        # name file by unique_id if present, else by port name
        uid = data.get("unique_id") or port.replace("/dev/", "")
        out_file = f"{args.out_prefix}_{uid}.json"
        with open(out_file, "w") as f:
            json.dump(data, f, indent=2)
        print(f"✔ Wrote {out_file}")


if __name__ == "__main__":
    main()
