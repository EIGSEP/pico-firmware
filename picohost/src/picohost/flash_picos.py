#!/usr/bin/env python3
"""
flash_picos.py – Multithreaded flasher / interrogator for multiple Raspberry Pi Picos

This script flashes the same UF2 to every Pico connected to the host, waits for
it to reboot, then captures a line of JSON that each board prints on its USB
CDC serial port.  All results are written to a single JSON file.

Originally this ran the boards **sequentially**; it is now fully **multithreaded**
so that flashing and interrogation happen in parallel, cutting total run-time
when you have several devices on the bench.
"""

import argparse
import subprocess
import time
import sys
import json
from pathlib import Path
from serial import Serial
from serial.tools import list_ports
from concurrent.futures import ThreadPoolExecutor, as_completed

# -----------------------------------------------------------------------------
# Constants – USB VID/PID pairs used by the Raspberry Pi Pico
# -----------------------------------------------------------------------------
PICO_VID = 0x2E8A              # Raspberry Pi Foundation USB vendor ID
PICO_PID_BOOTSEL = 0x0003      # BOOTSEL-mode PID
PICO_PID_CDC = 0x0009          # CDC serial mode PID

# -----------------------------------------------------------------------------
# Helper functions
# -----------------------------------------------------------------------------

def find_pico_ports():
    """
    Return a dict of device: serial pairs for all ttyACM*/ttyUSB* ports
    whose USB VID/PID matches the Pico in CDC mode.
    """
    ports = {}
    for info in list_ports.comports():
        if info.vid == PICO_VID and info.pid in (PICO_PID_BOOTSEL, PICO_PID_CDC):
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
    print(res.stdout, end="", flush=True)


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

# -----------------------------------------------------------------------------
# Per-device worker
# -----------------------------------------------------------------------------

def process_pico(port_dev: str, port_serial: str, uf2_path: Path, baud: int, timeout: int):
    """Flash one Pico and return its JSON dict (or None on failure)."""

    try:
        flash_uf2(uf2_path, port_serial)
        # Give the Pico a moment to reboot into user firmware before opening CDC
        time.sleep(2)
        data = read_json_from_serial(port_dev, baud, timeout)
        data["port"] = port_dev
        data["usb_serial"] = port_serial
        print(f"[SER={port_serial}] OK – captured JSON", flush=True)
        return data
    except RuntimeError as exc:
        print(f"[SER={port_serial}] ERROR – {exc}", file=sys.stderr, flush=True)
        return None

# -----------------------------------------------------------------------------
# CLI entry point
# -----------------------------------------------------------------------------

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
    parser.add_argument(
        "--workers", type=int, default=None, help="Max parallel flashes (default = all)"
    )
    args = parser.parse_args()

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

    # ---------------------------------------------------------------------
    # Multithreaded processing
    # ---------------------------------------------------------------------
    max_workers = args.workers or len(ports)
    print(f"Running with up to {max_workers} worker threads…\n")

    all_devices = []
    with ThreadPoolExecutor(max_workers=max_workers) as pool:
        futures = {
            pool.submit(process_pico, dev, ser, uf2_path, args.baud, args.timeout): dev
            for dev, ser in ports.items()
        }
        for future in as_completed(futures):
            result = future.result()
            if result is not None:
                all_devices.append(result)

    # Write all device info to a single file
    with open(args.output, "w") as f:
        json.dump(all_devices, f, indent=2)
    print(
        f"\nWrote information for {len(all_devices)} device(s) to {args.output}."
    )


if __name__ == "__main__":
    main()
