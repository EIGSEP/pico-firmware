#!/usr/bin/env python3
import argparse
import json
import logging
import subprocess
import sys
import time
from pathlib import Path

from eigsep_redis import Transport
from serial import Serial
from serial.tools import list_ports

from .buses import PicoConfigStore
from .keys import PICO_CONFIG_KEY

logger = logging.getLogger(__name__)

PICO_VID = 0x2E8A  # Raspberry Pi Foundation USB vendor ID
PICO_PID_BOOTSEL = 0x0003  # BOOTSEL-mode PID
PICO_PID_CDC = 0x0009  # CDC serial mode PID


def find_pico_ports():
    """
    Return a dict of ``device: serial`` pairs for all ttyACM*/ttyUSB*
    ports whose USB VID/PID matches a Pico running CDC firmware
    (VID 0x2E8A, PID 0x0009).

    BOOTSEL-mode Picos (PID 0x0003) are mass-storage devices and do
    not appear in ``list_ports.comports()`` at all — use ``flash-test``
    to install a CDC-capable image first.
    """
    ports = {}
    for info in list_ports.comports():
        if info.vid == PICO_VID and info.pid == PICO_PID_CDC:
            ports[info.device] = info.serial_number
    return ports


def flash_uf2(uf2_path, serial):
    """
    Flash the UF2 onto the Pico whose USB serial number is `serial`,
    using picotool's --ser selector.
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


def flash_and_discover(
    uf2_path="build/pico_multi.uf2", port=None, baud=115200, timeout=10
):
    """
    Flash all attached Picos and read device config from each.

    Parameters
    ----------
    uf2_path : str or Path
        Path to the UF2 firmware file.
    port : str, optional
        Limit to a single serial port.  ``None`` means all discovered
        Picos.
    baud : int
        Serial baud rate for reading JSON after flash.
    timeout : int
        Seconds to wait for each Pico's JSON response.

    Returns
    -------
    list[dict]
        List of device info dicts, each with keys like ``app_id``,
        ``port``, ``usb_serial``.

    Raises
    ------
    FileNotFoundError
        If the UF2 file does not exist.
    """
    uf2_path = Path(uf2_path)
    if not uf2_path.is_file():
        raise FileNotFoundError(f"UF2 file not found: {uf2_path}")

    ports = find_pico_ports()
    if port:
        ports = {k: v for k, v in ports.items() if k == port}

    if not ports:
        logger.info("No Raspberry Pi Pico serial ports found")
        return []

    logger.info(f"Found Picos on: {ports}")
    all_devices = []

    for port_dev, port_serial in ports.items():
        logger.info(f"Flashing Pico on port: {port_dev}")
        try:
            flash_uf2(uf2_path, port_serial)
        except RuntimeError as e:
            logger.error(f"Flash failed on {port_dev}: {e}")
            continue

        time.sleep(2)  # wait for reboot

        try:
            data = read_json_from_serial(port_dev, baud, timeout)
        except RuntimeError as e:
            logger.error(f"Serial read failed on {port_dev}: {e}")
            continue

        data["port"] = port_dev
        data["usb_serial"] = port_serial
        all_devices.append(data)
        logger.info(f"Read device info from {port_dev}")

    return all_devices


def _publish_to_redis(devices, host, port):
    """Publish *devices* to Redis via :class:`PicoConfigStore`.

    Returns the :class:`PicoConfigStore` on success. Raises on
    connection failure so ``main`` can fall back to file output if the
    user asked for that.
    """
    transport = Transport(host=host, port=port)
    store = PicoConfigStore(transport)
    store.upload(devices)
    return store


def main():
    p = argparse.ArgumentParser(
        description=(
            "Flash all attached Picos, read JSON from each, and publish "
            "the device list to Redis (source of truth for PicoManager)."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
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
        help="Serial baud rate.",
    )
    p.add_argument(
        "--timeout",
        type=int,
        default=10,
        help="Seconds to wait for each Pico's JSON",
    )
    p.add_argument(
        "--redis-host",
        default="localhost",
        help="Redis host for PicoConfigStore publication",
    )
    p.add_argument(
        "--redis-port",
        type=int,
        default=6379,
        help="Redis port for PicoConfigStore publication",
    )
    p.add_argument(
        "--no-redis",
        action="store_true",
        help=(
            "Skip Redis publication. Use with --output-file for offline "
            "provisioning on a host without Redis."
        ),
    )
    p.add_argument(
        "--output-file",
        default=None,
        help=(
            "Optional: also write the device list to this JSON file. "
            "Not required — PicoManager reads from Redis directly."
        ),
    )
    args = p.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )

    try:
        all_devices = flash_and_discover(
            uf2_path=args.uf2,
            port=args.port,
            baud=args.baud,
            timeout=args.timeout,
        )
    except FileNotFoundError as e:
        print(str(e), file=sys.stderr)
        sys.exit(1)

    if not all_devices:
        print("No devices discovered.", file=sys.stderr)
        sys.exit(1)

    if not args.no_redis:
        try:
            _publish_to_redis(all_devices, args.redis_host, args.redis_port)
            print(
                f"Published {len(all_devices)} device(s) to Redis at "
                f"{args.redis_host}:{args.redis_port} "
                f"(key: {PICO_CONFIG_KEY})."
            )
        except Exception as e:
            print(
                f"Redis publication failed: {e}\n"
                "Re-run with --no-redis (and optionally --output-file) if "
                "Redis is not available.",
                file=sys.stderr,
            )

    if args.output_file:
        with open(args.output_file, "w") as f:
            json.dump(all_devices, f, indent=2)
        print(f"Wrote {len(all_devices)} device(s) to {args.output_file}.")


if __name__ == "__main__":
    main()
