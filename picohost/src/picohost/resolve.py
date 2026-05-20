#!/usr/bin/env python3
"""`picohost-resolve`: map a Pico USB serial to its current /dev/ttyACMn.

The serial number is the stable identifier; ``/dev/ttyACMn`` shuffles on
replug/reboot. Use this helper in field debug sessions when you know
the Pico by its role (e.g. from ``devices_info.json``) but need the
current port to attach a serial client.
"""
import argparse
import sys

from .flash_picos import find_pico_ports


def resolve_port(usb_serial):
    """Return the ``/dev/ttyACMn`` currently bound to *usb_serial*.

    Returns ``None`` if no attached CDC-mode Pico has that serial.
    BOOTSEL-mode Picos are invisible here — they have no serial port.
    """
    for device, serial in find_pico_ports().items():
        if serial == usb_serial:
            return device
    return None


def main():
    p = argparse.ArgumentParser(
        description=(
            "Resolve a Pico USB serial number to its current serial "
            "port. With no argument, lists all attached CDC-mode Picos "
            "as 'serial<TAB>port' lines."
        ),
    )
    p.add_argument(
        "usb_serial",
        nargs="?",
        default=None,
        help="USB serial number to look up. If omitted, list all.",
    )
    args = p.parse_args()

    if args.usb_serial is None:
        for device, serial in sorted(find_pico_ports().items()):
            print(f"{serial}\t{device}")
        return

    port = resolve_port(args.usb_serial)
    if port is None:
        print(
            f"No Pico with USB serial {args.usb_serial!r} found.",
            file=sys.stderr,
        )
        sys.exit(1)
    print(port)


if __name__ == "__main__":
    main()
