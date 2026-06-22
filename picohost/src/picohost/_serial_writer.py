#!/usr/bin/env python3
"""Standalone serial-write helper, run as a *child process* by flash_picos.

Sends a single newline-terminated line to a Pico's CDC port — the
``{"cmd":"bootsel"}`` reflash trigger — and exits. Run as a child process,
by file path, for the same reason as :mod:`_serial_reader`: opening or
closing a marginal (or about-to-reboot) CDC port can wedge in the kernel's
``cdc_acm`` teardown, and a wedge must pin an *abandonable* child, never
flash-picos itself.

The write is fire-and-forget: the caller confirms the reboot by watching
sysfs for the board re-appearing in BOOTSEL (``2e8a:000f``), not by this
child's exit code. So the child only has to deliver the bytes without
hanging the parent.

Kept dependency-light (stdlib + pyserial) and launched by path so running
it does not import the ``picohost`` package (and its heavy deps).
"""

import sys

from serial import Serial


def run(port, baud, line):
    """Open *port*, write *line* (a ``\\n`` is appended), flush, close.

    Returns ``0`` on a delivered write, ``1`` if the port could not be
    opened or the write failed. The newline terminator is added here so
    callers pass the bare command.
    """
    try:
        ser = Serial(port, baudrate=baud, timeout=1)
    except Exception:  # open failed (EACCES/EBUSY/ENOENT/-110/...)
        return 1
    try:
        ser.write((line + "\n").encode("utf-8"))
        ser.flush()
        return 0
    except Exception:
        return 1
    finally:
        try:
            ser.close()
        except Exception:
            pass


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    port, baud, line = argv[0], int(argv[1]), argv[2]
    return run(port, baud, line)


if __name__ == "__main__":
    sys.exit(main())
