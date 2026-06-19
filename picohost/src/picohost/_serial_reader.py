#!/usr/bin/env python3
"""Standalone serial-read helper, run as a *child process* by flash_picos.

Why a separate process, and why run by file path
------------------------------------------------
A board that enumerated but whose USB endpoint has wedged (the "-110
mute" state) can block the kernel's ``cdc_acm`` teardown: the final
``os.close()`` of the tty never returns (uninterruptible sleep). pyserial's
``timeout`` bounds reads, not open/close.

A *thread* cannot escape this. Bounding the wait only frees the calling
thread; the worker stays stuck in the kernel syscall, and a process cannot
complete ``exit_group(2)`` while any of its threads is in uninterruptible
sleep — so the whole process hangs at exit even though its main thread is
done. (That is exactly the residual hang that survived bounding the wait:
flash-picos finished its work but never exited, so ``eigsep-field patch``,
which waits on it unbounded, hung forever.)

Isolating the open/read/close in a child process is what fixes it: the
wedge pins *this* child, and ``flash_picos`` abandons it and exits cleanly.
The kernel fd-teardown that blocks then happens in the abandoned child
(reparented to init), not in flash-picos.

This file is launched by **file path** (``python .../_serial_reader.py``),
not imported as ``picohost._serial_reader`` — running it as a script does
not execute ``picohost/__init__`` and so does not pull in the package's
heavy imports (eigsep_redis, etc.) on every per-board readback. Keep it
dependency-light: stdlib plus pyserial only.

Protocol (one JSON object line written to stdout, then flushed):
  {"data": {...}}                  first valid JSON status line from the port
  {"timeout": true}                port opened but no JSON within the window
  {"err": "...", "errno": N|null}  open/read raised (errno preserved so the
                                   parent can classify EACCES/EBUSY/-110)
The line is delivered *before* the (possibly wedging) close, so the parent
gets the data even when the close never returns.
"""

import json
import sys
import time

from serial import Serial


def run(port, baud, timeout, out):
    """Open *port*, write one protocol line to *out*, then close.

    The write+flush always precede the close so a wedging close cannot
    swallow a line we already read. Returns ``0`` (process exit code).
    """
    try:
        ser = Serial(port, baudrate=baud, timeout=1)
    except Exception as e:  # open failed (EACCES/EBUSY/ENOENT/-110/...)
        out.write(
            json.dumps({"err": str(e), "errno": getattr(e, "errno", None)})
            + "\n"
        )
        out.flush()
        return 0
    try:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
            except Exception as e:  # read failed mid-stream
                out.write(
                    json.dumps(
                        {"err": str(e), "errno": getattr(e, "errno", None)}
                    )
                    + "\n"
                )
                out.flush()
                return 0
            if not line:
                continue
            try:
                parsed = json.loads(line)
            except json.JSONDecodeError:
                continue
            out.write(json.dumps({"data": parsed}) + "\n")
            out.flush()
            return 0
        out.write(json.dumps({"timeout": True}) + "\n")
        out.flush()
        return 0
    finally:
        # The line (if any) is already delivered. This is the call that can
        # wedge on a mute USB endpoint; if it does, the parent abandons us.
        try:
            ser.close()
        except Exception:
            pass


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    port, baud, timeout = argv[0], int(argv[1]), float(argv[2])
    return run(port, baud, timeout, sys.stdout)


if __name__ == "__main__":
    sys.exit(main())
