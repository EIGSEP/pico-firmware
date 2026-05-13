#!/usr/bin/env python3
"""`flash-test`: load the heartbeat test UF2 onto a Pico in BOOTSEL mode.

`flash-picos` discovers Picos by enumerating CDC serial ports, which
means a fresh / wiped Pico (USB PID 0x0003, mass-storage) is invisible
to it. This CLI is a thin wrapper around ``picotool load`` that targets
a BOOTSEL-mode Pico directly, so the test image can be installed,
USB-CDC comes up, and the normal ``flash-picos`` workflow becomes
available.
"""
import argparse
import subprocess
import sys
from pathlib import Path


def build_picotool_cmd(uf2_path, bus=None, address=None):
    """Return the argv for ``picotool load`` targeting BOOTSEL."""
    cmd = ["picotool", "load", "-f", "-x", str(uf2_path)]
    if bus is not None:
        cmd += ["--bus", str(bus)]
    if address is not None:
        cmd += ["--address", str(address)]
    return cmd


def flash_test_image(uf2_path, bus=None, address=None):
    """Flash *uf2_path* onto a BOOTSEL-mode Pico via picotool.

    Parameters
    ----------
    uf2_path : str or Path
        Path to the test UF2.
    bus, address : int, optional
        USB bus / device address. Forwarded to picotool to disambiguate
        when multiple BOOTSEL devices are connected. Discover values
        with ``picotool info -ab``.

    Raises
    ------
    FileNotFoundError
        UF2 not present.
    RuntimeError
        picotool exited non-zero.
    """
    uf2_path = Path(uf2_path)
    if not uf2_path.is_file():
        raise FileNotFoundError(f"UF2 file not found: {uf2_path}")

    cmd = build_picotool_cmd(uf2_path, bus=bus, address=address)
    print(f"Running: {' '.join(cmd)}")
    res = subprocess.run(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
    )
    print(res.stdout, end="")
    if res.returncode != 0:
        raise RuntimeError(
            f"picotool failed (exit {res.returncode}). "
            "If multiple BOOTSEL Picos are connected, pass "
            "--bus and --address from `picotool info -ab`."
        )


def main():
    p = argparse.ArgumentParser(
        description=(
            "Flash a small heartbeat-LED test image onto a Pico in "
            "BOOTSEL mode. After this, the Pico enumerates as USB-CDC "
            "and can be re-flashed with the production image using "
            "`flash-picos`."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument(
        "--uf2",
        default="build/pico_test_blink.uf2",
        help="Path to the test UF2 (build with ./build.sh).",
    )
    p.add_argument(
        "--bus",
        type=int,
        default=None,
        help=(
            "USB bus number of the target Pico. Use only when more "
            "than one BOOTSEL device is connected."
        ),
    )
    p.add_argument(
        "--address",
        type=int,
        default=None,
        help="USB device address of the target Pico (companion to --bus).",
    )
    args = p.parse_args()

    try:
        flash_test_image(args.uf2, bus=args.bus, address=args.address)
    except FileNotFoundError as e:
        print(
            f"{e}\nBuild the test image first with ./build.sh.",
            file=sys.stderr,
        )
        sys.exit(1)
    except RuntimeError as e:
        print(str(e), file=sys.stderr)
        sys.exit(1)

    print(
        "\nFlash complete. The Pico should now blink in a heartbeat "
        "pattern and enumerate as /dev/ttyACM* (USB VID:PID "
        "2e8a:0009).\nRun `flash-picos --uf2 build/pico_multi.uf2` to "
        "install the production image."
    )


if __name__ == "__main__":
    main()
