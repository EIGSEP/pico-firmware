#!/usr/bin/env python3
"""`flash-test`: load the heartbeat test UF2 onto Pico(s) in BOOTSEL mode.

`flash-picos` discovers Picos by enumerating CDC serial ports, which
means a fresh / wiped Pico (USB PID 0x0003, mass-storage) is invisible
to it. This CLI wraps ``picotool load`` to target BOOTSEL-mode Picos so
the test image can be installed, USB-CDC comes up, and the normal
``flash-picos`` workflow becomes available.

By default all connected BOOTSEL Picos are flashed sequentially (mirroring
``flash-picos``). Pass ``--bus/--address`` or ``--usb-serial`` to target
a single device.
"""
import argparse
import subprocess
import sys
from pathlib import Path

PICO_VID = "2e8a"
PICO_PID_BOOTSEL = "0003"
SYSFS_USB_DEVICES = Path("/sys/bus/usb/devices")


def find_bootsel_devices(sysfs_root=None):
    """Enumerate Picos currently in BOOTSEL mode by scanning Linux sysfs.

    Returns a list of ``{"usb_serial", "bus", "address"}`` dicts, one per
    attached Pico with VID:PID 2e8a:0003. BOOTSEL Picos are USB
    mass-storage devices and don't appear in pyserial's port list, so
    sysfs is the cheapest discovery path that avoids parsing picotool's
    free-form output or pulling in libusb bindings.

    Returns an empty list on systems without ``/sys/bus/usb/devices``
    (e.g. macOS); the caller should fall back to ``picotool``'s default
    selection.
    """
    if sysfs_root is None:
        sysfs_root = SYSFS_USB_DEVICES
    sysfs_root = Path(sysfs_root)
    devices = []
    if not sysfs_root.is_dir():
        return devices
    for entry in sorted(sysfs_root.iterdir()):
        try:
            vid = (entry / "idVendor").read_text().strip().lower()
            pid = (entry / "idProduct").read_text().strip().lower()
        except (OSError, ValueError):
            continue
        if vid != PICO_VID or pid != PICO_PID_BOOTSEL:
            continue
        try:
            serial = (entry / "serial").read_text().strip() or None
        except OSError:
            serial = None
        try:
            bus = int((entry / "busnum").read_text().strip())
            address = int((entry / "devnum").read_text().strip())
        except (OSError, ValueError):
            bus = None
            address = None
        devices.append(
            {"usb_serial": serial, "bus": bus, "address": address}
        )
    return devices


def build_picotool_cmd(uf2_path, bus=None, address=None, usb_serial=None):
    """Return the argv for ``picotool load`` targeting BOOTSEL."""
    cmd = ["picotool", "load", "-f"]
    if usb_serial is not None:
        cmd += ["--ser", str(usb_serial)]
    if bus is not None:
        cmd += ["--bus", str(bus)]
    if address is not None:
        cmd += ["--address", str(address)]
    cmd += ["-x", str(uf2_path)]
    return cmd


def flash_test_image(uf2_path, bus=None, address=None, usb_serial=None):
    """Flash *uf2_path* onto a BOOTSEL-mode Pico via picotool.

    Parameters
    ----------
    uf2_path : str or Path
        Path to the test UF2.
    bus, address : int, optional
        USB bus / device address. Forwarded to picotool to disambiguate
        when multiple BOOTSEL devices are connected. Discover values
        with ``picotool info -ab``.
    usb_serial : str, optional
        Pico board unique ID (flash serial). Stable across reboots and
        BOOTSEL toggles; preferred over bus/address when known.

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

    cmd = build_picotool_cmd(
        uf2_path, bus=bus, address=address, usb_serial=usb_serial
    )
    print(f"Running: {' '.join(cmd)}")
    res = subprocess.run(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
    )
    output = res.stdout or ""
    if res.returncode != 0:
        if output:
            print(output, end="", file=sys.stderr)
        raise RuntimeError(
            f"picotool failed (exit {res.returncode}). "
            "If multiple BOOTSEL Picos are connected, pass --usb-serial "
            "(from `picotool info -a`) or --bus/--address "
            "(from `picotool info -ab`)."
            + (f"\npicotool output:\n{output}" if output else "")
        )
    if output:
        print(output, end="")


_DONE_MSG = (
    "\nFlash complete. Picos should now blink in a heartbeat pattern "
    "and enumerate as /dev/ttyACM* (USB VID:PID 2e8a:0009).\nRun "
    "`flash-picos --uf2 build/pico_multi.uf2` to install the "
    "production image."
)


def main():
    p = argparse.ArgumentParser(
        description=(
            "Flash a small heartbeat-LED test image onto every Pico in "
            "BOOTSEL mode. After this, each Pico enumerates as USB-CDC "
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
            "USB bus number of a single target Pico. Skips auto-discovery; "
            "use only to override which BOOTSEL device gets flashed."
        ),
    )
    p.add_argument(
        "--address",
        type=int,
        default=None,
        help="USB device address of the target Pico (companion to --bus).",
    )
    p.add_argument(
        "--usb-serial",
        default=None,
        help=(
            "Pico board unique ID. Skips auto-discovery and flashes only "
            "this device. Discover with `picotool info -a`."
        ),
    )
    args = p.parse_args()
    if (args.bus is None) != (args.address is None):
        p.error("--bus and --address must be provided together.")
    if args.usb_serial is not None and args.bus is not None:
        p.error("--usb-serial cannot be combined with --bus/--address.")

    targeted = (
        args.bus is not None
        or args.address is not None
        or args.usb_serial is not None
    )

    try:
        if targeted or not SYSFS_USB_DEVICES.is_dir():
            flash_test_image(
                args.uf2,
                bus=args.bus,
                address=args.address,
                usb_serial=args.usb_serial,
            )
        else:
            devices = find_bootsel_devices()
            if not devices:
                print(
                    "No Picos in BOOTSEL mode were found. Hold BOOTSEL "
                    "while plugging the Pico in (or while pressing the "
                    "RUN button) and re-run.",
                    file=sys.stderr,
                )
                sys.exit(1)
            print(f"Found {len(devices)} Pico(s) in BOOTSEL mode.")
            failures = 0
            for dev in devices:
                serial = dev.get("usb_serial")
                bus = dev.get("bus")
                address = dev.get("address")
                has_bus_address = bus is not None and address is not None
                if not serial and not has_bus_address:
                    print(
                        "Skipping BOOTSEL device with incomplete selector "
                        f"(serial={serial}, bus={bus}, address={address}).",
                        file=sys.stderr,
                    )
                    failures += 1
                    continue
                label = serial or f"bus {bus} address {address}"
                print(f"\n→ Flashing Pico {label}")
                try:
                    if serial:
                        flash_test_image(args.uf2, usb_serial=serial)
                    else:
                        flash_test_image(
                            args.uf2, bus=bus, address=address
                        )
                except RuntimeError as e:
                    print(str(e), file=sys.stderr)
                    failures += 1
            if failures:
                sys.exit(1)
    except FileNotFoundError as e:
        print(
            f"{e}\nBuild the test image first with ./build.sh.",
            file=sys.stderr,
        )
        sys.exit(1)
    except RuntimeError as e:
        print(str(e), file=sys.stderr)
        sys.exit(1)

    print(_DONE_MSG)


if __name__ == "__main__":
    main()
