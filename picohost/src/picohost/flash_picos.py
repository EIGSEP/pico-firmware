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
PICO_PID_BOOTSEL = 0x000F  # RP2350 BOOTSEL-mode PID (RP2040 was 0x0003)
PICO_PID_CDC = 0x0009  # CDC serial mode PID

SYSFS_USB_DEVICES = Path("/sys/bus/usb/devices")


def find_pico_ports():
    """
    Return a dict of ``device: serial`` pairs for all ttyACM*/ttyUSB*
    ports whose USB VID/PID matches a Pico running CDC firmware
    (VID 0x2E8A, PID 0x0009).

    BOOTSEL-mode Picos (PID 0x000F on RP2350) are mass-storage devices
    and do not appear in ``list_ports.comports()`` at all — use
    ``flash-test`` to install a CDC-capable image first.
    """
    ports = {}
    for info in list_ports.comports():
        if info.vid == PICO_VID and info.pid == PICO_PID_CDC:
            ports[info.device] = info.serial_number
    return ports


def _resolve_bus_address(usb_serial, sysfs_root=None):
    """Return ``(bus, address, in_bootsel)`` for the Pico *usb_serial*.

    Reads USB topology from Linux sysfs and matches the device — in
    either CDC (``2e8a:0009``) or BOOTSEL (``2e8a:000f``) mode — whose
    serial equals *usb_serial*. ``in_bootsel`` is ``True`` when the match
    is a BOOTSEL device.

    picotool's ``--ser`` selector is unreliable on a busy hub: matching
    by serial forces a USB serial-string descriptor read that
    intermittently fails under bus contention, so ``picotool load``
    cannot find the device to flash. Selecting by ``--bus``/``--address``
    needs no descriptor read and is reliable. Each flash re-enumerates
    the Pico to a new USB address, so callers must re-resolve before
    every attempt.

    Matching BOOTSEL devices too lets a retry finish a load on a device a
    prior attempt left in BOOTSEL (no reboot needed) rather than
    stranding it: a BOOTSEL device is invisible to :func:`find_pico_ports`
    and would otherwise need a separate ``flash-test`` pass to recover.

    Returns ``(None, None, None)`` if no matching device is present
    (e.g. the target is momentarily mid-reboot, or sysfs is absent).
    """
    if sysfs_root is None:
        sysfs_root = SYSFS_USB_DEVICES
    sysfs_root = Path(sysfs_root)
    if not sysfs_root.is_dir():
        return (None, None, None)
    cdc_pid = f"{PICO_PID_CDC:04x}"
    bootsel_pid = f"{PICO_PID_BOOTSEL:04x}"
    for entry in sorted(sysfs_root.iterdir()):
        try:
            vid = (entry / "idVendor").read_text().strip().lower()
            pid = (entry / "idProduct").read_text().strip().lower()
        except (OSError, ValueError):
            continue
        if vid != "2e8a" or pid not in (cdc_pid, bootsel_pid):
            continue
        try:
            serial = (entry / "serial").read_text().strip()
        except OSError:
            continue
        if serial != usb_serial:
            continue
        try:
            bus = int((entry / "busnum").read_text().strip())
            address = int((entry / "devnum").read_text().strip())
        except (OSError, ValueError):
            return (None, None, None)
        return (bus, address, pid == bootsel_pid)
    return (None, None, None)


def _find_bootsel_devices(sysfs_root=None):
    """List all Picos currently in BOOTSEL mode from Linux sysfs.

    Returns a list of ``{"usb_serial", "bus", "address"}`` dicts (one
    per ``2e8a:000f`` device), sorted by ``(bus, address)`` for a
    deterministic flashing order. ``usb_serial`` is ``None`` for a
    wiped board that enumerates without a serial descriptor — such a
    device is still flashable by bus/address. Devices whose
    bus/address cannot be read are skipped; the result is empty on
    hosts without sysfs.
    """
    if sysfs_root is None:
        sysfs_root = SYSFS_USB_DEVICES
    sysfs_root = Path(sysfs_root)
    if not sysfs_root.is_dir():
        return []
    bootsel_pid = f"{PICO_PID_BOOTSEL:04x}"
    devices = []
    for entry in sorted(sysfs_root.iterdir()):
        try:
            vid = (entry / "idVendor").read_text().strip().lower()
            pid = (entry / "idProduct").read_text().strip().lower()
        except (OSError, ValueError):
            continue
        if vid != "2e8a" or pid != bootsel_pid:
            continue
        try:
            serial = (entry / "serial").read_text().strip()
        except OSError:
            serial = None
        try:
            bus = int((entry / "busnum").read_text().strip())
            address = int((entry / "devnum").read_text().strip())
        except (OSError, ValueError):
            continue
        devices.append(
            {"usb_serial": serial, "bus": bus, "address": address}
        )
    devices.sort(key=lambda d: (d["bus"], d["address"]))
    return devices


_BOOTSEL_REENUM_TIMEOUT_S = 10.0
_BOOTSEL_REENUM_POLL_S = 0.2


def _wait_for_bootsel(
    usb_serial,
    timeout=_BOOTSEL_REENUM_TIMEOUT_S,
    poll=_BOOTSEL_REENUM_POLL_S,
):
    """Poll sysfs until *usb_serial* re-enumerates in BOOTSEL.

    Returns the BOOTSEL ``(bus, address)`` once the device appears as
    ``2e8a:000f``, or ``(None, None)`` if it does not within *timeout*
    (e.g. the reboot request was lost on the congested hub). The kernel's
    sysfs serial is reliable; only picotool's *live* descriptor reads
    corrupt under contention, which is why we re-find via sysfs rather
    than letting ``picotool load -f`` track the device itself.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        bus, address, in_bootsel = _resolve_bus_address(usb_serial)
        if in_bootsel:
            return (bus, address)
        time.sleep(poll)
    return (None, None)


_BOOTSEL_SET_TIMEOUT_S = 15.0
_BOOTSEL_SET_STABLE_S = 2.0
_BOOTSEL_SET_POLL_S = 0.3


def _wait_for_stable_bootsel_set(
    timeout=_BOOTSEL_SET_TIMEOUT_S,
    stable=_BOOTSEL_SET_STABLE_S,
    poll=_BOOTSEL_SET_POLL_S,
):
    """Poll sysfs until the set of BOOTSEL Picos stops changing.

    After a mass BOOTSEL entry the devices enumerate one by one, so a
    single scan would race the slower ones. The set counts as settled
    when it is non-empty and unchanged for *stable* seconds; devices
    are compared by ``(usb_serial, bus, address)``.

    Returns the settled device list, or the last observation if the
    set never settles within *timeout* (possibly empty) — the caller
    decides whether a partial or empty set is an error.
    """
    deadline = time.monotonic() + timeout
    last = []
    last_keys = None
    stable_since = None
    while time.monotonic() < deadline:
        devices = _find_bootsel_devices()
        keys = {
            (d["usb_serial"], d["bus"], d["address"]) for d in devices
        }
        now = time.monotonic()
        if keys != last_keys:
            last_keys = keys
            last = devices
            stable_since = now
        elif keys and now - stable_since >= stable:
            return devices
        time.sleep(poll)
    return last


_FLASH_MAX_ATTEMPTS = 3
_FLASH_RETRY_BACKOFF_S = 2.0


def _picotool_load(bus, address, uf2_path, execute=True):
    """Run ``picotool load`` against the device at *bus*/*address*.

    *execute* appends ``-x`` (load then run). The GPIO mass-flash path
    passes ``execute=False``: the device stays in BOOTSEL with no
    re-enumeration (so bus/address remain valid for a retry) and a
    single mass reset boots every Pico afterwards. ``-f`` is never
    used — its reboot path re-acquires the device by a live USB
    serial-descriptor read that corrupts under hub contention.

    Returns the :class:`subprocess.CompletedProcess`.
    """
    cmd = ["picotool", "load", "--bus", str(bus), "--address", str(address)]
    if execute:
        cmd.append("-x")
    cmd.append(str(uf2_path))
    return subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        errors="replace",
    )


def flash_uf2(
    uf2_path,
    usb_serial,
    attempts=_FLASH_MAX_ATTEMPTS,
    backoff=_FLASH_RETRY_BACKOFF_S,
):
    """Flash the UF2 onto the Pico with USB serial *usb_serial*.

    Targets the device by USB ``--bus``/``--address`` (resolved from
    sysfs) rather than picotool's ``--ser``. A CDC device is flashed in
    two steps:

    1. ``picotool reboot -u -f --bus B --address A`` resets it into
       BOOTSEL (RP2350 enumerates as ``2e8a:000f``).
    2. After it re-enumerates (a *new* address), ``picotool load
       --bus B' --address A' -x`` loads and runs the image.

    This deliberately avoids ``picotool load -f``: its ``-f`` reboot path
    re-acquires the device by a *live* USB serial-string descriptor read,
    which intermittently returns garbage under bus contention on the
    observatory's deep hub (``Tracking device serial number ... for
    reboot`` → ``no accessible RP-series devices in BOOTSEL``) — the
    cause of the "random which Pico flashes" failures. ``reboot`` sends
    the reset and exits (no tracking read); the device is then re-found
    in BOOTSEL via sysfs, whose kernel-read serial is reliable. A device
    already in BOOTSEL is loaded directly (no reboot). Each step retries
    with linear backoff.
    """
    print(f"Flashing {uf2_path} → serial={usb_serial}")
    detail = ""
    for attempt in range(1, attempts + 1):
        bus, address, in_bootsel = _resolve_bus_address(usb_serial)
        if bus is None:
            detail = (
                f"serial={usb_serial} not visible as a "
                f"Pico (2e8a) USB device"
            )
            logger.warning("%s (attempt %d/%d)", detail, attempt, attempts)
        else:
            if not in_bootsel:
                rb = subprocess.run(
                    [
                        "picotool", "reboot", "-u", "-f",
                        "--bus", str(bus),
                        "--address", str(address),
                    ],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    errors="replace",
                )
                bus, address = _wait_for_bootsel(usb_serial)
                if bus is None:
                    detail = (rb.stdout or "").strip() or (
                        f"serial={usb_serial} did not reboot into BOOTSEL"
                    )
                    logger.warning(
                        "%s (attempt %d/%d)", detail, attempt, attempts
                    )
            if bus is not None:
                res = _picotool_load(bus, address, uf2_path, execute=True)
                if res.returncode == 0:
                    print(res.stdout, end="")
                    return
                detail = (res.stdout or "").strip()
                logger.warning(
                    "picotool load failed on serial=%s (bus=%d "
                    "address=%d) (attempt %d/%d)",
                    usb_serial,
                    bus,
                    address,
                    attempt,
                    attempts,
                )
        if attempt < attempts:
            time.sleep(backoff * attempt)
    if detail:
        print(detail, file=sys.stderr)
    raise RuntimeError(
        f"picotool failed on serial={usb_serial} after {attempts} attempts"
    )


def read_json_from_serial(port, baud, timeout):
    """
    Open the serial port, read until a valid JSON line appears or timeout.
    """
    with Serial(port, baudrate=baud, timeout=1) as ser:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            try:
                return json.loads(line)
            except json.JSONDecodeError:
                continue
    raise RuntimeError(f"[{port}] Timed out waiting for JSON")


_REENUMERATE_TIMEOUT_S = 10.0
_REENUMERATE_POLL_S = 0.2
_UDEV_SETTLE_TIMEOUT_S = 3.0
_INTER_DEVICE_SETTLE_S = 1.0


def _udev_settle(timeout=_UDEV_SETTLE_TIMEOUT_S):
    """Block until udev has drained its pending event queue.

    After post-flash re-enumeration the new ``/dev/ttyACMn`` node is
    created by devtmpfs with the driver-default mode before udev's
    stock rules chgrp it to ``dialout``. ``list_ports.comports()`` sees
    the device the instant the node exists, so opening it immediately
    races against udev's permission pass and fails intermittently with
    ``EACCES``. Settling here closes the window. No-op on systems
    without ``udevadm`` (e.g. macOS).
    """
    try:
        result = subprocess.run(
            ["udevadm", "settle", f"--timeout={int(timeout)}"],
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout + 1,
        )
        if result.returncode != 0:
            detail = (result.stderr or result.stdout or "").strip()
            if detail:
                logger.warning(
                    "udevadm settle failed with exit code %s: %s",
                    result.returncode,
                    detail,
                )
            else:
                logger.warning(
                    "udevadm settle failed with exit code %s",
                    result.returncode,
                )
    except FileNotFoundError:
        pass
    except subprocess.TimeoutExpired:
        logger.warning(
            "udevadm settle timed out after %.1f seconds", timeout + 1
        )


def _resolve_post_flash_port(
    usb_serial, timeout=_REENUMERATE_TIMEOUT_S, poll=_REENUMERATE_POLL_S
):
    """Return the current serial device path for *usb_serial*.

    Polls :func:`find_pico_ports` until the Pico re-appears after its
    post-flash reboot, or *timeout* seconds elapse. ``picotool load``
    triggers a USB re-enumeration, and the kernel does not guarantee
    that the Pico comes back at the same device path it had before —
    so the pre-flash path captured in the initial snapshot can be
    stale. ``usb_serial`` is the stable identity we trust.

    Returns ``None`` if the Pico does not re-enumerate within
    *timeout*. Callers should skip the device in that case rather
    than read JSON from whatever happens to occupy the pre-flash
    path (which is almost certainly a different Pico).
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for dev, sn in find_pico_ports().items():
            if sn == usb_serial:
                return dev
        time.sleep(poll)
    return None


def flash_and_discover(
    uf2_path="build/pico_multi.uf2",
    port=None,
    usb_serial=None,
    baud=115200,
    timeout=10,
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
    usb_serial : str, optional
        Limit to the Pico with this USB serial number (board unique
        ID). Stable across reboots and port renumbering — preferred
        over ``port`` for targeted flashing. If both are given, the
        device must match both.
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
    if usb_serial:
        ports = {k: v for k, v in ports.items() if v == usb_serial}

    if not ports:
        logger.info("No Raspberry Pi Pico serial ports found")
        return []

    logger.info(f"Found Picos on: {ports}")
    all_devices = []

    for idx, (port_dev, port_serial) in enumerate(ports.items()):
        if idx > 0:
            # Let the bus quiet after the previous Pico's post-flash
            # re-enumeration before forcing the next one into BOOTSEL.
            # Back-to-back resets on a shared hub disturb siblings and
            # cause cascading "not found in BOOTSEL" failures.
            time.sleep(_INTER_DEVICE_SETTLE_S)
        logger.info(f"Flashing Pico on port: {port_dev}")
        try:
            flash_uf2(uf2_path, port_serial)
        except RuntimeError as e:
            logger.error(f"Flash failed on {port_dev}: {e}")
            continue

        # picotool load reboots the Pico, which re-enumerates as a
        # CDC device. The kernel may assign a different /dev/ttyACMn
        # than it had pre-flash, so resolve the current path from
        # the stable usb_serial before reading JSON — otherwise a
        # different Pico that drifted into port_dev would have its
        # status attributed to *this* device's usb_serial.
        current_port = _resolve_post_flash_port(port_serial)
        if current_port is None:
            logger.error(
                f"Pico serial={port_serial} (pre-flash port "
                f"{port_dev}) did not re-enumerate within "
                f"{_REENUMERATE_TIMEOUT_S}s; skipping"
            )
            continue

        _udev_settle()

        try:
            data = read_json_from_serial(current_port, baud, timeout)
        except (RuntimeError, OSError) as e:
            logger.error(f"Serial read failed on {current_port}: {e}")
            continue

        data["port"] = current_port
        data["usb_serial"] = port_serial
        all_devices.append(data)
        logger.info(f"Read device info from {current_port}")

    return all_devices


def _load_bootsel_device(
    dev,
    uf2_path,
    attempts=_FLASH_MAX_ATTEMPTS,
    backoff=_FLASH_RETRY_BACKOFF_S,
):
    """Load *uf2_path* onto the BOOTSEL device *dev* (no execute).

    Plain ``picotool load`` does not re-enumerate the device, so the
    bus/address normally stay valid; between retries the address is
    nonetheless re-resolved from sysfs (by serial, when the device has
    one) in case the device dropped and re-enumerated meanwhile.

    Returns True on success, False once *attempts* are exhausted.
    """
    bus, address = dev["bus"], dev["address"]
    serial = dev["usb_serial"]
    for attempt in range(1, attempts + 1):
        res = _picotool_load(bus, address, uf2_path, execute=False)
        if res.returncode == 0:
            return True
        logger.warning(
            "picotool load failed on serial=%s (bus=%d address=%d) "
            "(attempt %d/%d): %s",
            serial,
            bus,
            address,
            attempt,
            attempts,
            (res.stdout or "").strip(),
        )
        if attempt < attempts:
            time.sleep(backoff * attempt)
            if serial is not None:
                for d in _find_bootsel_devices():
                    if d["usb_serial"] == serial:
                        bus, address = d["bus"], d["address"]
                        break
    return False


def flash_and_discover_gpio(
    uf2_path="build/pico_multi.uf2",
    baud=115200,
    timeout=10,
):
    """Mass-flash all Picos via the bussed GPIO BOOTSEL/RUN lines.

    Unlike :func:`flash_and_discover`, this never reboots devices over
    USB — the per-device ``picotool reboot`` round-trips are what fall
    over on the observatory's contended hub. Three phases:

    1. Snapshot CDC Picos (best effort, for a missing-device warning),
       drive every Pico into BOOTSEL at once via the shared GPIO lines
       (works even for wedged/bricked firmware), and wait for the
       BOOTSEL set to settle in sysfs.
    2. ``picotool load`` each device by bus/address — without ``-x``,
       so nothing re-enumerates and the bus stays quiet while every
       device is idle mass-storage.
    3. One mass GPIO reset boots all Picos into the new firmware
       simultaneously; then read each device's JSON over serial.

    Returns the device info dicts (``app_id``, ``port``,
    ``usb_serial``) for every Pico that flashed and re-enumerated.

    Raises ``FileNotFoundError`` for a missing UF2 and ``RuntimeError``
    when no Pico enters BOOTSEL (bad wiring, or no GPIO access).
    """
    from . import gpio  # deferred so --no-gpio paths never need gpiozero

    uf2_path = Path(uf2_path)
    if not uf2_path.is_file():
        raise FileNotFoundError(f"UF2 file not found: {uf2_path}")

    # Phase 1: everyone into BOOTSEL via the shared lines.
    snapshot = find_pico_ports()
    gpio.enter_bootsel()
    bootsel_devices = _wait_for_stable_bootsel_set()
    if not bootsel_devices:
        raise RuntimeError(
            "no Picos entered BOOTSEL after the mass GPIO entry; check "
            "the BOOTSEL/RUN wiring or re-run with --no-gpio"
        )
    seen = {d["usb_serial"] for d in bootsel_devices}
    missing = sorted(set(snapshot.values()) - seen)
    if missing:
        logger.warning(
            "CDC Picos missing from the BOOTSEL set: %s",
            ", ".join(missing),
        )
    logger.info(
        "Flashing %d Pico(s) in BOOTSEL: %s",
        len(bootsel_devices),
        ", ".join(str(d["usb_serial"]) for d in bootsel_devices),
    )

    # Phase 2: load each device over a quiet bus (no -x: everything
    # stays in BOOTSEL until the mass reset below).
    flashed = []
    for dev in bootsel_devices:
        if _load_bootsel_device(dev, uf2_path):
            flashed.append(dev)
        else:
            logger.error(
                "giving up on serial=%s (bus=%d address=%d); it will "
                "mass-reset with the others — re-run flash-picos to "
                "retry it",
                dev["usb_serial"],
                dev["bus"],
                dev["address"],
            )

    # Phase 3: one reset pulse boots the whole fleet, then read device
    # info from each flashed Pico.
    gpio.reset()
    all_devices = []
    for dev in flashed:
        serial = dev["usb_serial"]
        if serial is None:
            logger.warning(
                "flashed Pico at bus=%d address=%d has no USB serial; "
                "cannot map it to a CDC port for the device-info read",
                dev["bus"],
                dev["address"],
            )
            continue
        current_port = _resolve_post_flash_port(serial)
        if current_port is None:
            logger.error(
                f"Pico serial={serial} did not re-enumerate within "
                f"{_REENUMERATE_TIMEOUT_S}s after the mass reset; "
                "skipping"
            )
            continue
        _udev_settle()
        try:
            data = read_json_from_serial(current_port, baud, timeout)
        except (RuntimeError, OSError) as e:
            logger.error(f"Serial read failed on {current_port}: {e}")
            continue
        data["port"] = current_port
        data["usb_serial"] = serial
        all_devices.append(data)
        logger.info(f"Read device info from {current_port}")

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


def main(argv=None):
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
        "--usb-serial",
        default=None,
        help=(
            "USB serial number (Pico board unique ID). Stable across "
            "reboots and port renumbering — preferred over --port for "
            "targeted flashing. Look up in `devices_info.json` or with "
            "`picotool info -a`."
        ),
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
    p.add_argument(
        "--no-gpio",
        action="store_true",
        help=(
            "Skip the GPIO mass-BOOTSEL flash flow and reboot each "
            "Pico into BOOTSEL over USB instead. Required on hosts "
            "without the bussed BOOTSEL/RUN wiring."
        ),
    )
    args = p.parse_args(argv)

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )

    targeting = bool(args.port or args.usb_serial)
    use_gpio = not args.no_gpio and not targeting
    if targeting and not args.no_gpio:
        logger.info(
            "--port/--usb-serial target a single Pico; using the USB "
            "per-device flash path (the GPIO mass reset cannot target "
            "one Pico)"
        )
    if use_gpio:
        from . import gpio  # deferred so --no-gpio never needs gpiozero

        if not gpio.available():
            print(
                "GPIO backend unavailable (gpiozero could not load a "
                "pin factory). On the Pi hub, install lgpio; elsewhere "
                "re-run with --no-gpio to use the USB per-device flash "
                "path.",
                file=sys.stderr,
            )
            sys.exit(1)

    try:
        if use_gpio:
            all_devices = flash_and_discover_gpio(
                uf2_path=args.uf2,
                baud=args.baud,
                timeout=args.timeout,
            )
        else:
            all_devices = flash_and_discover(
                uf2_path=args.uf2,
                port=args.port,
                usb_serial=args.usb_serial,
                baud=args.baud,
                timeout=args.timeout,
            )
    except (FileNotFoundError, RuntimeError) as e:
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
