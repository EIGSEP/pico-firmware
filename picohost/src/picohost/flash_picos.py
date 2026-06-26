#!/usr/bin/env python3
import argparse
import errno
import json
import logging
import os
import select
import subprocess
import sys
import time
from pathlib import Path

from eigsep_redis import HeartbeatReader, Transport
from serial.tools import list_ports

from . import manager_service
from .buses import PicoConfigStore
from .keys import PICO_CONFIG_KEY, pico_heartbeat_name

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
        devices.append({"usb_serial": serial, "bus": bus, "address": address})
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
        keys = {(d["usb_serial"], d["bus"], d["address"]) for d in devices}
        now = time.monotonic()
        if keys != last_keys:
            last_keys = keys
            last = devices
            stable_since = now
        elif keys and now - stable_since >= stable:
            return devices
        time.sleep(poll)
    return last


_CONFIRM_TIMEOUT_S = 45.0   # > boot stagger (~7x1.5s) + udev + a couple 5s manager cadences
_CONFIRM_POLL_S = 0.5


def _await_manager_confirmation(expected, transport,
                                timeout=_CONFIRM_TIMEOUT_S, poll=_CONFIRM_POLL_S):
    """Poll the manager-owned pico_config until *expected* serials are alive.

    Returns (confirmed, stragglers). A serial is confirmed only when its board's
    per-name heartbeat is alive=True — mere presence in pico_config is NOT
    sufficient. pico_config has no TTL and the manager (never stopped during a
    flash) leaves stale entries, so a board that was already known before the
    flash would appear "present" before it even re-enumerates. The manager sets a
    board's heartbeat alive=False while it is in BOOTSEL during the flash (its
    reconnect fails) and only back to alive=True once it re-establishes the
    board afterward, so alive=True is a genuine "came back from this flash"
    signal.

    Circular-import note: manager.py imports from flash_picos at module load,
    so APP_NAMES is imported lazily inside this function, not at module top level.
    """
    # Lazy import to avoid a circular dependency: manager imports from
    # flash_picos at module load, so flash_picos must not import APP_NAMES
    # from manager at module top level.
    from .manager import APP_NAMES

    store = PicoConfigStore(transport)
    expected = set(expected)
    deadline = time.monotonic() + timeout
    while True:
        entries = store.get() or []
        serial_to_appid = {
            d.get("usb_serial"): d.get("app_id") for d in entries
        }
        confirmed = set()
        for serial in expected:
            app_id = serial_to_appid.get(serial)
            if app_id is None:
                continue
            name = APP_NAMES.get(app_id)
            if name is None:
                continue
            if HeartbeatReader(transport, name=pico_heartbeat_name(name)).check():
                confirmed.add(serial)
        if confirmed == expected or time.monotonic() >= deadline:
            return confirmed, expected - confirmed
        time.sleep(poll)


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
                f"serial={usb_serial} not visible as a Pico (2e8a) USB device"
            )
            logger.warning("%s (attempt %d/%d)", detail, attempt, attempts)
        else:
            if not in_bootsel:
                rb = subprocess.run(
                    [
                        "picotool",
                        "reboot",
                        "-u",
                        "-f",
                        "--bus",
                        str(bus),
                        "--address",
                        str(address),
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


# Extra wall-clock budget, beyond the read *timeout*, for the serial
# port's open and close to complete. A marginal CDC port whose USB
# endpoint has wedged (the "-110 mute" state) can block in the kernel's
# cdc_acm teardown — os.close() of the tty never returns — and pyserial's
# timeout bounds only reads, not open/close.
_SERIAL_TEARDOWN_GRACE_S = 2.0

# The open/read/close runs in a *child process* (see _serial_reader.py),
# launched by file path so it does not import the picohost package (and
# its heavy deps) on every per-board read.
_SERIAL_READER_SCRIPT = str(Path(__file__).with_name("_serial_reader.py"))


def _read_line_bounded(stream, bound):
    """Return the first newline-terminated line from *stream*, or ``None``.

    Polls the underlying fd with :func:`select.select` so the wait is
    bounded by *bound* seconds even if the writer never sends a newline or
    never closes its end (a wedged reader child). Returns ``None`` on
    timeout or on EOF-without-a-line.
    """
    fd = stream.fileno()
    buf = bytearray()
    deadline = time.monotonic() + bound
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        readable, _, _ = select.select([fd], [], [], remaining)
        if not readable:
            return None
        chunk = os.read(fd, 4096)
        if not chunk:  # EOF before any newline
            return None
        buf.extend(chunk)
        nl = buf.find(b"\n")
        if nl != -1:
            return bytes(buf[: nl + 1]).decode("utf-8", errors="ignore")


def _abandon_reader(proc):
    """Tear down the reader child without ever blocking on a wedged one.

    A reader stuck in ``os.close()`` of a mute CDC port is in
    uninterruptible sleep: it will not die on ``SIGKILL`` until the kernel
    teardown returns, so we must NOT wait for it — flash-picos has to stay
    free to exit. The orphan is reparented to init, which reaps it once the
    syscall finally unwinds (and ``subprocess`` reaps any cleanly-finished
    child on the next :class:`~subprocess.Popen`). A reader that already
    exited is reaped here with a short, bounded wait so it does not linger.
    """
    try:
        proc.stdout.close()
    except OSError:
        pass
    if proc.poll() is None:
        try:
            proc.kill()
        except OSError:
            pass
    try:
        proc.wait(timeout=_SERIAL_TEARDOWN_GRACE_S)
    except subprocess.TimeoutExpired:
        pass


def read_json_from_serial(port, baud, timeout):
    """Open *port* and return the first valid JSON line, or raise.

    The open/read/close runs in a child process (:mod:`_serial_reader`),
    not a thread, and is bounded by wall clock (*timeout* for the read,
    plus :data:`_SERIAL_TEARDOWN_GRACE_S` for open and close). A board
    whose USB endpoint has wedged can block the kernel's cdc_acm teardown
    indefinitely — ``os.close()`` of the tty never returns, and pyserial's
    ``timeout`` does not cover it. A *thread* cannot escape that: bounding
    the wait frees the calling thread, but the worker stays stuck in the
    kernel and a process cannot finish ``exit_group()`` while any thread is
    in uninterruptible sleep — so flash-picos hung at exit even after the
    main thread was done (and ``eigsep-field patch``, waiting on it
    unbounded, hung forever). Running it in a child process means the wedge
    pins the child; we abandon it and exit cleanly.

    The child writes one JSON line and flushes *before* its close, so a
    line read just before a blocking close is still returned. Raises
    :class:`RuntimeError` on timeout/wedge, or an ``OSError`` (with
    ``errno`` preserved) on open/read failure — mirroring the previous
    contract so :func:`_classify_read_failure` verdicts are unchanged.
    """
    proc = subprocess.Popen(
        [
            sys.executable,
            _SERIAL_READER_SCRIPT,
            str(port),
            str(baud),
            str(timeout),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
    )
    try:
        line = _read_line_bounded(
            proc.stdout, timeout + _SERIAL_TEARDOWN_GRACE_S
        )
    finally:
        _abandon_reader(proc)

    if line is None:
        raise RuntimeError(
            f"[{port}] serial open/read/close did not complete within "
            f"{timeout + _SERIAL_TEARDOWN_GRACE_S:.0f}s; the port is wedged "
            f"(USB endpoint stuck) — abandoning it"
        )
    try:
        msg = json.loads(line)
    except json.JSONDecodeError as e:
        raise RuntimeError(
            f"[{port}] serial read returned invalid JSON: {e.msg}"
        ) from e
    if "data" in msg:
        return msg["data"]
    if "err" in msg:
        code = msg.get("errno")
        if code is not None:
            raise OSError(code, msg["err"])
        raise OSError(msg["err"])
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


def _classify_read_failure(exc):
    """Map a readback exception to a short, operator-facing verdict.

    Distinguishes the stages an operator must tell apart when a board
    drops out of the device count: the port opened but the firmware
    stayed silent (a :class:`RuntimeError` timeout from
    :func:`read_json_from_serial`), a permission race on the freshly
    created node (``EACCES``), the port being held by another process
    (``EBUSY`` — ModemManager probing the new CDC device, or a stray
    PicoManager), or any other open/read error. ``pyserial`` sets
    ``errno`` on its :class:`~serial.SerialException` (an ``OSError``
    subclass); fall back to scanning the message when it does not.
    """
    if isinstance(exc, RuntimeError):
        return (
            "port opened but no JSON before timeout (silent firmware "
            "or wrong DIP switch?)"
        )
    code = getattr(exc, "errno", None)
    text = str(exc)
    if code == errno.EACCES or "Permission denied" in text:
        return (
            "permission denied opening port (EACCES — udev mode race, "
            "or the user is not in the dialout group)"
        )
    if code == errno.EBUSY or "resource busy" in text.lower():
        return (
            "port busy (EBUSY — held by another process, e.g. "
            "ModemManager probing the new CDC port, or a stray "
            "PicoManager)"
        )
    if (
        code == errno.ETIMEDOUT
        or "Errno 110" in text
        or "Connection timed out" in text
    ):
        return (
            "USB IN-endpoint timeout (-110/ETIMEDOUT — board enumerated "
            "but transiently mute after the mass re-enumeration; usually "
            "self-recovers on a re-read sweep)"
        )
    return f"error opening/reading port: {exc}"


def _read_cdc_outcome(port, usb_serial, baud, timeout):
    """Attempt one device-info read; return ``(data_or_None, reason)``.

    ``reason`` is ``None`` on success, otherwise a short verdict from
    :func:`_classify_read_failure` (or a no-port message). Used by the
    manager's discovery probe via :func:`_read_cdc_port`.
    """
    if port is None:
        return None, (
            f"did not re-enumerate as CDC within {_REENUMERATE_TIMEOUT_S:.0f}s"
        )
    _udev_settle()
    try:
        data = read_json_from_serial(port, baud, timeout)
    except (RuntimeError, OSError) as e:
        return None, _classify_read_failure(e)
    data["port"] = port
    data["usb_serial"] = usb_serial
    return data, None


def _read_cdc_port(port, usb_serial, baud, timeout):
    """Read one device-info JSON line from CDC *port*.

    Returns the device-info dict (with ``port``/``usb_serial`` added),
    or ``None`` on failure. A single attempt is enough: the post-flash
    re-enumeration races are absorbed before this point (fleet CDC
    discovery and bus settle on the GPIO path, port re-resolution on
    the USB path, udev settle here) and the read itself waits up to
    *timeout* seconds for a status line the firmware emits every
    200 ms — a board that still says nothing is persistently broken
    (wrong DIP, failed boot), not transiently busy.
    """
    data, reason = _read_cdc_outcome(port, usb_serial, baud, timeout)
    if data is None:
        if port is None:
            logger.error("no CDC port for serial=%s (%s)", usb_serial, reason)
        else:
            logger.error(
                "could not read device info for serial=%s on %s: %s (it "
                "flashed — re-run flash-picos, or check the board's DIP/app "
                "if it never reports)",
                usb_serial,
                port,
                reason,
            )
    return data




def flash_and_discover(
    uf2_path="build/pico_multi.uf2",
    port=None,
    usb_serial=None,
):
    """Flash all attached Picos and return the USB serials of those
    successfully flashed.

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

    Returns
    -------
    list[str]
        Sorted list of USB serials that were successfully flashed.
        Caller confirms via the manager-owned ``pico_config``.

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
    flashed_serials = []

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

        flashed_serials.append(port_serial)

    return sorted(flashed_serials)


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


_BOOT_FLEET_ATTEMPTS = 3
# Gap between per-board boots so re-enumeration never storms (a
# simultaneous storm is what leaves boards transiently mute). It also
# gives each board time to leave BOOTSEL before the next is rebooted.
_BOOT_STAGGER_S = 1.5


def _picotool_reboot_app(bus, address):
    """``picotool reboot -a`` (into the application) by bus/address.

    Boots a device that is in BOOTSEL into its loaded image. Targeted by
    ``--bus``/``--address`` (no ``--ser`` — its live descriptor read
    corrupts under hub contention) and no ``-f`` (the device is already
    in BOOTSEL). Returns the :class:`subprocess.CompletedProcess`.
    """
    return subprocess.run(
        [
            "picotool",
            "reboot",
            "-a",
            "--bus",
            str(bus),
            "--address",
            str(address),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        errors="replace",
    )


def _boot_fleet_staggered(
    flashed,
    stagger=_BOOT_STAGGER_S,
    attempts=_BOOT_FLEET_ATTEMPTS,
):
    """Boot each loaded board into its image, one at a time, staggered.

    Replaces the single shared ``gpio.reset()`` pulse. That pulse is
    unreliable across the full fleet (boards miss it and stay in BOOTSEL),
    and booting everything at once triggers a simultaneous USB
    re-enumeration storm that leaves some boards transiently mute — and a
    mute board cannot be reached over USB to read or recover. Booting each
    board individually with ``picotool reboot -a`` (by bus/address, no
    ``--ser``/``-f``) is reliable on a device already in BOOTSEL, and the
    *stagger* gap between boards spreads re-enumeration out so the bus
    never storms.

    Each board is retried up to *attempts* times: after the reboot it
    should drop out of the BOOTSEL set; if it has not (the reboot was
    lost, or it re-entered BOOTSEL), it is rebooted again from its freshly
    resolved BOOTSEL address. A board that never leaves BOOTSEL has its
    BOOTSEL/QSPI-CS line held low at boot (hardware) — left in BOOTSEL for
    the caller's stuck-board report.

    Returns the set of serials that left BOOTSEL, so the caller can wait
    for exactly that many CDC devices instead of timing out on a
    hardware-stuck board that will never re-enumerate.
    """
    booted = set()
    for dev in flashed:
        serial = dev["usb_serial"]
        bus, address = dev["bus"], dev["address"]
        for attempt in range(1, attempts + 1):
            logger.info(
                "booting serial=%s into its image (bus=%d address=%d) "
                "(attempt %d/%d)",
                serial,
                bus,
                address,
                attempt,
                attempts,
            )
            _picotool_reboot_app(bus, address)
            time.sleep(stagger)
            in_bootsel = {d["usb_serial"]: d for d in _find_bootsel_devices()}
            if serial is None or serial not in in_bootsel:
                if serial is not None:
                    booted.add(serial)
                break
            # still in BOOTSEL — retry from its (possibly new) address
            bus = in_bootsel[serial]["bus"]
            address = in_bootsel[serial]["address"]
    return booted



def flash_and_discover_gpio(
    uf2_path="build/pico_multi.uf2",
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
    3. Boot each loaded board individually and staggered via
       ``picotool reboot -a``.

    Returns the sorted list of USB serials that were successfully
    loaded and booted.  Caller confirms via the manager-owned
    ``pico_config``.

    Raises ``FileNotFoundError`` for a missing UF2 and ``RuntimeError``
    when no Pico enters BOOTSEL (bad wiring, or no GPIO access).
    """
    from . import gpio  # deferred so --no-gpio paths never import gpio

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
    # stays in BOOTSEL until the staggered per-board boot below).
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

    # Phase 3: boot each loaded board individually and staggered.
    #
    # Per-board picotool reboot -a, staggered, boots reliably without
    # storming the bus (simultaneous re-enumeration leaves some boards
    # transiently mute, and a mute board cannot be reached over USB to
    # recover).
    expected_serials = {d["usb_serial"] for d in flashed if d["usb_serial"]}
    booted = _boot_fleet_staggered(flashed)

    # Surface any board still in BOOTSEL, distinguishing the two causes:
    # a board we flashed that will not leave BOOTSEL even after its
    # per-board reboot has a valid image but its BOOTSEL/QSPI-CS line is
    # held low at boot (hardware); a board whose load never succeeded
    # simply has no image to boot.
    stuck = _find_bootsel_devices()
    if stuck:
        held = [d for d in stuck if d["usb_serial"] in expected_serials]
        unflashed = [
            d for d in stuck if d["usb_serial"] not in expected_serials
        ]
        if held:
            logger.error(
                "%d Pico(s) still in BOOTSEL after a per-board reboot — "
                "image loaded but the board will not leave BOOTSEL, so its "
                "BOOTSEL/QSPI-CS line is held low at boot (hardware, not "
                "fixable by re-flashing): %s. Check that board's BOOTSEL "
                "tap/solder joint on the shared line.",
                len(held),
                ", ".join(
                    f"bus={d['bus']} address={d['address']} "
                    f"serial={d['usb_serial']}"
                    for d in held
                ),
            )
        if unflashed:
            logger.error(
                "%d Pico(s) still in BOOTSEL — NOT flashed (load did not "
                "succeed): %s. Re-run flash-picos; if the same board fails "
                "every time, check its USB cable/power and `picotool info`.",
                len(unflashed),
                ", ".join(
                    f"bus={d['bus']} address={d['address']} "
                    f"serial={d['usb_serial']}"
                    for d in unflashed
                ),
            )

    return sorted(s for s in expected_serials if s)



def main(argv=None):
    p = argparse.ArgumentParser(
        description=(
            "Flash all attached Picos and confirm via the manager-owned "
            "pico_config that each board is reporting."
        ),
        epilog=(
            "flash-picos is FLASH-ONLY: it loads the UF2 firmware and confirms "
            "the result by polling picomanager's Redis pico_config (which the "
            "manager owns by self-discovering boards from their CDC status). "
            "If picomanager is not running, reports loaded count and exits "
            "successfully — the manager owns device discovery and publication."
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
            "targeted flashing. Look up with `picotool info -a`."
        ),
    )
    p.add_argument(
        "--uf2",
        default="build/pico_multi.uf2",
        help="Path to your pico_multi.uf2",
    )
    p.add_argument(
        "--redis-host",
        default="localhost",
        help="Redis host for manager confirmation polling",
    )
    p.add_argument(
        "--redis-port",
        type=int,
        default=6379,
        help="Redis port for manager confirmation polling",
    )
    p.add_argument(
        "--output-file",
        default=None,
        help=(
            "Optional: write the confirmed device list (from "
            "pico_config) to this JSON file after confirmation."
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
        from . import gpio  # deferred so --no-gpio paths never import gpio

        if not gpio.available():
            print(
                "GPIO backend unavailable: the `pinctrl` CLI was not "
                "found on PATH. Run on the Pi hub (pinctrl ships with "
                "Raspberry Pi OS), or re-run with --no-gpio to use the "
                "USB per-device flash path.",
                file=sys.stderr,
            )
            sys.exit(1)

    try:
        if use_gpio:
            all_serials = flash_and_discover_gpio(uf2_path=args.uf2)
        else:
            all_serials = flash_and_discover(
                uf2_path=args.uf2,
                port=args.port,
                usb_serial=args.usb_serial,
            )
    except (FileNotFoundError, RuntimeError) as e:
        print(str(e), file=sys.stderr)
        sys.exit(1)

    if not all_serials:
        print("No devices loaded.", file=sys.stderr)
        sys.exit(1)

    if not manager_service.manager_is_active():
        print(
            f"Loaded {len(all_serials)} board(s). picomanager is not active; "
            "start it to confirm device info (or this is a manager-less host)."
        )
        return

    transport = Transport(host=args.redis_host, port=args.redis_port)
    confirmed, stragglers = _await_manager_confirmation(all_serials, transport)
    print(
        f"Confirmed {len(confirmed)}/{len(all_serials)} board(s) via "
        "picomanager."
    )

    if args.output_file:
        store = PicoConfigStore(transport)
        with open(args.output_file, "w") as f:
            json.dump(store.get(), f, indent=2)
        print(f"Wrote confirmed device info to {args.output_file}.")

    if stragglers:
        print(
            f"  NOT confirmed (booted but not reporting): "
            f"{', '.join(sorted(stragglers))}",
            file=sys.stderr,
        )
        sys.exit(1)


if __name__ == "__main__":
    main()
