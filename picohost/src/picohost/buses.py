"""
Redis bus surfaces owned by picohost.

Mirrors the ``CorrWriter`` / ``CorrReader`` / ``CorrConfigStore``
pattern in ``eigsep_observing.corr``: each class takes an
``eigsep_redis.Transport`` at construction, and the concerns are
split into the smallest stable surface per bus.

Six buses here:

- :class:`PicoConfigStore` — persistent single-key blob holding the
  list of picos (app id, serial port, usb serial) written once by
  ``flash-picos`` and read on manager boot as the source of truth.
- :class:`PotCalStore` — persistent single-key blob holding the
  potentiometer calibration (voltage-to-angle slope/intercept for
  both pots). Written by ``calibrate-pot`` and read by
  :class:`picohost.PicoPotentiometer` at startup so a rebooted
  pot Pico picks up its cal from Redis without a local JSON file.
- :class:`MotorPositionStore` — persistent single-key blob holding
  the motor's last known step positions and the firmware
  ``boot_id`` they belong to. Written by
  :class:`picohost.PicoMotor`'s redis handler on every position
  change and read back to re-seed the firmware step counters after
  a pico power cycle (positions live in RAM and reset to 0).
- :class:`PicoCmdReader` — blocking reader for the pico command
  stream. Consumed by the manager's command-relay thread.
- :class:`PicoRespWriter` — writer for the pico response stream.
  Every command yields exactly one response entry, correlated by
  ``request_id``.
- :class:`PicoClaimStore` — TTL-backed soft claims for per-device
  ownership. Claims are advisory and the stream reader never
  rejects a command for claim reasons; the store exists so a
  consumer that wants to coordinate can see who currently holds
  a device.

Per-device liveness is tracked via
``eigsep_redis.HeartbeatWriter(transport, name=pico_heartbeat_name(dev))``
— one heartbeat key per pico. There is no picohost-owned heartbeat
class because the eigsep_redis surface already fits.
"""

import json
import logging

from eigsep_redis import SingleStreamReader, SingleStreamWriter

from .keys import (
    CURRENT_CAL_KEY,
    IMU_CAL_KEY,
    MOTOR_POS_KEY,
    PICO_CMD_STREAM,
    PICO_CONFIG_KEY,
    PICO_RESP_STREAM,
    POT_CAL_KEY,
    pico_claim_key,
)

logger = logging.getLogger(__name__)


class PicoConfigStore:
    """
    Persistent single-key store for the pico device list.

    The value under :data:`PICO_CONFIG_KEY` is a JSON object
    ``{"devices": [...], "upload_time": ...}`` — a *list* of device
    dicts is stored under the ``devices`` key so the canonical
    ``upload_time`` field injected by
    :meth:`Transport.upload_dict` stays at the top level next to
    it.

    The manager is the sole writer: it publishes the list via
    :meth:`upload` after each live-discovery pass (``_discover_new``
    adopts new boards and immediately updates the store).
    ``flash-picos`` only reads from this store — via
    :func:`~picohost.flash_picos._await_manager_confirmation` — to
    confirm that reflashed boards came back alive.
    """

    def __init__(self, transport):
        self.transport = transport

    def upload(self, devices):
        """Upload the device list.

        Parameters
        ----------
        devices : list of dict
            Each dict must carry ``app_id``, ``port``, and
            ``usb_serial`` (extra fields are preserved verbatim).
        """
        self.transport.upload_dict({"devices": list(devices)}, PICO_CONFIG_KEY)

    def get(self):
        """Return the stored device list.

        Returns
        -------
        list of dict or None
            ``None`` if no config has been uploaded, or if the stored
            JSON fails to decode — the manager then falls back to
            flash-and-discover.
        """
        raw = self.transport.get_raw(PICO_CONFIG_KEY)
        if raw is None:
            return None
        try:
            blob = json.loads(raw)
        except json.JSONDecodeError as e:
            logger.warning(
                f"Corrupted {PICO_CONFIG_KEY} in Redis ({e}); "
                "falling back to flash-and-discover."
            )
            return None
        return blob.get("devices", [])

    def clear(self):
        """Delete the stored config. Used by ``--clear-config``."""
        self.transport.r.delete(PICO_CONFIG_KEY)


class PotCalStore:
    """
    Persistent single-key store for potentiometer calibration.

    The value under :data:`POT_CAL_KEY` is a JSON object shaped like
    ``{"pot_az": [m, b], "metadata": {...},
    "upload_time": ...}`` — the canonical ``upload_time`` field is
    injected by :meth:`Transport.upload_dict` at the top level.
    ``metadata`` carries audit fields written by ``calibrate-pot``
    (timestamp, port, sample count, raw voltages) and is preserved
    verbatim on upload/get.

    ``calibrate-pot`` uploads after each calibration run. A fresh
    :class:`picohost.PicoPotentiometer` reads this store at
    ``__init__`` time when given a :class:`PotCalStore` and applies
    the cal before the first status tick — so a pot Pico that
    reboots without a local cal file comes up calibrated from Redis.
    """

    def __init__(self, transport):
        self.transport = transport

    def upload(self, cal):
        """Upload calibration parameters.

        Parameters
        ----------
        cal : dict
            Must carry a ``pot_az`` entry, a
            ``(slope, intercept)`` pair (list or tuple). Extra
            fields (e.g. ``metadata``) are preserved verbatim.
        """
        self.transport.upload_dict(cal, POT_CAL_KEY)

    def get(self):
        """Return the stored calibration dict.

        Returns
        -------
        dict or None
            ``None`` if no calibration has been uploaded, or if the
            stored JSON fails to decode. The caller falls back to
            its next source (JSON file, then uncalibrated).
        """
        raw = self.transport.get_raw(POT_CAL_KEY)
        if raw is None:
            return None
        try:
            return json.loads(raw)
        except json.JSONDecodeError as e:
            logger.warning(
                f"Corrupted {POT_CAL_KEY} in Redis ({e}); "
                "falling back to next calibration source."
            )
            return None

    def clear(self):
        """Delete the stored calibration."""
        self.transport.r.delete(POT_CAL_KEY)


class CurrentCalStore:
    """
    Persistent single-key store for the whole-system current monitor.

    The value under :data:`CURRENT_CAL_KEY` is a JSON object shaped like
    ``{"system_current": [slope, intercept], "metadata": {...},
    "upload_time": ...}`` — the canonical ``upload_time`` field is injected
    by :meth:`Transport.upload_dict` at the top level. ``system_current`` is
    the two-point calibration in amps-vs-volts form: ``slope`` (A/V) and
    ``intercept`` (A), stored exactly as published, so
    ``I = slope*V_adc + intercept``.

    ``calibrate-current`` uploads after each calibration run. A fresh
    :class:`picohost.PicoLidar` reads this store at ``__init__`` time when
    given a :class:`CurrentCalStore` and applies the cal before the first
    status tick — so the lidar Pico hosting the current sensor comes up
    calibrated from Redis after a reboot, with no on-disk file.
    """

    def __init__(self, transport):
        self.transport = transport

    def upload(self, cal):
        """Upload calibration parameters.

        Parameters
        ----------
        cal : dict
            Must carry a ``system_current`` entry, a ``(slope, intercept)``
            pair (list or tuple). Extra fields (e.g. ``metadata``) are
            preserved verbatim.
        """
        self.transport.upload_dict(cal, CURRENT_CAL_KEY)

    def get(self):
        """Return the stored calibration dict.

        Returns
        -------
        dict or None
            ``None`` if no calibration has been uploaded, or if the stored
            JSON fails to decode. The device then stays uncalibrated
            (``current_a`` publishes as ``None``).
        """
        raw = self.transport.get_raw(CURRENT_CAL_KEY)
        if raw is None:
            return None
        try:
            return json.loads(raw)
        except json.JSONDecodeError as e:
            logger.warning(
                f"Corrupted {CURRENT_CAL_KEY} in Redis ({e}); "
                "the device stays uncalibrated (current_a=None)."
            )
            return None

    def clear(self):
        """Delete the stored calibration."""
        self.transport.r.delete(CURRENT_CAL_KEY)


class ImuCalStore:
    """Persistent single-key store for IMU mount calibration.

    Value under :data:`IMU_CAL_KEY` is a JSON object with optional
    ``imu_el`` / ``imu_az`` sections (each a mount calibration dict) plus
    a ``metadata`` block; ``upload_time`` is injected by
    :meth:`Transport.upload_dict` at the top level. Each section is
    independent so a partially-calibrated fleet (e.g. ``imu_el`` dead)
    round-trips cleanly.
    A fresh :class:`picohost.PicoIMU` reads its own section at
    ``__init__`` and applies the conversion from the first status tick.
    """

    def __init__(self, transport):
        self.transport = transport

    def upload(self, cal):
        """Upload calibration parameters, merging into any existing store.

        Read-modify-write: top-level keys in ``cal`` (sections and
        ``metadata``) overwrite their counterparts, while sections absent
        from ``cal`` are preserved. This keeps a partially-calibrated fleet
        intact — e.g. a ``--mode elevation`` run that only produces
        ``imu_el`` must not wipe a previously-stored ``imu_az`` section.

        Parameters
        ----------
        cal : dict
            May carry ``imu_az`` and/or ``imu_el`` section dicts and an
            optional ``metadata`` block. Extra fields are preserved verbatim.
        """
        merged = self.get() or {}
        # Drop the stale top-level upload_time; upload_dict re-injects it.
        merged.pop("upload_time", None)
        merged.update(cal)
        self.transport.upload_dict(merged, IMU_CAL_KEY)

    def get(self):
        """Return the stored calibration dict.

        Returns
        -------
        dict or None
            ``None`` if no calibration has been uploaded, or if the
            stored JSON fails to decode. The caller falls back to
            uncalibrated operation.
        """
        raw = self.transport.get_raw(IMU_CAL_KEY)
        if raw is None:
            return None
        try:
            return json.loads(raw)
        except json.JSONDecodeError as e:
            logger.warning(
                f"Corrupted {IMU_CAL_KEY} in Redis ({e}); "
                "falling back to uncalibrated."
            )
            return None

    def clear(self):
        """Delete the stored calibration."""
        self.transport.r.delete(IMU_CAL_KEY)


class MotorPositionStore:
    """
    Persistent single-key store for the motor's last known position.

    The value under :data:`MOTOR_POS_KEY` is a JSON object
    ``{"az_pos": int, "el_pos": int, "boot_id": int,
    "upload_time": ...}`` — the canonical ``upload_time`` field is
    injected by :meth:`Transport.upload_dict` at the top level.

    :class:`picohost.PicoMotor`'s redis handler uploads on every
    position change and compares the stored ``boot_id`` against the
    one in each firmware status packet: a mismatch means the pico
    power-cycled (its RAM step counters reset to 0) and the stored
    positions are pushed back down via ``az_set_pos``/``el_set_pos``.
    The store therefore only ever holds positions paired with the
    boot they were counted in.

    This recovers the operator-defined zero across reboots; it cannot
    detect motion that happened while unpowered (the rig being moved
    by hand, or steps lost between the last status tick and a
    power cut) — an absolute sensor (pot) is the only cure for that.
    """

    def __init__(self, transport):
        self.transport = transport

    def upload(self, az_pos, el_pos, boot_id):
        """Upload the current position checkpoint.

        Parameters
        ----------
        az_pos, el_pos : int
            Step positions as reported by the firmware.
        boot_id : int
            The firmware boot id the positions were observed under.
        """
        self.transport.upload_dict(
            {
                "az_pos": int(az_pos),
                "el_pos": int(el_pos),
                "boot_id": int(boot_id),
            },
            MOTOR_POS_KEY,
        )

    def get(self):
        """Return the stored checkpoint dict.

        Returns
        -------
        dict or None
            ``None`` if no checkpoint has been uploaded, or if the
            stored JSON fails to decode or lacks integer ``az_pos`` /
            ``el_pos`` / ``boot_id`` fields. The caller treats that
            as "no checkpoint" — it never seeds from a blob it cannot
            fully validate.
        """
        raw = self.transport.get_raw(MOTOR_POS_KEY)
        if raw is None:
            return None
        try:
            blob = json.loads(raw)
        except json.JSONDecodeError as e:
            logger.warning(
                f"Corrupted {MOTOR_POS_KEY} in Redis ({e}); "
                "ignoring checkpoint."
            )
            return None
        try:
            for key in ("az_pos", "el_pos", "boot_id"):
                blob[key] = int(blob[key])
        except (KeyError, TypeError, ValueError) as e:
            logger.warning(
                f"Malformed {MOTOR_POS_KEY} in Redis ({e}); "
                "ignoring checkpoint."
            )
            return None
        return blob

    def clear(self):
        """Delete the stored checkpoint."""
        self.transport.r.delete(MOTOR_POS_KEY)


class PicoCmdReader(SingleStreamReader):
    """
    Blocking reader for the pico command stream.

    Subclasses :class:`eigsep_redis.SingleStreamReader` for cursor
    bookkeeping but overrides :meth:`read` because the manager's
    command-relay thread consumes up to ``count`` entries per call
    and wants the raw ``(msg_id, fields)`` shape — the base's
    single-decoded-entry contract doesn't fit. Only the manager's
    command-relay thread is expected to call :meth:`read`; other
    consumers should talk to :class:`PicoRespWriter` via the
    response stream.
    """

    stream = PICO_CMD_STREAM
    data_set = None  # singleton — no registry-set membership check

    def read(self, timeout=1.0, count=10):
        """Blocking read of up to ``count`` command entries.

        Parameters
        ----------
        timeout : float
            Maximum seconds to block waiting for new entries. ``0``
            returns immediately; any positive value is converted to
            milliseconds for Redis' ``XREAD BLOCK``.
        count : int
            Maximum number of entries to consume in one call.

        Returns
        -------
        list of (bytes, dict)
            ``[(msg_id, fields), ...]``. Empty list on timeout.
            ``fields`` keys and values are bytes exactly as returned
            by redis-py; decoding is the caller's responsibility.
        """
        block_time = int(timeout * 1000) if timeout else 0
        result = self.transport.r.xread(
            {self.stream: self.transport.get_last_read_id(self.stream)},
            block=block_time,
            count=count,
        )
        if not result:
            return []
        _stream, messages = result[0]
        self.transport.set_last_read_id(self.stream, messages[-1][0])
        return messages


class PicoRespWriter(SingleStreamWriter):
    """Writer for the pico response stream.

    Every processed command yields one entry on
    :data:`PICO_RESP_STREAM` carrying the command's ``target``,
    ``source``, ``request_id``, ``status`` (``"ok"``/``"error"``),
    and JSON-encoded ``data`` payload. An optional ``warning``
    field is included when present (used today to signal that a
    non-owner overrode a soft claim).

    The stream is intentionally not length-bounded: response
    volume tracks command volume — bounded by the caller — and
    a dead consumer starving its own responses is a bug, not a
    failure mode to paper over.
    """

    stream = PICO_RESP_STREAM
    data_set = None  # singleton — no DATA_STREAMS_SET registration
    maxlen = None  # response stream is intentionally unbounded

    def _encode(self, target, source, request_id, status, data, warning=None):
        entry = {
            "target": target,
            "source": source,
            "request_id": request_id,
            "status": status,
            "data": json.dumps(data),
        }
        if warning is not None:
            entry["warning"] = warning
        return entry

    def send(self, target, source, request_id, status, data, warning=None):
        """Publish one response entry.

        Parameters
        ----------
        target : str
        source : str
        request_id : str
        status : str
            ``"ok"`` or ``"error"``.
        data : dict
            JSON-serializable payload.
        warning : str or None
            Optional advisory message attached to an otherwise-ok
            response (e.g. claim override).
        """
        self.publish(target, source, request_id, status, data, warning=warning)


class PicoClaimStore:
    """TTL-backed soft claim store.

    Claims are advisory — a non-owner can still send commands, and
    the response stream simply tags such overrides with a
    ``warning`` field. The store exists so a coordinator (e.g. an
    observing loop) can see which device is currently held and by
    whom, and releases itself when its TTL expires.
    """

    def __init__(self, transport):
        self.transport = transport

    def set(self, device, owner, ttl):
        """Register ``owner`` as holding ``device`` for ``ttl`` seconds."""
        self.transport.r.set(pico_claim_key(device), owner, ex=int(ttl))

    def get(self, device):
        """Return the current owner string, or ``None`` if unclaimed."""
        raw = self.transport.r.get(pico_claim_key(device))
        if raw is None:
            return None
        if isinstance(raw, bytes):
            return raw.decode("utf-8")
        return str(raw)

    def delete(self, device):
        """Drop any existing claim on ``device``."""
        self.transport.r.delete(pico_claim_key(device))
