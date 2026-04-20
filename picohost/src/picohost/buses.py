"""
Redis bus surfaces owned by picohost.

Mirrors the ``CorrWriter`` / ``CorrReader`` / ``CorrConfigStore``
pattern in ``eigsep_observing.corr``: each class takes an
``eigsep_redis.Transport`` at construction, and the concerns are
split into the smallest stable surface per bus.

Four buses here:

- :class:`PicoConfigStore` — persistent single-key blob holding the
  list of picos (app id, serial port, usb serial) written once by
  ``flash-picos`` and read on manager boot as the source of truth.
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

from .keys import (
    PICO_CMD_STREAM,
    PICO_CONFIG_KEY,
    PICO_RESP_STREAM,
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

    ``flash-picos`` uploads the list after every flash pass. The
    manager reads it once at boot and treats it as the source of
    truth; if it's missing, the manager falls back to running
    ``flash-and-discover`` itself and writes the result back with
    :meth:`upload`.
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


class PicoCmdReader:
    """
    Blocking reader for the pico command stream.

    Mirrors :class:`eigsep_redis.StatusReader` — a single
    hard-coded stream (:data:`PICO_CMD_STREAM`) with per-transport
    last-read-id tracking so independent readers don't race. Only
    the manager's command-relay thread is expected to call
    :meth:`read`; other consumers should talk to
    :class:`PicoRespWriter` via the response stream.
    """

    def __init__(self, transport):
        self.transport = transport

    @property
    def stream(self):
        """``{PICO_CMD_STREAM: last_read_id}`` — view, used for blocking reads."""
        return {
            PICO_CMD_STREAM: self.transport._get_last_read_id(PICO_CMD_STREAM)
        }

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
            self.stream, block=block_time, count=count
        )
        if not result:
            return []
        # Single-stream read → result has exactly one (stream, messages) tuple.
        _stream, messages = result[0]
        last_id = messages[-1][0]
        self.transport._set_last_read_id(PICO_CMD_STREAM, last_id)
        return messages


class PicoRespWriter:
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

    def __init__(self, transport):
        self.transport = transport

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
        entry = {
            "target": target,
            "source": source,
            "request_id": request_id,
            "status": status,
            "data": json.dumps(data),
        }
        if warning is not None:
            entry["warning"] = warning
        self.transport.r.xadd(PICO_RESP_STREAM, entry)


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
