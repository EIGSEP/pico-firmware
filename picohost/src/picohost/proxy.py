"""
Redis-backed proxy for pico devices managed by PicoManager.

A proxy routes device method calls through Redis instead of serial.
Construction always succeeds — no hardware check. At command time the
proxy checks whether PicoManager has the device registered; if not, the
command is a no-op that returns ``None``. Every device method is invoked
by name via :meth:`PicoProxy.send_command`; there is intentionally no
per-device subclass.

Usage::

    from picohost.proxy import PicoProxy

    sw = PicoProxy("rfswitch", redis_client)
    sw.send_command("switch", state="RFANT")   # routed via PicoManager
    sw.is_available                             # True if registered

    peltier = PicoProxy("tempctrl", redis_client)
    peltier.send_command("set_temperature", T_LNA=25, T_LOAD=25)
"""

import json
import logging
import time
import uuid

from .manager import CMD_STREAM, HEALTH_HASH, PICOS_SET, RESP_STREAM

logger = logging.getLogger(__name__)


class PicoProxy:
    """
    Redis-backed proxy for a pico device managed by PicoManager.

    Parameters
    ----------
    name : str
        Device name as registered by PicoManager (e.g. ``"rfswitch"``).
    redis : redis.Redis
        The underlying Redis client (not an ``EigsepRedis`` wrapper).
    source : str
        Identifier included in command stream entries for logging and
        soft-claim tracking.
    timeout : float
        Default seconds to wait for a command response.
    """

    def __init__(self, name, redis, source="client", timeout=5.0):
        self.name = name
        self.redis = redis
        self.source = source
        self.timeout = timeout
        self.logger = logger

    @property
    def is_available(self):
        """True if PicoManager has registered this device."""
        return self.redis.sismember(PICOS_SET, self.name)

    @property
    def health(self):
        """Device health dict from PicoManager, or None."""
        raw = self.redis.hget(HEALTH_HASH, self.name)
        if raw is None:
            return None
        return json.loads(raw)

    def send_command(self, action, **kwargs):
        """
        Send a command to PicoManager and wait for the response.

        If the device is not available, logs a warning and returns
        ``None`` (no-op).

        Parameters
        ----------
        action : str
            Method name to invoke on the device (e.g. ``"switch"``).
        **kwargs
            Arguments forwarded to the device method.

        Returns
        -------
        dict or None
            Parsed response data, or ``None`` if the device is
            unavailable.

        Raises
        ------
        TimeoutError
            If no response arrives within ``self.timeout`` seconds.
        RuntimeError
            If PicoManager returns an error status.
        """
        if not self.is_available:
            self.logger.warning(
                f"{self.name} not available, skipping command"
            )
            return None

        request_id = str(uuid.uuid4())
        # Capture current stream end so we don't miss fast responses.
        try:
            info = self.redis.xinfo_stream(RESP_STREAM)
            last_id = info["last-generated-id"]
        except Exception:
            # Stream doesn't exist yet — read from the beginning.
            last_id = "0-0"
        cmd = {"action": action, **kwargs}
        self.redis.xadd(
            CMD_STREAM,
            {
                "target": self.name,
                "source": self.source,
                "request_id": request_id,
                "cmd": json.dumps(cmd),
            },
        )
        return self._wait_response(request_id, last_id)

    def _wait_response(self, request_id, last_id):
        """
        Poll ``stream:pico_resp`` for a response matching *request_id*.
        """
        deadline = time.monotonic() + self.timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(
                    f"No response for {self.name} within {self.timeout}s"
                )
            block_ms = int(remaining * 1000)
            result = self.redis.xread(
                {RESP_STREAM: last_id}, block=block_ms, count=10
            )
            if not result:
                raise TimeoutError(
                    f"No response for {self.name} within {self.timeout}s"
                )
            for _stream, messages in result:
                for msg_id, fields in messages:
                    last_id = msg_id
                    rid = fields.get(b"request_id") or fields.get(
                        "request_id"
                    )
                    if isinstance(rid, bytes):
                        rid = rid.decode()
                    if rid != request_id:
                        continue
                    # Found our response
                    status = fields.get(b"status") or fields.get(
                        "status"
                    )
                    if isinstance(status, bytes):
                        status = status.decode()
                    data_raw = fields.get(b"data") or fields.get(
                        "data", b"{}"
                    )
                    if isinstance(data_raw, bytes):
                        data_raw = data_raw.decode()
                    data = json.loads(data_raw)
                    if status == "error":
                        raise RuntimeError(
                            f"Command failed on {self.name}: "
                            f"{data.get('error', data)}"
                        )
                    return data
