"""
Redis-backed proxies for pico devices managed by PicoManager.

A proxy has the same command interface as the real device class but
routes calls through Redis instead of serial. Construction always
succeeds — no hardware check. At command time the proxy checks
whether PicoManager has the device registered; if not, the command
is a no-op that returns ``None``.

Usage::

    from picohost.proxy import RFSwitchProxy

    sw = RFSwitchProxy("rfswitch", redis_client)
    sw.switch("RFANT")       # routed via PicoManager
    sw.is_available           # True if PicoManager has registered it
"""

import json
import logging
import uuid

from .base import PicoRFSwitch
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
        return self._wait_response(request_id)

    def _wait_response(self, request_id):
        """
        Poll ``stream:pico_resp`` for a response matching *request_id*.
        """
        # Read only new messages from the response stream.
        last_id = "$"
        timeout_ms = int(self.timeout * 1000)
        while True:
            result = self.redis.xread(
                {RESP_STREAM: last_id}, block=timeout_ms, count=10
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


class RFSwitchProxy(PicoProxy):
    """
    Redis-backed proxy for an RF switch managed by PicoManager.

    Drop-in replacement for :class:`PicoRFSwitch` — exposes the same
    ``.switch()`` method and ``.path_str`` / ``.paths`` attributes so
    that ``cmt_vna.VNA`` can use it without modification.
    """

    path_str = PicoRFSwitch.path_str

    @staticmethod
    def rbin(s):
        return int(s[::-1], 2)

    @property
    def paths(self):
        return {k: self.rbin(v) for k, v in self.path_str.items()}

    def switch(self, state: str) -> bool:
        """
        Set RF switch state via PicoManager.

        Returns ``True`` on success, ``False`` if the device is
        unavailable (no-op).

        Raises
        ------
        ValueError
            If *state* is not a valid switch path.
        """
        if state not in self.path_str:
            raise ValueError(
                f"Invalid switch state '{state}'. "
                f"Valid states: {list(self.path_str.keys())}"
            )
        result = self.send_command("switch", state=state)
        if result is None:
            return False
        return True
