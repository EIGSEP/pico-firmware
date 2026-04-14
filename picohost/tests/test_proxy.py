"""
Tests for picohost.proxy — Redis-backed device proxies.

Uses an in-test MockRedis with thread-safe stream support so that
PicoManager's cmd_loop thread can process commands in the background
while the proxy reads responses in the foreground.
"""

import json
import threading
import time

import pytest

from picohost.manager import (
    PICOS_SET,
    RESP_STREAM,
    PicoManager,
)
from picohost.proxy import PicoProxy
from picohost.testing import DummyPicoRFSwitch


class MockRedis:
    """
    Thread-safe MockRedis with blocking ``xread`` support.

    Messages use integer-prefixed IDs (``"0-0"``, ``"1-0"``, ...)
    so that ``cmd_loop`` and the proxy can interleave reads.
    """

    def __init__(self):
        self._sets = {}
        self._hashes = {}
        self._keys = {}
        self._streams = {}
        self._counter = 0
        self._cv = threading.Condition()
        self.r = self

    def add_metadata(self, name, data):
        pass

    # -- sets --

    def sadd(self, key, *values):
        with self._cv:
            self._sets.setdefault(key, set()).update(
                v if isinstance(v, str) else v.decode()
                for v in values
            )

    def srem(self, key, *values):
        with self._cv:
            s = self._sets.get(key, set())
            for v in values:
                s.discard(v if isinstance(v, str) else v.decode())

    def smembers(self, key):
        with self._cv:
            return set(self._sets.get(key, set()))

    def sismember(self, key, member):
        with self._cv:
            if isinstance(member, bytes):
                member = member.decode()
            return member in self._sets.get(key, set())

    # -- hashes --

    def hset(self, name, key=None, value=None, mapping=None):
        with self._cv:
            self._hashes.setdefault(name, {})
            if key is not None:
                self._hashes[name][key] = value
            if mapping:
                self._hashes[name].update(mapping)

    def hget(self, name, key):
        with self._cv:
            return self._hashes.get(name, {}).get(key)

    def hgetall(self, name):
        with self._cv:
            return dict(self._hashes.get(name, {}))

    # -- keys --

    def set(self, key, value, ex=None):
        with self._cv:
            self._keys[key] = value

    def get(self, key):
        with self._cv:
            return self._keys.get(key)

    def delete(self, *keys):
        with self._cv:
            for key in keys:
                self._keys.pop(key, None)

    # -- streams (thread-safe with blocking xread) --

    def xadd(self, stream, fields, maxlen=None):
        with self._cv:
            msgs = self._streams.setdefault(stream, [])
            msg_id = f"{self._counter}-0"
            self._counter += 1
            msgs.append((msg_id, fields))
            self._cv.notify_all()
            return msg_id

    def xinfo_stream(self, stream):
        with self._cv:
            msgs = self._streams.get(stream, [])
            if not msgs:
                raise Exception(f"no such key: {stream}")
            return {"last-generated-id": msgs[-1][0]}

    def xread(self, streams, block=None, count=None):
        timeout_s = (block / 1000.0) if block else 0
        deadline = time.time() + timeout_s if block else 0

        with self._cv:
            # Resolve "$" → current stream end
            resolved = {}
            for stream_name, last_id in streams.items():
                if isinstance(last_id, bytes):
                    last_id = last_id.decode()
                if last_id == "$":
                    msgs = self._streams.get(stream_name, [])
                    last_id = msgs[-1][0] if msgs else "-1-0"
                resolved[stream_name] = last_id

            while True:
                result = self._collect(resolved, count)
                if result:
                    return result
                if not block or time.time() >= deadline:
                    return []
                remaining = deadline - time.time()
                if remaining <= 0:
                    return []
                self._cv.wait(timeout=min(remaining, 0.05))

    def _collect(self, resolved, count):
        result = []
        for stream_name, last_id in resolved.items():
            msgs = self._streams.get(stream_name, [])
            new = [
                (mid, f) for mid, f in msgs
                if self._id_gt(mid, last_id)
            ]
            if new:
                key = (
                    stream_name.encode()
                    if isinstance(stream_name, str)
                    else stream_name
                )
                result.append((key, new[:count] if count else new))
        return result

    @staticmethod
    def _id_gt(a, b):
        """Compare Redis stream IDs numerically."""
        try:
            return int(a.split("-")[0]) > int(b.split("-")[0])
        except (ValueError, IndexError):
            return a > b


# --- fixtures ---------------------------------------------------------------


@pytest.fixture
def redis():
    return MockRedis()


@pytest.fixture
def mgr(redis):
    """PicoManager with a DummyPicoRFSwitch, cmd_loop running."""
    m = PicoManager(redis)
    pico = DummyPicoRFSwitch(
        "/dev/dummy", eig_redis=redis, name="rfswitch"
    )
    m.picos["rfswitch"] = pico
    redis.sadd(PICOS_SET, "rfswitch")
    m.start()
    yield m
    m.stop()


@pytest.fixture
def sw(redis, mgr):
    """PicoProxy for the rfswitch device wired to the running manager."""
    return PicoProxy("rfswitch", redis, source="test", timeout=5.0)


# --- PicoProxy -------------------------------------------------------------


class TestPicoProxy:

    def test_send_command_round_trip(self, sw):
        """send_command routes through PicoManager and returns device result."""
        result = sw.send_command("switch", state="RFANT")
        assert result is not None
        # Manager wraps the device return as {"action": ..., "result": ...}
        assert result["action"] == "switch"

    def test_send_command_unavailable_returns_none(self, redis):
        proxy = PicoProxy("nonexistent", redis, timeout=1.0)
        assert proxy.send_command("switch", state="RFANT") is None

    def test_send_command_invalid_state_raises(self, sw):
        """Device-side validation surfaces as RuntimeError from the proxy."""
        with pytest.raises(RuntimeError, match="Invalid switch state"):
            sw.send_command("switch", state="BOGUS")

    def test_send_command_timeout(self, redis):
        """If manager never responds, proxy raises TimeoutError."""
        redis.sadd(PICOS_SET, "orphan")
        proxy = PicoProxy("orphan", redis, timeout=0.2)
        with pytest.raises(TimeoutError, match="No response"):
            proxy.send_command("switch", state="RFANT")

    def test_send_command_error_raises_runtime(self, redis, mgr):
        """If manager returns an error, proxy raises RuntimeError."""
        proxy = PicoProxy("rfswitch", redis, source="test", timeout=5.0)
        with pytest.raises(RuntimeError, match="not allowed"):
            proxy.send_command("disconnect")

    def test_is_available_true(self, sw):
        assert sw.is_available is True

    def test_is_available_false(self, redis):
        proxy = PicoProxy("ghost", redis, timeout=1.0)
        assert proxy.is_available is False

    def test_health_available(self, sw, redis):
        # PicoManager writes health in _check_health, but we can also
        # manually set it to verify the proxy reads it.
        redis.hset(
            "pico_health", "rfswitch",
            json.dumps({"connected": True, "last_seen": 1.0, "app_id": 5}),
        )
        h = sw.health
        assert h["connected"] is True
        assert h["app_id"] == 5

    def test_health_missing(self, redis):
        proxy = PicoProxy("ghost", redis)
        assert proxy.health is None


# --- request_id echo -------------------------------------------------------


class TestRequestId:

    def test_response_contains_request_id(self, sw, redis):
        """The manager echoes request_id in the response."""
        sw.send_command("switch", state="RFANT")
        # Check that RESP_STREAM has at least one message with request_id
        msgs = redis._streams.get(RESP_STREAM, [])
        assert len(msgs) > 0
        _, fields = msgs[-1]
        assert "request_id" in fields
        assert fields["request_id"] != ""

    def test_request_id_unique_per_call(self, sw, redis):
        sw.send_command("switch", state="RFANT")
        sw.send_command("switch", state="RFNON")
        msgs = redis._streams.get(RESP_STREAM, [])
        ids = [f["request_id"] for _, f in msgs]
        assert len(set(ids)) == len(ids)
