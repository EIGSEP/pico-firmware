"""
Tests for picohost.proxy — Redis-backed device proxies.

Uses the fakeredis-backed ``DummyTransport`` from eigsep_redis so the
full command/response round-trip exercises real xadd/xread semantics
with a live PicoManager.cmd_loop running in a background thread.
"""

import pytest
from eigsep_redis import HeartbeatWriter
from eigsep_redis.testing import DummyTransport

from picohost.keys import (
    PICO_RESP_STREAM,
    pico_heartbeat_name,
)
from picohost.manager import HEARTBEAT_TTL, PicoManager
from picohost.proxy import PicoProxy
from picohost.testing import DummyPicoRFSwitch


# --- fixtures ---------------------------------------------------------------


@pytest.fixture
def transport():
    return DummyTransport()


@pytest.fixture
def mgr(transport):
    """PicoManager with a DummyPicoRFSwitch, cmd_loop running."""
    m = PicoManager(transport)
    pico = DummyPicoRFSwitch(
        "/dev/dummy", metadata_writer=m._metadata_writer, name="rfswitch"
    )
    m.picos["rfswitch"] = pico
    m._heartbeats["rfswitch"] = HeartbeatWriter(
        transport, name=pico_heartbeat_name("rfswitch")
    )
    m._heartbeats["rfswitch"].set(ex=HEARTBEAT_TTL, alive=True)
    m.start()
    yield m
    m.stop()


@pytest.fixture
def sw(transport, mgr):
    """PicoProxy for the rfswitch device wired to the running manager."""
    return PicoProxy("rfswitch", transport, source="test", timeout=5.0)


# --- PicoProxy -------------------------------------------------------------


class TestPicoProxy:

    def test_send_command_round_trip(self, sw):
        """send_command routes through PicoManager and returns device result."""
        result = sw.send_command("switch", state="RFANT")
        assert result is not None
        # Manager wraps the device return as {"action": ..., "result": ...}
        assert result["action"] == "switch"

    def test_send_command_unavailable_returns_none(self, transport):
        proxy = PicoProxy("nonexistent", transport, timeout=1.0)
        assert proxy.send_command("switch", state="RFANT") is None

    def test_send_command_invalid_state_raises(self, sw):
        """Device-side validation surfaces as RuntimeError from the proxy."""
        with pytest.raises(RuntimeError, match="Invalid switch state"):
            sw.send_command("switch", state="BOGUS")

    def test_send_command_timeout(self, transport):
        """If manager never responds, proxy raises TimeoutError."""
        # Fake an "alive" device with no manager to route commands
        HeartbeatWriter(
            transport, name=pico_heartbeat_name("orphan")
        ).set(ex=60, alive=True)
        proxy = PicoProxy("orphan", transport, timeout=0.2)
        with pytest.raises(TimeoutError, match="No response"):
            proxy.send_command("switch", state="RFANT")

    def test_send_command_error_raises_runtime(self, transport, mgr):
        """If manager returns an error, proxy raises RuntimeError."""
        proxy = PicoProxy(
            "rfswitch", transport, source="test", timeout=5.0
        )
        with pytest.raises(RuntimeError, match="not allowed"):
            proxy.send_command("disconnect")

    def test_is_available_true(self, sw):
        assert sw.is_available is True

    def test_is_available_false(self, transport):
        proxy = PicoProxy("ghost", transport, timeout=1.0)
        assert proxy.is_available is False


# --- request_id echo -------------------------------------------------------


class TestRequestId:

    def test_response_contains_request_id(self, sw, transport):
        """The manager echoes request_id in the response."""
        sw.send_command("switch", state="RFANT")
        entries = transport.r.xrange(PICO_RESP_STREAM)
        assert entries
        _, fields = entries[-1]
        assert b"request_id" in fields
        assert fields[b"request_id"] != b""

    def test_request_id_unique_per_call(self, sw, transport):
        sw.send_command("switch", state="RFANT")
        sw.send_command("switch", state="RFNON")
        entries = transport.r.xrange(PICO_RESP_STREAM)
        ids = [fields[b"request_id"].decode() for _, fields in entries]
        assert len(set(ids)) == len(ids)
