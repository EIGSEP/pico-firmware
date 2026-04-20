"""
Integration tests: PicoManager with emulator-backed Dummy devices.

Unlike test_manager.py (which tests method-level routing and response
formatting) and test_emulator_integration.py (which tests individual
devices directly), these tests verify the full pipeline:

    Redis command -> PicoManager -> DummyDevice -> MockSerial ->
    Emulator -> state change -> reader thread -> MetadataWriter.add()

The transport here is a fakeredis-backed ``DummyTransport``, so
``cmd_loop`` really blocks on ``xread`` and ``MetadataWriter.add``
really writes to an in-process Redis snapshot.
"""

import json
import threading
import time

import pytest
from conftest import wait_for_condition, wait_for_settle
from eigsep_redis import HeartbeatReader, MetadataSnapshotReader
from eigsep_redis.testing import DummyTransport

from picohost.buses import PicoConfigStore
from picohost.keys import (
    PICO_CMD_STREAM,
    PICO_RESP_STREAM,
    pico_heartbeat_name,
)
from picohost.manager import HEARTBEAT_TTL, PicoManager
from picohost.testing import (
    DummyPicoMotor,
    DummyPicoPeltier,
    DummyPicoRFSwitch,
)

CADENCE_MS = 50  # matches DummyPico* EMULATOR_CADENCE_MS


# --- helpers -----------------------------------------------------------------


def _attach(mgr, name, dummy_cls):
    """Build a Dummy device, register it, and emit initial heartbeat."""
    from eigsep_redis import HeartbeatWriter

    pico = dummy_cls(
        "/dev/dummy",
        metadata_writer=mgr._metadata_writer,
        name=name,
    )
    mgr.picos[name] = pico
    mgr._heartbeats[name] = HeartbeatWriter(
        mgr.transport, name=pico_heartbeat_name(name)
    )
    mgr._heartbeats[name].set(ex=HEARTBEAT_TTL, alive=True)
    return pico


def _last_response(transport):
    entries = transport.r.xrange(PICO_RESP_STREAM)
    assert entries, "expected at least one response entry"
    _, fields = entries[-1]
    return {k.decode(): v.decode() for k, v in fields.items()}


def _all_responses(transport):
    return [
        {k.decode(): v.decode() for k, v in fields.items()}
        for _, fields in transport.r.xrange(PICO_RESP_STREAM)
    ]


def _metadata_snapshot(transport):
    return MetadataSnapshotReader(transport)


@pytest.fixture
def mgr():
    """PicoManager wired to a fakeredis-backed transport."""
    m = PicoManager(DummyTransport())
    yield m
    m._running = False
    if m._cmd_thread:
        m._cmd_thread.join(timeout=2)
    if m._health_thread:
        m._health_thread.join(timeout=2)
    for pico in list(m.picos.values()):
        try:
            pico.disconnect()
        except Exception:
            pass
    m.picos.clear()


# --- TestStatusPublication ---------------------------------------------------


class TestStatusPublication:
    """Emulator status flows through MetadataWriter into Redis."""

    def test_metadata_snapshot_receives_status(self, mgr):
        _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        snap = _metadata_snapshot(mgr.transport)
        wait_for_condition(
            lambda: "rfswitch" in snap.get(),
            cadence_ms=CADENCE_MS,
        )
        data = snap.get("rfswitch")
        assert data["sensor_name"] == "rfswitch"
        assert "sw_state" in data

    def test_multiple_devices_publish_independently(self, mgr):
        _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        _attach(mgr, "motor", DummyPicoMotor)
        snap = _metadata_snapshot(mgr.transport)
        wait_for_condition(
            lambda: {"rfswitch", "motor"}.issubset(snap.get()),
            cadence_ms=CADENCE_MS,
        )

    def test_last_status_time_kept_fresh(self, mgr):
        pico = _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        wait_for_condition(
            lambda: pico.last_status_time is not None,
            cadence_ms=CADENCE_MS,
        )
        t1 = pico.last_status_time
        time.sleep(CADENCE_MS * 3 / 1000.0)
        t2 = pico.last_status_time
        assert t2 > t1


# --- TestHealthMonitoring ----------------------------------------------------


class TestHealthMonitoring:
    """_check_health() against live emulator-backed devices."""

    def test_live_device_heartbeat_alive(self, mgr):
        pico = _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        wait_for_condition(
            lambda: pico.last_status_time is not None,
            cadence_ms=CADENCE_MS,
        )
        mgr._check_health()
        hb_reader = HeartbeatReader(
            mgr.transport, name=pico_heartbeat_name("rfswitch")
        )
        assert hb_reader.check() is True

    def test_stopped_device_heartbeat_dead(self, mgr, monkeypatch):
        pico = _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        wait_for_condition(
            lambda: pico.last_status_time is not None,
            cadence_ms=CADENCE_MS,
        )
        # Simulate a silent, unreachable device: disconnect, backdate
        # last_status_time past HEALTH_TIMEOUT, and force reconnect()
        # to fail so _check_health gives up and writes alive=False.
        pico.disconnect()
        pico.last_status_time = time.time() - 60
        monkeypatch.setattr(pico, "reconnect", lambda: False)
        mgr._check_health()
        hb_reader = HeartbeatReader(
            mgr.transport, name=pico_heartbeat_name("rfswitch")
        )
        assert hb_reader.check() is False


# --- TestCommandRelay -------------------------------------------------------


class TestCommandRelay:
    """_process_command() produces real emulator state changes."""

    def test_rfswitch_state_changes(self, mgr):
        pico = _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        wait_for_condition(
            lambda: pico.last_status.get("sensor_name") is not None,
            cadence_ms=CADENCE_MS,
        )
        before = pico.last_status.get("sw_state")
        mgr._process_command(
            "1-0",
            {
                "target": "rfswitch",
                "cmd": json.dumps({"action": "switch", "state": "VNAO"}),
                "source": "test",
            },
        )
        settled = wait_for_settle(
            lambda: pico.last_status.get("sw_state"),
            initial=before,
            cadence_ms=CADENCE_MS,
            max_cycles=10,
        )
        assert settled == pico.paths["VNAO"]
        resp = _last_response(mgr.transport)
        assert resp["status"] == "ok"

    def test_motor_moves_to_target(self, mgr):
        pico = _attach(mgr, "motor", DummyPicoMotor)
        wait_for_condition(
            lambda: pico.last_status_time is not None,
            cadence_ms=CADENCE_MS,
        )
        mgr._process_command(
            "1-0",
            {
                "target": "motor",
                "cmd": json.dumps(
                    {
                        "action": "motor_command",
                        "az_set_target_pos": 300,
                    }
                ),
                "source": "test",
            },
        )
        # 300 steps / 60 steps_per_op = 5 ops + margin
        settled = wait_for_settle(
            lambda: pico.last_status.get("az_pos"),
            initial=0,
            cadence_ms=CADENCE_MS,
            max_cycles=20,
        )
        assert settled == 300

    def test_peltier_target_set(self, mgr):
        pico = _attach(mgr, "tempctrl", DummyPicoPeltier)
        wait_for_condition(
            lambda: pico.last_status_time is not None,
            cadence_ms=CADENCE_MS,
        )
        mgr._process_command(
            "1-0",
            {
                "target": "tempctrl",
                "cmd": json.dumps(
                    {
                        "action": "set_temperature",
                        "T_LNA": 30.0,
                        "LNA_hyst": 0.5,
                    }
                ),
                "source": "test",
            },
        )
        wait_for_condition(
            lambda: pico.last_status.get("LNA_T_target") == 30.0,
            cadence_ms=CADENCE_MS,
            max_cycles=10,
        )
        resp = _last_response(mgr.transport)
        assert resp["status"] == "ok"

    def test_raw_command_rejected(self, mgr):
        pico = _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        wait_for_condition(
            lambda: pico.last_status.get("sensor_name") is not None,
            cadence_ms=CADENCE_MS,
        )
        mgr._process_command(
            "1-0",
            {
                "target": "rfswitch",
                "cmd": json.dumps({"sw_state": 1}),
                "source": "test",
            },
        )
        resp = _last_response(mgr.transport)
        assert resp["status"] == "error"
        assert "action" in resp["data"]


# --- TestCmdLoopEndToEnd -----------------------------------------------------


class TestCmdLoopEndToEnd:
    """Full cmd_loop thread picks up commands from stream."""

    def _start_cmd_loop(self, mgr):
        """Start only the cmd thread (not the full manager)."""
        mgr._running = True
        mgr._cmd_thread = threading.Thread(
            target=mgr.cmd_loop,
            daemon=True,
            name="cmd-test",
        )
        mgr._cmd_thread.start()

    def _count_responses(self, mgr):
        return mgr.transport.r.xlen(PICO_RESP_STREAM)

    def test_stream_message_processed(self, mgr):
        pico = _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        wait_for_condition(
            lambda: pico.last_status.get("sensor_name") is not None,
            cadence_ms=CADENCE_MS,
        )
        self._start_cmd_loop(mgr)
        before = pico.last_status.get("sw_state")
        mgr.transport.r.xadd(
            PICO_CMD_STREAM,
            {
                "target": "rfswitch",
                "cmd": json.dumps({"action": "switch", "state": "VNAO"}),
                "source": "e2e_test",
            },
        )
        wait_for_condition(
            lambda: self._count_responses(mgr) > 0,
            cadence_ms=CADENCE_MS,
            max_cycles=40,
        )
        resp = _last_response(mgr.transport)
        assert resp["status"] == "ok"
        assert resp["target"] == "rfswitch"
        settled = wait_for_settle(
            lambda: pico.last_status.get("sw_state"),
            initial=before,
            cadence_ms=CADENCE_MS,
            max_cycles=10,
        )
        assert settled == pico.paths["VNAO"]

    def test_multiple_commands_sequenced(self, mgr):
        pico = _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        wait_for_condition(
            lambda: pico.last_status.get("sensor_name") is not None,
            cadence_ms=CADENCE_MS,
        )
        self._start_cmd_loop(mgr)
        mgr.transport.r.xadd(
            PICO_CMD_STREAM,
            {
                "target": "rfswitch",
                "cmd": json.dumps({"action": "switch", "state": "VNAO"}),
                "source": "test",
            },
        )
        mgr.transport.r.xadd(
            PICO_CMD_STREAM,
            {
                "target": "rfswitch",
                "cmd": json.dumps({"action": "switch", "state": "RFANT"}),
                "source": "test",
            },
        )
        wait_for_condition(
            lambda: self._count_responses(mgr) >= 2,
            cadence_ms=CADENCE_MS,
            max_cycles=40,
        )
        assert all(
            resp["status"] == "ok" for resp in _all_responses(mgr.transport)
        )
        settled = wait_for_settle(
            lambda: pico.last_status.get("sw_state"),
            cadence_ms=CADENCE_MS,
            max_cycles=10,
        )
        assert settled == pico.paths["RFANT"]

    def test_error_for_unknown_target(self, mgr):
        self._start_cmd_loop(mgr)
        # Give cmd_loop a moment to reach its first blocking xread so
        # the $ cursor is in place before we xadd (otherwise the add
        # races ahead of the read and the message is missed).
        time.sleep(0.1)
        mgr.transport.r.xadd(
            PICO_CMD_STREAM,
            {
                "target": "nonexistent",
                "cmd": "{}",
                "source": "test",
            },
        )
        wait_for_condition(
            lambda: self._count_responses(mgr) > 0,
            cadence_ms=CADENCE_MS,
            max_cycles=40,
        )
        resp = _last_response(mgr.transport)
        assert resp["status"] == "error"
        assert "unknown target" in json.loads(resp["data"])["error"]


# --- TestDiscoverIntegration -------------------------------------------------


class TestDiscoverIntegration:
    """discover() with Dummy classes produces live devices."""

    def test_discover_creates_live_device(self, mgr, monkeypatch):
        import picohost.manager as mgr_mod

        PicoConfigStore(mgr.transport).upload(
            [
                {
                    "app_id": 5,
                    "port": "/dev/dummy",
                    "usb_serial": "INT123",
                },
            ]
        )
        monkeypatch.setitem(
            mgr_mod.PICO_CLASSES,
            "rfswitch",
            DummyPicoRFSwitch,
        )
        mgr.discover()
        assert "rfswitch" in mgr.picos
        pico = mgr.picos["rfswitch"]
        assert pico.is_connected
        wait_for_condition(
            lambda: pico.last_status.get("sensor_name") == "rfswitch",
            cadence_ms=CADENCE_MS,
        )
        # Status flows into the metadata snapshot.
        snap = _metadata_snapshot(mgr.transport)
        wait_for_condition(
            lambda: "rfswitch" in snap.get(),
            cadence_ms=CADENCE_MS,
        )
