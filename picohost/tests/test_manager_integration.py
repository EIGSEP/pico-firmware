"""
Integration tests: PicoManager with emulator-backed Dummy devices.

Unlike test_manager.py (which tests method-level routing and response
formatting) and test_emulator_integration.py (which tests individual
devices directly), these tests verify the full pipeline:

    Redis command -> PicoManager -> DummyDevice -> MockSerial ->
    Emulator -> state change -> reader thread -> redis_handler ->
    MockRedis.add_metadata()

The enhanced MockRedis here captures ``add_metadata`` calls and supports
blocking ``xread`` so that ``cmd_loop`` can pick up injected commands.
"""

import json
import threading
import time

import pytest

from conftest import wait_for_condition, wait_for_settle
from picohost.manager import (
    APP_IDS,
    CMD_STREAM,
    CONFIG_HASH,
    HEALTH_HASH,
    PICOS_SET,
    RESP_STREAM,
    PicoManager,
)
from picohost.testing import (
    DummyPicoMotor,
    DummyPicoPeltier,
    DummyPicoRFSwitch,
)

CADENCE_MS = 50  # matches DummyPico* EMULATOR_CADENCE_MS


# --- Enhanced MockRedis ------------------------------------------------------


class MockRedis:
    """
    In-process Redis substitute with metadata capture and blocking xread.

    Extends the minimal stub pattern from test_manager.py with two
    capabilities needed for integration testing:

    * ``_metadata_log`` captures every ``add_metadata(name, data)`` call
      so tests can assert that emulator status flows through the reader
      thread's ``redis_handler``.

    * ``xread`` supports blocking via ``threading.Event`` so that
      ``cmd_loop()`` can pick up commands injected by tests.
    """

    def __init__(self):
        self._sets = {}
        self._hashes = {}
        self._keys = {}
        self._streams = {}
        self._metadata_log = []
        self._xread_event = threading.Event()
        # PicoManager._redis() returns self.r if it exists, else self.
        self.r = self

    # -- redis_handler entry point --

    def add_metadata(self, name, data):
        self._metadata_log.append((name, dict(data)))

    # -- sets --

    def sadd(self, key, *values):
        self._sets.setdefault(key, set()).update(values)

    def srem(self, key, *values):
        if key in self._sets:
            self._sets[key] -= set(values)

    def smembers(self, key):
        return set(self._sets.get(key, set()))

    # -- hashes --

    def hset(self, name, key=None, value=None, mapping=None):
        self._hashes.setdefault(name, {})
        if key is not None:
            self._hashes[name][key] = value
        if mapping:
            self._hashes[name].update(mapping)

    def hget(self, name, key):
        return self._hashes.get(name, {}).get(key)

    def hgetall(self, name):
        return dict(self._hashes.get(name, {}))

    # -- keys --

    def set(self, key, value, ex=None):
        self._keys[key] = value

    def get(self, key):
        return self._keys.get(key)

    def delete(self, *keys):
        for key in keys:
            self._keys.pop(key, None)

    # -- streams --

    def xadd(self, stream, fields, maxlen=None):
        msgs = self._streams.setdefault(stream, [])
        msg_id = f"{len(msgs)}-0"
        msgs.append((msg_id, fields))
        self._xread_event.set()
        return msg_id

    def xread(self, streams, block=None, count=None):
        """Return messages newer than the requested ID.

        When ``last_id == "$"`` and ``block > 0``, waits for new messages
        via ``_xread_event`` (set by ``xadd``).
        """
        for stream_name, last_id in streams.items():
            msgs = self._streams.get(stream_name, [])

            if last_id == "$":
                # "$" means only messages added after this call.
                snapshot = len(msgs)
                if block and block > 0:
                    self._xread_event.wait(timeout=min(block / 1000.0, 0.5))
                    self._xread_event.clear()
                    msgs = self._streams.get(stream_name, [])
                    new_msgs = msgs[snapshot:]
                else:
                    new_msgs = []
            else:
                # Return messages after last_id.
                new_msgs = []
                for i, (mid, _) in enumerate(msgs):
                    if mid == last_id:
                        new_msgs = msgs[i + 1 :]
                        break

            if new_msgs:
                if count:
                    new_msgs = new_msgs[:count]
                return [(stream_name, new_msgs)]
        return []


# --- helpers -----------------------------------------------------------------


def _attach(mgr, name, dummy_cls):
    """
    Build a Dummy* device, register it in the manager, and mark it
    as published in Redis -- same effect as ``discover()``.
    """
    pico = dummy_cls("/dev/dummy", eig_redis=mgr.eig_redis, name=name)
    mgr.picos[name] = pico
    r = mgr._redis()
    r.sadd(PICOS_SET, name)
    r.hset(
        CONFIG_HASH,
        name,
        json.dumps(
            {
                "port": "/dev/dummy",
                "app_id": APP_IDS.get(name, -1),
                "usb_serial": "DUMMY",
            }
        ),
    )
    return pico


def _metadata_names(mock_redis):
    """Set of sensor names that have appeared in add_metadata calls."""
    return {name for name, _ in mock_redis._metadata_log}


@pytest.fixture
def mgr():
    """PicoManager wired to an enhanced MockRedis."""
    m = PicoManager(MockRedis())
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
    """Emulator status flows through redis_handler into add_metadata()."""

    def test_add_metadata_receives_status(self, mgr):
        mock = mgr.eig_redis
        _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        wait_for_condition(
            lambda: len(mock._metadata_log) > 0,
            cadence_ms=CADENCE_MS,
        )
        names = _metadata_names(mock)
        assert "rfswitch" in names
        _, data = next(
            (n, d) for n, d in mock._metadata_log if n == "rfswitch"
        )
        assert "sensor_name" in data
        assert "sw_state" in data

    def test_multiple_devices_publish_independently(self, mgr):
        mock = mgr.eig_redis
        _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        _attach(mgr, "motor", DummyPicoMotor)
        wait_for_condition(
            lambda: _metadata_names(mock) >= {"rfswitch", "motor"},
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

    def test_live_device_reported_healthy(self, mgr):
        pico = _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        wait_for_condition(
            lambda: pico.last_status_time is not None,
            cadence_ms=CADENCE_MS,
        )
        mgr._check_health()
        r = mgr._redis()
        health = json.loads(r.hget(HEALTH_HASH, "rfswitch"))
        assert health["connected"] is True
        assert health["last_seen"] > 0
        assert health["app_id"] == APP_IDS["rfswitch"]

    def test_health_reflects_actual_status_time(self, mgr):
        pico = _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        wait_for_condition(
            lambda: pico.last_status_time is not None,
            cadence_ms=CADENCE_MS,
        )
        mgr._check_health()
        r = mgr._redis()
        health = json.loads(r.hget(HEALTH_HASH, "rfswitch"))
        assert health["last_seen"] == pico.last_status_time


# --- TestCommandRelay -------------------------------------------------------


class TestCommandRelay:
    """_process_command() produces real emulator state changes."""

    def test_rfswitch_state_changes(self, mgr):
        pico = _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        wait_for_condition(
            lambda: pico.last_status.get("sensor_name") is not None,
            cadence_ms=CADENCE_MS,
        )
        r = mgr._redis()
        before = pico.last_status.get("sw_state")
        mgr._process_command(
            r,
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
        _, resp = r._streams[RESP_STREAM][0]
        assert resp["status"] == "ok"

    def test_motor_moves_to_target(self, mgr):
        pico = _attach(mgr, "motor", DummyPicoMotor)
        wait_for_condition(
            lambda: pico.last_status_time is not None,
            cadence_ms=CADENCE_MS,
        )
        r = mgr._redis()
        mgr._process_command(
            r,
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
        r = mgr._redis()
        mgr._process_command(
            r,
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
        # Only verify the target was set, not convergence.
        wait_for_condition(
            lambda: pico.last_status.get("LNA_T_target") == 30.0,
            cadence_ms=CADENCE_MS,
            max_cycles=10,
        )
        _, resp = r._streams[RESP_STREAM][0]
        assert resp["status"] == "ok"

    def test_raw_command_rejected(self, mgr):
        pico = _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        wait_for_condition(
            lambda: pico.last_status.get("sensor_name") is not None,
            cadence_ms=CADENCE_MS,
        )
        r = mgr._redis()
        mgr._process_command(
            r,
            "1-0",
            {
                "target": "rfswitch",
                "cmd": json.dumps({"sw_state": 1}),
                "source": "test",
            },
        )
        _, resp = r._streams[RESP_STREAM][0]
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

    def test_stream_message_processed(self, mgr):
        pico = _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        wait_for_condition(
            lambda: pico.last_status.get("sensor_name") is not None,
            cadence_ms=CADENCE_MS,
        )
        self._start_cmd_loop(mgr)
        r = mgr._redis()
        before = pico.last_status.get("sw_state")
        r.xadd(
            CMD_STREAM,
            {
                "target": "rfswitch",
                "cmd": json.dumps({"action": "switch", "state": "VNAO"}),
                "source": "e2e_test",
            },
        )
        # Wait for response in RESP_STREAM.
        wait_for_condition(
            lambda: len(r._streams.get(RESP_STREAM, [])) > 0,
            cadence_ms=CADENCE_MS,
            max_cycles=40,
        )
        _, resp = r._streams[RESP_STREAM][0]
        assert resp["status"] == "ok"
        assert resp["target"] == "rfswitch"
        # Verify emulator state actually changed.
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
        r = mgr._redis()
        r.xadd(
            CMD_STREAM,
            {
                "target": "rfswitch",
                "cmd": json.dumps({"action": "switch", "state": "VNAO"}),
                "source": "test",
            },
        )
        r.xadd(
            CMD_STREAM,
            {
                "target": "rfswitch",
                "cmd": json.dumps({"action": "switch", "state": "RFANT"}),
                "source": "test",
            },
        )
        wait_for_condition(
            lambda: len(r._streams.get(RESP_STREAM, [])) >= 2,
            cadence_ms=CADENCE_MS,
            max_cycles=40,
        )
        assert all(
            resp["status"] == "ok" for _, resp in r._streams[RESP_STREAM]
        )
        # Final state should match the last command.
        settled = wait_for_settle(
            lambda: pico.last_status.get("sw_state"),
            cadence_ms=CADENCE_MS,
            max_cycles=10,
        )
        assert settled == pico.paths["RFANT"]

    def test_error_for_unknown_target(self, mgr):
        self._start_cmd_loop(mgr)
        r = mgr._redis()
        r.xadd(
            CMD_STREAM,
            {
                "target": "nonexistent",
                "cmd": "{}",
                "source": "test",
            },
        )
        wait_for_condition(
            lambda: len(r._streams.get(RESP_STREAM, [])) > 0,
            cadence_ms=CADENCE_MS,
            max_cycles=40,
        )
        _, resp = r._streams[RESP_STREAM][0]
        assert resp["status"] == "error"
        assert "unknown target" in json.loads(resp["data"])["error"]


# --- TestDiscoverIntegration -------------------------------------------------


class TestDiscoverIntegration:
    """discover() with Dummy classes produces live devices."""

    def test_discover_creates_live_device(self, mgr, monkeypatch, tmp_path):
        import picohost.manager as mgr_mod

        cfg = tmp_path / "pico_config.json"
        cfg.write_text(
            json.dumps(
                [
                    {
                        "app_id": 5,
                        "port": "/dev/dummy",
                        "usb_serial": "INT123",
                    },
                ]
            )
        )
        mgr.config_file = cfg
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
        # Status should also flow to add_metadata.
        mock = mgr.eig_redis
        wait_for_condition(
            lambda: "rfswitch" in _metadata_names(mock),
            cadence_ms=CADENCE_MS,
        )
