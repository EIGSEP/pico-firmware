"""
Tests for picohost.manager.PicoManager.

Uses the existing emulator-backed Dummy* devices from picohost.testing
plus ``eigsep_redis.testing.DummyTransport`` (fakeredis-backed) so the
manager talks to a real in-process Redis without a live server.
"""

import json

import pytest
from eigsep_redis import HeartbeatReader
from eigsep_redis.testing import DummyTransport

from picohost.buses import PicoConfigStore
from picohost.keys import (
    PICO_CMD_STREAM,
    PICO_CONFIG_KEY,
    PICO_RESP_STREAM,
    pico_claim_key,
    pico_heartbeat_name,
)
from picohost.manager import (
    APP_IDS,
    APP_NAMES,
    HEARTBEAT_TTL,
    PICO_CLASSES,
    PicoManager,
    _BLOCKED_ACTIONS,
)
from picohost.testing import (
    DummyPicoMotor,
    DummyPicoPeltier,
    DummyPicoRFSwitch,
)


# --- helpers --------------------------------------------------------------


def _attach(mgr, name, dummy_cls):
    """Build a Dummy device and register it the way ``discover()`` would."""
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
    """Return the most recent response dict written to ``PICO_RESP_STREAM``."""
    entries = transport.r.xrange(PICO_RESP_STREAM)
    assert entries, "expected at least one response entry"
    _, fields = entries[-1]
    return {k.decode(): v.decode() for k, v in fields.items()}


def _all_responses(transport):
    """Return every response dict written to ``PICO_RESP_STREAM``."""
    return [
        {k.decode(): v.decode() for k, v in fields.items()}
        for _, fields in transport.r.xrange(PICO_RESP_STREAM)
    ]


@pytest.fixture
def mgr():
    """A bare PicoManager wired to a fresh in-memory transport."""
    m = PicoManager(DummyTransport())
    yield m
    for pico in list(m.picos.values()):
        try:
            pico.disconnect()
        except Exception:
            pass
    m.picos.clear()


# --- mapping sanity -------------------------------------------------------


class TestAppMappings:
    def test_app_names_match_pico_multi_h(self):
        # Locked-in mapping; should fail loudly if pico_multi.h drifts.
        assert APP_NAMES == {
            0: "motor",
            1: "tempctrl",
            2: "potmon",
            3: "imu_el",
            4: "lidar",
            5: "rfswitch",
            6: "imu_az",
        }

    def test_app_ids_inverse(self):
        for app_id, name in APP_NAMES.items():
            assert APP_IDS[name] == app_id

    def test_pico_classes_cover_specialized_apps(self):
        for name in APP_NAMES.values():
            assert name in PICO_CLASSES


# --- _route_command -------------------------------------------------------


class TestRouteCommand:
    def test_missing_action_rejected(self, mgr):
        pico = _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        with pytest.raises(ValueError, match="'action' is required"):
            mgr._route_command(pico, "rfswitch", {"some_key": "value"})

    def test_named_action_invokes_method(self, mgr):
        pico = _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        result = mgr._route_command(
            pico, "rfswitch", {"action": "switch", "state": "RFANT"}
        )
        assert result["action"] == "switch"
        assert result["result"] is None

    @pytest.mark.parametrize("action", sorted(_BLOCKED_ACTIONS))
    def test_blocked_actions_rejected(self, mgr, action):
        pico = _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        with pytest.raises(ValueError, match="not allowed"):
            mgr._route_command(pico, "rfswitch", {"action": action})

    def test_private_method_rejected(self, mgr):
        pico = _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        with pytest.raises(ValueError, match="not allowed"):
            mgr._route_command(
                pico, "rfswitch", {"action": "_reader_thread_func"}
            )

    def test_unknown_action_rejected(self, mgr):
        pico = _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        with pytest.raises(ValueError, match="Unknown action"):
            mgr._route_command(
                pico, "rfswitch", {"action": "definitely_not_a_method"}
            )

    def test_peltier_set_temperature(self, mgr):
        pico = _attach(mgr, "tempctrl", DummyPicoPeltier)
        result = mgr._route_command(
            pico,
            "tempctrl",
            {"action": "set_temperature", "T_LNA": 25.0, "LNA_hyst": 0.5},
        )
        assert result["action"] == "set_temperature"
        assert result["result"] is None


# --- _process_command -----------------------------------------------------


class TestProcessCommand:
    def test_valid_command_publishes_ok_response(self, mgr):
        _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        mgr._process_command(
            "1-0",
            {
                "target": "rfswitch",
                "cmd": json.dumps({"action": "switch", "state": "RFANT"}),
                "source": "test",
            },
        )
        resp = _last_response(mgr.transport)
        assert resp["status"] == "ok"
        assert resp["source"] == "test"

    def test_unknown_target_publishes_error(self, mgr):
        mgr._process_command(
            "1-0",
            {
                "target": "ghost",
                "cmd": "{}",
                "source": "test",
            },
        )
        resp = _last_response(mgr.transport)
        assert resp["status"] == "error"
        assert "unknown target" in json.loads(resp["data"])["error"]

    def test_invalid_json_publishes_error(self, mgr):
        _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        mgr._process_command(
            "1-0",
            {
                "target": "rfswitch",
                "cmd": "not json",
                "source": "test",
            },
        )
        resp = _last_response(mgr.transport)
        assert resp["status"] == "error"

    def test_bytes_fields_decoded(self, mgr):
        """Real Redis returns bytes; the manager must decode both keys
        and values before parsing."""
        _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        mgr._process_command(
            b"1-0",
            {
                b"target": b"rfswitch",
                b"cmd": json.dumps(
                    {"action": "switch", "state": "RFANT"}
                ).encode(),
                b"source": b"test",
            },
        )
        resp = _last_response(mgr.transport)
        assert resp["status"] == "ok"


# --- soft claims ----------------------------------------------------------


class TestClaims:
    def test_claim_sets_owner(self, mgr):
        _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        mgr._process_command(
            "1-0",
            {
                "target": "rfswitch",
                "cmd": json.dumps({"action": "claim", "ttl": 60}),
                "source": "switch_loop",
            },
        )
        assert (
            mgr.transport.r.get(pico_claim_key("rfswitch")).decode()
            == "switch_loop"
        )

    def test_release_clears_owner(self, mgr):
        _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        mgr.transport.r.set(pico_claim_key("rfswitch"), "switch_loop")
        mgr._process_command(
            "1-0",
            {
                "target": "rfswitch",
                "cmd": json.dumps({"action": "release"}),
                "source": "switch_loop",
            },
        )
        assert mgr.transport.r.get(pico_claim_key("rfswitch")) is None

    def test_override_warns_but_allows(self, mgr):
        _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        mgr.transport.r.set(pico_claim_key("rfswitch"), "switch_loop")
        mgr._process_command(
            "1-0",
            {
                "target": "rfswitch",
                "cmd": json.dumps({"action": "switch", "state": "RFANT"}),
                "source": "interactive",
            },
        )
        resp = _last_response(mgr.transport)
        assert resp["status"] == "ok"
        assert "switch_loop" in resp["warning"]

    def test_owner_no_warning(self, mgr):
        _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        mgr.transport.r.set(pico_claim_key("rfswitch"), "switch_loop")
        mgr._process_command(
            "1-0",
            {
                "target": "rfswitch",
                "cmd": json.dumps({"action": "switch", "state": "RFANT"}),
                "source": "switch_loop",
            },
        )
        resp = _last_response(mgr.transport)
        assert "warning" not in resp


# --- lifecycle ------------------------------------------------------------


class TestLifecycle:
    def test_start_stop(self, mgr):
        _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        mgr.start()
        assert mgr._running
        mgr.stop()
        assert not mgr._running
        assert mgr.picos == {}

    def test_stop_disconnects_devices(self, mgr):
        switch = _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        motor = _attach(mgr, "motor", DummyPicoMotor)
        mgr.stop()
        assert switch.ser is None
        assert motor.ser is None

    def test_stop_marks_heartbeats_dead(self, mgr):
        _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        mgr.stop()
        hb_reader = HeartbeatReader(
            mgr.transport, name=pico_heartbeat_name("rfswitch")
        )
        assert hb_reader.check() is False


# --- discovery ------------------------------------------------------------


class TestDiscover:
    def test_empty_redis_is_a_noop_without_uf2(self, mgr, tmp_path):
        """discover() with empty Redis and no UF2 just produces zero picos."""
        mgr.uf2_path = tmp_path / "nonexistent.uf2"
        mgr.discover()
        assert mgr.picos == {}

    def test_redis_config_instantiates_devices(self, mgr, monkeypatch):
        """discover() reads the PicoConfigStore and builds devices."""
        import picohost.manager as mgr_mod

        monkeypatch.setitem(
            mgr_mod.PICO_CLASSES, "rfswitch", DummyPicoRFSwitch
        )
        PicoConfigStore(mgr.transport).upload(
            [
                {"app_id": 5, "port": "/dev/dummy", "usb_serial": "ABC"},
            ]
        )
        mgr.discover()
        assert "rfswitch" in mgr.picos

    def test_discover_emits_initial_heartbeat(self, mgr, monkeypatch):
        """Each registered device gets an alive heartbeat as soon as it lands."""
        import picohost.manager as mgr_mod

        monkeypatch.setitem(
            mgr_mod.PICO_CLASSES, "rfswitch", DummyPicoRFSwitch
        )
        PicoConfigStore(mgr.transport).upload(
            [
                {"app_id": 5, "port": "/dev/dummy", "usb_serial": "ABC"},
            ]
        )
        mgr.discover()
        hb_reader = HeartbeatReader(
            mgr.transport, name=pico_heartbeat_name("rfswitch")
        )
        assert hb_reader.check() is True

    def test_flash_fallback_on_empty_redis(self, mgr, monkeypatch):
        """discover() falls through to flash when Redis is empty."""
        import picohost.flash_picos as fp_mod
        import picohost.manager as mgr_mod

        monkeypatch.setitem(
            mgr_mod.PICO_CLASSES, "rfswitch", DummyPicoRFSwitch
        )
        monkeypatch.setattr(
            fp_mod,
            "flash_and_discover",
            lambda **kw: [
                {"app_id": 5, "port": "/dev/dummy", "usb_serial": "X"},
            ],
        )
        mgr.discover()
        assert "rfswitch" in mgr.picos
        # Result is persisted back into Redis
        stored = PicoConfigStore(mgr.transport).get()
        assert stored == [
            {"app_id": 5, "port": "/dev/dummy", "usb_serial": "X"},
        ]

    def test_flash_fallback_uf2_missing_is_noop(self, mgr, tmp_path):
        mgr.uf2_path = tmp_path / "nonexistent.uf2"
        mgr._try_flash_discover()
        assert mgr.picos == {}


# --- manager commands -----------------------------------------------------


class TestManagerCommand:
    def test_rediscover_clears_and_reloads(self, mgr, monkeypatch):
        import picohost.manager as mgr_mod

        monkeypatch.setitem(
            mgr_mod.PICO_CLASSES, "rfswitch", DummyPicoRFSwitch
        )
        _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        assert "rfswitch" in mgr.picos

        # Stage Redis config for the rediscover path to pick up
        PicoConfigStore(mgr.transport).upload(
            [
                {"app_id": 5, "port": "/dev/dummy", "usb_serial": "X"},
            ]
        )

        mgr._process_command(
            "1-0",
            {
                "target": "manager",
                "cmd": json.dumps({"action": "rediscover"}),
                "source": "test",
            },
        )
        resp = _last_response(mgr.transport)
        assert resp["status"] == "ok"
        data = json.loads(resp["data"])
        assert "rfswitch" in data["devices"]
        assert data["count"] == 1

    def test_unknown_manager_action_returns_error(self, mgr):
        mgr._process_command(
            "1-0",
            {
                "target": "manager",
                "cmd": json.dumps({"action": "nope"}),
                "source": "test",
            },
        )
        resp = _last_response(mgr.transport)
        assert resp["status"] == "error"
        assert "unknown manager action" in json.loads(resp["data"])["error"]


# --- reconnect & timeout regressions --------------------------------------


class TestReconnectHook:
    def test_motor_on_reconnect_replays_delay(self, mgr):
        motor = _attach(mgr, "motor", DummyPicoMotor)
        assert motor._delay_kwargs is not None

        calls = []
        original = motor.set_delay

        def spy(**kwargs):
            calls.append(kwargs)
            return original(**kwargs)

        motor.set_delay = spy  # type: ignore[method-assign]
        motor.on_reconnect()
        assert len(calls) == 1
        assert calls[0] == motor._delay_kwargs

    def test_port_rediscovery_updates_port(self, mgr, monkeypatch):
        """When usb_serial maps to a new port, reconnect uses it."""
        import picohost.base as base_mod

        switch = _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        switch.usb_serial = "SER_ABC"
        old_port = switch.port

        monkeypatch.setattr(
            base_mod,
            "find_pico_ports",
            lambda: {"/dev/ttyACM5": "SER_ABC"},
        )
        switch._rediscover_port()
        assert switch.port == "/dev/ttyACM5"
        assert switch.port != old_port

    def test_port_rediscovery_noop_when_unchanged(self, mgr, monkeypatch):
        import picohost.base as base_mod

        switch = _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        switch.usb_serial = "SER_ABC"
        switch.port = "/dev/ttyACM0"

        monkeypatch.setattr(
            base_mod,
            "find_pico_ports",
            lambda: {"/dev/ttyACM0": "SER_ABC"},
        )
        switch._rediscover_port()
        assert switch.port == "/dev/ttyACM0"

    def test_port_rediscovery_noop_without_usb_serial(self, mgr):
        switch = _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        original_port = switch.port
        switch.usb_serial = ""
        switch._rediscover_port()
        assert switch.port == original_port


# --- module-level constants -----------------------------------------------


def test_blocked_actions_includes_lifecycle():
    for action in ("connect", "disconnect", "reconnect"):
        assert action in _BLOCKED_ACTIONS


def test_cmd_stream_constant_unchanged():
    # Other consumers (eigsep_observing) read from this stream by name.
    assert PICO_CMD_STREAM == "stream:pico_cmd"
    assert PICO_RESP_STREAM == "stream:pico_resp"
    assert PICO_CONFIG_KEY == "pico_config"
