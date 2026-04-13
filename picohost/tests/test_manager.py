"""
Tests for picohost.manager.PicoManager.

Uses the existing emulator-backed Dummy* devices from picohost.testing
plus a small in-test MockRedis stub. We deliberately don't pull in
fakeredis here — once the eigsep_redis package lands (Phase 2), tests
can switch to its fakeredis-backed DummyEigsepRedisBase.
"""

import json

import pytest

from picohost.manager import (
    APP_IDS,
    APP_NAMES,
    CMD_STREAM,
    CONFIG_HASH,
    HEALTH_HASH,
    PICO_CLASSES,
    PICOS_SET,
    RESP_STREAM,
    PicoManager,
    _BLOCKED_ACTIONS,
)
from picohost.testing import (
    DummyPicoMotor,
    DummyPicoPeltier,
    DummyPicoRFSwitch,
)


class MockRedis:
    """
    Minimal in-process Redis substitute that implements only the calls
    PicoManager makes plus ``add_metadata`` so the picohost reader
    thread doesn't blow up when it tries to publish status.
    """

    def __init__(self):
        self._sets = {}
        self._hashes = {}
        self._keys = {}
        self._streams = {}
        # PicoManager._redis() returns self.r if it exists, else self.
        self.r = self

    # -- the picohost.base.redis_handler entry point --

    def add_metadata(self, name, data):
        pass

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
        return msg_id

    def xread(self, streams, block=None, count=None):
        # The cmd_loop only uses xread for live polling — tests drive
        # _process_command directly, so an empty result is correct.
        return []


# --- helpers --------------------------------------------------------------


def _attach(mgr, name, dummy_cls):
    """
    Build a Dummy* device, register it under ``name`` in the manager's
    dict, and mark it as published in the manager's Redis state — the
    same effect ``discover()`` would have.
    """
    pico = dummy_cls("/dev/dummy", eig_redis=mgr.eig_redis, name=name)
    mgr.picos[name] = pico
    r = mgr._redis()
    r.sadd(PICOS_SET, name)
    return pico


@pytest.fixture
def mgr():
    """A bare PicoManager wired to a fresh MockRedis."""
    m = PicoManager(MockRedis())
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
        assert result["result"] is True

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
        assert result["result"] is True


# --- _process_command -----------------------------------------------------


class TestProcessCommand:
    def test_valid_command_publishes_ok_response(self, mgr):
        _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        r = mgr._redis()
        mgr._process_command(
            r,
            "1-0",
            {
                "target": "rfswitch",
                "cmd": json.dumps({"action": "switch", "state": "RFANT"}),
                "source": "test",
            },
        )
        assert len(r._streams[RESP_STREAM]) == 1
        _, resp = r._streams[RESP_STREAM][0]
        assert resp["status"] == "ok"
        assert resp["source"] == "test"

    def test_unknown_target_publishes_error(self, mgr):
        r = mgr._redis()
        mgr._process_command(
            r,
            "1-0",
            {
                "target": "ghost",
                "cmd": "{}",
                "source": "test",
            },
        )
        _, resp = r._streams[RESP_STREAM][0]
        assert resp["status"] == "error"
        assert "unknown target" in json.loads(resp["data"])["error"]

    def test_invalid_json_publishes_error(self, mgr):
        _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        r = mgr._redis()
        mgr._process_command(
            r,
            "1-0",
            {
                "target": "rfswitch",
                "cmd": "not json",
                "source": "test",
            },
        )
        _, resp = r._streams[RESP_STREAM][0]
        assert resp["status"] == "error"

    def test_bytes_fields_decoded(self, mgr):
        """
        Real Redis returns bytes; the manager must decode both keys and
        values before parsing.
        """
        _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        r = mgr._redis()
        mgr._process_command(
            r,
            b"1-0",
            {
                b"target": b"rfswitch",
                b"cmd": json.dumps(
                    {"action": "switch", "state": "RFANT"}
                ).encode(),
                b"source": b"test",
            },
        )
        _, resp = r._streams[RESP_STREAM][0]
        assert resp["status"] == "ok"


# --- soft claims ----------------------------------------------------------


class TestClaims:
    def test_claim_sets_owner(self, mgr):
        _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        r = mgr._redis()
        mgr._process_command(
            r,
            "1-0",
            {
                "target": "rfswitch",
                "cmd": json.dumps({"action": "claim", "ttl": 60}),
                "source": "switch_loop",
            },
        )
        assert r.get("pico_claim:rfswitch") == "switch_loop"

    def test_release_clears_owner(self, mgr):
        _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        r = mgr._redis()
        r.set("pico_claim:rfswitch", "switch_loop")
        mgr._process_command(
            r,
            "1-0",
            {
                "target": "rfswitch",
                "cmd": json.dumps({"action": "release"}),
                "source": "switch_loop",
            },
        )
        assert r.get("pico_claim:rfswitch") is None

    def test_override_warns_but_allows(self, mgr):
        _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        r = mgr._redis()
        r.set("pico_claim:rfswitch", "switch_loop")
        mgr._process_command(
            r,
            "1-0",
            {
                "target": "rfswitch",
                "cmd": json.dumps({"action": "switch", "state": "RFANT"}),
                "source": "interactive",
            },
        )
        _, resp = r._streams[RESP_STREAM][0]
        assert resp["status"] == "ok"
        assert "switch_loop" in resp["warning"]

    def test_owner_no_warning(self, mgr):
        _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        r = mgr._redis()
        r.set("pico_claim:rfswitch", "switch_loop")
        mgr._process_command(
            r,
            "1-0",
            {
                "target": "rfswitch",
                "cmd": json.dumps({"action": "switch", "state": "RFANT"}),
                "source": "switch_loop",
            },
        )
        _, resp = r._streams[RESP_STREAM][0]
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


# --- discovery ------------------------------------------------------------


class TestDiscover:
    def test_missing_config_is_a_noop(self, mgr, tmp_path):
        mgr.config_file = tmp_path / "does-not-exist.json"
        mgr.discover()
        assert mgr.picos == {}

    def test_invalid_json_config_is_a_noop(self, mgr, tmp_path):
        cfg = tmp_path / "bad.json"
        cfg.write_text("{not valid json!!")
        mgr.config_file = cfg
        mgr.discover()  # must not raise
        assert mgr.picos == {}

    def test_flash_fallback_on_missing_config(
        self, mgr, monkeypatch, tmp_path
    ):
        """discover() cascades to flash when Redis + file are empty."""
        import picohost.manager as mgr_mod
        import picohost.flash_picos as fp_mod

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

        mgr.config_file = tmp_path / "does-not-exist.json"
        mgr.discover()
        assert "rfswitch" in mgr.picos

    def test_flash_fallback_uf2_missing_is_noop(self, mgr, tmp_path):
        mgr.config_file = tmp_path / "does-not-exist.json"
        mgr.uf2_path = tmp_path / "nonexistent.uf2"
        mgr._try_flash_discover()
        assert mgr.picos == {}

    def test_publishes_devices_to_redis(self, mgr, monkeypatch, tmp_path):
        # Stand the manager up against a config that points "rfswitch"
        # at /dev/dummy, then patch PICO_CLASSES so discover()
        # instantiates the dummy class instead of the real one.
        import picohost.manager as mgr_mod

        cfg = tmp_path / "pico_config.json"
        cfg.write_text(
            json.dumps(
                [
                    {
                        "app_id": 5,
                        "port": "/dev/dummy",
                        "usb_serial": "ABC123",
                    },
                ]
            )
        )
        mgr.config_file = cfg
        monkeypatch.setitem(
            mgr_mod.PICO_CLASSES, "rfswitch", DummyPicoRFSwitch
        )
        mgr.discover()
        assert "rfswitch" in mgr.picos
        r = mgr._redis()
        assert "rfswitch" in r.smembers(PICOS_SET)
        health = json.loads(r.hget(HEALTH_HASH, "rfswitch"))
        assert health["app_id"] == 5
        assert health["connected"] is True


# --- Redis config store ---------------------------------------------------


class TestRedisConfig:
    def test_discover_from_redis(self, mgr, monkeypatch):
        """When Redis has config, discover() uses it without touching file."""
        import picohost.manager as mgr_mod

        monkeypatch.setitem(
            mgr_mod.PICO_CLASSES, "rfswitch", DummyPicoRFSwitch
        )
        r = mgr._redis()
        r.hset(
            CONFIG_HASH,
            "rfswitch",
            json.dumps(
                {
                    "app_id": 5,
                    "port": "/dev/dummy",
                    "usb_serial": "ABC",
                }
            ),
        )
        mgr.config_file = "/nonexistent/path.json"  # shouldn't be read
        mgr.discover()
        assert "rfswitch" in mgr.picos

    def test_discover_skips_redis_when_disabled(
        self, mgr, monkeypatch, tmp_path
    ):
        """--no-redis-config makes discover() skip Redis."""
        import picohost.manager as mgr_mod

        monkeypatch.setitem(
            mgr_mod.PICO_CLASSES, "rfswitch", DummyPicoRFSwitch
        )
        # Populate Redis — should be ignored
        r = mgr._redis()
        r.hset(
            CONFIG_HASH,
            "motor",
            json.dumps(
                {
                    "app_id": 0,
                    "port": "/dev/dummy",
                    "usb_serial": "OLD",
                }
            ),
        )
        # Provide a file with rfswitch instead
        cfg = tmp_path / "pico_config.json"
        cfg.write_text(
            json.dumps(
                [
                    {"app_id": 5, "port": "/dev/dummy", "usb_serial": "NEW"},
                ]
            )
        )
        mgr.config_file = cfg
        mgr.use_redis_config = False
        mgr.discover()
        assert "rfswitch" in mgr.picos
        assert "motor" not in mgr.picos

    def test_discover_writes_back_to_file(self, mgr, monkeypatch, tmp_path):
        """After discovering from Redis, config is written back to file."""
        import picohost.manager as mgr_mod

        monkeypatch.setitem(
            mgr_mod.PICO_CLASSES, "rfswitch", DummyPicoRFSwitch
        )
        r = mgr._redis()
        r.hset(
            CONFIG_HASH,
            "rfswitch",
            json.dumps(
                {
                    "app_id": 5,
                    "port": "/dev/dummy",
                    "usb_serial": "ABC",
                }
            ),
        )
        cfg = tmp_path / "pico_config.json"
        mgr.config_file = cfg
        mgr.discover()
        assert cfg.exists()
        written = json.loads(cfg.read_text())
        assert len(written) == 1
        assert written[0]["app_id"] == 5

    def test_file_fallback_when_redis_empty(self, mgr, monkeypatch, tmp_path):
        """When Redis is empty, discover() falls through to file."""
        import picohost.manager as mgr_mod

        monkeypatch.setitem(
            mgr_mod.PICO_CLASSES, "rfswitch", DummyPicoRFSwitch
        )
        cfg = tmp_path / "pico_config.json"
        cfg.write_text(
            json.dumps(
                [
                    {"app_id": 5, "port": "/dev/dummy", "usb_serial": "X"},
                ]
            )
        )
        mgr.config_file = cfg
        mgr.discover()
        assert "rfswitch" in mgr.picos
        # Verify config was also published to Redis
        stored = json.loads(mgr._redis().hget(CONFIG_HASH, "rfswitch"))
        assert stored["app_id"] == 5


# --- manager commands -----------------------------------------------------


class TestManagerCommand:
    def test_rediscover_clears_and_reloads(self, mgr, monkeypatch, tmp_path):
        import picohost.manager as mgr_mod

        monkeypatch.setitem(
            mgr_mod.PICO_CLASSES, "rfswitch", DummyPicoRFSwitch
        )
        # Start with a device
        _attach(mgr, "rfswitch", DummyPicoRFSwitch)
        assert "rfswitch" in mgr.picos

        # Set up Redis config for rediscover to find
        r = mgr._redis()
        r.hset(
            CONFIG_HASH,
            "rfswitch",
            json.dumps(
                {
                    "app_id": 5,
                    "port": "/dev/dummy",
                    "usb_serial": "X",
                }
            ),
        )
        cfg = tmp_path / "pico_config.json"
        mgr.config_file = cfg

        mgr._process_command(
            r,
            "1-0",
            {
                "target": "manager",
                "cmd": json.dumps({"action": "rediscover"}),
                "source": "test",
            },
        )
        _, resp = r._streams[RESP_STREAM][0]
        assert resp["status"] == "ok"
        data = json.loads(resp["data"])
        assert "rfswitch" in data["devices"]
        assert data["count"] == 1

    def test_unknown_manager_action_returns_error(self, mgr):
        r = mgr._redis()
        mgr._process_command(
            r,
            "1-0",
            {
                "target": "manager",
                "cmd": json.dumps({"action": "nope"}),
                "source": "test",
            },
        )
        _, resp = r._streams[RESP_STREAM][0]
        assert resp["status"] == "error"
        assert "unknown manager action" in json.loads(resp["data"])["error"]


# --- reconnect & timeout regressions --------------------------------------


class TestReconnectHook:
    def test_motor_on_reconnect_replays_delay(self, mgr):
        motor = _attach(mgr, "motor", DummyPicoMotor)
        # Sanity: ctor stored the delay kwargs
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
        # _rediscover_port should update port
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


class _StubDevice:
    """Bare attribute holder for testing wait_for_updates in isolation."""

    def __init__(self):
        self.status = {}
        self.name = "stub"


class TestTimeoutError:
    """
    Both wait_for_updates methods used to use ``assert``, which gets
    stripped under ``python -O``. They now raise TimeoutError; verify
    that explicitly.
    """

    def test_peltier_wait_for_updates_raises_timeout(self):
        from picohost.base import PicoPeltier

        with pytest.raises(TimeoutError, match="No status"):
            PicoPeltier.wait_for_updates(_StubDevice(), timeout=0.05)

    def test_motor_wait_for_updates_raises_timeout(self):
        from picohost.motor import PicoMotor

        with pytest.raises(TimeoutError, match="No status"):
            PicoMotor.wait_for_updates(_StubDevice(), timeout=0.05)


# --- module-level constants -----------------------------------------------


def test_blocked_actions_includes_lifecycle():
    for action in ("connect", "disconnect", "reconnect"):
        assert action in _BLOCKED_ACTIONS


def test_cmd_stream_constant_unchanged():
    # Other consumers (eigsep_observing) read from this stream by name.
    assert CMD_STREAM == "stream:pico_cmd"
    assert RESP_STREAM == "stream:pico_resp"
