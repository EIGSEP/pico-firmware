"""
Tests for PicoManager.
"""

import json
import pytest

from picohost.testing import (
    DummyPicoManager,
    DummyPicoDevice,
    DummyPicoRFSwitch,
    DummyPicoMotor,
    DummyPicoPeltier,
    DummyPicoTherm,
    DummyPicoLidar,
    MockRedis,
    DUMMY_PICO_CLASSES,
)
from picohost.manager import (
    PICOS_SET,
    HEALTH_HASH,
    CMD_STREAM,
    RESP_STREAM,
    APP_NAMES,
    APP_IDS,
    PICO_CLASSES,
    _BLOCKED_ACTIONS,
)


class TestAppMappings:
    """Test app_id <-> name mappings."""

    def test_app_names_cover_all_ids(self):
        for app_id in range(6):
            assert app_id in APP_NAMES

    def test_app_ids_inverse(self):
        for app_id, name in APP_NAMES.items():
            assert APP_IDS[name] == app_id

    def test_pico_classes_match_names(self):
        for name in APP_NAMES.values():
            assert name in PICO_CLASSES

    def test_dummy_classes_match_names(self):
        for name in APP_NAMES.values():
            assert name in DUMMY_PICO_CLASSES


class TestPicoManagerDiscovery:
    """Test device discovery and registration."""

    def test_add_dummy_device(self):
        mgr = DummyPicoManager()
        pico = mgr.add_dummy_device("switch")
        assert "switch" in mgr.picos
        assert isinstance(pico, DummyPicoRFSwitch)

    def test_add_motor(self):
        mgr = DummyPicoManager()
        pico = mgr.add_dummy_device("motor")
        assert isinstance(pico, DummyPicoMotor)

    def test_add_peltier(self):
        mgr = DummyPicoManager()
        pico = mgr.add_dummy_device("peltier")
        assert isinstance(pico, DummyPicoPeltier)

    def test_add_generic_device(self):
        mgr = DummyPicoManager()
        pico = mgr.add_dummy_device("therm")
        assert isinstance(pico, DummyPicoDevice)

    def test_add_multiple_devices(self):
        mgr = DummyPicoManager()
        mgr.add_dummy_device("switch")
        mgr.add_dummy_device("motor")
        mgr.add_dummy_device("peltier")
        assert len(mgr.picos) == 3

    def test_redis_set_updated(self):
        mgr = DummyPicoManager()
        mgr.add_dummy_device("switch")
        r = mgr._redis()
        assert "switch" in r.smembers(PICOS_SET)

    def test_discover_is_noop(self):
        mgr = DummyPicoManager()
        mgr.discover()
        assert len(mgr.picos) == 0


class TestCommandRouting:
    """Test _route_command dispatch."""

    def test_switch_action(self):
        mgr = DummyPicoManager()
        pico = mgr.add_dummy_device("switch")
        result = mgr._route_command(
            pico, "switch", {"action": "switch", "state": "RFANT"}
        )
        assert result["action"] == "switch"
        assert result["result"] is True

    def test_raw_command(self):
        mgr = DummyPicoManager()
        pico = mgr.add_dummy_device("therm")
        result = mgr._route_command(
            pico, "therm", {"some_key": "value"}
        )
        assert result["sent"] is True

    def test_blocked_action_disconnect(self):
        mgr = DummyPicoManager()
        pico = mgr.add_dummy_device("switch")
        with pytest.raises(ValueError, match="not allowed"):
            mgr._route_command(
                pico, "switch", {"action": "disconnect"}
            )

    def test_blocked_action_connect(self):
        mgr = DummyPicoManager()
        pico = mgr.add_dummy_device("switch")
        with pytest.raises(ValueError, match="not allowed"):
            mgr._route_command(
                pico, "switch", {"action": "connect"}
            )

    def test_blocked_action_reconnect(self):
        mgr = DummyPicoManager()
        pico = mgr.add_dummy_device("switch")
        with pytest.raises(ValueError, match="not allowed"):
            mgr._route_command(
                pico, "switch", {"action": "reconnect"}
            )

    def test_private_action_blocked(self):
        mgr = DummyPicoManager()
        pico = mgr.add_dummy_device("switch")
        with pytest.raises(ValueError, match="not allowed"):
            mgr._route_command(
                pico, "switch", {"action": "_reader_thread_func"}
            )

    def test_unknown_action(self):
        mgr = DummyPicoManager()
        pico = mgr.add_dummy_device("switch")
        with pytest.raises(ValueError, match="Unknown action"):
            mgr._route_command(
                pico, "switch", {"action": "nonexistent"}
            )

    def test_motor_command_action(self):
        mgr = DummyPicoManager()
        pico = mgr.add_dummy_device("motor")
        result = mgr._route_command(
            pico, "motor",
            {"action": "motor_command", "az_set_target_pos": 1000},
        )
        assert result["action"] == "motor_command"

    def test_peltier_set_temperature(self):
        mgr = DummyPicoManager()
        pico = mgr.add_dummy_device("peltier")
        result = mgr._route_command(
            pico, "peltier",
            {"action": "set_temperature", "T_A": 25.0, "A_hyst": 0.5},
        )
        assert result["action"] == "set_temperature"
        assert result["result"] is True


class TestProcessCommand:
    """Test full command processing pipeline."""

    def test_valid_switch_command(self):
        mgr = DummyPicoManager()
        mgr.add_dummy_device("switch")
        r = mgr._redis()
        fields = {
            "target": "switch",
            "cmd": json.dumps(
                {"action": "switch", "state": "RFANT"}
            ),
            "source": "test",
        }
        mgr._process_command(r, "1-0", fields)

        assert len(r._streams.get(RESP_STREAM, [])) == 1
        _, resp = r._streams[RESP_STREAM][0]
        assert resp["status"] == "ok"
        assert resp["source"] == "test"

    def test_unknown_target(self):
        mgr = DummyPicoManager()
        r = mgr._redis()
        fields = {
            "target": "nonexistent",
            "cmd": "{}",
            "source": "test",
        }
        mgr._process_command(r, "1-0", fields)

        _, resp = r._streams[RESP_STREAM][0]
        assert resp["status"] == "error"
        data = json.loads(resp["data"])
        assert "unknown target" in data["error"]

    def test_invalid_json_command(self):
        mgr = DummyPicoManager()
        mgr.add_dummy_device("switch")
        r = mgr._redis()
        fields = {
            "target": "switch",
            "cmd": "not json",
            "source": "test",
        }
        mgr._process_command(r, "1-0", fields)

        _, resp = r._streams[RESP_STREAM][0]
        assert resp["status"] == "error"

    def test_blocked_action_returns_error(self):
        mgr = DummyPicoManager()
        mgr.add_dummy_device("switch")
        r = mgr._redis()
        fields = {
            "target": "switch",
            "cmd": json.dumps({"action": "disconnect"}),
            "source": "test",
        }
        mgr._process_command(r, "1-0", fields)

        _, resp = r._streams[RESP_STREAM][0]
        assert resp["status"] == "error"

    def test_bytes_fields_handled(self):
        """Test that byte-encoded fields from Redis are handled."""
        mgr = DummyPicoManager()
        mgr.add_dummy_device("switch")
        r = mgr._redis()
        fields = {
            b"target": b"switch",
            b"cmd": json.dumps(
                {"action": "switch", "state": "RFANT"}
            ).encode(),
            b"source": b"test",
        }
        mgr._process_command(r, b"1-0", fields)

        assert len(r._streams.get(RESP_STREAM, [])) == 1
        _, resp = r._streams[RESP_STREAM][0]
        assert resp["status"] == "ok"


class TestClaims:
    """Test soft claim mechanism."""

    def test_claim_device(self):
        mgr = DummyPicoManager()
        mgr.add_dummy_device("switch")
        r = mgr._redis()
        fields = {
            "target": "switch",
            "cmd": json.dumps({"action": "claim", "ttl": 60}),
            "source": "switch_loop",
        }
        mgr._process_command(r, "1-0", fields)

        assert r.get("pico_claim:switch") == "switch_loop"
        _, resp = r._streams[RESP_STREAM][0]
        assert resp["status"] == "ok"

    def test_release_device(self):
        mgr = DummyPicoManager()
        mgr.add_dummy_device("switch")
        r = mgr._redis()

        # Set a claim
        r.set("pico_claim:switch", "switch_loop")

        fields = {
            "target": "switch",
            "cmd": json.dumps({"action": "release"}),
            "source": "switch_loop",
        }
        mgr._process_command(r, "1-0", fields)

        assert r.get("pico_claim:switch") is None

    def test_claim_warning_on_override(self):
        mgr = DummyPicoManager()
        mgr.add_dummy_device("switch")
        r = mgr._redis()

        # Claim by switch_loop
        r.set("pico_claim:switch", "switch_loop")

        # Command from different source
        fields = {
            "target": "switch",
            "cmd": json.dumps(
                {"action": "switch", "state": "RFANT"}
            ),
            "source": "interactive",
        }
        mgr._process_command(r, "1-0", fields)

        _, resp = r._streams[RESP_STREAM][0]
        assert resp["status"] == "ok"
        assert "warning" in resp
        assert "switch_loop" in resp["warning"]

    def test_no_warning_from_claim_owner(self):
        mgr = DummyPicoManager()
        mgr.add_dummy_device("switch")
        r = mgr._redis()

        r.set("pico_claim:switch", "switch_loop")

        fields = {
            "target": "switch",
            "cmd": json.dumps(
                {"action": "switch", "state": "RFANT"}
            ),
            "source": "switch_loop",
        }
        mgr._process_command(r, "1-0", fields)

        _, resp = r._streams[RESP_STREAM][0]
        assert resp["status"] == "ok"
        assert "warning" not in resp

    def test_no_warning_without_claim(self):
        mgr = DummyPicoManager()
        mgr.add_dummy_device("switch")
        r = mgr._redis()

        fields = {
            "target": "switch",
            "cmd": json.dumps(
                {"action": "switch", "state": "RFANT"}
            ),
            "source": "anyone",
        }
        mgr._process_command(r, "1-0", fields)

        _, resp = r._streams[RESP_STREAM][0]
        assert "warning" not in resp


class TestLifecycle:
    """Test manager lifecycle."""

    def test_start_stop(self):
        mgr = DummyPicoManager()
        mgr.add_dummy_device("switch")
        mgr.start()
        assert mgr._running is True
        mgr.stop()
        assert mgr._running is False
        assert len(mgr.picos) == 0

    def test_stop_cleans_up_devices(self):
        mgr = DummyPicoManager()
        switch = mgr.add_dummy_device("switch")
        motor = mgr.add_dummy_device("motor")
        mgr.stop()
        assert switch.ser is None
        assert motor.ser is None


class TestNewDeviceClasses:
    """Test PicoTherm and PicoLidar dummy devices."""

    def test_therm_device(self):
        mgr = DummyPicoManager()
        pico = mgr.add_dummy_device("therm")
        assert isinstance(pico, DummyPicoTherm)
        assert pico.is_connected
        assert "sensor_name" in pico.status

    def test_lidar_device(self):
        mgr = DummyPicoManager()
        pico = mgr.add_dummy_device("lidar")
        assert isinstance(pico, DummyPicoLidar)
        assert pico.is_connected
        assert "sensor_name" in pico.status

    def test_therm_raw_command(self):
        mgr = DummyPicoManager()
        pico = mgr.add_dummy_device("therm")
        result = mgr._route_command(
            pico, "therm", {"query": "temperature"}
        )
        assert result["sent"] is True

    def test_lidar_raw_command(self):
        mgr = DummyPicoManager()
        pico = mgr.add_dummy_device("lidar")
        result = mgr._route_command(
            pico, "lidar", {"query": "distance"}
        )
        assert result["sent"] is True


class TestMotorInheritance:
    """Test PicoMotor inherits from PicoStatus correctly."""

    def test_motor_has_status_methods(self):
        motor = DummyPicoMotor("/dev/dummy")
        assert hasattr(motor, "update_status")
        assert hasattr(motor, "status")
        assert hasattr(motor, "verbose")

    def test_motor_on_reconnect(self):
        """Test that on_reconnect re-sends set_delay."""
        motor = DummyPicoMotor("/dev/dummy")
        # Clear the write buffer
        motor.ser.peer._read_buffer = bytearray()
        motor.on_reconnect()
        # set_delay should have written a command
        sent = motor.ser.peer._read_buffer.decode("utf-8").strip()
        assert "az_up_delay_us" in sent

    def test_motor_reconnect_calls_hook(self):
        """Test that reconnect() calls on_reconnect()."""
        motor = DummyPicoMotor("/dev/dummy")
        motor.ser.peer._read_buffer = bytearray()
        motor.reconnect()
        assert motor.is_connected
        # on_reconnect should have sent set_delay
        sent = motor.ser.peer._read_buffer.decode("utf-8").strip()
        assert "az_up_delay_us" in sent


class TestTimeoutError:
    """Test that wait_for_updates raises TimeoutError, not AssertionError."""

    def test_pico_status_timeout(self):
        """PicoStatus.wait_for_updates should raise TimeoutError."""
        from picohost.base import PicoStatus
        # DummyPicoTherm inherits from PicoStatus so has .status
        device = DummyPicoTherm("/dev/dummy")
        device.status = {}  # clear the dummy status
        with pytest.raises(TimeoutError, match="No status"):
            PicoStatus.wait_for_updates(device, timeout=0.2)
