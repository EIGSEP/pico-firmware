"""
Unit tests for the picohost base classes.

Tests PicoDevice, PicoMotor, PicoRFSwitch, and PicoPeltier through their
DummyPico* wrappers, which use MockSerial + emulators instead of real hardware.
"""

import json
import pytest
from conftest import wait_for_condition, wait_for_settle
from picohost.testing import (
    DummyPicoDevice,
    DummyPicoMotor,
    DummyPicoRFSwitch,
    DummyPicoPeltier,
)


class TestPicoDevice:
    """Test the base PicoDevice interface using DummyPicoDevice (no emulator)."""

    def test_find_pico_ports_exists(self):
        """find_pico_ports is a static method on PicoDevice."""
        assert hasattr(DummyPicoDevice, "find_pico_ports")
        assert callable(DummyPicoDevice.find_pico_ports)

    def test_connect_success(self):
        """DummyPicoDevice.connect() creates a MockSerial pair."""
        device = DummyPicoDevice("/dev/dummy")
        assert device.is_connected is True
        assert device.ser is not None
        assert device.ser.is_open
        device.disconnect()

    def test_connect_failure(self):
        """Dummy devices always connect; this test is a placeholder."""
        pass

    def test_send_command_writes_json(self):
        """send_command() serializes a dict as compact JSON + newline."""
        device = DummyPicoDevice("/dev/dummy")
        cmd = {"cmd": "test", "value": 42}
        device.send_command(cmd)

        # No emulator on base DummyPicoDevice, so bytes stay in peer buffer
        expected_data = json.dumps(cmd, separators=(",", ":")) + "\n"
        assert device.ser.peer._read_buffer == expected_data.encode("utf-8")
        device.disconnect()

    def test_send_command_raises_when_disconnected(self):
        """send_command() raises ConnectionError when the port is closed."""
        device = DummyPicoDevice("/dev/dummy")
        device.disconnect()
        with pytest.raises(ConnectionError):
            device.send_command({"cmd": "test"})

    def test_parse_response_valid_json(self):
        """parse_response() returns a dict for valid JSON."""
        device = DummyPicoDevice("/dev/dummy")
        data = device.parse_response('{"status": "ok", "value": 123}')
        assert data == {"status": "ok", "value": 123}
        device.disconnect()

    def test_parse_response_invalid_json(self):
        """parse_response() returns None for non-JSON input."""
        device = DummyPicoDevice("/dev/dummy")
        assert device.parse_response("not json") is None
        device.disconnect()

    def test_context_manager_connects_and_disconnects(self):
        """__enter__ provides a connected device; __exit__ disconnects."""
        with DummyPicoDevice("/dev/dummy") as device:
            assert device.ser is not None
            assert device._running is True
        assert device.ser is None
        assert device._running is False

    def test_reader_thread_populates_last_status(self):
        """The reader thread picks up JSON written to the peer and sets last_status."""
        device = DummyPicoDevice("/dev/dummy")
        # Simulate firmware writing a JSON status line to host
        status_json = '{"sensor_name":"test","value":99}\n'
        device.ser.peer.write(status_json.encode())
        wait_for_condition(
            lambda: device.last_status.get("sensor_name") == "test",
            cadence_ms=device.EMULATOR_CADENCE_MS,
        )
        assert device.last_status.get("value") == 99
        device.disconnect()

    def test_redis_handler_is_bound_before_connect(self):
        """__init__ binds redis_handler before connect() is invoked."""

        class ProbePicoDevice(DummyPicoDevice):
            def connect(self):
                self.redis_handler_seen_in_connect = self.redis_handler
                return super().connect()

        device = ProbePicoDevice("/dev/dummy")
        assert device.redis_handler_seen_in_connect is None
        device.disconnect()

    def test_redis_handler_with_client_is_bound_before_connect(self):
        """Configured Redis handler is available during connect()."""

        class FakeRedis:
            def add_metadata(self, _name, _data):
                pass

        class ProbePicoDevice(DummyPicoDevice):
            def connect(self):
                self.redis_handler_seen_in_connect = self.redis_handler
                return super().connect()

        device = ProbePicoDevice("/dev/dummy", eig_redis=FakeRedis())
        assert callable(device.redis_handler_seen_in_connect)
        device.disconnect()


class TestPicoMotor:
    """Test PicoMotor commands and status via DummyPicoMotor (with emulator)."""

    def test_deg_to_steps_conversion(self):
        """Verify degree-to-step conversion: 1.8 deg * 113 teeth = 113 steps."""
        motor = DummyPicoMotor("/dev/dummy")
        assert motor.deg_to_steps(0) == 0
        assert motor.deg_to_steps(1.8) == 113
        assert motor.deg_to_steps(360) == 22600
        motor.disconnect()

    def test_steps_to_deg_conversion(self):
        """Verify step-to-degree conversion is the inverse of deg_to_steps."""
        motor = DummyPicoMotor("/dev/dummy")
        assert motor.steps_to_deg(113) == pytest.approx(1.8, abs=0.01)
        assert motor.steps_to_deg(0) == 0.0
        motor.disconnect()

    def test_move_command_updates_target(self):
        """Sending az_target_deg updates az_target_pos in the emulator status."""
        motor = DummyPicoMotor("/dev/dummy")
        cadence = motor.EMULATOR_CADENCE_MS
        az_deg = 10.0
        expected_steps = motor.deg_to_steps(az_deg)
        before = motor.last_status.get("az_target_pos")
        motor.az_target_deg(az_deg, wait_for_start=False, wait_for_stop=False)
        assert (
            wait_for_settle(
                lambda: motor.last_status.get("az_target_pos"),
                initial=before,
                cadence_ms=cadence,
                max_cycles=20,
            )
            == expected_steps
        )
        motor.disconnect()

    def test_status_has_motor_fields(self):
        """Motor status should contain all expected fields from the emulator."""
        motor = DummyPicoMotor("/dev/dummy")
        wait_for_condition(
            lambda: motor.last_status.get("sensor_name") == "motor",
            cadence_ms=motor.EMULATOR_CADENCE_MS,
        )
        for key in ("az_pos", "az_target_pos", "el_pos", "el_target_pos"):
            assert key in motor.last_status, f"Missing key: {key}"
        motor.disconnect()


class TestPicoRFSwitch:
    """Test PicoRFSwitch command dispatch via DummyPicoRFSwitch (with emulator)."""

    def test_switch_state_updates_emulator(self):
        """switch('VNAO') sends the correct sw_state to the emulator."""
        switch = DummyPicoRFSwitch("/dev/dummy")
        cadence = switch.EMULATOR_CADENCE_MS
        expected_state = switch.rbin(switch.path_str["VNAO"])
        switch.switch("VNAO")
        wait_for_condition(
            lambda: switch.last_status.get("sw_state") == expected_state,
            cadence_ms=cadence,
            max_cycles=10,
        )
        switch.disconnect()

    def test_switch_all_valid_states(self):
        """Every defined path string can be switched without error."""
        switch = DummyPicoRFSwitch("/dev/dummy")
        for state in switch.paths:
            switch.switch(state)
        switch.disconnect()

    def test_switch_invalid_state_raises(self):
        """Switching to an undefined state raises ValueError."""
        switch = DummyPicoRFSwitch("/dev/dummy")
        with pytest.raises(ValueError, match="Invalid switch state"):
            switch.switch("INVALID")
        switch.disconnect()

    def test_rbin_lsb_first(self):
        """rbin() interprets the first character as LSB."""
        assert DummyPicoRFSwitch.rbin("10000000") == 1
        assert DummyPicoRFSwitch.rbin("01000000") == 2
        assert DummyPicoRFSwitch.rbin("11000000") == 3

    def test_paths_dict_values(self):
        """paths property converts path_str to integer values correctly."""
        switch = DummyPicoRFSwitch("/dev/dummy")
        paths = switch.paths
        assert paths["VNAO"] == 1  # "10000000" LSB-first = 1
        assert paths["RFANT"] == 0  # "00000000" = 0
        assert paths["VNAS"] == 3  # "11000000" LSB-first = 3
        switch.disconnect()


class TestPicoPeltier:
    """Test PicoPeltier commands via DummyPicoPeltier (with emulator)."""

    def test_set_temperature_channel_lna(self):
        """Setting LNA channel target updates emulator status."""
        peltier = DummyPicoPeltier("/dev/dummy")
        cadence = peltier.EMULATOR_CADENCE_MS
        before = peltier.last_status.get("LNA_T_target")
        peltier.set_temperature(T_LNA=25.5, LNA_hyst=0.5)
        assert wait_for_settle(
            lambda: peltier.last_status.get("LNA_T_target"),
            initial=before,
            cadence_ms=cadence,
            max_cycles=10,
        ) == pytest.approx(25.5)
        assert peltier.last_status.get("LNA_hysteresis") == pytest.approx(0.5)
        peltier.disconnect()

    def test_set_temperature_channel_load(self):
        """Setting LOAD channel target updates emulator status."""
        peltier = DummyPicoPeltier("/dev/dummy")
        cadence = peltier.EMULATOR_CADENCE_MS
        before = peltier.last_status.get("LOAD_hysteresis")
        peltier.set_temperature(T_LOAD=30.0, LOAD_hyst=1.0)
        assert wait_for_settle(
            lambda: peltier.last_status.get("LOAD_hysteresis"),
            initial=before,
            cadence_ms=cadence,
            max_cycles=10,
        ) == pytest.approx(1.0)
        assert peltier.last_status.get("LOAD_T_target") == pytest.approx(30.0)
        peltier.disconnect()

    def test_set_temperature_both_channels(self):
        """Setting both channels in one call updates both in emulator."""
        peltier = DummyPicoPeltier("/dev/dummy")
        cadence = peltier.EMULATOR_CADENCE_MS
        before = peltier.last_status.get("LNA_T_target")
        peltier.set_temperature(
            T_LNA=28.0, LNA_hyst=0.3, T_LOAD=32.0, LOAD_hyst=0.8
        )
        assert wait_for_settle(
            lambda: peltier.last_status.get("LNA_T_target"),
            initial=before,
            cadence_ms=cadence,
            max_cycles=10,
        ) == pytest.approx(28.0)
        assert peltier.last_status.get("LOAD_T_target") == pytest.approx(32.0)
        peltier.disconnect()

    def test_enable_channels(self):
        """set_enable() updates the enabled flags in emulator status."""
        peltier = DummyPicoPeltier("/dev/dummy")
        cadence = peltier.EMULATOR_CADENCE_MS
        peltier.set_enable(LNA=True, LOAD=True)
        wait_for_condition(
            lambda: peltier.last_status.get("LNA_enabled") is True,
            cadence_ms=cadence,
        )
        assert peltier.last_status.get("LOAD_enabled") is True
        peltier.disconnect()

    def test_disable_channels(self):
        """Disabling channels is reflected in emulator status."""
        peltier = DummyPicoPeltier("/dev/dummy")
        cadence = peltier.EMULATOR_CADENCE_MS
        peltier.set_enable(LNA=False, LOAD=False)
        wait_for_condition(
            lambda: peltier.last_status.get("LNA_enabled") is False,
            cadence_ms=cadence,
        )
        assert peltier.last_status.get("LOAD_enabled") is False
        peltier.disconnect()

    def test_status_has_tempctrl_fields(self):
        """Peltier status should contain all tempctrl fields from the emulator."""
        peltier = DummyPicoPeltier("/dev/dummy")
        wait_for_condition(
            lambda: peltier.last_status.get("sensor_name") == "tempctrl",
            cadence_ms=peltier.EMULATOR_CADENCE_MS,
        )
        for key in (
            "LNA_T_now",
            "LOAD_T_now",
            "LNA_drive_level",
            "LOAD_drive_level",
        ):
            assert key in peltier.last_status, f"Missing key: {key}"
        peltier.disconnect()
