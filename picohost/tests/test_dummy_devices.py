"""
Tests for DummyPico* device classes.

These tests verify that the DummyPicoDevice infrastructure (MockSerial pair +
emulators) correctly replaces real hardware for testing. Tests cover connection
lifecycle, command dispatch, status propagation, and MockSerial integration.
"""

import json
import numpy as np
import pytest
import mockserial

from conftest import wait_for_condition, wait_for_settle
from picohost.testing import (
    DummyPicoDevice,
    DummyPicoMotor,
    DummyPicoRFSwitch,
    DummyPicoPeltier,
)


# ---------------------------------------------------------------------------
# DummyPicoDevice (base class, no emulator)
# ---------------------------------------------------------------------------


class TestDummyPicoDevice:
    """Test DummyPicoDevice connection and serial operations.

    DummyPicoDevice has no emulator (EMULATOR_CLASS = None), so the peer
    buffer is not consumed by background threads, making it safe to assert
    on raw bytes written to the peer.
    """

    def test_connect_creates_mock_serial(self):
        """connect() creates a MockSerial instance with a peer."""
        device = DummyPicoDevice(port="/dev/ttyUSB0")
        assert isinstance(device.ser, mockserial.MockSerial)
        assert device.ser.is_open
        device.disconnect()

    def test_disconnect_clears_serial(self):
        """disconnect() sets ser to None and stops the reader thread."""
        device = DummyPicoDevice(port="/dev/ttyUSB0")
        device.disconnect()
        assert device.ser is None
        assert device._running is False

    def test_send_command_writes_json_to_peer(self):
        """send_command() writes compact JSON + newline to the peer's read buffer."""
        device = DummyPicoDevice(port="/dev/ttyUSB0")
        cmd = {"cmd": "test", "value": 123}
        device.send_command(cmd)

        expected = json.dumps(cmd, separators=(",", ":")) + "\n"
        assert device.ser.peer.read(len(expected)) == expected.encode()
        device.disconnect()

    def test_send_command_raises_when_disconnected(self):
        """send_command() raises ConnectionError when no active connection."""
        device = DummyPicoDevice(port="/dev/ttyUSB0")
        device.disconnect()
        with pytest.raises(ConnectionError):
            device.send_command({"cmd": "test"})

    def test_reader_thread_populates_last_status(self):
        """JSON written by the peer is parsed by the reader thread into last_status."""
        device = DummyPicoDevice(port="/dev/ttyUSB0")
        device.ser.peer.write(b'{"sensor_name":"test","value":42}\n')
        wait_for_condition(
            lambda: device.last_status.get("sensor_name") == "test",
            cadence_ms=device.EMULATOR_CADENCE_MS,
        )
        assert device.last_status == {"sensor_name": "test", "value": 42}
        device.disconnect()

    def test_parse_response_valid_json(self):
        """parse_response() returns a dict for well-formed JSON."""
        device = DummyPicoDevice(port="/dev/ttyUSB0")
        result = device.parse_response('{"status": "ok", "data": 123}')
        assert result == {"status": "ok", "data": 123}
        device.disconnect()

    def test_parse_response_invalid_json(self):
        """parse_response() returns None for malformed input."""
        device = DummyPicoDevice(port="/dev/ttyUSB0")
        assert device.parse_response("not json") is None
        device.disconnect()

    def test_context_manager_lifecycle(self):
        """__enter__/__exit__ connect and disconnect the device."""
        device = DummyPicoDevice(port="/dev/ttyUSB0")
        with device as dev:
            assert isinstance(dev.ser, mockserial.MockSerial)
            assert dev.ser.is_open
        assert device.ser is None


# ---------------------------------------------------------------------------
# DummyPicoMotor (emulator-backed)
# ---------------------------------------------------------------------------


class TestDummyPicoMotor:
    """Test DummyPicoMotor with the MotorEmulator.

    Commands sent through the PicoMotor API are dispatched to the emulator,
    which updates its state and sends periodic status JSON that the reader
    thread picks up into motor.last_status.
    """

    def test_deg_to_steps_known_values(self):
        """Verify degree-to-step conversion for known values."""
        motor = DummyPicoMotor(port="/dev/ttyUSB0")
        assert motor.deg_to_steps(0) == 0
        assert motor.deg_to_steps(1.8) == 113  # one full step * 113 teeth
        assert motor.deg_to_steps(360) == 22600  # full revolution
        motor.disconnect()

    def test_motor_command_updates_emulator_target(self):
        """motor_command() is dispatched to the emulator and reflected in status."""
        motor = DummyPicoMotor(port="/dev/ttyUSB0")
        cadence = motor.EMULATOR_CADENCE_MS
        before = motor.last_status.get("az_target_pos")
        motor.motor_command(az_set_target_pos=1000, el_set_target_pos=500)
        assert (
            wait_for_settle(
                lambda: motor.last_status.get("az_target_pos"),
                initial=before,
                cadence_ms=cadence,
                max_cycles=10,
            )
            == 1000
        )
        assert motor.last_status["el_target_pos"] == 500
        motor.disconnect()

    def test_halt_sets_target_to_current(self):
        """After halt, target_pos should equal current pos."""
        motor = DummyPicoMotor(port="/dev/ttyUSB0")
        cadence = motor.EMULATOR_CADENCE_MS
        motor.motor_command(az_set_target_pos=1000)
        wait_for_condition(
            lambda: motor.last_status.get("az_target_pos") == 1000,
            cadence_ms=cadence,
        )
        motor.halt()
        wait_for_condition(
            lambda: (
                motor.last_status.get("az_target_pos") == motor.last_status.get("az_pos")
            ),
            cadence_ms=cadence,
        )
        motor.disconnect()

    def test_status_populated_after_first_cycle(self):
        """First emulator status cycle populates the required motor keys."""
        motor = DummyPicoMotor(port="/dev/ttyUSB0")
        wait_for_condition(
            lambda: motor.last_status.get("sensor_name") == "motor",
            cadence_ms=motor.EMULATOR_CADENCE_MS,
        )
        assert "az_pos" in motor.last_status
        assert "el_pos" in motor.last_status
        assert "az_target_pos" in motor.last_status
        assert "el_target_pos" in motor.last_status
        motor.disconnect()

    def test_scan_homes_after_normal_completion(self):
        """scan() returns motors to (0, 0) one at a time after finishing."""
        motor = DummyPicoMotor(port="/dev/ttyUSB0")
        cadence = motor.EMULATOR_CADENCE_MS
        wait_for_condition(
            lambda: "az_target_pos" in motor.last_status, cadence_ms=cadence
        )
        motor.scan(
            az_range_deg=np.array([-10.0, 0.0, 10.0]),
            el_range_deg=np.array([-10.0, 0.0, 10.0]),
            repeat_count=1,
        )
        # after scan completes, motors should be back at 0
        wait_for_condition(
            lambda: (
                motor.last_status.get("az_pos") == 0
                and motor.last_status.get("el_pos") == 0
            ),
            cadence_ms=cadence,
            max_cycles=10,
        )
        assert motor.last_status["az_pos"] == 0
        assert motor.last_status["el_pos"] == 0
        motor.disconnect()

    def test_scan_does_not_home_on_interrupt(self):
        """scan() only halts (does not home) when interrupted."""
        motor = DummyPicoMotor(port="/dev/ttyUSB0")
        wait_for_condition(
            lambda: "az_target_pos" in motor.last_status,
            cadence_ms=motor.EMULATOR_CADENCE_MS,
        )
        # patch wait_for_stop to raise after the initial homing
        # (2 calls for home-az + home-el, then interrupt during scan)
        real_wait = motor.wait_for_stop
        call_count = 0

        def interrupt_after_homing(*a, **kw):
            nonlocal call_count
            call_count += 1
            real_wait(*a, **kw)
            if call_count > 2:
                raise KeyboardInterrupt

        motor.wait_for_stop = interrupt_after_homing
        with pytest.raises(KeyboardInterrupt):
            motor.scan(
                az_range_deg=np.array([-10.0, 0.0, 10.0]),
                el_range_deg=np.array([-10.0, 0.0, 10.0]),
            )
        # halt was called, but post-scan homing did not run
        assert motor.last_status["az_target_pos"] == motor.last_status["az_pos"]
        assert motor.last_status["el_target_pos"] == motor.last_status["el_pos"]
        motor.disconnect()


# ---------------------------------------------------------------------------
# DummyPicoRFSwitch (emulator-backed)
# ---------------------------------------------------------------------------


class TestDummyPicoRFSwitch:
    """Test DummyPicoRFSwitch with the RFSwitchEmulator."""

    def test_rbin_converts_lsb_first(self):
        """rbin() interprets the first character as the LSB."""
        assert DummyPicoRFSwitch.rbin("10000000") == 1
        assert DummyPicoRFSwitch.rbin("01000000") == 2
        assert DummyPicoRFSwitch.rbin("11000000") == 3
        assert DummyPicoRFSwitch.rbin("00100000") == 4

    def test_paths_property_returns_integer_values(self):
        """paths converts path_str binary strings to integer switch states."""
        switch = DummyPicoRFSwitch(port="/dev/ttyUSB0")
        paths = switch.paths
        assert isinstance(paths, dict)
        assert paths["VNAO"] == 1  # "10000000" reversed = 1
        assert paths["RFANT"] == 0  # "00000000" = 0
        switch.disconnect()

    def test_switch_valid_state_updates_emulator(self):
        """Each valid switch state is dispatched to the emulator and reflected in status."""
        switch = DummyPicoRFSwitch(port="/dev/ttyUSB0")
        cadence = switch.EMULATOR_CADENCE_MS
        for state in switch.paths:
            switch.switch(state)

        # Verify the last state is reflected
        last_state = list(switch.paths.keys())[-1]
        wait_for_condition(
            lambda: (
                switch.last_status.get("sw_state") == switch.paths[last_state]
            ),
            cadence_ms=cadence,
            max_cycles=10,
        )
        switch.disconnect()

    def test_switch_invalid_state_raises_valueerror(self):
        """Switching to an undefined state raises ValueError."""
        switch = DummyPicoRFSwitch(port="/dev/ttyUSB0")
        with pytest.raises(ValueError, match="Invalid switch state"):
            switch.switch("INVALID_STATE")
        switch.disconnect()


# ---------------------------------------------------------------------------
# DummyPicoPeltier (emulator-backed)
# ---------------------------------------------------------------------------


class TestDummyPicoPeltier:
    """Test DummyPicoPeltier with the TempCtrlEmulator.

    Commands set temperature targets, hysteresis, and enable flags in the
    emulator, which are then reflected in the periodic status updates.
    """

    def test_set_temperature_channel_lna(self):
        """Setting LNA channel target and hysteresis updates emulator status."""
        peltier = DummyPicoPeltier(port="/dev/ttyUSB0")
        cadence = peltier.EMULATOR_CADENCE_MS
        before = peltier.last_status.get("LNA_T_target")
        peltier.set_temperature(T_LNA=25.5, LNA_hyst=0.5)
        assert wait_for_settle(
            lambda: peltier.last_status.get("LNA_T_target"),
            initial=before,
            cadence_ms=cadence,
            max_cycles=10,
        ) == pytest.approx(25.5)
        assert peltier.last_status["LNA_hysteresis"] == pytest.approx(0.5)
        peltier.disconnect()

    def test_set_temperature_both_channels(self):
        """Setting both channels in one call updates both in emulator."""
        peltier = DummyPicoPeltier(port="/dev/ttyUSB0")
        cadence = peltier.EMULATOR_CADENCE_MS
        before = peltier.last_status.get("LOAD_T_target")
        peltier.set_temperature(
            T_LNA=30.0, LNA_hyst=1.0, T_LOAD=25.0, LOAD_hyst=0.5
        )
        assert wait_for_settle(
            lambda: peltier.last_status.get("LOAD_T_target"),
            initial=before,
            cadence_ms=cadence,
            max_cycles=10,
        ) == pytest.approx(25.0)
        assert peltier.last_status["LNA_T_target"] == pytest.approx(30.0)
        peltier.disconnect()

    def test_set_enable_mixed(self):
        """Enabling LNA and disabling LOAD is reflected in emulator status."""
        peltier = DummyPicoPeltier(port="/dev/ttyUSB0")
        cadence = peltier.EMULATOR_CADENCE_MS
        peltier.set_enable(LNA=True, LOAD=False)
        wait_for_condition(
            lambda: peltier.last_status.get("LNA_enabled") is True,
            cadence_ms=cadence,
        )
        assert peltier.last_status["LOAD_enabled"] is False
        peltier.disconnect()

    def test_enable_both_channels(self):
        """Enabling both channels is reflected in emulator status."""
        peltier = DummyPicoPeltier(port="/dev/ttyUSB0")
        cadence = peltier.EMULATOR_CADENCE_MS
        peltier.set_enable(LNA=True, LOAD=True)
        wait_for_condition(
            lambda: peltier.last_status.get("LNA_enabled") is True,
            cadence_ms=cadence,
        )
        assert peltier.last_status["LOAD_enabled"] is True
        peltier.disconnect()

    def test_disable_both_channels(self):
        """Disabling both channels is reflected in emulator status."""
        peltier = DummyPicoPeltier(port="/dev/ttyUSB0")
        cadence = peltier.EMULATOR_CADENCE_MS
        peltier.set_enable(LNA=False, LOAD=False)
        wait_for_condition(
            lambda: peltier.last_status.get("LNA_enabled") is False,
            cadence_ms=cadence,
        )
        assert peltier.last_status["LOAD_enabled"] is False
        peltier.disconnect()

    def test_status_populated_after_first_cycle(self):
        """First emulator status cycle populates the required peltier keys."""
        peltier = DummyPicoPeltier(port="/dev/ttyUSB0")
        wait_for_condition(
            lambda: peltier.last_status.get("sensor_name") == "tempctrl",
            cadence_ms=peltier.EMULATOR_CADENCE_MS,
        )
        assert "LNA_T_now" in peltier.last_status
        assert "LOAD_T_now" in peltier.last_status
        assert "LNA_drive_level" in peltier.last_status
        peltier.disconnect()


# ---------------------------------------------------------------------------
# MockSerial integration
# ---------------------------------------------------------------------------


class TestMockSerialIntegration:
    """Test low-level MockSerial operations through DummyPicoDevice.

    These tests verify that MockSerial read/write, readline, and buffer
    operations work correctly when wired through DummyPicoDevice.connect().
    """

    def test_write_and_read_through_peer(self):
        """Data written to ser appears in the peer's read buffer."""
        device = DummyPicoDevice(port="/dev/ttyUSB0")
        test_data = b"hello world\n"
        device.ser.write(test_data)
        assert device.ser.peer.read(len(test_data)) == test_data
        device.disconnect()

    def test_readline_multiple_lines(self):
        """readline() returns one line at a time from the peer's writes."""
        device = DummyPicoDevice(port="/dev/ttyUSB0")
        lines = [b"line1\n", b"line2\n", b"line3\n"]
        for line in lines:
            device.ser.peer.write(line)
        for expected in lines:
            assert device.ser.readline() == expected
        device.disconnect()

    def test_is_open_and_buffer_operations(self):
        """is_open, in_waiting, and reset_input_buffer work on MockSerial."""
        device = DummyPicoDevice(port="/dev/ttyUSB0")
        assert device.ser.is_open is True
        device.ser.reset_input_buffer()
        assert device.ser.in_waiting == 0
        device.disconnect()
