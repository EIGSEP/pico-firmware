"""
Unit tests for the picohost base classes.

Tests PicoDevice, PicoMotor, PicoRFSwitch, and PicoPeltier through their
DummyPico* wrappers, which use MockSerial + emulators instead of real hardware.
"""

import json
import pytest
from conftest import wait_for_condition, wait_for_settle
from picohost.base import PicoRFSwitch
from picohost.emulators import RFSwitchEmulator
from picohost.testing import (
    DummyPicoDevice,
    DummyPicoMotor,
    DummyPicoRFSwitch,
    DummyPicoPeltier,
    DummyPicoLidar,
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

    def test_attempt_reopen_fires_on_reconnect_on_success(self):
        """_attempt_reopen() runs the on_reconnect hook after a successful reopen.

        Both the public reconnect() and the reader thread's in-thread
        self-heal route through _attempt_reopen, so this is the choke
        point that keeps the post-open contract consistent between
        them.
        """
        device = DummyPicoDevice("/dev/dummy")
        try:
            calls = []
            device.on_reconnect = lambda: calls.append("hook")  # type: ignore[method-assign]
            device._open_serial = lambda: True  # type: ignore[method-assign]
            assert device._attempt_reopen() is True
            assert calls == ["hook"]
        finally:
            device.disconnect()

    def test_attempt_reopen_skips_on_reconnect_on_failure(self):
        """A failed reopen must not fire on_reconnect — there's no port yet."""
        device = DummyPicoDevice("/dev/dummy")
        try:
            calls = []
            device.on_reconnect = lambda: calls.append("hook")  # type: ignore[method-assign]
            device._open_serial = lambda: False  # type: ignore[method-assign]
            assert device._attempt_reopen() is False
            assert calls == []
        finally:
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

    def test_redis_handler_with_writer_is_bound_before_connect(self):
        """Configured metadata handler is available during connect()."""

        class FakeMetadataWriter:
            def add(self, _key, _value):
                pass

        class ProbePicoDevice(DummyPicoDevice):
            def connect(self):
                self.redis_handler_seen_in_connect = self.redis_handler
                return super().connect()

        device = ProbePicoDevice(
            "/dev/dummy", metadata_writer=FakeMetadataWriter()
        )
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
        expected_state = switch.paths["VNAO"]
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

    def test_paths_dict_values(self):
        """paths property maps names to the burned EEPROM addresses."""
        switch = DummyPicoRFSwitch("/dev/dummy")
        paths = switch.paths
        assert paths["RFANT"] == 0x00  # LNA -> Feed, fail-safe default
        assert paths["VNAO"] == 0x02  # VNA -> Cal Open
        assert paths["VNAS"] == 0x03  # VNA -> Cal Short
        assert paths["RFSP2"] == 0x0F  # last burned address
        switch.disconnect()

    def test_paths_within_firmware_range(self):
        """All path addresses fit the firmware's accepted range (0-15)."""
        for addr in DummyPicoRFSwitch.PATHS.values():
            assert 0 <= addr < RFSwitchEmulator.NUM_PATHS

    def test_paths_addresses_unique(self):
        """Each path name maps to a distinct EEPROM address."""
        addrs = list(DummyPicoRFSwitch.PATHS.values())
        assert len(addrs) == len(set(addrs))


class TestRFSwitchRedisHandler:
    """Verify _rfswitch_redis_handler augments the payload with sw_state_name.

    The published payload (what the base handler receives) must include a
    human-readable name for every known ``sw_state`` integer, and ``None``
    for integers not in :attr:`PicoRFSwitch.PATHS` (mid-switch, manual
    override, firmware bug). The published shape stays stable either way,
    and every added field must satisfy the scalar-only contract documented
    on :func:`picohost.base.redis_handler`.
    """

    _SCALAR_TYPES = (str, int, float, bool, type(None))

    def _capture(self, switch, data):
        """Run the redis handler against a given status dict and return
        what the base handler would receive."""
        captured = {}
        switch._base_redis_handler = lambda d: captured.update(d)
        switch._rfswitch_redis_handler(data)
        return captured

    def test_known_state_maps_to_name(self):
        """Every entry in PATHS round-trips via sw_state_name."""
        switch = DummyPicoRFSwitch("/dev/dummy")
        try:
            for name, sw_state in switch.paths.items():
                published = self._capture(
                    switch,
                    {"sensor_name": "rfswitch", "sw_state": sw_state},
                )
                assert published["sw_state"] == sw_state
                assert published["sw_state_name"] == name
        finally:
            switch.disconnect()

    def test_unknown_state_publishes_none_name(self):
        """Integers not in PATHS get sw_state_name = None."""
        switch = DummyPicoRFSwitch("/dev/dummy")
        try:
            unknown = max(switch.paths.values()) + 1
            assert unknown not in switch.paths.values()
            published = self._capture(
                switch,
                {"sensor_name": "rfswitch", "sw_state": unknown},
            )
            assert published["sw_state"] == unknown
            assert published["sw_state_name"] is None
        finally:
            switch.disconnect()

    def test_transition_sentinel_maps_to_unknown(self):
        """SW_STATE_UNKNOWN (-1) firmware sentinel publishes as "UNKNOWN"."""
        switch = DummyPicoRFSwitch("/dev/dummy")
        try:
            published = self._capture(
                switch,
                {
                    "sensor_name": "rfswitch",
                    "sw_state": switch.SW_STATE_UNKNOWN,
                },
            )
            assert published["sw_state"] == switch.SW_STATE_UNKNOWN
            assert published["sw_state_name"] == switch.SW_STATE_UNKNOWN_NAME
        finally:
            switch.disconnect()

    def test_missing_sw_state_does_not_crash(self):
        """A status dict without sw_state still publishes (name=None)."""
        switch = DummyPicoRFSwitch("/dev/dummy")
        try:
            published = self._capture(switch, {"sensor_name": "rfswitch"})
            assert published["sw_state_name"] is None
        finally:
            switch.disconnect()

    def test_handler_does_not_mutate_input(self):
        """The caller's dict is untouched by the handler."""
        switch = DummyPicoRFSwitch("/dev/dummy")
        try:
            data = {"sensor_name": "rfswitch", "sw_state": 0}
            self._capture(switch, data)
            assert data == {"sensor_name": "rfswitch", "sw_state": 0}
        finally:
            switch.disconnect()

    def test_published_dict_is_scalar_only(self):
        """Every value in the published dict is a permitted scalar type."""
        switch = DummyPicoRFSwitch("/dev/dummy")
        try:
            for sw_state in (0, 1, 255):
                published = self._capture(
                    switch,
                    {"sensor_name": "rfswitch", "sw_state": sw_state},
                )
                for k, v in published.items():
                    assert isinstance(v, self._SCALAR_TYPES), (
                        f"field {k!r} has non-scalar type {type(v).__name__}"
                    )
        finally:
            switch.disconnect()

    def test_published_shape_stable_across_state(self):
        """Field set is identical whether sw_state is known or unknown.

        Mirrors ``test_published_shape_stable_across_calibration_state``
        on the pot handler: the added key set must not depend on the
        value of the raw field, so downstream schemas can validate a
        single stable shape regardless of switch position.
        """
        switch = DummyPicoRFSwitch("/dev/dummy")
        try:
            known = set(
                self._capture(
                    switch, {"sensor_name": "rfswitch", "sw_state": 0}
                )
            )
            unknown_int = max(switch.paths.values()) + 1
            unknown = set(
                self._capture(
                    switch,
                    {"sensor_name": "rfswitch", "sw_state": unknown_int},
                )
            )
            assert known == unknown
            assert "sw_state_name" in known
        finally:
            switch.disconnect()


class TestRFSwitchThermistorFanout:
    """The three PCB thermistors fan out of the switch-state line into a
    separate rfswitch_therm stream (mirrors PicoLidar -> system_current),
    carrying raw volts + host-derived degrees C. The switch-state line
    stays a pure categorical signal (no thermistor keys)."""

    def _capture(self, switch, data):
        published = []
        switch._base_redis_handler = lambda d: published.append(dict(d))
        switch._rfswitch_redis_handler(data)
        return published

    def test_fans_thermistors_into_separate_stream(self):
        switch = DummyPicoRFSwitch("/dev/dummy")
        try:
            pub = self._capture(
                switch,
                {
                    "sensor_name": "rfswitch",
                    "status": "update",
                    "app_id": 5,
                    "sw_state": switch.PATHS["RFANT"],
                    "volt_therm0": 2.5,
                    "volt_therm1": 2.5,
                    "volt_therm2": 2.5,
                },
            )
            assert [p["sensor_name"] for p in pub] == [
                "rfswitch",
                "rfswitch_therm",
            ]
            # switch-state line is pure: thermistor keys stripped
            assert "volt_therm0" not in pub[0]
            assert pub[0]["sw_state_name"] == "RFANT"
            therm = pub[1]
            assert therm["status"] == "update"
            assert (
                "app_id" not in therm
            )  # fanned/derived stream, cf. system_current
            for i in range(3):
                assert therm[f"volt_therm{i}"] == 2.5
                # 2.5 V over the 5 V / 10k divider -> R = 10k -> 25 C
                assert therm[f"temp_therm{i}"] == pytest.approx(25.0, abs=0.05)
        finally:
            switch.disconnect()

    def test_out_of_range_voltage_maps_to_none_temp(self):
        switch = DummyPicoRFSwitch("/dev/dummy")
        try:
            pub = self._capture(
                switch,
                {
                    "sensor_name": "rfswitch",
                    "sw_state": switch.PATHS["RFANT"],
                    "volt_therm0": 0.0,  # v <= 0 (dead/short) -> None
                    "volt_therm1": 3.4,  # >= 3.3 ADC ceiling (saturated) -> None
                    "volt_therm2": 2.5,  # valid -> 25 C
                },
            )
            therm = pub[1]
            assert therm["temp_therm0"] is None
            assert therm["temp_therm1"] is None
            assert therm["temp_therm2"] == pytest.approx(25.0, abs=0.05)
            # raw volts always pass through, even when temp is None
            assert therm["volt_therm0"] == 0.0
        finally:
            switch.disconnect()

    def test_no_thermistor_fields_publishes_only_rfswitch(self):
        switch = DummyPicoRFSwitch("/dev/dummy")
        try:
            pub = self._capture(
                switch,
                {"sensor_name": "rfswitch", "sw_state": switch.PATHS["RFANT"]},
            )
            assert [p["sensor_name"] for p in pub] == ["rfswitch"]
        finally:
            switch.disconnect()

    def test_therm_temp_c_known_points(self):
        # 2.5 V -> R=10k -> exactly 25 C; lower V (hotter) -> higher C.
        assert PicoRFSwitch._therm_temp_c(2.5) == pytest.approx(25.0, abs=0.05)
        assert PicoRFSwitch._therm_temp_c(1.169) == pytest.approx(
            60.0, abs=1.0
        )
        # Just below the 3.3 V ADC ceiling is still a valid ~8.7 C reading;
        # at/above it the ADC has saturated (true voltage unknown) -> None.
        assert PicoRFSwitch._therm_temp_c(3.29) == pytest.approx(8.7, abs=0.3)
        assert PicoRFSwitch._therm_temp_c(3.3) is None
        assert PicoRFSwitch._therm_temp_c(3.4) is None
        assert PicoRFSwitch._therm_temp_c(0.0) is None
        assert PicoRFSwitch._therm_temp_c(5.0) is None
        assert PicoRFSwitch._therm_temp_c(None) is None


class TestMotorRedisHandler:
    """Verify _motor_redis_handler coerces position fields to float.

    The C firmware emits ``az_pos``/``el_pos``/``az_target_pos``/
    ``el_target_pos`` with ``KV_INT``, which the JSON parser surfaces
    as Python ``int``. The downstream consumer schema declares these
    as ``float`` so the per-integration reduction picks the
    float→mean policy (positions legitimately change within an
    integration during a scan). Coercing here at the publish boundary
    is what makes the producer satisfy that contract.
    """

    _POSITION_KEYS = ("az_pos", "az_target_pos", "el_pos", "el_target_pos")

    def _capture(self, motor, data):
        captured = {}
        motor._base_redis_handler = lambda d: captured.update(d)
        motor._motor_redis_handler(data)
        return captured

    def test_int_positions_are_published_as_float(self):
        motor = DummyPicoMotor("/dev/dummy")
        try:
            published = self._capture(
                motor,
                {
                    "sensor_name": "motor",
                    "status": "update",
                    "app_id": 0,
                    "az_pos": 100,
                    "az_target_pos": 200,
                    "el_pos": -50,
                    "el_target_pos": 0,
                },
            )
            for key in self._POSITION_KEYS:
                assert isinstance(published[key], float), (
                    f"{key} published as {type(published[key]).__name__}"
                )
            assert published["az_pos"] == 100.0
            assert published["el_pos"] == -50.0
        finally:
            motor.disconnect()

    def test_float_positions_pass_through(self):
        """Already-float values are not double-cast or otherwise mangled."""
        motor = DummyPicoMotor("/dev/dummy")
        try:
            published = self._capture(
                motor,
                {
                    "sensor_name": "motor",
                    "az_pos": 1.5,
                    "az_target_pos": 2.5,
                    "el_pos": 3.5,
                    "el_target_pos": 4.5,
                },
            )
            assert published["az_pos"] == 1.5
            assert published["el_pos"] == 3.5
        finally:
            motor.disconnect()

    def test_missing_position_keys_do_not_crash(self):
        """A partial payload (e.g. early status before all fields are
        populated) still publishes; absent keys stay absent."""
        motor = DummyPicoMotor("/dev/dummy")
        try:
            published = self._capture(
                motor, {"sensor_name": "motor", "az_pos": 0}
            )
            assert published["az_pos"] == 0.0
            assert "el_pos" not in published
        finally:
            motor.disconnect()

    def test_none_positions_pass_through(self):
        """None gap-fill values are preserved (not coerced to 0.0)."""
        motor = DummyPicoMotor("/dev/dummy")
        try:
            published = self._capture(
                motor,
                {
                    "sensor_name": "motor",
                    "az_pos": None,
                    "az_target_pos": None,
                    "el_pos": None,
                    "el_target_pos": None,
                },
            )
            for key in self._POSITION_KEYS:
                assert published[key] is None
        finally:
            motor.disconnect()

    def test_handler_does_not_mutate_input(self):
        """The caller's dict is untouched by the handler."""
        motor = DummyPicoMotor("/dev/dummy")
        try:
            data = {"sensor_name": "motor", "az_pos": 7, "el_pos": 9}
            self._capture(motor, data)
            assert data == {"sensor_name": "motor", "az_pos": 7, "el_pos": 9}
            assert isinstance(data["az_pos"], int)
        finally:
            motor.disconnect()

    def test_emulator_payload_publishes_float_positions(self):
        """End-to-end: a fresh MotorEmulator status, run through the
        handler, must publish all position fields as floats. Catches
        regressions where the C firmware adds a new position field that
        bypasses the cast."""
        from picohost.emulators import MotorEmulator

        motor = DummyPicoMotor("/dev/dummy")
        try:
            published = self._capture(motor, MotorEmulator().get_status())
            for key in self._POSITION_KEYS:
                assert isinstance(published[key], float), (
                    f"{key} published as {type(published[key]).__name__}"
                )
        finally:
            motor.disconnect()


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
            "LNA_Kp",
            "LNA_Ki",
            "LNA_integral",
        ):
            assert key in peltier.last_status, f"Missing key: {key}"
        peltier.disconnect()

    def test_set_gains_round_trip(self):
        """set_gains() updates emulator Kp/Ki and surfaces in status."""
        peltier = DummyPicoPeltier("/dev/dummy")
        cadence = peltier.EMULATOR_CADENCE_MS
        before = peltier.last_status.get("LNA_Kp")
        peltier.set_gains(LNA_Kp=0.4, LNA_Ki=0.02, LOAD_Kp=0.5)
        assert wait_for_settle(
            lambda: peltier.last_status.get("LNA_Kp"),
            initial=before,
            cadence_ms=cadence,
            max_cycles=10,
        ) == pytest.approx(0.4)
        assert peltier.last_status.get("LNA_Ki") == pytest.approx(0.02)
        assert peltier.last_status.get("LOAD_Kp") == pytest.approx(0.5)
        peltier.disconnect()

    def test_reset_integral_sends_only_selected_channels(self):
        """reset_integral() sends an *_integral_reset only for chosen channels."""
        peltier = DummyPicoPeltier("/dev/dummy", keepalive_interval=0)
        try:
            sent = []
            original = peltier.send_command

            def spy(cmd):
                sent.append(dict(cmd))
                return original(cmd)

            peltier.send_command = spy  # type: ignore[method-assign]
            peltier.reset_integral(LNA=True)
            assert sent[-1] == {"LNA_integral_reset": True}
            peltier.reset_integral(LNA=True, LOAD=True)
            assert sent[-1] == {
                "LNA_integral_reset": True,
                "LOAD_integral_reset": True,
            }
            before = len(sent)
            peltier.reset_integral()  # default: both False → no-op
            assert len(sent) == before
        finally:
            peltier.disconnect()

    def test_reset_integral_not_cached_for_replay(self):
        """One-shot command; firmware reset clears the integral implicitly."""
        peltier = DummyPicoPeltier("/dev/dummy", keepalive_interval=0)
        try:
            peltier.reset_integral(LNA=True, LOAD=True)
            sent = []
            original = peltier.send_command

            def spy(cmd):
                sent.append(dict(cmd))
                return original(cmd)

            peltier.send_command = spy  # type: ignore[method-assign]
            peltier.on_reconnect()
            for payload in sent:
                assert "LNA_integral_reset" not in payload
                assert "LOAD_integral_reset" not in payload
        finally:
            peltier.disconnect()


class TestPicoPeltierReconnectReplay:
    """Cache last-applied config in setters; replay in on_reconnect.

    A serial-link drop on EIGSEP picos is our proxy for "firmware may
    have rebooted" (hard watchdog, brownout, picotool re-flash all drop
    USB CDC). ``on_reconnect`` replays cached setpoints/clamps/enable
    flags so the firmware doesn't silently resume at defaults.
    """

    def test_fresh_instance_has_empty_cache(self):
        peltier = DummyPicoPeltier("/dev/dummy")
        try:
            assert peltier._last_watchdog_timeout_ms is None
            assert peltier._last_clamp == {}
            assert peltier._last_cooling == {}
            assert peltier._last_gains == {}
            assert peltier._last_temperature == {}
            assert peltier._last_enable is None
        finally:
            peltier.disconnect()

    def test_setters_populate_cache(self):
        peltier = DummyPicoPeltier("/dev/dummy")
        try:
            peltier.set_watchdog_timeout(15000)
            peltier.set_clamp(LNA=0.5, LOAD=0.6)
            peltier.set_cooling_enabled(LNA=False, LOAD=True)
            peltier.set_gains(LNA_Kp=0.3, LNA_Ki=0.02, LOAD_Kp=0.25)
            peltier.set_temperature(T_LNA=25.0, LNA_hyst=0.3)
            peltier.set_enable(LNA=True, LOAD=False)
            assert peltier._last_watchdog_timeout_ms == 15000
            assert peltier._last_clamp == {
                "LNA_clamp": 0.5,
                "LOAD_clamp": 0.6,
            }
            assert peltier._last_cooling == {
                "LNA_cooling_enabled": False,
                "LOAD_cooling_enabled": True,
            }
            assert peltier._last_gains == {
                "LNA_Kp": 0.3,
                "LNA_Ki": 0.02,
                "LOAD_Kp": 0.25,
            }
            assert peltier._last_temperature == {
                "LNA_temp_target": 25.0,
                "LNA_hysteresis": 0.3,
            }
            assert peltier._last_enable == {
                "LNA_enable": True,
                "LOAD_enable": False,
            }
        finally:
            peltier.disconnect()

    def test_partial_updates_merge_across_channels(self):
        """Setting one channel, then the other, keeps both in the cache.

        Firmware holds per-channel state independently, so a replay must
        restore both channels even if the host only ever set them in
        separate calls.
        """
        peltier = DummyPicoPeltier("/dev/dummy")
        try:
            peltier.set_clamp(LNA=0.4)
            peltier.set_clamp(LOAD=0.7)
            assert peltier._last_clamp == {
                "LNA_clamp": 0.4,
                "LOAD_clamp": 0.7,
            }
            peltier.set_temperature(T_LNA=20.0, LNA_hyst=0.2)
            peltier.set_temperature(T_LOAD=30.0, LOAD_hyst=0.5)
            assert peltier._last_temperature == {
                "LNA_temp_target": 20.0,
                "LNA_hysteresis": 0.2,
                "LOAD_temp_target": 30.0,
                "LOAD_hysteresis": 0.5,
            }
        finally:
            peltier.disconnect()

    def test_on_reconnect_replays_in_safe_order(self):
        """watchdog → clamp → cooling_enabled → gains → temperature → enable.

        cooling_enabled lands between clamp and gains so the asymmetric-
        clamp safety setting is in place before any drive can result
        from the next setpoint. Gains land before temperature so the
        channel is fully tuned the instant it goes active. Disables
        keepalive so the spy only sees replay traffic — the background
        ``{}`` keepalive is tested independently.
        """
        peltier = DummyPicoPeltier("/dev/dummy", keepalive_interval=0)
        try:
            peltier.set_watchdog_timeout(15000)
            peltier.set_clamp(LNA=0.5, LOAD=0.6)
            peltier.set_cooling_enabled(LNA=False, LOAD=True)
            peltier.set_gains(LNA_Kp=0.25, LNA_Ki=0.01)
            peltier.set_temperature(
                T_LNA=25.0, LNA_hyst=0.3, T_LOAD=28.0, LOAD_hyst=0.4
            )
            peltier.set_enable(LNA=True, LOAD=False)

            sent = []
            original = peltier.send_command

            def spy(cmd):
                sent.append(dict(cmd))
                return original(cmd)

            peltier.send_command = spy  # type: ignore[method-assign]
            peltier.on_reconnect()

            assert sent == [
                {"watchdog_timeout_ms": 15000},
                {"LNA_clamp": 0.5, "LOAD_clamp": 0.6},
                {
                    "LNA_cooling_enabled": False,
                    "LOAD_cooling_enabled": True,
                },
                {"LNA_Kp": 0.25, "LNA_Ki": 0.01},
                {
                    "LNA_temp_target": 25.0,
                    "LNA_hysteresis": 0.3,
                    "LOAD_temp_target": 28.0,
                    "LOAD_hysteresis": 0.4,
                },
                {"LNA_enable": True, "LOAD_enable": False},
            ]
        finally:
            peltier.disconnect()

    def test_set_cooling_enabled_partial_no_command(self):
        """``set_cooling_enabled()`` with both args None must not touch
        the wire or the replay cache — matches the shape shared by
        ``set_temperature`` / ``set_clamp`` / ``set_gains`` / and
        ``reset_integral``."""
        peltier = DummyPicoPeltier("/dev/dummy", keepalive_interval=0)
        try:
            sent = []
            original = peltier.send_command

            def spy(cmd):
                sent.append(dict(cmd))
                return original(cmd)

            peltier.send_command = spy  # type: ignore[method-assign]
            peltier.set_cooling_enabled()
            assert sent == []
            assert peltier._last_cooling == {}
        finally:
            peltier.disconnect()

    def test_set_cooling_enabled_round_trip(self):
        """Single-channel set forwards just that channel and updates the
        cache; second call on the other channel merges into the cache."""
        peltier = DummyPicoPeltier("/dev/dummy", keepalive_interval=0)
        try:
            peltier.set_cooling_enabled(LNA=False)
            assert peltier._last_cooling == {"LNA_cooling_enabled": False}
            peltier.set_cooling_enabled(LOAD=True)
            assert peltier._last_cooling == {
                "LNA_cooling_enabled": False,
                "LOAD_cooling_enabled": True,
            }
        finally:
            peltier.disconnect()

    def test_on_reconnect_skips_unset_groups(self):
        """Groups never configured by the host aren't replayed."""
        peltier = DummyPicoPeltier("/dev/dummy", keepalive_interval=0)
        try:
            peltier.set_clamp(LNA=0.5)

            sent = []
            original = peltier.send_command

            def spy(cmd):
                sent.append(dict(cmd))
                return original(cmd)

            peltier.send_command = spy  # type: ignore[method-assign]
            peltier.on_reconnect()

            assert sent == [{"LNA_clamp": 0.5}]
        finally:
            peltier.disconnect()

    def test_on_reconnect_restarts_keepalive(self):
        """Keepalive thread is re-armed even if no config was ever set."""
        peltier = DummyPicoPeltier("/dev/dummy")
        try:
            peltier._keepalive_running = False
            peltier._keepalive_thread = None
            peltier.on_reconnect()
            assert peltier._keepalive_running is True
            assert peltier._keepalive_thread is not None
        finally:
            peltier.disconnect()

    def test_reader_thread_self_heal_replays_config(self):
        """Reader-thread reconnect replays cached config, not just reopen.

        Regression guard: before _attempt_reopen existed, the reader
        thread's self-heal called _open_serial directly and skipped
        on_reconnect. A Pico that rebooted and re-enumerated faster
        than PicoManager's health check was silently left at firmware
        defaults.
        """
        peltier = DummyPicoPeltier("/dev/dummy", keepalive_interval=0)
        try:
            peltier.set_watchdog_timeout(15000)
            peltier.set_clamp(LNA=0.5, LOAD=0.6)
            peltier.set_enable(LNA=True, LOAD=False)

            sent = []
            original = peltier.send_command

            def spy(cmd):
                sent.append(dict(cmd))
                return original(cmd)

            peltier.send_command = spy  # type: ignore[method-assign]
            peltier._open_serial = lambda: True  # type: ignore[method-assign]

            # _attempt_reopen is exactly what the reader thread calls on
            # a drop (see base.py::_reader_thread_func).
            assert peltier._attempt_reopen() is True

            assert {"watchdog_timeout_ms": 15000} in sent
            assert {"LNA_clamp": 0.5, "LOAD_clamp": 0.6} in sent
            assert {"LNA_enable": True, "LOAD_enable": False} in sent
        finally:
            peltier.disconnect()


class TestPicoPeltierRedisHandler:
    """Verify _peltier_redis_handler fans the firmware tick into two streams.

    The firmware emits one combined dict per status tick (sensor_name
    "tempctrl", flat LNA_*/LOAD_* fields, device-wide watchdog_*). The
    handler must publish two streams ("tempctrl_lna" and "tempctrl_load"),
    each carrying a top-level ``status`` derived from the matching
    ``{channel}_status`` and the device-wide watchdog fields duplicated.
    """

    _SCALAR_TYPES = (str, int, float, bool, type(None))

    _SAMPLE = {
        "sensor_name": "tempctrl",
        "app_id": 1,
        "watchdog_tripped": False,
        "watchdog_timeout_ms": 30000,
        "LNA_status": "update",
        "LNA_T_now": 25.4,
        "LNA_voltage": 2.51,
        "LNA_resistance": 37200.0,
        "LNA_timestamp": 750.0,
        "LNA_T_target": 25.0,
        "LNA_drive_level": 0.42,
        "LNA_enabled": True,
        "LNA_active": True,
        "LNA_int_disabled": False,
        "LNA_stall_tripped": False,
        "LNA_cooling_enabled": True,
        "LNA_hysteresis": 0.5,
        "LNA_clamp": 0.8,
        "LNA_Kp": 0.2,
        "LNA_Ki": 0.01,
        "LNA_integral": 1.25,
        "LOAD_status": "error",
        "LOAD_T_now": None,
        "LOAD_voltage": 0.0,
        "LOAD_resistance": 0.0,
        "LOAD_timestamp": 750.0,
        "LOAD_T_target": 25.0,
        "LOAD_drive_level": 0.0,
        "LOAD_enabled": True,
        "LOAD_active": False,
        "LOAD_int_disabled": True,
        "LOAD_stall_tripped": False,
        "LOAD_cooling_enabled": False,
        "LOAD_hysteresis": 0.5,
        "LOAD_clamp": 0.8,
        "LOAD_Kp": 0.25,
        "LOAD_Ki": 0.0,
        "LOAD_integral": 0.0,
    }

    _EXPECTED_KEYS = {
        "sensor_name",
        "app_id",
        "status",
        "watchdog_tripped",
        "watchdog_timeout_ms",
        "T_now",
        "voltage",
        "resistance",
        "timestamp",
        "T_target",
        "drive_level",
        "enabled",
        "active",
        "int_disabled",
        "stall_tripped",
        "cooling_enabled",
        "hysteresis",
        "clamp",
        "Kp",
        "Ki",
        "integral",
    }

    def _capture_all(self, peltier, data):
        captured = []
        peltier._base_redis_handler = (
            lambda d: captured.append(dict(d))  # type: ignore[method-assign]
        )
        peltier._peltier_redis_handler(data)
        return captured

    def test_publishes_two_streams_per_tick(self):
        peltier = DummyPicoPeltier("/dev/dummy")
        try:
            published = self._capture_all(peltier, self._SAMPLE)
            assert len(published) == 2
            names = [p["sensor_name"] for p in published]
            assert names == ["tempctrl_lna", "tempctrl_load"]
        finally:
            peltier.disconnect()

    def test_channel_status_derived_from_prefixed_status(self):
        peltier = DummyPicoPeltier("/dev/dummy")
        try:
            lna, load = self._capture_all(peltier, self._SAMPLE)
            assert lna["status"] == "update"
            assert load["status"] == "error"
        finally:
            peltier.disconnect()

    def test_channel_fields_have_prefix_stripped(self):
        peltier = DummyPicoPeltier("/dev/dummy")
        try:
            lna, load = self._capture_all(peltier, self._SAMPLE)
            assert lna["T_now"] == pytest.approx(25.4)
            assert lna["voltage"] == pytest.approx(2.51)
            assert lna["resistance"] == pytest.approx(37200.0)
            assert lna["drive_level"] == pytest.approx(0.42)
            assert lna["active"] is True
            assert lna["int_disabled"] is False
            assert load["T_now"] is None
            assert load["active"] is False
            assert load["int_disabled"] is True
        finally:
            peltier.disconnect()

    def test_watchdog_fields_duplicated(self):
        peltier = DummyPicoPeltier("/dev/dummy")
        try:
            lna, load = self._capture_all(peltier, self._SAMPLE)
            for entry in (lna, load):
                assert entry["watchdog_tripped"] is False
                assert entry["watchdog_timeout_ms"] == 30000
                assert entry["app_id"] == 1
        finally:
            peltier.disconnect()

    def test_published_shape_stable_across_channel_state(self):
        """Both streams expose the same key set regardless of values."""
        peltier = DummyPicoPeltier("/dev/dummy")
        try:
            lna, load = self._capture_all(peltier, self._SAMPLE)
            assert set(lna) == self._EXPECTED_KEYS
            assert set(load) == self._EXPECTED_KEYS
        finally:
            peltier.disconnect()

    def test_published_dict_is_scalar_only(self):
        peltier = DummyPicoPeltier("/dev/dummy")
        try:
            for entry in self._capture_all(peltier, self._SAMPLE):
                for k, v in entry.items():
                    assert isinstance(v, self._SCALAR_TYPES), (
                        f"field {k!r} has non-scalar type {type(v).__name__}"
                    )
        finally:
            peltier.disconnect()

    def test_handler_does_not_mutate_input(self):
        peltier = DummyPicoPeltier("/dev/dummy")
        try:
            data = dict(self._SAMPLE)
            original = dict(self._SAMPLE)
            self._capture_all(peltier, data)
            assert data == original
        finally:
            peltier.disconnect()

    def test_missing_fields_publish_none(self):
        """Channel status absent from the source dict publishes None."""
        peltier = DummyPicoPeltier("/dev/dummy")
        try:
            partial = {"sensor_name": "tempctrl", "app_id": 1}
            lna, load = self._capture_all(peltier, partial)
            assert lna["status"] is None
            assert load["status"] is None
            assert lna["T_now"] is None
            assert lna["watchdog_tripped"] is None
            assert lna["watchdog_timeout_ms"] is None
        finally:
            peltier.disconnect()


class TestLidarRedisHandler:
    """PicoLidar fans the merged lidar status line into two metadata
    publishes: distance under 'lidar', current under 'system_current'.

    The current entry is decoupled from the lidar I2C status. current_a and
    the cal scalars (current_cal_slope A/V, current_cal_intercept A) come from
    the measured two-point cal stored in amps-vs-volts form; all three are None
    when no cal is loaded (no nominal fallback). Every added field is a scalar
    (scalar-only contract on picohost.base.redis_handler).
    """

    # Measured cal stored in the amps-vs-volts form (slope A/V, intercept A).
    _CAL = (8.4223, -12.5248)

    def _capture(self, lidar, data):
        published = []
        lidar._base_redis_handler = lambda d: published.append(dict(d))
        lidar._lidar_redis_handler(data)
        return published

    def test_splits_into_lidar_and_system_current_uncalibrated(self):
        """No measured cal: current_a and both cal fields are None; the raw
        current_voltage still rides along on its own system_current entry."""
        lidar = DummyPicoLidar("/dev/dummy")
        try:
            assert lidar._current_cal is None  # boots uncalibrated
            pub = self._capture(
                lidar,
                {
                    "sensor_name": "lidar",
                    "status": "update",
                    "app_id": 4,
                    "distance_m": 1.23,
                    "current_voltage": 0.7057,
                },
            )
            assert [p["sensor_name"] for p in pub] == [
                "lidar",
                "system_current",
            ]
            assert pub[0]["distance_m"] == 1.23
            assert "current_voltage" not in pub[0]
            sc = pub[1]
            assert sc["current_voltage"] == 0.7057
            assert sc["current_a"] is None
            assert sc["current_cal_slope"] is None
            assert sc["current_cal_intercept"] is None
        finally:
            lidar.disconnect()

    def test_calibrated_publishes_cal_and_invariant(self):
        """With a measured cal loaded the handler passes the stored
        amps-vs-volts pair through and current_a == slope*V + intercept."""
        lidar = DummyPicoLidar("/dev/dummy")
        try:
            lidar._current_cal = self._CAL
            cal_slope, cal_intercept = self._CAL
            v_adc = 1.84
            pub = self._capture(
                lidar,
                {
                    "sensor_name": "lidar",
                    "status": "update",
                    "app_id": 4,
                    "distance_m": 0.0,
                    "current_voltage": v_adc,
                },
            )
            sc = pub[1]
            assert sc["current_cal_slope"] == cal_slope
            assert sc["current_cal_intercept"] == cal_intercept
            assert sc["current_a"] == pytest.approx(
                cal_slope * v_adc + cal_intercept
            )
        finally:
            lidar.disconnect()

    def test_current_status_decoupled_from_lidar_error(self):
        """A lidar I2C error does not taint the current half's status."""
        lidar = DummyPicoLidar("/dev/dummy")
        try:
            lidar._current_cal = self._CAL
            pub = self._capture(
                lidar,
                {
                    "sensor_name": "lidar",
                    "status": "error",
                    "app_id": 4,
                    "distance_m": 9.9,
                    "current_voltage": 1.60,
                },
            )
            assert pub[0]["status"] == "error"
            assert pub[1]["status"] == "update"
            assert pub[1]["current_a"] is not None
        finally:
            lidar.disconnect()

    def test_no_current_field_publishes_only_lidar(self):
        """A status line without current_voltage (e.g. pre-feature firmware)
        publishes only the lidar entry — no system_current."""
        lidar = DummyPicoLidar("/dev/dummy")
        try:
            pub = self._capture(
                lidar,
                {
                    "sensor_name": "lidar",
                    "status": "update",
                    "app_id": 4,
                    "distance_m": 2.0,
                },
            )
            assert [p["sensor_name"] for p in pub] == ["lidar"]
        finally:
            lidar.disconnect()
