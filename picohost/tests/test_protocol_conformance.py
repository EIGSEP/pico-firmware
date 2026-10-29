"""
Protocol conformance tests for pico firmware emulators.

Each test documents the exact command/response contract derived from reading
the C firmware source.  These run against emulators in CI and serve as a
reference for manual hardware verification.

See CLAUDE.md § Command Protocol for the general framing.
"""

import json

import pytest

from picohost.emulators import (
    MotorEmulator,
    TempCtrlEmulator,
    ImuEmulator,
    LidarEmulator,
    PotMonEmulator,
    RFSwitchEmulator,
    LnaMonEmulator,
)
from picohost.emulators.tempctrl import MAX_REJECTS

# ---------------------------------------------------------------------------
# Base protocol tests (apply to all apps)
# ---------------------------------------------------------------------------


class TestBaseProtocol:
    """Behaviour defined by main.c's command loop, shared by all apps."""

    def test_malformed_json_silently_ignored(self):
        """C firmware: cJSON_Parse returns NULL → function returns early.

        Feed malformed and valid JSON lines into _cmd_buffer and use a
        minimal peer so _read_commands() processes the buffer through the
        real JSONDecodeError path.
        """

        class _NullPeer:
            """Peer with nothing to read — lets _read_commands() reach the
            buffer-processing loop without providing new data."""

            @property
            def in_waiting(self):
                return 0

        emu = MotorEmulator()
        emu.attach(_NullPeer())
        emu.server({"az_set_target_pos": 500})
        initial_pos = emu.azimuth.target_pos

        # Malformed JSON (missing closing brace) — should be silently ignored.
        emu._cmd_buffer = '{"az_set_target_pos": 999\n'
        emu._read_commands()
        assert emu.azimuth.target_pos == initial_pos

        # Valid command after the bad one should still work.
        emu._cmd_buffer = '{"az_set_target_pos": 700}\n'
        emu._read_commands()
        assert emu.azimuth.target_pos == 700

    def test_no_command_acknowledgment(self):
        """No app sends a response when a command is received.

        All feedback comes via periodic status messages.
        """
        # Emulators only produce output via get_status(); server() returns
        # None for all emulators.
        for Cls in (
            MotorEmulator,
            TempCtrlEmulator,
            ImuEmulator,
            LidarEmulator,
            PotMonEmulator,
            RFSwitchEmulator,
            LnaMonEmulator,
        ):
            kwargs = {"settle_ms": 0} if Cls is RFSwitchEmulator else {}
            emu = Cls(**kwargs)
            result = emu.server({})
            assert result is None

    def test_status_json_is_compact(self):
        """C firmware uses cJSON_PrintUnformatted → no spaces."""
        emu = MotorEmulator()
        status = emu.get_status()
        rendered = json.dumps(status, separators=(",", ":"))
        assert " " not in rendered  # no gratuitous whitespace

    def test_empty_json_is_noop(self):
        """All apps handle {} without side effects."""
        emulators = [
            MotorEmulator(),
            TempCtrlEmulator(),
            ImuEmulator(),
            LidarEmulator(),
            PotMonEmulator(),
            RFSwitchEmulator(settle_ms=0),
            LnaMonEmulator(),
        ]
        for emu in emulators:
            before = emu.get_status()
            emu.server({})
            after = emu.get_status()
            # Status should be unchanged (timestamps may differ for tempctrl)
            for key in before:
                if "timestamp" not in key:
                    assert before[key] == after[key], (
                        f"{type(emu).__name__}: {key} changed after empty cmd"
                    )


# ---------------------------------------------------------------------------
# Motor protocol (src/motor.c)
# ---------------------------------------------------------------------------


class TestMotorProtocol:
    """Protocol conformance tests for APP_MOTOR (app_id=0).

    Command processing order in motor_server():
      1. az_set_pos / el_set_pos  (resets both position and target)
      2. az_set_target_pos / el_set_target_pos  (overrides target only)
      3. halt  (sets target = current position for both axes)
      4. delay settings
    """

    def test_sensor_name(self):
        assert MotorEmulator().get_status()["sensor_name"] == "motor"

    def test_app_id(self):
        emu = MotorEmulator(app_id=0)
        assert emu.get_status()["app_id"] == 0

    def test_set_pos_resets_target(self):
        """motor.c line 136-138: position AND target_pos both get set."""
        emu = MotorEmulator()
        emu.server({"az_set_pos": 999})
        assert emu.azimuth.position == 999
        assert emu.azimuth.target_pos == 999

    def test_processing_order_set_pos_then_target(self):
        """motor.c processes set_pos before set_target_pos."""
        emu = MotorEmulator()
        emu.server({"az_set_pos": 100, "az_set_target_pos": 200})
        assert emu.azimuth.position == 100
        assert emu.azimuth.target_pos == 200

    def test_halt_checks_key_presence_not_value(self):
        """motor.c line 149: cJSON_GetObjectItem checks existence only."""
        emu = MotorEmulator()
        emu.server({"az_set_target_pos": 1000})
        for _ in range(3):
            emu.op()
        # halt with value 0, False, None — all should still halt
        for val in (0, False, None, ""):
            emu.server({"az_set_target_pos": 1000})
            for _ in range(3):
                emu.op()
            emu.server({"halt": val})
            assert emu.azimuth.target_pos == emu.azimuth.position

    def test_stepper_convergence(self):
        """Motor moves max_pulses=60 steps per op() call toward target."""
        emu = MotorEmulator()
        emu.server({"az_set_target_pos": 120})
        emu.op()
        assert emu.azimuth.position == 60  # first call: min(60, 120) = 60
        emu.op()
        assert emu.azimuth.position == 120  # second call: min(60, 60) = 60

    def test_direction_tracking(self):
        """steps_in_direction resets on direction change."""
        emu = MotorEmulator()
        emu.server({"az_set_target_pos": 100})
        emu.op()
        assert emu.azimuth.dir == 1
        assert emu.azimuth.steps_in_direction == 60

        emu.server({"az_set_target_pos": 0})
        emu.op()
        assert emu.azimuth.dir == -1
        assert emu.azimuth.steps_in_direction == 60  # reset and counted


# ---------------------------------------------------------------------------
# TempCtrl protocol (src/tempctrl.c)
# ---------------------------------------------------------------------------


class TestTempCtrlProtocol:
    """Protocol conformance tests for APP_TEMPCTRL (app_id=1)."""

    def test_sensor_name(self):
        assert TempCtrlEmulator().get_status()["sensor_name"] == "tempctrl"

    def test_defaults_match_firmware(self):
        """LoadHeaterState defaults (tempctrl.h/tempctrl.c load_heater_init)."""
        emu = TempCtrlEmulator()
        assert emu.load.T_target == 30.0
        assert emu.load.hysteresis == 2.0
        assert emu.load.enabled is False
        assert emu.load.installed is True
        # LOAD is a single low-side FET heater under on/off hysteresis —
        # no PI loop, so no Kp/Ki/clamp/integral fields exist at all.
        assert not hasattr(emu.load, "Kp")
        assert not hasattr(emu.load, "clamp")

    def test_enable_via_int(self):
        """tempctrl.c: valueint ? true : false."""
        emu = TempCtrlEmulator()
        emu.server({"LOAD_enable": 1})
        assert emu.load.enabled is True
        emu.server({"LOAD_enable": 0})
        assert emu.load.enabled is False

    def test_hysteresis_band_stays_inactive(self):
        """Inside the hysteresis band: the FET stays off."""
        emu = TempCtrlEmulator()
        emu.load.T_now = 29.8
        emu.server(
            {
                "LOAD_temp_target": 30.0,
                "LOAD_enable": True,
                "LOAD_hysteresis": 0.5,
            }
        )
        for _ in range(2):  # two-to-anchor
            emu.op()
        assert emu.load.active is False

    def test_sensor_error_disables_drive(self):
        """tempctrl.c: a plausibility-failed cycle gates drive off (the
        sensor may be gone entirely)."""
        emu = TempCtrlEmulator()
        emu.server({"LOAD_enable": True, "LOAD_temp_target": 50.0})
        # Two samples: the first seeds the candidate reference
        # (two-to-anchor); control engages on the second, mirroring
        # firmware.
        for _ in range(2):
            emu.op()
        assert emu.load.active is True
        emu.inject_sensor_error()
        emu.op()
        assert emu.load.active is False

    def test_kp_ki_keys_are_silently_ignored(self):
        """LOAD_Kp / LOAD_Ki update state.

        LOAD is a single low-side FET heater under on/off hysteresis, not
        a PI loop, so it has no Kp/Ki at all — LOAD_Kp/LOAD_Ki are
        silently ignored (mirrors firmware tempctrl_server, which never
        parses these keys for LOAD).
        """
        emu = TempCtrlEmulator()
        emu.server({"LOAD_Kp": 0.4, "LOAD_Ki": 0.05})
        assert not hasattr(emu.load, "Kp")
        assert not hasattr(emu.load, "Ki")

    def test_sensor_error_status_field(self):
        """tempctrl.c: per-channel status reports data validity — a
        plausibility failure errors the channel and nulls the derived
        values while the raw voltage stays reported."""
        emu = TempCtrlEmulator()
        emu.op()
        assert emu.get_status()["LOAD_status"] == "update"
        emu.inject_sensor_error()
        emu.op()
        status = emu.get_status()
        assert status["LOAD_status"] == "error"
        assert status["LOAD_T_now"] is None
        assert status["LOAD_resistance"] is None
        assert isinstance(status["LOAD_voltage"], float)

    def test_rate_reject_still_reports_measured_value(self):
        """tempctrl.c: the rate guard is control-only. A rejected sample
        was still a plausible ADC conversion, so the cycle reports it
        (status "update", raw value in T_now) with the published
        sensor_rejects counter as the cross-check marker; only the
        plausibility chain — no measurement exists — errors the stream.
        The firmware never withholds science data on rate statistics
        alone; downstream owns that call."""
        emu = TempCtrlEmulator()
        emu.load.thermal_frozen = True
        emu.op()
        emu.op()  # two-to-anchor at the model temperature (25.0)
        emu.load.inject_sensor_glitch(90.0)
        emu.op()
        status = emu.get_status()
        assert status["LOAD_status"] == "update"
        assert status["LOAD_T_now"] == pytest.approx(90.0)
        assert status["LOAD_sensor_rejects"] == 1
        # Control held the last good reference — the raw value is
        # reported, never acted on.
        assert emu.load.T_now == pytest.approx(25.0)

    def test_installed_default_true_and_in_status(self):
        """LoadHeaterState defaults installed=true; tempctrl_status
        reports LOAD_installed on every tick."""
        emu = TempCtrlEmulator()
        assert emu.load.installed is True
        status = emu.get_status()
        assert status["LOAD_installed"] is True

    def test_installed_parse_via_int(self):
        """tempctrl.c: item_json->valueint ? true : false — a JSON string
        like "false" has valueint 0, so it uninstalls (cJSON parity)."""
        emu = TempCtrlEmulator()
        emu.server({"LOAD_installed": 0})
        assert emu.load.installed is False
        emu.server({"LOAD_installed": 1})
        assert emu.load.installed is True
        emu.server({"LOAD_installed": "false"})
        assert emu.load.installed is False

    def test_uninstalled_channel_skips_sampling_and_drive(self):
        """load_heater_update: an uninstalled channel skips the read and
        never drives: data_invalid every cycle, controller state reset,
        drive forced off."""
        emu = TempCtrlEmulator()
        emu.server({"LOAD_enable": True, "LOAD_temp_target": 50.0})
        for _ in range(2):  # two-to-anchor, then control engages
            emu.op()
        assert emu.load.active is True
        emu.server({"LOAD_installed": 0})
        emu.op()
        assert emu.load.active is False
        assert emu.load.data_invalid is True
        status = emu.get_status()
        assert status["LOAD_status"] == "error"
        assert status["LOAD_T_now"] is None

    def test_uninstalled_drops_anchor_reinstall_reseeds(self):
        """Uninstalling drops the rate anchor (it is only valid across
        continuous good data); re-install re-seeds two-to-anchor before
        control resumes, mirroring recovery from a plausibility outage."""
        emu = TempCtrlEmulator()
        emu.server({"LOAD_enable": True, "LOAD_temp_target": 50.0})
        for _ in range(2):
            emu.op()
        assert emu.load.rate_ref_valid is True
        emu.server({"LOAD_installed": 0})
        emu.op()
        assert emu.load.rate_ref_valid is False
        emu.server({"LOAD_installed": 1})
        emu.op()  # candidate seed only — control still gated
        assert emu.load.active is False
        emu.op()  # anchor confirmed — control resumes
        assert emu.load.active is True

    def test_uninstall_preserves_latches_enable_ack_clears(self):
        """Uninstalling is not a trip ack: sticky latches survive it and
        clear only via LOAD_enable=true (load_heater_apply_enable)."""
        emu = TempCtrlEmulator()
        emu.server({"LOAD_enable": True})
        for _ in range(2):
            emu.op()
        emu.load.inject_sensor_glitch(1000.0, count=MAX_REJECTS)
        for _ in range(MAX_REJECTS):
            emu.op()
        assert emu.load.sensor_tripped is True
        emu.server({"LOAD_installed": 0})
        emu.op()
        assert emu.load.sensor_tripped is True
        emu.server({"LOAD_enable": True})
        assert emu.load.sensor_tripped is False


# ---------------------------------------------------------------------------
# IMU protocol (src/imu.c — UART RVC mode)
# ---------------------------------------------------------------------------


class TestImuProtocol:
    """Protocol conformance tests for APP_IMU_EL / APP_IMU_AZ."""

    def test_name_for_app_imu_el(self):
        """imu.c: app_id==APP_IMU_EL → "imu_el"."""
        emu = ImuEmulator(app_id=3)
        assert emu.get_status()["sensor_name"] == "imu_el"

    def test_name_for_app_imu_az(self):
        """imu.c: app_id==APP_IMU_AZ → "imu_az"."""
        emu = ImuEmulator(app_id=6)
        assert emu.get_status()["sensor_name"] == "imu_az"

    def test_server_is_noop(self):
        """RVC mode: no commands supported."""
        emu = ImuEmulator()
        before = emu.get_status()
        emu.server({"anything": True})
        after = emu.get_status()
        assert before == after

    def test_status_is_per_cycle(self):
        """imu.c: status="update" iff a packet arrived since last get_status()."""
        emu = ImuEmulator()
        # No op() yet: no packet this cycle → "error".
        assert emu.get_status()["status"] == "error"
        # op() simulates a fresh packet.
        emu.op()
        assert emu.get_status()["status"] == "update"
        # get_status() resets the flag; without another op() it's "error" again.
        assert emu.get_status()["status"] == "error"

    def test_euler_angles_are_degrees(self):
        """RVC output is in degrees, not radians.

        For imu_el with identity mount and el=30°, the forward model
        produces roll≈30 (degrees).  If the emitter used radians the
        value would be ~0.52, which is distinguishable from 30.
        """
        emu = ImuEmulator(app_id=3)  # imu_el
        emu.set_orientation(az_deg=0.0, el_deg=30.0)
        emu.op()
        status = emu.get_status()
        # roll encodes the elevation rotation for imu_el with identity mount
        assert abs(status["roll"] - 30.0) < 1.0


# ---------------------------------------------------------------------------
# Lidar protocol (src/lidar.c)
# ---------------------------------------------------------------------------


class TestLidarProtocol:
    """Protocol conformance tests for APP_LIDAR (app_id=4)."""

    def test_sensor_name(self):
        assert LidarEmulator().get_status()["sensor_name"] == "lidar"

    def test_no_commands_accepted(self):
        emu = LidarEmulator()
        emu.server({"distance_m": 999})
        assert emu.distance == 0.0  # unchanged

    def test_distance_is_float(self):
        """lidar.c: distance = dist_cm / 100.0 → always float."""
        emu = LidarEmulator()
        emu.op()
        assert isinstance(emu.get_status()["distance_m"], float)

    def test_current_voltage_is_float(self):
        """currentmon.c reports the raw divided ADC voltage as a float."""
        emu = LidarEmulator()
        emu.op()
        assert isinstance(emu.get_status()["current_voltage"], float)

    def test_current_independent_of_lidar_failure(self):
        """currentmon_op() is a separate dispatch call: the current voltage
        refreshes every cycle even when the lidar I2C read fails."""
        emu = LidarEmulator()
        emu.op()
        emu.get_status()["current_voltage"]
        emu.simulate_sensor_failure()
        emu.op()
        bad = emu.get_status()
        assert bad["status"] == "error"  # lidar half failed
        assert "current_voltage" in bad  # current half still present
        assert isinstance(bad["current_voltage"], float)

    def test_status_is_per_cycle(self):
        """lidar.c: status="update" iff op() refreshed distance this cycle."""
        emu = LidarEmulator()
        # No op() yet: status defaults to "error".
        assert emu.get_status()["status"] == "error"
        emu.op()
        assert emu.get_status()["status"] == "update"
        # get_status() resets the flag.
        assert emu.get_status()["status"] == "error"

    def test_simulated_failure_emits_error_with_stale_distance(self):
        """Failure path: distance unchanged from previous good read, status="error"."""
        emu = LidarEmulator()
        emu.op()
        good = emu.get_status()
        assert good["status"] == "update"
        prev_distance = good["distance_m"]

        emu.simulate_sensor_failure()
        emu.op()
        bad = emu.get_status()
        assert bad["status"] == "error"
        assert bad["distance_m"] == prev_distance


# ---------------------------------------------------------------------------
# RFSwitch protocol (src/rfswitch.c)
# ---------------------------------------------------------------------------


class TestRFSwitchProtocol:
    """Protocol conformance tests for APP_RFSWITCH (app_id=5).

    These tests pass ``settle_ms=0`` to exercise command/state logic
    without waiting on the transition timer; the settle behavior is
    covered by :class:`TestRFSwitchEmulator` in test_emulators.py.
    """

    def test_sensor_name(self):
        emu = RFSwitchEmulator(settle_ms=0)
        assert emu.get_status()["sensor_name"] == "rfswitch"

    def test_thermistor_voltages_in_status(self):
        """rfswitch_status() reports raw ADC volts for the three PCB
        thermistors (volt_therm<i> on ADC input i); conversion to
        temperature is host-side (constants unmeasured)."""
        emu = RFSwitchEmulator(settle_ms=0)
        status = emu.get_status()
        for i in range(3):
            volts = status[f"volt_therm{i}"]
            assert isinstance(volts, float)
            assert 0.0 <= volts <= 3.3

    def test_initial_state_zero(self):
        """rfswitch_init() sets sw_state = 0 once settled."""
        emu = RFSwitchEmulator(settle_ms=0)
        assert emu.get_status()["sw_state"] == 0

    def test_set_state(self):
        emu = RFSwitchEmulator(settle_ms=0)
        emu.server({"sw_state": 10})
        assert emu.get_status()["sw_state"] == 10

    def test_path_address_range(self):
        """rfswitch_server() accepts every burned path address (0-15)."""
        emu = RFSwitchEmulator(settle_ms=0)
        for val in range(RFSwitchEmulator.NUM_PATHS):
            emu.server({"sw_state": val})
            assert emu.get_status()["sw_state"] == val

    def test_unburned_addresses_rejected(self):
        """Addresses >= NUM_PATHS hold 0xFF on the EEPROMs (all switch
        inputs closed + noise diode on) and must never reach the bus."""
        emu = RFSwitchEmulator(settle_ms=0)
        emu.server({"sw_state": 3})
        for val in (16, 31, 42, 255):
            emu.server({"sw_state": val})
            assert emu.get_status()["sw_state"] == 3

    def test_transition_sentinel_during_settle(self):
        """sw_state reports SW_STATE_UNKNOWN (-1) while settling."""
        emu = RFSwitchEmulator(settle_ms=30)
        emu.server({"sw_state": 7})
        assert emu.get_status()["sw_state"] == emu.SW_STATE_UNKNOWN


# ---------------------------------------------------------------------------
# LNA rail monitor protocol (src/lnamon.c)
# ---------------------------------------------------------------------------


class TestLnaMonProtocol:
    """Protocol conformance tests for APP_LNAMON (app_id=7)."""

    def test_sensor_name(self):
        assert LnaMonEmulator().get_status()["sensor_name"] == "lnamon"

    def test_app_id(self):
        assert LnaMonEmulator().get_status()["app_id"] == 7

    def test_no_commands_accepted(self):
        """lnamon_server is a no-op."""
        emu = LnaMonEmulator()
        emu.op()
        before = emu.get_status()
        emu.server({"lna_rail_voltage": 0.0})
        assert emu.get_status() == before

    def test_voltages_are_float(self):
        """lnamon.c reports both voltages as KV_FLOAT."""
        emu = LnaMonEmulator()
        emu.op()
        status = emu.get_status()
        assert isinstance(status["lna_adc_voltage"], float)
        assert isinstance(status["lna_rail_voltage"], float)

    def test_rail_is_adc_times_divider_gain(self):
        """lnamon.c: Vrail = Vadc * (R1 + R2) / R2 with R1=51k, R2=75.1k."""
        emu = LnaMonEmulator()
        emu.op()
        status = emu.get_status()
        assert status["lna_rail_voltage"] == pytest.approx(
            status["lna_adc_voltage"] * 126.1 / 75.1
        )

    def test_nominal_rail_reading(self):
        """A 5.0V rail reads ~2.98V at the pin and ~5.0V after scaling."""
        emu = LnaMonEmulator(rail_voltage=5.0)
        emu.op()
        status = emu.get_status()
        assert status["lna_adc_voltage"] == pytest.approx(2.978, abs=0.03)
        assert status["lna_rail_voltage"] == pytest.approx(5.0, abs=0.05)
