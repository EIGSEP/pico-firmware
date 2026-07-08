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
        """init_single_tempctrl() sets these defaults."""
        emu = TempCtrlEmulator()
        assert emu.lna.T_target == 30.0
        assert emu.lna.Kp == 0.2
        assert emu.lna.Ki == 0.0
        assert emu.lna.integral == 0.0
        assert emu.lna.clamp == 0.2
        assert emu.lna.hysteresis == 0.5
        assert emu.lna.enabled is False

    def test_clamp_validation(self):
        """tempctrl.c line 77: fminf(1.0, fmaxf(0.0, val))."""
        emu = TempCtrlEmulator()
        emu.server({"LNA_clamp": 2.0})
        assert emu.lna.clamp == 1.0
        emu.server({"LNA_clamp": -1.0})
        assert emu.lna.clamp == 0.0
        emu.server({"LNA_clamp": 0.5})
        assert emu.lna.clamp == 0.5

    def test_enable_via_int(self):
        """tempctrl.c line 73: valueint ? true : false."""
        emu = TempCtrlEmulator()
        emu.server({"LNA_enable": 1})
        assert emu.lna.enabled is True
        emu.server({"LNA_enable": 0})
        assert emu.lna.enabled is False

    def test_hysteresis_band_drive_zero(self):
        """Inside deadband: drive = 0, integrator frozen + cleared."""
        emu = TempCtrlEmulator()
        emu.lna.T_now = 29.8
        emu.lna.integral = 5.0  # leftover state must not leak through
        emu.server(
            {
                "LNA_temp_target": 30.0,
                "LNA_enable": True,
                "LNA_hysteresis": 0.5,
                "LNA_Ki": 0.1,
            }
        )
        emu.op()
        assert emu.lna.drive == 0.0
        assert emu.lna.active is False
        assert emu.lna.integral == 0.0

    def test_drive_clamped_to_max(self):
        """Drive magnitude limited by clamp."""
        emu = TempCtrlEmulator()
        emu.lna.T_now = 0.0  # large delta from default target 30.0
        emu.server({"LNA_enable": True, "LNA_clamp": 0.3})
        emu.op()
        assert abs(emu.lna.drive) <= 0.3 + 1e-9

    def test_sensor_error_disables_drive(self):
        """tempctrl.c: a plausibility-failed cycle gates drive to 0 and
        clears the integrator (the sensor may be gone entirely)."""
        emu = TempCtrlEmulator()
        emu.server(
            {"LNA_enable": True, "LNA_temp_target": 50.0, "LNA_Ki": 0.1}
        )
        # Two samples so PI fires and drive leaves zero. The first fresh
        # sample only seeds the candidate reference (two-to-anchor); control
        # engages on the second, mirroring firmware.
        for _ in range(2):
            emu.op()
        assert emu.lna.drive != 0.0
        emu.inject_sensor_error("LNA")
        emu.op()
        assert emu.lna.drive == 0.0
        assert emu.lna.integral == 0.0

    def test_kp_ki_commands_round_trip(self):
        """LNA_Kp / LNA_Ki / LOAD_Kp / LOAD_Ki update channel state."""
        emu = TempCtrlEmulator()
        emu.server(
            {
                "LNA_Kp": 0.35,
                "LNA_Ki": 0.02,
                "LOAD_Kp": 0.4,
                "LOAD_Ki": 0.05,
            }
        )
        assert emu.lna.Kp == 0.35
        assert emu.lna.Ki == 0.02
        assert emu.load.Kp == 0.4
        assert emu.load.Ki == 0.05

    def test_integral_reset_clears_state(self):
        """*_integral_reset zeroes the integrator without touching gains."""
        emu = TempCtrlEmulator()
        emu.lna.integral = 12.5
        emu.lna.Kp = 0.4
        emu.lna.Ki = 0.1
        emu.server({"LNA_integral_reset": True})
        assert emu.lna.integral == 0.0
        assert emu.lna.Kp == 0.4
        assert emu.lna.Ki == 0.1

    def test_anti_windup_freezes_integral_at_saturation(self):
        """Integrator must not grow while the output is clamped against
        the direction of error."""
        emu = TempCtrlEmulator()
        emu.lna.T_now = 0.0  # huge positive T_delta vs default target 30
        emu.server(
            {
                "LNA_enable": True,
                "LNA_Ki": 0.5,
                "LNA_clamp": 0.6,
            }
        )
        # Many ticks against the saturation rail — integrator must stay put.
        for _ in range(20):
            emu.op()
            emu.lna.T_now = 0.0  # pin T_now so we stay deeply saturated
        saturated_integral = emu.lna.integral
        for _ in range(20):
            emu.op()
            emu.lna.T_now = 0.0
        assert emu.lna.integral == saturated_integral
        assert abs(emu.lna.drive) == pytest.approx(0.6)

    def test_no_baseline_step_when_leaving_hysteresis(self):
        """Drive must ramp smoothly through small values when T_delta
        just exits the deadband — the old bang-bang law produced a
        ~40 % PWM step here, which is the bug this controller fixes.
        """
        emu = TempCtrlEmulator()
        emu.lna.T_now = 29.4  # 0.6 below target, just outside ±0.5 band
        emu.lna.thermal_frozen = True  # pin T_now so first PI sees T_delta=0.6
        emu.server(
            {
                "LNA_temp_target": 30.0,
                "LNA_enable": True,
                "LNA_hysteresis": 0.5,
            }
        )
        # Two samples: the first seeds the candidate reference (two-to-anchor),
        # the second anchors and runs the first PI step.
        for _ in range(2):
            emu.op()
        # With Kp=0.2 and T_delta=0.6, drive should be ~0.12 (12 % PWM),
        # not >=0.4 like the old baseline-kick law.
        assert 0.0 < emu.lna.drive < 0.2

    def test_sensor_error_status_field(self):
        """tempctrl.c: per-channel status reports data validity — a
        plausibility failure errors the channel and nulls the derived
        values while the raw voltage stays reported."""
        emu = TempCtrlEmulator()
        emu.op()  # LOAD samples once so its data is valid
        emu.inject_sensor_error("LNA")
        emu.op()
        status = emu.get_status()
        assert status["LNA_status"] == "error"
        assert status["LNA_T_now"] is None
        assert status["LNA_resistance"] is None
        assert isinstance(status["LNA_voltage"], float)
        assert status["LOAD_status"] == "update"

    def test_installed_default_true_and_in_status(self):
        """init_single_tempctrl() defaults installed=true; tempctrl_status
        reports LNA_installed / LOAD_installed on every tick."""
        emu = TempCtrlEmulator()
        assert emu.lna.installed is True
        assert emu.load.installed is True
        status = emu.get_status()
        assert status["LNA_installed"] is True
        assert status["LOAD_installed"] is True

    def test_installed_parse_via_int(self):
        """tempctrl.c: item_json->valueint ? true : false — a JSON string
        like "false" has valueint 0, so it uninstalls (cJSON parity)."""
        emu = TempCtrlEmulator()
        emu.server({"LNA_installed": 0})
        assert emu.lna.installed is False
        emu.server({"LNA_installed": 1})
        assert emu.lna.installed is True
        emu.server({"LNA_installed": "false"})
        assert emu.lna.installed is False
        assert emu.load.installed is True

    def test_uninstalled_channel_skips_sampling_and_drive(self):
        """tempctrl_update_sensor_drive: an uninstalled channel returns
        before temp_sensor_read — its ADC input is never mux-selected
        (the potmon crosstalk lesson) — and never drives: data_invalid
        every cycle, controller state reset, drive forced to 0."""
        emu = TempCtrlEmulator()
        emu.server({"LNA_enable": True, "LNA_temp_target": 50.0})
        for _ in range(2):  # two-to-anchor, then control engages
            emu.op()
        assert emu.lna.drive != 0.0
        emu.server({"LNA_installed": 0})
        emu.op()
        assert emu.lna.drive == 0.0
        assert emu.lna.data_invalid is True
        status = emu.get_status()
        assert status["LNA_status"] == "error"
        assert status["LNA_T_now"] is None
        # LOAD is untouched — still sampling normally.
        assert status["LOAD_status"] == "update"

    def test_uninstalled_drops_anchor_reinstall_reseeds(self):
        """Uninstalling drops the rate anchor (it is only valid across
        continuous good data); re-install re-seeds two-to-anchor before
        control resumes, mirroring recovery from a plausibility outage."""
        emu = TempCtrlEmulator()
        emu.server({"LNA_enable": True, "LNA_temp_target": 50.0})
        for _ in range(2):
            emu.op()
        assert emu.lna.rate_ref_valid is True
        emu.server({"LNA_installed": 0})
        emu.op()
        assert emu.lna.rate_ref_valid is False
        emu.server({"LNA_installed": 1})
        emu.op()  # candidate seed only — control still gated
        assert emu.lna.drive == 0.0
        emu.op()  # anchor confirmed — control resumes
        assert emu.lna.drive != 0.0

    def test_uninstall_preserves_latches_enable_ack_clears(self):
        """Uninstalling is not a trip ack: sticky latches survive it and
        clear only via *_enable=true (tempctrl_apply_enable)."""
        emu = TempCtrlEmulator()
        emu.server({"LNA_enable": True})
        for _ in range(2):
            emu.op()
        emu.lna.inject_sensor_glitch(1000.0, count=MAX_REJECTS)
        for _ in range(MAX_REJECTS):
            emu.op()
        assert emu.lna.sensor_tripped is True
        emu.server({"LNA_installed": 0})
        emu.op()
        assert emu.lna.sensor_tripped is True
        emu.server({"LNA_enable": True})
        assert emu.lna.sensor_tripped is False


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
