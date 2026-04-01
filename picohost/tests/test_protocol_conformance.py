"""
Protocol conformance tests for pico firmware emulators.

Each test documents the exact command/response contract derived from reading
the C firmware source.  These run against emulators in CI and serve as a
reference for manual hardware verification.

See CLAUDE.md § Command Protocol for the general framing.
"""

import json
import time

import pytest
from picohost.emulators import (
    MotorEmulator,
    TempCtrlEmulator,
    TempMonEmulator,
    ImuEmulator,
    LidarEmulator,
    RFSwitchEmulator,
)
from picohost.emulators.base import PicoEmulator

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
        for Cls in (MotorEmulator, TempCtrlEmulator, TempMonEmulator,
                    ImuEmulator, LidarEmulator, RFSwitchEmulator):
            emu = Cls()
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
            MotorEmulator(), TempCtrlEmulator(), TempMonEmulator(),
            ImuEmulator(), LidarEmulator(), RFSwitchEmulator(),
        ]
        for emu in emulators:
            before = emu.get_status()
            emu.server({})
            after = emu.get_status()
            # Status should be unchanged (timestamps may differ for tempctrl/tempmon)
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
        pos = emu.azimuth.position
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
        assert emu.A.T_target == 30.0
        assert emu.A.gain == 0.2
        assert emu.A.baseline == 0.4
        assert emu.A.clamp == 0.6
        assert emu.A.hysteresis == 0.5
        assert emu.A.enabled is False

    def test_clamp_validation(self):
        """tempctrl.c line 77: fminf(1.0, fmaxf(0.0, val))."""
        emu = TempCtrlEmulator()
        emu.server({"A_clamp": 2.0})
        assert emu.A.clamp == 1.0
        emu.server({"A_clamp": -1.0})
        assert emu.A.clamp == 0.0
        emu.server({"A_clamp": 0.5})
        assert emu.A.clamp == 0.5

    def test_enable_via_int(self):
        """tempctrl.c line 73: valueint ? true : false."""
        emu = TempCtrlEmulator()
        emu.server({"A_enable": 1})
        assert emu.A.enabled is True
        emu.server({"A_enable": 0})
        assert emu.A.enabled is False

    def test_hysteresis_band_drive_zero(self):
        """Within hysteresis band, drive = 0 and active = false."""
        emu = TempCtrlEmulator()
        emu.A.T_now = 29.8
        emu.server({"A_temp_target": 30.0, "A_enable": True, "A_hysteresis": 0.5})
        emu.op()
        assert emu.A.drive == 0.0
        assert emu.A.active is False

    def test_drive_clamped_to_max(self):
        """Drive magnitude limited by clamp."""
        emu = TempCtrlEmulator()
        emu.A.T_now = 0.0  # large delta from default target 30.0
        emu.server({"A_enable": True, "A_clamp": 0.3})
        emu.op()
        assert abs(emu.A.drive) <= 0.3 + 1e-9

    def test_sensor_error_disables_drive(self):
        """tempctrl.c line 137-142: if internally_disabled, drive = 0."""
        emu = TempCtrlEmulator()
        emu.server({"A_enable": True, "A_temp_target": 50.0})
        emu.op()
        assert emu.A.drive != 0.0
        emu.inject_sensor_error("A")
        emu.op()
        assert emu.A.drive == 0.0

    def test_sensor_error_status_field(self):
        """tempctrl.c line 93-94: error status on sensor failure."""
        emu = TempCtrlEmulator()
        emu.inject_sensor_error("A")
        status = emu.get_status()
        assert status["A_status"] == "error"
        assert status["B_status"] == "update"


# ---------------------------------------------------------------------------
# TempMon protocol (src/tempmon.c)
# ---------------------------------------------------------------------------


class TestTempMonProtocol:
    """Protocol conformance tests for APP_TEMPMON (app_id=2)."""

    def test_sensor_name(self):
        assert TempMonEmulator().get_status()["sensor_name"] == "temp_mon"

    def test_no_commands_accepted(self):
        """tempmon_server() is an empty function."""
        emu = TempMonEmulator()
        emu.server({"A_temp_target": 99.0, "sw_state": 5})
        # no crash, no state change
        assert emu._base_temp_a == 25.0

    def test_sensor_error_status(self):
        emu = TempMonEmulator()
        emu.inject_sensor_error("A")
        status = emu.get_status()
        assert status["A_status"] == "error"
        assert status["B_status"] == "update"


# ---------------------------------------------------------------------------
# IMU protocol (src/imu.c — UART RVC mode)
# ---------------------------------------------------------------------------


class TestImuProtocol:
    """Protocol conformance tests for APP_IMU (app_id=3)."""

    def test_name_for_app_imu(self):
        """imu.c: app_id==APP_IMU → "imu_panda"."""
        emu = ImuEmulator(app_id=3)
        assert emu.get_status()["sensor_name"] == "imu_panda"

    def test_name_for_other_app(self):
        """imu.c: app_id!=APP_IMU → "imu_antenna"."""
        emu = ImuEmulator(app_id=5)
        assert emu.get_status()["sensor_name"] == "imu_antenna"

    def test_server_is_noop(self):
        """RVC mode: no commands supported."""
        emu = ImuEmulator()
        before = emu.get_status()
        emu.server({"anything": True})
        after = emu.get_status()
        assert before == after

    def test_error_status_when_not_initialized(self):
        """imu.c: status="error" if !is_initialized."""
        emu = ImuEmulator()
        emu.inject_init_failure()
        assert emu.get_status()["status"] == "error"

    def test_euler_angles_are_degrees(self):
        """RVC output is in degrees."""
        emu = ImuEmulator()
        emu.az_angle = 0.5  # ~28.6 degrees
        emu.op()
        status = emu.get_status()
        # yaw should be close to degrees(0.5) ≈ 28.6
        assert abs(status["yaw"] - 28.6) < 1.0


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
        assert emu.distance == 100.0  # unchanged

    def test_distance_is_float(self):
        """lidar.c: distance = dist_cm / 100.0 → always float."""
        emu = LidarEmulator()
        emu.op()
        assert isinstance(emu.get_status()["distance_m"], float)


# ---------------------------------------------------------------------------
# RFSwitch protocol (src/rfswitch.c)
# ---------------------------------------------------------------------------


class TestRFSwitchProtocol:
    """Protocol conformance tests for APP_RFSWITCH (app_id=5)."""

    def test_sensor_name(self):
        assert RFSwitchEmulator().get_status()["sensor_name"] == "rfswitch"

    def test_initial_state_zero(self):
        """rfswitch_init() sets sw_state = 0."""
        assert RFSwitchEmulator().get_status()["sw_state"] == 0

    def test_set_state(self):
        emu = RFSwitchEmulator()
        emu.server({"sw_state": 42})
        assert emu.get_status()["sw_state"] == 42

    def test_8bit_bitmask_range(self):
        """rfswitch_op() iterates bits 0-7."""
        emu = RFSwitchEmulator()
        for val in (0, 1, 128, 255):
            emu.server({"sw_state": val})
            assert emu.get_status()["sw_state"] == val


