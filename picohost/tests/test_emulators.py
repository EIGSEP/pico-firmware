"""
Unit tests for firmware emulators.
Tests emulators standalone (no mock serial), calling methods directly.
"""

import time

import numpy as np
import pytest
from picohost.emulators import (
    MotorEmulator,
    TempCtrlEmulator,
    TempMonEmulator,
    ImuEmulator,
    LidarEmulator,
    RFSwitchEmulator,
)


class TestMotorEmulator:
    def test_initial_state(self):
        emu = MotorEmulator()
        status = emu.get_status()
        assert status["sensor_name"] == "motor"
        assert status["az_pos"] == 0
        assert status["el_pos"] == 0
        assert status["az_target_pos"] == 0
        assert status["el_target_pos"] == 0

    def test_set_target_and_converge(self):
        emu = MotorEmulator()
        emu.server({"az_set_target_pos": 1000})
        # Call op() enough times for position to converge
        # max_pulses=60 per call, so ceil(1000/60) = 17 calls needed
        for _ in range(20):
            emu.op()
        assert emu.azimuth.position == 1000
        # Position should remain stable after convergence
        for _ in range(5):
            emu.op()
        assert emu.azimuth.position == 1000

    def test_retarget(self):
        """Move to one position, then change target to another."""
        emu = MotorEmulator()
        emu.server({"az_set_target_pos": 500})
        for _ in range(20):
            emu.op()
        assert emu.azimuth.position == 500
        emu.server({"az_set_target_pos": 200})
        for _ in range(20):
            emu.op()
        assert emu.azimuth.position == 200

    def test_reverse_direction(self):
        """Move forward then backward past origin."""
        emu = MotorEmulator()
        emu.server({"az_set_target_pos": 300})
        for _ in range(20):
            emu.op()
        assert emu.azimuth.position == 300
        emu.server({"az_set_target_pos": -100})
        for _ in range(20):
            emu.op()
        assert emu.azimuth.position == -100

    def test_elevation(self):
        emu = MotorEmulator()
        emu.server({"el_set_target_pos": -500})
        for _ in range(20):
            emu.op()
        assert emu.elevation.position == -500

    def test_delay_settings(self):
        emu = MotorEmulator()
        emu.server({"az_up_delay_us": 3000, "az_dn_delay_us": 4000})
        assert emu.azimuth.up_delay_us == 3000
        assert emu.azimuth.dn_delay_us == 4000

    def test_noop_when_at_target(self):
        """op() should be a no-op when position == target (no needless work).

        In C firmware, this corresponds to skipping enable/disable GPIO
        toggling when there are zero steps to take.
        """
        emu = MotorEmulator()
        # Position already at target (both 0)
        emu.op()
        assert emu.azimuth.position == 0
        assert emu.azimuth.dir == 0
        assert emu.azimuth.steps_in_direction == 0
        # After convergence, further op() calls should be no-ops
        emu.server({"az_set_target_pos": 120})
        for _ in range(5):
            emu.op()
        assert emu.azimuth.position == 120
        steps_before = emu.azimuth.steps_in_direction
        emu.op()
        assert emu.azimuth.steps_in_direction == steps_before

    def test_status_fields(self):
        emu = MotorEmulator()
        status = emu.get_status()
        expected_keys = {
            "sensor_name",
            "status",
            "app_id",
            "az_pos",
            "az_target_pos",
            "el_pos",
            "el_target_pos",
        }
        assert set(status.keys()) == expected_keys


class TestTempCtrlEmulator:
    def test_initial_state(self):
        emu = TempCtrlEmulator()
        status = emu.get_status()
        assert status["sensor_name"] == "tempctrl"
        assert status["LNA_enabled"] is False
        assert status["LOAD_enabled"] is False
        assert status["LNA_T_target"] == 30.0

    def test_enable_and_converge(self):
        """Enable LNA channel, verify convergence to within hysteresis of target."""
        emu = TempCtrlEmulator()
        emu.lna.T_now = 25.0
        emu.server(
            {
                "LNA_temp_target": 30.0,
                "LNA_enable": True,
                "LNA_hysteresis": 0.5,
            }
        )
        for _ in range(500):
            emu.op()
        assert abs(emu.lna.T_now - 30.0) < 0.5

    def test_converge_to_non_default_target(self):
        """Converge to a target different from the 30.0 default."""
        emu = TempCtrlEmulator()
        emu.lna.T_now = 25.0
        emu.server(
            {
                "LNA_temp_target": 40.0,
                "LNA_enable": True,
                "LNA_hysteresis": 0.5,
            }
        )
        for _ in range(1000):
            emu.op()
        assert abs(emu.lna.T_now - 40.0) < 0.5

    def test_cool_back_down(self):
        """Heat to target, then set a lower target and verify cooling."""
        emu = TempCtrlEmulator()
        emu.lna.T_now = 25.0
        emu.server(
            {
                "LNA_temp_target": 35.0,
                "LNA_enable": True,
                "LNA_hysteresis": 0.5,
            }
        )
        for _ in range(1000):
            emu.op()
        assert abs(emu.lna.T_now - 35.0) < 0.5
        # Now cool back to 25
        emu.server({"LNA_temp_target": 25.0})
        for _ in range(1000):
            emu.op()
        assert abs(emu.lna.T_now - 25.0) < 0.5

    def test_channel_load(self):
        """LOAD channel works independently of LNA channel."""
        emu = TempCtrlEmulator()
        emu.load.T_now = 20.0
        emu.server(
            {
                "LOAD_temp_target": 28.0,
                "LOAD_enable": True,
                "LOAD_hysteresis": 0.5,
            }
        )
        for _ in range(1000):
            emu.op()
        assert abs(emu.load.T_now - 28.0) < 0.5
        # LNA channel should not have moved (not enabled)
        assert emu.lna.drive == 0.0

    def test_status_fields(self):
        emu = TempCtrlEmulator()
        status = emu.get_status()
        expected_keys = {
            "sensor_name",
            "app_id",
            "watchdog_tripped",
            "watchdog_timeout_ms",
            "LNA_status",
            "LNA_T_now",
            "LNA_timestamp",
            "LNA_T_target",
            "LNA_drive_level",
            "LNA_enabled",
            "LNA_active",
            "LNA_int_disabled",
            "LNA_hysteresis",
            "LNA_clamp",
            "LOAD_status",
            "LOAD_T_now",
            "LOAD_timestamp",
            "LOAD_T_target",
            "LOAD_drive_level",
            "LOAD_enabled",
            "LOAD_active",
            "LOAD_int_disabled",
            "LOAD_hysteresis",
            "LOAD_clamp",
        }
        assert set(status.keys()) == expected_keys


class TestTempCtrlWatchdog:
    def test_watchdog_trips_after_timeout(self):
        """Watchdog disables peltiers when no command arrives within timeout."""
        emu = TempCtrlEmulator()
        emu.server({"LNA_enable": True, "LOAD_enable": True})
        assert emu.lna.enabled is True
        assert emu.load.enabled is True
        # Simulate time passing beyond the watchdog timeout
        emu._last_cmd_time = time.time() - (emu.watchdog_timeout_ms / 1000 + 1)
        emu.op()
        assert emu.watchdog_tripped is True
        assert emu.lna.enabled is False
        assert emu.load.enabled is False

    def test_server_resets_watchdog(self):
        """Any command resets the watchdog timer and clears the trip flag."""
        emu = TempCtrlEmulator()
        # Trip the watchdog
        emu._last_cmd_time = time.time() - (emu.watchdog_timeout_ms / 1000 + 1)
        emu.op()
        assert emu.watchdog_tripped is True
        # Any command clears the trip
        emu.server({"LNA_enable": True})
        assert emu.watchdog_tripped is False

    def test_watchdog_does_not_trip_before_timeout(self):
        """Watchdog stays clear while commands arrive within timeout."""
        emu = TempCtrlEmulator()
        emu.server({"LNA_enable": True})
        emu.op()
        assert emu.watchdog_tripped is False
        assert emu.lna.enabled is True

    def test_watchdog_timeout_configurable(self):
        """Watchdog timeout can be changed via server command."""
        emu = TempCtrlEmulator()
        emu.server({"watchdog_timeout_ms": 5000})
        assert emu.watchdog_timeout_ms == 5000

    def test_watchdog_disabled_with_zero_timeout(self):
        """Setting watchdog_timeout_ms to 0 disables the watchdog."""
        emu = TempCtrlEmulator()
        emu.server({"LNA_enable": True, "watchdog_timeout_ms": 0})
        emu._last_cmd_time = time.time() - 999
        emu.op()
        assert emu.watchdog_tripped is False
        assert emu.lna.enabled is True


class TestTempMonEmulator:
    def test_initial_state(self):
        emu = TempMonEmulator()
        status = emu.get_status()
        assert status["sensor_name"] == "temp_mon"
        assert status["app_id"] == 2

    def test_noise_is_mean_reverting(self):
        emu = TempMonEmulator()
        for _ in range(1000):
            emu.op()
        # Mean-reverting noise stays tightly around base temperature
        assert abs(emu.temp_lna - 25.0) < 1.0
        assert abs(emu.temp_load - 25.0) < 1.0

    def test_status_fields(self):
        emu = TempMonEmulator()
        status = emu.get_status()
        expected_keys = {
            "sensor_name",
            "app_id",
            "LNA_status",
            "LNA_temp",
            "LNA_timestamp",
            "LOAD_status",
            "LOAD_temp",
            "LOAD_timestamp",
        }
        assert set(status.keys()) == expected_keys


class TestImuEmulator:
    def test_initial_state(self):
        emu = ImuEmulator(app_id=3)
        assert emu.name == "imu_el"
        status = emu.get_status()
        assert status["sensor_name"] == "imu_el"

    def test_antenna_name(self):
        emu = ImuEmulator(app_id=6)
        assert emu.name == "imu_az"

    def test_accel_magnitude(self):
        """Accelerometer should read ~9.81 m/s^2 magnitude (stationary)."""
        emu = ImuEmulator()
        for _ in range(10):
            emu.op()
        mag = np.sqrt(emu.accel_x**2 + emu.accel_y**2 + emu.accel_z**2)
        assert abs(mag - 9.81) < 0.1

    def test_euler_from_angles(self):
        """Setting az/el angles produces matching yaw/pitch.

        NOTE: assumes yaw=az, pitch=el — needs hardware verification.
        See TODO in emulators/imu.py.
        """
        emu = ImuEmulator()
        emu.az_angle = np.pi / 4  # 45 degrees azimuth
        emu.el_angle = np.pi / 6  # 30 degrees elevation
        emu.op()
        assert abs(emu.yaw - 45.0) < 1.0
        assert abs(emu.pitch - 30.0) < 1.0

    def test_sensor_failure_triggers_reinit(self):
        """IMU reports error after event timeout, then recovers."""
        import picohost.emulators.imu as imu_mod

        emu = ImuEmulator()
        emu.op()
        assert emu.is_initialized is True
        assert emu.get_status()["status"] == "update"

        # Simulate sensor crash and push last_event_time into the past
        emu.simulate_sensor_failure()
        emu._last_event_time -= imu_mod.IMU_EVENT_TIMEOUT_S + 1
        emu.op()
        assert emu.is_initialized is False
        assert emu.get_status()["status"] == "error"

        # Sensor comes back — next op() should re-initialize
        emu.simulate_sensor_recovery()
        emu.op()
        assert emu.is_initialized is True
        assert emu.get_status()["status"] == "update"

    def test_sensor_failure_before_timeout_stays_initialized(self):
        """IMU stays initialized if failure is shorter than timeout."""
        emu = ImuEmulator()
        emu.op()
        emu.simulate_sensor_failure()
        # Without advancing the clock, still within timeout
        emu.op()
        assert emu.is_initialized is True

    def test_status_fields(self):
        emu = ImuEmulator()
        status = emu.get_status()
        expected_keys = {
            "sensor_name",
            "status",
            "app_id",
            "yaw",
            "pitch",
            "roll",
            "accel_x",
            "accel_y",
            "accel_z",
        }
        assert set(status.keys()) == expected_keys


class TestLidarEmulator:
    def test_initial_state(self):
        emu = LidarEmulator()
        status = emu.get_status()
        assert status["sensor_name"] == "lidar"
        assert status["distance_m"] == 100.0

    def test_noise_is_mean_reverting(self):
        emu = LidarEmulator()
        for _ in range(1000):
            emu.op()
        # Mean-reverting noise stays tightly around base distance
        assert abs(emu.distance - 100.0) < 0.5

    def test_status_fields(self):
        emu = LidarEmulator()
        status = emu.get_status()
        expected_keys = {"sensor_name", "status", "app_id", "distance_m"}
        assert set(status.keys()) == expected_keys


class TestRFSwitchEmulator:
    def test_initial_state_settled_when_instant(self):
        """settle_ms=0 skips the boot transition; first status is settled 0."""
        emu = RFSwitchEmulator(settle_ms=0)
        status = emu.get_status()
        assert status["sensor_name"] == "rfswitch"
        assert status["sw_state"] == 0

    def test_status_fields(self):
        emu = RFSwitchEmulator(settle_ms=0)
        status = emu.get_status()
        expected_keys = {"sensor_name", "status", "app_id", "sw_state"}
        assert set(status.keys()) == expected_keys

    def test_boot_starts_in_transition(self):
        """Default settle_ms > 0: boot reports UNKNOWN until settle."""
        emu = RFSwitchEmulator(settle_ms=30)
        assert emu.in_transition is True
        assert emu.get_status()["sw_state"] == emu.SW_STATE_UNKNOWN
        # After the settle window, op() clears the transition.
        time.sleep(0.05)
        emu.op()
        assert emu.in_transition is False
        assert emu.get_status()["sw_state"] == 0

    def test_command_enters_transition(self):
        """A state-change command re-enters the UNKNOWN transition window."""
        emu = RFSwitchEmulator(settle_ms=0)
        assert emu.get_status()["sw_state"] == 0
        emu.settle_ms = 30  # arm transition for this command
        emu.server({"sw_state": 7})
        assert emu.in_transition is True
        assert emu.get_status()["sw_state"] == emu.SW_STATE_UNKNOWN
        time.sleep(0.05)
        emu.op()
        assert emu.in_transition is False
        assert emu.get_status()["sw_state"] == 7

    def test_same_state_command_is_noop(self):
        """Commanding the current state must not re-enter transition."""
        emu = RFSwitchEmulator(settle_ms=0)
        emu.server({"sw_state": 5})
        assert emu.get_status()["sw_state"] == 5
        emu.settle_ms = 30  # would arm transition if a change actually occurred
        emu.server({"sw_state": 5})
        assert emu.in_transition is False
        assert emu.get_status()["sw_state"] == 5


# ---------------------------------------------------------------------------
# Status field TYPE verification
# ---------------------------------------------------------------------------


class TestMotorStatusTypes:
    """Verify status field types match C firmware KV_* type tags."""

    def test_status_field_types(self):
        emu = MotorEmulator()
        status = emu.get_status()
        assert isinstance(status["sensor_name"], str)
        assert isinstance(status["status"], str)
        assert isinstance(status["app_id"], int)
        assert isinstance(status["az_pos"], int)
        assert isinstance(status["az_target_pos"], int)
        assert isinstance(status["el_pos"], int)
        assert isinstance(status["el_target_pos"], int)


class TestTempCtrlStatusTypes:
    def test_status_field_types(self):
        emu = TempCtrlEmulator()
        emu.op()  # populate timestamps
        status = emu.get_status()
        for prefix in ("LNA", "LOAD"):
            assert isinstance(status[f"{prefix}_status"], str)
            assert isinstance(status[f"{prefix}_T_now"], float)
            assert isinstance(status[f"{prefix}_timestamp"], float)
            assert isinstance(status[f"{prefix}_T_target"], float)
            assert isinstance(status[f"{prefix}_drive_level"], float)
            assert isinstance(status[f"{prefix}_enabled"], bool)
            assert isinstance(status[f"{prefix}_active"], bool)
            assert isinstance(status[f"{prefix}_int_disabled"], bool)
            assert isinstance(status[f"{prefix}_hysteresis"], float)
            assert isinstance(status[f"{prefix}_clamp"], float)


class TestTempMonStatusTypes:
    def test_status_field_types(self):
        emu = TempMonEmulator()
        emu.op()
        status = emu.get_status()
        for prefix in ("LNA", "LOAD"):
            assert isinstance(status[f"{prefix}_status"], str)
            assert isinstance(status[f"{prefix}_temp"], float)
            assert isinstance(status[f"{prefix}_timestamp"], float)


class TestImuStatusTypes:
    def test_status_field_types(self):
        emu = ImuEmulator()
        emu.op()
        status = emu.get_status()
        assert isinstance(status["sensor_name"], str)
        assert isinstance(status["status"], str)
        assert isinstance(status["app_id"], int)
        for key in ("yaw", "pitch", "roll", "accel_x", "accel_y", "accel_z"):
            assert isinstance(status[key], float), f"{key} should be float"


class TestLidarStatusTypes:
    def test_status_field_types(self):
        emu = LidarEmulator()
        emu.op()
        status = emu.get_status()
        assert isinstance(status["sensor_name"], str)
        assert isinstance(status["status"], str)
        assert isinstance(status["app_id"], int)
        assert isinstance(status["distance_m"], float)


class TestRFSwitchStatusTypes:
    def test_status_field_types(self):
        emu = RFSwitchEmulator(settle_ms=0)
        status = emu.get_status()
        assert isinstance(status["sensor_name"], str)
        assert isinstance(status["status"], str)
        assert isinstance(status["app_id"], int)
        assert isinstance(status["sw_state"], int)


# ---------------------------------------------------------------------------
# Malformed input handling
# ---------------------------------------------------------------------------


class TestMalformedInput:
    """Verify emulators survive bad input without crashing.

    The C firmware silently ignores malformed JSON (cJSON_Parse returns NULL)
    and non-numeric values (valueint/valuedouble return 0).  Emulators must
    do the same.
    """

    def test_motor_invalid_type_values(self):
        emu = MotorEmulator()
        emu.server({"az_set_pos": "not_a_number"})
        assert emu.azimuth.position == 0  # unchanged (default kept)

    def test_motor_null_value(self):
        emu = MotorEmulator()
        emu.server({"az_set_pos": None})
        assert emu.azimuth.position == 0

    def test_motor_empty_json(self):
        emu = MotorEmulator()
        emu.server({"az_set_target_pos": 500})
        emu.server({})
        assert emu.azimuth.target_pos == 500  # unchanged

    def test_motor_unknown_keys_ignored(self):
        emu = MotorEmulator()
        emu.server({"unknown_key": 42, "az_set_pos": 100})
        assert emu.azimuth.position == 100

    def test_rfswitch_invalid_type(self):
        emu = RFSwitchEmulator(settle_ms=0)
        emu.server({"sw_state": "abc"})
        # commanded_state must be untouched by a non-numeric payload.
        assert emu.commanded_state == 0
        assert emu.get_status()["sw_state"] == 0

    def test_rfswitch_null_value(self):
        emu = RFSwitchEmulator(settle_ms=0)
        emu.server({"sw_state": None})
        assert emu.commanded_state == 0
        assert emu.get_status()["sw_state"] == 0

    def test_tempctrl_invalid_type(self):
        emu = TempCtrlEmulator()
        emu.server({"LNA_temp_target": "hot"})
        assert emu.lna.T_target == 30.0  # unchanged (default)

    def test_tempctrl_null_value(self):
        emu = TempCtrlEmulator()
        emu.server({"LNA_clamp": None})
        assert emu.lna.clamp == 0.6  # unchanged


# ---------------------------------------------------------------------------
# Error state injection
# ---------------------------------------------------------------------------


class TestTempCtrlErrorState:
    def test_sensor_error_clear(self):
        emu = TempCtrlEmulator()
        emu.inject_sensor_error("LOAD")
        assert emu.get_status()["LOAD_status"] == "error"
        emu.inject_sensor_error("LOAD", error=False)
        assert emu.get_status()["LOAD_status"] == "update"

    def test_independent_channel_errors(self):
        emu = TempCtrlEmulator()
        emu.inject_sensor_error("LNA")
        emu.inject_sensor_error("LOAD")
        status = emu.get_status()
        assert status["LNA_status"] == "error"
        assert status["LOAD_status"] == "error"


class TestTempMonErrorState:
    def test_sensor_error_channel_load(self):
        emu = TempMonEmulator()
        emu.inject_sensor_error("LOAD")
        status = emu.get_status()
        assert status["LNA_status"] == "update"
        assert status["LOAD_status"] == "error"

    def test_sensor_error_clear(self):
        emu = TempMonEmulator()
        emu.inject_sensor_error("LNA")
        emu.inject_sensor_error("LNA", error=False)
        assert emu.get_status()["LNA_status"] == "update"


class TestImuErrorState:
    def test_init_failure_reports_error(self):
        """When not initialized, status reports error."""
        emu = ImuEmulator()
        emu.inject_init_failure()
        assert emu.get_status()["status"] == "error"


# ---------------------------------------------------------------------------
# Edge case behavioral tests
# ---------------------------------------------------------------------------


class TestTempCtrlEdgeCases:
    def test_both_channels_converge_independently(self):
        emu = TempCtrlEmulator()
        emu.lna.T_now = 20.0
        emu.load.T_now = 40.0
        emu.server(
            {
                "LNA_temp_target": 30.0,
                "LNA_enable": True,
                "LOAD_temp_target": 30.0,
                "LOAD_enable": True,
            }
        )
        for _ in range(1000):
            emu.op()
        assert abs(emu.lna.T_now - 30.0) < 0.5
        assert abs(emu.load.T_now - 30.0) < 0.5

    def test_disable_mid_convergence(self):
        emu = TempCtrlEmulator()
        emu.lna.T_now = 20.0
        emu.server({"LNA_temp_target": 40.0, "LNA_enable": True})
        for _ in range(100):
            emu.op()
        assert emu.lna.drive != 0.0
        emu.server({"LNA_enable": False})
        emu.op()
        assert emu.lna.drive == 0.0


ALL_EMULATORS = [
    MotorEmulator,
    TempCtrlEmulator,
    TempMonEmulator,
    ImuEmulator,
    LidarEmulator,
    RFSwitchEmulator,
]


class TestSensorName:
    """Ensure every emulator status contains 'sensor_name' for redis_handler."""

    @pytest.mark.parametrize(
        "emu_cls", ALL_EMULATORS, ids=lambda c: c.__name__
    )
    def test_get_status_has_sensor_name(self, emu_cls):
        emu = emu_cls()
        result = emu.get_status()
        # Normalize to list — composite emulators return multiple dicts,
        # each of which reaches redis_handler individually.
        statuses = result if isinstance(result, list) else [result]
        for status in statuses:
            assert "sensor_name" in status
