"""
Unit tests for firmware emulators.
Tests emulators standalone (no mock serial), calling methods directly.
"""

import numpy as np
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
            "sensor_name", "status", "app_id",
            "az_pos", "az_target_pos", "el_pos", "el_target_pos",
        }
        assert set(status.keys()) == expected_keys


class TestTempCtrlEmulator:

    def test_initial_state(self):
        emu = TempCtrlEmulator()
        status = emu.get_status()
        assert status["sensor_name"] == "tempctrl"
        assert status["A_enabled"] is False
        assert status["B_enabled"] is False
        assert status["A_T_target"] == 30.0

    def test_enable_and_converge(self):
        """Enable channel A, verify convergence to within hysteresis of target."""
        emu = TempCtrlEmulator()
        emu.A.T_now = 25.0
        emu.server({"A_temp_target": 30.0, "A_enable": True, "A_hysteresis": 0.5})
        for _ in range(500):
            emu.op()
        assert abs(emu.A.T_now - 30.0) < 0.5

    def test_converge_to_non_default_target(self):
        """Converge to a target different from the 30.0 default."""
        emu = TempCtrlEmulator()
        emu.A.T_now = 25.0
        emu.server({"A_temp_target": 40.0, "A_enable": True, "A_hysteresis": 0.5})
        for _ in range(1000):
            emu.op()
        assert abs(emu.A.T_now - 40.0) < 0.5

    def test_cool_back_down(self):
        """Heat to target, then set a lower target and verify cooling."""
        emu = TempCtrlEmulator()
        emu.A.T_now = 25.0
        emu.server({"A_temp_target": 35.0, "A_enable": True, "A_hysteresis": 0.5})
        for _ in range(1000):
            emu.op()
        assert abs(emu.A.T_now - 35.0) < 0.5
        # Now cool back to 25
        emu.server({"A_temp_target": 25.0})
        for _ in range(1000):
            emu.op()
        assert abs(emu.A.T_now - 25.0) < 0.5

    def test_channel_b(self):
        """Channel B works independently of channel A."""
        emu = TempCtrlEmulator()
        emu.B.T_now = 20.0
        emu.server({"B_temp_target": 28.0, "B_enable": True, "B_hysteresis": 0.5})
        for _ in range(1000):
            emu.op()
        assert abs(emu.B.T_now - 28.0) < 0.5
        # Channel A should not have moved (not enabled)
        assert emu.A.drive == 0.0

    def test_status_fields(self):
        emu = TempCtrlEmulator()
        status = emu.get_status()
        expected_keys = {
            "sensor_name", "app_id",
            "A_status", "A_T_now", "A_timestamp", "A_T_target",
            "A_drive_level", "A_enabled", "A_active", "A_int_disabled",
            "A_hysteresis", "A_clamp",
            "B_status", "B_T_now", "B_timestamp", "B_T_target",
            "B_drive_level", "B_enabled", "B_active", "B_int_disabled",
            "B_hysteresis", "B_clamp",
        }
        assert set(status.keys()) == expected_keys


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
        assert abs(emu.temp_a - 25.0) < 1.0
        assert abs(emu.temp_b - 25.0) < 1.0

    def test_status_fields(self):
        emu = TempMonEmulator()
        status = emu.get_status()
        expected_keys = {
            "sensor_name", "app_id",
            "A_status", "A_temp", "A_timestamp",
            "B_status", "B_temp", "B_timestamp",
        }
        assert set(status.keys()) == expected_keys


class TestImuEmulator:

    def test_initial_state(self):
        emu = ImuEmulator(app_id=3)
        assert emu.name == "imu_panda"
        status = emu.get_status()
        assert status["sensor_name"] == "imu_panda"
        assert status["calibrated"] is False

    def test_antenna_name(self):
        emu = ImuEmulator(app_id=5)
        assert emu.name == "imu_antenna"

    def test_quaternion_normalization(self):
        emu = ImuEmulator()
        for _ in range(100):
            emu.op()
        assert abs(np.linalg.norm(emu.q) - 1.0) < 0.01

    def test_gravity_consistency(self):
        """Accelerometer should read ~9.81 m/s^2 magnitude (stationary)."""
        emu = ImuEmulator()
        for _ in range(10):
            emu.op()
        assert abs(np.linalg.norm(emu.a) - 9.81) < 0.1

    def test_orientation_from_angles(self):
        """Setting az/el angles produces consistent quaternion."""
        emu = ImuEmulator()
        emu.az_angle = np.pi / 4  # 45 degrees azimuth
        emu.el_angle = 0.0
        emu.op()
        # Quaternion should represent ~45 deg rotation around z
        assert abs(emu.q[3] - np.cos(np.pi / 8)) < 0.02  # real part
        assert abs(emu.q[2] - np.sin(np.pi / 8)) < 0.02  # k component (z-axis)

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
            "sensor_name", "status", "app_id",
            "quat_i", "quat_j", "quat_k", "quat_real",
            "accel_x", "accel_y", "accel_z",
            "lin_accel_x", "lin_accel_y", "lin_accel_z",
            "gyro_x", "gyro_y", "gyro_z",
            "mag_x", "mag_y", "mag_z",
            "calibrated", "accel_cal", "mag_cal",
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

    def test_initial_state(self):
        emu = RFSwitchEmulator()
        status = emu.get_status()
        assert status["sensor_name"] == "rfswitch"
        assert status["sw_state"] == 0

    def test_status_fields(self):
        emu = RFSwitchEmulator()
        status = emu.get_status()
        expected_keys = {"sensor_name", "status", "app_id", "sw_state"}
        assert set(status.keys()) == expected_keys


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
        for prefix in ("A", "B"):
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
        for prefix in ("A", "B"):
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
        for key in ("quat_i", "quat_j", "quat_k", "quat_real",
                     "accel_x", "accel_y", "accel_z",
                     "lin_accel_x", "lin_accel_y", "lin_accel_z",
                     "gyro_x", "gyro_y", "gyro_z",
                     "mag_x", "mag_y", "mag_z"):
            assert isinstance(status[key], float), f"{key} should be float"
        # calibrated is a BOOL in C firmware (KV_BOOL)
        assert isinstance(status["calibrated"], bool)
        assert isinstance(status["accel_cal"], int)
        assert isinstance(status["mag_cal"], int)


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
        emu = RFSwitchEmulator()
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
        emu = RFSwitchEmulator()
        emu.server({"sw_state": "abc"})
        assert emu.sw_state == 0  # unchanged

    def test_rfswitch_null_value(self):
        emu = RFSwitchEmulator()
        emu.server({"sw_state": None})
        assert emu.sw_state == 0

    def test_tempctrl_invalid_type(self):
        emu = TempCtrlEmulator()
        emu.server({"A_temp_target": "hot"})
        assert emu.A.T_target == 30.0  # unchanged (default)

    def test_tempctrl_null_value(self):
        emu = TempCtrlEmulator()
        emu.server({"A_clamp": None})
        assert emu.A.clamp == 0.6  # unchanged


# ---------------------------------------------------------------------------
# Error state injection
# ---------------------------------------------------------------------------

class TestTempCtrlErrorState:

    def test_sensor_error_clear(self):
        emu = TempCtrlEmulator()
        emu.inject_sensor_error("B")
        assert emu.get_status()["B_status"] == "error"
        emu.inject_sensor_error("B", error=False)
        assert emu.get_status()["B_status"] == "update"

    def test_independent_channel_errors(self):
        emu = TempCtrlEmulator()
        emu.inject_sensor_error("A")
        emu.inject_sensor_error("B")
        status = emu.get_status()
        assert status["A_status"] == "error"
        assert status["B_status"] == "error"


class TestTempMonErrorState:

    def test_sensor_error_channel_b(self):
        emu = TempMonEmulator()
        emu.inject_sensor_error("B")
        status = emu.get_status()
        assert status["A_status"] == "update"
        assert status["B_status"] == "error"

    def test_sensor_error_clear(self):
        emu = TempMonEmulator()
        emu.inject_sensor_error("A")
        emu.inject_sensor_error("A", error=False)
        assert emu.get_status()["A_status"] == "update"


class TestImuErrorState:

    def test_init_failure_calibration_noop(self):
        """When not initialized, calibrate command should have no effect.

        In C firmware, calibrate_imu() returns early if !is_initialized.
        """
        emu = ImuEmulator()
        emu.inject_init_failure()
        emu.server({"calibrate": True})
        # The server still sets the flag (firmware does too), but the
        # status should report error
        assert emu.get_status()["status"] == "error"


# ---------------------------------------------------------------------------
# Edge case behavioral tests
# ---------------------------------------------------------------------------

class TestTempCtrlEdgeCases:

    def test_both_channels_converge_independently(self):
        emu = TempCtrlEmulator()
        emu.A.T_now = 20.0
        emu.B.T_now = 40.0
        emu.server({
            "A_temp_target": 30.0, "A_enable": True,
            "B_temp_target": 30.0, "B_enable": True,
        })
        for _ in range(1000):
            emu.op()
        assert abs(emu.A.T_now - 30.0) < 0.5
        assert abs(emu.B.T_now - 30.0) < 0.5

    def test_disable_mid_convergence(self):
        emu = TempCtrlEmulator()
        emu.A.T_now = 20.0
        emu.server({"A_temp_target": 40.0, "A_enable": True})
        for _ in range(100):
            emu.op()
        assert emu.A.drive != 0.0
        emu.server({"A_enable": False})
        emu.op()
        assert emu.A.drive == 0.0


