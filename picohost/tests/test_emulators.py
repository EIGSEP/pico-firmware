"""
Unit tests for firmware emulators.
Tests emulators standalone (no mock serial), calling methods directly.
"""

import math
from picohost.emulators import (
    MotorEmulator,
    TempCtrlEmulator,
    TempMonEmulator,
    ImuEmulator,
    LidarEmulator,
    RFSwitchEmulator,
    RFSwitchWithImuEmulator,
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

    def test_set_pos_resets_target(self):
        """Matching C behavior: az_set_pos resets target to position."""
        emu = MotorEmulator()
        emu.server({"az_set_target_pos": 500})
        for _ in range(10):
            emu.op()
        emu.server({"az_set_pos": 0})
        assert emu.azimuth.position == 0
        assert emu.azimuth.target_pos == 0

    def test_halt(self):
        emu = MotorEmulator()
        emu.server({"az_set_target_pos": 1000})
        for _ in range(5):
            emu.op()
        pos_before = emu.azimuth.position
        assert pos_before > 0 and pos_before < 1000
        # C firmware only checks key presence via cJSON_GetObjectItem(root, "halt");
        # the value (0 here) is irrelevant. Using 0 rather than true because
        # true would imply false disables halt, which is not the case.
        emu.server({"halt": 0})
        assert emu.azimuth.target_pos == emu.azimuth.position

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

    def test_hysteresis_band(self):
        """When T_now is within hysteresis of target, drive should be 0."""
        emu = TempCtrlEmulator()
        emu.A.T_now = 29.8
        emu.server({"A_temp_target": 30.0, "A_enable": True, "A_hysteresis": 0.5})
        emu.op()
        assert emu.A.drive == 0.0
        assert emu.A.active is False

    def test_clamp_bounded(self):
        emu = TempCtrlEmulator()
        emu.server({"A_clamp": 1.5})
        assert emu.A.clamp == 1.0
        emu.server({"A_clamp": -0.5})
        assert emu.A.clamp == 0.0

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

    def test_no_commands(self):
        emu = TempMonEmulator()
        emu.server({"anything": True})  # should be no-op
        assert emu.temp_a == 25.0  # unchanged before any op()

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
        assert status["calibrated"] == "False"

    def test_antenna_name(self):
        emu = ImuEmulator(app_id=5)
        assert emu.name == "imu_antenna"

    def test_quaternion_normalization(self):
        emu = ImuEmulator()
        for _ in range(100):
            emu.op()
        norm = math.sqrt(sum(x * x for x in emu.q))
        assert abs(norm - 1.0) < 0.01

    def test_calibration_flow(self):
        """Calibrate command sets flag, op clears it when statuses == 3."""
        emu = ImuEmulator()
        emu.accel_status = 3
        emu.mag_status = 3
        emu.server({"calibrate": True})
        assert emu.do_calibration is True
        assert emu.get_status()["calibrated"] == "True"
        emu.op()
        assert emu.do_calibration is False
        assert emu.get_status()["calibrated"] == "False"

    def test_calibration_waits_for_status(self):
        """Calibration doesn't clear until both statuses reach 3."""
        emu = ImuEmulator()
        emu.accel_status = 2
        emu.mag_status = 3
        emu.server({"calibrate": True})
        emu.op()
        assert emu.do_calibration is True  # not cleared yet

    def test_calibrate_false_is_noop(self):
        """{"calibrate": false} does not trigger calibration (matches cJSON_IsTrue)."""
        emu = ImuEmulator()
        emu.server({"calibrate": False})
        assert emu.do_calibration is False

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
        assert status["distance_m"] == 1.5

    def test_noise_is_mean_reverting(self):
        emu = LidarEmulator()
        for _ in range(1000):
            emu.op()
        # Mean-reverting noise stays tightly around base distance
        assert abs(emu.distance - 1.5) < 0.5

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

    def test_set_state(self):
        emu = RFSwitchEmulator()
        emu.server({"sw_state": 42})
        assert emu.sw_state == 42
        assert emu.get_status()["sw_state"] == 42

    def test_status_fields(self):
        emu = RFSwitchEmulator()
        status = emu.get_status()
        expected_keys = {"sensor_name", "status", "app_id", "sw_state"}
        assert set(status.keys()) == expected_keys


class TestRFSwitchWithImuEmulator:

    def test_composite_status(self):
        """Composite emulator returns both IMU and RFSwitch status."""
        emu = RFSwitchWithImuEmulator()
        status = emu.get_status()
        assert isinstance(status, list)
        assert len(status) == 2
        sensor_names = {s["sensor_name"] for s in status}
        assert "imu_antenna" in sensor_names
        assert "rfswitch" in sensor_names

    def test_composite_server(self):
        """Commands dispatch to both sub-emulators."""
        emu = RFSwitchWithImuEmulator()
        emu.server({"sw_state": 7})
        assert emu._rfswitch.sw_state == 7

        emu.server({"calibrate": True})
        assert emu._imu.do_calibration is True
