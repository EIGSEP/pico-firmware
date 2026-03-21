"""
Integration tests: emulators through DummyDevice interface with mock serial.

These tests verify the pipeline (PicoDevice -> MockSerial -> emulator thread ->
status -> reader thread). State machine logic is tested in test_emulators.py;
here we focus on: status arrives with correct fields, commands round-trip
through serial, and redis handler is called.
"""

import time
import pytest
from picohost.testing import (
    DummyPicoMotor,
    DummyPicoRFSwitch,
    DummyPicoPeltier,
    DummyPicoIMU,
    DummyPicoTempMon,
    DummyPicoLidar,
)

# Expected field sets from C firmware send_json calls
MOTOR_FIELDS = {
    "sensor_name", "status", "app_id",
    "az_pos", "az_target_pos", "el_pos", "el_target_pos",
}

TEMPCTRL_FIELDS = {
    "sensor_name", "app_id",
    "A_status", "A_T_now", "A_timestamp", "A_T_target",
    "A_drive_level", "A_enabled", "A_active", "A_int_disabled",
    "A_hysteresis", "A_clamp",
    "B_status", "B_T_now", "B_timestamp", "B_T_target",
    "B_drive_level", "B_enabled", "B_active", "B_int_disabled",
    "B_hysteresis", "B_clamp",
}

TEMPMON_FIELDS = {
    "sensor_name", "app_id",
    "A_status", "A_temp", "A_timestamp",
    "B_status", "B_temp", "B_timestamp",
}

IMU_FIELDS = {
    "sensor_name", "status", "app_id",
    "quat_i", "quat_j", "quat_k", "quat_real",
    "accel_x", "accel_y", "accel_z",
    "lin_accel_x", "lin_accel_y", "lin_accel_z",
    "gyro_x", "gyro_y", "gyro_z",
    "mag_x", "mag_y", "mag_z",
    "calibrated", "accel_cal", "mag_cal",
}

LIDAR_FIELDS = {"sensor_name", "status", "app_id", "distance_m"}

RFSWITCH_FIELDS = {"sensor_name", "status", "app_id", "sw_state"}


# --- Fixtures ---

@pytest.fixture
def motor():
    m = DummyPicoMotor("/dev/dummy")
    yield m
    m.disconnect()


@pytest.fixture
def rfswitch():
    s = DummyPicoRFSwitch("/dev/dummy")
    yield s
    s.disconnect()


@pytest.fixture
def peltier():
    p = DummyPicoPeltier("/dev/dummy")
    yield p
    p.disconnect()


@pytest.fixture
def imu():
    i = DummyPicoIMU("/dev/dummy")
    yield i
    i.disconnect()


@pytest.fixture
def tempmon():
    m = DummyPicoTempMon("/dev/dummy")
    yield m
    m.disconnect()


@pytest.fixture
def lidar():
    d = DummyPicoLidar("/dev/dummy")
    yield d
    d.disconnect()


# --- Motor ---

class TestMotorIntegration:

    def test_status_fields(self, motor):
        """Motor emulator populates all status fields via reader thread."""
        assert set(motor.status.keys()) == MOTOR_FIELDS
        assert motor.status["sensor_name"] == "motor"

    def test_command_round_trip(self, motor):
        """Command sent through serial is processed by emulator."""
        motor.motor_command(az_set_target_pos=500)
        time.sleep(0.3)
        assert motor.status["az_target_pos"] == 500
        assert motor.status["az_pos"] == motor.status["az_target_pos"]


# --- RFSwitch ---

class TestRFSwitchIntegration:

    def test_status_populated(self, rfswitch):
        """RFSwitch+IMU emulator sends status via reader thread."""
        time.sleep(0.2)
        # Composite emulator alternates rfswitch and imu status;
        # last_status holds whichever arrived last
        assert rfswitch.last_status.get("sensor_name") in ("rfswitch", "imu_antenna")

    def test_command_round_trip(self, rfswitch):
        """Switch command round-trips through serial."""
        rfswitch.switch("VNAO")
        time.sleep(0.3)
        assert rfswitch.last_status.get("sw_state") == rfswitch.paths["VNAO"]


# --- Peltier ---

class TestPeltierIntegration:

    def test_status_fields(self, peltier):
        """Peltier emulator populates all status fields via reader thread."""
        assert set(peltier.status.keys()) == TEMPCTRL_FIELDS
        assert peltier.status["sensor_name"] == "tempctrl"

    def test_command_round_trip(self, peltier):
        """Temperature control converges to target through serial pipeline."""
        peltier.set_temperature(T_A=35.0)
        peltier.set_enable(A=True)
        time.sleep(1.5)
        assert abs(peltier.status["A_T_now"] - 35.0) < 0.5


# --- IMU ---

class TestIMUIntegration:

    def test_status_fields(self, imu):
        """IMU emulator populates all status fields via reader thread."""
        time.sleep(0.2)
        assert set(imu.last_status.keys()) == IMU_FIELDS
        assert imu.last_status["sensor_name"] == "imu_panda"

    def test_calibrated_is_bool_over_serial(self, imu):
        """calibrated field must arrive as JSON boolean through the serial pipeline.

        This catches mismatches between firmware (KV_BOOL) and host code
        that previously compared against string "True"/"False".
        """
        time.sleep(0.2)
        assert isinstance(imu.last_status["calibrated"], bool)
        assert imu.last_status["calibrated"] is False

    def test_calibrate_round_trip(self, imu):
        """PicoIMU.calibrate() round-trips through serial and is processed.

        When both accel_status and mag_status are already 3 (fully
        calibrated), the emulator saves calibration and clears the flag
        in the same op() cycle — so by the time we read status, calibrated
        is back to False.  This matches real firmware behavior: calibration
        completes instantly when the sensor is already calibrated.

        To observe the transient True state, we lower accel_status so the
        calibration stays pending until we restore it.
        """
        imu._emulator.accel_status = 2  # prevent auto-clear
        imu.calibrate()
        time.sleep(0.3)
        assert imu.last_status["calibrated"] is True
        # Now let calibration complete
        imu._emulator.accel_status = 3
        time.sleep(0.3)
        assert imu.last_status["calibrated"] is False

    def test_status_types(self, imu):
        """Verify value types match the JSON protocol, not just field names.

        Field type mismatches (e.g. string vs bool) silently break host
        code that compares with 'is True' vs '== \"True\"'.
        """
        time.sleep(0.2)
        s = imu.last_status
        # Booleans
        assert isinstance(s["calibrated"], bool)
        # Integers
        assert isinstance(s["app_id"], int)
        assert isinstance(s["accel_cal"], int)
        assert isinstance(s["mag_cal"], int)
        # Floats
        for key in ("quat_i", "quat_j", "quat_k", "quat_real",
                     "accel_x", "accel_y", "accel_z",
                     "gyro_x", "gyro_y", "gyro_z",
                     "mag_x", "mag_y", "mag_z"):
            assert isinstance(s[key], float), f"{key} should be float, got {type(s[key])}"
        # Strings
        assert isinstance(s["sensor_name"], str)
        assert isinstance(s["status"], str)


# --- Motor ---

class TestMotorIntegrationTypes:

    def test_status_types(self, motor):
        """Verify motor status value types through serial pipeline."""
        s = motor.status
        assert isinstance(s["sensor_name"], str)
        assert isinstance(s["status"], str)
        assert isinstance(s["app_id"], int)
        assert isinstance(s["az_pos"], int)
        assert isinstance(s["az_target_pos"], int)
        assert isinstance(s["el_pos"], int)
        assert isinstance(s["el_target_pos"], int)


# --- Peltier ---

class TestPeltierIntegrationTypes:

    def test_status_types(self, peltier):
        """Verify peltier status value types through serial pipeline."""
        s = peltier.status
        assert isinstance(s["sensor_name"], str)
        assert isinstance(s["app_id"], int)
        # Booleans
        for key in ("A_enabled", "A_active", "A_int_disabled",
                     "B_enabled", "B_active", "B_int_disabled"):
            assert isinstance(s[key], bool), f"{key} should be bool, got {type(s[key])}"
        # Floats
        for key in ("A_T_now", "A_T_target", "A_drive_level",
                     "A_hysteresis", "A_clamp", "A_timestamp",
                     "B_T_now", "B_T_target", "B_drive_level",
                     "B_hysteresis", "B_clamp", "B_timestamp"):
            assert isinstance(s[key], (int, float)), f"{key} should be numeric, got {type(s[key])}"


# --- TempMon ---

class TestTempMonIntegration:

    def test_status_fields(self, tempmon):
        """TempMon emulator populates all status fields via reader thread."""
        time.sleep(0.2)
        assert set(tempmon.last_status.keys()) == TEMPMON_FIELDS
        assert tempmon.last_status["sensor_name"] == "temp_mon"


# --- Lidar ---

class TestLidarIntegration:

    def test_status_fields(self, lidar):
        """Lidar emulator populates all status fields via reader thread."""
        time.sleep(0.2)
        assert set(lidar.last_status.keys()) == LIDAR_FIELDS
        assert lidar.last_status["sensor_name"] == "lidar"


# --- Redis ---

class TestRedisIntegration:

    def test_redis_handler_called(self):
        """Verify that redis_handler receives status data when configured."""
        received = []

        class FakeRedis:
            def add_metadata(self, name, data):
                received.append((name, data))

        mon = DummyPicoTempMon("/dev/dummy", eig_redis=FakeRedis())
        time.sleep(0.3)
        mon.disconnect()

        assert len(received) > 0
        names = [name for name, _ in received]
        assert "temp_mon" in names
