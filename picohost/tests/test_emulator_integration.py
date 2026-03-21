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
