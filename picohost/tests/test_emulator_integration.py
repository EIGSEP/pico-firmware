"""
Integration tests: emulators through DummyDevice interface with mock serial.

These tests verify the pipeline (PicoDevice -> MockSerial -> emulator thread ->
status -> reader thread). State machine logic is tested in test_emulators.py;
here we focus on: status arrives with correct fields, commands round-trip
through serial, convergence timing matches emulator model, and redis handler
is called.
"""

import math
import time
import pytest
from conftest import wait_for_condition, wait_for_settle
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
    "watchdog_tripped", "watchdog_timeout_ms",
    "LNA_status", "LNA_T_now", "LNA_timestamp", "LNA_T_target",
    "LNA_drive_level", "LNA_enabled", "LNA_active", "LNA_int_disabled",
    "LNA_hysteresis", "LNA_clamp",
    "LOAD_status", "LOAD_T_now", "LOAD_timestamp", "LOAD_T_target",
    "LOAD_drive_level", "LOAD_enabled", "LOAD_active", "LOAD_int_disabled",
    "LOAD_hysteresis", "LOAD_clamp",
}

TEMPMON_FIELDS = {
    "sensor_name", "app_id",
    "LNA_status", "LNA_temp", "LNA_timestamp",
    "LOAD_status", "LOAD_temp", "LOAD_timestamp",
}

IMU_FIELDS = {
    "sensor_name", "status", "app_id",
    "yaw", "pitch", "roll",
    "accel_x", "accel_y", "accel_z",
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
        cadence = motor.EMULATOR_CADENCE_MS
        before = motor.status.get("az_target_pos")
        motor.motor_command(az_set_target_pos=500)
        assert wait_for_settle(
            lambda: motor.status.get("az_target_pos"),
            initial=before, cadence_ms=cadence, max_cycles=10,
        ) == 500
        # Motor moves 60 steps/op -> ceil(500/60)=9 ops + margin
        steps_per_op = motor._emulator.azimuth.max_pulses
        expected_ops = math.ceil(500 / steps_per_op)
        assert wait_for_settle(
            lambda: motor.status.get("az_pos"),
            initial=0, cadence_ms=cadence, max_cycles=expected_ops + 10,
        ) == 500


# --- RFSwitch ---

class TestRFSwitchIntegration:

    def test_status_populated(self, rfswitch):
        """RFSwitch emulator sends status via reader thread."""
        cadence = rfswitch.EMULATOR_CADENCE_MS
        wait_for_condition(
            lambda: rfswitch.last_status.get("sensor_name") is not None,
            cadence_ms=cadence,
        )
        assert rfswitch.last_status["sensor_name"] == "rfswitch"

    def test_command_round_trip(self, rfswitch):
        """Switch command round-trips through serial."""
        cadence = rfswitch.EMULATOR_CADENCE_MS
        before = rfswitch.last_status.get("sw_state")
        rfswitch.switch("VNAO")
        assert wait_for_settle(
            lambda: rfswitch.last_status.get("sw_state"),
            initial=before, cadence_ms=cadence, max_cycles=10,
        ) == rfswitch.paths["VNAO"]


# --- Peltier ---

class TestPeltierIntegration:

    def test_status_fields(self, peltier):
        """Peltier emulator populates all status fields via reader thread."""
        assert set(peltier.status.keys()) == TEMPCTRL_FIELDS
        assert peltier.status["sensor_name"] == "tempctrl"

    def test_command_round_trip(self, peltier):
        """Temperature control converges to target through serial pipeline."""
        cadence = peltier.EMULATOR_CADENCE_MS
        peltier.set_temperature(T_LNA=35.0)
        peltier.set_enable(LNA=True)
        # 10°C delta, drive clamped at 0.6, drift 0.05/op -> ~0.03°C/op
        # ~333 ops to converge + margin for hysteresis settling
        settled = wait_for_settle(
            lambda: round(peltier.status.get("LNA_T_now", 0), 1),
            cadence_ms=cadence, max_cycles=500, stable_count=5,
        )
        assert abs(settled - 35.0) <= 0.5


# --- IMU ---

class TestIMUIntegration:

    def test_status_fields(self, imu):
        """IMU emulator populates all status fields via reader thread."""
        cadence = imu.EMULATOR_CADENCE_MS
        wait_for_condition(
            lambda: len(imu.last_status) > 0, cadence_ms=cadence,
        )
        assert set(imu.last_status.keys()) == IMU_FIELDS
        assert imu.last_status["sensor_name"] == "imu_el"

    def test_status_types(self, imu):
        """Verify value types match the JSON protocol, not just field names."""
        cadence = imu.EMULATOR_CADENCE_MS
        wait_for_condition(
            lambda: len(imu.last_status) > 0, cadence_ms=cadence,
        )
        s = imu.last_status
        # Integers
        assert isinstance(s["app_id"], int)
        # Floats
        for key in ("yaw", "pitch", "roll",
                     "accel_x", "accel_y", "accel_z"):
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
        for key in ("LNA_enabled", "LNA_active", "LNA_int_disabled",
                     "LOAD_enabled", "LOAD_active", "LOAD_int_disabled"):
            assert isinstance(s[key], bool), f"{key} should be bool, got {type(s[key])}"
        # Floats
        for key in ("LNA_T_now", "LNA_T_target", "LNA_drive_level",
                     "LNA_hysteresis", "LNA_clamp", "LNA_timestamp",
                     "LOAD_T_now", "LOAD_T_target", "LOAD_drive_level",
                     "LOAD_hysteresis", "LOAD_clamp", "LOAD_timestamp"):
            assert isinstance(s[key], (int, float)), f"{key} should be numeric, got {type(s[key])}"


# --- TempMon ---

class TestTempMonIntegration:

    def test_status_fields(self, tempmon):
        """TempMon emulator populates all status fields via reader thread."""
        cadence = tempmon.EMULATOR_CADENCE_MS
        wait_for_condition(
            lambda: len(tempmon.last_status) > 0, cadence_ms=cadence,
        )
        assert set(tempmon.last_status.keys()) == TEMPMON_FIELDS
        assert tempmon.last_status["sensor_name"] == "temp_mon"

    def test_status_types(self, tempmon):
        """Verify tempmon status value types through serial pipeline."""
        cadence = tempmon.EMULATOR_CADENCE_MS
        wait_for_condition(
            lambda: len(tempmon.last_status) > 0, cadence_ms=cadence,
        )
        s = tempmon.last_status
        assert isinstance(s["sensor_name"], str)
        assert isinstance(s["app_id"], int)
        for ch in ("LNA", "LOAD"):
            assert isinstance(s[f"{ch}_status"], str)
            assert isinstance(s[f"{ch}_temp"], (int, float)), \
                f"{ch}_temp should be numeric"
            assert isinstance(s[f"{ch}_timestamp"], (int, float)), \
                f"{ch}_timestamp should be numeric"


# --- Lidar ---

class TestLidarIntegration:

    def test_status_fields(self, lidar):
        """Lidar emulator populates all status fields via reader thread."""
        cadence = lidar.EMULATOR_CADENCE_MS
        wait_for_condition(
            lambda: len(lidar.last_status) > 0, cadence_ms=cadence,
        )
        assert set(lidar.last_status.keys()) == LIDAR_FIELDS
        assert lidar.last_status["sensor_name"] == "lidar"

    def test_status_types(self, lidar):
        """Verify lidar status value types through serial pipeline."""
        cadence = lidar.EMULATOR_CADENCE_MS
        wait_for_condition(
            lambda: len(lidar.last_status) > 0, cadence_ms=cadence,
        )
        s = lidar.last_status
        assert isinstance(s["sensor_name"], str)
        assert isinstance(s["status"], str)
        assert isinstance(s["app_id"], int)
        assert isinstance(s["distance_m"], (int, float))


# --- RFSwitch ---

class TestRFSwitchIntegrationTypes:

    def test_status_types(self, rfswitch):
        """Verify rfswitch status value types through serial pipeline."""
        cadence = rfswitch.EMULATOR_CADENCE_MS
        wait_for_condition(
            lambda: rfswitch.last_status.get("sensor_name") == "rfswitch",
            cadence_ms=cadence,
        )
        s = rfswitch.last_status
        assert s["sensor_name"] == "rfswitch"
        assert isinstance(s["status"], str)
        assert isinstance(s["app_id"], int)
        assert isinstance(s["sw_state"], int)


# --- Peltier Watchdog ---

class TestPeltierWatchdog:

    def test_keepalive_prevents_watchdog(self):
        """Keepalive commands prevent the watchdog from tripping."""
        p = DummyPicoPeltier("/dev/dummy", keepalive_interval=0.2)
        try:
            p.set_watchdog_timeout(500)
            time.sleep(1.0)
            assert p.status.get("watchdog_tripped") is False
        finally:
            p.disconnect()

    def test_watchdog_trips_without_keepalive(self):
        """Without keepalive, the watchdog trips and disables peltiers."""
        p = DummyPicoPeltier("/dev/dummy", keepalive_interval=0)
        try:
            cadence = p.EMULATOR_CADENCE_MS
            p.set_watchdog_timeout(200)
            # 200ms watchdog / 50ms cadence = 4 ops + margin
            wait_for_condition(
                lambda: p.status.get("watchdog_tripped") is True,
                cadence_ms=cadence, max_cycles=20,
            )
        finally:
            p.disconnect()


# --- Redis ---

class TestRedisIntegration:

    def test_redis_handler_called(self):
        """Verify that redis_handler receives status data when configured."""
        received = []

        class FakeRedis:
            def add_metadata(self, name, data):
                received.append((name, data))

        mon = DummyPicoTempMon("/dev/dummy", eig_redis=FakeRedis())
        cadence = mon.EMULATOR_CADENCE_MS
        wait_for_condition(
            lambda: len(received) > 0, cadence_ms=cadence,
        )
        mon.disconnect()
        names = [name for name, _ in received]
        assert "temp_mon" in names


# --- Convergence Timing ---

class TestConvergenceTiming:
    """Verify that state changes complete within the cycle count predicted
    by each emulator's physics model.

    These tests catch regressions where the emulator becomes slower than
    the model predicts (e.g. a change to steps_per_op or drift rate that
    doesn't get reflected in the expected bounds).
    """

    def test_motor_convergence_cycles(self, motor):
        """Motor reaches target within ceil(distance / steps_per_op) + margin."""
        cadence = motor.EMULATOR_CADENCE_MS
        target = 500
        steps_per_op = motor._emulator.azimuth.max_pulses
        expected_ops = math.ceil(target / steps_per_op)
        # margin covers serial round-trip + status delivery latency
        margin = 10
        motor.motor_command(az_set_target_pos=target)
        settled = wait_for_settle(
            lambda: motor.status.get("az_pos"),
            initial=0, cadence_ms=cadence, max_cycles=expected_ops + margin,
        )
        assert settled == target

    def test_motor_large_move(self, motor):
        """Larger move scales linearly with step count."""
        cadence = motor.EMULATOR_CADENCE_MS
        target = 3000
        steps_per_op = motor._emulator.azimuth.max_pulses
        expected_ops = math.ceil(target / steps_per_op)
        margin = 10
        motor.motor_command(az_set_target_pos=target)
        settled = wait_for_settle(
            lambda: motor.status.get("az_pos"),
            initial=0, cadence_ms=cadence, max_cycles=expected_ops + margin,
        )
        assert settled == target

    def test_motor_both_axes(self, motor):
        """Both axes converge within the slower axis's predicted time."""
        cadence = motor.EMULATOR_CADENCE_MS
        az_target, el_target = 600, 300
        steps_per_op = motor._emulator.azimuth.max_pulses
        slowest_ops = max(
            math.ceil(az_target / steps_per_op),
            math.ceil(el_target / steps_per_op),
        )
        margin = 10
        motor.motor_command(
            az_set_target_pos=az_target, el_set_target_pos=el_target,
        )
        az = wait_for_settle(
            lambda: motor.status.get("az_pos"),
            initial=0, cadence_ms=cadence, max_cycles=slowest_ops + margin,
        )
        el = wait_for_settle(
            lambda: motor.status.get("el_pos"),
            initial=0, cadence_ms=cadence, max_cycles=slowest_ops + margin,
        )
        assert az == az_target
        assert el == el_target

    def test_command_ack_within_few_cycles(self, motor):
        """Target-position update is visible in status within a few cycles."""
        cadence = motor.EMULATOR_CADENCE_MS
        before = motor.status.get("az_target_pos")
        motor.motor_command(az_set_target_pos=999)
        # Should appear within ~2-3 status cycles (command + next status send)
        settled = wait_for_settle(
            lambda: motor.status.get("az_target_pos"),
            initial=before, cadence_ms=cadence, max_cycles=6,
        )
        assert settled == 999

    def test_peltier_convergence_bounded(self):
        """Temperature converges within a cycle count derived from the model.

        With default params (gain=0.2, baseline=0.4, clamp=0.6) and
        THERMAL_DRIFT_RATE=0.05, each op moves ~0.03°C when clamped.
        A 10°C delta (25->35) needs ~333 ops; we allow 500 as upper bound.
        """
        p = DummyPicoPeltier("/dev/dummy", keepalive_interval=0.2)
        try:
            cadence = p.EMULATOR_CADENCE_MS
            p.set_temperature(T_LNA=35.0)
            p.set_enable(LNA=True)
            settled = wait_for_settle(
                lambda: round(p.status.get("LNA_T_now", 0), 1),
                cadence_ms=cadence, max_cycles=500, stable_count=5,
            )
            assert abs(settled - 35.0) <= 0.5
        finally:
            p.disconnect()

    def test_watchdog_trip_timing(self):
        """Watchdog trips within predicted wall-time cycles.

        With 200ms timeout and 50ms cadence, the watchdog should trip
        within ~4-8 emulator cycles (200ms / ~1ms loop + cadence jitter).
        """
        p = DummyPicoPeltier("/dev/dummy", keepalive_interval=0)
        try:
            cadence = p.EMULATOR_CADENCE_MS
            p.set_watchdog_timeout(200)
            watchdog_cycles = math.ceil(200 / cadence)
            # Allow generous margin for thread scheduling, but still
            # much tighter than the old hardcoded 2s timeout.
            wait_for_condition(
                lambda: p.status.get("watchdog_tripped") is True,
                cadence_ms=cadence, max_cycles=watchdog_cycles + 15,
            )
        finally:
            p.disconnect()
