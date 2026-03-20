"""
Integration tests: emulators through DummyDevice interface with mock serial.
"""

import time
from picohost.testing import (
    DummyPicoMotor,
    DummyPicoRFSwitch,
    DummyPicoPeltier,
    DummyPicoIMU,
    DummyPicoTempMon,
    DummyPicoLidar,
)


class TestMotorIntegration:

    def test_status_populated(self):
        """Motor emulator populates status via reader thread."""
        motor = DummyPicoMotor("/dev/dummy")
        assert "az_pos" in motor.status
        assert motor.status["sensor_name"] == "motor"
        motor.disconnect()

    def test_set_target_reflects_in_status(self):
        motor = DummyPicoMotor("/dev/dummy")
        motor.motor_command(az_set_target_pos=500)
        time.sleep(0.3)
        assert motor.status["az_target_pos"] == 500
        motor.disconnect()

    def test_motor_moves_to_target(self):
        motor = DummyPicoMotor("/dev/dummy")
        motor.motor_command(az_set_target_pos=100)
        # Wait enough for emulator op() to converge
        time.sleep(1.0)
        assert motor.status["az_pos"] == 100
        motor.disconnect()


class TestRFSwitchIntegration:

    def test_status_populated(self):
        switch = DummyPicoRFSwitch("/dev/dummy")
        time.sleep(0.2)
        # Should have both rfswitch and imu status merged in last_status
        assert "sw_state" in switch.last_status or "sensor_name" in switch.last_status
        switch.disconnect()

    def test_switch_state_change(self):
        switch = DummyPicoRFSwitch("/dev/dummy")
        switch.switch("VNAO")
        time.sleep(0.3)
        assert switch.last_status.get("sw_state") == switch.paths["VNAO"]
        switch.disconnect()


class TestPeltierIntegration:

    def test_status_populated(self):
        peltier = DummyPicoPeltier("/dev/dummy")
        assert "A_T_now" in peltier.status
        assert peltier.status["sensor_name"] == "tempctrl"
        peltier.disconnect()

    def test_temperature_control(self):
        """Enable control, verify temperature moves toward target."""
        peltier = DummyPicoPeltier("/dev/dummy")
        peltier.set_temperature(T_A=35.0)
        peltier.set_enable(A=True)
        time.sleep(0.5)
        # Temperature should have started moving from 25 toward 35
        assert peltier.status["A_T_now"] > 25.0
        peltier.disconnect()


class TestIMUIntegration:

    def test_status_populated(self):
        imu = DummyPicoIMU("/dev/dummy")
        time.sleep(0.2)
        assert imu.last_status.get("sensor_name") == "imu_panda"
        assert "quat_i" in imu.last_status
        imu.disconnect()


class TestTempMonIntegration:

    def test_status_populated(self):
        mon = DummyPicoTempMon("/dev/dummy")
        time.sleep(0.2)
        assert mon.last_status.get("sensor_name") == "temp_mon"
        assert "A_temp" in mon.last_status
        assert "B_temp" in mon.last_status
        mon.disconnect()


class TestLidarIntegration:

    def test_status_populated(self):
        lidar = DummyPicoLidar("/dev/dummy")
        time.sleep(0.2)
        assert lidar.last_status.get("sensor_name") == "lidar"
        assert "distance_m" in lidar.last_status
        lidar.disconnect()


class TestRedisIntegration:

    def test_redis_handler_called(self):
        """Verify that redis_handler receives status data when configured."""
        received = []

        class FakeRedis:
            def add_metadata(self, name, data):
                received.append((name, data))

        # Use DummyPicoTempMon which inherits PicoDevice directly
        # (PicoMotor.__init__ doesn't forward eig_redis)
        mon = DummyPicoTempMon("/dev/dummy", eig_redis=FakeRedis())
        time.sleep(0.3)
        mon.disconnect()

        assert len(received) > 0
        names = [name for name, _ in received]
        assert "temp_mon" in names
