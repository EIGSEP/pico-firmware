import time

import numpy as np

from .base import PicoEmulator

NOISE_STDDEV = 0.001
IMU_EVENT_TIMEOUT_S = 5.0  # matches IMU_EVENT_TIMEOUT_MS in imu.h


class ImuEmulator(PicoEmulator):
    """Emulates src/imu.c firmware (UART RVC mode).

    Models orientation as rotations around two axes (azimuth and
    elevation), with sensor readings derived from the orientation.
    The BNO08x in RVC mode outputs euler angles and acceleration;
    this emulator produces matching values.
    """

    def __init__(self, app_id=3, **kwargs):
        # Orientation state (radians)
        self.az_angle = 0.0
        self.el_angle = 0.0
        # Sensor data (degrees for angles, m/s^2 for accel)
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 9.81
        self.is_initialized = True
        self._last_event_time = time.monotonic()
        self._sensor_failed = False  # set via simulate_sensor_failure()
        # Name depends on app_id: mirrors imu.cpp init_eigsep_imu()
        APP_IMU_EL, APP_IMU_AZ = 3, 6
        if app_id == APP_IMU_EL:
            self.name = "imu_el"
        elif app_id == APP_IMU_AZ:
            self.name = "imu_az"
        else:
            raise ValueError(f"ImuEmulator: unexpected app_id={app_id}, expected {APP_IMU_EL} or {APP_IMU_AZ}")
        super().__init__(app_id=app_id, **kwargs)

    def init(self):
        pass  # state set in __init__

    def inject_init_failure(self):
        """Simulate a BNO08x initialization failure."""
        self.is_initialized = False

    def server(self, cmd):
        pass  # RVC mode: no commands supported

    def simulate_sensor_failure(self):
        """Simulate BNO08x crash / power loss (no more events)."""
        self._sensor_failed = True

    def simulate_sensor_recovery(self):
        """Simulate BNO08x coming back after a failure."""
        self._sensor_failed = False

    def op(self):
        # Re-init if previously timed out (matches imu_op -> imu_init)
        if not self.is_initialized and not self._sensor_failed:
            self.is_initialized = True
            self._last_event_time = time.monotonic()

        if not self.is_initialized:
            return

        # When sensor has failed, no events arrive -- check for timeout
        if self._sensor_failed:
            if (time.monotonic() - self._last_event_time) > IMU_EVENT_TIMEOUT_S:
                self.is_initialized = False
            return

        # Normal operation: sensor produces events
        self._last_event_time = time.monotonic()

        # Small random angular drift (mean-reverting toward 0)
        self.az_angle = 0.99 * self.az_angle + np.random.normal(0, 0.001)
        self.el_angle = 0.99 * self.el_angle + np.random.normal(0, 0.001)

        # Convert internal radians to degrees for output.
        # TODO: verify on hardware which BNO08x euler angle (yaw/pitch/roll)
        # corresponds to each mechanical axis (az/el).  The mapping below
        # assumes yaw=az and pitch=el, but that depends on how the sensor
        # is physically mounted on the antenna box.
        self.yaw = float(np.degrees(self.az_angle))
        self.pitch = float(np.degrees(self.el_angle))
        self.roll = float(np.random.normal(0, NOISE_STDDEV))

        # Acceleration: gravity rotated by pitch/roll + noise
        cp = np.cos(self.el_angle)
        sp = np.sin(self.el_angle)
        cr = np.cos(np.radians(self.roll))
        sr = np.sin(np.radians(self.roll))
        self.accel_x = float(9.81 * sp + np.random.normal(0, NOISE_STDDEV))
        self.accel_y = float(-9.81 * cp * sr + np.random.normal(0, NOISE_STDDEV))
        self.accel_z = float(9.81 * cp * cr + np.random.normal(0, NOISE_STDDEV))

    def get_status(self):
        status = "update" if self.is_initialized else "error"
        return {
            "sensor_name": self.name,
            "status": status,
            "app_id": self.app_id,
            "yaw": self.yaw,
            "pitch": self.pitch,
            "roll": self.roll,
            "accel_x": self.accel_x,
            "accel_y": self.accel_y,
            "accel_z": self.accel_z,
        }
