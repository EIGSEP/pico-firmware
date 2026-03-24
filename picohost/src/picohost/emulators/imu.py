import time

import numpy as np

from .base import PicoEmulator

NOISE_STDDEV = 0.001
IMU_EVENT_TIMEOUT_S = 5.0  # matches IMU_EVENT_TIMEOUT_MS in imu.h
# Earth's magnetic field (approximate, in sensor frame when upright)
MAG_FIELD = np.array([20.0, 0.0, -45.0])  # microtesla, northern hemisphere
GRAVITY = np.array([0.0, 0.0, 9.81])


def quat_from_az_el(az, el):
    """Quaternion from azimuth (z-axis) then elevation (x-axis) rotation.

    Returns [i, j, k, real] to match BNO08x convention.
    """
    # q_az = rotation around z by az
    caz, saz = np.cos(az / 2), np.sin(az / 2)
    # q_el = rotation around x by el
    cel, sel = np.cos(el / 2), np.sin(el / 2)
    # q = q_az * q_el (Hamilton product)
    q = np.array([
        caz * sel,             # i
        saz * sel,             # j
        saz * cel,             # k
        caz * cel,             # real
    ])
    return q / np.linalg.norm(q)


def rotate_by_quat(v, q):
    """Rotate vector v by quaternion q = [i, j, k, real]."""
    u = q[:3]
    s = q[3]
    return v + 2 * s * np.cross(u, v) + 2 * np.cross(u, np.cross(u, v))


class ImuEmulator(PicoEmulator):
    """Emulates src/imu.cpp firmware.

    Models orientation as rotations around two axes (azimuth and
    elevation), with sensor readings derived from the orientation.
    This allows future coupling with the motor emulator where motor
    steps drive the IMU angles.
    """

    def __init__(self, app_id=3, **kwargs):
        # Orientation state (radians)
        self.az_angle = 0.0
        self.el_angle = 0.0
        self._prev_az = 0.0
        self._prev_el = 0.0
        # Sensor data
        self.q = np.array([0.0, 0.0, 0.0, 1.0])
        self.a = np.array([0.0, 0.0, 9.81])
        self.la = np.zeros(3)
        self.g = np.zeros(3)
        self.m = MAG_FIELD.copy()
        self.grav = np.array([0.0, 0.0, 9.81])
        self.accel_status = 3
        self.mag_status = 3
        self.do_calibration = False
        self.is_initialized = True
        self._last_event_time = time.monotonic()
        self._sensor_failed = False  # set via simulate_sensor_failure()
        # Name depends on app_id: "imu_panda" if APP_IMU (3), else "imu_antenna"
        self.name = "imu_panda" if app_id == 3 else "imu_antenna"
        super().__init__(app_id=app_id, **kwargs)

    def init(self):
        pass  # state set in __init__

    def inject_init_failure(self):
        """Simulate a BNO08x initialization failure.

        In the real firmware, if ``imu.begin()`` fails then
        ``is_initialized`` stays false and all status reports show
        ``"status": "error"``.
        """
        self.is_initialized = False

    def server(self, cmd):
        # Firmware checks for {"calibrate": true} via cJSON_IsTrue(),
        # which only accepts literal JSON true — not 1, "yes", etc.
        if cmd.get("calibrate") is True:
            self.do_calibration = True

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

        # When sensor has failed, no events arrive — check for timeout
        if self._sensor_failed:
            if (time.monotonic() - self._last_event_time) > IMU_EVENT_TIMEOUT_S:
                self.is_initialized = False
            return

        # Normal operation: sensor produces events
        self._last_event_time = time.monotonic()

        # Small random angular drift (mean-reverting toward 0)
        self.az_angle = 0.99 * self.az_angle + np.random.normal(0, 0.001)
        self.el_angle = 0.99 * self.el_angle + np.random.normal(0, 0.001)

        # Quaternion from orientation
        self.q = quat_from_az_el(self.az_angle, self.el_angle)

        # Inverse rotation (conjugate) to transform world vectors to sensor frame
        q_inv = np.array([-self.q[0], -self.q[1], -self.q[2], self.q[3]])

        # Gravity in sensor frame
        self.grav = rotate_by_quat(GRAVITY, q_inv)
        # Accelerometer = gravity (stationary, no linear acceleration)
        self.a = self.grav + np.random.normal(0, NOISE_STDDEV, 3)
        # Linear acceleration (near zero for stationary sensor)
        self.la = np.random.normal(0, NOISE_STDDEV, 3)

        # Gyroscope: angular velocity from angle changes
        d_az = self.az_angle - self._prev_az
        d_el = self.el_angle - self._prev_el
        self.g = np.array([d_el, 0.0, d_az]) + np.random.normal(0, NOISE_STDDEV, 3)
        self._prev_az = self.az_angle
        self._prev_el = self.el_angle

        # Magnetometer: earth's field in sensor frame
        self.m = rotate_by_quat(MAG_FIELD, q_inv) + np.random.normal(0, NOISE_STDDEV, 3)

        # Calibration logic: save when both statuses == 3 and do_calibration
        if (self.do_calibration and self.accel_status == 3
                and self.mag_status == 3):
            self.do_calibration = False

    def get_status(self):
        calibrated = bool(self.do_calibration)
        status = "update" if self.is_initialized else "error"
        return {
            "sensor_name": self.name,
            "status": status,
            "app_id": self.app_id,
            "quat_i": float(self.q[0]),
            "quat_j": float(self.q[1]),
            "quat_k": float(self.q[2]),
            "quat_real": float(self.q[3]),
            "accel_x": float(self.a[0]),
            "accel_y": float(self.a[1]),
            "accel_z": float(self.a[2]),
            "lin_accel_x": float(self.la[0]),
            "lin_accel_y": float(self.la[1]),
            "lin_accel_z": float(self.la[2]),
            "gyro_x": float(self.g[0]),
            "gyro_y": float(self.g[1]),
            "gyro_z": float(self.g[2]),
            "mag_x": float(self.m[0]),
            "mag_y": float(self.m[1]),
            "mag_z": float(self.m[2]),
            "calibrated": calibrated,
            "accel_cal": self.accel_status,
            "mag_cal": self.mag_status,
        }
