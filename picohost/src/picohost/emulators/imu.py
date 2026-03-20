import math
import random

from .base import PicoEmulator

NOISE_STDDEV = 0.001


class ImuEmulator(PicoEmulator):
    """Emulates src/imu.cpp firmware."""

    def __init__(self, app_id=3, **kwargs):
        # Sensor data arrays
        self.q = [0.0, 0.0, 0.0, 1.0]  # quaternion [i, j, k, real]
        self.a = [0.0, 0.0, 9.81]       # accelerometer
        self.la = [0.0, 0.0, 0.0]       # linear acceleration
        self.g = [0.0, 0.0, 0.0]        # gyroscope
        self.m = [0.0, 0.0, 0.0]        # magnetometer
        self.grav = [0.0, 0.0, 9.81]    # gravity
        self.accel_status = 3
        self.mag_status = 3
        self.do_calibration = False
        self.is_initialized = True
        # Name depends on app_id: "imu_panda" if APP_IMU (3), else "imu_antenna"
        self.name = "imu_panda" if app_id == 3 else "imu_antenna"
        super().__init__(app_id=app_id, **kwargs)

    def init(self):
        pass  # state set in __init__

    def server(self, cmd):
        # Firmware checks for {"calibrate": true}
        if cmd.get("calibrate") or cmd.get("cmd") == "calibrate":
            self.do_calibration = True

    def op(self):
        # Add noise to all sensor arrays
        for arr in (self.a, self.la, self.g, self.m, self.grav):
            for i in range(len(arr)):
                arr[i] += random.gauss(0, NOISE_STDDEV)

        # Add noise to quaternion and renormalize
        for i in range(4):
            self.q[i] += random.gauss(0, NOISE_STDDEV * 0.1)
        norm = math.sqrt(sum(x * x for x in self.q))
        if norm > 0:
            self.q = [x / norm for x in self.q]

        # Calibration logic: save when both statuses == 3 and do_calibration
        if (self.do_calibration and self.accel_status == 3
                and self.mag_status == 3):
            self.do_calibration = False

    def get_status(self):
        # "calibrated" field: "True" when do_calibration is set,
        # "False" otherwise (matching C firmware's inverted logic)
        calibrated = "True" if self.do_calibration else "False"
        status = "update" if self.is_initialized else "error"
        return {
            "sensor_name": self.name,
            "status": status,
            "app_id": self.app_id,
            "quat_i": self.q[0],
            "quat_j": self.q[1],
            "quat_k": self.q[2],
            "quat_real": self.q[3],
            "accel_x": self.a[0],
            "accel_y": self.a[1],
            "accel_z": self.a[2],
            "lin_accel_x": self.la[0],
            "lin_accel_y": self.la[1],
            "lin_accel_z": self.la[2],
            "gyro_x": self.g[0],
            "gyro_y": self.g[1],
            "gyro_z": self.g[2],
            "mag_x": self.m[0],
            "mag_y": self.m[1],
            "mag_z": self.m[2],
            "calibrated": calibrated,
            "accel_cal": self.accel_status,
            "mag_cal": self.mag_status,
        }
