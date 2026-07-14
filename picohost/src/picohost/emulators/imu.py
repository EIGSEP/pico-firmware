import time

import numpy as np

from .. import imu_geometry as ig
from .base import PicoEmulator

NOISE_STDDEV = 0.001
IMU_EVENT_TIMEOUT_S = 5.0  # matches IMU_EVENT_TIMEOUT_MS in imu.h


class ImuEmulator(PicoEmulator):
    """Emulates src/imu.c firmware (UART RVC mode).

    Models orientation as rotations around two axes (azimuth and
    elevation), with sensor readings derived from the orientation.
    The BNO08x in RVC mode outputs euler angles and acceleration;
    this emulator produces matching values via the same forward model
    that imu_geometry inverts.
    """

    def __init__(self, app_id=3, **kwargs):
        # Forward-model state (set before super().__init__() in case
        # super triggers init paths that reference these).
        self._mount = np.eye(3)
        self._accel_bias = np.zeros(3)
        self._accel_scale = 1.0
        self._hold = False  # True once set_orientation is called

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
        # RFI standby: hold the BNO08x in reset (RST low) and go quiet.
        self._standby = False
        # Per-cycle freshness flag: True iff a packet was produced since
        # the last get_status() call. Drives the "status" field.
        self.got_packet_this_cycle = False
        # Name depends on app_id: mirrors imu.cpp init_eigsep_imu()
        APP_IMU_EL, APP_IMU_AZ = 3, 6
        if app_id == APP_IMU_EL:
            self.name = "imu_el"
        elif app_id == APP_IMU_AZ:
            self.name = "imu_az"
        else:
            raise ValueError(
                f"ImuEmulator: unexpected app_id={app_id}, expected {APP_IMU_EL} or {APP_IMU_AZ}"
            )
        super().__init__(app_id=app_id, **kwargs)

    def init(self):
        pass  # state set in __init__

    def inject_init_failure(self):
        """Simulate a BNO08x initialization failure."""
        self.is_initialized = False

    def server(self, cmd):
        # RVC mode has no sensor commands, but the universal RFI standby
        # controls apply: standby holds RST low (mirrored here by dropping
        # is_initialized so resume re-runs the full init), resume re-inits.
        action = cmd.get("cmd")
        if action == "standby":
            self._standby = True
            self.is_initialized = False
        elif action == "resume":
            self._standby = False
            self.is_initialized = False  # next op() re-inits, like imu_init()

    def simulate_sensor_failure(self):
        """Simulate BNO08x crash / power loss (no more events)."""
        self._sensor_failed = True

    def simulate_sensor_recovery(self):
        """Simulate BNO08x coming back after a failure."""
        self._sensor_failed = False

    def set_orientation(self, az_deg, el_deg):
        """Hold a fixed (az, el) pose; disables the idle drift model."""
        self.az_angle = np.radians(az_deg)
        self.el_angle = np.radians(el_deg)
        self._hold = True

    def set_mount(self, M):
        self._mount = np.asarray(M, dtype=float)

    def set_accel_error(self, bias=(0.0, 0.0, 0.0), scale=1.0):
        self._accel_bias = np.asarray(bias, dtype=float)
        self._accel_scale = float(scale)

    def _render(self):
        """Compute yaw/pitch/roll/accel from current az/el/mount/error.

        imu_el sees elevation only; imu_az sees elevation then azimuth.
        R = R_x(theta) [@ R_z(phi)] @ M ; accel = R[2,:] * g (+ error).
        """
        theta, phi = self.el_angle, self.az_angle
        if self.name == "imu_az":
            R = ig.R_x(theta) @ ig.R_z(phi) @ self._mount
        else:  # imu_el ignores azimuth
            R = ig.R_x(theta) @ self._mount
        g_body = R[2, :] * ig.GRAVITY  # R^T @ [0,0,g]
        g_body = g_body * self._accel_scale + self._accel_bias
        self.accel_x, self.accel_y, self.accel_z = (float(v) for v in g_body)
        # ZYX euler of R for yaw/pitch/roll (diagnostic channels)
        self.pitch = float(np.degrees(-np.arcsin(np.clip(R[2, 0], -1, 1))))
        self.roll = float(np.degrees(np.arctan2(R[2, 1], R[2, 2])))
        self.yaw = float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))

    def op(self):
        if self._standby:
            return  # RST held low: no re-init, no UART drain, no packet
        # Mirrors imu_op -> imu_init, including the memset(&imu.data, 0)
        # at imu.c:109 that zeros sensor data on every (re-)init.
        if not self.is_initialized:
            self.is_initialized = True
            self._last_event_time = time.monotonic()
            self.az_angle = 0.0
            self.el_angle = 0.0
            self.yaw = 0.0
            self.pitch = 0.0
            self.roll = 0.0
            self.accel_x = 0.0
            self.accel_y = 0.0
            self.accel_z = 0.0

        if self._sensor_failed:
            if (
                time.monotonic() - self._last_event_time
            ) > IMU_EVENT_TIMEOUT_S:
                self.is_initialized = False
            return

        # Normal operation: sensor produces events
        self._last_event_time = time.monotonic()
        if not self._hold:
            # idle: gentle mean-reverting drift (matches prior behaviour)
            self.az_angle = 0.99 * self.az_angle + np.random.normal(0, 0.001)
            self.el_angle = 0.99 * self.el_angle + np.random.normal(0, 0.001)
        self._render()
        self.got_packet_this_cycle = True

    def get_status(self):
        if self._standby:
            # Commanded-off looks like an error tick (no valid data) but with
            # standby=true so the host distinguishes it from a real fault.
            return {
                "sensor_name": self.name,
                "status": "error",
                "app_id": self.app_id,
                "standby": True,
            }
        status = "update" if self.got_packet_this_cycle else "error"
        self.got_packet_this_cycle = False
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
