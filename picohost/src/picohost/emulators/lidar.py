import numpy as np

from .base import PicoEmulator

NOISE_STDDEV = 0.01  # meters


class LidarEmulator(PicoEmulator):
    """Emulates src/lidar.c firmware."""

    def __init__(self, app_id=4, **kwargs):
        self._base_distance = 100.0  # meters
        self.distance = self._base_distance
        self._sensor_failed = False  # set via simulate_sensor_failure()
        # Per-cycle freshness flag: True iff op() refreshed the
        # distance reading since the last get_status() call.
        self.last_op_ok = False
        super().__init__(app_id=app_id, **kwargs)

    def init(self):
        self.distance = self._base_distance

    def server(self, cmd):
        pass  # lidar does not handle commands

    def simulate_sensor_failure(self):
        """Simulate i2c read failure / TF-Luna not-ready loop."""
        self._sensor_failed = True

    def simulate_sensor_recovery(self):
        """Simulate sensor coming back after a failure."""
        self._sensor_failed = False

    def op(self):
        if self._sensor_failed:
            # Matches firmware: i2c_read_timeout or dist_cm==0 → reset+return,
            # leaving previous distance unchanged and last_op_ok at False.
            return
        self.distance = self._base_distance + np.random.normal(0, NOISE_STDDEV)
        self.last_op_ok = True

    def get_status(self):
        status = "update" if self.last_op_ok else "error"
        self.last_op_ok = False
        return {
            "sensor_name": "lidar",
            "status": status,
            "app_id": self.app_id,
            "distance_m": self.distance,
        }
