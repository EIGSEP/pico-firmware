import random

from .base import PicoEmulator

NOISE_STDDEV = 0.01  # meters


class LidarEmulator(PicoEmulator):
    """Emulates src/lidar.c firmware."""

    def __init__(self, app_id=4, **kwargs):
        self._base_distance = 1.5  # meters
        self.distance = self._base_distance
        super().__init__(app_id=app_id, **kwargs)

    def init(self):
        self.distance = self._base_distance

    def server(self, cmd):
        pass  # lidar does not handle commands

    def op(self):
        self.distance = self._base_distance + random.gauss(0, NOISE_STDDEV)

    def get_status(self):
        return {
            "sensor_name": "lidar",
            "status": "update",
            "app_id": self.app_id,
            "distance_m": self.distance,
        }
