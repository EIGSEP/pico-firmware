import random
import time

from .base import PicoEmulator

NOISE_STDDEV = 0.05  # degrees C


class TempMonEmulator(PicoEmulator):
    """Emulates src/tempmon.c firmware."""

    def __init__(self, app_id=2, **kwargs):
        self.temp_a = 25.0
        self.temp_b = 25.0
        self.timestamp_a = 0.0
        self.timestamp_b = 0.0
        super().__init__(app_id=app_id, **kwargs)

    def init(self):
        self.temp_a = 25.0
        self.temp_b = 25.0
        self.timestamp_a = 0.0
        self.timestamp_b = 0.0

    def server(self, cmd):
        pass  # tempmon does not handle commands

    def op(self):
        self.temp_a += random.gauss(0, NOISE_STDDEV)
        self.temp_b += random.gauss(0, NOISE_STDDEV)
        self.timestamp_a = time.time()
        self.timestamp_b = time.time()

    def get_status(self):
        return {
            "sensor_name": "temp_mon",
            "app_id": self.app_id,
            "A_status": "update",
            "A_temp": self.temp_a,
            "A_timestamp": self.timestamp_a,
            "B_status": "update",
            "B_temp": self.temp_b,
            "B_timestamp": self.timestamp_b,
        }
