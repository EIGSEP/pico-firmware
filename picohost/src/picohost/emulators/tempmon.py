import time

import numpy as np

from .base import PicoEmulator

NOISE_STDDEV = 0.05  # degrees C


class TempMonEmulator(PicoEmulator):
    """Emulates src/tempmon.c firmware."""

    def __init__(self, app_id=2, **kwargs):
        self._base_temp_a = 25.0
        self._base_temp_b = 25.0
        self.temp_a = self._base_temp_a
        self.temp_b = self._base_temp_b
        self.timestamp_a = 0.0
        self.timestamp_b = 0.0
        self._sensor_error_a = False
        self._sensor_error_b = False
        super().__init__(app_id=app_id, **kwargs)

    def init(self):
        self.temp_a = self._base_temp_a
        self.temp_b = self._base_temp_b
        self.timestamp_a = 0.0
        self.timestamp_b = 0.0

    def inject_sensor_error(self, channel, error=True):
        """Simulate a OneWire sensor failure on channel "A" or "B".

        In the real firmware ``temp_sensor_has_error()`` returns true when the
        DS18B20 read fails, causing the status field to report ``"error"``.
        """
        if channel == "A":
            self._sensor_error_a = error
        else:
            self._sensor_error_b = error

    def server(self, cmd):
        pass  # tempmon does not handle commands

    def op(self):
        self.temp_a = self._base_temp_a + np.random.normal(0, NOISE_STDDEV)
        self.temp_b = self._base_temp_b + np.random.normal(0, NOISE_STDDEV)
        self.timestamp_a = time.time()
        self.timestamp_b = time.time()

    def get_status(self):
        a_status = "error" if self._sensor_error_a else "update"
        b_status = "error" if self._sensor_error_b else "update"
        return {
            "sensor_name": "temp_mon",
            "app_id": self.app_id,
            "A_status": a_status,
            "A_temp": self.temp_a,
            "A_timestamp": self.timestamp_a,
            "B_status": b_status,
            "B_temp": self.temp_b,
            "B_timestamp": self.timestamp_b,
        }
