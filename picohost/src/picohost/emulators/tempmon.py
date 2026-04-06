import time

import numpy as np

from .base import PicoEmulator

NOISE_STDDEV = 0.05  # degrees C


class TempMonEmulator(PicoEmulator):
    """Emulates src/tempmon.c firmware."""

    def __init__(self, app_id=2, **kwargs):
        self._base_temp_lna = 25.0
        self._base_temp_load = 25.0
        self.temp_lna = self._base_temp_lna
        self.temp_load = self._base_temp_load
        self.timestamp_lna = 0.0
        self.timestamp_load = 0.0
        self._sensor_error_lna = False
        self._sensor_error_load = False
        super().__init__(app_id=app_id, **kwargs)

    def init(self):
        self.temp_lna = self._base_temp_lna
        self.temp_load = self._base_temp_load
        self.timestamp_lna = 0.0
        self.timestamp_load = 0.0
        self._sensor_error_lna = False
        self._sensor_error_load = False

    def inject_sensor_error(self, channel, error=True):
        """Simulate a OneWire sensor failure on channel "LNA" or "LOAD".

        In the real firmware ``temp_sensor_has_error()`` returns true when the
        DS18B20 read fails, causing the status field to report ``"error"``.
        """
        if channel == "LNA":
            self._sensor_error_lna = error
        else:
            self._sensor_error_load = error

    def server(self, cmd):
        pass  # tempmon does not handle commands

    def op(self):
        self.temp_lna = self._base_temp_lna + np.random.normal(0, NOISE_STDDEV)
        self.temp_load = self._base_temp_load + np.random.normal(0, NOISE_STDDEV)
        self.timestamp_lna = time.time()
        self.timestamp_load = time.time()

    def get_status(self):
        lna_status = "error" if self._sensor_error_lna else "update"
        load_status = "error" if self._sensor_error_load else "update"
        return {
            "sensor_name": "temp_mon",
            "app_id": self.app_id,
            "LNA_status": lna_status,
            "LNA_temp": self.temp_lna,
            "LNA_timestamp": self.timestamp_lna,
            "LOAD_status": load_status,
            "LOAD_temp": self.temp_load,
            "LOAD_timestamp": self.timestamp_load,
        }
