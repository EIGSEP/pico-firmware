import numpy as np

from .base import PicoEmulator

NOISE_STDDEV = 0.005  # volts, ~1 LSB of 12-bit ADC at 3.3V ref
VREF = 3.3


class PotMonEmulator(PicoEmulator):
    """Emulates src/potmon.c firmware."""

    def __init__(self, app_id=2, **kwargs):
        self._base_voltage_0 = 1.5  # midrange default
        self._base_voltage_1 = 1.5
        self.voltage_0 = self._base_voltage_0
        self.voltage_1 = self._base_voltage_1
        super().__init__(app_id=app_id, **kwargs)

    def init(self):
        self.voltage_0 = self._base_voltage_0
        self.voltage_1 = self._base_voltage_1

    def server(self, cmd):
        pass  # potmon does not handle commands

    def op(self):
        self.voltage_0 = float(np.clip(
            self._base_voltage_0 + np.random.normal(0, NOISE_STDDEV),
            0.0, VREF,
        ))
        self.voltage_1 = float(np.clip(
            self._base_voltage_1 + np.random.normal(0, NOISE_STDDEV),
            0.0, VREF,
        ))

    def get_status(self):
        return {
            "sensor_name": "potmon",
            "app_id": self.app_id,
            "status": "update",
            "pot0_voltage": round(self.voltage_0, 4),
            "pot1_voltage": round(self.voltage_1, 4),
        }
