import numpy as np

from .base import PicoEmulator

NOISE_STDDEV = 0.005  # volts, ~1 LSB of 12-bit ADC at 3.3V ref
VREF = 3.3


class PotMonEmulator(PicoEmulator):
    """Emulates src/potmon.c firmware."""

    def __init__(self, app_id=2, **kwargs):
        self._base_voltage_el = 1.5  # midrange default
        self._base_voltage_az = 1.5
        self.voltage_el = self._base_voltage_el
        self.voltage_az = self._base_voltage_az
        super().__init__(app_id=app_id, **kwargs)

    def init(self):
        self.voltage_el = self._base_voltage_el
        self.voltage_az = self._base_voltage_az

    def server(self, cmd):
        pass  # potmon does not handle commands

    def op(self):
        self.voltage_el = float(np.clip(
            self._base_voltage_el + np.random.normal(0, NOISE_STDDEV),
            0.0, VREF,
        ))
        self.voltage_az = float(np.clip(
            self._base_voltage_az + np.random.normal(0, NOISE_STDDEV),
            0.0, VREF,
        ))

    def get_status(self):
        return {
            "sensor_name": "potmon",
            "app_id": self.app_id,
            "status": "update",
            "pot_el_voltage": round(self.voltage_el, 4),
            "pot_az_voltage": round(self.voltage_az, 4),
        }
