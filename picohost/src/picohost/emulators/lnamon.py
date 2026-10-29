import numpy as np

from .base import PicoEmulator

NOISE_STDDEV = 0.005  # volts, ~1 LSB of 12-bit ADC at 3.3V ref
VREF = 3.3

# Mirrors LNAMON_R1_OHMS / LNAMON_R2_OHMS in src/lnamon.h.
R1_OHMS = 51000.0
R2_OHMS = 75100.0
DIVIDER_GAIN = (R1_OHMS + R2_OHMS) / R2_OHMS


class LnaMonEmulator(PicoEmulator):
    """Emulates src/lnamon.c firmware."""

    def __init__(self, app_id=7, rail_voltage=5.0, **kwargs):
        self._base_rail_voltage = rail_voltage
        self.adc_voltage = 0.0
        self.rail_voltage = 0.0
        super().__init__(app_id=app_id, **kwargs)

    def init(self):
        # Mirrors lnamon_init: readings stay 0.0 until the first op().
        self.adc_voltage = 0.0
        self.rail_voltage = 0.0

    def server(self, cmd):
        # Mirrors lnamon_server: read-only, no commands accepted.
        pass

    def op(self):
        self.adc_voltage = float(
            np.clip(
                self._base_rail_voltage / DIVIDER_GAIN
                + np.random.normal(0, NOISE_STDDEV),
                0.0,
                VREF,
            )
        )
        self.rail_voltage = self.adc_voltage * DIVIDER_GAIN

    def get_status(self):
        return {
            "sensor_name": "lnamon",
            "app_id": self.app_id,
            "status": "update",
            "lna_adc_voltage": self.adc_voltage,
            "lna_rail_voltage": self.rail_voltage,
        }
