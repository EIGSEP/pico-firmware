import numpy as np

from .base import PicoEmulator

NOISE_STDDEV = 0.005  # volts, ~1 LSB of 12-bit ADC at 3.3V ref
VREF = 3.3

# Mirrors LNAMON_R1_OHMS / LNAMON_R2_OHMS in src/lnamon.h.
R1_OHMS = 51000.0
R2_OHMS = 75100.0
DIVIDER_GAIN = (R1_OHMS + R2_OHMS) / R2_OHMS
# Mirrors LNAMON_CAL_GAIN / LNAMON_CAL_OFFSET in src/lnamon.h.
CAL_GAIN = 1.01164
CAL_OFFSET = -0.03222


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
        # Invert the cal so the emulated ADC reads what uncalibrated
        # hardware would for the true rail; the firmware's correction then
        # recovers that rail.
        uncal_rail = (self._base_rail_voltage - CAL_OFFSET) / CAL_GAIN
        self.adc_voltage = float(
            np.clip(
                uncal_rail / DIVIDER_GAIN + np.random.normal(0, NOISE_STDDEV),
                0.0,
                VREF,
            )
        )
        self.rail_voltage = (
            CAL_GAIN * (self.adc_voltage * DIVIDER_GAIN) + CAL_OFFSET
        )

    def get_status(self):
        return {
            "sensor_name": "lnamon",
            "app_id": self.app_id,
            "status": "update",
            "lna_adc_voltage": self.adc_voltage,
            "lna_rail_voltage": self.rail_voltage,
        }
