import numpy as np

from .base import PicoEmulator

NOISE_STDDEV = 0.005  # volts, ~1 LSB of 12-bit ADC at 3.3V ref
VREF = 3.3


class PotMonEmulator(PicoEmulator):
    """Emulates src/potmon.c firmware."""

    # Mirrors POTMON_SP1_TERM_SHORT / POTMON_SP1_TERM_OPEN in
    # src/potmon.h: the level driven on the SP1 failsafe termination
    # GPIO. LOW/0 = SHORT is the failsafe (unpowered/reboot state).
    SP1_TERM_SHORT = 0
    SP1_TERM_OPEN = 1

    def __init__(self, app_id=2, **kwargs):
        self._base_voltage_az = 1.5
        self.voltage_az = self._base_voltage_az
        self.sp1_term = self.SP1_TERM_SHORT
        super().__init__(app_id=app_id, **kwargs)

    def init(self):
        self.voltage_az = self._base_voltage_az
        # Mirrors potmon_init: gpio_put(POTMON_GPIO_SP1_TERM, SHORT).
        self.sp1_term = self.SP1_TERM_SHORT

    def server(self, cmd):
        # Mirrors potmon_server: accept {"sp1_term": 0|1} only.
        # cJSON_IsNumber matches only real JSON numbers; bools parse as
        # cJSON_True/cJSON_False and must be rejected here too (same
        # guard as RFSwitchEmulator.server).
        if "sp1_term" not in cmd:
            return
        raw = cmd["sp1_term"]
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            return
        if raw == 0 or raw == 1:
            self.sp1_term = int(raw)

    def op(self):
        self.voltage_az = float(
            np.clip(
                self._base_voltage_az + np.random.normal(0, NOISE_STDDEV),
                0.0,
                VREF,
            )
        )

    def get_status(self):
        return {
            "sensor_name": "potmon",
            "app_id": self.app_id,
            "status": "update",
            "pot_az_voltage": self.voltage_az,
            "sp1_term": self.sp1_term,
        }
