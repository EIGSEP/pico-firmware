import math

import numpy as np

from .base import PicoEmulator

# Mirrors SIMPLE_THERM_* in src/simple_therm.h (and THERMISTOR_SUPPLY_VOLTS
# in temp_simple.h): Vishay NTCLE100E3103HB0 10k NTC behind a 10.01k
# pull-up to 3V3, datasheet extended Steinhart-Hart fit.
NUM_CHANNELS = 3
THERMISTOR_SUPPLY_VOLTS = 3.3
SIMPLE_THERM_PULLUP_OHMS = 10010.0
SIMPLE_THERM_REF_OHMS = 10000.0
SIMPLE_THERM_SH_A1 = 3.354016e-3
SIMPLE_THERM_SH_B1 = 2.569850e-4
SIMPLE_THERM_SH_C1 = 2.620131e-6
SIMPLE_THERM_SH_D1 = 6.383091e-8

AMBIENT_C = 22.0
DIE_AMBIENT_C = 27.0
NOISE_STDDEV_C = 0.02


def _thermistor_temp_c(resistance):
    """Ohms -> deg C, mirroring voltage_to_temperature() in simple_therm.c."""
    x = math.log(resistance / SIMPLE_THERM_REF_OHMS)
    inv_t = (
        SIMPLE_THERM_SH_A1
        + SIMPLE_THERM_SH_B1 * x
        + SIMPLE_THERM_SH_C1 * x * x
        + SIMPLE_THERM_SH_D1 * x * x * x
    )
    return 1.0 / inv_t - 273.15


def _thermistor_resistance(temp_c):
    """Deg C -> ohms: Newton's method on x = ln(R/Rref) for the cubic
    1/T = A1 + B1 x + C1 x^2 + D1 x^3 (monotonic over the fit range)."""
    target = 1.0 / (temp_c + 273.15)
    x = 0.0
    for _ in range(50):
        f = (
            SIMPLE_THERM_SH_A1
            + SIMPLE_THERM_SH_B1 * x
            + SIMPLE_THERM_SH_C1 * x * x
            + SIMPLE_THERM_SH_D1 * x * x * x
            - target
        )
        df = (
            SIMPLE_THERM_SH_B1
            + 2.0 * SIMPLE_THERM_SH_C1 * x
            + 3.0 * SIMPLE_THERM_SH_D1 * x * x
        )
        step = f / df
        x -= step
        if abs(step) < 1e-12:
            break
    return SIMPLE_THERM_REF_OHMS * math.exp(x)


def _thermistor_voltage(resistance):
    return (
        THERMISTOR_SUPPLY_VOLTS
        * resistance
        / (SIMPLE_THERM_PULLUP_OHMS + resistance)
    )


class SimpleThermEmulator(PicoEmulator):
    """Emulates src/simple_therm.c firmware: status-only thermistor logger.

    Set ``base_temps_c[i]`` to ``None`` to model an open/shorted divider,
    which the firmware reports as null temperature and resistance.
    """

    def __init__(self, app_id=7, **kwargs):
        self.base_temps_c = [AMBIENT_C] * NUM_CHANNELS
        self.temps_c = [None] * NUM_CHANNELS
        self.resistances = [None] * NUM_CHANNELS
        self.voltages = [0.0] * NUM_CHANNELS
        self.die_temp_c = DIE_AMBIENT_C
        super().__init__(app_id=app_id, **kwargs)

    def init(self):
        self.op()

    def server(self, cmd):
        # Mirrors simple_therm_server: no commands.
        pass

    def op(self):
        for i, base in enumerate(self.base_temps_c):
            if base is None:
                # Open divider: pin pulled to the rail, invalid reading.
                self.voltages[i] = THERMISTOR_SUPPLY_VOLTS
                self.temps_c[i] = None
                self.resistances[i] = None
                continue
            t = base + float(np.random.normal(0, NOISE_STDDEV_C))
            r = _thermistor_resistance(t)
            self.temps_c[i] = t
            self.resistances[i] = r
            self.voltages[i] = _thermistor_voltage(r)
        self.die_temp_c = DIE_AMBIENT_C + float(
            np.random.normal(0, NOISE_STDDEV_C)
        )

    def get_status(self):
        status = {
            "sensor_name": "simple_therm",
            "app_id": self.app_id,
            "status": "update",
        }
        for i in range(NUM_CHANNELS):
            status[f"therm{i}_T"] = self.temps_c[i]
            status[f"therm{i}_voltage"] = self.voltages[i]
            status[f"therm{i}_resistance"] = self.resistances[i]
        status["die_T"] = self.die_temp_c
        return status
