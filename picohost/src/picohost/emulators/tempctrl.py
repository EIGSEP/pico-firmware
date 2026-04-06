import time

from .base import PicoEmulator, _safe_float, _safe_int


class TempControlState:
    """Models the TempControl struct from tempctrl.h."""

    def __init__(self):
        self.T_now = 25.0
        self.T_target = 30.0
        self.drive = 0.0
        self.gain = 0.2
        self.baseline = 0.4
        self.clamp = 0.6
        self.hysteresis = 0.5
        self.enabled = False
        self.active = False
        self.internally_disabled = False
        self.timestamp = 0.0


def tempctrl_hysteresis_drive(tc):
    """Matches tempctrl_hysteresis_drive() from tempctrl.c."""
    T_delta = tc.T_target - tc.T_now
    sign = 1 if T_delta >= 0 else -1

    if abs(T_delta) <= tc.hysteresis:
        tc.drive = 0.0
        tc.active = False
    else:
        tc.active = True
        tc.drive = T_delta * tc.gain + sign * tc.baseline
        if abs(tc.drive) > tc.clamp:
            tc.drive = sign * tc.clamp


THERMAL_DRIFT_RATE = 0.05


class TempCtrlEmulator(PicoEmulator):
    """Emulates src/tempctrl.c firmware."""

    def __init__(self, app_id=1, **kwargs):
        self.lna = TempControlState()
        self.load = TempControlState()
        self.watchdog_timeout_ms = 30000
        self.watchdog_tripped = False
        self._last_cmd_time = time.time()
        super().__init__(app_id=app_id, **kwargs)

    def init(self):
        self.lna = TempControlState()
        self.load = TempControlState()
        self.watchdog_timeout_ms = 30000
        self.watchdog_tripped = False
        self._last_cmd_time = time.time()

    def server(self, cmd):
        # Any valid command resets the watchdog timer and clears the trip flag.
        # The host must explicitly re-enable peltiers after a trip.
        self._last_cmd_time = time.time()
        self.watchdog_tripped = False

        for prefix, tc in [("LNA", self.lna), ("LOAD", self.load)]:
            key = f"{prefix}_temp_target"
            if key in cmd:
                tc.T_target = _safe_float(cmd[key], tc.T_target)

            key = f"{prefix}_enable"
            if key in cmd:
                tc.enabled = bool(cmd[key])

            key = f"{prefix}_hysteresis"
            if key in cmd:
                tc.hysteresis = _safe_float(cmd[key], tc.hysteresis)

            key = f"{prefix}_clamp"
            if key in cmd:
                tc.clamp = min(1.0, max(0.0, _safe_float(cmd[key], tc.clamp)))

        if "watchdog_timeout_ms" in cmd:
            self.watchdog_timeout_ms = _safe_int(
                cmd["watchdog_timeout_ms"], self.watchdog_timeout_ms
            )

    def inject_sensor_error(self, channel, error=True):
        """Simulate a OneWire sensor failure on channel "LNA" or "LOAD".

        In the real firmware ``temp_sensor_has_error()`` returns true when the
        DS18B20 read fails, which sets ``internally_disabled`` and causes the
        status field to report ``"error"`` instead of ``"update"``.
        """
        tc = self.lna if channel == "LNA" else self.load
        tc.internally_disabled = error

    def _update_channel(self, tc):
        if tc.enabled and not tc.internally_disabled:
            tempctrl_hysteresis_drive(tc)
            tc.T_now += tc.drive * THERMAL_DRIFT_RATE
        else:
            tc.drive = 0.0
        tc.timestamp = time.time()

    def op(self):
        # Communication watchdog: disable peltiers if no command within timeout
        if self.watchdog_timeout_ms > 0 and not self.watchdog_tripped:
            elapsed_ms = (time.time() - self._last_cmd_time) * 1000
            if elapsed_ms > self.watchdog_timeout_ms:
                self.lna.enabled = False
                self.load.enabled = False
                self.watchdog_tripped = True

        self._update_channel(self.lna)
        self._update_channel(self.load)

    def get_status(self):
        lna_status = "error" if self.lna.internally_disabled else "update"
        load_status = "error" if self.load.internally_disabled else "update"
        return {
            "sensor_name": "tempctrl",
            "app_id": self.app_id,
            "watchdog_tripped": self.watchdog_tripped,
            "watchdog_timeout_ms": self.watchdog_timeout_ms,
            "LNA_status": lna_status,
            "LNA_T_now": self.lna.T_now,
            "LNA_timestamp": self.lna.timestamp,
            "LNA_T_target": self.lna.T_target,
            "LNA_drive_level": self.lna.drive,
            "LNA_enabled": self.lna.enabled,
            "LNA_active": self.lna.active,
            "LNA_int_disabled": self.lna.internally_disabled,
            "LNA_hysteresis": self.lna.hysteresis,
            "LNA_clamp": self.lna.clamp,
            "LOAD_status": load_status,
            "LOAD_T_now": self.load.T_now,
            "LOAD_timestamp": self.load.timestamp,
            "LOAD_T_target": self.load.T_target,
            "LOAD_drive_level": self.load.drive,
            "LOAD_enabled": self.load.enabled,
            "LOAD_active": self.load.active,
            "LOAD_int_disabled": self.load.internally_disabled,
            "LOAD_hysteresis": self.load.hysteresis,
            "LOAD_clamp": self.load.clamp,
        }
