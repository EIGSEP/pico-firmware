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
        self.A = TempControlState()
        self.B = TempControlState()
        self.watchdog_timeout_ms = 30000
        self.watchdog_tripped = False
        self._last_cmd_time = time.time()
        super().__init__(app_id=app_id, **kwargs)

    def init(self):
        self.A = TempControlState()
        self.B = TempControlState()
        self.watchdog_timeout_ms = 30000
        self.watchdog_tripped = False
        self._last_cmd_time = time.time()

    def server(self, cmd):
        # Any valid command resets the watchdog timer and clears the trip flag.
        # The host must explicitly re-enable peltiers after a trip.
        self._last_cmd_time = time.time()
        self.watchdog_tripped = False

        for prefix, tc in [("A", self.A), ("B", self.B)]:
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
        """Simulate a OneWire sensor failure on channel "A" or "B".

        In the real firmware ``temp_sensor_has_error()`` returns true when the
        DS18B20 read fails, which sets ``internally_disabled`` and causes the
        status field to report ``"error"`` instead of ``"update"``.
        """
        tc = self.A if channel == "A" else self.B
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
                self.A.enabled = False
                self.B.enabled = False
                self.watchdog_tripped = True

        self._update_channel(self.A)
        self._update_channel(self.B)

    def get_status(self):
        a_status = "error" if self.A.internally_disabled else "update"
        b_status = "error" if self.B.internally_disabled else "update"
        return {
            "sensor_name": "tempctrl",
            "app_id": self.app_id,
            "watchdog_tripped": self.watchdog_tripped,
            "watchdog_timeout_ms": self.watchdog_timeout_ms,
            "A_status": a_status,
            "A_T_now": self.A.T_now,
            "A_timestamp": self.A.timestamp,
            "A_T_target": self.A.T_target,
            "A_drive_level": self.A.drive,
            "A_enabled": self.A.enabled,
            "A_active": self.A.active,
            "A_int_disabled": self.A.internally_disabled,
            "A_hysteresis": self.A.hysteresis,
            "A_clamp": self.A.clamp,
            "B_status": b_status,
            "B_T_now": self.B.T_now,
            "B_timestamp": self.B.timestamp,
            "B_T_target": self.B.T_target,
            "B_drive_level": self.B.drive,
            "B_enabled": self.B.enabled,
            "B_active": self.B.active,
            "B_int_disabled": self.B.internally_disabled,
            "B_hysteresis": self.B.hysteresis,
            "B_clamp": self.B.clamp,
        }
