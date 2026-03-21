import time

from .base import PicoEmulator


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
        super().__init__(app_id=app_id, **kwargs)

    def init(self):
        self.A = TempControlState()
        self.B = TempControlState()

    def server(self, cmd):
        for prefix, tc in [("A", self.A), ("B", self.B)]:
            key = f"{prefix}_temp_target"
            if key in cmd:
                tc.T_target = float(cmd[key])

            key = f"{prefix}_enable"
            if key in cmd:
                tc.enabled = bool(cmd[key])

            key = f"{prefix}_hysteresis"
            if key in cmd:
                tc.hysteresis = float(cmd[key])

            key = f"{prefix}_clamp"
            if key in cmd:
                tc.clamp = min(1.0, max(0.0, float(cmd[key])))

    def _update_channel(self, tc):
        if tc.enabled and not tc.internally_disabled:
            tempctrl_hysteresis_drive(tc)
            tc.T_now += tc.drive * THERMAL_DRIFT_RATE
        else:
            tc.drive = 0.0
        tc.timestamp = time.time()

    def op(self):
        self._update_channel(self.A)
        self._update_channel(self.B)

    def get_status(self):
        return {
            "sensor_name": "tempctrl",
            "app_id": self.app_id,
            "A_status": "update",
            "A_T_now": self.A.T_now,
            "A_timestamp": self.A.timestamp,
            "A_T_target": self.A.T_target,
            "A_drive_level": self.A.drive,
            "A_enabled": self.A.enabled,
            "A_active": self.A.active,
            "A_int_disabled": self.A.internally_disabled,
            "A_hysteresis": self.A.hysteresis,
            "A_clamp": self.A.clamp,
            "B_status": "update",
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
