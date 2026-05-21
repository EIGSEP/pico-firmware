import time

from .base import PicoEmulator, _safe_float, _safe_int


# Seconds of simulated wall-clock time per emulator op tick. Chosen to
# match the firmware's effective PI sample period (DS18B20 conversion
# completes about every 750 ms), so a unit Ki value produces the same
# integral growth here as on real hardware.
DT_PER_OP_S = 0.75


class TempControlState:
    """Models the TempControl struct from tempctrl.h."""

    def __init__(self):
        self.T_now = 25.0
        self.T_target = 30.0
        self.drive = 0.0
        self.Kp = 0.2
        self.Ki = 0.0
        self.integral = 0.0
        self.last_sample_seen = False
        self.clamp = 0.6
        self.hysteresis = 0.5
        self.enabled = False
        self.active = False
        self.internally_disabled = False
        self.timestamp = 0.0
        # Stall guard mirror (see tempctrl_check_stall in tempctrl.c).
        self.stall_tripped = False
        self.stall_window_active = False
        self.stall_check_T = 0.0
        self.stall_check_time = 0.0
        # Test hook: when True, skip the thermal-drift update even with the
        # drive engaged, so the stall guard can be exercised deterministically.
        self.thermal_frozen = False


def _reset_controller_state(tc):
    """Match tempctrl_reset_controller_state() in tempctrl.c."""
    tc.drive = 0.0
    tc.integral = 0.0
    tc.last_sample_seen = False
    tc.active = False


def tempctrl_pi_drive(tc, dt=DT_PER_OP_S):
    """Matches tempctrl_pi_drive() from tempctrl.c.

    Deadband + PI with conditional-integration anti-windup. First sample
    after reset uses dt=0 to avoid an initial integrator jump.
    """
    T_delta = tc.T_target - tc.T_now

    if abs(T_delta) <= tc.hysteresis:
        _reset_controller_state(tc)
        return

    tc.active = True

    effective_dt = dt if tc.last_sample_seen else 0.0
    tc.last_sample_seen = True

    p_term = tc.Kp * T_delta
    # Pure-P (Ki==0): freeze the integrator. Matches firmware
    # tempctrl_pi_drive — bumpless retune is enforced on Ki transitions
    # in server(), not here.
    if tc.Ki == 0.0:
        tentative_i = tc.integral
    else:
        tentative_i = tc.integral + T_delta * effective_dt
    tentative_drive = p_term + tc.Ki * tentative_i

    sat_high = tentative_drive > tc.clamp and T_delta > 0
    sat_low = tentative_drive < -tc.clamp and T_delta < 0

    if sat_high:
        tc.drive = tc.clamp
    elif sat_low:
        tc.drive = -tc.clamp
    else:
        tc.integral = tentative_i
        tc.drive = max(-tc.clamp, min(tc.clamp, tentative_drive))


THERMAL_DRIFT_RATE = 0.05

# Mirrors TEMPCTRL_STALL_WINDOW_MS / TEMPCTRL_STALL_MIN_DELTA in tempctrl.h.
STALL_WINDOW_MS = 60000
STALL_MIN_DELTA = 0.5


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
        # Any valid command refreshes the watchdog timer, but the trip flag
        # is sticky: the host clears it by explicitly sending *_enable=true
        # (see firmware tempctrl_apply_enable).
        self._last_cmd_time = time.time()

        for prefix, tc in [("LNA", self.lna), ("LOAD", self.load)]:
            key = f"{prefix}_temp_target"
            if key in cmd:
                tc.T_target = _safe_float(cmd[key], tc.T_target)

            key = f"{prefix}_enable"
            if key in cmd:
                # cJSON's valueint is 0 for non-numeric types — mirror that
                # so {"LNA_enable": "false"} disables on the emulator just
                # like it does on firmware (instead of being truthy).
                new_enabled = bool(_safe_int(cmd[key], 0))
                # *_enable=true is the host's ack of sticky trips: it clears
                # this channel's stall flag and the app-wide watchdog flag.
                # `enabled` is host intent only; firmware never mutates it.
                if new_enabled:
                    tc.stall_tripped = False
                    tc.stall_window_active = False
                    self.watchdog_tripped = False
                tc.enabled = new_enabled

            key = f"{prefix}_hysteresis"
            if key in cmd:
                tc.hysteresis = _safe_float(cmd[key], tc.hysteresis)

            key = f"{prefix}_clamp"
            if key in cmd:
                tc.clamp = min(1.0, max(0.0, _safe_float(cmd[key], tc.clamp)))

            key = f"{prefix}_Kp"
            if key in cmd:
                tc.Kp = _safe_float(cmd[key], tc.Kp)

            key = f"{prefix}_Ki"
            if key in cmd:
                new_ki = _safe_float(cmd[key], tc.Ki)
                if new_ki != tc.Ki:
                    # Bumpless retune: drop the accumulator so the next PI
                    # step does not multiply a stale integral by a freshly
                    # changed gain.
                    tc.integral = 0.0
                    tc.last_sample_seen = False
                tc.Ki = new_ki

            key = f"{prefix}_integral_reset"
            if key in cmd and _safe_int(cmd[key], 0):
                tc.integral = 0.0
                tc.last_sample_seen = False

        if "watchdog_timeout_ms" in cmd:
            val = _safe_int(cmd["watchdog_timeout_ms"], self.watchdog_timeout_ms)
            # Firmware clamps negatives to 0 (see tempctrl.c watchdog parse).
            self.watchdog_timeout_ms = 0 if val < 0 else val

    def inject_sensor_error(self, channel, error=True):
        """Simulate a OneWire sensor failure on channel "LNA" or "LOAD".

        In the real firmware ``temp_sensor_has_error()`` returns true when the
        DS18B20 read fails, which sets ``internally_disabled`` and causes the
        status field to report ``"error"`` instead of ``"update"``.
        """
        tc = self.lna if channel == "LNA" else self.load
        tc.internally_disabled = error

    def _drive_allowed(self, tc):
        return (
            tc.enabled
            and not tc.internally_disabled
            and not tc.stall_tripped
            and not self.watchdog_tripped
        )

    def _update_channel(self, tc):
        if self._drive_allowed(tc):
            tempctrl_pi_drive(tc)
            if not tc.thermal_frozen:
                tc.T_now += tc.drive * THERMAL_DRIFT_RATE
            self._check_stall(tc)
        else:
            _reset_controller_state(tc)
            tc.stall_window_active = False
        tc.timestamp = time.time()

    def _check_stall(self, tc):
        """Mirror tempctrl_check_stall() from tempctrl.c."""
        if not tc.active:
            tc.stall_window_active = False
            return
        now = time.time()
        if not tc.stall_window_active:
            tc.stall_check_T = tc.T_now
            tc.stall_check_time = now
            tc.stall_window_active = True
            return
        elapsed_ms = (now - tc.stall_check_time) * 1000
        if elapsed_ms < STALL_WINDOW_MS:
            return
        if abs(tc.T_now - tc.stall_check_T) < STALL_MIN_DELTA:
            tc.stall_tripped = True
            tc.active = False
            tc.drive = 0.0
            tc.stall_window_active = False
        else:
            tc.stall_check_T = tc.T_now
            tc.stall_check_time = now

    def op(self):
        # Communication watchdog: trip the app-wide flag if no command has
        # arrived within the timeout. `enabled` is host intent and stays
        # untouched; the trip flag is the runtime gate.
        if self.watchdog_timeout_ms > 0 and not self.watchdog_tripped:
            elapsed_ms = (time.time() - self._last_cmd_time) * 1000
            if elapsed_ms > self.watchdog_timeout_ms:
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
            "LNA_stall_tripped": self.lna.stall_tripped,
            "LNA_hysteresis": self.lna.hysteresis,
            "LNA_clamp": self.lna.clamp,
            "LNA_Kp": self.lna.Kp,
            "LNA_Ki": self.lna.Ki,
            "LNA_integral": self.lna.integral,
            "LOAD_status": load_status,
            "LOAD_T_now": self.load.T_now,
            "LOAD_timestamp": self.load.timestamp,
            "LOAD_T_target": self.load.T_target,
            "LOAD_drive_level": self.load.drive,
            "LOAD_enabled": self.load.enabled,
            "LOAD_active": self.load.active,
            "LOAD_int_disabled": self.load.internally_disabled,
            "LOAD_stall_tripped": self.load.stall_tripped,
            "LOAD_hysteresis": self.load.hysteresis,
            "LOAD_clamp": self.load.clamp,
            "LOAD_Kp": self.load.Kp,
            "LOAD_Ki": self.load.Ki,
            "LOAD_integral": self.load.integral,
        }
