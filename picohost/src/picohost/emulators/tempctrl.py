import math
import time

from .base import PicoEmulator, _safe_int


# Mirror the firmware's fixed sampling cadence: sampling, the rate guard,
# and the PI step run on the TEMPCTRL_SAMPLE_MS = 200 ms timer inside
# tempctrl_op() (between timer ticks op is a no-op and PWM holds the drive).
# One emulator op() call models one sample tick. dt argument to
# tempctrl_pi_drive matches the firmware's (now_ms - last_sample_ms)
# elapsed time between sample ticks.
DT_PER_SAMPLE_S = 0.2
# Thermal effect per sample tick at unit drive. The drive value set by PI
# is held between ticks (mirrors continuous PWM), so the per-tick rate is
# what determines convergence speed.
THERMAL_DRIFT_PER_OP = 0.05

# Thermistor divider constants, mirroring temp_simple.h.
THERMISTOR_SUPPLY_VOLTS = 3.3
THERMISTOR_FIXED_OHMS = 10680.0
THERMISTOR_BOARD_PULLUP_OHMS = 4700.0
THERMISTOR_TOP_OHMS = (
    THERMISTOR_FIXED_OHMS
    * THERMISTOR_BOARD_PULLUP_OHMS
    / (THERMISTOR_FIXED_OHMS + THERMISTOR_BOARD_PULLUP_OHMS)
)
THERMISTOR_SH_A = 9.2463455e-4
THERMISTOR_SH_B = 2.2246310e-4
THERMISTOR_SH_C = 1.2326590e-7


def _thermistor_resistance(temp_c):
    """Invert the firmware's Steinhart-Hart fit: temperature -> ohms."""
    inv_kelvin = 1.0 / (temp_c + 273.15)
    y = (THERMISTOR_SH_A - inv_kelvin) / THERMISTOR_SH_C
    z = math.sqrt(
        (THERMISTOR_SH_B / (3.0 * THERMISTOR_SH_C)) ** 3 + (y / 2.0) ** 2
    )
    log_r = (z - y / 2.0) ** (1.0 / 3.0) - (z + y / 2.0) ** (1.0 / 3.0)
    return math.exp(log_r)


def _thermistor_voltage(resistance):
    return (
        THERMISTOR_SUPPLY_VOLTS
        * resistance
        / (THERMISTOR_TOP_OHMS + resistance)
    )


def _set_thermistor_diagnostics(tc, temp_c):
    tc.resistance = _thermistor_resistance(temp_c)
    tc.voltage = _thermistor_voltage(tc.resistance)


def _coerce_float(val):
    """Mirror cJSON ``valuedouble``: non-numeric JSON yields 0.0.

    Unlike ``_safe_float`` (which preserves a caller-supplied default on
    bad input), the firmware assigns ``item_json->valuedouble`` directly
    — which cJSON sets to 0.0 for strings/null/arrays/objects.
    """
    if isinstance(val, (str, bytes, list, dict)):
        return 0.0
    try:
        return float(val)
    except (ValueError, TypeError):
        return 0.0


class TempControlState:
    """Models the TempControl struct from tempctrl.h."""

    def __init__(self):
        self.T_now = 25.0
        self.T_target = 30.0
        self.voltage = 0.0
        self.resistance = 0.0
        self.drive = 0.0
        self.Kp = 0.2
        self.Ki = 0.0
        self.integral = 0.0
        self.last_sample_seen = False
        self.clamp = 0.2
        self.hysteresis = 0.5
        self.enabled = False
        self.active = False
        # Module physically present (host config knob, mirrors `installed`
        # in tempctrl.h). Distinct from `enabled` (drive intent) and
        # `cooling_enabled` (drive-polarity guard): an uninstalled channel
        # is never sampled — its ADC input is never mux-selected — and
        # never driven. Default true so a rebooted pico behaves exactly
        # as before the flag existed until the host replays config.
        self.installed = True
        # Per-cycle data validity (NOT a latch, mirrors data_invalid in
        # tempctrl.h): True when the most recent sample cycle produced no
        # trustworthy temperature — plausibility failure or rate-guard
        # reject. Drives the per-channel status string and the null
        # T_now/resistance reporting. True at boot: no sample taken yet.
        self.data_invalid = True
        # Asymmetric clamp guard (see tempctrl.h). True preserves the
        # original symmetric drive range; False forbids drive<0 so the
        # PI loop saturates at [0, +clamp] instead of [-clamp, +clamp].
        self.cooling_enabled = True
        self.timestamp = 0.0
        # Stall guard mirror (see tempctrl_check_stall in tempctrl.c).
        # stall_tripped = drive did nothing for a full window;
        # runaway_tripped = T moved against the drive for consecutive
        # windows. Separate flags because the field diagnoses differ.
        self.stall_tripped = False
        self.runaway_tripped = False
        self.stall_window_active = False
        self.stall_check_T = 0.0
        self.stall_check_drive = 0.0
        self.stall_check_time = 0.0
        # Test hook: when True, skip the thermal-drift update even with the
        # drive engaged, so the stall guard can be exercised deterministically.
        self.thermal_frozen = False
        # Mirrors temp_sensor_has_error()'s underlying state (plausibility
        # failure: railed divider). Self-clears into data_invalid on every
        # tick, so a recovered read fault recovers status (matches firmware
        # tempctrl_update_sensor_drive). The rate-sanity latch
        # (sensor_tripped) is sticky by contrast.
        self._sensor_error = False
        # Runaway guard mirror (see tempctrl_check_stall in tempctrl.c):
        # consecutive wrong-direction windows.
        self.runaway_strikes = 0
        # Sensor sanity guard mirror (see tempctrl_update_sensor_drive).
        # rate_ref_valid gates control and the rate guard; sensor_rejects
        # counts consecutive rejected samples and latches at MAX_REJECTS.
        # seed_pending marks a first (candidate) sample awaiting confirmation:
        # the reference anchors only when two consecutive samples agree within
        # the rate budget (two-to-anchor), so a lone transient cannot poison it.
        # sensor_tripped latches once sensor_rejects reaches MAX_REJECTS and
        # is cleared only by a *_enable=true host ack (like stall_tripped),
        # so a sensor that produced a burst of garbage cannot silently
        # re-enable drive when a later reading happens to look plausible.
        self.rate_ref_valid = False
        self.seed_pending = False
        self.sensor_rejects = 0
        self.sensor_tripped = False
        # Test hook: queued bogus sensor readings, one consumed per op tick,
        # standing in for a failing thermistor. A normal tick (empty queue)
        # reads the thermal-model T_now and always passes the rate guard.
        # See inject_sensor_glitch.
        self._glitch_queue = []
        _set_thermistor_diagnostics(self, self.T_now)

    def inject_sensor_glitch(self, value, count=1):
        """Queue ``count`` bogus raw sensor readings of ``value``.

        Each is consumed on a subsequent op tick and run through
        the rate-of-change guard, mirroring a thermistor/ADC path that
        returns garbage. A jump larger than ``MAX_RATE_C_PER_S * dt`` is
        rejected (``T_now`` holds, ``sensor_rejects`` increments, the cycle
        reports ``data_invalid``); after ``MAX_REJECTS`` consecutive rejects
        the channel latches the sticky ``sensor_tripped``, which gates drive
        until the host acks with ``*_enable=true``.
        """
        self._glitch_queue.extend([float(value)] * count)


def _reset_controller_state(tc):
    """Match tempctrl_reset_controller_state() in tempctrl.c."""
    tc.drive = 0.0
    tc.integral = 0.0
    tc.last_sample_seen = False
    tc.active = False


def tempctrl_pi_drive(tc, dt=DT_PER_SAMPLE_S):
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

    # Asymmetric clamp: cooling_enabled=False forbids negative drive so
    # the PI loop saturates at [0, +clamp] instead of [-clamp, +clamp].
    # Mirrors the firmware lower_clamp in tempctrl_pi_drive (tempctrl.c).
    lower_clamp = -tc.clamp if tc.cooling_enabled else 0.0

    sat_high = tentative_drive > tc.clamp and T_delta > 0
    sat_low = tentative_drive < lower_clamp and T_delta < 0

    if sat_high:
        tc.drive = tc.clamp
    elif sat_low:
        tc.drive = lower_clamp
    else:
        tc.integral = tentative_i
        tc.drive = max(lower_clamp, min(tc.clamp, tentative_drive))


# Mirrors TEMPCTRL_STALL_WINDOW_MS / TEMPCTRL_STALL_MIN_DELTA in tempctrl.h.
STALL_WINDOW_MS = 60000
STALL_MIN_DELTA = 0.5

# Mirrors TEMPCTRL_RUNAWAY_STRIKES / TEMPCTRL_MAX_RATE_C_PER_S /
# TEMPCTRL_MAX_REJECTS in tempctrl.h. See the firmware comments for the
# rationale (wrong-direction runaway trip; physically-impossible-jump
# sensor rejection with a consecutive-reject latch).
RUNAWAY_STRIKES = 2
MAX_RATE_C_PER_S = 5.0
MAX_REJECTS = 3


class TempCtrlEmulator(PicoEmulator):
    """Emulates src/tempctrl.c firmware."""

    def __init__(self, app_id=1, **kwargs):
        self.lna = TempControlState()
        self.load = TempControlState()
        self.watchdog_timeout_ms = 30000
        self.watchdog_tripped = False
        self._last_cmd_time = time.time()
        # Boot reference for the *_timestamp field. Firmware sends
        # temp_sensor_get_sample_time() — uint32_t ms since boot, cast
        # to double in the KV_FLOAT slot. monotonic() so the value cannot
        # jump under NTP adjustments.
        self._boot_monotonic = time.monotonic()
        super().__init__(app_id=app_id, **kwargs)

    def init(self):
        self.lna = TempControlState()
        self.load = TempControlState()
        self.watchdog_timeout_ms = 30000
        self.watchdog_tripped = False
        self._last_cmd_time = time.time()
        self._boot_monotonic = time.monotonic()

    def _ms_since_boot(self):
        return float(int((time.monotonic() - self._boot_monotonic) * 1000))

    def server(self, cmd):
        # Any valid command refreshes the watchdog timer, but the trip flag
        # is sticky: the host clears it by explicitly sending *_enable=true
        # (see firmware tempctrl_apply_enable).
        self._last_cmd_time = time.time()

        for prefix, tc in [("LNA", self.lna), ("LOAD", self.load)]:
            # Numeric fields use _coerce_float (not _safe_float): cJSON
            # writes valuedouble=0.0 for non-numeric JSON, and firmware
            # assigns that directly into the struct.
            key = f"{prefix}_temp_target"
            if key in cmd:
                tc.T_target = _coerce_float(cmd[key])

            key = f"{prefix}_installed"
            if key in cmd:
                # Mirror firmware bool parse (item_json->valueint ? true
                # : false). Pure presence flag — NOT a trip ack: sticky
                # latches survive an uninstall/re-install cycle and clear
                # only via *_enable=true (see the enable parse below).
                tc.installed = bool(_safe_int(cmd[key], 0))

            key = f"{prefix}_enable"
            if key in cmd:
                # cJSON's valueint is 0 for non-numeric types — mirror that
                # so {"LNA_enable": "false"} disables on the emulator just
                # like it does on firmware (instead of being truthy).
                new_enabled = bool(_safe_int(cmd[key], 0))
                # *_enable=true is the host's ack of sticky trips: it clears
                # this channel's trip flags and the app-wide watchdog flag.
                # `enabled` is host intent only; firmware never mutates it.
                if new_enabled:
                    tc.stall_tripped = False
                    tc.runaway_tripped = False
                    tc.stall_window_active = False
                    tc.runaway_strikes = 0
                    tc.sensor_rejects = 0
                    tc.sensor_tripped = False
                    tc.rate_ref_valid = False
                    tc.seed_pending = False
                    self.watchdog_tripped = False
                tc.enabled = new_enabled

            key = f"{prefix}_hysteresis"
            if key in cmd:
                tc.hysteresis = _coerce_float(cmd[key])

            key = f"{prefix}_clamp"
            if key in cmd:
                tc.clamp = min(1.0, max(0.0, _coerce_float(cmd[key])))

            key = f"{prefix}_Kp"
            if key in cmd:
                tc.Kp = _coerce_float(cmd[key])

            key = f"{prefix}_Ki"
            if key in cmd:
                new_ki = _coerce_float(cmd[key])
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

            key = f"{prefix}_cooling_enabled"
            if key in cmd:
                # Mirror firmware bool parse (item_json->valueint ? true : false)
                # and the existing *_enable handling: cJSON's valueint is 0 for
                # non-numeric JSON, so a string like "false" disables.
                tc.cooling_enabled = bool(_safe_int(cmd[key], 0))

        if "watchdog_timeout_ms" in cmd:
            val = _safe_int(
                cmd["watchdog_timeout_ms"], self.watchdog_timeout_ms
            )
            # Firmware clamps negatives to 0 (see tempctrl.c watchdog parse).
            self.watchdog_timeout_ms = 0 if val < 0 else val

    def inject_sensor_error(self, channel, error=True):
        """Simulate a plausibility failure on channel "LNA" or "LOAD".

        Models a railed divider (open/short thermistor): the conversion
        fails, so the cycle reports invalid data. Sets both the underlying
        sensor-error state and the current ``data_invalid`` flag so callers
        that introspect status without first running op() see the change
        immediately. op() then keeps ``data_invalid`` in sync with the
        sensor state on every tick (mirroring firmware's per-cycle
        recompute), so a recovered fault self-clears.
        """
        tc = self.lna if channel == "LNA" else self.load
        tc._sensor_error = error
        tc.data_invalid = error

    def _drive_allowed(self, tc):
        return (
            tc.enabled
            and not tc.sensor_tripped
            and not tc.stall_tripped
            and not tc.runaway_tripped
            and not self.watchdog_tripped
        )

    def _read_sensor(self, tc):
        """Mirror the sensor read + rate-of-change sanity guard in
        tempctrl_update_sensor_drive. Returns ``(plausible, rejected)``:
        ``plausible`` is False when the voltage->temperature conversion
        failed (mirrors temp_sensor_read()); ``rejected`` is True when an
        anchored rate check discarded the sample. ``raw`` is the bogus
        queued reading when a glitch is pending, else the thermal-model
        ``T_now`` (a normal reading, which equals the current ``T_now`` and
        so always passes). dt is one sample tick (``DT_PER_SAMPLE_S``)
        because the firmware advances its rate reference on every plausible
        sample.

        Two-to-anchor: until rate_ref_valid is set, the first sample is only a
        candidate (held in T_now) and the reference anchors only when a second
        sample confirms it within the rate budget. An unconfirmed candidate is
        replaced and counts toward the latch, so a sensor that never produces
        two consistent readings still latches. A plausibility-failed cycle
        drops the anchor entirely (mirrors firmware): it is only valid across
        continuous good data, so recovery after an outage re-seeds instead of
        judging a legitimate drift against a stale reference.
        """
        if tc._sensor_error:
            # Railed divider: the measured voltage is still stored (open
            # thermistor pulls the node to supply), temperature/resistance
            # hold last-good — mirrors temp_sensor_read() storing voltage
            # before the conversion check.
            tc.voltage = THERMISTOR_SUPPLY_VOLTS
            tc.rate_ref_valid = False
            tc.seed_pending = False
            return False, False
        raw = tc._glitch_queue.pop(0) if tc._glitch_queue else tc.T_now
        _set_thermistor_diagnostics(tc, raw)
        if not tc.rate_ref_valid:
            if not tc.seed_pending:
                # First sample: hold as candidate, await confirmation.
                tc.T_now = raw
                tc.seed_pending = True
                tc.sensor_rejects = 0
                return True, False
            rate = abs(raw - tc.T_now) / DT_PER_SAMPLE_S
            if rate > MAX_RATE_C_PER_S:
                # Candidate unconfirmed: replace it, count toward the latch.
                tc.T_now = raw
                if tc.sensor_rejects < MAX_REJECTS:
                    tc.sensor_rejects += 1
            else:
                # Two consecutive consistent samples: anchor the reference.
                tc.T_now = raw
                tc.rate_ref_valid = True
                tc.seed_pending = False
                tc.sensor_rejects = 0
            return True, False
        rate = abs(raw - tc.T_now) / DT_PER_SAMPLE_S
        if rate > MAX_RATE_C_PER_S:
            # Reject: hold the last good T_now, count toward the latch.
            if tc.sensor_rejects < MAX_REJECTS:
                tc.sensor_rejects += 1
            return True, True
        tc.T_now = raw
        tc.sensor_rejects = 0
        return True, False

    def _update_channel(self, tc):
        # Channel hardware not present: return before the sensor read so
        # the ADC input is never mux-selected (the potmon crosstalk
        # lesson — a dead divider must not share the mux with a live
        # one) and force drive off. data_invalid every cycle; the rate
        # anchor drops so a later re-install re-seeds two-to-anchor.
        # Mirrors the !installed early return in
        # tempctrl_update_sensor_drive (tempctrl.c).
        if not tc.installed:
            tc.data_invalid = True
            tc.rate_ref_valid = False
            tc.seed_pending = False
            _reset_controller_state(tc)
            tc.stall_window_active = False
            tc.runaway_strikes = 0
            return

        # One sample tick (mirrors firmware: read + rate guard on the fixed
        # TEMPCTRL_SAMPLE_MS timer, then control on the filtered T_now).
        plausible, rejected = self._read_sensor(tc)
        if plausible:
            # Mirror firmware: tempctrl_status sends
            # temp_sensor_get_sample_time(), updated on each decode.
            tc.timestamp = self._ms_since_boot()

        # The rate-sanity latch is sticky once sensor_rejects hits
        # MAX_REJECTS; only the enable ack clears it (matches firmware
        # tempctrl_update_sensor_drive / tempctrl_apply_enable).
        if tc.sensor_rejects >= MAX_REJECTS:
            tc.sensor_tripped = True
        # Data validity for this cycle only: feeds status and the null
        # T_now/resistance reporting. A latched channel with plausible,
        # rate-consistent samples reports valid data — only drive gates.
        tc.data_invalid = not plausible or rejected

        # rate_ref_valid gates control too: until the reference anchors T_now
        # is only a candidate, so the channel stays idle (drive held at 0)
        # rather than driving on an unconfirmed reading. A
        # plausibility-failed cycle also gates — the sensor may be gone
        # entirely (matches firmware).
        if self._drive_allowed(tc) and tc.rate_ref_valid and plausible:
            if not rejected:
                tempctrl_pi_drive(tc)
                self._check_stall(tc)
            # Lone reject: nothing new to act on — hold the previous drive
            # (PWM keeps it) and the stall window (matches firmware).
        else:
            _reset_controller_state(tc)
            tc.stall_window_active = False
            tc.runaway_strikes = 0

        # Thermal model: drive set by PI is held between PI ticks (mirrors
        # continuous PWM), so the effect accumulates every op tick. When
        # the controller is disengaged, drive=0 (from _reset_controller_state)
        # and T_now stays put — matches firmware behavior where T_now is
        # the sensor reading and no thermal source is being driven.
        if not tc.thermal_frozen:
            tc.T_now += tc.drive * THERMAL_DRIFT_PER_OP

    def _check_stall(self, tc):
        """Mirror tempctrl_check_stall() from tempctrl.c."""
        # `active` alone isn't sufficient: with cooling_enabled=False the PI
        # loop can sit outside the deadband (active=True) while saturated at
        # drive=0, which is the configured refusal-to-cool, not a stall.
        if not tc.active or tc.drive == 0.0:
            tc.stall_window_active = False
            tc.runaway_strikes = 0
            return
        now = time.time()
        if not tc.stall_window_active:
            tc.stall_check_T = tc.T_now
            tc.stall_check_drive = tc.drive
            tc.stall_check_time = now
            tc.stall_window_active = True
            return
        elapsed_ms = (now - tc.stall_check_time) * 1000
        if elapsed_ms < STALL_WINDOW_MS:
            return
        delta = tc.T_now - tc.stall_check_T
        if abs(delta) < STALL_MIN_DELTA:
            # No movement — sensor stuck or Peltier ineffective.
            tc.stall_tripped = True
            tc.active = False
            tc.drive = 0.0
            tc.stall_window_active = False
            tc.runaway_strikes = 0
            return
        # Moved the wrong direction for the drive → runaway signature.
        # Trip only after RUNAWAY_STRIKES consecutive wrong-direction
        # windows (tolerates the startup/soak transient). Separate flag
        # from stall_tripped: "drive made it worse" is a different field
        # diagnosis than "drive did nothing".
        if delta * tc.stall_check_drive < 0.0:
            tc.runaway_strikes += 1
            if tc.runaway_strikes >= RUNAWAY_STRIKES:
                tc.runaway_tripped = True
                tc.active = False
                tc.drive = 0.0
                tc.stall_window_active = False
                tc.runaway_strikes = 0
                return
        else:
            tc.runaway_strikes = 0
        # Roll the window forward.
        tc.stall_check_T = tc.T_now
        tc.stall_check_drive = tc.drive
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
        # Per-channel status reports DATA VALIDITY ONLY; the sticky control
        # latches (sensor/stall/runaway_tripped) gate drive but never set
        # status. Invalid cycles report T_now/resistance as None (the
        # firmware sends NaN KV_FLOATs, which cJSON prints as JSON null)
        # while voltage stays live for open-vs-short diagnosis.
        lna_status = "error" if self.lna.data_invalid else "update"
        load_status = "error" if self.load.data_invalid else "update"
        return {
            "sensor_name": "tempctrl",
            "app_id": self.app_id,
            "watchdog_tripped": self.watchdog_tripped,
            "watchdog_timeout_ms": self.watchdog_timeout_ms,
            "LNA_status": lna_status,
            "LNA_T_now": None if self.lna.data_invalid else self.lna.T_now,
            "LNA_voltage": self.lna.voltage,
            "LNA_resistance": (
                None if self.lna.data_invalid else self.lna.resistance
            ),
            "LNA_timestamp": self.lna.timestamp,
            "LNA_T_target": self.lna.T_target,
            "LNA_drive_level": self.lna.drive,
            "LNA_installed": self.lna.installed,
            "LNA_enabled": self.lna.enabled,
            "LNA_active": self.lna.active,
            "LNA_sensor_tripped": self.lna.sensor_tripped,
            "LNA_stall_tripped": self.lna.stall_tripped,
            "LNA_runaway_tripped": self.lna.runaway_tripped,
            "LNA_cooling_enabled": self.lna.cooling_enabled,
            "LNA_hysteresis": self.lna.hysteresis,
            "LNA_clamp": self.lna.clamp,
            "LNA_Kp": self.lna.Kp,
            "LNA_Ki": self.lna.Ki,
            "LNA_integral": self.lna.integral,
            "LOAD_status": load_status,
            "LOAD_T_now": None if self.load.data_invalid else self.load.T_now,
            "LOAD_voltage": self.load.voltage,
            "LOAD_resistance": (
                None if self.load.data_invalid else self.load.resistance
            ),
            "LOAD_timestamp": self.load.timestamp,
            "LOAD_T_target": self.load.T_target,
            "LOAD_drive_level": self.load.drive,
            "LOAD_installed": self.load.installed,
            "LOAD_enabled": self.load.enabled,
            "LOAD_active": self.load.active,
            "LOAD_sensor_tripped": self.load.sensor_tripped,
            "LOAD_stall_tripped": self.load.stall_tripped,
            "LOAD_runaway_tripped": self.load.runaway_tripped,
            "LOAD_cooling_enabled": self.load.cooling_enabled,
            "LOAD_hysteresis": self.load.hysteresis,
            "LOAD_clamp": self.load.clamp,
            "LOAD_Kp": self.load.Kp,
            "LOAD_Ki": self.load.Ki,
            "LOAD_integral": self.load.integral,
        }
