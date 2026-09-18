import math
import time

from .base import PicoEmulator, _safe_int


# Mirror the firmware's fixed sampling cadence: sampling, the rate guard,
# and the on/off control step run on the TEMPCTRL_SAMPLE_MS = 200 ms timer
# inside tempctrl_op() (between timer ticks op is a no-op and the FET GPIO
# holds its level). One emulator op() call models one sample tick.
DT_PER_SAMPLE_S = 0.2

# Thermistor divider constants, mirroring temp_simple.h.
THERMISTOR_SUPPLY_VOLTS = 3.3

# LOAD channel thermistor: a 30k NTC (Beta=3943) behind a plain 10k
# pull-up to 3V3. Mirrors LOAD_THERM_* in tempctrl.h.
LOAD_THERM_PULLUP_OHMS = 10000.0
LOAD_THERM_R0_OHMS = 30000.0
LOAD_THERM_T0_KELVIN = 298.15  # 25 C
LOAD_THERM_BETA_KELVIN = 3943.0
LOAD_MAX_SAFE_TEMP_C = 100.0
LOAD_DEFAULT_HYSTERESIS_C = 2.0

# LOAD thermal model: a fixed rise per op tick while the FET is on, and a
# passive cooldown toward ambient while it's off, standing in for a real
# heater/thermistor pair — enough to exercise the hysteresis, stall, and
# runaway logic under test without modeling real thermal mass.
LOAD_THERMAL_DRIFT_PER_OP = 0.05
LOAD_AMBIENT_C = 22.0
LOAD_COOLING_PER_OP = 0.01


def _load_thermistor_resistance(temp_c):
    """Beta-equation forward fit: temperature (deg C) -> ohms. Inverse of
    thermistor_voltage_to_temperature() in tempctrl.c."""
    kelvin = temp_c + 273.15
    return LOAD_THERM_R0_OHMS * math.exp(
        LOAD_THERM_BETA_KELVIN * (1.0 / kelvin - 1.0 / LOAD_THERM_T0_KELVIN)
    )


def _load_thermistor_voltage(resistance):
    return (
        THERMISTOR_SUPPLY_VOLTS
        * resistance
        / (LOAD_THERM_PULLUP_OHMS + resistance)
    )


def _set_thermistor_diagnostics(lh, temp_c):
    """Derive (resistance, voltage) from temp_c for any channel using the
    shared Beta-equation characterization (LOAD, LNA1, LNA2 alike)."""
    lh.resistance = _load_thermistor_resistance(temp_c)
    lh.voltage = _load_thermistor_voltage(lh.resistance)


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


class LoadHeaterState:
    """Models the LoadHeater struct from tempctrl.h.

    A single low-side FET heater behind an analog thermistor — on/off
    hysteresis control, no PI loop, no reverse (cooling) direction.
    """

    def __init__(self):
        self.T_now = 25.0
        self.T_target = 30.0
        self.voltage = 0.0
        self.resistance = 0.0
        # Mirrors LoadHeater.T_raw (tempctrl.h): the most recent plausible
        # conversion, updated on every plausible read including
        # rate-guard-rejected ones. This is the REPORTED value; T_now is
        # the rate-guard-filtered control reference and holds on reject.
        self.temperature = 0.0
        self.hysteresis = LOAD_DEFAULT_HYSTERESIS_C
        self.active = False
        self.enabled = False
        # Module physically present (host config knob, mirrors `installed`
        # in tempctrl.h). Default True so a rebooted pico behaves as
        # before the flag existed until the host replays config.
        self.installed = True
        # Per-cycle data validity (NOT a latch, mirrors data_invalid in
        # tempctrl.h). True at boot: no sample taken yet.
        self.data_invalid = True
        self.timestamp = 0.0

        # Stall/runaway guard mirror (see load_heater_check_stall in
        # tempctrl.c). stall_tripped = FET on but T_now didn't move for a
        # full window; runaway_tripped = T_now fell for
        # TEMPCTRL_RUNAWAY_STRIKES consecutive windows while the FET was
        # on — also what the LOAD_MAX_SAFE_TEMP_C ceiling latches.
        self.stall_tripped = False
        self.runaway_tripped = False
        self.stall_window_active = False
        self.stall_check_T = 0.0
        self.stall_check_time = 0.0
        self.runaway_strikes = 0

        # Sensor sanity guard mirror (see load_heater_update). Two-to-anchor
        # scheme: rate_ref_valid gates control and the rate guard;
        # sensor_rejects counts consecutive rejected samples and latches
        # at MAX_REJECTS. seed_pending marks a first (candidate) sample
        # awaiting confirmation: the reference anchors only when two
        # consecutive samples agree within the rate budget, so a lone
        # transient cannot poison it. sensor_tripped latches once
        # sensor_rejects reaches MAX_REJECTS and is cleared only by a
        # LOAD_enable=true host ack (like stall_tripped), so a sensor that
        # produced a burst of garbage cannot silently re-enable drive when
        # a later reading happens to look plausible.
        self.rate_ref_valid = False
        self.seed_pending = False
        self.sensor_rejects = 0
        self.sensor_tripped = False

        # Mirrors temp_sensor_read()-equivalent plausibility failure
        # (railed divider). Self-clears into data_invalid on every tick.
        self._sensor_error = False
        # Test hook: queued bogus raw readings, one consumed per op tick.
        self._glitch_queue = []
        # Test hook: skip the thermal model even while active, so the
        # stall guard can be exercised deterministically.
        self.thermal_frozen = False
        _set_thermistor_diagnostics(self, self.T_now)

    def inject_sensor_glitch(self, value, count=1):
        """Queue ``count`` bogus raw sensor readings of ``value``.

        Each is consumed on a subsequent op tick and run through the
        rate-of-change guard, mirroring a thermistor/ADC path that returns
        garbage. A jump larger than ``MAX_RATE_C_PER_S * dt`` is rejected
        for control (``T_now`` holds, ``sensor_rejects`` increments) but
        still reported — the raw conversion stays in the stream with the
        reject counter as the marker; after ``MAX_REJECTS`` consecutive
        rejects the channel latches the sticky ``sensor_tripped``, which
        gates drive until the host acks with ``LOAD_enable=true``.
        """
        self._glitch_queue.extend([float(value)] * count)


class LnaThermistorState:
    """Models the LnaThermistor struct from tempctrl.h: read-only
    thermistor telemetry, no control logic (no target, no drive, no
    trip latches, no rate guard — those exist to protect a heater drive
    loop, which this channel doesn't have)."""

    def __init__(self):
        self.T_now = 25.0
        self.voltage = 0.0
        self.resistance = 0.0
        # Per-cycle data validity (NOT a latch, mirrors data_invalid in
        # tempctrl.h). True at boot: no sample taken yet.
        self.data_invalid = True
        self.timestamp = 0.0
        # Mirrors a railed-divider plausibility failure. Self-clears
        # into data_invalid on every tick.
        self._sensor_error = False
        _set_thermistor_diagnostics(self, self.T_now)


# Mirrors TEMPCTRL_STALL_WINDOW_MS / TEMPCTRL_STALL_MIN_DELTA in tempctrl.h.
STALL_WINDOW_MS = 120000
STALL_MIN_DELTA = 0.3

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
        self.load = LoadHeaterState()
        self.lna1 = LnaThermistorState()
        self.lna2 = LnaThermistorState()
        self.watchdog_timeout_ms = 30000
        self.watchdog_tripped = False
        self._last_cmd_time = time.time()
        # Boot reference for the *_timestamp field. Firmware sends
        # ms since boot as a uint32_t, cast to double in the KV_FLOAT
        # slot. monotonic() so the value cannot jump under NTP adjustments.
        self._boot_monotonic = time.monotonic()
        super().__init__(app_id=app_id, **kwargs)

    def init(self):
        self.load = LoadHeaterState()
        self.lna1 = LnaThermistorState()
        self.lna2 = LnaThermistorState()
        self.watchdog_timeout_ms = 30000
        self.watchdog_tripped = False
        self._last_cmd_time = time.time()
        self._boot_monotonic = time.monotonic()

    def _ms_since_boot(self):
        return float(int((time.monotonic() - self._boot_monotonic) * 1000))

    def server(self, cmd):
        # Any valid command refreshes the watchdog timer, but the trip flag
        # is sticky: the host clears it by explicitly sending
        # LOAD_enable=true (see firmware load_heater_apply_enable).
        self._last_cmd_time = time.time()

        lh = self.load
        if "LOAD_temp_target" in cmd:
            lh.T_target = _coerce_float(cmd["LOAD_temp_target"])
        if "LOAD_installed" in cmd:
            lh.installed = bool(_safe_int(cmd["LOAD_installed"], 0))
        if "LOAD_enable" in cmd:
            new_enabled = bool(_safe_int(cmd["LOAD_enable"], 0))
            if new_enabled:
                lh.stall_tripped = False
                lh.runaway_tripped = False
                lh.stall_window_active = False
                lh.runaway_strikes = 0
                lh.sensor_rejects = 0
                lh.sensor_tripped = False
                lh.rate_ref_valid = False
                lh.seed_pending = False
                self.watchdog_tripped = False
            lh.enabled = new_enabled
        if "LOAD_hysteresis" in cmd:
            lh.hysteresis = _coerce_float(cmd["LOAD_hysteresis"])

        if "watchdog_timeout_ms" in cmd:
            val = _safe_int(
                cmd["watchdog_timeout_ms"], self.watchdog_timeout_ms
            )
            # Firmware clamps negatives to 0 (see tempctrl.c watchdog parse).
            self.watchdog_timeout_ms = 0 if val < 0 else val

    def inject_sensor_error(self, error=True, channel="LOAD"):
        """Simulate a plausibility failure on one channel.

        Models a railed divider (open/short thermistor): the conversion
        fails, so the cycle reports invalid data. Sets both the underlying
        sensor-error state and the current ``data_invalid`` flag so callers
        that introspect status without first running op() see the change
        immediately. op() then keeps ``data_invalid`` in sync with the
        sensor state on every tick (mirroring firmware's per-cycle
        recompute), so a recovered fault self-clears.

        Parameters
        ----------
        channel : {"LOAD", "LNA1", "LNA2"}
            Which channel's thermistor to fail. Default "LOAD" matches
            this method's pre-LNA behavior.
        """
        state = {"LOAD": self.load, "LNA1": self.lna1, "LNA2": self.lna2}[
            channel
        ]
        state._sensor_error = error
        state.data_invalid = error

    def _load_drive_allowed(self, lh):
        """Mirror load_heater_drive_allowed() from tempctrl.c."""
        return (
            lh.installed
            and lh.enabled
            and not lh.sensor_tripped
            and not lh.stall_tripped
            and not lh.runaway_tripped
            and not self.watchdog_tripped
        )

    def _read_load_sensor(self, lh):
        """Mirror load_heater_read() + the rate guard in
        load_heater_update() (tempctrl.c).

        Returns ``(plausible, rejected)``: ``plausible`` is False when the
        voltage->temperature conversion failed; ``rejected`` is True when
        an anchored rate check discarded the sample.

        Two-to-anchor: until rate_ref_valid is set, the first sample is
        only a candidate (held in T_now) and the reference anchors only
        when a second sample confirms it within the rate budget. An
        unconfirmed candidate is replaced and counts toward the latch, so
        a sensor that never produces two consistent readings still
        latches. A plausibility-failed cycle drops the anchor entirely
        (mirrors firmware): it is only valid across continuous good data,
        so recovery after an outage re-seeds instead of judging a
        legitimate drift against a stale reference.
        """
        if lh._sensor_error:
            # Railed divider: the measured voltage is still stored (open
            # thermistor pulls the node to supply), temperature/resistance
            # hold last-good.
            lh.voltage = THERMISTOR_SUPPLY_VOLTS
            lh.rate_ref_valid = False
            lh.seed_pending = False
            return False, False
        raw = lh._glitch_queue.pop(0) if lh._glitch_queue else lh.T_now
        _set_thermistor_diagnostics(lh, raw)
        lh.temperature = raw
        if not lh.rate_ref_valid:
            if not lh.seed_pending:
                lh.T_now = raw
                lh.seed_pending = True
                lh.sensor_rejects = 0
                return True, False
            rate = abs(raw - lh.T_now) / DT_PER_SAMPLE_S
            if rate > MAX_RATE_C_PER_S:
                lh.T_now = raw
                if lh.sensor_rejects < MAX_REJECTS:
                    lh.sensor_rejects += 1
            else:
                lh.T_now = raw
                lh.rate_ref_valid = True
                lh.seed_pending = False
                lh.sensor_rejects = 0
            return True, False
        rate = abs(raw - lh.T_now) / DT_PER_SAMPLE_S
        if rate > MAX_RATE_C_PER_S:
            if lh.sensor_rejects < MAX_REJECTS:
                lh.sensor_rejects += 1
            return True, True
        lh.T_now = raw
        lh.sensor_rejects = 0
        return True, False

    def _load_heater_control(self, lh):
        """Mirror load_heater_control() from tempctrl.c: on/off hysteresis
        (heating-only — no deadband-freeze step, no clamp/gain: the FET is
        either fully on or fully off)."""
        if lh.active:
            if lh.T_now >= lh.T_target:
                lh.active = False
        else:
            if lh.T_now <= lh.T_target - lh.hysteresis:
                lh.active = True

    def _check_load_stall(self, lh):
        """Mirror load_heater_check_stall() from tempctrl.c."""
        if not lh.active:
            lh.stall_window_active = False
            lh.runaway_strikes = 0
            return
        now = time.time()
        if not lh.stall_window_active:
            lh.stall_check_T = lh.T_now
            lh.stall_check_time = now
            lh.stall_window_active = True
            return
        elapsed_ms = (now - lh.stall_check_time) * 1000
        if elapsed_ms < STALL_WINDOW_MS:
            return
        delta = lh.T_now - lh.stall_check_T
        if delta < -STALL_MIN_DELTA:
            # Heating engaged but temperature FELL — runaway signature.
            # Score a strike; trip only after RUNAWAY_STRIKES consecutive
            # windows (tolerates a brief startup/soak transient).
            lh.runaway_strikes += 1
            if lh.runaway_strikes >= RUNAWAY_STRIKES:
                lh.runaway_tripped = True
                lh.active = False
                lh.stall_window_active = False
                lh.runaway_strikes = 0
                return
            lh.stall_check_T = lh.T_now
            lh.stall_check_time = now
            return
        lh.runaway_strikes = 0
        if delta < STALL_MIN_DELTA:
            # Heating engaged for a full window with no meaningful rise —
            # heater ineffective or sensor stuck.
            lh.stall_tripped = True
            lh.active = False
            lh.stall_window_active = False
            return
        # Healthy progress; roll the window forward.
        lh.stall_check_T = lh.T_now
        lh.stall_check_time = now

    def _update_load_channel(self, lh):
        """Mirror load_heater_update() from tempctrl.c."""
        if not lh.installed:
            lh.data_invalid = True
            lh.rate_ref_valid = False
            lh.seed_pending = False
            lh.active = False
            lh.stall_window_active = False
            lh.runaway_strikes = 0
            return

        plausible, rejected = self._read_load_sensor(lh)
        if plausible:
            lh.timestamp = self._ms_since_boot()

        if lh.sensor_rejects >= MAX_REJECTS:
            if not lh.sensor_tripped:
                lh.rate_ref_valid = False
                lh.seed_pending = False
            lh.sensor_tripped = True
        lh.data_invalid = not plausible

        # Absolute safety ceiling, independent of target/hysteresis/
        # rate-guard state — mirrors the LOAD_MAX_SAFE_TEMP_C check in
        # load_heater_update().
        if plausible and lh.rate_ref_valid and lh.T_now >= LOAD_MAX_SAFE_TEMP_C:
            lh.runaway_tripped = True

        if self._load_drive_allowed(lh) and lh.rate_ref_valid and plausible:
            if not rejected:
                self._load_heater_control(lh)
                self._check_load_stall(lh)
        else:
            lh.active = False
            lh.stall_window_active = False
            lh.runaway_strikes = 0

        # Thermal model: heat while the FET is on, passively cool toward
        # ambient while it's off — enough to exercise hysteresis/stall/
        # runaway under test without modeling real thermal mass.
        if not lh.thermal_frozen:
            if lh.active:
                lh.T_now += LOAD_THERMAL_DRIFT_PER_OP
            elif lh.T_now > LOAD_AMBIENT_C:
                lh.T_now = max(LOAD_AMBIENT_C, lh.T_now - LOAD_COOLING_PER_OP)

    def _update_lna_channel(self, lt):
        """Mirror lna_thermistor_update() from tempctrl.c: plain sample +
        report, no control logic, no rate guard. Tests drive the
        reported value by setting ``lt.T_now`` directly before calling
        ``op()`` (there is no thermal model to override, unlike LOAD)."""
        if lt._sensor_error:
            # Railed divider: voltage pinned to supply, temperature/
            # resistance hold last-good — mirrors _read_load_sensor.
            lt.voltage = THERMISTOR_SUPPLY_VOLTS
            lt.data_invalid = True
            return
        _set_thermistor_diagnostics(lt, lt.T_now)
        lt.data_invalid = False
        lt.timestamp = self._ms_since_boot()

    def op(self):
        # Communication watchdog: trip the app-wide flag if no command has
        # arrived within the timeout. `enabled` is host intent and stays
        # untouched; the trip flag is the runtime gate.
        if self.watchdog_timeout_ms > 0 and not self.watchdog_tripped:
            elapsed_ms = (time.time() - self._last_cmd_time) * 1000
            if elapsed_ms > self.watchdog_timeout_ms:
                self.watchdog_tripped = True

        self._update_load_channel(self.load)
        self._update_lna_channel(self.lna1)
        self._update_lna_channel(self.lna2)

    def get_status(self):
        # Status reports DATA EXISTENCE ONLY: "error" means the cycle
        # produced no measurement (plausibility failure / channel not
        # installed), reported as None T_now/resistance (the firmware
        # sends NaN KV_FLOATs, which cJSON prints as JSON null) while
        # voltage stays live for open-vs-short diagnosis. T_now reports
        # the raw conversion (load.temperature), NOT the control
        # reference — a rate-guard-rejected sample stays in the stream
        # with the sensor_rejects counter as the cross-check marker. The
        # sticky control latches (sensor/stall/runaway_tripped) gate
        # drive but never set status.
        load_status = "error" if self.load.data_invalid else "update"
        # LOAD_drive_level is a float for shape consistency with the rest
        # of the status fields, but is binary: 1.0 while the FET is on,
        # 0.0 otherwise (there is no PWM duty to report). Mirrors
        # tempctrl_status's load_drive_level in tempctrl.c.
        load_drive_level = 1.0 if self.load.active else 0.0
        out = {
            "sensor_name": "tempctrl",
            "app_id": self.app_id,
            "watchdog_tripped": self.watchdog_tripped,
            "watchdog_timeout_ms": self.watchdog_timeout_ms,
            "LOAD_status": load_status,
            "LOAD_T_now": (
                None if self.load.data_invalid else self.load.temperature
            ),
            "LOAD_voltage": self.load.voltage,
            "LOAD_resistance": (
                None if self.load.data_invalid else self.load.resistance
            ),
            "LOAD_timestamp": self.load.timestamp,
            "LOAD_T_target": self.load.T_target,
            "LOAD_drive_level": load_drive_level,
            "LOAD_installed": self.load.installed,
            "LOAD_enabled": self.load.enabled,
            "LOAD_active": self.load.active,
            "LOAD_sensor_tripped": self.load.sensor_tripped,
            "LOAD_sensor_rejects": self.load.sensor_rejects,
            "LOAD_stall_tripped": self.load.stall_tripped,
            "LOAD_runaway_tripped": self.load.runaway_tripped,
            "LOAD_hysteresis": self.load.hysteresis,
        }
        # LNA1/LNA2: same "error" iff no measurement this cycle" status
        # semantics as LOAD, no control state to report alongside it.
        for prefix, lt in (("LNA1", self.lna1), ("LNA2", self.lna2)):
            out[f"{prefix}_status"] = "error" if lt.data_invalid else "update"
            out[f"{prefix}_T_now"] = None if lt.data_invalid else lt.T_now
            out[f"{prefix}_voltage"] = lt.voltage
            out[f"{prefix}_resistance"] = (
                None if lt.data_invalid else lt.resistance
            )
            out[f"{prefix}_timestamp"] = lt.timestamp
        return out
