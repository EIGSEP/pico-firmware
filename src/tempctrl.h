#ifndef TEMPCTRL_H
#define TEMPCTRL_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "pico/time.h"
#include "hardware/gpio.h"
#include "eigsep_command.h"
#include "temp_simple.h"

// LOAD Temperature Control configuration. A single low-side N-FET heater:
// it can only heat, so control is on/off hysteresis, not PI. There is no
// PWM/direction pin pair — just the ADC-capable thermistor pin and the FET
// gate (see LOAD_FET_PIN).
#define TEMP_SENSOR_LOAD_PIN    26
#define LOAD_FET_PIN             17  // low-side N-FET gate (active high)

// LNA thermistor channels — read-only telemetry, no control logic (no
// FET, no target/hysteresis, no rate/stall/runaway guards). Physically
// the same NTC part and Steinhart-Hart characterization as LOAD (see
// LOAD_THERM_* below); update these independently if a different part
// is used for either channel.
#define LNA1_THERM_PIN          27
#define LNA2_THERM_PIN          28

// LOAD (and LNA1/LNA2) thermistor: Vishay NTCLE100E3103HB0, 10 kOhm at
// 25 C (+/-3 %), B25/85 = 3977 K (+/-0.75 %), rated -40..+125 C
// continuous; wired `3V3 -[10.01k fixed]- ADC pin -[NTC]- GND`.
// Deliberately independent constants; do not merge with THERMISTOR_* in
// temp_simple.h (a different divider).
//
// Vishay's extended Steinhart-Hart (NTCLE100E3 datasheet, document 29049,
// "parameter for determining nominal resistance values", row 9,
// B25/85 = 3977 K):
//   1/T = A1 + B1 ln(R/Rref) + C1 ln^2(R/Rref) + D1 ln^3(R/Rref)
// with T in kelvin and Rref = R25 = 10 kOhm. Vishay quotes it as agreeing
// with the nominal R-T curve to 0.005 C over 25..125 C (0.015 C over
// -40..25 C). Absolute accuracy is set by the part tolerance: +/-3 % on
// R25 is about +/-0.7 C at 25 C. LOAD_THERM_PULLUP_OHMS is the fixed
// resistor's value (10.01 kOhm); a 1 % error in it is a ~0.2-0.5 C
// temperature error over 25..90 C.
#define LOAD_THERM_PULLUP_OHMS      10010.0f
#define LOAD_THERM_REF_OHMS         10000.0f
#define LOAD_THERM_SH_A1            3.354016e-3f
#define LOAD_THERM_SH_B1            2.569850e-4f
#define LOAD_THERM_SH_C1            2.620131e-6f
#define LOAD_THERM_SH_D1            6.383091e-8f

// Absolute safety ceiling, independent of whatever target/hysteresis the
// host has configured: a plausible, rate-guard-anchored reading at or
// above this forces the FET off and latches LOAD_runaway_tripped (the
// control loop failed to prevent overheating). Not firmware-enforced on
// the setpoint itself — setpoint sanity is a host-side concern — but is
// documented here as the margin the ceiling assumes.
#define LOAD_MAX_SAFE_TEMP_C         110.0f
#define LOAD_DEFAULT_HYSTERESIS_C    2.0f

// Sensor sampling cadence. tempctrl_op() runs on every main-loop pass, but
// sampling, the rate guard, and the PI step only execute when this timer
// elapses. A fixed cadence is what calibrates the rate guard: its rejection
// step is TEMPCTRL_MAX_RATE_C_PER_S * (TEMPCTRL_SAMPLE_MS/1000) and its
// latch evidence is TEMPCTRL_MAX_REJECTS consecutive sample periods. With
// unthrottled per-loop sampling those quantities depended on loop speed
// (sub-ms idle, ms-scale during serial drains), making the guard bypassed
// or hair-triggered depending on host traffic.
#define TEMPCTRL_SAMPLE_MS         200

// Stall detection: if the channel is actively driving (FET on) but T_now
// fails to move by at least TEMPCTRL_STALL_MIN_DELTA over a
// TEMPCTRL_STALL_WINDOW_MS window, the sensor or heater is stuck and we
// trip the channel. A healthy heater moves the load several C/min, so a
// healthy run rolls the window forward long before reaching the trip
// threshold.
#define TEMPCTRL_STALL_WINDOW_MS   120000
#define TEMPCTRL_STALL_MIN_DELTA   0.1f

// Runaway guard: a channel that is actively driving (FET on) but whose
// temperature moves the wrong direction (falls while heating) is
// mis-wired, has lost thermal contact, or has a swapped sensor — a
// thermal-runaway signature the no-movement stall guard above cannot catch
// (the temperature is moving, so |delta| stays above
// TEMPCTRL_STALL_MIN_DELTA). Evaluated on the same stall window: a
// wrong-direction window scores a strike, and the channel trips (via
// runaway_tripped) after TEMPCTRL_RUNAWAY_STRIKES consecutive strikes.
// Requiring consecutive windows tolerates the startup transient where
// T_now lags the junction without false-tripping a channel that is
// actually controlling.
#define TEMPCTRL_RUNAWAY_STRIKES   2

// Sensor sanity guard: a thermistor/ADC path can return electrically valid
// but physically impossible values (multiplexed-ADC channel crosstalk is the
// observed failure mode on this hardware). Reject any fresh sample whose
// implied rate of change exceeds TEMPCTRL_MAX_RATE_C_PER_S (well above any
// real thermal slew — a healthy heater moves a few C/min, ~0.1 C/s); hold
// the last good T_now instead. At the fixed 200 ms sample cadence this
// rejects steps larger than ~1 C. After TEMPCTRL_MAX_REJECTS consecutive
// rejects (~600 ms of sustained garbage) the channel latches
// sensor_tripped, which gates drive until the host acks with *_enable=true.
// The latch transition also drops the rate anchor: with drive gated the
// frozen reference serves no control purpose, so the channel re-seeds
// two-to-anchor from the sensor's actual level (a trip whose sensor settles
// at a shifted level would otherwise reject every healthy sample against
// the stale reference forever, pegging sensor_rejects).
// The guard is CONTROL-ONLY: a rejected sample was still a plausible ADC
// conversion, so it is reported (status "update", raw value) with the
// published sensor_rejects counter as the cross-check marker — the firmware
// never withholds science data on rate statistics alone. A lone glitch is
// absorbed: control continues on the next accepted sample.
#define TEMPCTRL_MAX_RATE_C_PER_S  5.0f
#define TEMPCTRL_MAX_REJECTS       3

// LOAD channel: analog NTC thermistor + single low-side FET heater. Drive
// is on/off (`active`) rather than a continuous PI drive level, since a
// single low-side FET has no reverse (cooling) direction and no PWM duty
// to modulate — see load_heater_control() in tempctrl.c.
typedef struct {
    uint fet_pin;
    uint adc_input;
    bool adc_configured;

    // T_raw is the most recent plausible conversion: updated on every
    // plausible read, including rate-guard-rejected ones, and is what
    // status reports as *_T_now. T_now is the rate-guard-filtered control
    // reference (holds on reject) and what the hysteresis control and
    // stall/runaway guard act on.
    float T_raw;
    float T_now;
    float voltage;
    float resistance;
    float T_target;
    float hysteresis;
    uint32_t last_sample_time;  // ms since boot of last accepted ADC sample

    bool active;      // FET actually driving (on) this tick
    // `enabled` is host intent only; firmware never mutates it. The
    // channel drives only when enabled && no trip flag is set below.
    bool enabled;
    // Module physically present (host config knob via LOAD_installed;
    // firmware never mutates it). An uninstalled channel is never sampled
    // (ADC input never mux-selected) and never driven; the picohost
    // fan-out suppresses its Redis stream entirely. Default true.
    bool installed;
    // Per-cycle data validity (NOT a latch): true when this sample cycle
    // produced no measurement (plausibility failure or not installed).
    // Drives the per-channel status string and null T_now/resistance
    // reporting; self-clears on the next plausible sample.
    bool data_invalid;

    // Sensor sanity (rate) guard — two-to-anchor scheme (see
    // load_heater_update in tempctrl.c).
    bool rate_ref_valid;
    bool seed_pending;
    uint32_t rate_ref_ms;
    uint8_t sensor_rejects;
    bool sensor_tripped;

    // Stall / runaway guard over a rolling TEMPCTRL_STALL_WINDOW_MS
    // window while active. stall_tripped = FET on for a full window with
    // no meaningful rise (heater ineffective or sensor stuck).
    // runaway_tripped = temperature fell for TEMPCTRL_RUNAWAY_STRIKES
    // consecutive windows while the FET was on (mis-wired FET/sensor, or
    // the sensor reads a different thermal mass than the heater) — also
    // the flag the absolute LOAD_MAX_SAFE_TEMP_C ceiling latches, since
    // both mean "the control loop is not doing what it should."
    bool stall_window_active;
    float stall_check_T;
    absolute_time_t stall_check_time;
    uint8_t runaway_strikes;
    bool stall_tripped;
    bool runaway_tripped;
} LoadHeater;

// Read-only thermistor channel: samples + reports T_now via the shared
// Steinhart-Hart conversion, no heater, no control loop, no host-facing
// settings (nothing to command — see LNA1_THERM_PIN / LNA2_THERM_PIN).
typedef struct {
    uint adc_input;
    bool adc_configured;

    float T_now;
    float voltage;
    float resistance;
    // True when this sample cycle produced no measurement (plausibility
    // failure or ADC not configured) — same per-cycle (not latched)
    // semantics as LoadHeater.data_invalid.
    bool data_invalid;
    uint32_t last_sample_time;  // ms since boot of last plausible sample
} LnaThermistor;

// Standard app interface functions
void tempctrl_init(uint8_t app_id);
void tempctrl_server(uint8_t app_id, const char *json_str);
void tempctrl_op(uint8_t app_id);
void tempctrl_status(uint8_t app_id);

#endif // TEMPCTRL_H
