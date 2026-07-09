#ifndef TEMPCTRL_H
#define TEMPCTRL_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "pico/time.h"
#include "hardware/gpio.h"
#include "eigsep_command.h"
#include "temp_simple.h"

// LNA Temperature Control configuration
#define TEMP_SENSOR_LNA_PIN     27  // thermistor data pin
#define PELTIER_LNA_PWM_PIN     8   // enable1
#define PELTIER_LNA_DIR_PIN1    10  // in1
#define PELTIER_LNA_DIR_PIN2    12  // in2

// LOAD Temperature Control configuration
#define TEMP_SENSOR_LOAD_PIN    26
#define PELTIER_LOAD_PWM_PIN    9   // enable2
#define PELTIER_LOAD_DIR_PIN3   11  // in3
#define PELTIER_LOAD_DIR_PIN4   13  // in4

// PWM configuration
#define PWM_WRAP            1000

// Sensor sampling cadence. tempctrl_op() runs on every main-loop pass, but
// sampling, the rate guard, and the PI step only execute when this timer
// elapses. A fixed cadence is what calibrates the rate guard: its rejection
// step is TEMPCTRL_MAX_RATE_C_PER_S * (TEMPCTRL_SAMPLE_MS/1000) and its
// latch evidence is TEMPCTRL_MAX_REJECTS consecutive sample periods. With
// unthrottled per-loop sampling those quantities depended on loop speed
// (sub-ms idle, ms-scale during serial drains), making the guard bypassed
// or hair-triggered depending on host traffic.
#define TEMPCTRL_SAMPLE_MS         200

// Stall detection: if the channel is actively driving (drive!=0) but T_now
// fails to move by at least TEMPCTRL_STALL_MIN_DELTA over a
// TEMPCTRL_STALL_WINDOW_MS window, the sensor or Peltier is stuck and we
// trip the channel. A healthy half-power Peltier moves the load several
// C/min, so a healthy run rolls the window forward long before reaching
// the trip threshold.
#define TEMPCTRL_STALL_WINDOW_MS   120000
#define TEMPCTRL_STALL_MIN_DELTA   0.3f

// Runaway guard: a channel that is actively driving but whose temperature
// moves the *opposite* direction of the drive (cooling drive while T rises,
// or heating drive while T falls) is mis-wired, has lost hot-side
// dissipation, or has a swapped sensor — a thermal-runaway signature the
// no-movement stall guard above cannot catch (the temperature is moving, so
// |delta| stays above TEMPCTRL_STALL_MIN_DELTA). Evaluated on the same
// stall window: a wrong-direction window scores a strike, and the channel
// trips (via stall_tripped) after TEMPCTRL_RUNAWAY_STRIKES consecutive
// strikes. Requiring consecutive windows tolerates the startup transient
// where T_now lags the junction (heat already in the mass keeps the sensor
// rising for a window after cooling engages) without false-tripping a
// channel that is actually controlling.
#define TEMPCTRL_RUNAWAY_STRIKES   2

// Sensor sanity guard: a thermistor/ADC path can return electrically valid
// but physically impossible values (multiplexed-ADC channel crosstalk is the
// observed failure mode on this hardware). Reject any fresh sample whose
// implied rate of change exceeds TEMPCTRL_MAX_RATE_C_PER_S (well above any
// real thermal slew — a healthy half-power Peltier moves a few C/min,
// ~0.1 C/s); hold the last good T_now instead. At the fixed 200 ms sample
// cadence this rejects steps larger than ~1 C. After TEMPCTRL_MAX_REJECTS
// consecutive rejects (~600 ms of sustained garbage) the channel latches
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

// Temperature control structure
typedef struct {
    uint dir_pin1;
    uint dir_pin2;
    uint pwm_pin;
    uint pwm_slice;
    TempSensor temp_sensor;
    float T_now;
    float T_target;
    float drive;
    float Kp;
    float Ki;
    float integral;       /* accumulated error (deg C * s) */
    uint32_t last_sample_ms;  /* timestamp of last PI tick; 0 = no prior sample */
    float hysteresis;
    float clamp;
    bool active;
    // `enabled` is host intent only — firmware never mutates it. A channel
    // drives only when enabled && none of the trip flags below are set; the
    // host distinguishes "asked off" from "blocked by trip" by reading
    // enabled together with the trip flags.
    bool enabled;
    // Module physically present (host config knob via LNA_installed /
    // LOAD_installed; firmware never mutates it). Distinct from `enabled`
    // (drive intent for present hardware) and `cooling_enabled` (drive-
    // polarity guard): an uninstalled channel is never sampled — its ADC
    // input is never mux-selected, so a dead divider cannot crosstalk
    // into the live channel — and never driven, and the picohost fan-out
    // suppresses its Redis stream entirely. Default true, so a reboot
    // behaves exactly as before this flag existed until the host replays
    // config. Not a trip ack: sticky latches survive uninstall and clear
    // only via *_enable=true.
    bool installed;
    // Per-cycle data validity (NOT a latch): true when this sample cycle
    // produced no measurement at all — the plausibility conversion failed
    // (railed divider) or the channel is not installed. Drives the
    // per-channel status string ("error") and the null T_now/resistance
    // reporting; clears by itself on the next plausible sample. Rate-guard
    // rejects do NOT set it: a rejected sample is still reported (see the
    // guard comment above TEMPCTRL_MAX_RATE_C_PER_S).
    bool data_invalid;
    // Asymmetric clamp: when false, the PI controller saturates at
    // [0, +clamp] instead of [-clamp, +clamp], forbidding cooling drive.
    // Default true preserves the original symmetric behavior. Deployments
    // that cannot dissipate Peltier heat (insufficient sink, hot enclosure)
    // should set this false on the affected channel to block the
    // cooling-mode thermal-runaway failure mode.
    bool cooling_enabled;
    // Stall guard: sticky fault tripped when an active drive fails to move
    // T_now over a full window. Runaway guard: sticky fault tripped when
    // T_now moves the wrong direction for TEMPCTRL_RUNAWAY_STRIKES
    // consecutive windows (mis-wired Peltier, lost hot-side dissipation,
    // swapped sensors) — kept as a separate flag because the field
    // diagnoses differ: a stall is "drive did nothing" (safe to retry), a
    // runaway is "drive made it worse" (check wiring before re-enabling).
    // Both cleared by an explicit *_enable=true command from the host
    // (mirrors the watchdog ack pattern).
    bool stall_tripped;
    bool runaway_tripped;
    bool stall_window_active;
    float stall_check_T;
    float stall_check_drive;
    absolute_time_t stall_check_time;
    // Consecutive wrong-direction windows seen by the runaway guard; reset
    // by a correct-direction (or no-) movement window, a trip, a disable,
    // or an enable ack.
    uint8_t runaway_strikes;
    // Sensor sanity guard state. rate_ref_ms advances on every fresh sample
    // (so the rate denominator is one sample); T_now itself is the value
    // reference (held on reject — the REPORTED value is the raw conversion
    // in temp_sensor.temperature). sensor_rejects counts consecutive
    // rejected samples and is published in status as the per-cycle
    // cross-check marker; when it reaches TEMPCTRL_MAX_REJECTS the channel
    // latches via the sticky sensor_tripped flag. sensor_tripped is cleared only by an
    // explicit *_enable=true host ack (like stall_tripped), so a sensor that
    // produced a burst of garbage cannot silently re-enable drive when a
    // later reading happens to fall back within the rate budget.
    //
    // Two-to-anchor seeding: until rate_ref_valid is set, the rate guard has no
    // reference to check against, so the reference is only trusted once two
    // consecutive samples agree within the rate budget. seed_pending marks that
    // a first (candidate) sample has been taken and is awaiting confirmation;
    // the candidate value lives in T_now and its timestamp in rate_ref_ms (both
    // unused for control while unanchored). Rationale: the guard judges every
    // future sample against its anchor, so a lone garbage sample must never
    // BECOME the anchor — it would make healthy samples look like impossible
    // jumps and false-latch the channel. The anchor is also only valid across
    // continuous good data: any plausibility-failed cycle drops it back to
    // unseeded, so recovery after a sensor outage re-anchors instead of
    // judging a legitimate temperature drift against a stale reference.
    bool rate_ref_valid;
    bool seed_pending;
    uint32_t rate_ref_ms;
    uint8_t sensor_rejects;
    bool sensor_tripped;
} TempControl;

// Standard app interface functions
void tempctrl_init(uint8_t app_id);
void tempctrl_server(uint8_t app_id, const char *json_str);
void tempctrl_op(uint8_t app_id);
void tempctrl_status(uint8_t app_id);

#endif // TEMPCTRL_H
