#include "tempctrl.h"
#include "temp_simple.h"
#include "pico/stdlib.h"
#include "cJSON.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Static instances
static LoadHeater load_heater;
static LnaThermistor lna1;
static LnaThermistor lna2;

// Communication watchdog state (app-level)
static absolute_time_t last_cmd_time;
static uint32_t watchdog_timeout_ms = 30000;  // default 30s, 0 = disabled
static bool watchdog_tripped = false;

// Fixed sensor-sampling timer. See TEMPCTRL_SAMPLE_MS in tempctrl.h for why
// the cadence is fixed.
static absolute_time_t next_sensor_sample;

// LOAD channel (thermistor + FET heater) forward declarations.
static void load_heater_init(LoadHeater *, uint fet_pin, uint temp_gpio_pin);
static void load_heater_update(LoadHeater *);
static void load_heater_apply_drive(LoadHeater *);
static void load_heater_control(LoadHeater *);
static void load_heater_check_stall(LoadHeater *);
static void load_heater_apply_enable(LoadHeater *, bool);
static bool load_heater_drive_allowed(const LoadHeater *);
static bool load_heater_read(LoadHeater *);
static bool thermistor_voltage_to_temperature(float voltage,
                                               float *resistance,
                                               float *temperature);

// LNA thermistor channels: read-only, no control logic.
static void lna_thermistor_init(LnaThermistor *, uint gpio_pin);
static void lna_thermistor_update(LnaThermistor *);

void tempctrl_init(uint8_t app_id) {
    load_heater_init(&load_heater, LOAD_FET_PIN, TEMP_SENSOR_LOAD_PIN);
    lna_thermistor_init(&lna1, LNA1_THERM_PIN);
    lna_thermistor_init(&lna2, LNA2_THERM_PIN);
    last_cmd_time = get_absolute_time();
    next_sensor_sample = get_absolute_time();  // first op tick samples
}

void tempctrl_server(uint8_t app_id, const char *json_str) {
    cJSON *item_json;
    cJSON *root = cJSON_Parse(json_str);
    if (!root || !cJSON_IsObject(root)) {
        cJSON_Delete(root);
        return;
    }

    // Any valid command refreshes the watchdog timer, but the trip flag
    // is sticky: a keepalive that arrives after the timeout already fired
    // must not silently re-engage the heater. The host clears
    // watchdog_tripped by explicitly sending LOAD_enable=true (see
    // load_heater_apply_enable), mirroring the stall-trip ack pattern.
    last_cmd_time = get_absolute_time();

    item_json = cJSON_GetObjectItem(root, "LOAD_temp_target");
    load_heater.T_target = item_json ? item_json->valuedouble : load_heater.T_target;
    item_json = cJSON_GetObjectItem(root, "LOAD_installed");
    if (item_json) load_heater.installed = item_json->valueint ? true : false;
    item_json = cJSON_GetObjectItem(root, "LOAD_enable");
    if (item_json) load_heater_apply_enable(&load_heater, item_json->valueint ? true : false);
    item_json = cJSON_GetObjectItem(root, "LOAD_hysteresis");
    load_heater.hysteresis = item_json ? item_json->valuedouble : load_heater.hysteresis;

    // Watchdog timeout configuration (0 = disabled)
    item_json = cJSON_GetObjectItem(root, "watchdog_timeout_ms");
    if (item_json && cJSON_IsNumber(item_json)) {
        int val = item_json->valueint;
        watchdog_timeout_ms = val < 0 ? 0 : (uint32_t)val;
    }

    cJSON_Delete(root);
}

void tempctrl_status(uint8_t app_id) {
    /* Status reports DATA EXISTENCE ONLY: "error" while the most recent
       sample cycle produced no measurement at all (plausibility failure or
       channel not installed — see data_invalid in tempctrl.h), in which
       case T_now/resistance are reported as JSON null (NaN KV_FLOAT) while
       voltage stays live for open-vs-short diagnosis. T_now reports the
       raw conversion (LoadHeater.T_raw), NOT the control reference: a
       rate-guard-rejected sample stays in the stream with the published
       sensor_rejects counter as the cross-check marker. The sticky control
       latches (sensor/stall/runaway_tripped, watchdog_tripped) gate drive
       but never set status. The picohost fan-out lifts this string to the
       stream's top-level status consumed by
       eigsep_observing._avg_sensor_values, which drops "error" rows — so
       an errored cycle deletes that row from the corr record, and only
       measurement absence may do that. */
    const char *status_load = load_heater.data_invalid ? "error" : "update";
    const float T_load = load_heater.data_invalid ? NAN : load_heater.T_raw;
    const float R_load =
        load_heater.data_invalid ? NAN : load_heater.resistance;
    /* LOAD_drive_level is a float for shape consistency with the rest of
       the status fields, but is binary: 1.0 while the FET is on, 0.0
       otherwise (there is no PWM duty to report). */
    const float load_drive_level = load_heater.active ? 1.0f : 0.0f;

    /* LNA1/LNA2 are read-only thermistor channels: no target, no drive,
       no trip latches — same "error" iff no measurement this cycle"
       status semantics as LOAD, just without any control state to
       report alongside it. */
    const char *status_lna1 = lna1.data_invalid ? "error" : "update";
    const float T_lna1 = lna1.data_invalid ? NAN : lna1.T_now;
    const float R_lna1 = lna1.data_invalid ? NAN : lna1.resistance;
    const char *status_lna2 = lna2.data_invalid ? "error" : "update";
    const float T_lna2 = lna2.data_invalid ? NAN : lna2.T_now;
    const float R_lna2 = lna2.data_invalid ? NAN : lna2.resistance;

    /* 29 KV pairs: 4 device-wide + 15 LOAD + 5 LNA1 + 5 LNA2. send_json
       silently truncates if the count argument disagrees with the
       actual entries — re-count when editing. */
    send_json(29,
        KV_STR, "sensor_name", "tempctrl",
        KV_INT, "app_id", app_id,
        KV_BOOL, "watchdog_tripped", watchdog_tripped,
        KV_INT, "watchdog_timeout_ms", (int)watchdog_timeout_ms,
        KV_STR, "LOAD_status", status_load,
        KV_FLOAT, "LOAD_T_now", T_load,
        KV_FLOAT, "LOAD_voltage", load_heater.voltage,
        KV_FLOAT, "LOAD_resistance", R_load,
        KV_FLOAT, "LOAD_timestamp", (double)load_heater.last_sample_time,
        KV_FLOAT, "LOAD_T_target", load_heater.T_target,
        KV_FLOAT, "LOAD_drive_level", load_drive_level,
        KV_BOOL, "LOAD_installed", load_heater.installed,
        KV_BOOL, "LOAD_enabled", load_heater.enabled,
        KV_BOOL, "LOAD_active", load_heater.active,
        KV_BOOL, "LOAD_sensor_tripped", load_heater.sensor_tripped,
        KV_INT, "LOAD_sensor_rejects", load_heater.sensor_rejects,
        KV_BOOL, "LOAD_stall_tripped", load_heater.stall_tripped,
        KV_BOOL, "LOAD_runaway_tripped", load_heater.runaway_tripped,
        KV_FLOAT, "LOAD_hysteresis", load_heater.hysteresis,
        KV_STR, "LNA1_status", status_lna1,
        KV_FLOAT, "LNA1_T_now", T_lna1,
        KV_FLOAT, "LNA1_voltage", lna1.voltage,
        KV_FLOAT, "LNA1_resistance", R_lna1,
        KV_FLOAT, "LNA1_timestamp", (double)lna1.last_sample_time,
        KV_STR, "LNA2_status", status_lna2,
        KV_FLOAT, "LNA2_T_now", T_lna2,
        KV_FLOAT, "LNA2_voltage", lna2.voltage,
        KV_FLOAT, "LNA2_resistance", R_lna2,
        KV_FLOAT, "LNA2_timestamp", (double)lna2.last_sample_time
    );
}

void tempctrl_op(uint8_t app_id) {
    // Communication watchdog: trip the flag if no command has arrived
    // within the timeout. The flag is the runtime gate — `enabled` is host
    // intent and stays untouched, so the host can see exactly whether it
    // had asked for drive vs. whether it's blocked by the trip.
    if (watchdog_timeout_ms > 0 && !watchdog_tripped) {
        int64_t elapsed_us = absolute_time_diff_us(last_cmd_time, get_absolute_time());
        if (elapsed_us > (int64_t)watchdog_timeout_ms * 1000) {
            watchdog_tripped = true;
        }
    }

    // Sampling, the rate guard, and the on/off control step run on the
    // fixed TEMPCTRL_SAMPLE_MS timer; between ticks op is a no-op (beyond
    // the watchdog check above) and the FET GPIO holds its level.
    if (!time_reached(next_sensor_sample)) {
        return;
    }
    next_sensor_sample = make_timeout_time_ms(TEMPCTRL_SAMPLE_MS);

    load_heater_update(&load_heater);
    lna_thermistor_update(&lna1);
    lna_thermistor_update(&lna2);
}

// ---------------------------------------------------------------------
// Shared thermistor conversion: 30k NTC, Beta=3943, plain 10k pull-up to
// 3V3 — used by LOAD and both LNA channels alike (see LOAD_THERM_* in
// tempctrl.h; update those constants if an LNA channel ever uses a
// different physical part). Reuses the shared ADC sampling helpers from
// temp_simple.c (adc_channel_init / adc_read_avg_voltage — generic, no
// thermistor-specific constants).
// ---------------------------------------------------------------------

static bool thermistor_voltage_to_temperature(float voltage,
                                               float *resistance,
                                               float *temperature) {
    if (!isfinite(voltage) ||
        voltage <= 0.0f ||
        voltage >= THERMISTOR_SUPPLY_VOLTS) {
        return false;
    }

    float r_thermistor = LOAD_THERM_PULLUP_OHMS
        * voltage / (THERMISTOR_SUPPLY_VOLTS - voltage);
    if (!isfinite(r_thermistor) || r_thermistor <= 0.0f) {
        return false;
    }

    float inverse_kelvin = (1.0f / LOAD_THERM_T0_KELVIN)
        + (1.0f / LOAD_THERM_BETA_KELVIN)
          * logf(r_thermistor / LOAD_THERM_R0_OHMS);
    if (!isfinite(inverse_kelvin) || inverse_kelvin <= 0.0f) {
        return false;
    }

    float temp_c = 1.0f / inverse_kelvin - 273.15f;
    if (!isfinite(temp_c) || temp_c < -40.0f || temp_c > 150.0f) {
        return false;
    }

    *resistance = r_thermistor;
    *temperature = temp_c;
    return true;
}

static void load_heater_init(LoadHeater *lh, uint fet_pin, uint temp_gpio_pin) {
    gpio_init(fet_pin);
    gpio_set_dir(fet_pin, GPIO_OUT);
    gpio_put(fet_pin, 0);  // FET off at boot

    lh->fet_pin = fet_pin;
    lh->adc_configured = adc_channel_init(temp_gpio_pin, &lh->adc_input);

    lh->T_raw = 0.0f;
    lh->T_now = 0.0f;
    lh->voltage = 0.0f;
    lh->resistance = 0.0f;
    lh->T_target = 30.0;
    lh->hysteresis = LOAD_DEFAULT_HYSTERESIS_C;
    lh->last_sample_time = 0;

    lh->active = false;
    lh->enabled = false;
    lh->installed = true;
    // No sample taken yet, so there is no valid temperature to report;
    // the first op tick samples before the first status message fires.
    lh->data_invalid = true;

    lh->rate_ref_valid = false;
    lh->seed_pending = false;
    lh->rate_ref_ms = 0;
    lh->sensor_rejects = 0;
    lh->sensor_tripped = false;

    lh->stall_window_active = false;
    lh->stall_check_T = 0.0f;
    lh->stall_check_time = get_absolute_time();
    lh->runaway_strikes = 0;
    lh->stall_tripped = false;
    lh->runaway_tripped = false;
}

static void load_heater_apply_drive(LoadHeater *lh) {
    gpio_put(lh->fet_pin, lh->active);
}

// On/off hysteresis: turn on once T_now drops `hysteresis` below target,
// turn off once T_now reaches target. No deadband-freeze step is needed
// (there is no integrator), and no clamp/gain — the FET is either fully
// on or fully off.
static void load_heater_control(LoadHeater *lh) {
    if (lh->active) {
        if (lh->T_now >= lh->T_target) {
            lh->active = false;
        }
    } else {
        if (lh->T_now <= (lh->T_target - lh->hysteresis)) {
            lh->active = true;
        }
    }
    load_heater_apply_drive(lh);
}

static void load_heater_check_stall(LoadHeater *lh) {
    if (!lh->active) {
        lh->stall_window_active = false;
        lh->runaway_strikes = 0;
        return;
    }
    absolute_time_t now = get_absolute_time();
    if (!lh->stall_window_active) {
        lh->stall_check_T = lh->T_now;
        lh->stall_check_time = now;
        lh->stall_window_active = true;
        return;
    }
    int64_t elapsed_us = absolute_time_diff_us(lh->stall_check_time, now);
    if (elapsed_us < (int64_t)TEMPCTRL_STALL_WINDOW_MS * 1000) {
        return;
    }
    float delta = lh->T_now - lh->stall_check_T;
    if (delta < -TEMPCTRL_STALL_MIN_DELTA) {
        // Heating engaged but temperature FELL — runaway signature
        // (mis-wired FET/sensor, or the sensor reads a different thermal
        // mass than the heater). Score a strike; trip only after
        // TEMPCTRL_RUNAWAY_STRIKES consecutive windows so a brief
        // startup/soak transient cannot false-trip a channel that is
        // actually controlling.
        lh->runaway_strikes++;
        if (lh->runaway_strikes >= TEMPCTRL_RUNAWAY_STRIKES) {
            lh->runaway_tripped = true;
            lh->active = false;
            load_heater_apply_drive(lh);
            lh->stall_window_active = false;
            lh->runaway_strikes = 0;
            return;
        }
        lh->stall_check_T = lh->T_now;
        lh->stall_check_time = now;
        return;
    }
    lh->runaway_strikes = 0;
    if (delta < TEMPCTRL_STALL_MIN_DELTA) {
        // Heating engaged for a full window with no meaningful rise —
        // heater ineffective or sensor stuck.
        lh->stall_tripped = true;
        lh->active = false;
        load_heater_apply_drive(lh);
        lh->stall_window_active = false;
        return;
    }
    // Healthy progress; roll the window forward.
    lh->stall_check_T = lh->T_now;
    lh->stall_check_time = now;
}

static void load_heater_apply_enable(LoadHeater *lh, bool new_enabled) {
    // LOAD_enable=true is the host's explicit ack of sticky trips: it
    // clears this channel's trip flags and the app-wide watchdog flag.
    // `enabled` itself is pure host intent — firmware never mutates it.
    if (new_enabled) {
        lh->stall_tripped = false;
        lh->runaway_tripped = false;
        lh->stall_window_active = false;
        lh->runaway_strikes = 0;
        lh->sensor_rejects = 0;
        lh->sensor_tripped = false;
        lh->rate_ref_valid = false;
        lh->seed_pending = false;
        watchdog_tripped = false;
    }
    lh->enabled = new_enabled;
}

static bool load_heater_drive_allowed(const LoadHeater *lh) {
    // installed is defense in depth here: the !installed early return in
    // load_heater_update already forces drive off before the control
    // path runs.
    return lh->installed
        && lh->enabled
        && !lh->sensor_tripped
        && !lh->stall_tripped
        && !lh->runaway_tripped
        && !watchdog_tripped;
}

// Sample the thermistor. The measured voltage is stored unconditionally
// (a railed/implausible voltage — near supply for open, near 0 for short
// — is exactly the field diagnostic), only resistance/T_raw/
// last_sample_time update on a plausible conversion.
static bool load_heater_read(LoadHeater *lh) {
    float voltage = adc_read_avg_voltage(lh->adc_input);
    lh->voltage = voltage;

    float resistance = 0.0f;
    float temperature = 0.0f;
    if (!thermistor_voltage_to_temperature(voltage, &resistance,
                                            &temperature)) {
        return false;
    }

    lh->resistance = resistance;
    lh->T_raw = temperature;
    lh->last_sample_time = to_ms_since_boot(get_absolute_time());
    return true;
}

// One sample tick: read + rate guard (two-to-anchor) on the fixed
// TEMPCTRL_SAMPLE_MS timer, then on/off control + stall/runaway check on
// the filtered T_now.
static void load_heater_update(LoadHeater *lh) {
    if (!lh->adc_configured || !lh->installed) {
        lh->data_invalid = true;
        lh->rate_ref_valid = false;
        lh->seed_pending = false;
        lh->active = false;
        load_heater_apply_drive(lh);
        lh->stall_window_active = false;
        lh->runaway_strikes = 0;
        return;
    }

    bool plausible = load_heater_read(lh);
    bool rejected = false;

    if (!plausible) {
        lh->rate_ref_valid = false;
        lh->seed_pending = false;
    } else {
        float raw = lh->T_raw;
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (!lh->rate_ref_valid) {
            if (!lh->seed_pending) {
                lh->T_now = raw;
                lh->seed_pending = true;
                lh->sensor_rejects = 0;
            } else {
                float dt = (float)(now_ms - lh->rate_ref_ms) / 1000.0f;
                if (dt > 0.0f &&
                    fabsf(raw - lh->T_now) / dt > TEMPCTRL_MAX_RATE_C_PER_S) {
                    lh->T_now = raw;
                    if (lh->sensor_rejects < TEMPCTRL_MAX_REJECTS) {
                        lh->sensor_rejects++;
                    }
                } else {
                    lh->T_now = raw;
                    lh->rate_ref_valid = true;
                    lh->seed_pending = false;
                    lh->sensor_rejects = 0;
                }
            }
        } else {
            float dt = (float)(now_ms - lh->rate_ref_ms) / 1000.0f;
            if (dt > 0.0f &&
                fabsf(raw - lh->T_now) / dt > TEMPCTRL_MAX_RATE_C_PER_S) {
                rejected = true;
                if (lh->sensor_rejects < TEMPCTRL_MAX_REJECTS) {
                    lh->sensor_rejects++;
                }
            } else {
                lh->T_now = raw;
                lh->sensor_rejects = 0;
            }
        }
        lh->rate_ref_ms = now_ms;
    }

    if (lh->sensor_rejects >= TEMPCTRL_MAX_REJECTS) {
        if (!lh->sensor_tripped) {
            lh->rate_ref_valid = false;
            lh->seed_pending = false;
        }
        lh->sensor_tripped = true;
    }
    lh->data_invalid = !plausible;

    // Absolute safety ceiling, independent of target/hysteresis/rate-guard
    // state: a plausible, anchored reading at or above the hard limit
    // forces off and latches runaway (the control loop failed to prevent
    // overheating) regardless of whatever target the host configured.
    if (plausible && lh->rate_ref_valid && lh->T_now >= LOAD_MAX_SAFE_TEMP_C) {
        lh->runaway_tripped = true;
    }

    if (load_heater_drive_allowed(lh) && lh->rate_ref_valid && plausible) {
        if (!rejected) {
            load_heater_control(lh);
            load_heater_check_stall(lh);
        }
        /* Lone reject: nothing new to act on. Hold the previous drive
           state (GPIO keeps it) and the stall window; the next accepted
           sample resumes control. */
    } else {
        lh->active = false;
        load_heater_apply_drive(lh);
        lh->stall_window_active = false;
        lh->runaway_strikes = 0;
    }
}

// ---------------------------------------------------------------------
// LNA thermistor channels: read-only telemetry, no control logic. No
// rate guard / stall / runaway checks — those exist to protect a heater
// drive loop, which these channels don't have.
// ---------------------------------------------------------------------

static void lna_thermistor_init(LnaThermistor *lt, uint gpio_pin) {
    lt->adc_configured = adc_channel_init(gpio_pin, &lt->adc_input);
    lt->T_now = 0.0f;
    lt->voltage = 0.0f;
    lt->resistance = 0.0f;
    // No sample taken yet, so there is no valid temperature to report;
    // the first op tick samples before the first status message fires.
    lt->data_invalid = true;
    lt->last_sample_time = 0;
}

// One sample tick: read the thermistor and report the plausible
// conversion directly (no rate-guard/anchor scheme — nothing downstream
// depends on a filtered reference the way LOAD's control loop does).
static void lna_thermistor_update(LnaThermistor *lt) {
    if (!lt->adc_configured) {
        lt->data_invalid = true;
        return;
    }

    float voltage = adc_read_avg_voltage(lt->adc_input);
    lt->voltage = voltage;

    float resistance = 0.0f;
    float temperature = 0.0f;
    if (!thermistor_voltage_to_temperature(voltage, &resistance,
                                            &temperature)) {
        lt->data_invalid = true;
        return;
    }

    lt->resistance = resistance;
    lt->T_now = temperature;
    lt->last_sample_time = to_ms_since_boot(get_absolute_time());
    lt->data_invalid = false;
}
