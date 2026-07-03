#include "tempctrl.h"
#include "temp_simple.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "cJSON.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Static instances
static TempControl tempctrl_lna;
static TempControl tempctrl_load;

// Communication watchdog state (app-level, not per-channel)
static absolute_time_t last_cmd_time;
static uint32_t watchdog_timeout_ms = 30000;  // default 30s, 0 = disabled
static bool watchdog_tripped = false;

// Fixed sensor-sampling timer (app-level; both channels sample on the same
// tick). See TEMPCTRL_SAMPLE_MS in tempctrl.h for why the cadence is fixed.
static absolute_time_t next_sensor_sample;

// Forward declarations
static void init_single_tempctrl(TempControl *, uint, uint, uint, pwm_config *, uint);
static void tempctrl_update_sensor_drive(TempControl *);
static void tempctrl_apply_drive(TempControl *);
static void tempctrl_check_stall(TempControl *);
static void tempctrl_apply_enable(TempControl *, bool);
static bool tempctrl_drive_allowed(const TempControl *);
static void tempctrl_pi_drive(TempControl *);
static void tempctrl_reset_controller_state(TempControl *);

static void init_single_tempctrl(TempControl *tempctrl,
                                 uint dir_pin1, uint dir_pin2, uint pwm_pin,
                                 pwm_config *config, uint temp_sensor_pin) {
    // Initialize GPIO for Peltier
    gpio_init(dir_pin1);
    gpio_set_dir(dir_pin1, GPIO_OUT);
    gpio_init(dir_pin2);
    gpio_set_dir(dir_pin2, GPIO_OUT);
    // Set up PWM for Peltier
    gpio_set_function(pwm_pin, GPIO_FUNC_PWM);
    tempctrl->pwm_slice = pwm_gpio_to_slice_num(pwm_pin);
    pwm_init(tempctrl->pwm_slice, config, true);

    temp_sensor_init(&tempctrl->temp_sensor, temp_sensor_pin);

    // Initialize Temperature Control structure. Ki defaults to 0 so an
    // un-tuned deployment behaves as pure P + deadband (no integral
    // creep until the host opts in via LNA_Ki / LOAD_Ki).
    tempctrl->dir_pin1 = dir_pin1;
    tempctrl->dir_pin2 = dir_pin2;
    tempctrl->pwm_pin = pwm_pin;
    tempctrl->T_target = 30.0;
    tempctrl->Kp = 0.2;
    tempctrl->Ki = 0.0;
    tempctrl->integral = 0.0;
    tempctrl->last_sample_ms = 0;
    tempctrl->clamp = 0.2;  // Maximum drive level; low default keeps Peltier current manageable
    tempctrl->hysteresis = 0.5;
    tempctrl->enabled = false;
    tempctrl->active = false;
    // No sample taken yet, so there is no valid temperature to report;
    // the first op tick samples before the first status message fires.
    tempctrl->data_invalid = true;
    tempctrl->cooling_enabled = true;
    tempctrl->T_now = 0;
    tempctrl->drive = 0.0;
    tempctrl->stall_tripped = false;
    tempctrl->runaway_tripped = false;
    tempctrl->stall_window_active = false;
    tempctrl->stall_check_T = 0.0;
    tempctrl->stall_check_time = get_absolute_time();
    tempctrl->runaway_strikes = 0;
    tempctrl->rate_ref_valid = false;
    tempctrl->seed_pending = false;
    tempctrl->rate_ref_ms = 0;
    tempctrl->sensor_rejects = 0;
    tempctrl->sensor_tripped = false;
}

void tempctrl_init(uint8_t app_id) {
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 145.0f);         // PWM frequency = System_Clock / (Clock_Divider × (WRAP + 1)), system_clock = 150 MHz default
    pwm_config_set_wrap(&config, PWM_WRAP);
    init_single_tempctrl(&tempctrl_lna, PELTIER_LNA_DIR_PIN1, PELTIER_LNA_DIR_PIN2,
            PELTIER_LNA_PWM_PIN, &config, TEMP_SENSOR_LNA_PIN);
    init_single_tempctrl(&tempctrl_load, PELTIER_LOAD_DIR_PIN3, PELTIER_LOAD_DIR_PIN4,
            PELTIER_LOAD_PWM_PIN, &config, TEMP_SENSOR_LOAD_PIN);
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
    // must not silently re-engage the peltiers. The host clears
    // watchdog_tripped by explicitly sending *_enable=true (see
    // tempctrl_apply_enable), mirroring the stall-trip ack pattern.
    last_cmd_time = get_absolute_time();

    // Parse channel selection (default to both)
    item_json = cJSON_GetObjectItem(root, "LNA_temp_target");
    tempctrl_lna.T_target = item_json ? item_json->valuedouble : tempctrl_lna.T_target;
    item_json = cJSON_GetObjectItem(root, "LNA_enable");
    if (item_json) tempctrl_apply_enable(&tempctrl_lna, item_json->valueint ? true : false);
    item_json = cJSON_GetObjectItem(root, "LNA_hysteresis");
    tempctrl_lna.hysteresis = item_json ? item_json->valuedouble : tempctrl_lna.hysteresis;
    item_json = cJSON_GetObjectItem(root, "LNA_clamp");
    if (item_json) tempctrl_lna.clamp = fminf(1.0, fmaxf(0.0, item_json->valuedouble));
    item_json = cJSON_GetObjectItem(root, "LNA_Kp");
    if (item_json) tempctrl_lna.Kp = item_json->valuedouble;
    item_json = cJSON_GetObjectItem(root, "LNA_Ki");
    if (item_json) {
        float new_ki = (float)item_json->valuedouble;
        if (new_ki != tempctrl_lna.Ki) {
            /* Bumpless retune: drop the accumulator so the next PI step
               does not multiply a stale integral by a freshly-changed
               gain. */
            tempctrl_lna.integral = 0.0f;
            tempctrl_lna.last_sample_ms = 0;
        }
        tempctrl_lna.Ki = new_ki;
    }
    item_json = cJSON_GetObjectItem(root, "LNA_integral_reset");
    if (item_json && item_json->valueint) {
        tempctrl_lna.integral = 0.0f;
        tempctrl_lna.last_sample_ms = 0;
    }
    item_json = cJSON_GetObjectItem(root, "LNA_cooling_enabled");
    if (item_json) tempctrl_lna.cooling_enabled = item_json->valueint ? true : false;
    item_json = cJSON_GetObjectItem(root, "LOAD_temp_target");
    tempctrl_load.T_target = item_json ? item_json->valuedouble : tempctrl_load.T_target;
    item_json = cJSON_GetObjectItem(root, "LOAD_enable");
    if (item_json) tempctrl_apply_enable(&tempctrl_load, item_json->valueint ? true : false);
    item_json = cJSON_GetObjectItem(root, "LOAD_hysteresis");
    tempctrl_load.hysteresis = item_json ? item_json->valuedouble : tempctrl_load.hysteresis;
    item_json = cJSON_GetObjectItem(root, "LOAD_clamp");
    if (item_json) tempctrl_load.clamp = fminf(1.0, fmaxf(0.0, item_json->valuedouble));
    item_json = cJSON_GetObjectItem(root, "LOAD_Kp");
    if (item_json) tempctrl_load.Kp = item_json->valuedouble;
    item_json = cJSON_GetObjectItem(root, "LOAD_Ki");
    if (item_json) {
        float new_ki = (float)item_json->valuedouble;
        if (new_ki != tempctrl_load.Ki) {
            tempctrl_load.integral = 0.0f;
            tempctrl_load.last_sample_ms = 0;
        }
        tempctrl_load.Ki = new_ki;
    }
    item_json = cJSON_GetObjectItem(root, "LOAD_integral_reset");
    if (item_json && item_json->valueint) {
        tempctrl_load.integral = 0.0f;
        tempctrl_load.last_sample_ms = 0;
    }
    item_json = cJSON_GetObjectItem(root, "LOAD_cooling_enabled");
    if (item_json) tempctrl_load.cooling_enabled = item_json->valueint ? true : false;

    // Watchdog timeout configuration (0 = disabled)
    item_json = cJSON_GetObjectItem(root, "watchdog_timeout_ms");
    if (item_json && cJSON_IsNumber(item_json)) {
        int val = item_json->valueint;
        watchdog_timeout_ms = val < 0 ? 0 : (uint32_t)val;
    }

    cJSON_Delete(root);
}

void tempctrl_status(uint8_t app_id) {
    const uint32_t time_lna = temp_sensor_get_sample_time(&tempctrl_lna.temp_sensor);
    const uint32_t time_load = temp_sensor_get_sample_time(&tempctrl_load.temp_sensor);

    /* Per-channel status reports DATA VALIDITY ONLY: "error" while the most
       recent sample cycle produced no trustworthy temperature (plausibility
       failure or rate-guard reject — see data_invalid in tempctrl.h), in
       which case T_now/resistance are reported as JSON null (NaN KV_FLOAT)
       while voltage stays live for open-vs-short diagnosis. The sticky
       control latches (sensor/stall/runaway_tripped, watchdog_tripped) gate
       drive but never set status: a latched channel with a recovered sensor
       keeps publishing valid science data. The picohost fan-out lifts these
       strings to the per-stream top-level status consumed by
       eigsep_observing._avg_sensor_values. */
    const char *status_lna = tempctrl_lna.data_invalid ? "error" : "update";
    const char *status_load = tempctrl_load.data_invalid ? "error" : "update";
    const float T_lna = tempctrl_lna.data_invalid ? NAN : tempctrl_lna.T_now;
    const float T_load = tempctrl_load.data_invalid ? NAN : tempctrl_load.T_now;
    const float R_lna =
        tempctrl_lna.data_invalid ? NAN : tempctrl_lna.temp_sensor.resistance;
    const float R_load =
        tempctrl_load.data_invalid ? NAN : tempctrl_load.temp_sensor.resistance;

    /* 40 KV pairs: 4 device-wide + 18 per channel * 2 channels. send_json
       silently truncates if the count argument disagrees with the actual
       entries — re-count when editing. */
    send_json(40,
        KV_STR, "sensor_name", "tempctrl",
        KV_INT, "app_id", app_id,
        KV_BOOL, "watchdog_tripped", watchdog_tripped,
        KV_INT, "watchdog_timeout_ms", (int)watchdog_timeout_ms,
        KV_STR, "LNA_status", status_lna,
        KV_FLOAT, "LNA_T_now", T_lna,
        KV_FLOAT, "LNA_voltage", tempctrl_lna.temp_sensor.voltage,
        KV_FLOAT, "LNA_resistance", R_lna,
        KV_FLOAT, "LNA_timestamp", (double)time_lna,
        KV_FLOAT, "LNA_T_target", tempctrl_lna.T_target,
        KV_FLOAT, "LNA_drive_level", tempctrl_lna.drive,
        KV_BOOL, "LNA_enabled", tempctrl_lna.enabled,
        KV_BOOL, "LNA_active", tempctrl_lna.active,
        KV_BOOL, "LNA_sensor_tripped", tempctrl_lna.sensor_tripped,
        KV_BOOL, "LNA_stall_tripped", tempctrl_lna.stall_tripped,
        KV_BOOL, "LNA_runaway_tripped", tempctrl_lna.runaway_tripped,
        KV_BOOL, "LNA_cooling_enabled", tempctrl_lna.cooling_enabled,
        KV_FLOAT, "LNA_hysteresis", tempctrl_lna.hysteresis,
        KV_FLOAT, "LNA_clamp", tempctrl_lna.clamp,
        KV_FLOAT, "LNA_Kp", tempctrl_lna.Kp,
        KV_FLOAT, "LNA_Ki", tempctrl_lna.Ki,
        KV_FLOAT, "LNA_integral", tempctrl_lna.integral,
        KV_STR, "LOAD_status", status_load,
        KV_FLOAT, "LOAD_T_now", T_load,
        KV_FLOAT, "LOAD_voltage", tempctrl_load.temp_sensor.voltage,
        KV_FLOAT, "LOAD_resistance", R_load,
        KV_FLOAT, "LOAD_timestamp", (double)time_load,
        KV_FLOAT, "LOAD_T_target", tempctrl_load.T_target,
        KV_FLOAT, "LOAD_drive_level", tempctrl_load.drive,
        KV_BOOL, "LOAD_enabled", tempctrl_load.enabled,
        KV_BOOL, "LOAD_active", tempctrl_load.active,
        KV_BOOL, "LOAD_sensor_tripped", tempctrl_load.sensor_tripped,
        KV_BOOL, "LOAD_stall_tripped", tempctrl_load.stall_tripped,
        KV_BOOL, "LOAD_runaway_tripped", tempctrl_load.runaway_tripped,
        KV_BOOL, "LOAD_cooling_enabled", tempctrl_load.cooling_enabled,
        KV_FLOAT, "LOAD_hysteresis", tempctrl_load.hysteresis,
        KV_FLOAT, "LOAD_clamp", tempctrl_load.clamp,
        KV_FLOAT, "LOAD_Kp", tempctrl_load.Kp,
        KV_FLOAT, "LOAD_Ki", tempctrl_load.Ki,
        KV_FLOAT, "LOAD_integral", tempctrl_load.integral
    );
}

void tempctrl_update_sensor_drive(TempControl *tempctrl) {
    // Sample the thermistor. Called on the fixed TEMPCTRL_SAMPLE_MS timer
    // (not every op pass), so the rate guard's dt is ~one sample period.
    // `plausible` is false when the voltage->temperature conversion failed
    // (railed divider: open/short thermistor); the measured voltage is
    // still stored for open-vs-short diagnosis in status.
    bool plausible = temp_sensor_read(&tempctrl->temp_sensor);
    bool rejected = false;

    if (!plausible) {
        // No trustworthy sample this cycle. Drop the rate anchor: it is
        // only valid across continuous good data, so recovery after an
        // outage must re-seed (two-to-anchor) rather than judge a
        // legitimate temperature drift against a stale reference, which
        // would false-latch the channel on healthy data.
        tempctrl->rate_ref_valid = false;
        tempctrl->seed_pending = false;
    } else {
    // Sensor sanity guard: reject a plausible-looking sample whose implied
    // rate of change is physically impossible (multiplexed-ADC crosstalk
    // garbage) and hold the last good T_now; latch the channel after
    // TEMPCTRL_MAX_REJECTS consecutive rejects. rate_ref_ms advances on
    // every plausible sample so the rate denominator stays ~one sample;
    // T_now is the value reference.
    //
    // Before the reference is anchored there is nothing to rate-check
    // against, so the first sample is only a candidate (held in T_now,
    // timestamp in rate_ref_ms): it becomes the trusted reference once a
    // second sample confirms it within the rate budget. A candidate that
    // fails confirmation is replaced and counts toward the same latch
    // ceiling, so a sensor that can never produce two consistent readings
    // still latches instead of seeding control from a lone transient.
    // Candidate samples are reported as valid data — they passed the
    // plausibility check — while control stays gated until the anchor
    // confirms.
        float raw = temp_sensor_get_temp(&tempctrl->temp_sensor);
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (!tempctrl->rate_ref_valid) {
            if (!tempctrl->seed_pending) {
                // First sample: hold as candidate, await confirmation.
                tempctrl->T_now = raw;
                tempctrl->seed_pending = true;
                tempctrl->sensor_rejects = 0;
            } else {
                float dt = (float)(now_ms - tempctrl->rate_ref_ms) / 1000.0f;
                if (dt > 0.0f &&
                    fabsf(raw - tempctrl->T_now) / dt > TEMPCTRL_MAX_RATE_C_PER_S) {
                    // Candidate unconfirmed: replace it, count toward the latch.
                    tempctrl->T_now = raw;
                    if (tempctrl->sensor_rejects < TEMPCTRL_MAX_REJECTS) {
                        tempctrl->sensor_rejects++;
                    }
                } else {
                    // Two consecutive consistent samples: anchor the reference.
                    tempctrl->T_now = raw;
                    tempctrl->rate_ref_valid = true;
                    tempctrl->seed_pending = false;
                    tempctrl->sensor_rejects = 0;
                }
            }
        } else {
            float dt = (float)(now_ms - tempctrl->rate_ref_ms) / 1000.0f;
            if (dt > 0.0f &&
                fabsf(raw - tempctrl->T_now) / dt > TEMPCTRL_MAX_RATE_C_PER_S) {
                // Reject: hold T_now, count toward the latch ceiling.
                rejected = true;
                if (tempctrl->sensor_rejects < TEMPCTRL_MAX_REJECTS) {
                    tempctrl->sensor_rejects++;
                }
            } else {
                tempctrl->T_now = raw;
                tempctrl->sensor_rejects = 0;
            }
        }
        tempctrl->rate_ref_ms = now_ms;
    }

    // The rate-sanity latch is sticky: once sensor_rejects reaches the
    // ceiling, sensor_tripped stays set until the host acks with
    // *_enable=true (see tempctrl_apply_enable). A later plausible sample
    // resets sensor_rejects but must not re-enable a channel whose sensor
    // just produced a burst of garbage.
    if (tempctrl->sensor_rejects >= TEMPCTRL_MAX_REJECTS) {
        tempctrl->sensor_tripped = true;
    }
    // Data validity for this cycle only (feeds the per-channel status
    // string and the null T_now/resistance reporting). A latched channel
    // whose samples are plausible and rate-consistent again reports valid
    // data — only drive stays gated.
    tempctrl->data_invalid = !plausible || rejected;

    // rate_ref_valid gates control as well as the guard: until the reference
    // is anchored T_now is only a candidate, so the channel stays idle (the
    // not-allowed branch holds drive at 0) rather than driving on an
    // unconfirmed reading. A plausibility-failed cycle also gates — the
    // sensor may be gone entirely, so drive must not keep running on a
    // frozen T_now.
    if (tempctrl_drive_allowed(tempctrl) && tempctrl->rate_ref_valid
            && plausible) {
        if (!rejected) {
            tempctrl_pi_drive(tempctrl);
            tempctrl_check_stall(tempctrl);
        }
        /* Lone reject: the sample was discarded, so there is nothing new
           to act on. Hold the previous drive (PWM hardware keeps it) and
           the stall window; the next accepted sample resumes control. */
    } else {
        tempctrl_reset_controller_state(tempctrl);
        tempctrl_apply_drive(tempctrl);
        // Not actively driving — no stall window pending, and any
        // partial runaway strike count is stale once drive is gated
        // (disable/error/watchdog). Clearing it here keeps the next
        // wrong-direction window from tripping early and matches the
        // emulator's not-drive-allowed branch.
        tempctrl->stall_window_active = false;
        tempctrl->runaway_strikes = 0;
    }
}

void tempctrl_op(uint8_t app_id) {
    // Communication watchdog: trip the (app-wide) flag if no command has
    // arrived within the timeout. The flag is the runtime gate — `enabled`
    // is host intent and stays untouched, so the host can see exactly which
    // channels it had asked for vs. which are blocked by the trip.
    if (watchdog_timeout_ms > 0 && !watchdog_tripped) {
        int64_t elapsed_us = absolute_time_diff_us(last_cmd_time, get_absolute_time());
        if (elapsed_us > (int64_t)watchdog_timeout_ms * 1000) {
            watchdog_tripped = true;
        }
    }

    // Sampling, the rate guard, and the PI step run on the fixed
    // TEMPCTRL_SAMPLE_MS timer; between ticks op is a no-op (beyond the
    // watchdog check above) and the PWM hardware holds the drive level.
    if (!time_reached(next_sensor_sample)) {
        return;
    }
    next_sensor_sample = make_timeout_time_ms(TEMPCTRL_SAMPLE_MS);

    tempctrl_update_sensor_drive(&tempctrl_lna);
    tempctrl_update_sensor_drive(&tempctrl_load);
}

// Helper functions
static void tempctrl_apply_drive(TempControl *tempctrl) {
    uint32_t pwm_level = (uint32_t)(fabsf(tempctrl->drive) * PWM_WRAP);
    bool forward = (tempctrl->drive >= 0);

    if (tempctrl->drive == 0.0f) {
        /* tri-state / brake-off */
        gpio_put(tempctrl->dir_pin1, 0);
        gpio_put(tempctrl->dir_pin2, 0);
        pwm_set_gpio_level(tempctrl->pwm_pin, 0);
    } else {
        gpio_put(tempctrl->dir_pin1, forward);
        gpio_put(tempctrl->dir_pin2, !forward);
        pwm_set_gpio_level(tempctrl->pwm_pin, pwm_level);
    }
}

/* Clear the integrator and "first sample" sentinel. Called when the
   channel is disabled, sensor-errored, or inside the hysteresis
   deadband — any case where the next active PI step must not see stale
   accumulator state. */
static void tempctrl_reset_controller_state(TempControl *tc) {
    tc->drive = 0.0f;
    tc->integral = 0.0f;
    tc->last_sample_ms = 0;
    tc->active = false;
}

static void tempctrl_pi_drive(TempControl *tc) {
    float T_delta = tc->T_target - tc->T_now;

    if (fabsf(T_delta) <= tc->hysteresis) {
        /* Inside deadband: zero drive AND freeze the integrator. This
           preserves the existing anti-chatter design and prevents the
           integrator from winding up at setpoint. */
        tempctrl_reset_controller_state(tc);
        tempctrl_apply_drive(tc);
        return;
    }

    tc->active = true;

    /* dt from real elapsed time since last PI tick. First sample after a
       reset uses dt=0 so the integrator does not jump. */
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    float dt = 0.0f;
    if (tc->last_sample_ms != 0) {
        dt = (float)(now_ms - tc->last_sample_ms) / 1000.0f;
    }
    tc->last_sample_ms = now_ms;

    float p_term = tc->Kp * T_delta;
    /* Pure-P (Ki==0): freeze the integrator. Ki*integral is zero either
       way, but skipping the accumulation keeps `*_integral` clean over
       long sessions. Bumpless transfer on a later Ki retune is enforced
       in tempctrl_server by resetting the integral when Ki changes. */
    float tentative_i = (tc->Ki == 0.0f) ? tc->integral
                                         : (tc->integral + T_delta * dt);
    float tentative_drive = p_term + tc->Ki * tentative_i;

    /* Asymmetric clamp: cooling_enabled=false forbids negative drive
       (the cooling-mode thermal-runaway guard). The lower bound is the
       saturation floor used by both anti-windup and the final clamp. */
    float lower_clamp = tc->cooling_enabled ? -tc->clamp : 0.0f;

    /* Anti-windup: only commit the new integral if it would not push
       further into the saturation we're already against. With cooling
       disabled and T_delta<0 (we want to cool but can't), tentative_drive
       is negative — sat_low fires against lower_clamp=0, freezing the
       integrator instead of letting it wind up. */
    bool sat_high = (tentative_drive >  tc->clamp)  && (T_delta > 0);
    bool sat_low  = (tentative_drive <  lower_clamp) && (T_delta < 0);

    if (sat_high) {
        tc->drive = tc->clamp;
    } else if (sat_low) {
        tc->drive = lower_clamp;
    } else {
        tc->integral = tentative_i;
        tc->drive = tentative_drive;
        if (tc->drive >  tc->clamp)   tc->drive =  tc->clamp;
        if (tc->drive <  lower_clamp) tc->drive =  lower_clamp;
    }

    tempctrl_apply_drive(tc);
}

static void tempctrl_check_stall(TempControl *tempctrl) {
    // Only check while we're actually driving — in the hysteresis band the
    // Peltier is off and T_now can legitimately sit nearly still. `active`
    // alone isn't sufficient: with cooling_enabled=false the PI loop can sit
    // outside the deadband (active=true) while saturated at drive=0, which
    // is the configured refusal-to-cool, not a stall.
    if (!tempctrl->active || tempctrl->drive == 0.0f) {
        tempctrl->stall_window_active = false;
        tempctrl->runaway_strikes = 0;
        return;
    }
    absolute_time_t now = get_absolute_time();
    if (!tempctrl->stall_window_active) {
        tempctrl->stall_check_T = tempctrl->T_now;
        tempctrl->stall_check_drive = tempctrl->drive;
        tempctrl->stall_check_time = now;
        tempctrl->stall_window_active = true;
        return;
    }
    int64_t elapsed_us = absolute_time_diff_us(tempctrl->stall_check_time, now);
    if (elapsed_us < (int64_t)TEMPCTRL_STALL_WINDOW_MS * 1000) {
        return;
    }
    float delta = tempctrl->T_now - tempctrl->stall_check_T;
    if (fabsf(delta) < TEMPCTRL_STALL_MIN_DELTA) {
        // Drive engaged for a full window with no meaningful temperature
        // movement — sensor stuck or Peltier ineffective. Trip the channel;
        // `enabled` stays as the host set it, and the trip flag is the
        // runtime gate. Host clears it via *_enable=true (see
        // tempctrl_apply_enable).
        tempctrl->stall_tripped = true;
        tempctrl->active = false;
        tempctrl->drive = 0.0;
        tempctrl_apply_drive(tempctrl);
        tempctrl->stall_window_active = false;
        tempctrl->runaway_strikes = 0;
        return;
    }
    // Temperature moved. If it moved the *opposite* direction of the drive
    // (delta * drive < 0) the channel is running away — mis-wired Peltier,
    // lost hot-side dissipation, or swapped sensor. Score a strike; trip
    // only after TEMPCTRL_RUNAWAY_STRIKES consecutive wrong-direction
    // windows so a single startup/soak transient cannot false-trip a
    // channel that is actually controlling.
    if (delta * tempctrl->stall_check_drive < 0.0f) {
        tempctrl->runaway_strikes++;
        if (tempctrl->runaway_strikes >= TEMPCTRL_RUNAWAY_STRIKES) {
            tempctrl->runaway_tripped = true;
            tempctrl->active = false;
            tempctrl->drive = 0.0;
            tempctrl_apply_drive(tempctrl);
            tempctrl->stall_window_active = false;
            tempctrl->runaway_strikes = 0;
            return;
        }
    } else {
        // Healthy progress in the driven direction — clear any strikes.
        tempctrl->runaway_strikes = 0;
    }
    // Roll the window forward for the next evaluation.
    tempctrl->stall_check_T = tempctrl->T_now;
    tempctrl->stall_check_drive = tempctrl->drive;
    tempctrl->stall_check_time = now;
}

static void tempctrl_apply_enable(TempControl *tempctrl, bool new_enabled) {
    // *_enable=true is the host's explicit ack of sticky trips: it clears
    // this channel's stall flag and the app-wide watchdog flag. `enabled`
    // itself is pure host intent — firmware never mutates it, so the host
    // can always tell from status whether a channel is off because it was
    // asked off (`enabled=false`) or because a trip is gating it
    // (`enabled=true` with a trip flag set).
    if (new_enabled) {
        tempctrl->stall_tripped = false;
        tempctrl->runaway_tripped = false;
        tempctrl->stall_window_active = false;
        tempctrl->runaway_strikes = 0;
        tempctrl->sensor_rejects = 0;
        tempctrl->sensor_tripped = false;
        tempctrl->rate_ref_valid = false;
        tempctrl->seed_pending = false;
        watchdog_tripped = false;
    }
    tempctrl->enabled = new_enabled;
}

static bool tempctrl_drive_allowed(const TempControl *tempctrl) {
    return tempctrl->enabled
        && !tempctrl->sensor_tripped
        && !tempctrl->stall_tripped
        && !tempctrl->runaway_tripped
        && !watchdog_tripped;
}
