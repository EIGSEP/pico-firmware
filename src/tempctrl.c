#include "tempctrl.h"
#include "temp_simple.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "onewire_library.pio.h"
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

// Forward declarations
static void init_single_tempctrl(TempControl *, uint, uint, uint, pwm_config *, uint, PIO);
static void tempctrl_update_sensor_drive(TempControl *);
static void tempctrl_drive_raw(TempControl *);
static void tempctrl_hysteresis_drive(TempControl *);
static void tempctrl_check_stall(TempControl *);
static void tempctrl_apply_enable(TempControl *, bool);
static bool tempctrl_drive_allowed(const TempControl *);

void init_single_tempctrl(TempControl *tempctrl,
                          uint dir_pin1, uint dir_pin2, uint pwm_pin,
                          pwm_config *config, uint temp_sensor_pin, PIO pio) {
    // Initialize GPIO for Peltier 
    gpio_init(dir_pin1);
    gpio_set_dir(dir_pin1, GPIO_OUT);
    gpio_init(dir_pin2);
    gpio_set_dir(dir_pin2, GPIO_OUT);
    // Set up PWM for Peltier 
    gpio_set_function(pwm_pin, GPIO_FUNC_PWM);
    tempctrl->pwm_slice = pwm_gpio_to_slice_num(pwm_pin);
    pwm_init(tempctrl->pwm_slice, config, true);
    
    uint offset = pio_add_program(pio, &onewire_program);
    temp_sensor_init(&tempctrl->temp_sensor, temp_sensor_pin, pio, offset);

    // Initialize Temperature Control structure
    tempctrl->dir_pin1 = dir_pin1;
    tempctrl->dir_pin2 = dir_pin2;
    tempctrl->pwm_pin = pwm_pin;
    tempctrl->T_target = 30.0;
    tempctrl->gain = 0.2;
    tempctrl->baseline = 0.4;  // Baseline drive level
    tempctrl->clamp = 0.6;  // Maximum drive level
    tempctrl->hysteresis = 0.5;
    tempctrl->enabled = false;
    tempctrl->active = false;
    tempctrl->internally_disabled = false;
    tempctrl->T_now = 0;
    tempctrl->drive = 0.0;
    tempctrl->stall_tripped = false;
    tempctrl->stall_window_active = false;
    tempctrl->stall_check_T = 0.0;
    tempctrl->stall_check_time = get_absolute_time();
}

void tempctrl_init(uint8_t app_id) {
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 145.0f);         // PWM frequency = System_Clock / (Clock_Divider × (WRAP + 1)), system_clock = 150 MHz default
    pwm_config_set_wrap(&config, PWM_WRAP);
    init_single_tempctrl(&tempctrl_lna, PELTIER_LNA_DIR_PIN1, PELTIER_LNA_DIR_PIN2,
            PELTIER_LNA_PWM_PIN, &config, TEMP_SENSOR_LNA_PIN, pio0);
    init_single_tempctrl(&tempctrl_load, PELTIER_LOAD_DIR_PIN3, PELTIER_LOAD_DIR_PIN4,
            PELTIER_LOAD_PWM_PIN, &config, TEMP_SENSOR_LOAD_PIN, pio1);
    last_cmd_time = get_absolute_time();
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
    item_json = cJSON_GetObjectItem(root, "LOAD_temp_target");
    tempctrl_load.T_target = item_json ? item_json->valuedouble : tempctrl_load.T_target;
    item_json = cJSON_GetObjectItem(root, "LOAD_enable");
    if (item_json) tempctrl_apply_enable(&tempctrl_load, item_json->valueint ? true : false);
    item_json = cJSON_GetObjectItem(root, "LOAD_hysteresis");
    tempctrl_load.hysteresis = item_json ? item_json->valuedouble : tempctrl_load.hysteresis;
    item_json = cJSON_GetObjectItem(root, "LOAD_clamp");
    if (item_json) tempctrl_load.clamp = fminf(1.0, fmaxf(0.0, item_json->valuedouble));

    // Watchdog timeout configuration (0 = disabled)
    item_json = cJSON_GetObjectItem(root, "watchdog_timeout_ms");
    if (item_json && cJSON_IsNumber(item_json)) {
        int val = item_json->valueint;
        watchdog_timeout_ms = val < 0 ? 0 : (uint32_t)val;
    }

    cJSON_Delete(root);
}

void tempctrl_status(uint8_t app_id) {
    const uint32_t time_lna = temp_sensor_get_conversion_time(&tempctrl_lna.temp_sensor);
    const uint32_t time_load = temp_sensor_get_conversion_time(&tempctrl_load.temp_sensor);

    /* read_error is set/cleared on every temp_sensor_read() attempt, so
       LNA_status / LOAD_status reflect the most recent attempt rather
       than a stale timeout window. Matches the per-cycle status contract
       enforced by eigsep_observing._avg_sensor_values. */
    const char *status_lna = temp_sensor_has_error(&tempctrl_lna.temp_sensor) ? "error" : "update";
    const char *status_load = temp_sensor_has_error(&tempctrl_load.temp_sensor) ? "error" : "update";

    send_json(26,
        KV_STR, "sensor_name", "tempctrl",
        KV_INT, "app_id", app_id,
        KV_BOOL, "watchdog_tripped", watchdog_tripped,
        KV_INT, "watchdog_timeout_ms", (int)watchdog_timeout_ms,
        KV_STR, "LNA_status", status_lna,
        KV_FLOAT, "LNA_T_now", tempctrl_lna.T_now,
        KV_FLOAT, "LNA_timestamp", (double)time_lna,
        KV_FLOAT, "LNA_T_target", tempctrl_lna.T_target,
        KV_FLOAT, "LNA_drive_level", tempctrl_lna.drive,
        KV_BOOL, "LNA_enabled", tempctrl_lna.enabled,
        KV_BOOL, "LNA_active", tempctrl_lna.active,
        KV_BOOL, "LNA_int_disabled", tempctrl_lna.internally_disabled,
        KV_BOOL, "LNA_stall_tripped", tempctrl_lna.stall_tripped,
        KV_FLOAT, "LNA_hysteresis", tempctrl_lna.hysteresis,
        KV_FLOAT, "LNA_clamp", tempctrl_lna.clamp,
        KV_STR, "LOAD_status", status_load,
        KV_FLOAT, "LOAD_T_now", tempctrl_load.T_now,
        KV_FLOAT, "LOAD_timestamp", (double)time_load,
        KV_FLOAT, "LOAD_T_target", tempctrl_load.T_target,
        KV_FLOAT, "LOAD_drive_level", tempctrl_load.drive,
        KV_BOOL, "LOAD_enabled", tempctrl_load.enabled,
        KV_BOOL, "LOAD_active", tempctrl_load.active,
        KV_BOOL, "LOAD_int_disabled", tempctrl_load.internally_disabled,
        KV_BOOL, "LOAD_stall_tripped", tempctrl_load.stall_tripped,
        KV_FLOAT, "LOAD_hysteresis", tempctrl_load.hysteresis,
        KV_FLOAT, "LOAD_clamp", tempctrl_load.clamp
    );
}

void tempctrl_update_sensor_drive(TempControl *tempctrl) {
    // Start conversions if not already started
    if (!tempctrl->temp_sensor.conversion_started) {
        temp_sensor_start_conversion(&tempctrl->temp_sensor);
    }
    
    // Read sensors (auto-skip if conversion not ready)
    temp_sensor_read(&tempctrl->temp_sensor);
    
    // Update current temperatures
    tempctrl->T_now = temp_sensor_get_temp(&tempctrl->temp_sensor);
    
    // Handle sensor 1 error or drive Peltiers based on hysteresis control
    tempctrl->internally_disabled = temp_sensor_has_error(&tempctrl->temp_sensor) ? true : false;

    if (tempctrl_drive_allowed(tempctrl)) {
        tempctrl_hysteresis_drive(tempctrl);
        tempctrl_check_stall(tempctrl);
    } else {
        tempctrl->drive = 0.0;
        tempctrl->active = false;
        tempctrl_drive_raw(tempctrl);
        // Not actively driving — no stall window pending.
        tempctrl->stall_window_active = false;
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

    tempctrl_update_sensor_drive(&tempctrl_lna);
    tempctrl_update_sensor_drive(&tempctrl_load);
}

// Helper functions
static void tempctrl_drive_raw(TempControl *tempctrl) {
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

static void tempctrl_hysteresis_drive(TempControl *tempctrl) {
    float T_delta = tempctrl->T_target - tempctrl->T_now;
    int sign = (T_delta >= 0) ? 1 : -1;

    if (fabsf(T_delta) <= tempctrl->hysteresis) {
        // Within hysteresis band - turn off
        tempctrl->drive = 0.0;
        tempctrl->active = false;
    } else {
        // Outside hysteresis band - engage control
        tempctrl->active = true;
        // Simple proportional control using gain and baseline drive
        tempctrl->drive = T_delta * tempctrl->gain + sign * tempctrl->baseline;
        // Limit drive to maximum power (clamp acts as max power)
        if (fabsf(tempctrl->drive) > tempctrl->clamp) {
            tempctrl->drive = sign * tempctrl->clamp;
        }
    }
    tempctrl_drive_raw(tempctrl);
}

static void tempctrl_check_stall(TempControl *tempctrl) {
    // Only check while we're actively driving — in the hysteresis band the
    // Peltier is off and T_now can legitimately sit nearly still.
    if (!tempctrl->active) {
        tempctrl->stall_window_active = false;
        return;
    }
    absolute_time_t now = get_absolute_time();
    if (!tempctrl->stall_window_active) {
        tempctrl->stall_check_T = tempctrl->T_now;
        tempctrl->stall_check_time = now;
        tempctrl->stall_window_active = true;
        return;
    }
    int64_t elapsed_us = absolute_time_diff_us(tempctrl->stall_check_time, now);
    if (elapsed_us < (int64_t)TEMPCTRL_STALL_WINDOW_MS * 1000) {
        return;
    }
    if (fabsf(tempctrl->T_now - tempctrl->stall_check_T) < TEMPCTRL_STALL_MIN_DELTA) {
        // Drive engaged for a full window with no meaningful temperature
        // movement — sensor stuck or Peltier ineffective. Trip the channel;
        // `enabled` stays as the host set it, and the trip flag is the
        // runtime gate. Host clears it via *_enable=true (see
        // tempctrl_apply_enable).
        tempctrl->stall_tripped = true;
        tempctrl->active = false;
        tempctrl->drive = 0.0;
        tempctrl_drive_raw(tempctrl);
        tempctrl->stall_window_active = false;
    } else {
        // Healthy: roll the window forward.
        tempctrl->stall_check_T = tempctrl->T_now;
        tempctrl->stall_check_time = now;
    }
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
        tempctrl->stall_window_active = false;
        watchdog_tripped = false;
    }
    tempctrl->enabled = new_enabled;
}

static bool tempctrl_drive_allowed(const TempControl *tempctrl) {
    return tempctrl->enabled
        && !tempctrl->internally_disabled
        && !tempctrl->stall_tripped
        && !watchdog_tripped;
}
