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
static TempControl tempctrlA;
static TempControl tempctrlB;

// Forward declarations
static void init_single_tempctrl(TempControl *, uint, uint, uint, pwm_config *, uint, PIO);
static void tempctrl_update_sensor_drive(TempControl *);
static void tempctrl_drive_raw(TempControl *);
static void tempctrl_hysteresis_drive(TempControl *);

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
}

void tempctrl_init(uint8_t app_id) {
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, PWM_WRAP);
    init_single_tempctrl(&tempctrlA, PELTIER1_DIR_PIN1, PELTIER1_DIR_PIN2,
            PELTIER1_PWM_PIN, &config, TEMP_SENSOR1_PIN, pio0);
    init_single_tempctrl(&tempctrlB, PELTIER2_DIR_PIN3, PELTIER2_DIR_PIN4,
            PELTIER2_PWM_PIN, &config, TEMP_SENSOR2_PIN, pio1);
}

void tempctrl_server(uint8_t app_id, const char *json_str) {
    cJSON *item_json;
    cJSON *root = cJSON_Parse(json_str);
    if (!root) return;
    
    // Parse channel selection (default to both)
    item_json = cJSON_GetObjectItem(root, "A_temp_target");
    tempctrlA.T_target = item_json ? item_json->valuedouble : tempctrlA.T_target;
    item_json = cJSON_GetObjectItem(root, "A_enable");
    if (item_json) tempctrlA.enabled = item_json->valueint ? true : false;
    item_json = cJSON_GetObjectItem(root, "A_hysteresis");
    tempctrlA.hysteresis = item_json ? item_json->valuedouble : tempctrlA.hysteresis;
    item_json = cJSON_GetObjectItem(root, "B_temp_target");
    tempctrlB.T_target = item_json ? item_json->valuedouble : tempctrlB.T_target;
    item_json = cJSON_GetObjectItem(root, "B_enable");
    if (item_json) tempctrlB.enabled = item_json->valueint ? true : false;
    item_json = cJSON_GetObjectItem(root, "B_hysteresis");
    tempctrlB.hysteresis = item_json ? item_json->valuedouble : tempctrlB.hysteresis;
    cJSON_Delete(root);
}

void tempctrl_status(uint8_t app_id) {
    const float timeA = temp_sensor_get_conversion_time(&tempctrlA.temp_sensor);
    const float timeB = temp_sensor_get_conversion_time(&tempctrlB.temp_sensor);
    
    const char *statusA = temp_sensor_has_error(&tempctrlA.temp_sensor) ? "error" : "update";
    const char *statusB = temp_sensor_has_error(&tempctrlB.temp_sensor) ? "error" : "update";
    
    send_json(18,
        KV_STR, "sensor_name", "tempctrl",
        KV_INT, "app_id", app_id,
        KV_STR, "A_status", statusA,
        KV_FLOAT, "A_T_now", tempctrlA.T_now,
        KV_FLOAT, "A_timestamp", timeA,
        KV_FLOAT, "A_T_target", tempctrlA.T_target,
        KV_FLOAT, "A_drive_level", tempctrlA.drive,
        KV_BOOL, "A_enabled", tempctrlA.enabled,
        KV_BOOL, "A_int_disabled", tempctrlA.internally_disabled,
        KV_FLOAT, "A_hysteresis", tempctrlA.hysteresis,
        KV_STR, "B_status", statusB,
        KV_FLOAT, "B_T_now", tempctrlB.T_now,
        KV_FLOAT, "B_timestamp", timeB,
        KV_FLOAT, "B_T_target", tempctrlB.T_target,
        KV_FLOAT, "B_drive_level", tempctrlB.drive,
        KV_BOOL, "B_enabled", tempctrlB.enabled,
        KV_BOOL, "B_int_disabled", tempctrlB.internally_disabled,
        KV_FLOAT, "B_hysteresis", tempctrlB.hysteresis
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

    if (tempctrl->enabled && !tempctrl->internally_disabled) {
        tempctrl_hysteresis_drive(tempctrl);
    } else {
        tempctrl->drive = 0.0;
        tempctrl_drive_raw(tempctrl);
    }
}

void tempctrl_op(uint8_t app_id) {
    // Start conversions if not already started
    tempctrl_update_sensor_drive(&tempctrlA);
    tempctrl_update_sensor_drive(&tempctrlB);
}

// Helper functions
static void tempctrl_drive_raw(TempControl *tempctrl) {
    uint32_t pwm_level = (uint32_t)(fabsf(tempctrl->drive) * PWM_WRAP);
    bool forward = (tempctrl->drive >= 0);
    gpio_put(tempctrl->dir_pin1, forward);
    gpio_put(tempctrl->dir_pin2, !forward);
    pwm_set_gpio_level(tempctrl->pwm_pin, pwm_level);
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
