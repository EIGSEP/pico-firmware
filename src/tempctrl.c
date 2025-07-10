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
static TempControl tempctrl1;
static TempControl tempctrl2;
static TempSensor temp_sensor1;
static TempSensor temp_sensor2;

// Forward declarations
static void tempctrl_drive_raw(TempControl *pc);
static void tempctrl_hysteresis_drive(TempControl *pc);

void tempctrl_init(uint8_t app_id) {
    // Initialize GPIO for Peltier 1
    gpio_init(PELTIER1_DIR_PIN1);
    gpio_set_dir(PELTIER1_DIR_PIN1, GPIO_OUT);
    gpio_init(PELTIER1_DIR_PIN2);
    gpio_set_dir(PELTIER1_DIR_PIN2, GPIO_OUT);
    
    // Initialize GPIO for Peltier 2
    gpio_init(PELTIER2_DIR_PIN3);
    gpio_set_dir(PELTIER2_DIR_PIN3, GPIO_OUT);
    gpio_init(PELTIER2_DIR_PIN4);
    gpio_set_dir(PELTIER2_DIR_PIN4, GPIO_OUT);
    
    // Set up PWM for Peltier 1
    gpio_set_function(PELTIER1_PWM_PIN, GPIO_FUNC_PWM);
    tempctrl1.pwm_slice = pwm_gpio_to_slice_num(PELTIER1_PWM_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, PWM_WRAP);
    pwm_init(tempctrl1.pwm_slice, &config, true);
    
    // Set up PWM for Peltier 2
    gpio_set_function(PELTIER2_PWM_PIN, GPIO_FUNC_PWM);
    tempctrl2.pwm_slice = pwm_gpio_to_slice_num(PELTIER2_PWM_PIN);
    pwm_init(tempctrl2.pwm_slice, &config, true);
    
    // Initialize Temperature Control 1 structure
    tempctrl1.T_target = 30.0;
    tempctrl1.gain = 0.2;
    tempctrl1.baseline = 0.4;  // Baseline drive level
    tempctrl1.clamp = 0.6;  // Maximum drive level
    tempctrl1.hysteresis = 0.5;
    tempctrl1.enabled = false;
    tempctrl1.active = false;
    tempctrl1.permanently_disabled = false;
    tempctrl1.channel = 1;
    tempctrl1.T_now = 0;
    tempctrl1.drive = 0.0;
    tempctrl1.error_count = 0;
    tempctrl1.last_error_time = 0;
    
    // Initialize Temperature Control 2 structure
    tempctrl2.T_target = 32.0;
    tempctrl2.gain = 0.2;
    tempctrl2.baseline = 0.4;  // Baseline drive level
    tempctrl2.clamp = 0.6;  // Maximum drive level
    tempctrl2.hysteresis = 0.5;
    tempctrl2.enabled = false;
    tempctrl2.active = false;
    tempctrl2.permanently_disabled = false;
    tempctrl2.channel = 2;
    tempctrl2.T_now = 0;
    tempctrl2.drive = 0.0;
    tempctrl2.error_count = 0;
    tempctrl2.last_error_time = 0;
    
    // Initialize temperature sensors on separate pins
    uint offset1 = pio_add_program(pio0, &onewire_program);
    uint offset2 = pio_add_program(pio1, &onewire_program);
    
    temp_sensor_init(&temp_sensor1, TEMP_SENSOR1_PIN, pio0, offset1);
    temp_sensor_init(&temp_sensor2, TEMP_SENSOR2_PIN, pio1, offset2);
}

void tempctrl_server(uint8_t app_id, const char *json_str) {
    cJSON *root = cJSON_Parse(json_str);
    if (!root) return;
    
    // Parse channel selection (default to both)
    cJSON *channel_json = cJSON_GetObjectItem(root, "channel");
    int channel = channel_json ? channel_json->valueint : 0;
    
    // Parse command
    cJSON *cmd_json = cJSON_GetObjectItem(root, "cmd");
    const char *cmd = cmd_json ? cmd_json->valuestring : "";
    
    if (strcmp(cmd, "set_temp") == 0) {
        cJSON *temp_json = cJSON_GetObjectItem(root, "temperature");
        if (temp_json) {
            float temp = temp_json->valuedouble;
            if (channel == 0 || channel == 1) {
                tempctrl1.T_target = temp;
            }
            if (channel == 0 || channel == 2) {
                tempctrl2.T_target = temp;
            }
        }
    } else if (strcmp(cmd, "set_hysteresis") == 0) {
        cJSON *hyst_json = cJSON_GetObjectItem(root, "hysteresis");
        if (hyst_json) {
            float hyst = hyst_json->valuedouble;
            if (channel == 0 || channel == 1) {
                tempctrl1.hysteresis = hyst;
            }
            if (channel == 0 || channel == 2) {
                tempctrl2.hysteresis = hyst;
            }
        }
    } else if (strcmp(cmd, "enable") == 0) {
        if (channel == 0 || channel == 1) {
            if (!tempctrl1.permanently_disabled) {
                tempctrl1.enabled = true;
            }
        }
        if (channel == 0 || channel == 2) {
            if (!tempctrl2.permanently_disabled) {
                tempctrl2.enabled = true;
            }
        }
    } else if (strcmp(cmd, "disable") == 0) {
        if (channel == 0 || channel == 1) {
            tempctrl1.enabled = false;
            tempctrl1.drive = 0.0;
            tempctrl_drive_raw(&tempctrl1);
        }
        if (channel == 0 || channel == 2) {
            tempctrl2.enabled = false;
            tempctrl2.drive = 0.0;
            tempctrl_drive_raw(&tempctrl2);
        }
    }
    
    cJSON_Delete(root);
}

void tempctrl_status(uint8_t app_id) {
    const float time1 = temp_sensor_get_conversion_time(&temp_sensor1);
    const float time2 = temp_sensor_get_conversion_time(&temp_sensor2);
    
    const char *status1 = temp_sensor_has_error(&temp_sensor1) ? "error" : "update";
    const char *status2 = temp_sensor_has_error(&temp_sensor2) ? "error" : "update";
    
    send_json(18,
        KV_STR, "sensor_name", "tempctrl",
        KV_INT, "app_id", app_id,
        KV_STR, "A_status", status1,
        KV_FLOAT, "A_temp_now", tempctrl1.T_now,
        KV_FLOAT, "A_timestamp", time1,
        KV_FLOAT, "A_temp_target", tempctrl1.T_target,
        KV_FLOAT, "A_drive_level", tempctrl1.drive,
        KV_BOOL, "A_enabled", tempctrl1.enabled,
        KV_BOOL, "A_perm_disabled", tempctrl1.permanently_disabled,
        KV_FLOAT, "A_hysteresis", tempctrl1.hysteresis,
        KV_STR, "B_status", status2,
        KV_FLOAT, "B_temp_now", tempctrl2.T_now,
        KV_FLOAT, "B_timestamp", time2,
        KV_FLOAT, "B_temp_target", tempctrl2.T_target,
        KV_FLOAT, "B_drive_level", tempctrl2.drive,
        KV_BOOL, "B_enabled", tempctrl2.enabled,
        KV_BOOL, "B_perm_disabled", tempctrl2.permanently_disabled,
        KV_FLOAT, "B_hysteresis", tempctrl2.hysteresis
    );
}

void tempctrl_op(uint8_t app_id) {
    // Start conversions if not already started
    if (!temp_sensor1.conversion_started) {
        temp_sensor_start_conversion(&temp_sensor1);
    }
    if (!temp_sensor2.conversion_started) {
        temp_sensor_start_conversion(&temp_sensor2);
    }
    
    // Read sensors (auto-skip if conversion not ready)
    temp_sensor_read(&temp_sensor1);
    temp_sensor_read(&temp_sensor2);
    
    // Update current temperatures
    tempctrl1.T_now = temp_sensor_get_temp(&temp_sensor1);
    tempctrl2.T_now = temp_sensor_get_temp(&temp_sensor2);
    
    // Check for sensor errors and track error counts
    uint32_t now = to_ms_since_boot(get_absolute_time());
    
    // Handle sensor 1 error or drive Peltiers based on hysteresis control
    if (temp_sensor_has_error(&temp_sensor1)) {
        if ((now - tempctrl1.last_error_time) > ERROR_TIME_WINDOW_MS) {
            tempctrl1.error_count = 0;  // Reset count if outside time window
        }
        tempctrl1.error_count++;
        tempctrl1.last_error_time = now;
        
        if (tempctrl1.error_count >= ERROR_COUNT_THRESHOLD) {
            tempctrl1.permanently_disabled = true;
        }
        
        if (tempctrl1.enabled) {
            tempctrl1.drive = 0.0;
            tempctrl_drive_raw(&tempctrl1);
        }
    }
    else if (tempctrl1.enabled && !tempctrl1.permanently_disabled) {
        tempctrl_hysteresis_drive(&tempctrl1);
    }
    
    // Handle sensor 2 errors or drive Peltiers based on hysteresis control
    if (temp_sensor_has_error(&temp_sensor2)) {
        if ((now - tempctrl2.last_error_time) > ERROR_TIME_WINDOW_MS) {
            tempctrl2.error_count = 0;  // Reset count if outside time window
        }
        tempctrl2.error_count++;
        tempctrl2.last_error_time = now;
        
        if (tempctrl2.error_count >= ERROR_COUNT_THRESHOLD) {
            tempctrl2.permanently_disabled = true;
        }
        
        if (tempctrl2.enabled) {
            tempctrl2.drive = 0.0;
            tempctrl_drive_raw(&tempctrl2);
        }
    }
    else if (tempctrl2.enabled && !tempctrl2.permanently_disabled) {
        tempctrl_hysteresis_drive(&tempctrl2);
    }
}

// Helper functions
static void tempctrl_drive_raw(TempControl *pc) {
    uint32_t pwm_level = (uint32_t)(fabsf(pc->drive) * PWM_WRAP);
    bool forward = (pc->drive >= 0);
    if (pc->channel == 1) {
        gpio_put(PELTIER1_DIR_PIN1, forward);
        gpio_put(PELTIER1_DIR_PIN2, !forward);
        pwm_set_gpio_level(PELTIER1_PWM_PIN, pwm_level);
    } else if (pc->channel == 2) {
        gpio_put(PELTIER2_DIR_PIN3, forward);
        gpio_put(PELTIER2_DIR_PIN4, !forward);
        pwm_set_gpio_level(PELTIER2_PWM_PIN, pwm_level);
    }
}

static void tempctrl_hysteresis_drive(TempControl *pc) {
    float error = pc->T_target - pc->T_now;
    int sign = (error >= 0) ? 1 : -1;
    
    if (fabsf(error) <= pc->hysteresis) {
        // Within hysteresis band - turn off
        pc->drive = 0.0;
        pc->active = false;
    } else {
        // Outside hysteresis band - engage control
        pc->active = true;
        
        // Simple proportional control using gain and baseline drive
        pc->drive = error * pc->gain + sign * pc->baseline;
        
        // Limit drive to maximum power (clamp acts as max power)
        if (fabsf(pc->drive) > pc->clamp) {
            pc->drive = sign * pc->clamp;
        }
    }
    
    tempctrl_drive_raw(pc);
}
