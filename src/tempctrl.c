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
static volatile bool temperature_reading_active = false;
static volatile float last_temp1 = 25.0;
static volatile float last_temp2 = 25.0;

// Forward declarations
static void tempctrl_drive_raw(TempControl *pc, bool forward, uint32_t level);
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
    tempctrl1.hysteresis = 0.5;
    tempctrl1.enabled = true;
    tempctrl1.active = true;
    tempctrl1.channel = 1;
    tempctrl1.T_now = tempctrl1.T_target;
    tempctrl1.drive = 0.0;
    
    // Initialize Temperature Control 2 structure
    tempctrl2.T_target = 32.0;
    tempctrl2.gain = 0.2;
    tempctrl2.hysteresis = 0.5;
    tempctrl2.enabled = true;
    tempctrl2.active = true;
    tempctrl2.channel = 2;
    tempctrl2.T_now = tempctrl2.T_target;
    tempctrl2.drive = 0.0;
    
    // Initialize temperature sensors on separate pins
    uint offset1 = pio_add_program(pio0, &onewire_program);
    uint offset2 = pio_add_program(pio1, &onewire_program);
    
    if (temp_sensor_init(&temp_sensor1, TEMP_SENSOR1_PIN, pio0, offset1) &&
        temp_sensor_init(&temp_sensor2, TEMP_SENSOR2_PIN, pio1, offset2)) {
        temperature_reading_active = true;
    }
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
            tempctrl1.enabled = true;
        }
        if (channel == 0 || channel == 2) {
            tempctrl2.enabled = true;
        }
    } else if (strcmp(cmd, "disable") == 0) {
        if (channel == 0 || channel == 1) {
            tempctrl1.enabled = false;
            tempctrl1.drive = 0.0;
            tempctrl_drive_raw(&tempctrl1, true, 0);
        }
        if (channel == 0 || channel == 2) {
            tempctrl2.enabled = false;
            tempctrl2.drive = 0.0;
            tempctrl_drive_raw(&tempctrl2, true, 0);
        }
    }
    
    cJSON_Delete(root);
}

void tempctrl_status(uint8_t app_id) {
    send_json(11,
        KV_STR, "status", "update",
        KV_INT, "app_id", app_id,
        KV_FLOAT, "temp1", tempctrl1.T_now,
        KV_FLOAT, "target1", tempctrl1.T_target,
        KV_FLOAT, "drive1", tempctrl1.drive,
        KV_BOOL, "enabled1", tempctrl1.enabled,
        KV_FLOAT, "temp2", tempctrl2.T_now,
        KV_FLOAT, "target2", tempctrl2.T_target,
        KV_FLOAT, "drive2", tempctrl2.drive,
        KV_BOOL, "enabled2", tempctrl2.enabled,
        KV_BOOL, "sensors_active", temperature_reading_active
    );
}

void tempctrl_op(uint8_t app_id) {
    static uint32_t last_temp_read = 0;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    
    // Read temperatures and control every 750ms
    if (temperature_reading_active && (now - last_temp_read) >= 750) {
        last_temp_read = now;
        
        // Read temperatures from individual sensors
        bool read1 = temp_sensor_read(&temp_sensor1);
        bool read2 = temp_sensor_read(&temp_sensor2);
        
        if (read1) {
            float temp1 = temp_sensor_get_temp(&temp_sensor1);
            if (!isnan(temp1)) {
                last_temp1 = temp1;
                tempctrl1.T_now = temp1;
            }
        }
        
        if (read2) {
            float temp2 = temp_sensor_get_temp(&temp_sensor2);
            if (!isnan(temp2)) {
                last_temp2 = temp2;
                tempctrl2.T_now = temp2;
            }
        }
        
        // Drive Peltiers based on hysteresis control
        if (tempctrl1.enabled) {
            tempctrl_hysteresis_drive(&tempctrl1);
        }
        if (tempctrl2.enabled) {
            tempctrl_hysteresis_drive(&tempctrl2);
        }
        
        // Start new temperature conversion for next cycle
        temp_sensor_start_conversion(&temp_sensor1);
        temp_sensor_start_conversion(&temp_sensor2);
    }
}

// Helper functions
static void tempctrl_drive_raw(TempControl *pc, bool forward, uint32_t level) {
    if (pc->channel == 1) {
        gpio_put(PELTIER1_DIR_PIN1, forward);
        gpio_put(PELTIER1_DIR_PIN2, !forward);
        pwm_set_gpio_level(PELTIER1_PWM_PIN, level);
    } else if (pc->channel == 2) {
        gpio_put(PELTIER2_DIR_PIN3, forward);
        gpio_put(PELTIER2_DIR_PIN4, !forward);
        pwm_set_gpio_level(PELTIER2_PWM_PIN, level);
    }
}

static void tempctrl_hysteresis_drive(TempControl *pc) {
    float error = pc->T_target - pc->T_now;
    
    if (fabsf(error) <= pc->hysteresis) {
        // Within hysteresis band - turn off
        pc->drive = 0.0;
        pc->active = false;
    } else {
        // Outside hysteresis band - engage control
        pc->active = true;
        
        // Simple proportional control with gain limiting
        pc->drive = error * 0.1;  // Proportional factor
        
        // Limit drive to gain setting
        if (pc->drive > pc->gain) {
            pc->drive = pc->gain;
        } else if (pc->drive < -pc->gain) {
            pc->drive = -pc->gain;
        }
    }
    
    // Apply drive signal
    if (pc->drive >= 0) {
        // Heating mode
        tempctrl_drive_raw(pc, true, (uint32_t)(pc->drive * PWM_WRAP));
    } else {
        // Cooling mode
        tempctrl_drive_raw(pc, false, (uint32_t)(-pc->drive * PWM_WRAP));
    }
}
