#include "tempmon.h"
#include "temp_simple.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "onewire_library.pio.h"
#include "cJSON.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

// Static instances
static TempSensor sensor1;
static TempSensor sensor2;

void tempmon_init(uint8_t app_id) {
    // Initialize temperature monitor structure
    // (removed - no longer using TempMonitor struct)
    
    // Initialize temperature sensors on separate pins
    uint offset1 = pio_add_program(pio0, &onewire_program);
    uint offset2 = pio_add_program(pio1, &onewire_program);
    
    temp_sensor_init(&sensor1, TEMPMON_SENSOR1_PIN, pio0, offset1);
    temp_sensor_init(&sensor2, TEMPMON_SENSOR2_PIN, pio1, offset2);
}

void tempmon_server(uint8_t app_id, const char *json_str) {
    // tempmon does not handle commands
}

void tempmon_status(uint8_t app_id) {
    const float time1 = temp_sensor_get_conversion_time(&sensor1);
    const float time2 = temp_sensor_get_conversion_time(&sensor2);
    const float temp1 = temp_sensor_get_temp(&sensor1);
    const float temp2 = temp_sensor_get_temp(&sensor2);
    
    const char *status1 = temp_sensor_has_error(&sensor1) ? "error" : "update";
    const char *status2 = temp_sensor_has_error(&sensor2) ? "error" : "update";

    send_json(8,
        KV_STR, "sensor_name", "temp_mon",
        KV_INT, "app_id", app_id,
        KV_STR, "A_status", status1,
        KV_FLOAT, "A_temp", temp1,
        KV_FLOAT, "A_timestamp", time1,
        KV_STR, "B_status", status2,
        KV_FLOAT, "B_temp", temp2,
        KV_FLOAT, "B_timestamp", time2
    );
}

void tempmon_op(uint8_t app_id) {
    // check if converstion has started
    if (!sensor1.conversion_started) {
        // start conversion for sensor 1
        temp_sensor_start_conversion(&sensor1);
    }
    if (!sensor2.conversion_started) {
        // start conversion for sensor 2
        temp_sensor_start_conversion(&sensor2);
    }

    // read sensors (auto-skip if conversion not done)
    temp_sensor_read(&sensor1);
    temp_sensor_read(&sensor2);
}
