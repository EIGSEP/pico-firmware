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
static TempMonitor tempmon;
static TempSensor sensor1;
static TempSensor sensor2;

void tempmon_init(uint8_t app_id) {
    // Initialize temperature monitor structure
    memset(&tempmon, 0, sizeof(TempMonitor));
    tempmon.initialized = false;
    
    // Initialize temperature sensors on separate pins
    uint offset1 = pio_add_program(pio0, &onewire_program);
    uint offset2 = pio_add_program(pio1, &onewire_program);
    
    tempmon.sensor1_valid = temp_sensor_init(&sensor1, TEMPMON_SENSOR1_PIN, pio0, offset1);
    tempmon.sensor2_valid = temp_sensor_init(&sensor2, TEMPMON_SENSOR2_PIN, pio1, offset2);
    
    if (tempmon.sensor1_valid || tempmon.sensor2_valid) {
        tempmon.initialized = true;
    }
}

void tempmon_server(uint8_t app_id, const char *json_str) {
    // tempmon does not handle commands
}

void tempmon_status(uint8_t app_id) {
    if (!tempmon.initialized) {
        send_json(3,
            KV_STR, "status", "error",
            KV_INT, "app_id", app_id,
            KV_BOOL, "initialized", false
        );
        return;
    }
    
    send_json(9,
        KV_STR, "status", "update",
        KV_INT, "app_id", app_id,
        KV_FLOAT, "temperature1", tempmon.temperature1,
        KV_INT, "temperature1_gpio", TEMPMON_SENSOR1_PIN,
        KV_FLOAT, "temperature2", tempmon.temperature2,
        KV_INT, "temperature2_gpio", TEMPMON_SENSOR2_PIN,
        KV_BOOL, "sensor1_connected", tempmon.sensor1_valid,
        KV_BOOL, "sensor2_connected", tempmon.sensor2_valid,
        KV_INT, "last_read_ms", (int)tempmon.last_read
    );
}

void tempmon_op(uint8_t app_id) {
    if (!tempmon.initialized) return;
    
    // Read all sensors
    tempmon_read_sensors();
}

bool tempmon_read_sensors(void) {
    if (!tempmon.initialized) return false;
    
    uint32_t now = to_ms_since_boot(get_absolute_time());
    
    // Only read every 750ms to allow conversion time
    if ((now - tempmon.last_read) < 750) {
        return false;
    }
    
    tempmon.last_read = now;
    bool success = false;
    
    // Read sensor 1
    if (tempmon.sensor1_valid && temp_sensor_read(&sensor1)) {
        tempmon.temperature1 = temp_sensor_get_temp(&sensor1);
        tempmon.sensor1_valid = temp_sensor_is_valid(&sensor1);
        if (tempmon.sensor1_valid) success = true;
    }
    
    // Read sensor 2
    if (tempmon.sensor2_valid && temp_sensor_read(&sensor2)) {
        tempmon.temperature2 = temp_sensor_get_temp(&sensor2);
        tempmon.sensor2_valid = temp_sensor_is_valid(&sensor2);
        if (tempmon.sensor2_valid) success = true;
    }
    
    if (success) {
        // Start next conversion cycle
        temp_sensor_start_conversion(&sensor1);
        temp_sensor_start_conversion(&sensor2);
    }
    
    return success;
}

float tempmon_get_temperature1(void) {
    return tempmon.sensor1_valid ? tempmon.temperature1 : NAN;
}

float tempmon_get_temperature2(void) {
    return tempmon.sensor2_valid ? tempmon.temperature2 : NAN;
}
