#include "tempmon.h"
#include "temp_shared.h"
#include "pico/stdlib.h"
#include "cJSON.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

// Static instances
static TempMonitor tempmon;

// Forward declarations
static void format_rom_code(uint64_t rom, char *buffer);

void tempmon_init(uint8_t app_id) {
    // Initialize temperature monitor structure
    memset(&tempmon, 0, sizeof(TempMonitor));
    tempmon.initialized = false;
    
    // Initialize shared temperature system
    if (temp_shared_init()) {
        // Get sensor information from shared system
        tempmon.sensor_count = temp_shared_get_sensor_count();
        
        // Copy sensor information to local structure
        for (int i = 0; i < tempmon.sensor_count && i < 8; i++) {
            tempmon.sensors[i].rom_code = temp_shared_get_rom_by_index(i);
            tempmon.sensors[i].temperature = 0.0;
            tempmon.sensors[i].valid = false;
            tempmon.sensors[i].last_read = 0;
            format_rom_code(tempmon.sensors[i].rom_code, tempmon.sensors[i].sensor_id);
        }
        
        tempmon.initialized = true;
        send_json(3,
            KV_STR, "status", "initialized",
            KV_INT, "app_id", app_id,
            KV_INT, "sensor_count", tempmon.sensor_count
        );
    } else {
        send_json(2,
            KV_STR, "error", "No DS18B20 sensors found",
            KV_INT, "app_id", app_id
        );
    }
}

void tempmon_server(uint8_t app_id, const char *json_str) {
    // Simplified tempmon does not handle commands
    // All timing and control is handled by main.c
}

void tempmon_status(uint8_t app_id) {
    if (!tempmon.initialized) {
        send_json(3,
            KV_STR, "status", "not_initialized",
            KV_INT, "app_id", app_id,
            KV_BOOL, "initialized", false
        );
        return;
    }
    
    // Create JSON array for sensor data
    cJSON *sensor_array = cJSON_CreateArray();
    for (int i = 0; i < tempmon.sensor_count; i++) {
        cJSON *sensor_obj = cJSON_CreateObject();
        cJSON_AddStringToObject(sensor_obj, "id", tempmon.sensors[i].sensor_id);
        cJSON_AddNumberToObject(sensor_obj, "temperature", tempmon.sensors[i].temperature);
        cJSON_AddBoolToObject(sensor_obj, "valid", tempmon.sensors[i].valid);
        cJSON_AddItemToArray(sensor_array, sensor_obj);
    }
    
    char *json_string = cJSON_Print(sensor_array);
    send_json(4,
        KV_STR, "status", "update",
        KV_INT, "app_id", app_id,
        KV_INT, "sensor_count", tempmon.sensor_count,
        KV_STR, "sensors", json_string
    );
    
    free(json_string);
    cJSON_Delete(sensor_array);
}

void tempmon_op(uint8_t app_id) {
    if (!tempmon.initialized) return;
    
    // Read all sensors using shared system
    tempmon_read_sensors();
    
    // Start next conversion cycle
    temp_shared_start_conversion();
}

bool tempmon_search_sensors(void) {
    // Use shared system to search for sensors
    int count = temp_shared_search_sensors();
    
    if (count > 0) {
        tempmon.sensor_count = count;
        for (int i = 0; i < count && i < 8; i++) {
            tempmon.sensors[i].rom_code = temp_shared_get_rom_by_index(i);
            tempmon.sensors[i].valid = false;
            tempmon.sensors[i].temperature = 0.0;
            tempmon.sensors[i].last_read = 0;
            format_rom_code(tempmon.sensors[i].rom_code, tempmon.sensors[i].sensor_id);
        }
        return true;
    }
    
    tempmon.sensor_count = 0;
    return false;
}

bool tempmon_read_sensors(void) {
    if (!tempmon.initialized) return false;
    
    // Use shared system to read all sensors
    bool success = temp_shared_read_all();
    
    if (success) {
        time_t current_time = time(NULL);
        
        // Update local sensor data from shared system
        for (int i = 0; i < tempmon.sensor_count; i++) {
            float temp = temp_shared_get_temp_by_index(i);
            bool valid = temp_shared_is_sensor_valid(i);
            
            if (valid) {
                tempmon.sensors[i].temperature = temp;
                tempmon.sensors[i].valid = true;
                tempmon.sensors[i].last_read = current_time;
            } else {
                tempmon.sensors[i].valid = false;
            }
        }
    }
    
    return success;
}

float tempmon_get_temperature(uint64_t rom_code) {
    // Use shared system for direct access
    return temp_shared_read_by_rom(rom_code);
}

int tempmon_get_sensor_count(void) {
    return tempmon.sensor_count;
}

bool tempmon_get_sensor_by_index(int index, TempSensor *sensor) {
    if (index >= 0 && index < tempmon.sensor_count && sensor) {
        *sensor = tempmon.sensors[index];
        return true;
    }
    return false;
}

// Helper functions
static void format_rom_code(uint64_t rom, char *buffer) {
    sprintf(buffer, "%016llX", rom);
}