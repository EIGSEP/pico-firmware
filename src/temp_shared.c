#include "temp_shared.h"
#include "pico/stdlib.h"
#include <string.h>
#include <math.h>

// Static shared instance
static TempShared temp_shared;

bool temp_shared_init(void) {
    if (temp_shared.initialized) {
        return true;  // Already initialized
    }
    
    // Initialize structure
    memset(&temp_shared, 0, sizeof(TempShared));
    
    // Initialize OneWire
    uint offset = pio_add_program(pio0, &onewire_program);
    ow_init(&temp_shared.ow, pio0, offset, TEMP_SHARED_DS_PIN);
    
    // Search for sensors
    temp_shared.sensor_count = temp_shared_search_sensors();
    
    if (temp_shared.sensor_count > 0) {
        temp_shared.initialized = true;
        temp_shared_start_conversion();
        return true;
    }
    
    return false;
}

int temp_shared_search_sensors(void) {
    int count = ow_romsearch(&temp_shared.ow, temp_shared.rom_codes, 8, OW_SEARCH_ROM);
    
    if (count > 0) {
        temp_shared.sensor_count = count;
        // Initialize temperature arrays
        for (int i = 0; i < count; i++) {
            temp_shared.temperatures[i] = 0.0;
            temp_shared.sensor_valid[i] = false;
        }
    }
    
    return count;
}

void temp_shared_start_conversion(void) {
    if (!temp_shared.initialized) return;
    
    // Start temperature conversion for all sensors
    ow_reset(&temp_shared.ow);
    ow_send(&temp_shared.ow, OW_SKIP_ROM);
    ow_send(&temp_shared.ow, DS18B20_CONVERT_T);
    
    temp_shared.last_conversion_time = to_ms_since_boot(get_absolute_time());
}

float temp_shared_read_by_rom(uint64_t rom_code) {
    if (!temp_shared.initialized) return NAN;
    
    // Read scratchpad from specific sensor
    ow_reset(&temp_shared.ow);
    ow_send(&temp_shared.ow, OW_MATCH_ROM);
    for (int i = 0; i < 8; i++) {
        ow_send(&temp_shared.ow, (rom_code >> (i * 8)) & 0xFF);
    }
    ow_send(&temp_shared.ow, DS18B20_READ_SCRATCHPAD);
    
    uint8_t data[9];
    for (int i = 0; i < 9; i++) {
        data[i] = ow_read(&temp_shared.ow);
    }
    
    // Convert to temperature
    int16_t raw_temp = (data[1] << 8) | data[0];
    return raw_temp / 16.0;
}

bool temp_shared_read_all(void) {
    if (!temp_shared.initialized) return false;
    
    bool success = false;
    
    // Wait at least 750ms since last conversion start
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if ((now - temp_shared.last_conversion_time) < 750) {
        return false;  // Not enough time has passed
    }
    
    // Read all sensors
    for (int i = 0; i < temp_shared.sensor_count; i++) {
        float temp = temp_shared_read_by_rom(temp_shared.rom_codes[i]);
        
        // Validate temperature reading (DS18B20 range)
        if (temp > -55.0 && temp < 125.0) {
            temp_shared.temperatures[i] = temp;
            temp_shared.sensor_valid[i] = true;
            success = true;
        } else {
            temp_shared.sensor_valid[i] = false;
        }
    }
    
    return success;
}

int temp_shared_get_sensor_count(void) {
    return temp_shared.sensor_count;
}

uint64_t temp_shared_get_rom_by_index(int index) {
    if (index >= 0 && index < temp_shared.sensor_count) {
        return temp_shared.rom_codes[index];
    }
    return 0;
}

float temp_shared_get_temp_by_index(int index) {
    if (index >= 0 && index < temp_shared.sensor_count && temp_shared.sensor_valid[index]) {
        return temp_shared.temperatures[index];
    }
    return NAN;
}

bool temp_shared_is_sensor_valid(int index) {
    if (index >= 0 && index < temp_shared.sensor_count) {
        return temp_shared.sensor_valid[index];
    }
    return false;
}

TempShared* temp_shared_get_instance(void) {
    return &temp_shared;
}