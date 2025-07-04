#ifndef TEMP_SHARED_H
#define TEMP_SHARED_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "hardware/pio.h"

// OneWire includes
#include "onewire_library.h"
#include "onewire_library.pio.h"
#include "ds18b20.h"
#include "ow_rom.h"

// OneWire DS18B20 temperature sensor pin
#define TEMP_SHARED_DS_PIN              22

// Shared temperature monitoring functions
typedef struct {
    bool initialized;
    OW ow;
    uint64_t rom_codes[8];
    int sensor_count;
    float temperatures[8];
    bool sensor_valid[8];
    uint32_t last_conversion_time;
} TempShared;

// Initialize the shared temperature system
bool temp_shared_init(void);

// Search for DS18B20 sensors
int temp_shared_search_sensors(void);

// Start temperature conversion on all sensors
void temp_shared_start_conversion(void);

// Read temperature from a specific sensor by ROM code
float temp_shared_read_by_rom(uint64_t rom_code);

// Read temperatures from all sensors
bool temp_shared_read_all(void);

// Get sensor count
int temp_shared_get_sensor_count(void);

// Get ROM code by index
uint64_t temp_shared_get_rom_by_index(int index);

// Get temperature by index
float temp_shared_get_temp_by_index(int index);

// Check if sensor is valid by index
bool temp_shared_is_sensor_valid(int index);

// Get the shared instance (for advanced usage)
TempShared* temp_shared_get_instance(void);

#endif // TEMP_SHARED_H