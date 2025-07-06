#include "temp_simple.h"
#include "pico/stdlib.h"
#include "onewire_library.pio.h"
#include "ow_rom.h"
#include <math.h>

void temp_sensor_init(TempSensor *sensor, uint gpio_pin, PIO pio, uint sm_offset) {
    sensor->gpio_pin = gpio_pin;
    sensor->temperature = 0.0;
    sensor->last_conversion_time = 0;
    sensor->conversion_started = false;
    sensor->read_error = false;
    
    // Initialize OneWire for this sensor
    ow_init(&sensor->ow, pio, sm_offset, gpio_pin);

    // Start initial conversion
    temp_sensor_start_conversion(sensor);
}

void temp_sensor_start_conversion(TempSensor *sensor) {
    // Reset and send convert command
    if (ow_reset(&sensor->ow)) {
        ow_send(&sensor->ow, OW_SKIP_ROM);  // Skip ROM since only one device
        ow_send(&sensor->ow, DS18B20_CONVERT_T);
        sensor->last_conversion_time = to_ms_since_boot(get_absolute_time());
        sensor->conversion_started = true;
    }
}

void temp_sensor_read(TempSensor *sensor) {
    // Check if enough time has passed since conversion start
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if ((now - sensor->last_conversion_time) < 750) {
        return;
    }
    
    // Reset and read scratchpad
    if (!ow_reset(&sensor->ow)) {
        sensor->read_error = true;
        return;
    }
    
    ow_send(&sensor->ow, OW_SKIP_ROM);  // Skip ROM since only one device
    ow_send(&sensor->ow, DS18B20_READ_SCRATCHPAD);
    
    // Read 9 bytes of scratchpad
    uint8_t data[9];
    for (int i = 0; i < 9; i++) {
        data[i] = ow_read(&sensor->ow);
    }
    
    
    // Convert to temperature
    int16_t raw_temp = (data[1] << 8) | data[0];
    float temp = raw_temp / 16.0;
    
    // Check for valid temperature range and NaN
    if (isnan(temp) || temp < -55.0 || temp > 125.0) {
        sensor->read_error = true;
        return;
    }
    
    sensor->temperature = temp;
    sensor->conversion_started = false;  // Ready for next conversion
    sensor->read_error = false;  // Successful read
}

float temp_sensor_get_temp(TempSensor *sensor) {
    return sensor->temperature;
}

float temp_sensor_get_conversion_time(TempSensor *sensor) {
    return sensor->last_conversion_time;
}

bool temp_sensor_has_error(TempSensor *sensor) {
    return sensor->read_error;
}
