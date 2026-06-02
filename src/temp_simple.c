#include "temp_simple.h"
#include "pico/stdlib.h"
#include "onewire_library.pio.h"
#include "ow_rom.h"
#include <math.h>
#include <stddef.h>

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

// Dallas/Maxim 1-Wire CRC-8 (polynomial X^8 + X^5 + X^4 + 1, reflected 0x8C,
// processed LSB-first). The DS18B20 stores the CRC of scratchpad bytes 0..7 in
// byte 8, so a frame corrupted on the wire (marginal pull-up, line noise) is
// caught here even though its decoded value may land inside the valid range.
static uint8_t ow_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        for (int b = 0; b < 8; b++) {
            uint8_t mix = (crc ^ byte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            byte >>= 1;
        }
    }
    return crc;
}

bool temp_sensor_read(TempSensor *sensor) {
    // Check if enough time has passed since conversion start
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if ((now - sensor->last_conversion_time) < DS18B20_CONVERSION_TIME_MS) {
        return false;
    }

    // Reset and read scratchpad
    if (!ow_reset(&sensor->ow)) {
        sensor->read_error = true;
        return false;
    }

    ow_send(&sensor->ow, OW_SKIP_ROM);  // Skip ROM since only one device
    ow_send(&sensor->ow, DS18B20_READ_SCRATCHPAD);

    // Read 9 bytes of scratchpad
    uint8_t data[9];
    for (int i = 0; i < 9; i++) {
        data[i] = ow_read(&sensor->ow);
    }

    // An all-zero scratchpad has a valid (zero) CRC and decodes to a plausible
    // 0.0 C, so a stuck-low bus would slip past the CRC check below. Reject it
    // explicitly before trusting any byte.
    bool all_zero = true;
    for (int i = 0; i < 9; i++) {
        if (data[i] != 0x00) { all_zero = false; break; }
    }
    if (all_zero) {
        sensor->read_error = true;
        return false;
    }

    // Reject frames corrupted in transit before trusting any byte.
    if (ow_crc8(data, 8) != data[8]) {
        sensor->read_error = true;
        return false;
    }

    // Convert to temperature
    int16_t raw_temp = (data[1] << 8) | data[0];
    float temp = raw_temp / 16.0;

    // Check for valid temperature range and NaN
    if (isnan(temp) || temp < -55.0 || temp > 125.0) {
        sensor->read_error = true;
        return false;
    }

    sensor->temperature = temp;
    sensor->conversion_started = false;  // Ready for next conversion
    sensor->read_error = false;  // Successful read
    return true;
}

float temp_sensor_get_temp(TempSensor *sensor) {
    return sensor->temperature;
}

uint32_t temp_sensor_get_conversion_time(TempSensor *sensor) {
    return sensor->last_conversion_time;
}

bool temp_sensor_has_error(TempSensor *sensor) {
    return sensor->read_error;
}
