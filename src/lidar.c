#include "lidar.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "cJSON.h"
#include "eigsep_command.h"
#include <stdio.h>

#define I2C_PORT i2c0
#define I2C_SDA 0
#define I2C_SCL 1
#define I2C_FREQ 100000
#define I2C_ADDR 0x66

static struct {
    float distance;
} lidar_data = {0};

static void init_i2c() {
    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}

void lidar_init(uint8_t app_id) {
    init_i2c();
}

void lidar_server(uint8_t app_id, const char *json_str) {}

void lidar_status(uint8_t app_id) {
    send_json(4,
        KV_STR, "sensor_name", "lidar",
        KV_STR, "status", "update",
        KV_INT, "app_id", app_id,
        KV_FLOAT, "distance_m", lidar_data.distance
    );
}

void lidar_op(uint8_t app_id) {
    uint8_t resp[2];
    int read = i2c_read_blocking(I2C_PORT, I2C_ADDR, resp, 2, false);
    
    if (read == 2) {
        int32_t dist_raw = (int32_t)(resp[0] << 8) | (resp[1]) ;
        
        lidar_data.distance = dist_raw / 100.0;
    }
}
