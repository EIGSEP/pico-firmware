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


static void free_i2c_bus() {
    /* Temporarily bit-bang SCL as a GPIO to clock out any half-byte */
    gpio_set_function(I2C_SCL, GPIO_FUNC_SIO);
    gpio_set_dir(I2C_SCL, GPIO_OUT);
    gpio_set_function(I2C_SDA, GPIO_FUNC_SIO);
    gpio_set_dir(I2C_SDA, GPIO_IN);          // SDA as input (pull-up already on)

    for (int i = 0; i < 9; ++i) {        // 9 clocks releases most slaves
        gpio_put(I2C_SCL, 0); sleep_us(5);
        gpio_put(I2C_SCL, 1); sleep_us(5);
        if (gpio_get(I2C_SDA)) break;        // bus released
    }

    /* Restore pins to I²C mode */
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
}


void lidar_init(uint8_t app_id) {
    init_i2c();
    free_i2c_bus();
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

bool sf30d_init(void) {
    const uint8_t start[2] = {0x00, 0x04};
    if (i2c_write_timeout_us(I2C_PORT, I2C_ADDR, start, 2, false, 1000) < 0)
        return false;
    //sleep_ms(50);
    sleep_ms(100);
    return true;
}

bool lidar_reset(uint8_t app_id) {
    i2c_deinit(I2C_PORT);
    sleep_ms(100);
    lidar_init(app_id);
}

void lidar_op(uint8_t app_id) {
    uint8_t buf[2];
    //lidar_init(app_id);
    if (i2c_read_timeout_us(I2C_PORT, I2C_ADDR, buf, 2, false, 1000) != 2) {
        lidar_reset(app_id);
        //free_i2c_bus();                 // bus recovery
        //sf30d_init();                   // re-issue start cmd
        return;
    }

    uint16_t dist_cm = (uint16_t)(buf[0] << 8) | buf[1];
    if (dist_cm == 0) {                  // still not ready – ignore
        lidar_reset(app_id);
    //    return;
    }
    lidar_data.distance = dist_cm / 100.0;
}


