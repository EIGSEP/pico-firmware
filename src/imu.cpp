#include "imu.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "cJSON.h"
#include "bno08x.h"

#define I2C_BAUDRATE 400000
// sample rate in ms
#define SAMPLE_PERIOD 50
#define IMU_ADDR 0x4A

static BNO08x imu1;
static bool imu_initialized = false;
static bool calibration_requested = false;

// Last sensor data
static struct {
    float q[4];
    float a[3];
    float la[3];
    float g[3];
    float m[3];
    float grav[3];
    int accel_status;
    int mag_status;
} sensor_data;

static void init_i2c_bus(i2c_inst_t *i2c, uint sda, uint scl) {
    i2c_init(i2c, I2C_BAUDRATE);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_pull_up(scl);
}

static void enable_imu_features(BNO08x& imu) {
    imu.enableRotationVector(SAMPLE_RATE);
    imu.enableAccelerometer(SAMPLE_RATE);
    imu.enableLinearAccelerometer(SAMPLE_RATE);
    imu.enableGyro(SAMPLE_RATE);
    imu.enableMagnetometer(SAMPLE_RATE);
    imu.enableGravity();
}

void imu_init(uint8_t app_id) {
    memset(&sensor_data, 0, sizeof(sensor_data));
    init_i2c_bus(i2c0, 0, 1);
    
    // Initialize IMU
    if (imu1.begin(IMU_ADDR, i2c0)) {
        enable_imu_features(imu1);
        imu_initialized = true;
    }
}

void calibrate_imu() {
    if (imu_initialized) {
        sensor_data.accel_status = -1;
        sensor_data.mag_status = -1;        
        absolute_time_t deadline = make_timeout_time_ms(SAMPLE_RATE);
        while (!time_reached(deadline)) {
            if (!imu1.getSensorEvent()) continue;
            sh2_SensorValue_t event = imu1.sensorValue;
            switch (event.sensorId) {
                case SENSOR_REPORTID_ACCELEROMETER:
                    sensor_data.accel_status = event.status;
                    break;
                case SENSOR_REPORTID_MAGNETIC_FIELD:
                    sensor_data.mag_status = event.status;
                    break;
            }
                    
            // exit early if both sensors are calibrated
            if (sensor_data.accel_status == 3 && sensor_data.mag_status == 3) break;
        }
                
        if (sensor_data.accel_status >= 3 && sensor_data.mag_status >= 3) {
            imu1.saveCalibration();
        }
    }
}

// calibrate if user sends {calibrate: true} in JSON
void imu_server(uint8_t app_id, const char *json_str) {
    cJSON *root = cJSON_Parse(json_str);
    if (root == NULL) {
        // Invalid JSON input, exit early
        calibration_requested = false;
        return;
    }
    cJSON *cal = cJSON_GetObjectItem(root, "calibrate");
    if (cal && cJSON_IsTrue(cal)) {
        calibration_requested = true;
    }
    else {
        calibration_requested = false;
    } 
    cJSON_Delete(root);
}

void imu_op(uint8_t app_id) {
    if (!imu_initialized) return;
    
    // Handle calibration request
    if (calibration_requested) {
        calibrate_imu();
        calibration_requested = false;
    }
    
    // Read sensor events
    while (imu1.getSensorEvent()) {
        sh2_SensorValue_t event = imu1.sensorValue;
        switch (event.sensorId) {
            case SENSOR_REPORTID_ROTATION_VECTOR:
                sensor_data.q[0] = event.un.rotationVector.i;
                sensor_data.q[1] = event.un.rotationVector.j;
                sensor_data.q[2] = event.un.rotationVector.k;
                sensor_data.q[3] = event.un.rotationVector.real;
                break;
            case SENSOR_REPORTID_ACCELEROMETER:
                sensor_data.a[0] = event.un.accelerometer.x;
                sensor_data.a[1] = event.un.accelerometer.y;
                sensor_data.a[2] = event.un.accelerometer.z;
                sensor_data.accel_status = event.status;
                break;
            case SENSOR_REPORTID_LINEAR_ACCELERATION:
                sensor_data.la[0] = event.un.linearAcceleration.x;
                sensor_data.la[1] = event.un.linearAcceleration.y;
                sensor_data.la[2] = event.un.linearAcceleration.z;
                break;
            case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:
                sensor_data.g[0] = event.un.gyroscope.x;
                sensor_data.g[1] = event.un.gyroscope.y;
                sensor_data.g[2] = event.un.gyroscope.z;
                break;
            case SENSOR_REPORTID_MAGNETIC_FIELD:
                sensor_data.m[0] = event.un.magneticField.x;
                sensor_data.m[1] = event.un.magneticField.y;
                sensor_data.m[2] = event.un.magneticField.z;
                sensor_data.mag_status = event.status;
                break;
            case SENSOR_REPORTID_GRAVITY:
                sensor_data.grav[0] = event.un.gravity.x;
                sensor_data.grav[1] = event.un.gravity.y;
                sensor_data.grav[2] = event.un.gravity.z;
                break;
        }
    }
}

void imu_status(uint8_t app_id) {
    const char *status;
    if (!imu_initialized) {
        status = "error";
    }
    else {
        status = "update";
    }
    
    send_json(20,
        KV_STR, "status", status,
        KV_INT, "app_id", app_id,
        KV_FLOAT, "quat_i", sensor_data.q[0],
        KV_FLOAT, "quat_j", sensor_data.q[1],
        KV_FLOAT, "quat_k", sensor_data.q[2],
        KV_FLOAT, "quat_real", sensor_data.q[3],
        KV_FLOAT, "accel_x", sensor_data.a[0],
        KV_FLOAT, "accel_y", sensor_data.a[1],
        KV_FLOAT, "accel_z", sensor_data.a[2],
        KV_FLOAT, "lin_accel_x", sensor_data.la[0],
        KV_FLOAT, "lin_accel_y", sensor_data.la[1],
        KV_FLOAT, "lin_accel_z", sensor_data.la[2],
        KV_FLOAT, "gyro_x", sensor_data.g[0],
        KV_FLOAT, "gyro_y", sensor_data.g[1],
        KV_FLOAT, "gyro_z", sensor_data.g[2],
        KV_FLOAT, "mag_x", sensor_data.m[0],
        KV_FLOAT, "mag_y", sensor_data.m[1],
        KV_FLOAT, "mag_z", sensor_data.m[2],
        KV_INT, "accel_cal", sensor_data.accel_status,
        KV_INT, "mag_cal", sensor_data.mag_status
    );
}
