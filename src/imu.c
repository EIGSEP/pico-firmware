#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "bno08x.h"
#include "utils.h"

#define I2C_BAUDRATE 400000
#define IMU_ADDR 0x4A

BNO08x imu1;
//BNO08x imu2;

void init_i2c_bus(i2c_inst_t *i2c, uint sda, uint scl) {
    i2c_init(i2c, I2C_BAUDRATE);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_pull_up(scl);
}

void enable_imu_features(BNO08x& imu) {
    imu.enableRotationVector();
    imu.enableAccelerometer();
    imu.enableLinearAccelerometer();
    imu.enableGyro();
    imu.enableMagnetometer();
    imu.enableGravity();
}

void print_sensor_data(BNO08x& imu, const char* label) {
    // Initialize all fields to a known invalid state
    float q[4] = {NAN, NAN, NAN, NAN};
    float a[3] = {NAN, NAN, NAN};
    float la[3] = {NAN, NAN, NAN};
    float g[3] = {NAN, NAN, NAN};
    float m[3] = {NAN, NAN, NAN};
    float grav[3] = {NAN, NAN, NAN};

    // Collect sensor data until timeout or all found
    absolute_time_t deadline = make_timeout_time_ms(100);
    while (!time_reached(deadline)) {
        if (!imu.getSensorEvent())
            continue;

        sh2_SensorValue_t event = imu.sensorValue;
        switch (event.sensorId) {
            case SENSOR_REPORTID_ROTATION_VECTOR:
                q[0] = event.un.rotationVector.i;
                q[1] = event.un.rotationVector.j;
                q[2] = event.un.rotationVector.k;
                q[3] = event.un.rotationVector.real;
                break;
            case SENSOR_REPORTID_ACCELEROMETER:
                a[0] = event.un.accelerometer.x;
                a[1] = event.un.accelerometer.y;
                a[2] = event.un.accelerometer.z;
                break;
            case SENSOR_REPORTID_LINEAR_ACCELERATION:
                la[0] = event.un.linearAcceleration.x;
                la[1] = event.un.linearAcceleration.y;
                la[2] = event.un.linearAcceleration.z;
                break;
            case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:
                g[0] = event.un.gyroscope.x;
                g[1] = event.un.gyroscope.y;
                g[2] = event.un.gyroscope.z;
                break;
            case SENSOR_REPORTID_MAGNETIC_FIELD:
                m[0] = event.un.magneticField.x;
                m[1] = event.un.magneticField.y;
                m[2] = event.un.magneticField.z;
                break;
            case SENSOR_REPORTID_GRAVITY:
                grav[0] = event.un.gravity.x;
                grav[1] = event.un.gravity.y;
                grav[2] = event.un.gravity.z;
                break;
        }
    }

    printf("[%s] ", label);
    printf("q:%.3f:%.3f:%.3f:%.3f,", q[0], q[1], q[2], q[3]);
    printf("a:%.3f:%.3f:%.3f,", a[0], a[1], a[2]);
    printf("la:%.3f:%.3f:%.3f,", la[0], la[1], la[2]);
    printf("g:%.3f:%.3f:%.3f,", g[0], g[1], g[2]);
    printf("m:%.3f:%.3f:%.3f,", m[0], m[1], m[2]);
    printf("grav:%.3f:%.3f:%.3f\n", grav[0], grav[1], grav[2]);
}

void calibrate_imu(BNO08x& imu) {
    int accel_status = -1;
    int mag_status = -1;

    absolute_time_t deadline = make_timeout_time_ms(50);

    while (!time_reached(deadline)) {
        if (!imu.getSensorEvent()) continue;

        sh2_SensorValue_t event = imu.sensorValue;

        switch (event.sensorId) {
            case SENSOR_REPORTID_ACCELEROMETER:
                accel_status = event.status;
                break;
            case SENSOR_REPORTID_MAGNETIC_FIELD:
                mag_status = event.status;
                break;
        }

        // Exit early if both are calibrated
        if (accel_status == 3 && mag_status == 3) break;
    }

    if (accel_status >= 3 && mag_status >= 3) {
        imu.saveCalibration();
    }
    printf("%d,%d\n", accel_status, mag_status);
}

int main() {
    sleep_ms(500);
    stdio_init_all();
    sleep_ms(1000);  // USB startup delay

    init_i2c_bus(i2c0, 0, 1);
    //init_i2c_bus(i2c1, 2, 3);
    char buf[16];

    while (!imu1.begin(IMU_ADDR, i2c0)) {
        if (fgets(buf, sizeof(buf), stdin)) {
            if (strncmp(buf, "REQ", 3) == 0 || strncmp(buf, "CAL", 3) == 0) {
                printf("IMU1 not detected on i2c0\n");
            }
        }
        sleep_ms(50);
    }
    //while (!imu2.begin(IMU_ADDR, i2c1)) {
    //    printf("IMU2 not detected on i2c1\n");
    //    sleep_ms(50);
    //}
    printf("IMUs Detected.\n");
    enable_imu_features(imu1);
    //enable_imu_features(imu2);

    while (true) {
        if (fgets(buf, sizeof(buf), stdin)) {
            if (strncmp(buf, "REQ", 3) == 0) {
                print_sensor_data(imu1, "IMU1");
                //print_sensor_data(imu2, "IMU2");
            } else if (strncmp(buf, "CAL", 3) == 0) {
                calibrate_imu(imu1);
                //calibrate_imu(imu2);
            }
        }
        sleep_ms(50);
    }
}
