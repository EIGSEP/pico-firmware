#include "imu.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "cJSON.h"
#include "bno08x.h"

#define I2C_BAUDRATE 400000
// sample rate in ms
#define SAMPLE_PERIOD 10
#define IMU_ADDR 0x4A

//static EigsepImu imu_panda;
static EigsepImu imu_antenna;

static void init_i2c_bus(i2c_inst_t *i2c, uint sda_pin, uint scl_pin) {
    i2c_init(i2c, I2C_BAUDRATE);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
}

static void free_i2c_bus(i2c_inst_t *i2c, uint sda_pin, uint scl_pin)
{
    /* Temporarily bit-bang SCL as a GPIO to clock out any half-byte */
    gpio_set_function(scl_pin, GPIO_FUNC_SIO);
    gpio_set_dir(scl_pin, GPIO_OUT);
    gpio_set_function(sda_pin, GPIO_FUNC_SIO);
    gpio_set_dir(sda_pin, GPIO_IN);          // SDA as input (pull-up already on)

    for (int i = 0; i < 9; ++i) {        // 9 clocks releases most slaves
        gpio_put(scl_pin, 0); sleep_us(5);
        gpio_put(scl_pin, 1); sleep_us(5);
        if (gpio_get(sda_pin)) break;        // bus released
    }

    /* Restore pins to I²C mode */
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
}

static void init_eigsep_imu(EigsepImu *eimu, uint index) {
    eimu->is_initialized = false;
    eimu->do_calibration = false;
    if (index == 0) {
        strncpy(eimu->name, "panda", IMU_NAME_LEN - 1);
        eimu->name[IMU_NAME_LEN-1] = '\0';
        eimu->i2c = i2c0;
        eimu->sda_pin = IMU1_SDA_GPIO;
        eimu->scl_pin = IMU1_SCL_GPIO;
        eimu->rst_pin = IMU1_RST_GPIO;
    } else {
        strncpy(eimu->name, "antenna", IMU_NAME_LEN - 1);
        eimu->name[IMU_NAME_LEN-1] = '\0';
        eimu->i2c = i2c1;
        eimu->sda_pin = IMU2_SDA_GPIO;
        eimu->scl_pin = IMU2_SCL_GPIO;
        eimu->rst_pin = IMU2_RST_GPIO;
    }
    init_i2c_bus(eimu->i2c, eimu->sda_pin, eimu->scl_pin);

    gpio_init(eimu->rst_pin);
    gpio_set_dir(eimu->rst_pin, GPIO_OUT);
    gpio_put(eimu->rst_pin, 0);
    sleep_ms(2);
    gpio_put(eimu->rst_pin, 1);
    sleep_ms(600);              // tSTARTUP max = 600 ms
    // detach anyone lingering on the bus
    //free_i2c_bus(eimu->i2c, eimu->sda_pin, eimu->scl_pin);

    //if (eimu->imu.softReset()) {
    //    sleep_ms(600);
    //}

    if (eimu->imu.begin(IMU_ADDR, eimu->i2c)) {
        eimu->imu.enableRotationVector(SAMPLE_PERIOD);
        eimu->imu.enableAccelerometer(SAMPLE_PERIOD);
        eimu->imu.enableLinearAccelerometer(SAMPLE_PERIOD);
        eimu->imu.enableGyro(SAMPLE_PERIOD);
        eimu->imu.enableMagnetometer(SAMPLE_PERIOD);
        eimu->imu.enableGravity();
        eimu->is_initialized = true;
    }
}

void imu_init(uint8_t app_id) {
    //init_eigsep_imu(&imu_panda, 0);
    init_eigsep_imu(&imu_antenna, 1);
}

void calibrate_imu(EigsepImu *eimu) {
    if (!eimu->is_initialized or !eimu->do_calibration) return;
    eimu->sensor_data.accel_status = -1;
    eimu->sensor_data.mag_status = -1;        
    absolute_time_t deadline = make_timeout_time_ms(SAMPLE_PERIOD);
    while (!time_reached(deadline)) {
        if (!eimu->imu.getSensorEvent()) continue;
        sh2_SensorValue_t event = eimu->imu.sensorValue;
        switch (event.sensorId) {
            case SENSOR_REPORTID_ACCELEROMETER:
                eimu->sensor_data.accel_status = event.status;
                break;
            case SENSOR_REPORTID_MAGNETIC_FIELD:
                eimu->sensor_data.mag_status = event.status;
                break;
        }
                
        // exit early if both sensors are calibrated
        if (eimu->sensor_data.accel_status == 3 && eimu->sensor_data.mag_status == 3) break;
    }
            
    if (eimu->sensor_data.accel_status >= 3 && eimu->sensor_data.mag_status >= 3) {
        eimu->imu.saveCalibration();
    }
}

// calibrate if user sends {calibrate: true} in JSON
void imu_server(uint8_t app_id, const char *json_str) {
    cJSON *root = cJSON_Parse(json_str);
    if (root == NULL) {
        // Invalid JSON input, exit early
        return;
    }
    cJSON *cal = cJSON_GetObjectItem(root, "calibrate");
    if (cal && cJSON_IsTrue(cal)) {
        //imu_panda.do_calibration = true;
        imu_antenna.do_calibration = true;
    }
    cJSON_Delete(root);
}

void process_imu_events(EigsepImu *eimu) {
    if (!eimu->is_initialized) return;
    
    // Read sensor events
    while (eimu->imu.getSensorEvent()) {
        sh2_SensorValue_t event = eimu->imu.sensorValue;
        switch (event.sensorId) {
            case SENSOR_REPORTID_ROTATION_VECTOR:
                eimu->sensor_data.q[0] = event.un.rotationVector.i;
                eimu->sensor_data.q[1] = event.un.rotationVector.j;
                eimu->sensor_data.q[2] = event.un.rotationVector.k;
                eimu->sensor_data.q[3] = event.un.rotationVector.real;
                break;
            case SENSOR_REPORTID_ACCELEROMETER:
                eimu->sensor_data.a[0] = event.un.accelerometer.x;
                eimu->sensor_data.a[1] = event.un.accelerometer.y;
                eimu->sensor_data.a[2] = event.un.accelerometer.z;
                eimu->sensor_data.accel_status = event.status;
                break;
            case SENSOR_REPORTID_LINEAR_ACCELERATION:
                eimu->sensor_data.la[0] = event.un.linearAcceleration.x;
                eimu->sensor_data.la[1] = event.un.linearAcceleration.y;
                eimu->sensor_data.la[2] = event.un.linearAcceleration.z;
                break;
            case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:
                eimu->sensor_data.g[0] = event.un.gyroscope.x;
                eimu->sensor_data.g[1] = event.un.gyroscope.y;
                eimu->sensor_data.g[2] = event.un.gyroscope.z;
                break;
            case SENSOR_REPORTID_MAGNETIC_FIELD:
                eimu->sensor_data.m[0] = event.un.magneticField.x;
                eimu->sensor_data.m[1] = event.un.magneticField.y;
                eimu->sensor_data.m[2] = event.un.magneticField.z;
                eimu->sensor_data.mag_status = event.status;
                break;
            case SENSOR_REPORTID_GRAVITY:
                eimu->sensor_data.grav[0] = event.un.gravity.x;
                eimu->sensor_data.grav[1] = event.un.gravity.y;
                eimu->sensor_data.grav[2] = event.un.gravity.z;
                break;
        }
    }
}

void imu_op(uint8_t app_id) {
    // Handle calibration request
    //calibrate_imu(&imu_panda);
    //process_imu_events(&imu_panda);
    calibrate_imu(&imu_antenna);
    process_imu_events(&imu_antenna);
}    


void send_imu_report(uint8_t app_id, EigsepImu *eimu) {
    const char *status;
    if (!eimu->is_initialized) {
        status = "error";
    }
    else {
        status = "update";
    }
    
    send_json(21,
        KV_STR, "status", status,
        KV_STR, "imu_name", eimu->name,
        KV_INT, "app_id", app_id,
        KV_FLOAT, "quat_i", eimu->sensor_data.q[0],
        KV_FLOAT, "quat_j", eimu->sensor_data.q[1],
        KV_FLOAT, "quat_k", eimu->sensor_data.q[2],
        KV_FLOAT, "quat_real", eimu->sensor_data.q[3],
        KV_FLOAT, "accel_x", eimu->sensor_data.a[0],
        KV_FLOAT, "accel_y", eimu->sensor_data.a[1],
        KV_FLOAT, "accel_z", eimu->sensor_data.a[2],
        KV_FLOAT, "lin_accel_x", eimu->sensor_data.la[0],
        KV_FLOAT, "lin_accel_y", eimu->sensor_data.la[1],
        KV_FLOAT, "lin_accel_z", eimu->sensor_data.la[2],
        KV_FLOAT, "gyro_x", eimu->sensor_data.g[0],
        KV_FLOAT, "gyro_y", eimu->sensor_data.g[1],
        KV_FLOAT, "gyro_z", eimu->sensor_data.g[2],
        KV_FLOAT, "mag_x", eimu->sensor_data.m[0],
        KV_FLOAT, "mag_y", eimu->sensor_data.m[1],
        KV_FLOAT, "mag_z", eimu->sensor_data.m[2],
        KV_INT, "accel_cal", eimu->sensor_data.accel_status,
        KV_INT, "mag_cal", eimu->sensor_data.mag_status
    );
}

void imu_status(uint8_t app_id) {
    //send_imu_report(app_id, &imu_panda);
    send_imu_report(app_id, &imu_antenna);
}
