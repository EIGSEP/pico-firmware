#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include "eigsep_command.h"
#include "hardware/i2c.h"

#ifdef __cplusplus
    #include "bno08x.h"          // real C++ class
    typedef BNO08x ImuDrv_t;     // usable as a value inside structs
#else
    typedef void *ImuDrv_t;      // opaque handle for C code
#endif

/* ------------------------------------------------------------------ */
/* Board-specific hardware constants                                  */
/* ------------------------------------------------------------------ */
#define I2C_BAUDRATE      400000
#define SAMPLE_PERIOD     10      /* ms */
#define IMU_ADDR          0x4A

#define IMU1_SDA_GPIO     0
#define IMU1_SCL_GPIO     1
#define IMU1_RST_GPIO     13
#define IMU1_I2C          i2c0

#define IMU2_SDA_GPIO     18
#define IMU2_SCL_GPIO     19
#define IMU2_RST_GPIO     21
#define IMU2_I2C          i2c1

#define IMU_NAME_LEN      8

/* ------------------------------------------------------------------ */
/* Data structures                                                    */
/* ------------------------------------------------------------------ */
typedef struct {
    float q[4];
    float a[3];
    float la[3];
    float g[3];
    float m[3];
    float grav[3];
    int   accel_status;
    int   mag_status;
} ImuData;

typedef struct {
    char          name[IMU_NAME_LEN];
    i2c_inst_t   *i2c;
    uint          sda_pin;
    uint          scl_pin;
    uint          rst_pin;
    ImuDrv_t      imu;            /* BNO08x in C++, opaque in C   */
    bool          is_initialized;
    bool          do_calibration;
    ImuData       sensor_data;
} EigsepImu;

/* ------------------------------------------------------------------ */
/* Function prototypes – keep them C-linkable                         */
/* ------------------------------------------------------------------ */
#ifdef __cplusplus
extern "C" {
#endif

void imu_init(uint8_t app_id);
void calibrate_imu(EigsepImu *eimu);
void imu_server(uint8_t app_id, const char *json_str);
void process_imu_events(EigsepImu *eimu);
void imu_op(uint8_t app_id);
void send_imu_report(uint8_t app_id, EigsepImu *eimu);
void imu_status(uint8_t app_id);

#ifdef __cplusplus
}   /* extern "C" */
#endif

#endif /* IMU_H */
