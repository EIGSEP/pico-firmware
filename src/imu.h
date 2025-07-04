#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include "eigsep_command.h"

#ifdef __cplusplus
extern "C" {
#endif

void imu_init(uint8_t app_id);
void imu_server(uint8_t app_id, const char *json_str);
void imu_op(uint8_t app_id);
void imu_status(uint8_t app_id);

#ifdef __cplusplus
}
#endif

#endif // IMU_H