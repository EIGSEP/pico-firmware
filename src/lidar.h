#ifndef LIDAR_H
#define LIDAR_H

#include <stdint.h>

void lidar_init(uint8_t app_id);
void lidar_server(uint8_t app_id, const char *json_str);
void lidar_status(uint8_t app_id);
void lidar_op(uint8_t app_id);

#endif // LIDAR_H