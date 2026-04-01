#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>
#include "eigsep_command.h"

/* ------------------------------------------------------------------ */
/* Hardware constants                                                  */
/* ------------------------------------------------------------------ */
#define IMU_UART        uart0
#define IMU_UART_RX_PIN 1
#define IMU_UART_BAUD   115200
#define IMU_RST_GPIO    13
#define IMU_NAME_LEN    32

/* BNO08x RVC packet: 2-byte header + 17 bytes payload */
#define RVC_PACKET_SIZE 19
#define RVC_HEADER_BYTE 0xAA

/* If no valid packets arrive within this window, assume the BNO08x has
   crashed or been power-cycled and trigger re-initialization. */
#define IMU_EVENT_TIMEOUT_MS 5000

/* Scaling constants for RVC packet fields */
#define RVC_ANGLE_SCALE   100.0f      /* centidegrees -> degrees   */
#define RVC_ACCEL_SCALE   1000.0f     /* milli-g -> g              */
#define GRAVITY_EARTH     9.80665f    /* m/s^2                     */

/* ------------------------------------------------------------------ */
/* Data structures                                                    */
/* ------------------------------------------------------------------ */
typedef struct {
    float yaw;
    float pitch;
    float roll;
    float accel_x;
    float accel_y;
    float accel_z;
} RvcData;

typedef struct {
    char      name[IMU_NAME_LEN];
    RvcData   data;
    bool      is_initialized;
    uint32_t  last_event_time;    /* ms since boot of last valid packet */
    /* Partial-packet receive buffer */
    uint8_t   rx_buf[RVC_PACKET_SIZE];
    uint8_t   rx_pos;
} ImuState;

/* ------------------------------------------------------------------ */
/* Function prototypes                                                */
/* ------------------------------------------------------------------ */
void imu_init(uint8_t app_id);
void imu_server(uint8_t app_id, const char *json_str);
void imu_op(uint8_t app_id);
void imu_status(uint8_t app_id);

#endif /* IMU_H */
