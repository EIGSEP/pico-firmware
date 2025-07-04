#ifndef PICO_MULTI_H
#define PICO_MULTI_H

#include "eigsep_command.h"

// DIP switch GPIO pins
#define DIP0_PIN 2
#define DIP1_PIN 3
#define DIP2_PIN 4

// LED GPIO pin
#define LED_PIN PICO_DEFAULT_LED_PIN

// Mapping of DIP switches to APPs
#define APP_MOTOR       3
#define APP_TEMPCTRL    1
#define APP_TEMPMON     2
#define APP_IMU         0
#define APP_LIDAR       4
#define APP_RFSWITCH    5

// status reporting cadence
#define STATUS_CADENCE_MS 200

#endif // PICO_MULTI_H
