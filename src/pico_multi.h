#ifndef PICO_MULTI_H
#define PICO_MULTI_H

#include "eigsep_command.h"

// DIP switch GPIO pins
#define DIP0_PIN 20 
#define DIP1_PIN 21 
#define DIP2_PIN 22 

// LED GPIO pin
#define LED_PIN PICO_DEFAULT_LED_PIN

// Mapping of DIP switches to APPs
#define APP_MOTOR       0
#define APP_TEMPCTRL    1
#define APP_POTMON      2
#define APP_IMU         3
#define APP_LIDAR       4
#define APP_RFSWITCH    5

// status reporting cadence
#define STATUS_CADENCE_MS 200

// Maximum time (µs) to spend reading serial before running app_op().
// The main loop prioritizes draining the serial FIFO (via continue) so
// that slow app_op() implementations (e.g. motor stepping at ~144-288ms)
// don't block command receipt.  However, if serial data arrives
// continuously without a newline terminator, op() would be starved
// indefinitely.  This threshold guarantees op() runs at least every
// MAX_READ_ONLY_US regardless of input.
#define MAX_READ_ONLY_US 50000  // 50 ms

#endif // PICO_MULTI_H
