/**
 * @file motor.h
 * @brief Header definitions for controlling a stepper motor on Raspberry Pi Pico.
 *
 * This header declares the Stepper struct which encapsulates GPIO pins,
 * motion parameters, and state information for a stepper motor driver.
 * It also provides function prototypes for initializing, moving, and
 * shutting down the stepper interface.
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "hardware/gpio.h"
#include "eigsep_command.h"

/**
 * @param m Pointer to the Stepper instance to initialize.
 * @param dir_pin GPIO pin for direction control.
 * @param pulse_pin GPIO pin for step pulses.
 * @param cw_val Logic level for clockwise rotation.
 * @param ccw_val Logic level for counter-clockwise rotation.
 * @param enable_pin GPIO pin to enable/disable the motor driver.
 */

// define motor pins
#define  EL_EN_PIN 5
#define EL_DIR_PIN 6
#define EL_PUL_PIN 7
#define  EL_CW_VAL 0

#define  AZ_EN_PIN 8
#define AZ_DIR_PIN 9
#define AZ_PUL_PIN 10
#define  AZ_CW_VAL 0

#define DEFAULT_DELAY_US 600
#define SLOWDOWN_FACTOR 2
#define SLOW_ZONE 100

/**
 * @struct Stepper
 * @brief Represents a stepper motor interface and its current state.
 */
typedef struct {
    uint    direction_pin; /**< GPIO pin for rotation direction */
    uint    pulse_pin;     /**< GPIO pin for step pulses */  
    uint    enable_pin;    /**< GPIO pin for driver enable */   
    uint8_t cw_val;        /**< Logic level for clockwise direction */      
    uint32_t delay_us;     /**< pulse width in microseconds and delay between steps */    
    uint32_t slowdown_factor; /**< extra multiplier on delay between steps */    
    uint32_t slow_zone;
    uint32_t steps_in_direction;
    int32_t position;      /**< Current motor position in steps */     
    int8_t  dir;           /**< Current direction flag (1 = CW, -1 = CCW) */         
    int32_t target_pos;
    int32_t max_pulses;    /**< Maximum steps to move in current operation */
} Stepper;

// report motor status
void motor_init(uint8_t);
void motor_server(uint8_t, const char *);
void motor_op(uint8_t);
void motor_status(uint8_t);
void stepper_op(Stepper *);
void stepper_disable(Stepper *);
void stepper_enable(Stepper *);

#endif // MOTOR_H

