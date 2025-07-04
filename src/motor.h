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
#define  EL_CW_VAL 1
#define EL_CCW_VAL 19

#define  AZ_EN_PIN 8
#define AZ_DIR_PIN 9
#define AZ_PUL_PIN 10
#define  AZ_CW_VAL 1
#define AZ_CCW_VAL 10

#define DEFAULT_DELAY_US 600

/**
 * @struct Stepper
 * @brief Represents a stepper motor interface and its current state.
 *
 * @var Stepper::direction_pin
 * GPIO pin used to select the motor rotation direction.
 * @var Stepper::pulse_pin
 * GPIO pin used to send step pulses to the motor driver.
 * @var Stepper::enable_pin
 * GPIO pin used to enable or disable the motor driver.
 * @var Stepper::cw_val
 * Logical level to set on direction_pin for clockwise rotation.
 * @var Stepper::ccw_val
 * Logical level to set on direction_pin for counter-clockwise rotation.
 * @var Stepper::delay_us
 * Microsecond delay between pulse transitions (step speed control).
 * @var Stepper::position
 * Current step count position of the motor (relative origin).
 * @var Stepper::dir
 * Direction flag: positive for CW, negative for CCW.
 */
typedef struct {
    uint    direction_pin; /**< GPIO pin for rotation direction */
    uint    pulse_pin;     /**< GPIO pin for step pulses */  
    uint    enable_pin;    /**< GPIO pin for driver enable */   
    uint8_t cw_val;        /**< Logic level for clockwise direction */      
    uint8_t ccw_val;       /**< Logic level for counter-clockwise direction */      
    uint32_t delay_us;     /**< Delay in microseconds between steps */    
    int32_t position;      /**< Current motor position in steps */     
    int8_t  dir;           /**< Current direction flag (1 = CW, -1 = CCW) */         
    int32_t remaining_steps; /**< Remaining steps to move in current operation */
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

