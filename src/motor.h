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
} Stepper;

/**
 * @brief Initialize the stepper motor interface.
 *
 * Sets up GPIO pins according to the provided parameters
 * and initializes the Stepper state to defaults (position = 0,
 * direction = CW, delay = 0).
 *
 * @param m Pointer to the Stepper instance to initialize.
 * @param dir_pin GPIO pin for direction control.
 * @param pulse_pin GPIO pin for step pulses.
 * @param cw_val Logic level for clockwise rotation.
 * @param ccw_val Logic level for counter-clockwise rotation.
 * @param enable_pin GPIO pin to enable/disable the motor driver.
 */
void stepper_init(Stepper *m,
                  uint dir_pin,
                  uint pulse_pin,
                  uint8_t cw_val,
                  uint8_t ccw_val,
                  uint enable_pin);

/**
 * @brief Advance the motor one step in the current direction.
 *
 * Toggles the pulse pin to generate one step pulse,
 * updates the position counter, and ensures the driver
 * is enabled during stepping.
 *
 * @param m Pointer to the Stepper instance representing the motor.
 */
void stepper_move(Stepper *m);

/**
 * @brief Disable the motor driver and clear outputs.
 *
 * Sets the pulse output low and disables the driver to
 * reduce power consumption and prevent further steps.
 *
 * @param m Pointer to the Stepper instance to disable.
 */
void stepper_close(Stepper *m);

#endif // MOTOR_H

