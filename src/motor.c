/**
 * @file motor.c
 * @brief Driver functions for controlling a stepper motor on Raspberry Pi Pico.
 *
 * These routines initialize the GPIO pins, step the motor, and
 * cleanly disable the motor when done. The stepper motor's position
 * is tracked in the Stepper struct.
 */

#include "motor.h"
#include "pico/stdlib.h"

/**
 * @brief Initialize a stepper motor interface.
 *
 * Configures the GPIO pins for direction, pulse, and enable
 * and sets initial motor state values.
 *
 * @param m Pointer to the Stepper instance to initialize.
 * @param dir_pin GPIO pin number used for direction control.
 * @param pulse_pin GPIO pin number used for pulse (step) control.
 * @param cw_val Logical value to set on dir_pin for clockwise rotation.
 * @param ccw_val Logical value to set on dir_pin for counter-clockwise rotation.
 * @param enable_pin GPIO pin number used to enable or disable the driver.
 */
void stepper_init(Stepper *m,
                  uint dir_pin, uint pulse_pin,
                  uint8_t cw_val, uint8_t ccw_val,
                  uint enable_pin) {
    m->direction_pin = dir_pin;
    m->pulse_pin     = pulse_pin;
    m->enable_pin    = enable_pin;
    m->cw_val        = cw_val;
    m->ccw_val       = ccw_val;
    m->delay_us      = 0;
    m->position      = 0;
    m->dir           = 1;

    gpio_init(dir_pin);
    gpio_set_dir(dir_pin, GPIO_OUT);

    gpio_init(pulse_pin);
    gpio_set_dir(pulse_pin, GPIO_OUT);

    gpio_init(enable_pin);
    gpio_set_dir(enable_pin, GPIO_OUT);

    /* Disable motor by default and ensure pulse pin is low */
    gpio_put(enable_pin, 1);
    gpio_put(pulse_pin, 0);
}

/**
 * @brief Perform one step of the motor in the currently set direction.
 *
 * Toggles the pulse pin to advance the motor one increment,
 * updates the internal position counter, and ensures the driver
 * is enabled for the pulse duration.
 *
 * @param m Pointer to the Stepper instance representing the motor.
 */
void stepper_move(Stepper *m) {
    /* Set direction pin and update position */
    if (m->dir > 0) {
        gpio_put(m->direction_pin, m->cw_val);
        m->position++;
    } else {
        gpio_put(m->direction_pin, m->ccw_val);
        m->position--;
    }

    /* Enable driver, send pulse, then disable */
    gpio_put(m->enable_pin, 0);
    gpio_put(m->pulse_pin, 1);
    sleep_us(m->delay_us);
    gpio_put(m->pulse_pin, 0);
    sleep_us(m->delay_us);
}

/**
 * @brief Disable the stepper motor and clear outputs.
 *
 * Sets the pulse output low and disables the motor driver
 * to reduce power consumption and hold torque.
 *
 * @param m Pointer to the Stepper instance to disable.
 */
void stepper_close(Stepper *m) {
    gpio_put(m->pulse_pin, 0);
    gpio_put(m->enable_pin, 1);
}

