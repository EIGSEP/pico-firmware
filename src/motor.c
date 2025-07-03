#include "motor.h"
#include "pico/stdlib.h"
#include "cJSON.h"


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
    m->delay_us      = 600;  // pause between delays 
    m->position      = 0;
    m->dir           = 1;
    // controlling steps
    m->remaining_steps = 0;  // Initialize remaining steps to 0
    m->max_pulses    = 60;  // pulses per command, ~1 deg

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
void one_step(Stepper *m) {
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

// cmd is a JSON command string with pulses and delay_us for az/el
void motor_server(Stepper *azimuth, Stepper *elevation, const char *json_str) {
    int32_t pulses_az, pulses_el;
    uint32_t delay_us;

    // Parse the JSON command string
    cJSON *root = cJSON_Parse(json_str);
    pulses_az = cJSON_GetObjectItem(root, "pulses_az")->valueint;
    pulses_el = cJSON_GetObjectItem(root, "pulses_el")->valueint;
    delay_us_az = cJSON_GetObjectItem(root, "delay_us_az")->valueint;
    delay_us_el = cJSON_GetObjectItem(root, "delay_us_el")->valueint;
    cJSON_Delete(root);

    // update the stepper motors
    azimuth->remaining_steps += pulses_az;
    elevation->remaining_steps += pulses_el;
    azimuth->delay_us = delay_us;
    elevation->delay_us = delay_us;
}


void motor_status(long azimuth_pos, long elevation_pos) {
	pos_az = "%ld", azimuth_pos;
	pos_el = "%ld", elevation_pos;
	send_json(2, KV_STR, "azimuth_pos", pos_az,
		  KV_STR, "elevation_pos", pos_el);
}

void motor_op(Stepper *azimuth, Stepper *elevation} {
	
	uint32_t az_remaining = azimuth.remaining_steps;
	uint32_t el_remaining = elevation.remaining_steps;
	// move the stepper motors max_move steps
        elevation.dir = el_remaining > 0 ? 1 : -1;
        azimuth.dir = az_remaining > 0 ? 1 : -1;

	// azimuth loop
	for (int i = 0; i < azimuth.max_pulses; i++} {
		if abs(az_remaining) == 0 {
		break;
		}
		stepper_move(&azimuth);
		az_remaining -= azimuth.dir;
	}
	azimuth->remaining_steps = az_remaining;
	
	// elevation loop
	for (int i = 0; i < elevation.max_pulses; i++} {
		if abs(el_remaining) == 0 {
		break;
		}
		stepper_move(&elevation);
		el_remaining -= elevation.dir;
	}
	elevation->remaining_steps = el_remaining;

	// report position
	motor_status(long azimuth.position, long elevation.position);
	
        // Disable coils until next command
        stepper_close(&elevation);
        stepper_close(&azimuth);

