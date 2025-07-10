#include "motor.h"
#include "pico/stdlib.h"
#include "cJSON.h"
#include <stdlib.h>

#ifndef MIN                   // avoid double-definition
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

static Stepper azimuth;
static Stepper elevation;

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
 * @param enable_pin GPIO pin number used to enable or disable the driver.
 */
void stepper_init(Stepper *m,
                uint dir_pin, uint pulse_pin, uint enable_pin,
                uint8_t cw_val) {
    m->direction_pin = dir_pin;
    m->pulse_pin     = pulse_pin;
    m->enable_pin    = enable_pin;
    m->cw_val        = cw_val;
    m->delay_us      = DEFAULT_DELAY_US;  // pause between delays 
    m->throttle_delay_us= THROTTLE_FACTOR * DEFAULT_DELAY_US; // slow direction changes
    m->position      = 0;
    m->dir           = 0;
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
    gpio_put(m->pulse_pin, 0);
    stepper_disable(m);
}

void motor_init(uint8_t app_id) {
    stepper_init(&azimuth, AZ_DIR_PIN, AZ_PUL_PIN, AZ_EN_PIN, AZ_CW_VAL);
    stepper_init(&elevation, EL_DIR_PIN, EL_PUL_PIN, EL_EN_PIN, EL_CW_VAL);
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
void stepper_tick(Stepper *m, int extra_delay_us) {
    gpio_put(m->pulse_pin, 1);
    sleep_us(m->delay_us); // keep active timing tight
    gpio_put(m->pulse_pin, 0);
    sleep_us(m->delay_us + extra_delay_us); // to throttle start/stop
    // Update position
    m->position += m->dir;
}

void stepper_op(Stepper *m) {
    int extra_delay_us = 0;
    if (m->remaining_steps > 0) {
        if (m->dir != 1) m->throttle_delay_us = 2 * THROTTLE_FACTOR * m->delay_us; // slow direction change
        m->dir = 1;
        gpio_put(m->direction_pin, m->cw_val);
    } else if (m->remaining_steps < 0) {
        if (m->dir != -1) m->throttle_delay_us = 2 * THROTTLE_FACTOR * m->delay_us; // slow direction change
        m->dir = -1;
        gpio_put(m->direction_pin, !m->cw_val);
    } else {
        m->dir = 0;
        return;
    }
    m->throttle_delay_us /= 2;
    m->throttle_delay_us = m->throttle_delay_us > m->delay_us ? m->throttle_delay_us : m->delay_us;

    int nsteps = MIN(m->max_pulses, abs(m->remaining_steps));
    stepper_enable(m);
    for (int i = 0; i < nsteps; i++) {
        stepper_tick(m, extra_delay_us);
    }
    stepper_disable(m);
    m->remaining_steps -= nsteps * m->dir;
}
	

/**
 * @brief Disable the stepper motor and clear outputs.
 *
 * Sets the pulse output low and disables the motor driver
 * to reduce power consumption and hold torque.
 *
 * @param m Pointer to the Stepper instance to disable.
 */
void stepper_enable(Stepper *m) {
    gpio_put(m->enable_pin, 0);
}

void stepper_disable(Stepper *m) {
    gpio_put(m->enable_pin, 1);
}

// cmd is a JSON command string with pulses and delay_us for az/el
void motor_server(uint8_t app_id, const char *json_str) {
    int32_t az_pulses, el_pulses;
    int32_t az_pos, el_pos;
    uint32_t delay_us_az, delay_us_el;

    cJSON *root = cJSON_Parse(json_str);
    cJSON *az_set_pos_json = cJSON_GetObjectItem(root, "az_set_pos");
    az_pos = az_set_pos_json ? az_set_pos_json->valueint : azimuth.position;
    cJSON *el_set_pos_json = cJSON_GetObjectItem(root, "el_set_pos");
    el_pos = el_set_pos_json ? el_set_pos_json->valueint : elevation.position;

    cJSON *az_add_pulses_json = cJSON_GetObjectItem(root, "az_add_pulses");
    if (az_add_pulses_json) {
        az_pulses = az_add_pulses_json->valueint;
        // Sending increment of zero halts motor immediately
        if (az_pulses == 0) azimuth.remaining_steps = 0;
        azimuth.remaining_steps += az_pulses;
    }
    cJSON *el_add_pulses_json = cJSON_GetObjectItem(root, "el_add_pulses");
    if (el_add_pulses_json) {
        el_pulses = el_add_pulses_json->valueint;
        // Sending increment of zero halts motor immediately
        if (el_pulses == 0) elevation.remaining_steps = 0;
        elevation.remaining_steps += el_pulses;
    }
        
    cJSON *az_dly_us_json = cJSON_GetObjectItem(root, "az_delay_us");
    azimuth.delay_us = az_dly_us_json ? az_dly_us_json->valueint : azimuth.delay_us;
    cJSON *el_dly_us_json = cJSON_GetObjectItem(root, "el_delay_us");
    elevation.delay_us = el_dly_us_json ? el_dly_us_json->valueint : elevation.delay_us;
}


void motor_status(uint8_t app_id) {
	send_json(11,
        KV_STR, "sensor_name", "motor",
        KV_STR, "status", "update",
        KV_INT, "app_id", app_id,
        KV_INT, "az_pos", azimuth.position,
        KV_INT, "az_dir", azimuth.dir,
        KV_INT, "az_remaining_steps", azimuth.remaining_steps,
        KV_INT, "az_max_pulses", azimuth.max_pulses,
        KV_INT, "el_pos", elevation.position,
        KV_INT, "el_dir", elevation.dir,
        KV_INT, "el_remaining_steps", elevation.remaining_steps,
        KV_INT, "el_max_pulses", elevation.max_pulses
    );
}

void motor_op(uint8_t app_id) {
	// move the stepper motors max_move steps
    stepper_op(&elevation);
    stepper_op(&azimuth);
}
