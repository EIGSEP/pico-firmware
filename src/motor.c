#include "motor.h"
#include "pico/stdlib.h"
#include "cJSON.h"
#include <stdlib.h>

#ifndef MIN                   // avoid double-definition
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX                   // avoid double-definition
#define MAX(a, b) ((a) > (b) ? (a) : (b))
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
    m->up_delay_us   = DEFAULT_DELAY_US;  // pause between delays 
    m->dn_delay_us   = DEFAULT_DELAY_US;  // pause between delays 
    m->slowdown_factor = SLOWDOWN_FACTOR; // slow direction changes
    m->slow_zone = SLOW_ZONE; // steps within which to slow down
    m->position      = 0;
    m->dir           = 0;
    // controlling steps
    m->target_pos    = 0;
    m->max_pulses    = 60;  // pulses per command, ~1 deg
    m->steps_in_direction = 0;

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
    sleep_us(m->up_delay_us); // keep active timing tight
    gpio_put(m->pulse_pin, 0);
    sleep_us(m->dn_delay_us + extra_delay_us); // to throttle start/stop
    // Update position
    m->position += m->dir;
}

void stepper_op(Stepper *m) {
    int new_dir;
    int remaining_steps = m->target_pos - m->position;
    int abs_steps = abs(remaining_steps);
    int nsteps = MIN(m->max_pulses, abs_steps);  // how many steps to take now
    bool near_stop = (abs_steps <= m->slow_zone);

    if      (remaining_steps > 0) new_dir =  1;
    else if (remaining_steps < 0) new_dir = -1;
    else                          new_dir =  0;   // already there

    bool change_dir = (new_dir != m->dir);
    m->dir = new_dir;
    if (change_dir) m->steps_in_direction = 0;
    bool near_start = (m->steps_in_direction <= m->slow_zone);

    int extra_delay_us = m->slowdown_factor * m->dn_delay_us;
    extra_delay_us = (near_start || near_stop) ? extra_delay_us : 0;

    // set correct direction for motor
    gpio_put(m->direction_pin, m->dir > 0 ? m->cw_val : !m->cw_val);

    stepper_enable(m);
    for (int i = 0; i < nsteps; i++) {
        stepper_tick(m, extra_delay_us);
    }
    stepper_disable(m);
    m->steps_in_direction += nsteps;
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
    cJSON *item_json;
    cJSON *root = cJSON_Parse(json_str);
    if (!root) return;
    item_json = cJSON_GetObjectItem(root, "az_set_pos");
    azimuth.position = item_json ? item_json->valueint : azimuth.position;
    // if changing position definitions, better reset target too
    azimuth.target_pos = item_json ? azimuth.position : azimuth.target_pos;
    item_json = cJSON_GetObjectItem(root, "el_set_pos");
    elevation.position = item_json ? item_json->valueint : elevation.position;
    // if changing position definitions, better reset target too
    elevation.target_pos = item_json ? elevation.position : elevation.target_pos;

    item_json = cJSON_GetObjectItem(root, "az_set_target_pos");
    azimuth.target_pos = item_json ? item_json->valueint : azimuth.target_pos;
    item_json = cJSON_GetObjectItem(root, "el_set_target_pos");
    elevation.target_pos = item_json ? item_json->valueint : elevation.target_pos;
    // Process halt request
    item_json = cJSON_GetObjectItem(root, "halt");
    azimuth.target_pos = item_json ? azimuth.position : azimuth.target_pos;
    elevation.target_pos = item_json ? elevation.position : elevation.target_pos;
        
    item_json = cJSON_GetObjectItem(root, "az_up_delay_us");
    azimuth.up_delay_us = item_json ? item_json->valueint : azimuth.up_delay_us;
    item_json = cJSON_GetObjectItem(root, "az_dn_delay_us");
    azimuth.dn_delay_us = item_json ? item_json->valueint : azimuth.dn_delay_us;
    item_json = cJSON_GetObjectItem(root, "el_up_delay_us");
    elevation.up_delay_us = item_json ? item_json->valueint : elevation.up_delay_us;
    item_json = cJSON_GetObjectItem(root, "el_dn_delay_us");
    elevation.dn_delay_us = item_json ? item_json->valueint : elevation.dn_delay_us;

    cJSON_Delete(root);
}


void motor_status(uint8_t app_id) {
	send_json(7,
        KV_STR, "sensor_name", "motor",
        KV_STR, "status", "update",
        KV_INT, "app_id", app_id,
        KV_INT, "az_pos", azimuth.position,
        KV_INT, "az_target_pos", azimuth.target_pos,
        KV_INT, "el_pos", elevation.position,
        KV_INT, "el_target_pos", elevation.target_pos
    );
}

void motor_op(uint8_t app_id) {
	// move the stepper motors max_move steps
    stepper_op(&elevation);
    stepper_op(&azimuth);
}
