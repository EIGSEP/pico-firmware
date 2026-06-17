/**
 * @file motor_ramp.h
 * @brief Pure constant-acceleration step-delay ramp for the stepper driver.
 *
 * Kept dependency-free (only <stdint.h>/<math.h>, no pico headers) so the
 * timing math can be compiled and unit-tested on the host independently of
 * the firmware that uses it. `motor.c` includes this and calls ramp_extra()
 * once per step inside stepper_op().
 */

#ifndef MOTOR_RAMP_H
#define MOTOR_RAMP_H

#include <math.h>
#include <stdint.h>

/**
 * @brief Extra per-step delay (us) above cruise for a constant-accel ramp.
 *
 * For a step that is `k` steps from the nearer slow end of a move (the start,
 * or the target), the step rate follows
 *     v(k) / v_cruise = sqrt(1/F^2 + (1 - 1/F^2) * k/D)
 * so the motor accelerates from v_cruise/F up to v_cruise over `ramp_steps`
 * steps at constant acceleration. This bounds the rate-of-change of speed the
 * rotor must track, which is what prevents missed steps when starting,
 * stopping, and reversing under load. Beyond the ramp (k >= ramp_steps) it
 * returns 0 (full cruise speed).
 *
 * @param k             steps from the nearer slow end (0 = slowest)
 * @param ramp_steps    ramp distance D in steps
 * @param start_factor  F > 1; first/last step runs F times slower than cruise
 * @param cruise_period cruise step period (up_delay + dn_delay), microseconds
 * @return extra microseconds to add to the step period (0 at/after cruise)
 */
static inline uint32_t ramp_extra(uint32_t k, uint32_t ramp_steps,
                                  float start_factor,
                                  uint32_t cruise_period) {
    if (k >= ramp_steps) {
        return 0;  /* cruising: no extra delay (also guards ramp_steps==0) */
    }
    float inv_f2 = 1.0f / (start_factor * start_factor);
    float frac = (float)k / (float)ramp_steps;
    float v_rel = sqrtf(inv_f2 + (1.0f - inv_f2) * frac);
    return (uint32_t)((float)cruise_period / v_rel - (float)cruise_period
                      + 0.5f);
}

#endif  /* MOTOR_RAMP_H */
