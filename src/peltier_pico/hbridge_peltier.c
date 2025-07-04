#include <stdio.h>
#include "hbridge_peltier.h"

// Call once at startup to configure PWM + direction pins
void hbridge_init(HBridge *hb, float T_target, float t_target, float gain) {
    
    // === Peltier 1 ===
    // PWM setup
    gpio_set_function(HBRIDGE_PWM_PIN, GPIO_FUNC_PWM);
    hb->hbridge_pwm_slice = pwm_gpio_to_slice_num(HBRIDGE_PWM_PIN);
    pwm_config cfg = pwm_get_default_config();     // do this once given currently we will drive peltier 1 & 2 to the same setpoint
    pwm_config_set_wrap(&cfg, PWM_WRAP);           // do this once (may change if setpoints are independent)
    pwm_init(hb->hbridge_pwm_slice, &cfg, true);
    
    // Direction pins for motor 1 (Peltier 1)
    gpio_init(HBRIDGE_DIR_PIN1);
    gpio_set_dir(HBRIDGE_DIR_PIN1, GPIO_OUT);
    gpio_init(HBRIDGE_DIR_PIN2);
    gpio_set_dir(HBRIDGE_DIR_PIN2, GPIO_OUT);

    // // === Peltier 2 ===
    // gpio_set_function(HBRIDGE_PWM_PIN2, GPIO_FUNC_PWM);
    // hb2->hbridge_pwm_slice2 = pwm_gpio_to_slice_num(HBRIDGE_PWM_PIN2); // hb2 
    // hb->hbridge_pwm_slice
    // pwm_init(hb->hbridge_pwm_slice2, &cfg, true);
    
    // Direction pins for motor 2 (Peltier 2)
    gpio_init(HBRIDGE_DIR_PIN3);
    gpio_set_dir(HBRIDGE_DIR_PIN3, GPIO_OUT);
    gpio_init(HBRIDGE_DIR_PIN4);
    gpio_set_dir(HBRIDGE_DIR_PIN4, GPIO_OUT);
    
    // === Time and temp targets Peltier-1 ===
    hb->T_prev = hb->T_now = hb->T_target = T_target;
    hb->t_target = t_target;
    hb->t_prev = hb->t_now = time(NULL);
    hb->drive = 0.0;
    hb->gain = gain;
    hb->hysteresis = 1.0f; // ∆T, fine-tune
    hb->active = true;     // starts as engaged, setpoint achieved once     ----- "off" if we do not want TEC to run on bootup
    hb->enabled = true;    // initially disabled                            ----- "off" if we do not want TEC to run on bootup
    
    // === Time and temp targets Peltier-2 ===
    
    
    // // === PID gain coefficients ===
    // hb->kp = 1.0f;
    // hb->ki = 0.0f; 
    // hb->kd = 0.0f;
    // hb->pid_integral = 0.0f;
    // hb->pid_prev_error = 0.0f;
}

// Update latest temperature reading and time
void hbridge_update_T(HBridge *hb, time_t t_now, float T_now) {
    hb->t_prev = hb->t_now;
    hb->T_prev = hb->T_now;
    hb->T_now = T_now;
    hb->t_now = t_now;
}

// clamp the drive level to the range [-max, max] to cap power
static inline float clamp_drive(float drive, float max) {
    if (drive > max) return max;
    if (drive < -max) return -max;
    return drive;
}

// // === Testing PID control ===
// float hbridge_pid_compute(HBridge *hb, float setpoint, float measured, float dt) {
//     float error = setpoint - measured;
//     hb->pid_integral += error *dt;
//     float derivative = (error - hb->pid_prev_error) / dt;
//     float output = hb->kp*error + hb->ki * hb->pid_integral + hb->kd * derivative;
//     hb->pid_prev_error = error;
//         
//     // clamp output to +/- gain, or hb->gain = 0.2; etc...
//     clamp_drive(output, hb->gain);
// }

// void hbridge_hysteresis_drive(HBridge *hb) {
//     /*
//      for single thermistor setup.
//     */
//     
//     float error = hb->T_target - hb->T_now;

//     if (!hb->enabled) {
// 	hb->drive = 0.0f;
// 	hb->active = false;                        // goes idle
// 	hbridge_raw_drive(false, 0);

// 	return;
//     }
//     if (hb->active) {                          // currently driving to setpoint
//         if (fabsf(error) <= hb->hysteresis) {  // within hysteresis window
//             hb->active = false;                // goes idle
//             hbridge_raw_drive(false, 0);

//             hb->drive = 0.0f;
//             return;
//         }
//         hbridge_smart_drive(hb);                // outside deadband, drive to setpoint
//     } else {
//         // currently on idle - wakes up when we move beyond ∆T
//         if (fabsf(error) > hb -> hysteresis) {
//             hb->active = true; 
//             hbridge_drive(hb);
//         } else {
//             hb->drive = 0.0f;                   // stays off
//             hbridge_raw_drive(false, 0);
//         }
//     }
// }

void hbridge_hysteresis_drive(HBridge *hb) {
    /*
    Implements independent control of TEC elements, H-bridge inputs, and PWM.
    */
    float error = hb->T_target - hb->T_now;

    if (!hb->enabled) {
	hb->drive = 0.0f;
	hb->active = false;                           // goes idle
    // hbridge_raw_drive(hb, false, 0);          // test
    hbridge_drive(hb);
	return;
    }

    if (hb->active) {                             // currently driving to setpoint
        if (fabsf(error) <= hb->hysteresis) {     // within hysteresis window
            hb->active = false;                   // goes idle
            // hbridge_raw_drive(hb, false, 0);   // test
            hbridge_drive(hb);
            hb->drive = 0.0f;
            return;
        }
        hbridge_smart_drive(hb);                  // outside deadband, drive to setpoint
    } else {
        // currently on idle - wakes up when we move beyond ∆T
        if (fabsf(error) > hb -> hysteresis) {
            hb->active = true; 
            // hbridge_drive(hb);
            hbridge_smart_drive(hb);
        } else {
            hb->drive = 0.0f;                     // stays off
            // hbridge_raw_drive(hb, false, 0);   // test
            hbridge_drive(hb);
        }
    }
}

// === testing hysteresis drive for PID control ===
// void hbridge_hysteresis_drive(HBridge *hb) {
//     float error = hb->T_target - hb->T_now;
//     float dt = (float)(hb->t_now - hb->t_prev); 

//     if (hb->active) {
//         if (fabsf(error) <= hb->hysteresis) {
//             hb->active = false;
//             hbridge_raw_drive(false, 0);
//             hb->drive = 0.0f;
//             return;
//         }
//         // PID control when outside hysteresis window
//         float pid_drive = hbridge_pid_compute(hb, hb->T_target, hb->T_now, dt);
//         hb->drive = pid_drive;
//         hbridge_drive(hb);
//     } else {
//         if (fabsf(error) > hb->hysteresis) {
//             hb->active = true;
//             float pid_drive = hbridge_pid_compute(hb, hb->T_target, hb->T_now, dt);
//             hb->drive = pid_drive;
//             hbridge_drive(hb);
//         } else {
//             hb->drive = 0.0f;
//             hbridge_raw_drive(false, 0);
//         }
//     }
// }

// Drive the hbridge
void hbridge_smart_drive(HBridge *hb) {
    float dT_now, dT_prev;
    
    // Calculate drive level (gain) and direction
    dT_now = hb->T_target - hb->T_now;
    if (dT_now > 0.1) {
	hb->drive = -0.2;
    } else if (dT_now < -0.1) {
	hb->drive = 0.2;
    } else {
	hb->drive = 0.0; // no drive needed
    }
    
    // clamp the drive level to gain
    hb->drive = clamp_drive(hb->drive, hb->gain);

    hbridge_drive(hb);
}

// void hbridge_drive(HBridge *hb) {
//     /*
//     Drives the H-bridge -- for single thermistor setup.
//     */
//     bool forward;
//     uint32_t level;
//     forward = hb->drive > 0 ? true : false;
//     level = (forward ? hb->drive : -hb->drive) * PWM_WRAP;
//     if (level > PWM_WRAP) level = PWM_WRAP;
//     hbridge_raw_drive(forward, level);
// }

void hbridge_drive(HBridge *hb) {
    /*
      hbridge_drive for independent control loops.
    • Drives the H-bridge for a given peltier based on hb->drive in hbridge_smart_drive().
    • If drive is zero, turn off the respective channel (peltier).
    */
    bool forward = hb->drive > 0 ? true : false;
    float magnitude = fabsf(hb->drive);                // fabsf() computes the abs. value of a floating point number
    if (magnitude > 1.0f) magnitude = 1.0f;
    uint32_t level = (uint32_t)(magnitude * PWM_WRAP);
    
    if (level == 0) {
        if (hb->channel ==1) {
            gpio_put(HBRIDGE_DIR_PIN1, false);
            gpio_put(HBRIDGE_DIR_PIN2, false);
            pwm_set_gpio_level(HBRIDGE_PWM_PIN, 0);
    } else if (hb->channel == 2) {
            gpio_put(HBRIDGE_DIR_PIN3, false);
            gpio_put(HBRIDGE_DIR_PIN4, false);
            pwm_set_gpio_level(HBRIDGE_PWM_PIN2, 0);
        }
    } else {
        hbridge_raw_drive(hb, forward, level);
    }
    // return;  // uncomment if we replace raw_drive() entirely with hbridge_drive() and remove the above else{} statement
}

void hbridge_raw_drive(HBridge *hb, bool forward, uint32_t level) {
        /*
          Applies a baseline PWM offset to avoid very low drive levels.
          The offset introduces a 40% minimum PWM duty when the TECs are active,
        */
        uint32_t adjusted_level = (uint32_t)(0.4 * PWM_WRAP + 0.1 * level);
        if (adjusted_level > PWM_WRAP) {
            adjusted_level = PWM_WRAP;
        }
        if (hb->channel == 1) {
            gpio_put(HBRIDGE_DIR_PIN1, forward);
            gpio_put(HBRIDGE_DIR_PIN2, !forward);
            pwm_set_gpio_level(HBRIDGE_PWM_PIN, adjusted_level);
        } else if (hb->channel == 2) {
            gpio_put(HBRIDGE_DIR_PIN3, forward);
            gpio_put(HBRIDGE_DIR_PIN4, !forward);
            pwm_set_gpio_level(HBRIDGE_PWM_PIN2, adjusted_level);
        }
}
// void hbridge_raw_drive(bool forward, uint32_t level) {
//     if (level == 0) {
//         // printf("Drive: off\n");             // uncomment as needed when debugging.
//         gpio_put(HBRIDGE_DIR_PIN1, false);
//         gpio_put(HBRIDGE_DIR_PIN2, false);
//         gpio_put(HBRIDGE_DIR_PIN3, false);     // pin 3 and 4 for peltier-2
//         gpio_put(HBRIDGE_DIR_PIN4, false);
//         
//     } else {
//         level = 0.4 * PWM_WRAP + 0.1 * level;
//         // printf("Drive: %b, %d\n", forward, level); // uncomment as needed when debugging.
//         gpio_put(HBRIDGE_DIR_PIN1, forward);
//         gpio_put(HBRIDGE_DIR_PIN2, !forward);
//         gpio_put(HBRIDGE_DIR_PIN3, forward);
//         gpio_put(HBRIDGE_DIR_PIN4, !forward); 
//     }
//     pwm_set_gpio_level(HBRIDGE_PWM_PIN, level);
//     pwm_set_gpio_level(HBRIDGE_PWM_PIN2, level);
// }


