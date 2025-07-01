#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "motor.h"

// Stepper instances (persist positions between commands)
static Stepper elevation, azimuth;

// Timer callback: fires every REPORT_INTERVAL_MS
// Prints current absolute positions of both steppers
bool status_timer_cb(repeating_timer_t *rt) {
    printf("{\"pos_az\":%ld,\"pos_el\":%ld}\n",
           (long)azimuth.position,
           (long)elevation.position);
    fflush(stdout);
    return true;  // keep repeating
}

int main() {
    // Initialize USB serial
    stdio_init_all();
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    printf("connected\n"); fflush(stdout);

    // Configure stepper pins
    const uint elev_pins[5] = {21, 18, 0, 1, 19};
    const uint az_pins[5]   = {11, 12, 0, 1, 10};
    stepper_init(&elevation,
        elev_pins[0], elev_pins[1], elev_pins[2], elev_pins[3], elev_pins[4]);
    stepper_init(&azimuth,
        az_pins[0], az_pins[1], az_pins[2], az_pins[3], az_pins[4]);

    // Start repeating timer (200 ms interval)
    const uint32_t REPORT_INTERVAL_MS = 200;
    repeating_timer_t timer;
    add_repeating_timer_ms(REPORT_INTERVAL_MS,
                           status_timer_cb,
                           NULL,
                           &timer);

    // Main command loop (unchanged)
    char buf[256];
    while (true) {
        if (!fgets(buf, sizeof(buf), stdin))
            continue;

        // Emergency STOP packet
        if (strstr(buf, "STOP")) {
            printf("EMERGENCY STOP\n"); fflush(stdout);
            continue;
        }

        // Parse JSON command
        unsigned delay_us, pulses_az, pulses_el, report;
        int dir_az, dir_el;
        if (sscanf(buf,
            "{\"delay\":%u,\"pulses_az\":%u,\"dir_az\":%d,"
            "\"pulses_el\":%u,\"dir_el\":%d,\"report\":%u}",
            &delay_us, &pulses_az, &dir_az,
            &pulses_el, &dir_el, &report) != 6) {
            printf("bad cmd: %s", buf); fflush(stdout);
            continue;
        }

        // Apply delay and direction
        elevation.delay_us = azimuth.delay_us = delay_us;
        elevation.dir     = dir_el > 0 ? 1 : -1;
        azimuth.dir       = dir_az > 0 ? 1 : -1;

        // Interleaved stepping
        unsigned rem_az = pulses_az;
        unsigned rem_el = pulses_el;
        unsigned step_count = 0;
        while ((rem_az | rem_el) > 0) {
            if (rem_az > 0) {
                stepper_move(&azimuth);
                rem_az--;
            }
            if (rem_el > 0) {
                stepper_move(&elevation);
                rem_el--;
            }
            step_count++;

            // Check for non-blocking emergency STOP
            int ch = getchar_timeout_us(0);
            if (ch != PICO_ERROR_TIMEOUT) {
                printf("EMERGENCY STOP\n"); fflush(stdout);
                break;
            }
        }

        // Final position report after move/STOP
        unsigned max_steps = pulses_az > pulses_el ? pulses_az : pulses_el;
        printf("{\"step\":%u,\"pos_az\":%ld,\"pos_el\":%ld}\n",
               max_steps,
               (long)azimuth.position,
               (long)elevation.position);
        fflush(stdout);

        // Disable coils until next command
        stepper_close(&elevation);
        stepper_close(&azimuth);
    }

    return 0;
}

