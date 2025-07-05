#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "pico/unique_id.h"
#include <stdio.h>
#include <string.h>

// App headers
#include "pico_multi.h"
#include "motor.h"
#include "rfswitch.h"
#include "tempctrl.h"
#include "tempmon.h"
#include "imu.h"
#include "lidar.h"


// Read 3-bit DIP switch code
static uint8_t read_dip_code(void) {
    return (gpio_get(DIP2_PIN) << 2) |
           (gpio_get(DIP1_PIN) << 1) |
           gpio_get(DIP0_PIN);
}

// Initialize DIP switch GPIOs (pull-ups for default HIGH)
static void init_dip_switches(void) {
    const uint dip_pins[] = { DIP0_PIN, DIP1_PIN, DIP2_PIN };
    for (int i = 0; i < 3; i++) {
        gpio_init(dip_pins[i]);
        gpio_set_dir(dip_pins[i], GPIO_IN);
        gpio_pull_up(dip_pins[i]);
    }
    sleep_ms(10); // allow switches to settle
}

// Initialize LED GPIO
static void init_led(void) {
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1); // Turn LED on
}


int main(void) {
    char line[BUFFER_SIZE];  // buffer to hold input command
    int index = 0;
    bool led_state=1;
    absolute_time_t next_sample = make_timeout_time_ms(STATUS_CADENCE_MS);

    // 1) Initialize DIP switches before USB init
    init_dip_switches();
    // 2) Initialize LED and turn it on
    init_led();
    // 3) Bring up USB CDC (stdio)
    stdio_init_all();

    // Read DIP code early
    uint8_t app_id = read_dip_code();

    // Get unique board ID
    pico_unique_board_id_t unique_id;
    pico_get_unique_board_id(&unique_id);
    char uid_str[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2 + 1];
    for (int i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; i++) {
        sprintf(&uid_str[i*2], "%02X", unique_id.id[i]);
    }

    // Run app-dependent initialization
    switch (app_id) {
        case APP_MOTOR: motor_init(app_id); break;
        case APP_RFSWITCH: rfswitch_init(app_id); break;
        case APP_TEMPCTRL: tempctrl_init(app_id); break;
        case APP_TEMPMON: tempmon_init(app_id); break;
        case APP_IMU: imu_init(app_id); break;
        case APP_LIDAR: lidar_init(app_id); break;
        default: break;
    }
   
    while (true) {
        // Process incoming json-formatted commands 
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            // Dispatch a complete command
            if (c == '\n') {
                line[index] = '\0';
                index = 0;
                // Dispatch command to appropriate app
                switch (app_id) {
                    case APP_MOTOR: motor_server(app_id, line); break;
                    case APP_RFSWITCH: rfswitch_server(app_id, line); break;
                    case APP_TEMPCTRL: tempctrl_server(app_id, line); break;
                    case APP_TEMPMON: tempmon_server(app_id, line); break;
                    case APP_IMU: imu_server(app_id, line); break;
                    case APP_LIDAR: lidar_server(app_id, line); break;
                    default:
                        send_json(2,
                            KV_STR, "status", "error",
                            KV_INT, "app_id", app_id
                        );
                }
            // Otherwise add incoming character to buffer
            } else {
                if (index < BUFFER_SIZE - 1) {
                    line[index++] = (char)c;
                }
                // prioritize reading a command before operations
                continue;
            }
        }

        // Perform every-loop operations
        switch (app_id) {
            case APP_MOTOR: motor_op(app_id); break;
            case APP_RFSWITCH: rfswitch_op(app_id); break;
            case APP_TEMPCTRL: tempctrl_op(app_id); break;
            case APP_TEMPMON: tempmon_op(app_id); break;
            case APP_IMU: imu_op(app_id); break;
            case APP_LIDAR: lidar_op(app_id); break;
            default:
                break;
        }

        // Perform scheduled status reporting
        if (absolute_time_diff_us(get_absolute_time(), next_sample) <= 0) {
            gpio_put(LED_PIN, led_state);
            led_state = !led_state;
            switch (app_id) {
                case APP_MOTOR: motor_status(app_id); break;
                case APP_RFSWITCH: rfswitch_status(app_id); break;
                case APP_TEMPCTRL: tempctrl_status(app_id); break;
                case APP_TEMPMON: tempmon_status(app_id); break;
                case APP_IMU: imu_status(app_id); break;
                case APP_LIDAR: lidar_status(app_id); break;
                default:
                    send_json(2,
                        KV_STR, "status", "error",
                        KV_INT, "app_id", app_id
                    );
            }
            next_sample = make_timeout_time_ms(STATUS_CADENCE_MS);
        }
    }
}

