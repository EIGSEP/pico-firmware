#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "pico/unique_id.h"
#include <stdio.h>
#include <string.h>

// App headers
#include "pico_multi.h"

// DIP switch GPIO pins
#define DIP0_PIN 2
#define DIP1_PIN 3
#define DIP2_PIN 4

// LED GPIO pin
#define LED_PIN PICO_DEFAULT_LED_PIN


void motor_server(char *cmd_str) {
    send_json(2, KV_STR,
            "name", "motor_server",
            "status", "ok");
}

void motor_op() {
    return;
}

void motor_status() {
    send_json(2,
        KV_STR, "name", "motor_status",
        KV_INT, "value", 17
    );
}

void no_server(char *cmd_str) {
    // We don't have an app for that
    send_json(2,
        KV_STR, "status", "error",
        KV_STR, "value", "Unknown app_code"
    );
    return;
}

void no_op() {
    return;
}

void no_status(app_code) {
    send_json(3,
        KV_STR, "name", "no_status",
        KV_STR, "status", "error",
        KV_INT, "value", app_code
    );
}

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
     //   gpio_pull_up(dip_pins[i]); // XXX is this needed?
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
    uint32_t cadence_ms = 300;  //cadence for loop?
    int index = 0;
    bool led_state=1;
    absolute_time_t next_sample = make_timeout_time_ms(cadence_ms);

    // 1) Initialize DIP switches before USB init
    init_dip_switches();
    // 2) Initialize LED and turn it on
    init_led();
    // 3) Bring up USB CDC (stdio)
    stdio_init_all();
    // allow host to enumerate
    sleep_ms(1000);

    // Read DIP code early
    uint8_t app_code = read_dip_code() & 0x07;
    printf("%d\n", app_code);

    // Get unique board ID
    pico_unique_board_id_t unique_id;
    pico_get_unique_board_id(&unique_id);
    char uid_str[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2 + 1];
    for (int i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; i++) {
        sprintf(&uid_str[i*2], "%02X", unique_id.id[i]);
    }

    // emit JSON identifier for panda to keep track
    send_json(2,
        KV_STR, "unique_id", uid_str,
        KV_INT, "app_code", app_code
    );
   
    while (true) {
        // Process incoming json-formatted commands 
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            // Dispatch a complete command
            if (c == '\n') {
                line[index] = '\0';
                index = 0;
                // Dispatch command to appropriate app
                switch (app_code) {
                    case APP_MOTOR:
                        motor_server(line);
                        break;
                    default:
                        no_server(line);
                }
            // Otherwise add incoming character to buffer
            } else {
                if (index < BUFFER_SIZE - 1) {
                    line[index++] = (char)c;
                }
            }
        }

        switch (app_code) {
            case APP_MOTOR:
                motor_op();
                break;
            default:
                no_op();
        }

        // Perform scheduled tasks
        if (absolute_time_diff_us(get_absolute_time(), next_sample) <= 0) {
            gpio_put(LED_PIN, led_state);
            led_state = !led_state;
            switch (app_code) {
                case APP_MOTOR:
                    motor_status();
                    break;
                default:
                    no_status(app_code);
            }
            next_sample = make_timeout_time_ms(cadence_ms);
        }
    }
}

