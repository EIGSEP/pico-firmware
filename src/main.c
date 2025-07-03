#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "pico/unique_id.h"
#include <stdio.h>
#include <string.h>

// App headers
#include "blink_app1.h"
#include "blink_app2.h"

// DIP switch GPIO pins
#define DIP0_PIN 2
#define DIP1_PIN 3
#define DIP2_PIN 4

// LED GPIO pin
#define LED_PIN PICO_DEFAULT_LED_PIN

// Number of supported apps
#define MAX_APPS 2

// App descriptor structure
typedef struct {
    const char* name;
    void (*app_func)(void);
} AppDescriptor;

// App dispatch table
static const AppDescriptor app_table[MAX_APPS] = {
    {"blink1",  blink_app1},   // 0b000 - LED blink (default)
    {"blink2",  blink_app2},   // 0b001 - LED blink (default)
};


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
        gpio_pull_up(dip_pins[i]); // XXX is this needed?
    }
    sleep_ms(10); // allow switches to settle
}

// Initialize LED GPIO
static void init_led(void) {
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1); // Turn LED on
}

// Log the device ID over USB-CDC
static void log_device_id(uint8_t code) {
    printf("Device ID: %u\r\n", code);
}

// Wait for "GO" command from serial
static void wait_for_go_command(void) {
    char buffer[10];
    int pos = 0;
    
    while (1) {
        int c = getchar();
        if (c != EOF) {
            if (c == '\r' || c == '\n') {
                buffer[pos] = '\0';
                if (strcmp(buffer, "GO") == 0) {
                    return;
                }
                pos = 0;
            } else if (pos < 9) {
                buffer[pos++] = (char)c;
            }
        }
    }
}

int main(void) {
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

    // Wait for GO command
    // wait_for_go_command();

    // Get unique board ID
    pico_unique_board_id_t unique_id;
    pico_get_unique_board_id(&unique_id);

    // Send response with unique ID and DIP code
    printf("ID:");
    for (int i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; i++) {
        printf("%02X", unique_id.id[i]);
    }
    printf(",DIP:%d\r\n", app_code);

    // Original device ID log
    log_device_id(app_code);

    // Validate app code
    if (app_code >= MAX_APPS || app_table[app_code].app_func == NULL) {
        printf("WARNING: invalid or unimplemented code %d, defaulting to blink\r\n", app_code);
        app_code = 0;
    }
    const AppDescriptor* app = &app_table[app_code];

    // Display startup info
    printf("\r\n=================================\r\n");
    printf("PICO Multi-App Firmware v2.0\r\n");
    printf("=================================\r\n");
    printf("DIP Switch Code: %d (0b%d%d%d)\r\n",
           app_code,
           (app_code >> 2) & 1,
           (app_code >> 1) & 1,
           app_code & 1);
    printf("Starting App: %s\r\n", app->name);
    printf("=================================\r\n\r\n");

    // Enable watchdog (8 seconds)
    watchdog_enable(8000, 1);

    // Launch the selected app
    app->app_func();

    // Should never return
    printf("ERROR: App returned unexpectedly\r\n");
    while (1) {
        tight_loop_contents();
    }
    return 0;
}

