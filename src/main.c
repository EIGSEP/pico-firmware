#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include <stdio.h>
#include <string.h>

// App headers
#include "motor_app.h"
#include "switch_app.h"
#include "blink_app.h"
#include "relay_app.h"
#include "adc_app.h"

// DIP switch GPIO pins
#define DIP0_PIN 2
#define DIP1_PIN 3
#define DIP2_PIN 4

// Number of supported apps
#define MAX_APPS 6

// App descriptor structure
typedef struct {
    const char* name;
    void (*app_func)(void);
} AppDescriptor;

// App dispatch table
static const AppDescriptor app_table[MAX_APPS] = {
    {"blink",  blink_app},   // 0b000 - LED blink (default)
    {"motor",  motor_app},   // 0b001 - Motor controller
    {"switch", switch_app},  // 0b010 - Switch network
    {"relay",  relay_app},   // 0b011 - Relay control
    {"adc",    adc_app},     // 0b100 - ADC monitor
    {"test5",  NULL},        // 0b101 - Future app
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
        gpio_pull_up(dip_pins[i]);
    }
    sleep_ms(10); // allow switches to settle
}

// Log the device ID over USB-CDC
static void log_device_id(uint8_t code) {
    printf("Device ID: %u\r\n", code);
}

int main(void) {
    // 1) Initialize DIP switches before USB init
    init_dip_switches();
    // 2) Bring up USB CDC (stdio)
    stdio_init_all();
    // allow host to enumerate
    sleep_ms(1000);

    // Read DIP code and log as device identifier
    uint8_t app_code = read_dip_code() & 0x07;
    log_device_id(app_code);

    // Validate app code
    if (app_code >= MAX_APPS || app_table[app_code].app_func == NULL) {
        printf("WARNING: invalid or unimplemented code %d, defaulting to blink\r\n", app_code);
        app_code = 0;
    }
    const AppDescriptor* app = &app_table[app_code];

    // Display startup info
    printf("\r\n=================================\r\n");
    printf("PICO Multi-App Firmware v1.0\r\n");
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

