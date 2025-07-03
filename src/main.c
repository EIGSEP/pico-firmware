#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include <stdio.h>
#include <string.h>

// App headers
#include "blink_app1.h"
#include "blink_app2.h"

// DIP switch GPIO pins
#define DIP0_PIN 2
#define DIP1_PIN 3
#define DIP2_PIN 4

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

// Global variables for status reporting
static uint8_t g_dip_code = 0;
static const char* g_app_name = NULL;

// Read 3-bit DIP switch code
static uint8_t read_dip_code(void) {
    return (gpio_get(DIP2_PIN) << 2) |
           (gpio_get(DIP1_PIN) << 1) |
           gpio_get(DIP0_PIN);
}

// Handle status query command
static void handle_status_query(void) {
    // Send JSON-formatted status response
    printf("{{\"type\":\"status\",\"dip_code\":%d,\"dip_binary\":\"0b%d%d%d\",\"app_name\":\"%s\",\"app_index\":%d,\"firmware_version\":\"1.0\"}}\r\n",
           g_dip_code,
           (g_dip_code >> 2) & 1,
           (g_dip_code >> 1) & 1,
           g_dip_code & 1,
           g_app_name ? g_app_name : "unknown",
           g_dip_code);
}

// Background task to check for status queries  
// This is exposed to apps via app_common.h
void check_for_status_query(void) {
    int c = getchar_timeout_us(0);
    if (c == '?' || c == 'q' || c == 'Q') {
        handle_status_query();
    }
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
    g_dip_code = app_code; // Store globally for status queries
    log_device_id(app_code);

    // Validate app code
    if (app_code >= MAX_APPS || app_table[app_code].app_func == NULL) {
        printf("WARNING: invalid or unimplemented code %d, defaulting to blink\r\n", app_code);
        app_code = 0;
    }
    const AppDescriptor* app = &app_table[app_code];
    g_app_name = app->name; // Store globally for status queries

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

