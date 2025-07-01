#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "hardware/watchdog.h"
#include <stdio.h>
#include <string.h>

// App headers
#include "therm_app.h"
#include "motor_app.h" 
#include "switch_app.h"
#include "sensor_app.h"
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
    {"therm",  NULL},        // 0b000 - Thermocouple app (not yet implemented)
    {"motor",  motor_app},   // 0b001 - Motor controller
    {"switch", switch_app},  // 0b010 - Switch network
    {"sensor", NULL},        // 0b011 - Sensor app (not yet implemented)  
    {"relay",  NULL},        // 0b100 - Relay control (not yet implemented)
    {"adc",    NULL},        // 0b101 - ADC monitor (not yet implemented)
};

// Read 3-bit DIP switch code
static uint8_t read_dip_code(void) {
    return (gpio_get(DIP2_PIN) << 2) |
           (gpio_get(DIP1_PIN) << 1) |
           gpio_get(DIP0_PIN);
}

// Initialize DIP switch GPIOs
static void init_dip_switches(void) {
    const uint dip_pins[] = {DIP0_PIN, DIP1_PIN, DIP2_PIN};
    
    for (int i = 0; i < 3; i++) {
        gpio_init(dip_pins[i]);
        gpio_set_dir(dip_pins[i], GPIO_IN);
        gpio_pull_down(dip_pins[i]);
    }
    
    // Allow switches to settle
    sleep_ms(10);
}

// Set USB serial number based on DIP code
static void set_usb_serial_number(uint8_t code) {
    static char serial_number[16];
    snprintf(serial_number, sizeof(serial_number), "PICO_%03d", code);
    
    // This would require modifying the USB descriptor
    // For now, just document the intended serial number
    printf("USB Serial Number: %s\n", serial_number);
}


int main(void) {
    // Initialize stdio for debug output
    stdio_init_all();
    
    // Brief delay to ensure USB enumeration
    sleep_ms(1000);
    
    // Initialize DIP switches
    init_dip_switches();
    
    // Read DIP switch code
    uint8_t app_code = read_dip_code();
    
    // Set USB serial number
    set_usb_serial_number(app_code);
    
    // Validate app code
    if (app_code >= MAX_APPS) {
        printf("ERROR: Invalid DIP code %d (max %d)\n", app_code, MAX_APPS - 1);
        while (1) {
            tight_loop_contents();
        }
    }
    
    // Get app descriptor
    const AppDescriptor* app = &app_table[app_code];
    
    // Check if app is implemented
    if (app->app_func == NULL) {
        printf("ERROR: App '%s' (code %d) not implemented\n", app->name, app_code);
        while (1) {
            tight_loop_contents();
        }
    }
    
    // Display startup info
    printf("\n");
    printf("=================================\n");
    printf("PICO Multi-App Firmware v1.0\n");
    printf("=================================\n");
    printf("DIP Switch Code: %d (0b%d%d%d)\n", 
           app_code,
           (app_code >> 2) & 1,
           (app_code >> 1) & 1, 
           app_code & 1);
    printf("Starting App: %s\n", app->name);
    printf("=================================\n\n");
    
    // Enable watchdog (8 seconds)
    watchdog_enable(8000, 1);
    
    // Launch the selected app
    app->app_func();
    
    // Should never reach here
    printf("ERROR: App returned unexpectedly\n");
    while (1) {
        tight_loop_contents();
    }
    
    return 0;
}