#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// DIP switch GPIO pins (same as main.c)
#define DIP0_PIN 2
#define DIP1_PIN 3
#define DIP2_PIN 4

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

// Test function for DIP switch reading
void test_dip_switches(void) {
    printf("\n=================================\n");
    printf("DIP Switch Test Harness\n");
    printf("=================================\n");
    printf("Pins: DIP0=%d, DIP1=%d, DIP2=%d\n", DIP0_PIN, DIP1_PIN, DIP2_PIN);
    printf("Reading DIP switches every 1 second...\n");
    printf("Press Ctrl+C to exit\n\n");
    
    init_dip_switches();
    
    uint8_t last_code = 0xFF; // Invalid initial value
    
    while (true) {
        uint8_t current_code = read_dip_code();
        
        // Only print when code changes
        if (current_code != last_code) {
            printf("DIP Code: %d (0b%d%d%d) - Pins: D2=%d D1=%d D0=%d\n",
                   current_code,
                   (current_code >> 2) & 1,
                   (current_code >> 1) & 1,
                   current_code & 1,
                   gpio_get(DIP2_PIN),
                   gpio_get(DIP1_PIN), 
                   gpio_get(DIP0_PIN));
            
            // Show which app this would select
            const char* app_names[] = {"therm", "motor", "switch", "sensor", "relay", "adc"};
            if (current_code < 6) {
                printf("  -> Would select: %s app\n", app_names[current_code]);
            } else {
                printf("  -> ERROR: Invalid app code (max 5)\n");
            }
            printf("\n");
            
            last_code = current_code;
        }
        
        sleep_ms(100);  // Check every 100ms
    }
}

#ifdef TEST_DIP_STANDALONE
int main(void) {
    stdio_init_all();
    sleep_ms(1000); // Wait for USB enumeration
    
    test_dip_switches();
    return 0;
}
#endif