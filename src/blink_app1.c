#include "pico/stdlib.h"
#include <stdio.h>
#include "blink_app1.h"

void blink_app1(void) {
    // Initialize LED pin
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    // Wait a bit for USB to stabilize
    sleep_ms(100);
    
    printf("Starting LED blink app 1 (v2.0 - FAST BLINK).\n");
    printf("LED on pin %d\n", LED_PIN);
    printf("Pattern: 100ms ON, 100ms OFF (5Hz)\n");
    
    // Main blink loop - FAST pattern (100ms on/off)
    while (true) {
        gpio_put(LED_PIN, 1);
        printf("APP1 v2: LED ON\n");
        sleep_ms(1000);  // Changed from 200ms to 100ms
        
        gpio_put(LED_PIN, 0);
        printf("APP1 v2: LED OFF\n");
        sleep_ms(1000);
    }
}
