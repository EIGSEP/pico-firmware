#include "pico/stdlib.h"
#include <stdio.h>
#include "blink_app2.h"
#include "app_common.h"

void blink_app2(void) {
    // Initialize LED pin
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    // Wait a bit for USB to stabilize
    sleep_ms(100);
    
    printf("Starting LED blink app 2 (v2.0 - SLOW DOUBLE BLINK).\n");
    printf("LED on pin %d\n", LED_PIN);
    printf("Pattern: Double blink (2x 150ms) then 1s pause\n");
    
    // Main blink loop - DOUBLE BLINK pattern
    while (true) {
        // First blink
        gpio_put(LED_PIN, 1);
        printf("APP2 v2: LED ON (blink 1)\n");
        sleep_ms(150);
        check_for_status_query();
        
        gpio_put(LED_PIN, 0);
        sleep_ms(150);
        check_for_status_query();
        
        // Second blink
        gpio_put(LED_PIN, 1);
        printf("APP2 v2: LED ON (blink 2)\n");
        sleep_ms(150);
        check_for_status_query();
        
        gpio_put(LED_PIN, 0);
        printf("APP2 v2: LED OFF (pause)\n");
        sleep_ms(1000);  // Long pause
        check_for_status_query();
    }
}
