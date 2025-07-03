#include "pico/stdlib.h"
#include <stdio.h>
#include "blink_app1.h"
#include "app_common.h"

void blink_app1(void) {
    // Initialize LED pin
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    // Wait a bit for USB to stabilize
    sleep_ms(100);
    
    printf("Starting LED blink app 1.\n");
    printf("LED on pin %d\n", LED_PIN);
    
    // Main blink loop
    while (true) {
        gpio_put(LED_PIN, 1);
        printf("APP 1 : LED ON\n");
        sleep_ms(200);
        check_for_status_query();  // Check for status requests
        
        gpio_put(LED_PIN, 0);
        printf("APP1 : LED OFF\n");
        sleep_ms(200);
        check_for_status_query();  // Check for status requests
    }
}
