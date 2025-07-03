#include "pico/stdlib.h"
#include <stdio.h>
#include "blink_app2.h"

void blink_app2(void) {
    // Initialize LED pin
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    // Wait a bit for USB to stabilize
    sleep_ms(100);
    
    printf("Starting LED blink app 2.\n");
    printf("LED on pin %d\n", LED_PIN);
    
    // Main blink loop
    while (true) {
        gpio_put(LED_PIN, 1);
        printf("APP 2 : LED ON\n");
        sleep_ms(1000);
        
        gpio_put(LED_PIN, 0);
        printf("APP2 : LED OFF\n");
        sleep_ms(1000);
    }
}
