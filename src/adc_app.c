#include "pico/stdlib.h" 
#include "hardware/watchdog.h"
#include "adc_app.h"


void adc_app(void) {
    while (true) {
        printf("[ADC] heartbeat\n");
        watchdog_update();
        sleep_ms(1000);
    }
}
