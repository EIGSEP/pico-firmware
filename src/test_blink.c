// Standalone test image: heartbeat-blinks the onboard LED and exposes
// USB-CDC so the Pico can be reflashed via `flash-picos`.
//
// The blink pattern (80 ms on / 80 ms off / 80 ms on / 760 ms off) is
// intentionally distinct from production firmware's steady 200 ms
// toggle (see src/main.c) so a tech can tell at a glance which image
// is running.

#include "pico/stdlib.h"

int main(void) {
    stdio_init_all();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    while (true) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(80);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(80);
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(80);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(760);
    }
}
