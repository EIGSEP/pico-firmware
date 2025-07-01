#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include "relay_app.h"

void relay_app(void) {
    while (true) {
	printf("[RELAY] heartbeat\n");
	watchdog_update();
	sleep_ms(1000); // Sleep for 1 second
    }
}
