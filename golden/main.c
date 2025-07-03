#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#define STAGE2_OFFSET 0x10008000u
typedef void (*entry_fn)(void);

static void __attribute__((noreturn)) jump_to_stage2(void) {
    uint32_t *vtab = (uint32_t*)STAGE2_OFFSET;
    __asm volatile ("msr msp, %0" :: "r"(vtab[0]) );
    ((entry_fn)vtab[1])();
    while (1);
}

int main() {
    // 1) USB CDC
    stdio_init_all();

    // 2) Unique ID
    ch id_str[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
    pico_get_unique_board_id_string(id_str, sizeof(id_str));
    printf("Pico Bootloader - Unique ID: %s\r\n", id_str);

    // 3) GPIOs 2, 3, 4 inputs
    for (int pin = 2; pin <= 4; pin++) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
    }

    // 4) LED so you know you're in bootloader
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    char buf[16];
    while (true) {
        // Blink LED once per loop
        gpio_put(LED_PIN, 1);
        sleep_ms(100);
        gpio_put(LED_PIN, 0);

        // Read & print the three pins
        int v2 = gpio_get(2);
        int v3 = gpio_get(3);
        int v4 = gpio_get(4);
        printf("BOOT1: GPIO2=%d 3=%d 4=%d  — type GO to continue\r\n", v2, v3, v4);

	// Read the unique ID again
	pico_get_unique_board_id_string(id_str, sizeof(id_str));
	printf("Unique ID: %s\r\n", id_str);
        // Check for a “GO” command from the host
        // non-blocking read of up to 15 chars
        int len = fread(buf, 1, sizeof(buf)-1, stdin);
        if (len > 0) {
            buf[len] = '\0';
            if (strcmp(buf, "GO\n") == 0 || strcmp(buf, "GO\r\n") == 0) {
                printf("Jumping to Stage 2…\r\n");
                sleep_ms(100);
                jump_to_stage2();
            }
        }

        sleep_ms(400);
    }
}
