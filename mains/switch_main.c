#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"

// GPIO pins for switches
const uint GPIOS[] = {6, 5, 11, 3, 15, 0, 8, 16};
const uint NUM_GPIOS = sizeof(GPIOS) / sizeof(GPIOS[0]);

// Maximum command length: NUM_GPIOS + '!' + '\n' + '\0'
#define MAX_COMMAND_LEN (NUM_GPIOS + 3)

void set_switch_states(const char* statestr) {
    size_t len = strlen(statestr);
    bool verify = false;
    
    // Check if verification is requested
    if (len > 0 && statestr[len - 1] == '!') {
        verify = true;
        len--;  // Remove the '!' from length
    }
    
    // Check if command length matches number of pins
    if (len != NUM_GPIOS) {
        return;
    }
    
    // Set each GPIO pin state
    for (uint i = 0; i < NUM_GPIOS && i < len; i++) {
        if (statestr[i] == '0') {
            gpio_put(GPIOS[i], 0);
        } else if (statestr[i] == '1') {
            gpio_put(GPIOS[i], 1);
        } else {
            // Invalid character, abort
            return;
        }
    }
    
    // If verification requested, send back current states
    if (verify) {
        printf("STATES:");
        for (uint i = 0; i < NUM_GPIOS; i++) {
            printf("%d", gpio_get(GPIOS[i]) ? 1 : 0);
        }
        printf("\n");
    }
}

int main() {
    // Initialize stdio
    stdio_init_all();
    
    // Initialize GPIO pins as outputs
    for (uint i = 0; i < NUM_GPIOS; i++) {
        gpio_init(GPIOS[i]);
        gpio_set_dir(GPIOS[i], GPIO_OUT);
        gpio_put(GPIOS[i], 0);  // Set all pins to low initially
    }
    
    char command[MAX_COMMAND_LEN];
    
    // Main loop
    while (true) {
        // Read command from stdin
        int ch;
        uint pos = 0;
        
        while ((ch = getchar()) != '\n' && ch != EOF && pos < MAX_COMMAND_LEN - 1) {
            command[pos++] = (char)ch;
        }
        
        if (pos > 0) {
            command[pos] = '\0';  // Null terminate
            set_switch_states(command);
        }
    }
    
    return 0;
}