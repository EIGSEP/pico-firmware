// main.c
#include "pico/stdlib.h"
#include "therm.h"
#include "motor.h"
#include "switch.h"
// … include all six headers …

// assign your DIP pins here:
#define DIP0 2
#define DIP1 3
#define DIP2 4

static uint8_t read_dip_code(void) {
  return (gpio_get(DIP2) << 2) |
         (gpio_get(DIP1) << 1) |
          gpio_get(DIP0);
}

int main() {
  stdio_init_all();

  // configure DIP GPIOs as inputs with pulls
  for (int pin = DIP0; pin <= DIP2; pin++) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_down(pin);
  }

  uint8_t code = read_dip_code();
  switch (code) {
    case 0:  therm_app();  break;
    case 1:  motor_app();  break;
    case 2:  switch_app(); break;
    // … cases 3,4,5 mapping to your other apps …
    default: while (1) tight_loop_contents();  // invalid code
  }

  // we never get here
  return 0;
}

