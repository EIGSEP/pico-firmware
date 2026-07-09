#ifndef POTMON_H
#define POTMON_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/adc.h"
#include "eigsep_command.h"

#define POTMON_ADC_CH_AZ        0

#define POTMON_GPIO_AZ          26

/* SP1 failsafe termination control. The pin was freed when the el pot
 * was removed. LOW = SHORT cap (failsafe: matches the unpowered state
 * of the termination switch, so a rebooted or dead pico leaves the
 * long cable shorted), HI = OPEN. */
#define POTMON_GPIO_SP1_TERM    27
#define POTMON_SP1_TERM_SHORT   0
#define POTMON_SP1_TERM_OPEN    1

#define POTMON_ADC_BITS         12
#define POTMON_ADC_MAX          ((1 << POTMON_ADC_BITS) - 1)
#define POTMON_VREF             3.3f

typedef struct {
    uint    gpio_pin;
    uint    adc_channel;
    float   voltage;

} PotSensor;

void potmon_init(uint8_t app_id);
void potmon_op(uint8_t app_id);
void potmon_server(uint8_t app_id, const char *json_str);
void potmon_status(uint8_t app_id);

#endif
