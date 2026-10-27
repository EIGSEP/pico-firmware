#ifndef SIMPLE_THERM_H
#define SIMPLE_THERM_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/types.h"
#include "eigsep_command.h"

// Read-only thermistor logger: every external ADC input (GPIO 26, 27,
// 28 = ADC0..2) is a thermistor divider, plus the RP2350's on-die
// temperature sensor. No commands, no control logic — status only.
// GPIO 29 (ADC3) is not read: on the Pico 2 board it is hard-wired to
// VSYS/3, not a free pin.
#define SIMPLE_THERM_NUM_CHANNELS   3
#define SIMPLE_THERM_FIRST_GPIO     26

// Sampling cadence (ms). Faster than STATUS_CADENCE_MS so every status
// tick carries a fresh reading.
#define SIMPLE_THERM_SAMPLE_MS      100

// Vishay NTCLE100E3103HB0 NTC (10 kOhm at 25 C, B25/85 = 3977 K) wired
// `3V3 -[10.01k]- ADC pin -[NTC]- GND`, so R = R_fixed * V / (3.3 - V).
// Extended Steinhart-Hart fit from the Vishay datasheet (document 29049,
// "Mat A" coefficient row), specified for -40..+125 C:
//   1/T = A1 + B1 ln(R/Rref) + C1 ln^2(R/Rref) + D1 ln^3(R/Rref)
// with T in kelvin and Rref the 25 C resistance. Same coefficients
// temp_simple.h once carried for this part (pico-firmware ba30131).
#define SIMPLE_THERM_PULLUP_OHMS    10010.0f
#define SIMPLE_THERM_REF_OHMS       10000.0f
#define SIMPLE_THERM_SH_A1          3.354016e-3f
#define SIMPLE_THERM_SH_B1          2.569850e-4f
#define SIMPLE_THERM_SH_C1          2.620131e-6f
#define SIMPLE_THERM_SH_D1          6.383091e-8f

typedef struct {
    uint  adc_input;
    bool  adc_configured;
    float voltage;
    float resistance;   // NAN when the reading is invalid
    float T_now;        // deg C, NAN when the reading is invalid
} SimpleThermChannel;

void simple_therm_init(uint8_t app_id);
void simple_therm_server(uint8_t app_id, const char *line);
void simple_therm_op(uint8_t app_id);
void simple_therm_status(uint8_t app_id);

#endif // SIMPLE_THERM_H
