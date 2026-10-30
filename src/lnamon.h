#ifndef LNAMON_H
#define LNAMON_H

#include <stdint.h>
#include "eigsep_command.h"

// LNA 5V supply-rail monitor. The rail is divided down by R1 (rail -> pin)
// and R2 (pin -> GND), with 100nF from the pin to GND, onto GP27 / ADC1.
// R1/R2 are the DMM-measured values of the through-hole parts fitted.
// Nominal 51k/75k puts a 5.0V rail at ~2.98V and keeps the ADC below 3.3V up to a
// ~5.54V rail, while R1||R2 (~30k) bounds the error from the pin's 1uA
// worst-case leakage to ~30mV at the pin.
#define LNAMON_GPIO             27
#define LNAMON_ADC_CH           1   // GP27 is ADC1

#define LNAMON_R1_OHMS          51000.0f
#define LNAMON_R2_OHMS          75100.0f
// Vrail = Vadc * (R1 + R2) / R2
#define LNAMON_DIVIDER_GAIN     ((LNAMON_R1_OHMS + LNAMON_R2_OHMS) / LNAMON_R2_OHMS)

// Linear correction applied to the divider-scaled rail voltage:
// Vrail = LNAMON_CAL_GAIN * (Vadc * LNAMON_DIVIDER_GAIN) + LNAMON_CAL_OFFSET.
// Least-squares fit against a DMM at the supply terminals (4.57, 5.19,
// 5.50V; residuals <= 2.2mV). Absorbs ADC gain/offset error, the actual
// ADC_VREF (3.27V measured vs the nominal LNAMON_VREF), and resistor
// tolerance. lna_adc_voltage is reported uncorrected so the cal can be redone.
#define LNAMON_CAL_GAIN         1.01164f
#define LNAMON_CAL_OFFSET       (-0.03222f)

#define LNAMON_ADC_BITS         12
#define LNAMON_ADC_MAX          ((1 << LNAMON_ADC_BITS) - 1)
#define LNAMON_VREF             3.3f
#define LNAMON_ADC_SAMPLES      16

// Minimum spacing between sample bursts. The divider's ~30k source
// impedance relies on the 100nF cap to supply the ADC's sampling charge;
// sampling back-to-back every main-loop pass would drain the cap faster
// than the ~3ms RC recharges it and bias the reading low.
#define LNAMON_SAMPLE_PERIOD_MS 20

void lnamon_init(uint8_t app_id);
void lnamon_server(uint8_t app_id, const char *json_str);
void lnamon_op(uint8_t app_id);
void lnamon_status(uint8_t app_id);

#endif // LNAMON_H
