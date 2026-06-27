#ifndef CURRENTMON_H
#define CURRENTMON_H

// Whole-system current monitor. Reads an ACS724 current sensor (through a
// 3.3k/4.7k resistive divider) on GP26 / ADC0 and exposes the raw ADC-pin
// voltage. Composed into the lidar app dispatch in main.c because the lidar
// Pico uses no other ADC channel, so this sensor is the sole occupant of the
// ADC mux (no adc_select_input swapping → no cross-channel correlation).
//
// Firmware stays "dumb": it reports volts only. The voltage->current
// conversion lives host-side (picohost PicoLidar redis handler).
void currentmon_init(void);
void currentmon_op(void);
float currentmon_voltage(void);

#endif // CURRENTMON_H
