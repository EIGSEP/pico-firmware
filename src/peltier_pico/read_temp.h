#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "pico/multicore.h"
#include "hardware/pwm.h"
#include <string.h> // for logger, usb_serial_request_reply in main.c

// === required for DS18B20 thermistor ===
#include "onewire_library.h"
#include "ds18b20.h"
#include "ow_rom.h"

// ADC targets (old thermistor, used with ADC on Pico -- also used to read onboard temp of Pico
#define ADC_INTERNAL_PICO 4 // pico ADC internal temp
#define ADC_THERMISTOR 0    // Pin 26
#define ADC_V 3.3f
#define ADC_BITS 12
#define ZEROC_IN_K 273.15

// // === THERMISTOR CALIBRATION ===
// #define NTC_R 10000.0f
// #define NTC_BETA 3950.0f
// #define STEIN_A 1.028671831e-03
// #define STEIN_B 2.392041087e-04
// #define STEIN_C 1.563817562e-07

// === DS18B20 Thermistor ===
extern OW ow; 


// === Commands ===
#define ETX 0x03 // ctrl + c byte, added for emergancy stop

static const float adc_v_per_cnt = ADC_V / (1 << ADC_BITS); // (old thermistor setup, used with ADC on Pico

float read_pico_temperature();
float read_peltier_thermistor(void);
float read_ds18b20_celsius(void);
float read_ds18b20_by_rom(uint64_t rom);
