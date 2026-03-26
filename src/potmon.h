#ifndef POTMON_H
#define POTMON_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/adc.h"
#include "eigsep_command.h"

/* ------------------------------------------------------------------ */
/* ADC configuration                                                   */
/* ------------------------------------------------------------------ */
#define POTMON_ADC_CH0          0
#define POTMON_ADC_CH1          1

#define POTMON_GPIO0            26
#define POTMON_GPIO1            27

#define POTMON_ADC_BITS         12
#define POTMON_ADC_MAX          ((1 << POTMON_ADC_BITS) - 1)
#define POTMON_VREF             3.3f

/* ------------------------------------------------------------------ */
/* Default calibration                                                 */
/*                                                                     */
/* 10-turn potentiometer spanning 0–3.3 V → 0–3600°.                  */
/*   angle = cal_m * voltage + cal_b                                   */
/*                                                                     */
/* Override at runtime by sending {"cal_m0":…,"cal_b0":…} etc.        */
/* Reset to these values with {"cal_reset": true}.                     */
/* ------------------------------------------------------------------ */
#define POTMON_DEFAULT_TURNS    10
#define POTMON_DEFAULT_M        ((360.0f * POTMON_DEFAULT_TURNS) / POTMON_VREF)
#define POTMON_DEFAULT_B        0.0f

/* ------------------------------------------------------------------ */
/* Data structure                                                      */
/* ------------------------------------------------------------------ */

typedef struct {
    uint    gpio_pin;
    uint    adc_channel;
    float   voltage;
    float   cal_m;   /* slope:     angle = cal_m * voltage + cal_b */
    float   cal_b;   /* intercept                                   */
} PotSensor;

/* ------------------------------------------------------------------ */
/* Standard app interface                                              */
/* ------------------------------------------------------------------ */
void potmon_init(uint8_t app_id);
void potmon_op(uint8_t app_id);
void potmon_server(uint8_t app_id, const char *json_str);
void potmon_status(uint8_t app_id);

#endif /* POTMON_H */
