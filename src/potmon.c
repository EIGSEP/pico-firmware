#include "potmon.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "cJSON.h"
#include <math.h>

/* ------------------------------------------------------------------ */
/* Static state                                                        */
/* ------------------------------------------------------------------ */

static PotSensor pot0;
static PotSensor pot1;

/* ------------------------------------------------------------------ */
/* Private helpers                                                     */
/* ------------------------------------------------------------------ */

static void pot_sensor_init(PotSensor *pot, uint gpio_pin, uint adc_channel)
{
    pot->gpio_pin    = gpio_pin;
    pot->adc_channel = adc_channel;
    pot->voltage     = 0.0f;
    pot->cal_m       = POTMON_DEFAULT_M;
    pot->cal_b       = POTMON_DEFAULT_B;

    adc_gpio_init(gpio_pin);
}

static void pot_sensor_read(PotSensor *pot)
{
    adc_select_input(pot->adc_channel);
    uint16_t raw = adc_read();
    float v = ((float)raw / (float)POTMON_ADC_MAX) * POTMON_VREF;
    pot->voltage = v;
}

/* ------------------------------------------------------------------ */
/* App interface                                                       */
/* ------------------------------------------------------------------ */

void potmon_init(uint8_t app_id)
{
    adc_init();
    pot_sensor_init(&pot0, POTMON_GPIO0, POTMON_ADC_CH0);
    pot_sensor_init(&pot1, POTMON_GPIO1, POTMON_ADC_CH1);
}

/*
 * potmon_server – accepted JSON keys
 * -----------------------------------
 * "cal_m0"    float   Slope for pot 0.
 * "cal_b0"    float   Intercept for pot 0.
 * "cal_m1"    float   Slope for pot 1.
 * "cal_b1"    float   Intercept for pot 1.
 * "cal_reset" any     Restore factory-default m/b for both channels.
 *
 * Calibration is computed externally (on the host) by collecting
 * (voltage, angle) pairs from the status stream and fitting a line.
 * The resulting m/b values are pushed here to update the mapping.
 */
void potmon_server(uint8_t app_id, const char *json_str)
{
    cJSON *root = cJSON_Parse(json_str);
    if (!root || !cJSON_IsObject(root)) {
        cJSON_Delete(root);
        return;
    }

    cJSON *item;

    /* --- restore factory defaults ----------------------------------- */
    item = cJSON_GetObjectItem(root, "cal_reset");
    if (item) {
        pot0.cal_m = POTMON_DEFAULT_M;
        pot0.cal_b = POTMON_DEFAULT_B;
        pot1.cal_m = POTMON_DEFAULT_M;
        pot1.cal_b = POTMON_DEFAULT_B;
    }

    /* --- direct parameter overrides --------------------------------- */
    item = cJSON_GetObjectItem(root, "cal_m0");
    if (item && cJSON_IsNumber(item)) pot0.cal_m = (float)item->valuedouble;

    item = cJSON_GetObjectItem(root, "cal_b0");
    if (item && cJSON_IsNumber(item)) pot0.cal_b = (float)item->valuedouble;

    item = cJSON_GetObjectItem(root, "cal_m1");
    if (item && cJSON_IsNumber(item)) pot1.cal_m = (float)item->valuedouble;

    item = cJSON_GetObjectItem(root, "cal_b1");
    if (item && cJSON_IsNumber(item)) pot1.cal_b = (float)item->valuedouble;

    cJSON_Delete(root);
}

void potmon_op(uint8_t app_id)
{
    pot_sensor_read(&pot0);
    pot_sensor_read(&pot1);
}

void potmon_status(uint8_t app_id)
{
    send_json(7,
        KV_STR,   "sensor_name",  "potmon",
        KV_INT,   "app_id",       (int)app_id,
        KV_STR,   "status",       "update",
        KV_FLOAT, "pot0_voltage", pot0.voltage,
        KV_FLOAT, "pot1_voltage", pot1.voltage,
        KV_FLOAT, "pot0_angle",   pot0.cal_m * pot0.voltage + pot0.cal_b,
        KV_FLOAT, "pot1_angle",   pot1.cal_m * pot1.voltage + pot1.cal_b
    );
}
