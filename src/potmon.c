#include "potmon.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "cJSON.h"
#include <math.h>

static PotSensor pot_az;

/*helper func to init one pot sensor*/
static void pot_sensor_init(PotSensor *pot, uint gpio_pin, uint adc_channel)
{
    pot->gpio_pin = gpio_pin;
    pot->adc_channel = adc_channel;
    pot->voltage = 0.0f;
    adc_gpio_init(gpio_pin);
}

/*helper func to read a value off of a pot sensor*/
static void pot_sensor_read(PotSensor *pot)
{
    adc_select_input(pot->adc_channel);
    uint16_t raw = adc_read();
    pot->voltage = ((float)raw / (float)POTMON_ADC_MAX) * POTMON_VREF;
}

/*app interface*/

void potmon_init(uint8_t app_id)
{
    adc_init();
    pot_sensor_init(&pot_az, POTMON_GPIO_AZ, POTMON_ADC_CH_AZ);
    /* SP1 failsafe termination: boot in SHORT (the failsafe level). */
    gpio_init(POTMON_GPIO_SP1_TERM);
    gpio_set_dir(POTMON_GPIO_SP1_TERM, GPIO_OUT);
    gpio_put(POTMON_GPIO_SP1_TERM, POTMON_SP1_TERM_SHORT);
}

void potmon_server(uint8_t app_id, const char *json_str)
{
    cJSON *root = cJSON_Parse(json_str);
    if (!root || !cJSON_IsObject(root)) {
        cJSON_Delete(root);
        return;
    }
    cJSON *term_json = cJSON_GetObjectItem(root, "sp1_term");
    if (term_json && cJSON_IsNumber(term_json)) {
        double v = cJSON_GetNumberValue(term_json);
        /* Exact 0 or 1 only; anything else is ignored. */
        if (v == 0.0 || v == 1.0) {
            gpio_put(POTMON_GPIO_SP1_TERM, v == 1.0);
        }
    }
    cJSON_Delete(root);
}
void potmon_op(uint8_t app_id)
{
    pot_sensor_read(&pot_az);
}

void potmon_status(uint8_t app_id)
{
    send_json(5,
        KV_STR,   "sensor_name",    "potmon",
        KV_INT,   "app_id",         (int)app_id,
        KV_STR,   "status",         "update",
        KV_FLOAT, "pot_az_voltage", pot_az.voltage,
        KV_INT,   "sp1_term",       (int)gpio_get(POTMON_GPIO_SP1_TERM)
    );
}

