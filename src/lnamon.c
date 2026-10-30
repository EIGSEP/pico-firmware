#include "lnamon.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"

static float adc_voltage = 0.0f;
static float rail_voltage = 0.0f;
static absolute_time_t next_sample;

void lnamon_init(uint8_t app_id)
{
    adc_init();
    adc_gpio_init(LNAMON_GPIO);
    next_sample = get_absolute_time();
}

/* No commands: the monitor is read-only. */
void lnamon_server(uint8_t app_id, const char *json_str)
{
}

void lnamon_op(uint8_t app_id)
{
    if (absolute_time_diff_us(get_absolute_time(), next_sample) > 0) {
        return;
    }
    next_sample = make_timeout_time_ms(LNAMON_SAMPLE_PERIOD_MS);

    adc_select_input(LNAMON_ADC_CH);
    (void)adc_read();  // discard first conversion after selecting the input

    uint32_t total = 0;
    for (uint i = 0; i < LNAMON_ADC_SAMPLES; i++) {
        total += adc_read();
    }
    float counts = (float)total / (float)LNAMON_ADC_SAMPLES;
    adc_voltage = counts * LNAMON_VREF / (float)LNAMON_ADC_MAX;
    rail_voltage = LNAMON_CAL_GAIN * (adc_voltage * LNAMON_DIVIDER_GAIN)
                   + LNAMON_CAL_OFFSET;
}

void lnamon_status(uint8_t app_id)
{
    send_json(5,
        KV_STR,   "sensor_name",      "lnamon",
        KV_INT,   "app_id",           (int)app_id,
        KV_STR,   "status",           "update",
        KV_FLOAT, "lna_adc_voltage",  adc_voltage,
        KV_FLOAT, "lna_rail_voltage", rail_voltage
    );
}
