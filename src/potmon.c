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
}

void potmon_server(uint8_t app_id, const char *json_str)
{
    //potmon does not handle json commands
}
void potmon_op(uint8_t app_id)
{
    pot_sensor_read(&pot_az);
}

void potmon_status(uint8_t app_id)
{
    send_json(4,
        KV_STR,   "sensor_name",    "potmon",
        KV_INT,   "app_id",         (int)app_id,
        KV_STR,   "status",         "update",
        KV_FLOAT, "pot_az_voltage", pot_az.voltage
    );
}

