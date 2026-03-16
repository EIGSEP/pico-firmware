<<<<<<< HEAD
#include "potmon.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "cJSON.h"
#include <math.h>

static PotSensor pot0;
static PotSensor pot1;

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
    /*read a channel*/
    adc_select_input(pot->adc_channel);
    uint16_t raw = adc_read();
    
    float v = ((float)raw / (float)POTMON_ADC_MAX) * POTMON_VREF;
    pot->voltage = v;
}

/*app interface*/

void potmon_init(uint8_t app_id)
{
    adc_init();
    pot_sensor_init(&pot0, POTMON_GPIO0, POTMON_ADC_CH0);
    pot_sensor_init(&pot1, POTMON_GPIO1, POTMON_ADC_CH1);
}

void potmon_server(uint8_t app_id, const char *json_str)
{
    //potmon does not handle json commands
}
/*read both ADC channels on every loop iteration*/
void potmon_op(uint8_t app_id)
{
    pot_sensor_read(&pot0);
    pot_sensor_read(&pot1);
}

/*send status json with voltage, resistance, r_ref, and valid status*/
void potmon_status(uint8_t app_id)
{
    send_json(7,
        KV_STR,   "sensor_name",     "potmon",
        KV_INT,   "app_id",          (int)app_id,
        KV_STR,   "status",          "update",
        KV_FLOAT, "pot0_voltage",    pot0.voltage,
        KV_FLOAT, "pot1_voltage",    pot1.voltage,
    );
}

