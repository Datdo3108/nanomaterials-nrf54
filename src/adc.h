#ifndef ADC_H
#define ADC_H

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <math.h>

void adc_init(void);
void adc_read_channel(uint8_t index);
void disable_adc(void);
void adc_read_channel_mV(uint8_t channel);

#endif // ADC_H