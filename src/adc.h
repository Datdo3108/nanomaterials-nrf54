#ifndef ADC_H
#define ADC_H

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <math.h>

extern int adc_mV;
extern int adc_ch0_mV;
extern int adc_ch1_mV;
extern int adc_ch2_mV;

void adc_init(void);
void adc_read_channel(void);
void disable_adc(void);
void adc_read_channel_mV(uint8_t channel);
void adc_read_channel_mV_all(void);

#endif // ADC_H