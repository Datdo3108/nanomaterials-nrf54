#include "adc.h"
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <stdio.h>

#define SEQUENCE_SAMPLE     1
#define ADC_NODE            DT_NODELABEL(adc)

float adc_average = 0.0f;
int adc_mV;
uint16_t window_len = 1;

static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);
static const struct adc_channel_cfg channel_cfgs[] = {
    DT_FOREACH_CHILD_SEP(ADC_NODE, ADC_CHANNEL_CFG_DT, (,))
};

uint16_t channel_reading[SEQUENCE_SAMPLE];

void adc_init(void) {
    if (!device_is_ready(adc_dev)) {
        printf("ADC controller device %s not ready\n", adc_dev->name);
        return;
    }
    printf("ADC initialized!\n");
}

/* Read channel by selecting index */
void adc_read_channel(uint8_t index){
    int err;
    uint32_t adc_sum = 0;
    uint16_t count_adc = 0;

    struct adc_sequence sequence = {
        .buffer = channel_reading,
        .buffer_size = sizeof(channel_reading),
        .resolution = 12,
        .channels = BIT(channel_cfgs[index].channel_id),
        .calibrate = true,
    };

    err = adc_channel_setup(adc_dev, &channel_cfgs[index]);
    if (err < 0) {
        printk("Could not setup channel %d (%d)\n", index, err);
        return;
    }

    while (count_adc < window_len) {
        err = adc_read(adc_dev, &sequence);
        if (err < 0) {
            printk("Could not read channel %d (%d)\n", index, err);
            continue;
        }
        adc_sum += channel_reading[0];
        count_adc++;
    }

    adc_average = (float)adc_sum / window_len;
}

// change adc to mVoltage
int adc_to_mV(float adc, uint8_t gain){
    int value;
    value = adc/4096*gain*0.9f*1000.0f;  // 0.9: external reference voltage 0.9V, 4: gain = 1/4
    return (int)value;
}

void adc_read_channel_mV(uint8_t channel) {
    adc_read_channel(channel);
    if(channel == 0){
        adc_mV = adc_to_mV(adc_average, 4);
    }
    else if (channel == 1)
    {
        adc_mV = adc_to_mV(adc_average, 4);
    }
    else if (channel == 2)
    {
        adc_mV = adc_to_mV(adc_average, 4);
    }

    printk("ADC\tChannel: %u\t Read value: %.2f\t Voltage value: %umV\n", channel, (double)adc_average, adc_mV);
}


void disable_adc(void) {
    pm_device_action_run(adc_dev, PM_DEVICE_ACTION_SUSPEND);
}