#include "adc.h"
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <stdio.h>

#define ADC_FAST
// #define ADC_OLD_VERSION

// Old version: SEQUENCE_SAMPLE = 1
#define SEQUENCE_SAMPLE     64
#define ADC_NODE            DT_NODELABEL(adc)
#define NUM_CHANNELS        3

float adc_average = 0.0f;
float adc_ch0_average = 0.0f;
float adc_ch1_average = 0.0f;
float adc_ch2_average = 0.0f;
int adc_mV;
int adc_ch0_mV;
int adc_ch1_mV;
int adc_ch2_mV;
uint16_t window_len = 1;

static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);
static const struct adc_channel_cfg channel_cfgs[] = {
    DT_FOREACH_CHILD_SEP(ADC_NODE, ADC_CHANNEL_CFG_DT, (,))
};

// Old version: channel_reading[SEQUENCE_SAMPLE]
uint16_t channel_reading[SEQUENCE_SAMPLE][NUM_CHANNELS];

const struct adc_sequence_options options = {
		.extra_samplings = SEQUENCE_SAMPLE - 1,
		.interval_us = 0,
	};

struct adc_sequence sequence = {
    .buffer = channel_reading,
    .buffer_size = sizeof(channel_reading),
    .resolution = 12,
    // .channels = BIT(channel_cfgs[index].channel_id),
    // .channels = BIT(channel_cfgs[0].channel_id) | BIT(channel_cfgs[1].channel_id) | BIT(channel_cfgs[2].channel_id),
    .channels = BIT(0) | BIT(1) | BIT(2),
    .calibrate = true,
    .options = &options,
};

void adc_init(void) {
    int err;

    for (int i = 0; i < ARRAY_SIZE(channel_cfgs); i++) {
        err = adc_channel_setup(adc_dev, &channel_cfgs[i]);
        if (err < 0) {
            printk("ADC channel %d setup failed (%d)\n", i, err);
        }
    }

    if (!device_is_ready(adc_dev)) {
        printf("ADC controller device %s not ready\n", adc_dev->name);
        return;
    }
    printf("ADC initialized!\n");
}

#ifdef ADC_TEST_CODE
void adc_read_channels(void)
{
    int err;
    int16_t buffer[64 * 2]; // 2 channels

    struct adc_sequence sequence = {
        .buffer = buffer,
        .buffer_size = sizeof(buffer),
        .resolution = 12,
        .channels = BIT(CH0) | BIT(CH1),
    };

    err = adc_read(adc_dev, &sequence);
    if (err < 0) {
        printk("ADC read error (%d)\n", err);
        return;
    }

    uint32_t sum_ch0 = 0, sum_ch1 = 0;

    for (int i = 0; i < 64; i++) {
        sum_ch0 += buffer[2*i];
        sum_ch1 += buffer[2*i + 1];
    }

    adc_avg_ch0 = sum_ch0 / 64;
    adc_avg_ch1 = sum_ch1 / 64;
}
#endif

#ifdef ADC_FAST
void adc_read_channel(){
    int err;
    // uint32_t adc_sum = 0;
    uint32_t adc_ch0_sum = 0;
    uint32_t adc_ch1_sum = 0;
    uint32_t adc_ch2_sum = 0;

    // const struct adc_sequence_options options = {
	// 	.extra_samplings = SEQUENCE_SAMPLE - 1,
	// 	.interval_us = 0,
	// };

    // struct adc_sequence sequence = {
    //     .buffer = channel_reading,
    //     .buffer_size = sizeof(channel_reading),
    //     .resolution = 12,
    //     // .channels = BIT(channel_cfgs[index].channel_id),
    //     // .channels = BIT(channel_cfgs[0].channel_id) | BIT(channel_cfgs[1].channel_id) | BIT(channel_cfgs[2].channel_id),
    //     .channels = BIT(channel_cfgs[0].channel_id) | BIT(channel_cfgs[1].channel_id) | BIT(channel_cfgs[1].channel_id),
    //     .calibrate = true,
	// 	.options = &options,
    // };

    // for (int i = 0; i < ARRAY_SIZE(channel_cfgs); i++) {
    //     err = adc_channel_setup(adc_dev, &channel_cfgs[i]);
    //     if (err < 0) {
    //         printk("ADC channel %d setup failed (%d)\n", i, err);
    //     }
    // }

    err = adc_read(adc_dev, &sequence);

    for (int i = 0; i < SEQUENCE_SAMPLE; i++) {
        // adc_sum += channel_reading[i];
        adc_ch0_sum += channel_reading[i][0];
        adc_ch1_sum += channel_reading[i][1];
        adc_ch2_sum += channel_reading[i][2];
    }

    #ifdef DEBUG_ADC_PRINT
    printk("Channel config %u\n", channel_cfgs[0].channel_id);
    printk("Channel config %u\n", channel_cfgs[1].channel_id);
    printk("Channel config %u\n", channel_cfgs[2].channel_id);

    for (int i = 0; i < NUM_CHANNELS; i++) {
        for(int j = 0; j < SEQUENCE_SAMPLE; j++){
            printk("Index %u\t%u: %u \n", i, j, channel_reading[j][i]);
        }
    }
    printk("----------------------------\n");
    #endif

    // adc_average = (float)adc_sum / SEQUENCE_SAMPLE;
    adc_ch0_average = (float)adc_ch0_sum / SEQUENCE_SAMPLE;
    adc_ch1_average = (float)adc_ch1_sum / SEQUENCE_SAMPLE;
    adc_ch2_average = (float)adc_ch2_sum / SEQUENCE_SAMPLE;
}
#endif

#ifdef ADC_OLD_VERSION
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
#endif

// change adc to mVoltage
int adc_to_mV(float adc, uint8_t gain){
    int value;
    value = adc/4096*gain*0.9f*1000.0f;  // 0.9: external reference voltage 0.9V, 4: gain = 1/4
    return (int)value;
}

void adc_read_channel_mV(uint8_t channel) {
    adc_read_channel();
    if(channel == 0){
        adc_mV = adc_to_mV(adc_ch0_average, 4);
    }
    else if (channel == 1)
    {
        adc_mV = adc_to_mV(adc_ch1_average, 4);
    }
    else if (channel == 2)
    {
        adc_mV = adc_to_mV(adc_ch2_average, 4);
    }

    // printk("ADC\tChannel: %u\t Read value: %.2f\t Voltage value: %umV\n", channel, (double)adc_average, adc_mV);
}

void adc_read_channel_mV_all() {
    adc_read_channel();
    adc_ch0_mV = adc_to_mV(adc_ch0_average, 4);
    adc_ch1_mV = adc_to_mV(adc_ch1_average, 4);
    adc_ch2_mV = adc_to_mV(adc_ch2_average, 4);
}

void disable_adc(void) {
    pm_device_action_run(adc_dev, PM_DEVICE_ACTION_SUSPEND);
}