/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "ads1220.h"

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   200

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static struct ads1220 adc = ADS1220_DT_SPEC_GET(DT_NODELABEL(ads1220));

// static struct spi_dt_spec spispec = SPI_DT_SPEC_GET(DT_NODELABEL(ads1220), ADS1220_SPI_OPERATION, 0);
// uint8_t ctrl;

// uint8_t tx_buffer[3] = {0x00, 0x00, 0x00};
// uint8_t rx_buffer[3];

// void spi_readback_register(uint8_t address){
//     struct spi_buf tx_spi_buf		= {.buf = tx_buffer, .len = 3};
//     struct spi_buf_set tx_spi_buf_set 	= {.buffers = &tx_spi_buf, .count = 1};
//     struct spi_buf rx_spi_bufs 		= {.buf = rx_buffer, .len = 3};
//     struct spi_buf_set rx_spi_buf_set	= {.buffers = &rx_spi_bufs, .count = 1};
    
//     tx_buffer[0] = address;
//     tx_buffer[1] = 0x00;
//     tx_buffer[2] = 0x00;

//     int err;

//     printk("--------");
//     printk("Start reading back address: 0x%02X\n", address);

//     err = spi_write_dt(&spispec, &tx_spi_buf_set);
//     err = spi_read_dt(&spispec, &rx_spi_buf_set);
//     printk("SPI transmitted! Received: 0x%02X\t0x%02X\t0x%02X\n", rx_buffer[0], rx_buffer[1], rx_buffer[2]);
// }

int main(void)
{
        int ret;
        bool led_state = true;

        if (!gpio_is_ready_dt(&led)) {
        return 0;
        }

        ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
        return 0;
        }

        ret = ads1220_init(&adc);
 
	if (ret != 0) {
		printf("ADS1220 init failed: %d\n", ret);
		return 0;
	}

        ads1220_set_compare_channels(&adc, ADS1220_MUX_0_1);  /* AIN0 - AIN1 */
	ads1220_set_gain(&adc, ADS1220_GAIN_1);
	ads1220_set_data_rate(&adc, ADS1220_DR_LVL_0);        /* 20 SPS in normal mode */

        while (1) {
                int32_t raw;
		float mv;

                ret = ads1220_get_raw_data(&adc, &raw);
		if (ret == 0) {
			ret = ads1220_get_voltage_mv(&adc, &mv);
		}
 
		if (ret == 0) {
			printf("raw: %d  voltage: %.4f mV\n", raw, (double)mv);
		} else {
			printf("read error: %d\n", ret);
		}

                // read_reg(&adc, ADS1220_REG_1, &ctrl);
                // printf("Read register data: 0x%02X\t\t", ctrl);

                ret = gpio_pin_toggle_dt(&led);
                if (ret < 0) {
                return 0;
                }

                led_state = !led_state;
                printf("LED state: %s\n", led_state ? "ON" : "OFF");

                k_msleep(SLEEP_TIME_MS);
        }
        return 0;
}