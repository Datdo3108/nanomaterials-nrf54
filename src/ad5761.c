#include "ad5761.h"
#include "adc.h"
#include <zephyr/kernel.h>
#include <stdio.h>

/*
    SPI_MODE_CPHA: set clock phase, in order to sync correct CLK sequence

*/

#define SPIOP       SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_TRANSFER_MSB
// #define AD5761_SYNC_NODE        DT_NODELABEL(ad5761_sync)
// #define AD5761_LDAC_NODE        DT_NODELABEL(ad5761_ldac)
// #define AD5761_RESET_NODE       DT_NODELABEL(ad5761_reset)
// //#define AD5761_ALERT_NODE     DT_NODELABEL
// //#define AD5761_CLEAR_NODE     DT_NODELABEL
// #define AD5761_SYNC_NODE_II     DT_NODELABEL(ad5761_sync_ii)
// #define AD5761_LDAC_NODE_II     DT_NODELABEL(ad5761_ldac_ii)

struct spi_dt_spec spispec = SPI_DT_SPEC_GET(DT_NODELABEL(gendev), SPIOP, 0);

uint8_t tx_buffer[3] = {0x00, 0x00, 0x00};
uint8_t rx_buffer[3];

// struct spi_buf tx_spi_buf		= {.buf = (void *)&tx_buffer, .len = 1};
struct spi_buf tx_spi_buf		= {.buf = tx_buffer, .len = sizeof(tx_buffer)};
struct spi_buf_set tx_spi_buf_set 	= {.buffers = &tx_spi_buf, .count = 1};
struct spi_buf rx_spi_bufs 		= {.buf = rx_buffer, .len = sizeof(rx_buffer)};
struct spi_buf_set rx_spi_buf_set	= {.buffers = &rx_spi_bufs, .count = 1};

// static const struct gpio_dt_spec ad5761_sync_spec = GPIO_DT_SPEC_GET(AD5761_SYNC_NODE, gpios);
// static const struct gpio_dt_spec ad5761_ldac_spec = GPIO_DT_SPEC_GET(AD5761_LDAC_NODE, gpios);
// static const struct gpio_dt_spec ad5761_reset_spec = GPIO_DT_SPEC_GET(AD5761_RESET_NODE, gpios);

// static const struct gpio_dt_spec ad5761_sync_spec_ii = GPIO_DT_SPEC_GET(AD5761_SYNC_NODE_II, gpios);
// static const struct gpio_dt_spec ad5761_ldac_spec_ii = GPIO_DT_SPEC_GET(AD5761_LDAC_NODE_II, gpios);

// static const struct ad5761_device_str ad5761_dev_i = {
//     .sync_pin = ad5761_sync_spec,
//     .ldac_pin = ad5761_ldac_spec,
//     .reset_pin = ad5761_reset_spec
// };

void ad5761_init(const struct ad5761_device_str *ad5761_dev){
    // gpio_pin_configure_dt(&ad5761_sync_spec, GPIO_OUTPUT);
    // gpio_pin_configure_dt(&ad5761_ldac_spec, GPIO_OUTPUT);
    // gpio_pin_configure_dt(&ad5761_reset_spec, GPIO_OUTPUT);
    gpio_pin_configure_dt(&ad5761_dev->sync_pin, GPIO_OUTPUT);
    gpio_pin_configure_dt(&ad5761_dev->ldac_pin, GPIO_OUTPUT);
    gpio_pin_configure_dt(&ad5761_dev->reset_pin, GPIO_OUTPUT);

    gpio_pin_set_dt(&ad5761_dev->ldac_pin, 1);
    gpio_pin_set_dt(&ad5761_dev->reset_pin, 1);
    printk("DAC init!\n");
}

void ad5761_ldac_update(const struct ad5761_device_str *ad5761_dev){
    gpio_pin_set_dt(&ad5761_dev->ldac_pin, 0);
    k_busy_wait(1);
    gpio_pin_set_dt(&ad5761_dev->ldac_pin, 1);
}

// void ad5761_reset(){
//     gpio_pin_set_dt(&ad5761_reset_spec, 0);
//     k_busy_wait(1);
//     gpio_pin_set_dt(&ad5761_reset_spec, 1);
// }

/*
    Should not initialize a communication longer than 20 SCLKs
    Because continuous SCLKs is limited (cannot config this...)

*/

void ad5761_readback_register(uint8_t address){
    struct spi_buf tx_spi_buf		= {.buf = tx_buffer, .len = 3};
    struct spi_buf_set tx_spi_buf_set 	= {.buffers = &tx_spi_buf, .count = 1};
    struct spi_buf rx_spi_bufs 		= {.buf = rx_buffer, .len = 3};
    struct spi_buf_set rx_spi_buf_set	= {.buffers = &rx_spi_bufs, .count = 1};
    
    tx_buffer[0] = address;
    tx_buffer[1] = 0x00;
    tx_buffer[2] = 0x00;

    int err;

    printk("--------");
    printk("Start reading back address: 0x%02X\n", address);

    err = spi_write_dt(&spispec, &tx_spi_buf_set);
    err = spi_read_dt(&spispec, &rx_spi_buf_set);
    printk("SPI transmitted! Received: 0x%02X\t0x%02X\t0x%02X\n", rx_buffer[0], rx_buffer[1], rx_buffer[2]);
}

void ad5761_two_cycle_write(uint8_t tx_data_0, uint8_t tx_data_1){
    struct spi_buf tx_spi_buf		= {.buf = tx_buffer, .len = 2};
    struct spi_buf_set tx_spi_buf_set 	= {.buffers = &tx_spi_buf, .count = 1};
    // struct spi_buf rx_spi_bufs 		= {.buf = rx_buffer, .len = 1};
    // struct spi_buf_set rx_spi_buf_set	= {.buffers = &rx_spi_bufs, .count = 1};

    int err;

    // tx_buffer[0] = (data >> 8) & 0xFF;  
    // tx_buffer[1] = data & 0xFF;  

    tx_buffer[0] = tx_data_0;  
    tx_buffer[1] = tx_data_1;  

    // printk("--------");
    // printk("Transmitted: 0x%02X\t0x%02X\n", tx_buffer[0], tx_buffer[1]);

    err = spi_write_dt(&spispec, &tx_spi_buf_set);

}

void ad5761_one_cycle_write(uint8_t data){
    struct spi_buf tx_spi_buf		= {.buf = tx_buffer, .len = 1};
    struct spi_buf_set tx_spi_buf_set 	= {.buffers = &tx_spi_buf, .count = 1};
    // struct spi_buf rx_spi_bufs 		= {.buf = rx_buffer, .len = 1};
    // struct spi_buf_set rx_spi_buf_set	= {.buffers = &rx_spi_bufs, .count = 1};

    int err;

    tx_buffer[0] = data;

    // printk("--------");
    // printk("Transmitted: 0x%02X\n", data);

    err = spi_write_dt(&spispec, &tx_spi_buf_set);

}

void ad5761_two_cycle_read(){
    // struct spi_buf tx_spi_buf		= {.buf = tx_buffer, .len = 1};
    // struct spi_buf_set tx_spi_buf_set 	= {.buffers = &tx_spi_buf, .count = 1};
    struct spi_buf rx_spi_bufs 		= {.buf = rx_buffer, .len = 2};
    struct spi_buf_set rx_spi_buf_set	= {.buffers = &rx_spi_bufs, .count = 1};

    int err;
    err = spi_read_dt(&spispec, &rx_spi_buf_set);

    printk("SPI transmitted! Received: 0x%02X\t0x%02X\n", rx_buffer[0], rx_buffer[1]);
    printk("\n");

}

void ad5761_one_cycle_read(){
    // struct spi_buf tx_spi_buf		= {.buf = tx_buffer, .len = 1};
    // struct spi_buf_set tx_spi_buf_set 	= {.buffers = &tx_spi_buf, .count = 1};
    struct spi_buf rx_spi_bufs 		= {.buf = rx_buffer, .len = 1};
    struct spi_buf_set rx_spi_buf_set	= {.buffers = &rx_spi_bufs, .count = 1};

    int err;
    err = spi_read_dt(&spispec, &rx_spi_buf_set);

    printk("SPI transmitted! Received: 0x%02X\n\n", rx_buffer[0]);
}

void ad5761_one_cycle(uint8_t data){
    struct spi_buf tx_spi_buf		= {.buf = tx_buffer, .len = 1};
    struct spi_buf_set tx_spi_buf_set 	= {.buffers = &tx_spi_buf, .count = 1};
    struct spi_buf rx_spi_bufs 		= {.buf = rx_buffer, .len = 1};
    struct spi_buf_set rx_spi_buf_set	= {.buffers = &rx_spi_bufs, .count = 1};

    int err;

    tx_buffer[0] = data;

    printk("--------");
    printk("Tranismitted: 0x%02X\n", data);

    err = spi_transceive_dt(&spispec, &tx_spi_buf_set, &rx_spi_buf_set);

    printk("SPI transmitted! Received: 0x%02X\n", rx_buffer[0]);
}  

//////// Readback format ////////
        /*
        4-bit:  ---0 (first 3-bit don't care)
        4-bit:  1010: Readback input register   (0x0A)
                1011: Readback DAC register     (0x0B)
                1100: Readback control register (0x0C)
        16-bit data: 0000 0000 (don't care)
        */

void ad5761_write_register(uint8_t address, uint16_t data){
    struct spi_buf tx_spi_buf		= {.buf = tx_buffer, .len = 3};
    struct spi_buf_set tx_spi_buf_set 	= {.buffers = &tx_spi_buf, .count = 1};

    tx_buffer[0] = address;
    tx_buffer[1] = (data >> 8) & 0xFF;  
    tx_buffer[2] = data & 0xFF;       

    int err;

    printk("--------");
    printk("Start writing address: 0x%02X\n", address);

    err = spi_write_dt(&spispec, &tx_spi_buf_set);
}

void ad5761_24bit_transceive(const struct ad5761_device_str *ad5761_dev, uint8_t tx_data_0, uint8_t tx_data_1, uint8_t tx_data_2){
    gpio_pin_set_dt(&ad5761_dev->sync_pin, 0);
    ad5761_one_cycle(tx_data_0);
    ad5761_one_cycle(tx_data_1);
    ad5761_one_cycle(tx_data_2);
    gpio_pin_set_dt(&ad5761_dev->sync_pin, 1);
}

void ad5761_24bit_read(const struct ad5761_device_str *ad5761_dev){
    gpio_pin_set_dt(&ad5761_dev->sync_pin, 0);
    ad5761_two_cycle_read();
    ad5761_one_cycle_read();
    gpio_pin_set_dt(&ad5761_dev->sync_pin, 1);
}

void ad5761_24bit_write(const struct ad5761_device_str *ad5761_dev, uint8_t tx_data_0, uint8_t tx_data_1, uint8_t tx_data_2){
    gpio_pin_set_dt(&ad5761_dev->sync_pin, 0);
    ad5761_two_cycle_write(tx_data_0, tx_data_1);
    ad5761_one_cycle_write(tx_data_2);
    gpio_pin_set_dt(&ad5761_dev->sync_pin, 1);
}

void ad5761_readback_input_register(const struct ad5761_device_str *ad5761_dev){
    ad5761_24bit_write(ad5761_dev, 0x0A, 0x00, 0x00);
    ad5761_24bit_read(ad5761_dev);
}

void ad5761_readback_DAC_register(const struct ad5761_device_str *ad5761_dev){
    ad5761_24bit_write(ad5761_dev, 0x0B, 0x00, 0x00);
    ad5761_24bit_read(ad5761_dev);
}

void ad5761_readback_control_register(const struct ad5761_device_str *ad5761_dev){
    ad5761_24bit_write(ad5761_dev, 0x0C, 0x00, 0x00);
    ad5761_24bit_read(ad5761_dev);
}

double ad5761_convert_to_mV(uint16_t value, uint16_t vref_n_mV, uint16_t vref_p_mV){
    double dac_mV = (double)value/65536*(vref_p_mV - vref_n_mV);

    return dac_mV;
}

void ad5761_generate_output_signal(const struct ad5761_device_str *ad5761_dev, uint16_t value){
    uint8_t byte_0 = (value >> 8) & 0xFF;  
    uint8_t byte_1 = value & 0xFF;     

    ad5761_24bit_write(ad5761_dev, 0x01, byte_0, byte_1);
    ad5761_ldac_update(ad5761_dev);

    double dac_mV = ad5761_convert_to_mV(value, 0, 2620);
    printk("DAC:\tChannel: %u\tSet value (mV): %.2f\n", ad5761_dev->channel, dac_mV);
    printk("DAC:\tSet value: %u\n", value);
}

void ad5761_print_dac_value(const struct ad5761_device_str *ad5761_dev, uint16_t value){
    double dac_mV = ad5761_convert_to_mV(value, 0, 2620);
    printk("DAC:\tChannel: %u\tSet value (mV): %.2f\n", ad5761_dev->channel, dac_mV);
    printk("DAC:\tSet value: %u\n", value);
}

void ad5761_software_full_reset(const struct ad5761_device_str *ad5761_dev){
    ad5761_24bit_write(ad5761_dev, 0x0F, 0x00, 0x00);   // Software full reset
}

void ad5761_generate_triangular(const struct ad5761_device_str *ad5761_dev, uint16_t start_value, uint16_t stop_value, uint32_t delay_us, uint16_t step, bool cycle){  
    uint16_t value = start_value;
    if(start_value < stop_value){
        while(value < stop_value){
            ad5761_generate_output_signal(ad5761_dev, value);
            value += step;
            k_busy_wait(delay_us);
            // adc_read_channel_mV(0);         /* Put this in state machine */
            // adc_read_channel_mV(1);
            // adc_read_channel_mV(2);
        }

        if(cycle == 1){
            while(value > start_value){
                ad5761_generate_output_signal(ad5761_dev, value);
                value -= step;
            k_busy_wait(delay_us);
            // adc_read_channel_mV(0);         /* Put this in state machine */
            // adc_read_channel_mV(1);
            // adc_read_channel_mV(2);
            }
        }
    }

    if(start_value > stop_value){
        while(value > stop_value){
            ad5761_generate_output_signal(ad5761_dev, value);
            value -= step;
            k_busy_wait(delay_us);
        }

        if(cycle == 1){
            while(value < start_value){
                ad5761_generate_output_signal(ad5761_dev, value);
                value += step;
            k_busy_wait(delay_us);
            }
        }
    }
}

void ad5761_fsm_step_output(const struct ad5761_device_str *dev,
                struct dac_output_fsm *fsm)
{
    if (fsm->state != DAC_STATE_ON) {
        if(fsm->bipolar == false){
            fsm->value = 0;
        } else {
            fsm->value = 32768;
        }
        ad5761_generate_output_signal(dev, fsm->value);
        return;
    }

    ad5761_generate_output_signal(dev, fsm->value);

    fsm->value = fsm->start + fsm->step * fsm->cycle_counter;
    fsm->state = DAC_STATE_OFF;
}

void ad5761_fsm_step_triangle(const struct ad5761_device_str *dev,
                struct dac_fsm *fsm)
{
    if (fsm->state != DAC_STATE_ON) {
        if(fsm->bipolar == false){
            fsm->value = 0;
        } else {
            fsm->value = 32768;
        }

        ad5761_generate_output_signal(dev, fsm->value);
        return;
    }

    ad5761_generate_output_signal(dev, fsm->value);

    if (fsm->direction == DAC_DIR_UP) {
        if (fsm->value + fsm->step >= fsm->stop) {
            if (fsm->backward == true){
                fsm->value = fsm->stop;
                fsm->direction = DAC_DIR_DOWN;
            } else {
                fsm->value = fsm->start;
            }
        } else {
            fsm->value += fsm->step;
        }
    } else {
        if (fsm->value <= fsm->start + fsm->step) {
            // if (fsm->continuous) {
            if (fsm->cycle_counter < fsm->cycles - 1) {
                fsm->value = fsm->start;
                fsm->direction = DAC_DIR_UP;
                fsm->cycle_counter += 1;
            } else {
                fsm->state = DAC_STATE_OFF;
                fsm->cycle_counter = 0;
            }
        } else {
            fsm->value -= fsm->step;
        }
    }
}

void ad5761_fsm_oect_output_curve(const struct ad5761_device_str *dev_1, 
                struct dac_fsm *fsm_1,
                const struct ad5761_device_str *dev_2,
                struct dac_output_fsm *fsm_2)
{
    fsm_2->state = fsm_1->state;
    fsm_2->cycle_counter = fsm_1->cycle_counter;

    ad5761_fsm_step_triangle(dev_1, fsm_1);
    ad5761_fsm_step_output(dev_2, fsm_2);
}

