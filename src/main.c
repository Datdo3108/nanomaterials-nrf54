#include "ad5761.h"
#include "ble.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

#define SPI_DEBUG
// #define BLE_DEBUG

#ifdef BLE_DEBUG

int main(void){
        ble_init();

        // int err;
        // err = bt_enable(bt_ready);

        while(1){
                if(default_conn){
			send_notify_data(default_conn);
		}
		k_msleep(250);
        }

        return 0;
}

#endif

#if(0)

#include <zephyr/logging/log.h>

#define LOG_MODULE_NAME my_spi
LOG_MODULE_REGISTER(LOG_MODULE_NAME);
#endif

#ifdef SPI_DEBUG

#define LED_NODE	DT_NODELABEL(led3)

int main(void){
        ad5761_init();

        static const struct gpio_dt_spec led_spec = GPIO_DT_SPEC_GET(LED_NODE, gpios);
	gpio_pin_configure_dt(&led_spec, GPIO_OUTPUT);

        while(1){
                // // Software full reset
                printk("Software full reset: \n");
                ad5761_24bit_write(0x0F, 0x00, 0x00);   // Software full reset
                ad5761_24bit_write(0x04, 0x00, 0x43);   // Write to control register

                ad5761_readback_control_register();

                printk("Control register completed!********\n");

                ad5761_generate_triangular();

                //// LED blink ////

                gpio_pin_set_dt(&led_spec, 1);
                printk("LED ON\n");
                // k_msleep(1000);
                k_busy_wait(1000000);
                gpio_pin_set_dt(&led_spec, 0);
                printk("LED OFF\n");
                // k_msleep(1000);
                k_busy_wait(1000000);
        }

        return 0;
}

#endif

#if(0)
//////// SPI initialize ////////

// #define SPI_NODE        DT_NODELABEL(spi21)
// static const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);

// #define MY_GPIO         DT_NODELABEL(gpio1)
// #define GPIO_1_CS       7
// const struct device *gpio1_dev = DEVICE_DT_GET(MY_GPIO);

// static struct spi_config spi_cfg = {
//         .frequency = 125000U,
//         .operation = SPI_WORD_SET(8),
//         .slave = 0,
//         // .cs = &spi_cs,
// };

// static void readRegister(uint8_t reg, uint8_t values[], uint8_t size){
//         int err;

//         uint8_t tx_buffer[1];
//         tx_buffer[0] = reg;

//         struct spi_buf tx_spi_bufs[] = {
//                 {.buf = tx_buffer, .len = sizeof(tx_buffer)},
//         };

//         struct spi_buf rx_spi_bufs[] = {
//                 {.buf = values, .len = size}
//         };

//         struct spi_buf_set spi_rx_buffer_set[] = {
//                 .buffers = rx_spi_bufs,
//                 .count = 1
//         };

//         gpio_pin_set(gpio1_dev, GPIO_1_CS, 1);
//         do{
//                 err = spi_write(spi_dev, &spi_cfg, &spi_tx_buffer_set());

//         }
// };

// #define SPI_MESSAGE             0xA5

// const struct spi_config         spi_cfg = {
//         .frequency = DT_PROP(DT_NODELABEL(spi21), clock_frequency),
//         .operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8),
//         // .cs = SPI_CS_CONTROL_PTR_DT(DT_NODELABEL(spidev), 10),
// };
#endif

#if(0)
#define SPIOP      SPI_WORD_SET(8) | SPI_TRANSFER_MSB
#define LED_NODE	DT_NODELABEL(led3)
#define AD5761_SYNC_NODE        DT_NODELABEL(ad5761_sync)


struct spi_dt_spec spispec = SPI_DT_SPEC_GET(DT_NODELABEL(gendev), SPIOP, 0);


int main(void)
{
        static const struct gpio_dt_spec led_spec = GPIO_DT_SPEC_GET(LED_NODE, gpios);
	gpio_pin_configure_dt(&led_spec, GPIO_OUTPUT);

        static const struct gpio_dt_spec ad5761_sync_spec = GPIO_DT_SPEC_GET(AD5761_SYNC_NODE, gpios);
	gpio_pin_configure_dt(&ad5761_sync_spec, GPIO_OUTPUT);

        int err = spi_is_ready_dt(&spispec);
        if (!err) {
                // LOG_ERR("Error: SPI device is not ready, err: %d", err);
                printk("Error: SPI device is not ready, err: %d\n", err);
                return 0;
        }
        else{
                // LOG_INF("SPI device is ready");
                printk("SPI device is ready\n");
        }

        //////// Readback format ////////
        /*
        4-bit: 0000 (first 3-bit don't care)
        4-bit:  1010: Readback input register   (0x0A)
                1011: Readback DAC register     (0x0B)
                1100: Readback control register (0x0C)
        16-bit data: 0000 0000 (don't care)
        */

        uint8_t tx_buffer[3] = {0x0B, 0x00, 0x00};
        uint8_t data[3];
        size_t size = 3;
        // struct spi_buf tx_spi_buf		= {.buf = (void *)&tx_buffer, .len = 1};
        struct spi_buf tx_spi_buf		= {.buf = tx_buffer, .len = sizeof(tx_buffer)};
        struct spi_buf_set tx_spi_buf_set 	= {.buffers = &tx_spi_buf, .count = 1};
        struct spi_buf rx_spi_bufs 		= {.buf = data, .len = size};
        struct spi_buf_set rx_spi_buf_set	= {.buffers = &rx_spi_bufs, .count = 1};

        while(1){
                //// Transceive: Read & Write ////
                // printk("-------- Read & Write 0x0B \n");
                // tx_buffer[0] = 0x0B;

                // gpio_pin_set_dt(&ad5761_sync_spec, 0);
                
                // err = spi_transceive_dt(&spispec, &tx_spi_buf_set, &rx_spi_buf_set);
                // if (err < 0) {
                //         // LOG_ERR("spi_transceive_dt() failed, err: %d", err);
                //         printk("spi_transceive_dt() failed, err: %d\n", err);
                // // return err;
                // }
                // else{
                //         // LOG_INF("SPI transmitted! Received: 0x%02X", data[0]);
                //         printk("SPI transmitted! Received: 0x%02X\t0x%02X\t0x%02X\n", data[0], data[1], data[2]);
                // }
                // gpio_pin_set_dt(&ad5761_sync_spec, 1);

                // k_busy_wait(1);

                //// Read then write ////
                printk("-------- Read - Delay - Write 0x0A \n");
                tx_buffer[0] = 0x0A;

                gpio_pin_set_dt(&ad5761_sync_spec, 0);
                err = spi_write_dt(&spispec, &tx_spi_buf_set);
                gpio_pin_set_dt(&ad5761_sync_spec, 1);
                k_busy_wait(1);
                gpio_pin_set_dt(&ad5761_sync_spec, 0);
                err = spi_read_dt(&spispec, &rx_spi_buf_set);
                printk("SPI transmitted! Received: 0x%02X\t0x%02X\t0x%02X\n", data[0], data[1], data[2]);
                gpio_pin_set_dt(&ad5761_sync_spec, 1);

                k_busy_wait(1);

                //// Transceive: Read & Write ////
                // printk("-------- Read & Write 0x0A \n");
                // tx_buffer[0] = 0x0A;

                // gpio_pin_set_dt(&ad5761_sync_spec, 0);
                
                // err = spi_transceive_dt(&spispec, &tx_spi_buf_set, &rx_spi_buf_set);
                // printk("SPI transmitted! Received: 0x%02X\t0x%02X\t0x%02X\n", data[0], data[1], data[2]);
                // gpio_pin_set_dt(&ad5761_sync_spec, 1);

                // k_busy_wait(1);

                //// Read then write ////
                printk("-------- Read - Delay - Write 0x0B \n");
                tx_buffer[0] = 0x0B;

                gpio_pin_set_dt(&ad5761_sync_spec, 0);
                err = spi_write_dt(&spispec, &tx_spi_buf_set);
                gpio_pin_set_dt(&ad5761_sync_spec, 1);
                k_busy_wait(1);
                gpio_pin_set_dt(&ad5761_sync_spec, 0);
                err = spi_read_dt(&spispec, &rx_spi_buf_set);
                printk("SPI transmitted! Received: 0x%02X\t0x%02X\t0x%02X\n", data[0], data[1], data[2]);
                gpio_pin_set_dt(&ad5761_sync_spec, 1);

                k_busy_wait(1);

                //// Read then write ////
                printk("-------- Read - Delay - Write 0x0C \n");
                tx_buffer[0] = 0x0C;

                gpio_pin_set_dt(&ad5761_sync_spec, 0);
                err = spi_write_dt(&spispec, &tx_spi_buf_set);
                gpio_pin_set_dt(&ad5761_sync_spec, 1);
                k_busy_wait(1);
                gpio_pin_set_dt(&ad5761_sync_spec, 0);
                err = spi_read_dt(&spispec, &rx_spi_buf_set);
                printk("SPI transmitted! Received: 0x%02X\t0x%02X\t0x%02X\n", data[0], data[1], data[2]);
                gpio_pin_set_dt(&ad5761_sync_spec, 1);

                k_busy_wait(1);

                //// LED blink ////

                gpio_pin_set_dt(&led_spec, 1);
                printk("LED ON\n");
                // k_msleep(1000);
                k_busy_wait(1000000);
                gpio_pin_set_dt(&led_spec, 0);
                printk("LED OFF\n");
                // k_msleep(1000);
                k_busy_wait(1000000);

                //////// Read back from AD5761 ////////


        }
        return 0;
}
#endif