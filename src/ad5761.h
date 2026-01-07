#ifndef AD5761_H
#define AD5761_H

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

void ad5761_init(void);
void ad5761_ldac_update(void);
void ad5761_reset(void);

void ad5761_readback_register(uint8_t address);
void ad5761_write_register(uint8_t address, uint16_t data);

void ad5761_readback_input_register(void);
void ad5761_readback_DAC_register(void);
void ad5761_readback_control_register(void);

void ad5761_one_cycle(uint8_t data);
void ad5761_one_cycle_write(uint8_t data);
void ad5761_one_cycle_read(void);
void ad5761_two_cycle_read(void);
void ad5761_two_cycle_write(uint8_t tx_data_0, uint8_t tx_data_1);

void ad5761_24bit_read(void);
void ad5761_24bit_write(uint8_t tx_data_0, uint8_t tx_data_1, uint8_t tx_data_2);
void ad5761_24bit_transceive(uint8_t tx_data_0, uint8_t tx_data_1, uint8_t tx_data_2);

void ad5761_generate_output_signal(uint16_t value);
void ad5761_generate_triangular(void);



#endif // AD5761_H