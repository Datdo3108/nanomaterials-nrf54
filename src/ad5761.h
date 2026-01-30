#ifndef AD5761_H
#define AD5761_H

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

#define AD5761_SYNC_NODE        DT_NODELABEL(ad5761_sync)
#define AD5761_LDAC_NODE        DT_NODELABEL(ad5761_ldac)
#define AD5761_RESET_NODE       DT_NODELABEL(ad5761_reset)
//#define AD5761_ALERT_NODE     DT_NODELABEL
//#define AD5761_CLEAR_NODE     DT_NODELABEL
#define AD5761_SYNC_NODE_II     DT_NODELABEL(ad5761_sync_ii)
#define AD5761_LDAC_NODE_II     DT_NODELABEL(ad5761_ldac_ii)

static const struct gpio_dt_spec ad5761_sync_spec = GPIO_DT_SPEC_GET(AD5761_SYNC_NODE, gpios);
static const struct gpio_dt_spec ad5761_ldac_spec = GPIO_DT_SPEC_GET(AD5761_LDAC_NODE, gpios);
static const struct gpio_dt_spec ad5761_reset_spec = GPIO_DT_SPEC_GET(AD5761_RESET_NODE, gpios);

static const struct gpio_dt_spec ad5761_sync_spec_ii = GPIO_DT_SPEC_GET(AD5761_SYNC_NODE_II, gpios);
static const struct gpio_dt_spec ad5761_ldac_spec_ii = GPIO_DT_SPEC_GET(AD5761_LDAC_NODE_II, gpios);

struct ad5761_device_str{
    struct gpio_dt_spec sync_pin;
    struct gpio_dt_spec ldac_pin;
    struct gpio_dt_spec reset_pin;
    uint8_t channel;
};

static const struct ad5761_device_str ad5761_dev_i = {
    .sync_pin = ad5761_sync_spec,
    .ldac_pin = ad5761_ldac_spec,
    .reset_pin = ad5761_reset_spec,
    .channel = 1,
};

static const struct ad5761_device_str ad5761_dev_ii = {
    .sync_pin = ad5761_sync_spec_ii,
    .ldac_pin = ad5761_ldac_spec_ii,
    .reset_pin = ad5761_reset_spec,
    .channel = 2,
};

typedef enum {
    DAC_STATE_ON,
    DAC_STATE_OFF,
} dac_state_t;

typedef enum {
    DAC_DIR_UP,
    DAC_DIR_DOWN,
} dac_dir_t;

struct dac_fsm {
    dac_state_t state;
    dac_dir_t direction;
    uint16_t value;
    uint16_t start;
    uint16_t stop;
    uint16_t step;
    bool continuous;
    uint16_t cycles;
    uint16_t cycle_counter;
    bool backward;
    bool bipolar;
};

struct dac_output_fsm {
    dac_state_t state;
    dac_dir_t direction;
    uint16_t value;
    uint16_t start;
    uint16_t stop;
    uint16_t step;
    uint16_t cycles;
    uint16_t cycle_counter;
    bool backward;
    bool bipolar;
};

void ad5761_init(const struct ad5761_device_str *ad5761_dev);
void ad5761_ldac_update(const struct ad5761_device_str *ad5761_dev);
void ad5761_reset(void);

void ad5761_readback_register(uint8_t address);
void ad5761_write_register(uint8_t address, uint16_t data);

void ad5761_readback_input_register(const struct ad5761_device_str *ad5761_dev);
void ad5761_readback_DAC_register(const struct ad5761_device_str *ad5761_dev);
void ad5761_readback_control_register(const struct ad5761_device_str *ad5761_dev);

void ad5761_one_cycle(uint8_t data);
void ad5761_one_cycle_write(uint8_t data);
void ad5761_one_cycle_read(void);
void ad5761_two_cycle_read(void);
void ad5761_two_cycle_write(uint8_t tx_data_0, uint8_t tx_data_1);

void ad5761_24bit_read(const struct ad5761_device_str *ad5761_dev);
void ad5761_24bit_write(const struct ad5761_device_str *ad5761_dev, uint8_t tx_data_0, uint8_t tx_data_1, uint8_t tx_data_2);
void ad5761_24bit_transceive(const struct ad5761_device_str *ad5761_dev, uint8_t tx_data_0, uint8_t tx_data_1, uint8_t tx_data_2);

void ad5761_generate_output_signal(const struct ad5761_device_str *ad5761_dev, uint16_t value);
void ad5761_generate_triangular(const struct ad5761_device_str *ad5761_dev, uint16_t start_value, uint16_t stop_value, uint32_t delay_us, uint16_t step, bool cycle);

void ad5761_print_dac_value(const struct ad5761_device_str *ad5761_dev, uint16_t value);

void ad5761_fsm_step_triangle(const struct ad5761_device_str *dev, struct dac_fsm *fsm);
void ad5761_fsm_step_output(const struct ad5761_device_str *dev, struct dac_output_fsm *fsm);
void ad5761_fsm_oect_output_curve(const struct ad5761_device_str *dev_1, 
                struct dac_fsm *fsm_1,
                const struct ad5761_device_str *dev_2,
                struct dac_output_fsm *fsm_2);
#endif // AD5761_H