#ifndef SWITCH_H
#define SWITCH_H

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#define SW_IN1_NODE        DT_NODELABEL(sw_in_1)
#define SW_IN2_NODE        DT_NODELABEL(sw_in_2)

static const struct gpio_dt_spec sw_in1_spec = GPIO_DT_SPEC_GET(SW_IN1_NODE, gpios);
static const struct gpio_dt_spec sw_in2_spec = GPIO_DT_SPEC_GET(SW_IN2_NODE, gpios);

void switch_reset(void);
void switch_init(void);
void switch_voltage_config(void);
void switch_inject_current_config(void);


#endif // SWITCH_H