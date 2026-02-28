#ifndef BUTTON_H
#define BUTTON_H

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#define BUTTON_NODE        DT_NODELABEL(power_button)

static struct gpio_callback button_cb;
static const struct gpio_dt_spec button_spec = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);

int button_init(void);


#endif // BUTTON_H