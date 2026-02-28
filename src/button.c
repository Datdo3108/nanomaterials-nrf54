#include "button.h"
#include <zephyr/kernel.h>
#include <stdio.h>

static struct k_poll_signal button_sig;

void button_isr(const struct device *dev,
                struct gpio_callback *cb,
                uint32_t pins)
{
    k_poll_signal_raise(&button_sig, 0);
}

int button_init()
{
    int ret;

    /* Configure button pin as input */
    ret = gpio_pin_configure_dt(&button_spec, GPIO_INPUT);
    if (ret) {
        return ret;
    }

    /* Configure interrupt on button press */
    ret = gpio_pin_interrupt_configure_dt(&button_spec,
                                          GPIO_INT_EDGE_TO_ACTIVE);
    if (ret) {
        return ret;
    }

    /* Initialize and register callback */
    gpio_init_callback(&button_cb,
                       button_isr,
                       BIT(button_spec.pin));

    gpio_add_callback(button_spec.port, &button_cb);

    return 0;
}