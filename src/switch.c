#include "switch.h"

void switch_reset() {
    gpio_pin_set_dt(&sw_in1_spec, 0);
    gpio_pin_set_dt(&sw_in2_spec, 0);    
}

void switch_init() {
    gpio_pin_configure_dt(&sw_in1_spec, GPIO_OUTPUT);
    gpio_pin_configure_dt(&sw_in2_spec, GPIO_OUTPUT);

    switch_reset();
}

void switch_voltage_config() {
    switch_reset();
    gpio_pin_set_dt(&sw_in1_spec, 1);
}

void switch_inject_current_config() {
    switch_reset();
    gpio_pin_set_dt(&sw_in2_spec, 1);
}
