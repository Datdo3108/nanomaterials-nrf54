#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

#define LED00_NODE        DT_NODELABEL(led_00)
#define LED01_NODE        DT_NODELABEL(led_01)

#define REG_STATUS   0x00
#define STATUS_READY_MASK 0x01

static const struct i2c_dt_spec lmp91000 = I2C_DT_SPEC_GET(DT_NODELABEL(lmp91000));

static const struct gpio_dt_spec led_00_spec = GPIO_DT_SPEC_GET(LED00_NODE, gpios);
static const struct gpio_dt_spec led_01_spec = GPIO_DT_SPEC_GET(LED01_NODE, gpios);


int lmp91000_read_status(uint8_t *status)
{
    if (!i2c_is_ready_dt(&lmp91000)) {
        printk("I2C bus not ready\n");
        return -ENODEV;
    }

    return i2c_reg_read_byte_dt(&lmp91000, REG_STATUS, status);
}

int main(void)
{
        gpio_pin_configure_dt(&led_00_spec, GPIO_OUTPUT);
        gpio_pin_configure_dt(&led_01_spec, GPIO_OUTPUT);
        gpio_pin_set_dt(&led_00_spec, 0);
        gpio_pin_set_dt(&led_01_spec, 0);

        uint8_t status;
        int ret;

        while(1) {
                ret = lmp91000_read_status(&status);
                if (ret) {
                        gpio_pin_set_dt(&led_00_spec, 1);
                        k_busy_wait(1000000);
                        gpio_pin_set_dt(&led_00_spec, 0);
                        k_busy_wait(1000000);

                        // printk("STATUS read failed: %d\n", ret);
                        return -ENODEV;
                }
                else{
                        gpio_pin_set_dt(&led_01_spec, 1);
                        k_busy_wait(200000);
                        gpio_pin_set_dt(&led_01_spec, 0);
                        k_busy_wait(200000);
                }
        }

//     if (ret) {
//         // printk("STATUS read failed: %d\n", ret);
//         return;
//     }

//     printk("STATUS = 0x%02X, READY = %d\n", status, status & STATUS_READY_MASK);
}