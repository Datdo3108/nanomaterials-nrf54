#include "ad5761.h"
#include "adc.h"
#include "ble.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

// #define SPI_DEBUG
// #define BLE_DEBUG
#define POLL_DEBUG

#ifdef BLE_DEBUG

int main(void){
        ble_init();

        while(1){
                if(default_conn){
			send_notify_data();
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

#ifdef POLL_DEBUG

// #define DEBUG_DAC_1_THEN_2              // Run transfer curve
// #define DEBUG_DAC_2_THEN_1              // Run output curve
#define DEBUG_INJECT_CURRENT            // Run constant current 

#define LED_NODE_2	DT_NODELABEL(led2)
#define BUTTON_NODE_3   DT_NODELABEL(button3)
static const struct gpio_dt_spec led_spec_2 = GPIO_DT_SPEC_GET(LED_NODE_2, gpios);
static const struct gpio_dt_spec button_spec_3 = GPIO_DT_SPEC_GET(BUTTON_NODE_3, gpios);

static struct gpio_callback button_cb;

struct k_timer led_timer;
struct k_timer dac_timer;
struct k_timer adc_0_timer;
struct k_poll_signal led_sig;
struct k_poll_signal dac_sig;
struct k_poll_signal adc_0_sig;
static struct k_poll_signal button_sig;


#define ADC_INTERVAL_US         50000
#define NYQUIST_FACTOR          1
#define DAC_INTERVAL_US         ADC_INTERVAL_US*NYQUIST_FACTOR

void adc_0_timer_cb(struct k_timer *timer)
{
    k_poll_signal_raise(&adc_0_sig, 0);
}

void dac_timer_cb(struct k_timer *timer)
{
    k_poll_signal_raise(&dac_sig, 0);
}

void led_timer_cb(struct k_timer *timer)
{
        k_poll_signal_raise(&led_sig, 0);
}

void button_isr(const struct device *dev,
                struct gpio_callback *cb,
                uint32_t pins)
{
    k_poll_signal_raise(&button_sig, 0);
}

static int button_init(void)
{
    int ret;

    /* Configure button pin as input */
    ret = gpio_pin_configure_dt(&button_spec_3, GPIO_INPUT);
    if (ret) {
        return ret;
    }

    /* Configure interrupt on button press */
    ret = gpio_pin_interrupt_configure_dt(&button_spec_3,
                                          GPIO_INT_EDGE_TO_ACTIVE);
    if (ret) {
        return ret;
    }

    /* Initialize and register callback */
    gpio_init_callback(&button_cb,
                       button_isr,
                       BIT(button_spec_3.pin));

    gpio_add_callback(button_spec_3.port, &button_cb);

    return 0;
}

typedef enum {
    LED_STATE_ON,
    LED_STATE_OFF,
} led_state_t;

static led_state_t led_state = LED_STATE_ON;

/* Per-state counters */
static uint8_t toggle_count = 0;

void led_state_machine_step(void)
{
    gpio_pin_toggle_dt(&led_spec_2);
    toggle_count++;

    switch (led_state) {

    case LED_STATE_ON:
        if (toggle_count >= 3) {
            /* Transition to OFF */
            toggle_count = 0;
            led_state = LED_STATE_OFF;

            /* Change timer period */
            k_timer_start(&led_timer,
                          K_MSEC(100),
                          K_MSEC(100));
        }
        break;

    case LED_STATE_OFF:
        if (toggle_count >= 5) {
            /* Transition to ON */
            toggle_count = 0;
            led_state = LED_STATE_ON;

            k_timer_start(&led_timer,
                          K_SECONDS(1),
                          K_SECONDS(1));
        }
        break;
    }
}

#ifdef DEBUG_DAC_1_THEN_2

// #define DEBUG_VG
#define DEBUG_DEMO_TF

#ifdef DEBUG_VG
static const struct dac_fsm dac_fsm_default={
        .state = DAC_STATE_ON,
        .direction = DAC_DIR_UP,
        .value = 0,
        .start = 0,
        .stop = 35000,
        .step = 500,
        .continuous = false,
        .cycle_counter = 0,
        .cycles = 6,
};

static const struct dac_output_fsm dac_2_output_fsm_default={
        .state = DAC_STATE_ON,
        .value = 0,
        .start = 0,
        .stop = 26000,
        .step = 5200,
        .cycle_counter = 0,
        .cycles = 6,
};
#endif

#ifdef DEBUG_DEMO_TF
static const struct dac_fsm dac_fsm_default={
        .state = DAC_STATE_ON,
        .direction = DAC_DIR_UP,
        .value = 0,
        .start = 0,
        .stop = 35000,
        .step = 250,
        .continuous = false,
        .backward = true,
        .cycle_counter = 0,
        .cycles = 1,
};

static const struct dac_output_fsm dac_2_output_fsm_default={
        .state = DAC_STATE_ON,
        .value = 0,
        .start = 2750,
        .stop = 26000,
        .step = 5200,
        .cycle_counter = 0,
        .cycles = 1,
};
#endif
#endif

#ifdef DEBUG_DAC_2_THEN_1
/* Device 2, Sweep VD from 0 to 1V */
static const struct dac_fsm dac_fsm_default={
        .state = DAC_STATE_ON,
        .direction = DAC_DIR_UP,
        .value = 0,
        .start = 0,
        .stop = 9500,
        .step = 50,
        .continuous = false,
        .cycle_counter = 0,
        .cycles = 6,
};

/* Device 1, Change VG from 0 to 1V, step 200mV */
static const struct dac_output_fsm dac_2_output_fsm_default={
        .state = DAC_STATE_ON,
        .value = 0,
        .start = 0,
        .stop = 26500,
        .step = 5300,
        .cycle_counter = 0,
        .cycles = 6,
};
#endif

#ifdef DEBUG_INJECT_CURRENT
/* 
        Device 2, Sweep voltage driver from 0 to 100mV (current driver 0 to 10uA) 
        Use resistor 10K

        Device 2, Sweep voltage driver from 0 to 200mV (current driver 0 to 1uA), duration 1000 cycles 
        Use resistor 200K
*/
static const struct dac_fsm dac_fsm_default={
        .state = DAC_STATE_ON,
        .direction = DAC_DIR_UP,
        .value = 0,
        .start = 0,
        .stop = 2000,
        .step = 2,
        .continuous = false,
        .backward = true,
        .cycle_counter = 0,
        .cycles = 1,
};

/* 
        Device 1, Bias VG at ON state (~800mV)
*/
static const struct dac_output_fsm dac_2_output_fsm_default={
        .state = DAC_STATE_ON,
        .value = 21000,
        .start = 21000,
        .stop = 21000,
        .step = 0,
        .cycle_counter = 0,
        .cycles = 1,
};
#endif

struct dac_fsm dac_fsm_runtime;
struct dac_output_fsm dac_2_output_fsm_runtime;

void dac_start_triangle(struct dac_fsm *fsm)
{
        *fsm = dac_fsm_default;

        /* Enable periodic stepping */
        k_timer_start(&dac_timer,
                        K_NO_WAIT,
                        K_USEC(DAC_INTERVAL_US));
}


void dac_start_oect_output(struct dac_fsm *fsm_1, struct dac_output_fsm *fsm_2)
{
        *fsm_1 = dac_fsm_default;
        *fsm_2 = dac_2_output_fsm_default;
        // *fsm_2 = dac_2_fsm_default;

        k_timer_start(&dac_timer,
                        K_NO_WAIT,
                        K_USEC(DAC_INTERVAL_US));
}

void dac_stop(struct dac_fsm *fsm)
{
    fsm->state = DAC_STATE_OFF;

    /* Stop stepping */
    k_timer_stop(&dac_timer);
}



int main(void){
	gpio_pin_configure_dt(&led_spec_2, GPIO_OUTPUT);


        gpio_pin_set_dt(&led_spec_2, 1);

        ad5761_init(&ad5761_dev_i);
        ad5761_init(&ad5761_dev_ii);
        adc_init();
        button_init();

        ad5761_24bit_write(&ad5761_dev_i, 0x0F, 0x00, 0x00);   // Software full reset
        ad5761_24bit_write(&ad5761_dev_i, 0x04, 0x00, 0x43);   // Write to control register
        ad5761_readback_control_register(&ad5761_dev_i);

        ad5761_24bit_write(&ad5761_dev_ii, 0x0F, 0x00, 0x00);   // Software full reset
        ad5761_24bit_write(&ad5761_dev_ii, 0x04, 0x00, 0x43);   // Write to control register
        ad5761_readback_control_register(&ad5761_dev_ii);

        #ifdef DEBUG_INJECT_CURRENT
        ad5761_generate_output_signal(&ad5761_dev_i, dac_2_output_fsm_default.value);
        #endif

        k_poll_signal_init(&led_sig);
        k_poll_signal_init(&dac_sig);
        k_poll_signal_init(&adc_0_sig);
        k_poll_signal_init(&button_sig);

        k_timer_init(&led_timer, led_timer_cb, NULL);
        k_timer_init(&dac_timer, dac_timer_cb, NULL);
        k_timer_init(&adc_0_timer, adc_0_timer_cb, NULL);

        // k_timer_start(&led_timer, K_SECONDS(1), K_SECONDS(1));
        k_timer_start(&led_timer, K_NO_WAIT, K_SECONDS(1));
        k_timer_start(&dac_timer, K_NO_WAIT, K_USEC(DAC_INTERVAL_US));
        k_timer_start(&adc_0_timer, K_NO_WAIT, K_USEC(ADC_INTERVAL_US));

        struct k_poll_event events[] = {
                K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
                                        K_POLL_MODE_NOTIFY_ONLY,
                                        &led_sig),
                K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
                                        K_POLL_MODE_NOTIFY_ONLY,
                                        &dac_sig),
                K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
                                        K_POLL_MODE_NOTIFY_ONLY,
                                        &adc_0_sig),
                K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
                                        K_POLL_MODE_NOTIFY_ONLY,
                                        &button_sig),
        };

        while (1) {
                k_poll(events, ARRAY_SIZE(events), K_FOREVER);

                if (events[0].signal->signaled) {
                        events[0].signal->signaled = 0;
                        // led_state_machine_step();
                }

                if (events[1].signal->signaled) {
                        events[1].signal->signaled = 0;
                        // ad5761_fsm_step_triangle(&ad5761_dev_i, &dac_fsm_runtime);
                        #ifdef DEBUG_DAC_1_THEN_2
                        ad5761_fsm_oect_output_curve(&ad5761_dev_i, &dac_fsm_runtime, &ad5761_dev_ii, &dac_2_output_fsm_runtime);
                        // ad5761_print_dac_value(&ad5761_dev_i, dac_fsm_runtime.value);

                        #endif
                        #ifdef DEBUG_DAC_2_THEN_1
                        ad5761_fsm_oect_output_curve(&ad5761_dev_ii, &dac_fsm_runtime, &ad5761_dev_i, &dac_2_output_fsm_runtime);
                        #endif

                        #ifdef DEBUG_INJECT_CURRENT
                        ad5761_fsm_step_triangle(&ad5761_dev_ii, &dac_fsm_runtime);
                        // ad5761_generate_output_signal(&ad5761_dev_ii, 465);      // Set V to 50mV -> Ids = 5uA, R = 10K
                        // ad5761_generate_output_signal(&ad5761_dev_ii, 9300);      // Set V to 1000mV -> Ids = 5uA, R = 200K
                        ad5761_generate_output_signal(&ad5761_dev_ii, 1800);      // Set V to 200mV -> Ids = 1uA, R = 200K
                        // ad5761_generate_output_signal(&ad5761_dev_i, dac_2_output_fsm_runtime.value);

                        // ad5761_fsm_oect_output_curve(&ad5761_dev_ii, &dac_fsm_runtime, &ad5761_dev_i, &dac_2_output_fsm_runtime);
                        #endif
                }

                if (events[2].signal->signaled) {
                        events[2].signal->signaled = 0;
                        int64_t time_now = k_uptime_get();
                        printk("Time stamp: %" PRId64 "\n", time_now);
                        adc_read_channel_mV(0);         /* Read Transimpedance IDS */
                        adc_read_channel_mV(1);         /* Read VDS */
                        adc_read_channel_mV(2);         /* Read VG */
                }

                if (events[3].signal->signaled) {
                        events[3].signal->signaled = 0;
                        
                        int64_t now = k_uptime_get();
                        int64_t last_press = now;
                        while(now - last_press <= 200){
                                now = k_uptime_get();
                                // printk("Now: %" PRId64 "\n", now);
                                // printk("Last press: %" PRId64 "\n", last_press);
                        }

                        printk("Start measuring\n");

                        // static int64_t last_press;
                        // int64_t now = k_uptime_get();

                        // if (now - last_press <= 1000) {
                        //         return 0;   /* ignore bounce */
                        // }

                        // last_press = now;

                        /* DAC channel 1 triangle waveform */
                        if (dac_fsm_runtime.state == DAC_STATE_OFF) {
                                // dac_start_triangle(&dac_fsm_runtime);
                                dac_start_oect_output(&dac_fsm_runtime, &dac_2_output_fsm_runtime);
                        } else {
                                dac_stop(&dac_fsm_runtime);
                        }
                }
        }

        return 0;
}

#endif

#ifdef SPI_DEBUG

#define LED_NODE	DT_NODELABEL(led3)

int main(void){
        ad5761_init(&ad5761_dev_i);
        ad5761_init(&ad5761_dev_ii);
        adc_init();
        ble_init();

        // static const struct gpio_dt_spec led_spec = GPIO_DT_SPEC_GET(LED_NODE, gpios);
	// gpio_pin_configure_dt(&led_spec, GPIO_OUTPUT);

        printk("\nDevice 2:\n");
        ad5761_24bit_write(&ad5761_dev_ii, 0x0F, 0x00, 0x00);   // Software full reset
        ad5761_24bit_write(&ad5761_dev_ii, 0x04, 0x00, 0x43);   // Write to control register
        ad5761_readback_control_register(&ad5761_dev_ii);

        /*
                Device 2 functions:
                        - Set VD
                        - Sweep ID through constant current + current mirror circuit
        */
        ad5761_generate_output_signal(&ad5761_dev_ii, 2700);    
        // // Set device 2: VD = 0.35V -> 3072
        // // Set device 2: VD = 0.3V -> 2700
        // ad5761_readback_DAC_register(&ad5761_dev_ii);

        ad5761_24bit_write(&ad5761_dev_i, 0x0F, 0x00, 0x00);   // Software full reset
        ad5761_24bit_write(&ad5761_dev_i, 0x04, 0x00, 0x43);   // Write to control register

        ad5761_readback_control_register(&ad5761_dev_i);
        
        // ad5761_generate_output_signal(&ad5761_dev_i, 17280);     // Set device 1: VG = 0.6V
        // ad5761_readback_DAC_register(&ad5761_dev_i);

        printk("Control register completed!********\n");

        while(1){
                if(default_conn){
			send_notify_data();
		}

                // ad5761_generate_triangular(&ad5761_dev_ii, 0, 2000, 50000, 100, true);
                /* 
                        1.2V: 30000
                        1.3V: 32500
                        1.4V: 35000
                */

                ad5761_generate_triangular(&ad5761_dev_i, 0, 35000, 2000, 400, true);
                /* 
                        1.1V: 30000
                        1.2V: 32500
                        1.3V: 35000
                */

                /* ADC reading */
                // adc_read_channel_mV(0);
                // adc_read_channel_mV(1);
                // adc_read_channel_mV(2);

                //// LED blink ////

                // gpio_pin_set_dt(&led_spec, 1);
                // printk("LED ON\n");
                // // k_msleep(1000);
                // k_busy_wait(1000000);
                // gpio_pin_set_dt(&led_spec, 0);
                // printk("LED OFF\n");
                // // k_msleep(1000);
                // k_busy_wait(1000000);
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