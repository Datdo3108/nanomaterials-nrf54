/*
 * ADS1220 24-bit, 4-channel ADC driver for Zephyr (plain C).
 *
 * Ported from the Arduino library ADS1220_WE by Wolfgang (Wolle) Ewald.
 * Register logic and enums are kept; SPI/GPIO/timing use Zephyr APIs.
 *
 * All functions return 0 on success or a negative errno, unless noted.
 */
#ifndef ADS1220_H
#define ADS1220_H

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

/* ---- SPI commands ---- */
#define ADS1220_CMD_RESET    0x06
#define ADS1220_CMD_START    0x08  /* START/SYNC: starts (or restarts) a conversion */
#define ADS1220_CMD_PWRDOWN  0x02
#define ADS1220_CMD_RDATA    0x10
#define ADS1220_CMD_WREG     0x40
#define ADS1220_CMD_RREG     0x20

/* ---- Configuration registers ---- */
#define ADS1220_REG_0        0x00
#define ADS1220_REG_1        0x01
#define ADS1220_REG_2        0x02
#define ADS1220_REG_3        0x03

#define ADS1220_RANGE        8388607.0f  /* 2^23 - 1 */

/*
 * SPI mode 1 (CPOL=0, CPHA=1): the ADS1220 shifts data out on the falling
 * SCLK edge and samples on the rising edge. In Zephyr that is SPI_MODE_CPHA
 * without SPI_MODE_CPOL. Clock frequency comes from spi-max-frequency in DT.
 */
#define ADS1220_SPI_OPERATION (SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA)

/* ---- Enums (values are the register bit patterns) ---- */
enum ads1220_mux {
	ADS1220_MUX_0_1 = 0x00,  /* default */
	ADS1220_MUX_0_2 = 0x10,
	ADS1220_MUX_0_3 = 0x20,
	ADS1220_MUX_1_2 = 0x30,
	ADS1220_MUX_1_3 = 0x40,
	ADS1220_MUX_2_3 = 0x50,
	ADS1220_MUX_1_0 = 0x60,
	ADS1220_MUX_3_2 = 0x70,
	ADS1220_MUX_0_AVSS = 0x80,
	ADS1220_MUX_1_AVSS = 0x90,
	ADS1220_MUX_2_AVSS = 0xA0,
	ADS1220_MUX_3_AVSS = 0xB0,
	ADS1220_MUX_REFPX_REFNX_4 = 0xC0,
	ADS1220_MUX_AVDD_M_AVSS_4 = 0xD0,
	ADS1220_MUX_AVDD_P_AVSS_2 = 0xE0,
};

enum ads1220_gain {
	ADS1220_GAIN_1 = 0x00,  /* default */
	ADS1220_GAIN_2 = 0x02,
	ADS1220_GAIN_4 = 0x04,
	ADS1220_GAIN_8 = 0x06,
	ADS1220_GAIN_16 = 0x08,
	ADS1220_GAIN_32 = 0x0A,
	ADS1220_GAIN_64 = 0x0C,
	ADS1220_GAIN_128 = 0x0E,
};

enum ads1220_data_rate {
	ADS1220_DR_LVL_0 = 0x00,  /* default */
	ADS1220_DR_LVL_1 = 0x20,
	ADS1220_DR_LVL_2 = 0x40,
	ADS1220_DR_LVL_3 = 0x60,
	ADS1220_DR_LVL_4 = 0x80,
	ADS1220_DR_LVL_5 = 0xA0,
	ADS1220_DR_LVL_6 = 0xC0,
};

enum ads1220_op_mode {
	ADS1220_NORMAL_MODE = 0x00,  /* default */
	ADS1220_DUTY_CYCLE_MODE = 0x08,
	ADS1220_TURBO_MODE = 0x10,
};

enum ads1220_conv_mode {
	ADS1220_SINGLE_SHOT = 0x00,  /* default */
	ADS1220_CONTINUOUS = 0x04,
};

enum ads1220_vref {
	ADS1220_VREF_INT = 0x00,  /* default, internal 2.048 V */
	ADS1220_VREF_REFP0_REFN0 = 0x40,
	ADS1220_VREF_REFP1_REFN1 = 0x80,
	ADS1220_VREF_AVDD_AVSS = 0xC0,
};

enum ads1220_fir {
	ADS1220_NONE = 0x00,  /* default */
	ADS1220_50HZ_60HZ = 0x10,
	ADS1220_50HZ = 0x20,
	ADS1220_60HZ = 0x30,
};

enum ads1220_psw {
	ADS1220_ALWAYS_OPEN = 0x00,  /* default */
	ADS1220_SWITCH = 0x08,
};

enum ads1220_idac_current {
	ADS1220_IDAC_OFF = 0x00,  /* default */
	ADS1220_IDAC_10_MU_A = 0x01,
	ADS1220_IDAC_50_MU_A = 0x02,
	ADS1220_IDAC_100_MU_A = 0x03,
	ADS1220_IDAC_250_MU_A = 0x04,
	ADS1220_IDAC_500_MU_A = 0x05,
	ADS1220_IDAC_1000_MU_A = 0x06,
	ADS1220_IDAC_1500_MU_A = 0x07,
};

enum ads1220_idac_routing {
	ADS1220_IDAC_NONE = 0x00,  /* default */
	ADS1220_IDAC_AIN0_REFP1 = 0x01,
	ADS1220_IDAC_AIN1 = 0x02,
	ADS1220_IDAC_AIN2 = 0x03,
	ADS1220_IDAC_AIN3_REFN1 = 0x04,
	ADS1220_IDAC_REFP0 = 0x05,
	ADS1220_IDAC_REFN0 = 0x06,
};

enum ads1220_drdy_mode {
	ADS1220_DRDY = 0x00,  /* default: DRDY pin only */
	ADS1220_DOUT_DRDY = 0x02,  /* DRDY also on DOUT/DRDY */
};

/* ---- Device instance ---- */
struct ads1220 {
	struct spi_dt_spec spi;
	struct gpio_dt_spec drdy;       /* .port == NULL if DRDY is not wired */
	struct gpio_callback drdy_cb;
	struct k_sem drdy_sem;
	uint32_t drdy_timeout_ms;       /* blocking-read timeout */

	float vref;                     /* volts, used to scale results */
	uint8_t gain;                   /* effective gain factor */
	bool ref_measurement;           /* mux is in a reference-monitor mode */
	bool no_bypass_pga;             /* keep PGA enabled whenever allowed */
	bool non_blocking;              /* skip START/DRDY wait in reads */
	enum ads1220_conv_mode conv_mode;
};

/*
 * Static initializer from a devicetree node, e.g.
 *   static struct ads1220 adc = ADS1220_DT_SPEC_GET(DT_NODELABEL(ads1220));
 *
 * Note: on Zephyr versions before 4.x, SPI_DT_SPEC_GET() takes a third
 * "delay" argument; add ", 0" if the compiler complains.
 */
#define ADS1220_DT_SPEC_GET(node_id)                                              \
	{                                                                         \
		.spi = SPI_DT_SPEC_GET(node_id, ADS1220_SPI_OPERATION, 0),          \
		.drdy = GPIO_DT_SPEC_GET_OR(node_id, drdy_gpios, {0}),            \
		.drdy_timeout_ms = 1000,                                          \
	}

/* ---- Commands ---- */
int ads1220_init(struct ads1220 *dev);  /* -ENODEV if the chip does not answer */
int ads1220_start(struct ads1220 *dev);
int ads1220_reset(struct ads1220 *dev);
int ads1220_power_down(struct ads1220 *dev);

/* ---- Configuration register 0 ---- */
int ads1220_set_compare_channels(struct ads1220 *dev, enum ads1220_mux mux);
int ads1220_set_gain(struct ads1220 *dev, enum ads1220_gain gain);
uint8_t ads1220_get_gain_factor(const struct ads1220 *dev);
int ads1220_bypass_pga(struct ads1220 *dev, bool bypass);
int ads1220_is_pga_bypassed(struct ads1220 *dev, bool *bypassed);

/* ---- Configuration register 1 ---- */
int ads1220_set_data_rate(struct ads1220 *dev, enum ads1220_data_rate rate);
int ads1220_set_operating_mode(struct ads1220 *dev, enum ads1220_op_mode mode);
int ads1220_set_conversion_mode(struct ads1220 *dev, enum ads1220_conv_mode mode);
int ads1220_enable_temperature_sensor(struct ads1220 *dev, bool enable);
int ads1220_enable_burn_out_current_sources(struct ads1220 *dev, bool enable);

/* ---- Configuration register 2 ---- */
int ads1220_set_vref_source(struct ads1220 *dev, enum ads1220_vref src);
int ads1220_set_fir_filter(struct ads1220 *dev, enum ads1220_fir fir);
int ads1220_set_low_side_power_switch(struct ads1220 *dev, enum ads1220_psw psw);
int ads1220_set_idac_current(struct ads1220 *dev, enum ads1220_idac_current current);

/* ---- Configuration register 3 ---- */
int ads1220_set_idac1_routing(struct ads1220 *dev, enum ads1220_idac_routing route);
int ads1220_set_idac2_routing(struct ads1220 *dev, enum ads1220_idac_routing route);
int ads1220_set_drdy_mode(struct ads1220 *dev, enum ads1220_drdy_mode mode);

/* ---- Reference handling (no error return) ---- */
void ads1220_set_vref_value_v(struct ads1220 *dev, float vref);
float ads1220_get_vref_v(const struct ads1220 *dev);
void ads1220_set_non_blocking_mode(struct ads1220 *dev, bool non_blocking);
bool ads1220_get_non_blocking_mode(const struct ads1220 *dev);

/*
 * The calibrate functions measure the reference through the ADC's reference
 * monitor and store the result as vref. They leave the vref source and mux
 * on the reference-monitor settings, so call set_compare_channels() afterwards.
 */
int ads1220_set_avdd_avss_as_vref_and_calibrate(struct ads1220 *dev);
int ads1220_set_refp0_refn0_as_vref_and_calibrate(struct ads1220 *dev);
int ads1220_set_refp1_refn1_as_vref_and_calibrate(struct ads1220 *dev);
int ads1220_set_int_vref(struct ads1220 *dev);

/* ---- Results ---- */
int ads1220_get_raw_data(struct ads1220 *dev, int32_t *raw);
int ads1220_get_voltage_mv(struct ads1220 *dev, float *mv);
int ads1220_get_voltage_uv(struct ads1220 *dev, float *uv);
int ads1220_get_temperature(struct ads1220 *dev, float *celsius);

int read_reg(const struct ads1220 *d, uint8_t reg, uint8_t *val);
#endif /* ADS1220_H */
