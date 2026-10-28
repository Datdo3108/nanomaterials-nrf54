/*
 * ADS1220 driver for Zephyr (plain C). See ads1220.h.
 * Ported from ADS1220_WE by Wolfgang (Wolle) Ewald.
 */
#include "ads1220.h"

#include <errno.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ads1220, LOG_LEVEL_INF);

#define PRINTF_EN

#define ADS_TRY(expr)                 \
	do {                          \
		int _r = (expr);      \
		if (_r != 0) {        \
			return _r;    \
		}                     \
	} while (0)

/* ------------------------------------------------------------------ */
/* Low-level SPI helpers                                              */
/* ------------------------------------------------------------------ */

/* One CS-framed transaction. rx may be NULL (write only). */
static int xfer(const struct ads1220 *d, const uint8_t *tx, uint8_t *rx, size_t len)
{
	const struct spi_buf tx_buf = { .buf = (void *)tx, .len = len };
	const struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };

	if (rx == NULL) {
		return spi_write_dt(&d->spi, &tx_set);
	}

	const struct spi_buf rx_buf = { .buf = rx, .len = len };
	const struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };

	return spi_transceive_dt(&d->spi, &tx_set, &rx_set);
}

static int command(const struct ads1220 *d, uint8_t cmd)
{
	return xfer(d, &cmd, NULL, 1);
}

/* RREG: 0010 rrnn, nn = (number of registers - 1) = 0 -> one register */
int read_reg(const struct ads1220 *d, uint8_t reg, uint8_t *val)
{
	uint8_t tx[2] = { ADS1220_CMD_RREG | (reg << 2), 0x00 };
	uint8_t rx[2];
	int ret = xfer(d, tx, rx, sizeof(tx));

	if (ret == 0) {
		*val = rx[1];  /* rx[0] is clocked in while the command goes out */
	}
	return ret;
}

/* WREG: 0100 rrnn */
static int write_reg(const struct ads1220 *d, uint8_t reg, uint8_t val)
{
	uint8_t tx[2] = { ADS1220_CMD_WREG | (reg << 2), val };

	return xfer(d, tx, NULL, sizeof(tx));
}

/* Read-modify-write: replace only the bits in mask. */
static int update_reg(const struct ads1220 *d, uint8_t reg, uint8_t mask, uint8_t bits)
{
	uint8_t v;

	ADS_TRY(read_reg(d, reg, &v));
	v = (uint8_t)((v & ~mask) | (bits & mask));
	return write_reg(d, reg, v);
}

/* ------------------------------------------------------------------ */
/* DRDY handling                                                      */
/* ------------------------------------------------------------------ */

static void drdy_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	struct ads1220 *d = CONTAINER_OF(cb, struct ads1220, drdy_cb);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);
	k_sem_give(&d->drdy_sem);
}

/*
 * DRDY is active-low (goes low when a conversion result is ready, returns
 * high after the read), so the devicetree entry must carry GPIO_ACTIVE_LOW.
 * If it is already asserted we return at once (same as the Arduino level
 * poll); otherwise sleep on the semaphore given by the falling-edge ISR.
 */
static int wait_drdy(struct ads1220 *d)
{
	if (d->drdy.port == NULL) {
		return -ENOTSUP;
	}
	if (gpio_pin_get_dt(&d->drdy) > 0) {
		return 0;
	}

	int ret = k_sem_take(&d->drdy_sem, K_MSEC(d->drdy_timeout_ms));

	return (ret == -EAGAIN) ? -ETIMEDOUT : ret;
}

/* ------------------------------------------------------------------ */
/* Internal helpers                                                   */
/* ------------------------------------------------------------------ */

/*
 * Single-ended inputs (AINx vs AVSS) and the supply monitors cannot use the
 * PGA: its inputs would leave their common-mode range. The datasheet forces
 * the PGA bypass for these muxes, and gain is then limited to 1, 2 or 4
 * (done by the modulator, not the PGA).
 */
static bool pga_must_be_bypassed(uint8_t mux)
{
	return (mux >= 0x80) && (mux <= 0xD0);
}

static int force_bypass_pga(const struct ads1220 *d)
{
	return update_reg(d, ADS1220_REG_0, 0x01, 0x01);
}

/* Reads the 24-bit conversion result (right-aligned in a uint32_t). */
static int read_result(struct ads1220 *d, uint32_t *raw24)
{
	uint8_t tx[3] = { 0 };
	uint8_t rx[3];

	if (d->conv_mode == ADS1220_SINGLE_SHOT && !d->non_blocking) {
		k_sem_reset(&d->drdy_sem);  /* drop a stale edge before converting */
		ADS_TRY(ads1220_start(d));
	}
	if (!d->non_blocking) {
		ADS_TRY(wait_drdy(d));
	}

	/* With DRDY low, the three bytes can be clocked out directly (no RDATA). */
	ADS_TRY(xfer(d, tx, rx, sizeof(tx)));
	k_sem_reset(&d->drdy_sem);

	*raw24 = ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | rx[2];
	return 0;
}

/*
 * Reference monitor: the chip measures (REF)/4 against its internal 2.048 V,
 * so the average of 10 readings (in mV) times 4, converted to V, is the
 * reference voltage: sum / 10 * 4 / 1000 = sum * 4 / 10000.
 */
static int calibrate_vref(struct ads1220 *d, enum ads1220_vref src, enum ads1220_mux mux)
{
	bool prev_non_blocking = d->non_blocking;
	float sum = 0.0f;
	float mv;
	int ret;

	d->non_blocking = false;

	ret = ads1220_set_vref_source(d, src);
	if (ret == 0) {
		ret = ads1220_set_compare_channels(d, mux);
	}
	for (int i = 0; (ret == 0) && (i < 10); i++) {
		ret = ads1220_get_voltage_mv(d, &mv);
		sum += mv;
	}
	if (ret == 0) {
		d->vref = sum * 4.0f / 10000.0f;
	}

	d->non_blocking = prev_non_blocking;
	return ret;
}

/* ------------------------------------------------------------------ */
/* Commands                                                           */
/* ------------------------------------------------------------------ */

int ads1220_init(struct ads1220 *d)
{
	uint8_t ctrl;

	d->vref = 2.048f;
	d->gain = 1;
	d->ref_measurement = false;
	d->conv_mode = ADS1220_SINGLE_SHOT;
	d->no_bypass_pga = true;
	d->non_blocking = false;

	if (!spi_is_ready_dt(&d->spi)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}
    // printf("SPI ready\n");

	k_sem_init(&d->drdy_sem, 0, 1);
    // printf("k_sem_init\n");

	if (d->drdy.port != NULL) {
		if (!gpio_is_ready_dt(&d->drdy)) {
			LOG_ERR("DRDY GPIO not ready");
			return -ENODEV;
		}
        // printf("DRDY ready\n");
		ADS_TRY(gpio_pin_configure_dt(&d->drdy, GPIO_INPUT));
		gpio_init_callback(&d->drdy_cb, drdy_isr, BIT(d->drdy.pin));
        // printf("1\n");
		ADS_TRY(gpio_add_callback(d->drdy.port, &d->drdy_cb));
        // printf("2\n");
		ADS_TRY(gpio_pin_interrupt_configure_dt(&d->drdy, GPIO_INT_EDGE_TO_ACTIVE));
        // printf("3\n");
	}

	ADS_TRY(ads1220_reset(d));
    // printf("ADS1220 reset cmd\n");
	ADS_TRY(ads1220_start(d));
    // printf("ADS1220 start cmd\n");

	/*
	 * Presence test: after RESET register 0 is 0x00; setting the PGA bypass
	 * bit must read back as 0x01. A missing chip reads 0x00 or 0xFF.
	 */
	ADS_TRY(ads1220_bypass_pga(d, true));
	ADS_TRY(read_reg(d, ADS1220_REG_0, &ctrl));
    // printf("Read register 0\n");
	ADS_TRY(ads1220_bypass_pga(d, false));

	if (ctrl != 0x01) {
		LOG_ERR("ADS1220 not detected (reg0 = 0x%02x)", ctrl);
		return -ENODEV;
	}
	return 0;
}

int ads1220_start(struct ads1220 *d)
{
	return command(d, ADS1220_CMD_START);
}

int ads1220_reset(struct ads1220 *d)
{
	ADS_TRY(command(d, ADS1220_CMD_RESET));
	k_msleep(1);  /* datasheet: ~50 us + 32 clocks after RESET */

	/* RESET puts the registers back to defaults; keep the cached state in sync. */
	d->gain = 1;
	d->ref_measurement = false;
	d->conv_mode = ADS1220_SINGLE_SHOT;
	return 0;
}

int ads1220_power_down(struct ads1220 *d)
{
	return command(d, ADS1220_CMD_PWRDOWN);
}

/* ------------------------------------------------------------------ */
/* Configuration register 0                                           */
/* ------------------------------------------------------------------ */

int ads1220_set_compare_channels(struct ads1220 *d, enum ads1220_mux mux)
{
	uint8_t reg;

	if ((mux == ADS1220_MUX_REFPX_REFNX_4) || (mux == ADS1220_MUX_AVDD_M_AVSS_4)) {
		d->gain = 1;  /* gain is 1 by definition in the monitor modes */
		d->ref_measurement = true;
	} else {
		ADS_TRY(read_reg(d, ADS1220_REG_0, &reg));
		d->gain = 1U << ((reg & 0x0E) >> 1);
		d->ref_measurement = false;
	}

	/* Mux bits [7:4] and PGA-bypass bit [0]; bypass follows bypass_pga()'s choice. */
	ADS_TRY(update_reg(d, ADS1220_REG_0, 0xF1,
			   (uint8_t)mux | (d->no_bypass_pga ? 0x00 : 0x01)));

	if (pga_must_be_bypassed((uint8_t)mux)) {
		if (d->gain > 4) {
			d->gain = 4;
		}
		ADS_TRY(force_bypass_pga(d));
	}
	return 0;
}

int ads1220_set_gain(struct ads1220 *d, enum ads1220_gain gain)
{
	uint8_t reg;

	ADS_TRY(read_reg(d, ADS1220_REG_0, &reg));
	uint8_t mux = reg & 0xF0;

	reg = (uint8_t)((reg & ~0x0E) | gain);
	ADS_TRY(write_reg(d, ADS1220_REG_0, reg));

	d->gain = 1U << (gain >> 1);

	if (pga_must_be_bypassed(mux)) {
		if (d->gain > 4) {
			d->gain = 4;
		}
		ADS_TRY(force_bypass_pga(d));
	}
	return 0;
}

uint8_t ads1220_get_gain_factor(const struct ads1220 *d)
{
	return d->gain;
}

int ads1220_bypass_pga(struct ads1220 *d, bool bypass)
{
	d->no_bypass_pga = !bypass;
	return update_reg(d, ADS1220_REG_0, 0x01, bypass ? 0x01 : 0x00);
}

int ads1220_is_pga_bypassed(struct ads1220 *d, bool *bypassed)
{
	uint8_t reg;

	ADS_TRY(read_reg(d, ADS1220_REG_0, &reg));
	*bypassed = (reg & 0x01) != 0;
	return 0;
}

/* ------------------------------------------------------------------ */
/* Configuration register 1                                           */
/* ------------------------------------------------------------------ */

int ads1220_set_data_rate(struct ads1220 *d, enum ads1220_data_rate rate)
{
	return update_reg(d, ADS1220_REG_1, 0xE0, (uint8_t)rate);
}

int ads1220_set_operating_mode(struct ads1220 *d, enum ads1220_op_mode mode)
{
	return update_reg(d, ADS1220_REG_1, 0x18, (uint8_t)mode);
}

int ads1220_set_conversion_mode(struct ads1220 *d, enum ads1220_conv_mode mode)
{
	d->conv_mode = mode;
	return update_reg(d, ADS1220_REG_1, 0x04, (uint8_t)mode);
}

int ads1220_enable_temperature_sensor(struct ads1220 *d, bool enable)
{
	return update_reg(d, ADS1220_REG_1, 0x02, enable ? 0x02 : 0x00);
}

int ads1220_enable_burn_out_current_sources(struct ads1220 *d, bool enable)
{
	return update_reg(d, ADS1220_REG_1, 0x01, enable ? 0x01 : 0x00);
}

/* ------------------------------------------------------------------ */
/* Configuration register 2                                           */
/* ------------------------------------------------------------------ */

int ads1220_set_vref_source(struct ads1220 *d, enum ads1220_vref src)
{
	return update_reg(d, ADS1220_REG_2, 0xC0, (uint8_t)src);
}

int ads1220_set_fir_filter(struct ads1220 *d, enum ads1220_fir fir)
{
	return update_reg(d, ADS1220_REG_2, 0x30, (uint8_t)fir);
}

int ads1220_set_low_side_power_switch(struct ads1220 *d, enum ads1220_psw psw)
{
	return update_reg(d, ADS1220_REG_2, 0x08, (uint8_t)psw);
}

int ads1220_set_idac_current(struct ads1220 *d, enum ads1220_idac_current current)
{
	ADS_TRY(update_reg(d, ADS1220_REG_2, 0x07, (uint8_t)current));
	k_busy_wait(200);  /* let the current source settle */
	return 0;
}

/* ------------------------------------------------------------------ */
/* Configuration register 3                                           */
/* ------------------------------------------------------------------ */

int ads1220_set_idac1_routing(struct ads1220 *d, enum ads1220_idac_routing route)
{
	return update_reg(d, ADS1220_REG_3, 0xE0, (uint8_t)(route << 5));
}

int ads1220_set_idac2_routing(struct ads1220 *d, enum ads1220_idac_routing route)
{
	return update_reg(d, ADS1220_REG_3, 0x1C, (uint8_t)(route << 2));
}

int ads1220_set_drdy_mode(struct ads1220 *d, enum ads1220_drdy_mode mode)
{
	return update_reg(d, ADS1220_REG_3, 0x02, (uint8_t)mode);
}

/* ------------------------------------------------------------------ */
/* Reference / mode settings                                          */
/* ------------------------------------------------------------------ */

void ads1220_set_vref_value_v(struct ads1220 *d, float vref)
{
	d->vref = vref;
}

float ads1220_get_vref_v(const struct ads1220 *d)
{
	return d->vref;
}

void ads1220_set_non_blocking_mode(struct ads1220 *d, bool non_blocking)
{
	d->non_blocking = non_blocking;
	if (!non_blocking) {
		k_sem_reset(&d->drdy_sem);  /* discard edges seen while not waiting */
	}
}

bool ads1220_get_non_blocking_mode(const struct ads1220 *d)
{
	return d->non_blocking;
}

int ads1220_set_avdd_avss_as_vref_and_calibrate(struct ads1220 *d)
{
	return calibrate_vref(d, ADS1220_VREF_AVDD_AVSS, ADS1220_MUX_AVDD_M_AVSS_4);
}

int ads1220_set_refp0_refn0_as_vref_and_calibrate(struct ads1220 *d)
{
	return calibrate_vref(d, ADS1220_VREF_REFP0_REFN0, ADS1220_MUX_REFPX_REFNX_4);
}

int ads1220_set_refp1_refn1_as_vref_and_calibrate(struct ads1220 *d)
{
	return calibrate_vref(d, ADS1220_VREF_REFP1_REFN1, ADS1220_MUX_REFPX_REFNX_4);
}

int ads1220_set_int_vref(struct ads1220 *d)
{
	ADS_TRY(ads1220_set_vref_source(d, ADS1220_VREF_INT));
	d->vref = 2.048f;
	return 0;
}

/* ------------------------------------------------------------------ */
/* Results                                                            */
/* ------------------------------------------------------------------ */

int ads1220_get_raw_data(struct ads1220 *d, int32_t *raw)
{
	uint32_t r;

	ADS_TRY(read_result(d, &r));

	/* Sign-extend 24-bit two's complement without relying on signed shifts. */
	*raw = (int32_t)(r ^ 0x800000U) - 0x800000;
	return 0;
}

int ads1220_get_voltage_mv(struct ads1220 *d, float *mv)
{
	int32_t raw;

	ADS_TRY(ads1220_get_raw_data(d, &raw));

	/* In the reference-monitor modes the conversion uses the internal 2.048 V. */
	float vref = d->ref_measurement ? 2.048f : d->vref;

	*mv = ((float)raw / ADS1220_RANGE) * vref * 1000.0f / (float)d->gain;
	return 0;
}

int ads1220_get_voltage_uv(struct ads1220 *d, float *uv)
{
	float mv;

	ADS_TRY(ads1220_get_voltage_mv(d, &mv));
	*uv = mv * 1000.0f;
	return 0;
}

int ads1220_get_temperature(struct ads1220 *d, float *celsius)
{
	uint32_t r;

	ADS_TRY(ads1220_enable_temperature_sensor(d, true));
	int ret = read_result(d, &r);
	int ret_off = ads1220_enable_temperature_sensor(d, false);

	if (ret != 0) {
		return ret;
	}
	if (ret_off != 0) {
		return ret_off;
	}

	/* 14-bit two's complement, left-justified in the 24-bit result, 0.03125 C/LSB */
	int16_t t = (int16_t)((r >> 10) & 0x3FFF);

	if (t & 0x2000) {
		t -= 0x4000;
	}
	*celsius = (float)t * 0.03125f;
	return 0;
}
