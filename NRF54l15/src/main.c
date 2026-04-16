/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>

#define BUF_SIZE 8
static const struct device *spis_dev = DEVICE_DT_GET(DT_ALIAS(spis));
static const struct spi_config spis_config = {.operation = SPI_OP_MODE_SLAVE | SPI_WORD_SET(8)};

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led), gpios);

LOG_MODULE_REGISTER(spi_slave_test, LOG_LEVEL_INF);

static uint8_t tx_buffer[BUF_SIZE];
static uint8_t rx_buffer[BUF_SIZE];

static void fill_tx_pattern(void)
{
	/* Fixed pattern so RP side can use deterministic expected bytes in sweep tests. */
	for (size_t i = 0; i < BUF_SIZE; ++i) {
		tx_buffer[i] = (i & 1u) ? 0xA5 : 0x5A;
	}
}

static uint8_t xor_checksum(const uint8_t *buf, size_t len)
{
	uint8_t x = 0;
	for (size_t i = 0; i < len; ++i) {
		x ^= buf[i];
	}
	return x;
}

int main(void)
{
	bool status;
	uint32_t frame_count = 0;
	struct spi_buf tx_spi_buf = {.buf = tx_buffer, .len = sizeof(tx_buffer)};
	struct spi_buf rx_spi_buf = {.buf = rx_buffer, .len = sizeof(rx_buffer)};
	struct spi_buf_set tx_set = {.buffers = &tx_spi_buf, .count = 1};
	struct spi_buf_set rx_set = {.buffers = &rx_spi_buf, .count = 1};

	LOG_INF("Hello world from %s", CONFIG_BOARD_TARGET);

	status = gpio_is_ready_dt(&led);
	__ASSERT(status, "Error: GPIO Device not ready");

	status = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	__ASSERT(status == 0, "Could not configure led GPIO");

	status = device_is_ready(spis_dev);
	__ASSERT(status, "Error: SPI device is not ready");

	fill_tx_pattern();
	LOG_INF("TX pattern[0..3]=%02X %02X %02X %02X",
		tx_buffer[0], tx_buffer[1], tx_buffer[2], tx_buffer[3]);

	while (1) {
		memset(rx_buffer, 0x00, sizeof(rx_buffer));
		gpio_pin_set_dt(&led, 0);
		int ret = spi_transceive(spis_dev, &spis_config, &tx_set, &rx_set);
		gpio_pin_set_dt(&led, 1);

		/* In Zephyr slave mode, spi_transceive() returns number of received frames on success. */
		if (ret < 0) {
			LOG_ERR("spi_transceive failed: %d", ret);
			k_msleep(10);
			continue;
		}

		frame_count++;
		if ((frame_count % 200u) == 0u) {
			uint8_t chk = xor_checksum(rx_buffer, sizeof(rx_buffer));
			LOG_INF("count=%u frames=%d RX[0..3]=%02X %02X %02X %02X xor=%02X",
				frame_count, ret, rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3], chk);
		}
	}

	return 0;
}
