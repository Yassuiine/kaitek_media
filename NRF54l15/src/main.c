/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>

#define BUF_SIZE 4096
#define BUTTON_DEBOUNCE_SAMPLES 4
#define BUTTON_RETRIGGER_GUARD_MS 350
static const struct device *spis_dev = DEVICE_DT_GET(DT_ALIAS(spis));
static const struct spi_config spis_config = {.operation = SPI_OP_MODE_SLAVE | SPI_WORD_SET(8)};

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led), gpios);
static const struct gpio_dt_spec btn0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec btn1 = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);
static const struct gpio_dt_spec btn2 = GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios);
static const struct gpio_dt_spec btn3 = GPIO_DT_SPEC_GET(DT_ALIAS(sw3), gpios);

LOG_MODULE_REGISTER(spi_slave_test, LOG_LEVEL_INF);

static uint8_t tx_buffer[BUF_SIZE];
static uint8_t rx_buffer[BUF_SIZE];
static uint8_t pending_button_events;
static uint8_t debounced_button_state;
static uint8_t button_change_count[4];
static int64_t button_next_press_ok_ms[4];
static uint32_t transfer_count;
static uint32_t error_count;
static uint32_t zero_frame_count;
static uint32_t short_frame_count;
static uint32_t marker_inject_count;
static uint8_t last_rx0;
static uint8_t last_rx1;
static uint8_t last_rx2;
static uint8_t last_rx3;

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

static bool buffer_all_value(const uint8_t *buf, size_t len, uint8_t value)
{
	for (size_t i = 0; i < len; ++i) {
		if (buf[i] != value) {
			return false;
		}
	}
	return true;
}

static uint8_t read_button_state(void)
{
	uint8_t state = 0;
	/* Active-low buttons: pressed => bit set in returned mask. */
	if (gpio_pin_get_dt(&btn0) == 0) state |= BIT(0);
	if (gpio_pin_get_dt(&btn1) == 0) state |= BIT(1);
	if (gpio_pin_get_dt(&btn2) == 0) state |= BIT(2);
	if (gpio_pin_get_dt(&btn3) == 0) state |= BIT(3);
	return state;
}

static void update_button_events(void)
{
	uint8_t raw = read_button_state();
	uint8_t rising_press = 0;
	int64_t now_ms = k_uptime_get();

	for (uint8_t i = 0; i < 4; ++i) {
		uint8_t bit = BIT(i);
		bool raw_pressed = (raw & bit) != 0;
		bool stable_pressed = (debounced_button_state & bit) != 0;

		if (raw_pressed == stable_pressed) {
			button_change_count[i] = 0;
			continue;
		}

		if (button_change_count[i] < 0xFF) {
			button_change_count[i]++;
		}
		if (button_change_count[i] < BUTTON_DEBOUNCE_SAMPLES) {
			continue;
		}

		button_change_count[i] = 0;
		if (raw_pressed) {
			debounced_button_state |= bit;
			if (now_ms >= button_next_press_ok_ms[i]) {
				rising_press |= bit;
				button_next_press_ok_ms[i] = now_ms + BUTTON_RETRIGGER_GUARD_MS;
			}
		} else {
			debounced_button_state &= (uint8_t)~bit;
		}
	}

	if (rising_press != 0) {
		LOG_INF("buttons rising=0x%02X raw=0x%02X stable=0x%02X", rising_press, raw, debounced_button_state);
	}
	pending_button_events |= rising_press;
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

	status = gpio_is_ready_dt(&btn0);
	__ASSERT(status, "Button0 GPIO not ready");
	status = gpio_is_ready_dt(&btn1);
	__ASSERT(status, "Button1 GPIO not ready");
	status = gpio_is_ready_dt(&btn2);
	__ASSERT(status, "Button2 GPIO not ready");
	status = gpio_is_ready_dt(&btn3);
	__ASSERT(status, "Button3 GPIO not ready");

	status = gpio_pin_configure_dt(&btn0, GPIO_INPUT | GPIO_PULL_UP);
	__ASSERT(status == 0, "Could not configure button0 GPIO");
	status = gpio_pin_configure_dt(&btn1, GPIO_INPUT | GPIO_PULL_UP);
	__ASSERT(status == 0, "Could not configure button1 GPIO");
	status = gpio_pin_configure_dt(&btn2, GPIO_INPUT | GPIO_PULL_UP);
	__ASSERT(status == 0, "Could not configure button2 GPIO");
	status = gpio_pin_configure_dt(&btn3, GPIO_INPUT | GPIO_PULL_UP);
	__ASSERT(status == 0, "Could not configure button3 GPIO");

	status = device_is_ready(spis_dev);
	__ASSERT(status, "Error: SPI device is not ready");

	fill_tx_pattern();
	LOG_INF("TX pattern[0..3]=%02X %02X %02X %02X",
		tx_buffer[0], tx_buffer[1], tx_buffer[2], tx_buffer[3]);
	LOG_INF("diag: BUF_SIZE=%u, SPI slave 8-bit mode, button aliases sw0..sw3 enabled", BUF_SIZE);
	debounced_button_state = read_button_state();
	memset(button_change_count, 0, sizeof(button_change_count));
	for (size_t i = 0; i < 4; ++i) {
		button_next_press_ok_ms[i] = 0;
	}
	pending_button_events = 0;
	LOG_INF("buttons initial=0x%02X", debounced_button_state);

	size_t prev_frame_len = sizeof(rx_buffer); /* start safe: full clear on first iteration */
	while (1) {
		/* Clear only as many bytes as the previous transfer received.
		 * Clearing all 4096 bytes on every 4-byte benchmark frame added ~300 µs of
		 * dead time between spi_transceive calls, causing the Pico to fire again
		 * before the slave re-armed (50 % failure with 200 µs gap). */
		memset(rx_buffer, 0x00, prev_frame_len);
		gpio_pin_set_dt(&led, 0);
		int ret = spi_transceive(spis_dev, &spis_config, &tx_set, &rx_set);
		gpio_pin_set_dt(&led, 1);

		/* In Zephyr slave mode, spi_transceive() returns number of received frames on success. */
		if (ret < 0) {
			error_count++;
			LOG_ERR("spi_transceive failed: %d", ret);
			k_msleep(10);
			continue;
		}

		transfer_count++;
		if (ret == 0) {
			zero_frame_count++;
		}
		if (ret > 0 && ret < 8) {
			short_frame_count++;
		}

		if (ret > 0) {
			prev_frame_len = (size_t)ret;
			update_button_events();
			memcpy(tx_buffer, rx_buffer, (size_t)ret);
			last_rx0 = rx_buffer[0];
			last_rx1 = (ret > 1) ? rx_buffer[1] : 0;
			last_rx2 = (ret > 2) ? rx_buffer[2] : 0;
			last_rx3 = (ret > 3) ? rx_buffer[3] : 0;
			if (pending_button_events != 0 && ret >= 2) {
				/* Inject one-shot event marker in next transfer payload. */
				tx_buffer[0] = 0xA5;
				tx_buffer[1] = pending_button_events;
				marker_inject_count++;
				LOG_INF("inject marker: evt=0x%02X count=%u ret=%d", pending_button_events, marker_inject_count, ret);
				pending_button_events = 0;
			}
		}

		frame_count++;
		if ((frame_count % 200u) == 0u) {
			uint8_t chk = xor_checksum(rx_buffer, (ret > 0) ? (size_t)ret : sizeof(rx_buffer));
			bool all_ff = (ret > 0) ? buffer_all_value(rx_buffer, (size_t)ret, 0xFF) : false;
			bool all_00 = (ret > 0) ? buffer_all_value(rx_buffer, (size_t)ret, 0x00) : false;
			LOG_INF("count=%u ret=%d RX[0..3]=%02X %02X %02X %02X xor=%02X pend=0x%02X err=%u zero=%u short=%u marker=%u all00=%u allFF=%u",
				frame_count, ret, last_rx0, last_rx1, last_rx2, last_rx3, chk, pending_button_events,
				error_count, zero_frame_count, short_frame_count, marker_inject_count,
				all_00 ? 1u : 0u, all_ff ? 1u : 0u);
		}
	}

	return 0;
}
