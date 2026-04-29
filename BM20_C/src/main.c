/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

//#define NRF_BENCH_MODE  /* undefine for production build */

#define BUF_SIZE 4096

static const struct device *spis_dev = DEVICE_DT_GET(DT_ALIAS(spis));
static const struct spi_config spis_config = {.operation = SPI_OP_MODE_SLAVE | SPI_WORD_SET(8) | SPI_MODE_CPHA};

#ifdef NRF_BENCH_MODE

#define BUTTON_DEBOUNCE_SAMPLES 4
#define BUTTON_RETRIGGER_GUARD_MS 350

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
static uint8_t last_rx13;
static uint8_t last_rx14;

static void fill_tx_pattern(void)
{
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
	int status;
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
	LOG_INF("diag: BUF_SIZE=%u, SPI slave 8-bit Mode1 (CPHA=1), button aliases sw0..sw3 enabled", BUF_SIZE);
	debounced_button_state = read_button_state();
	memset(button_change_count, 0, sizeof(button_change_count));
	for (size_t i = 0; i < 4; ++i) {
		button_next_press_ok_ms[i] = 0;
	}
	pending_button_events = 0;
	LOG_INF("buttons initial=0x%02X", debounced_button_state);

	size_t prev_frame_len = sizeof(rx_buffer);
	while (1) {
		memset(rx_buffer, 0x00, prev_frame_len);
		gpio_pin_set_dt(&led, 0);
		int ret = spi_transceive(spis_dev, &spis_config, &tx_set, &rx_set);
		gpio_pin_set_dt(&led, 1);

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
			last_rx13 = (ret > 13) ? rx_buffer[13] : 0;
			last_rx14 = (ret > 14) ? rx_buffer[14] : 0;
			if (pending_button_events != 0 && ret >= 2) {
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
			LOG_INF("count=%u ret=%d RX[0..3]=%02X %02X %02X %02X RX[13..14]=%02X %02X xor=%02X pend=0x%02X err=%u zero=%u short=%u marker=%u all00=%u allFF=%u",
				frame_count, ret, last_rx0, last_rx1, last_rx2, last_rx3, last_rx13, last_rx14, chk, pending_button_events,
				error_count, zero_frame_count, short_frame_count, marker_inject_count,
				all_00 ? 1u : 0u, all_ff ? 1u : 0u);
		}
	}

	return 0;
}

#else /* production build */

#define SENSOR_COUNT 3u
#define SENSOR_RING_LEN 320u
#define SENSOR_HEADER_BYTES 13u
#define SENSOR_PACKET_BYTES (SENSOR_HEADER_BYTES + SENSOR_RING_LEN)

#define SENSOR_MAGIC0 0x53u /* 'S' */
#define SENSOR_MAGIC1 0x42u /* 'B' */
#define SENSOR_PROTOCOL_VERSION 1u

#define CMD_STREAM_START  0xB0u
#define CMD_STREAM_STOP   0xB1u
#define CMD_STREAM_FETCH  0xB2u
#define CMD_SENSOR_SELECT 0xB3u

#define SENSOR_TS_MS 40u

#define TWO_PI 6.28318530717958647692f

LOG_MODULE_REGISTER(bm20c_spis_scope, LOG_LEVEL_INF);

static uint8_t tx_buffer[SENSOR_PACKET_BYTES];
static uint8_t rx_buffer[SENSOR_PACKET_BYTES];

static uint8_t sensor_ring_raw[SENSOR_RING_LEN];
static uint8_t sensor_ring_plot[SENSOR_RING_LEN];

static bool sensor_stream_enabled;
static uint8_t sensor_sequence;
static uint8_t selected_sensor;
static uint16_t sensor_write_index;
static uint32_t sensor_sample_counter;
static int64_t sensor_next_fill_ms;
static float sensor_phase_0;
static float sensor_phase_1;
static float sensor_phase_2;

static uint8_t xor_checksum(const uint8_t *buf, size_t len)
{
	uint8_t x = 0;
	for (size_t i = 0; i < len; ++i) {
		x ^= buf[i];
	}
	return x;
}

static int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
	if (v < lo) return lo;
	if (v > hi) return hi;
	return v;
}

static uint8_t scale_capture_to_u8(int32_t sample, int32_t min_capture, int32_t max_capture)
{
	int32_t clamped = clamp_i32(sample, min_capture, max_capture);
	int32_t range = max_capture - min_capture;
	int32_t scaled = (range > 0) ? ((clamped - min_capture) * 255) / range : 127;
	return (uint8_t)clamp_i32(scaled, 0, 255);
}

static uint8_t map_u8_to_plot_x(uint8_t u8_value)
{
	/* 120 is the zero axis center. 0..119 represent negative, 121..240 positive. */
	int32_t centered = (int32_t)u8_value - 128;
	int32_t x = 120 + (centered * 120) / 127;
	return (uint8_t)clamp_i32(x, 0, 240);
}

static int32_t sensor_sim_capture(uint8_t sensor_id, uint32_t sample_index)
{
	float t = (float)sample_index;

	switch (sensor_id) {
	case 1: {
		/* 10-bit triangular wave emulation: [-512 .. +511]. */
		int32_t phase = (int32_t)(sample_index % 160u);
		int32_t tri = (phase < 80) ? (phase * 13 - 520) : ((159 - phase) * 13 - 520);
		return clamp_i32(tri, -512, 511);
	}
	case 2: {
		/* 16-bit-like mixed signal emulation: [-32768 .. +32767]. */
		float mix =
			14000.0f * sinf(sensor_phase_2) +
			9000.0f * sinf(0.13f * t + 0.8f) +
			5000.0f * cosf(0.07f * t);
		sensor_phase_2 += 0.031f;
		if (sensor_phase_2 >= TWO_PI) sensor_phase_2 -= TWO_PI;
		return clamp_i32((int32_t)mix, -32768, 32767);
	}
	case 0:
	default: {
		/* 12-bit sine emulation: [-2048 .. +2047]. */
		float s = 1800.0f * sinf(sensor_phase_0) + 420.0f * sinf(sensor_phase_1);
		sensor_phase_0 += 0.12f;
		sensor_phase_1 += 0.049f;
		if (sensor_phase_0 >= TWO_PI) sensor_phase_0 -= TWO_PI;
		if (sensor_phase_1 >= TWO_PI) sensor_phase_1 -= TWO_PI;
		return clamp_i32((int32_t)s, -2048, 2047);
	}
	}
}

static void apply_sensor_selection(uint8_t sensor_id)
{
	if (sensor_id >= SENSOR_COUNT || sensor_id == selected_sensor) {
		return;
	}

	selected_sensor = sensor_id;
	/* Re-prime waveform state so the next packet clearly reflects the new source. */
	sensor_sample_counter = 0;
	sensor_phase_0 = 0.0f;
	sensor_phase_1 = 0.0f;
	sensor_phase_2 = 0.0f;
	memset(sensor_ring_raw, 127, sizeof(sensor_ring_raw));
	memset(sensor_ring_plot, 120, sizeof(sensor_ring_plot));
	sensor_write_index = 0;
	sensor_next_fill_ms = 0;
}

static void sensor_fill_ring_once(void)
{
	for (uint16_t i = 0; i < SENSOR_RING_LEN; ++i) {
		int32_t capture = sensor_sim_capture(selected_sensor, sensor_sample_counter++);
		uint8_t scaled_u8;
		switch (selected_sensor) {
		case 1: scaled_u8 = scale_capture_to_u8(capture, -512, 511); break;
		case 2: scaled_u8 = scale_capture_to_u8(capture, -32768, 32767); break;
		case 0:
		default: scaled_u8 = scale_capture_to_u8(capture, -2048, 2047); break;
		}

		sensor_ring_raw[sensor_write_index] = scaled_u8;
		sensor_ring_plot[sensor_write_index] = map_u8_to_plot_x(scaled_u8);
		sensor_write_index = (uint16_t)((sensor_write_index + 1u) % SENSOR_RING_LEN);
	}
}

static void process_command_frame(const uint8_t *rx, size_t len)
{
	if (!rx || len == 0u) {
		return;
	}

	switch (rx[0]) {
	case CMD_STREAM_START:
		sensor_stream_enabled = true;
		if (len >= 2u) {
			apply_sensor_selection(rx[1]);
		}
		sensor_next_fill_ms = 0;
		break;
	case CMD_STREAM_STOP:
		sensor_stream_enabled = false;
		memset(sensor_ring_raw, 127, sizeof(sensor_ring_raw));
		memset(sensor_ring_plot, 120, sizeof(sensor_ring_plot));
		sensor_write_index = 0;
		sensor_next_fill_ms = 0;
		break;
	case CMD_SENSOR_SELECT:
		if (len >= 2u) {
			apply_sensor_selection(rx[1]);
		}
		break;
	case CMD_STREAM_FETCH:
		/* Keep sensor selection sticky during streaming: master can resend desired sensor in every fetch. */
		if (len >= 2u) {
			apply_sensor_selection(rx[1]);
		}
		break;
	default:
		break;
	}
}

static void prepare_tx_packet(void)
{
	if (sensor_stream_enabled) {
		int64_t now_ms = k_uptime_get();
		if (sensor_next_fill_ms == 0) {
			sensor_fill_ring_once();
			sensor_sequence++;
			sensor_next_fill_ms = now_ms + SENSOR_TS_MS;
		} else {
			uint8_t fills = 0;
			while (now_ms >= sensor_next_fill_ms && fills < 4u) {
				sensor_fill_ring_once();
				sensor_sequence++;
				sensor_next_fill_ms += SENSOR_TS_MS;
				fills++;
			}
		}
	}

	memset(tx_buffer, 0, sizeof(tx_buffer));
	tx_buffer[0] = SENSOR_MAGIC0;
	tx_buffer[1] = SENSOR_MAGIC1;
	tx_buffer[2] = SENSOR_PROTOCOL_VERSION;
	tx_buffer[3] = sensor_stream_enabled ? 0x01u : 0x00u;
	tx_buffer[4] = sensor_sequence;
	tx_buffer[5] = selected_sensor;
	tx_buffer[6] = (uint8_t)(SENSOR_TS_MS & 0xFFu);
	tx_buffer[7] = (uint8_t)((SENSOR_TS_MS >> 8) & 0xFFu);
	tx_buffer[8] = (uint8_t)(SENSOR_RING_LEN & 0xFFu);
	tx_buffer[9] = (uint8_t)((SENSOR_RING_LEN >> 8) & 0xFFu);
	tx_buffer[10] = (uint8_t)(sensor_write_index & 0xFFu);
	tx_buffer[11] = (uint8_t)((sensor_write_index >> 8) & 0xFFu);
	tx_buffer[12] = 0u; /* checksum */

	for (uint16_t i = 0; i < SENSOR_RING_LEN; ++i) {
		uint16_t idx = (uint16_t)((sensor_write_index + i) % SENSOR_RING_LEN);
		tx_buffer[SENSOR_HEADER_BYTES + i] = sensor_ring_plot[idx];
	}
	tx_buffer[12] = xor_checksum(tx_buffer, sizeof(tx_buffer));
}

int main(void)
{
	struct spi_buf tx_spi_buf = {.buf = tx_buffer, .len = sizeof(tx_buffer)};
	struct spi_buf rx_spi_buf = {.buf = rx_buffer, .len = sizeof(rx_buffer)};
	struct spi_buf_set tx_set = {.buffers = &tx_spi_buf, .count = 1};
	struct spi_buf_set rx_set = {.buffers = &rx_spi_buf, .count = 1};

	__ASSERT(device_is_ready(spis_dev), "SPI device not ready");

	memset(tx_buffer, 0, sizeof(tx_buffer));
	memset(rx_buffer, 0, sizeof(rx_buffer));
	memset(sensor_ring_raw, 127, sizeof(sensor_ring_raw));
	memset(sensor_ring_plot, 120, sizeof(sensor_ring_plot));
	sensor_stream_enabled = false;
	sensor_sequence = 0;
	selected_sensor = 0;
	sensor_write_index = 0;
	sensor_sample_counter = 0;
	sensor_next_fill_ms = 0;
	sensor_phase_0 = 0.0f;
	sensor_phase_1 = 0.0f;
	sensor_phase_2 = 0.0f;
	prepare_tx_packet();

	LOG_INF("BM20_C SPIS scope ready (production, packet=%u bytes, samples=%u, Ts=%u ms)",
		(unsigned int)SENSOR_PACKET_BYTES,
		(unsigned int)SENSOR_RING_LEN,
		(unsigned int)SENSOR_TS_MS);

	uint32_t transfer_count = 0;
	while (1) {
		int ret = spi_transceive(spis_dev, &spis_config, &tx_set, &rx_set);
		if (ret < 0) {
			LOG_ERR("spi_transceive failed: %d", ret);
			k_msleep(10);
			continue;
		}

		process_command_frame(rx_buffer, sizeof(rx_buffer));
		prepare_tx_packet();

		transfer_count++;
		if ((transfer_count % 500u) == 0u) {
			LOG_INF("xfer=%u stream=%u seq=%u sensor=%u write=%u",
				(unsigned int)transfer_count,
				sensor_stream_enabled ? 1u : 0u,
				(unsigned int)sensor_sequence,
				(unsigned int)selected_sensor,
				(unsigned int)sensor_write_index);
		}
	}

	return 0;
}

#endif /* NRF_BENCH_MODE */
