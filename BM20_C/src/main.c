/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/dt-bindings/adc/nrf-saadc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

//#define NRF_BENCH_MODE  /* undefine for production build */
#define KAIMA_DEBUG_CARD  /* enable extension connector LED/UART/ANA debug card */

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
#define CMD_SET_MODE      0xB4u  /* payload byte: 0=raw waveform, 1=FFT */
#define CMD_LOOPBACK_MODE 0xB5u  /* payload byte: 0=scope packets, 1=previous-frame echo */
#define CMD_AUTO_SCALE    0xB6u  /* payload byte: 0=fixed hardware range, 1=window min/max */

#define SENSOR_TS_MS 40u

#define TWO_PI 6.28318530717958647692f

#define MIC_TDM_DEV DEVICE_DT_GET(DT_NODELABEL(tdm))
#define MIC_SAMPLE_RATE 48000u
#define MIC_BIT_DEPTH 32u
#define MIC_CHANNELS 2u
#define MIC_BLOCK_SIZE (MIC_SAMPLE_RATE / 100u * MIC_CHANNELS * (MIC_BIT_DEPTH / 8u))
#define MIC_BLOCK_COUNT 3u
#define MIC_THREAD_STACK_SIZE 2048u
#define MIC_THREAD_PRIORITY 5

#define PIEZO_ADC_DEV DEVICE_DT_GET(DT_NODELABEL(adc))
#define PIEZO_ADC_CHANNEL 0u
#define PIEZO_ADC_RESOLUTION 12u
#define PIEZO_ADC_SAMPLE_PERIOD_MS 10u
#define PIEZO_ADC_THREAD_STACK_SIZE 1024u
#define PIEZO_ADC_THREAD_PRIORITY 6
#define ANA31_ADC_CHANNEL 2u
#define ANA31_ADC_RESOLUTION 12u
#define ANA31_ADC_SAMPLE_PERIOD_MS 10u
#define ANA31_ADC_THREAD_STACK_SIZE 1024u
#define ANA31_ADC_THREAD_PRIORITY 6
#define ADC_12BIT_MAX_RAW ((1u << 12u) - 1u)
#define ADC_GAIN_1_4_FULL_SCALE_MV 2400u
#define ANA31_ELECTRET_FIXED_MIN_MV 1150u
#define ANA31_ELECTRET_FIXED_MAX_MV 2150u
#define ANA31_ELECTRET_AUTOSCALE_MIN_SPAN_MV 250u
#define ANA31_ELECTRET_FIXED_MIN_RAW \
	((ANA31_ELECTRET_FIXED_MIN_MV * ADC_12BIT_MAX_RAW) / ADC_GAIN_1_4_FULL_SCALE_MV)
#define ANA31_ELECTRET_FIXED_MAX_RAW \
	((ANA31_ELECTRET_FIXED_MAX_MV * ADC_12BIT_MAX_RAW) / ADC_GAIN_1_4_FULL_SCALE_MV)
#define ANA31_ELECTRET_AUTOSCALE_MIN_SPAN_RAW \
	((ANA31_ELECTRET_AUTOSCALE_MIN_SPAN_MV * ADC_12BIT_MAX_RAW) / ADC_GAIN_1_4_FULL_SCALE_MV)

LOG_MODULE_REGISTER(bm20c_spis_scope, LOG_LEVEL_INF);

K_MEM_SLAB_DEFINE(mic_rx_slab, MIC_BLOCK_SIZE, MIC_BLOCK_COUNT, 4);
K_MUTEX_DEFINE(mic_ring_lock);
K_MUTEX_DEFINE(piezo_ring_lock);
K_MUTEX_DEFINE(ana31_ring_lock);
K_MUTEX_DEFINE(adc_lock);

static uint8_t tx_buffer[SENSOR_PACKET_BYTES];
static uint8_t rx_buffer[SENSOR_PACKET_BYTES];

static uint8_t sensor_ring_raw[SENSOR_RING_LEN];
static uint8_t sensor_ring_plot[SENSOR_RING_LEN];
static int32_t mic_ring_capture[SENSOR_RING_LEN];
static uint8_t mic_ring_raw[SENSOR_RING_LEN];
static uint8_t mic_ring_plot[SENSOR_RING_LEN];
static uint16_t mic_write_index;
static int32_t piezo_ring_capture[SENSOR_RING_LEN];
static uint8_t piezo_ring_raw[SENSOR_RING_LEN];
static uint8_t piezo_ring_plot[SENSOR_RING_LEN];
static uint16_t piezo_write_index;
static int16_t piezo_sample_buffer;
static int32_t ana31_ring_capture[SENSOR_RING_LEN];
static uint8_t ana31_ring_raw[SENSOR_RING_LEN];
static uint8_t ana31_ring_plot[SENSOR_RING_LEN];
static uint16_t ana31_write_index;
static int16_t ana31_sample_buffer;

static bool sensor_stream_enabled;
static bool sensor_fft_enabled = false;
static bool loopback_enabled;
static bool sensor_auto_scale_enabled = true;
static uint8_t sensor_sequence;
static uint8_t selected_sensor;
static uint16_t sensor_write_index;
static uint32_t sensor_sample_counter;
static int64_t sensor_next_fill_ms;
static float sensor_phase_0;
static float sensor_phase_1;
static float sensor_phase_2;
static float sensor_phase_3;
static float sensor_phase_4;

#ifdef KAIMA_DEBUG_CARD

#define KAIMA_CARD_LED_COUNT 12u
#define KAIMA_CARD_STACK_SIZE 2048u
#define KAIMA_CARD_PRIORITY 8
#define KAIMA_CARD_BOOT_STEP_MS 500u
#define KAIMA_CARD_PULSE_MS 80u
#define KAIMA_CARD_UART_PERIOD_MS 1000u
#define KAIMA_CARD_ADC_PERIOD_MS 250u
#define KAIMA_CARD_HEARTBEAT_MS 500u
#define KAIMA_CARD_ADC_CHANNEL 0u
#define KAIMA_CARD_ADC_RESOLUTION 12u
#define KAIMA_CARD_UART_GAP_MS 20u
#define KAIMA_CARD_UART_PREAMBLE "XXXXXXXXXXXXXXXX\r\n"
#define KAIMA_CARD_ADC_REF_MV 600u
#define KAIMA_CARD_ADC_GAIN_DEN 4u
#define KAIMA_CARD_ANA_LOW_MV 1150u
#define KAIMA_CARD_ANA_HIGH_MV 2150u
#define KAIMA_CARD_ANA_LOW_RAW ((KAIMA_CARD_ANA_LOW_MV * ((1u << KAIMA_CARD_ADC_RESOLUTION) - 1u)) / \
				(KAIMA_CARD_ADC_REF_MV * KAIMA_CARD_ADC_GAIN_DEN))
#define KAIMA_CARD_ANA_HIGH_RAW ((KAIMA_CARD_ANA_HIGH_MV * ((1u << KAIMA_CARD_ADC_RESOLUTION) - 1u)) / \
				 (KAIMA_CARD_ADC_REF_MV * KAIMA_CARD_ADC_GAIN_DEN))

enum kaima_card_led {
	KAIMA_LED_SYSTEM = 0,    /* P1.27 */
	KAIMA_LED_SPI_TX,        /* P3.12 */
	KAIMA_LED_SPI_RX,        /* P1.24 */
	KAIMA_LED_STREAM,        /* P3.05 */
	KAIMA_LED_LOOPBACK,      /* P3.02 */
	KAIMA_LED_MIC,           /* P3.04 */
	KAIMA_LED_PIEZO,         /* P3.11 */
	KAIMA_LED_ANA10,         /* P3.00, ANA10 > 2.15 V */
	KAIMA_LED_ANA14,         /* P1.28, ANA14 > 2.15 V */
	KAIMA_LED_UART_TX,       /* P3.09 */
	KAIMA_LED_BLE0,          /* P1.22, ANA10 < 1.15 V */
	KAIMA_LED_BLE1,          /* P1.23, ANA14 < 1.15 V */
};

struct kaima_card_gpio {
	const struct device *port;
	gpio_pin_t pin;
	const char *name;
};

static const struct device *kaima_card_uart_dev = DEVICE_DT_GET(DT_ALIAS(kaima_uart));
static const struct device *kaima_card_adc_dev = PIEZO_ADC_DEV;

static const struct kaima_card_gpio kaima_card_leds[KAIMA_CARD_LED_COUNT] = {
	{DEVICE_DT_GET(DT_NODELABEL(gpio1)), 27, "P1.27"},
	{DEVICE_DT_GET(DT_NODELABEL(gpio3)), 12, "P3.12"},
	{DEVICE_DT_GET(DT_NODELABEL(gpio1)), 24, "P1.24"},
	{DEVICE_DT_GET(DT_NODELABEL(gpio3)), 5, "P3.05"},
	{DEVICE_DT_GET(DT_NODELABEL(gpio3)), 2, "P3.02"},
	{DEVICE_DT_GET(DT_NODELABEL(gpio3)), 4, "P3.04"},
	{DEVICE_DT_GET(DT_NODELABEL(gpio3)), 11, "P3.11"},
	{DEVICE_DT_GET(DT_NODELABEL(gpio3)), 0, "P3.00"},
	{DEVICE_DT_GET(DT_NODELABEL(gpio1)), 28, "P1.28"},
	{DEVICE_DT_GET(DT_NODELABEL(gpio3)), 9, "P3.09"},
	{DEVICE_DT_GET(DT_NODELABEL(gpio1)), 22, "P1.22"},
	{DEVICE_DT_GET(DT_NODELABEL(gpio1)), 23, "P1.23"},
};

static bool kaima_card_led_ok[KAIMA_CARD_LED_COUNT];
static int64_t kaima_card_pulse_until[KAIMA_CARD_LED_COUNT];
static bool kaima_card_mic_ok;
static bool kaima_card_piezo_ok;
static bool kaima_card_ana10_ok;
static bool kaima_card_ana14_ok;
static bool kaima_card_ana10_low;
static bool kaima_card_ana10_high;
static bool kaima_card_ana14_low;
static bool kaima_card_ana14_high;
static int16_t kaima_card_ana10_sample;
static int16_t kaima_card_ana14_sample;
static bool kaima_card_uart_ready;
static uint32_t kaima_card_uart_tx_count;
static uint32_t kaima_card_uart_drop_count;
static uint32_t kaima_card_uart_rx_count;
K_SEM_DEFINE(kaima_card_uart_tx_done, 0, 1);

static void kaima_card_set_led(enum kaima_card_led led, bool on)
{
	if (led >= KAIMA_CARD_LED_COUNT || !kaima_card_led_ok[led]) {
		return;
	}
	(void)gpio_pin_set(kaima_card_leds[led].port, kaima_card_leds[led].pin, on ? 1 : 0);
}

static void kaima_card_pulse(enum kaima_card_led led)
{
	if (led < KAIMA_CARD_LED_COUNT) {
		kaima_card_pulse_until[led] = k_uptime_get() + KAIMA_CARD_PULSE_MS;
	}
}

static void kaima_card_uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	if (evt->type == UART_TX_DONE || evt->type == UART_TX_ABORTED) {
		k_sem_give(&kaima_card_uart_tx_done);
	}
}

static void kaima_card_uart_write(const char *text)
{
	if (!kaima_card_uart_ready) {
		kaima_card_uart_drop_count++;
		return;
	}

	while (k_sem_take(&kaima_card_uart_tx_done, K_NO_WAIT) == 0) {
	}

	int ret = uart_tx(kaima_card_uart_dev, (const uint8_t *)text, strlen(text), 200000);
	if (ret < 0) {
		kaima_card_uart_drop_count++;
		LOG_ERR("Kaima UART tx failed: %d", ret);
		return;
	}

	if (k_sem_take(&kaima_card_uart_tx_done, K_MSEC(200)) < 0) {
		kaima_card_uart_drop_count++;
		(void)uart_tx_abort(kaima_card_uart_dev);
		LOG_ERR("Kaima UART tx timeout");
		return;
	}

	kaima_card_uart_tx_count++;
	kaima_card_pulse(KAIMA_LED_UART_TX);
	k_msleep(KAIMA_CARD_UART_GAP_MS);
}

static void kaima_card_uart_write_framed(const char *text)
{
	kaima_card_uart_write(KAIMA_CARD_UART_PREAMBLE);
	kaima_card_uart_write(text);
}

static int kaima_card_adc_read(uint8_t input_positive, int16_t *sample)
{
	struct adc_channel_cfg ch_cfg = {
		.gain = ADC_GAIN_1_4,
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME_DEFAULT,
		.channel_id = KAIMA_CARD_ADC_CHANNEL,
		.input_positive = input_positive,
	};
	struct adc_sequence sequence = {
		.channels = BIT(KAIMA_CARD_ADC_CHANNEL),
		.buffer = sample,
		.buffer_size = sizeof(*sample),
		.resolution = KAIMA_CARD_ADC_RESOLUTION,
	};

	int ret = adc_channel_setup(kaima_card_adc_dev, &ch_cfg);
	if (ret < 0) {
		return ret;
	}

	return adc_read(kaima_card_adc_dev, &sequence);
}

static void kaima_card_restore_piezo_adc(void)
{
	struct adc_channel_cfg ch_cfg = {
		.gain = ADC_GAIN_1_4,
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME_DEFAULT,
		.channel_id = PIEZO_ADC_CHANNEL,
		.input_positive = NRF_SAADC_AIN6,
	};

	(void)adc_channel_setup(kaima_card_adc_dev, &ch_cfg);
}

static void kaima_card_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	int64_t next_uart_ms = 0;
	int64_t next_adc_ms = 0;
	int64_t next_heartbeat_ms = 0;
	bool heartbeat_on = false;
	uint32_t uart_counter = 0;
	char uart_line[128];

	kaima_card_uart_ready = device_is_ready(kaima_card_uart_dev);
	LOG_INF("Kaima debug card enabled");
	LOG_INF("Kaima UART %s: TX=P3.01 RX=P3.10 baud=115200 8N1",
		kaima_card_uart_ready ? "ready" : "not ready");
	if (kaima_card_uart_ready) {
		int ret = uart_callback_set(kaima_card_uart_dev, kaima_card_uart_cb, NULL);

		if (ret < 0) {
			LOG_ERR("Kaima UART callback setup failed: %d", ret);
			kaima_card_uart_ready = false;
		}
	}

	for (uint8_t i = 0; i < KAIMA_CARD_LED_COUNT; i++) {
		const struct kaima_card_gpio *led = &kaima_card_leds[i];

		if (!device_is_ready(led->port)) {
			LOG_ERR("Kaima LED %s GPIO not ready", kaima_card_leds[i].name);
			continue;
		}

		if (gpio_pin_configure(led->port, led->pin, GPIO_OUTPUT_INACTIVE) < 0) {
			LOG_ERR("Kaima LED %s configure failed", kaima_card_leds[i].name);
			continue;
		}

		kaima_card_led_ok[i] = true;
	}

	kaima_card_uart_write_framed("0123456789abcdefghijklmnopqrstuvwxyz\r\n");
	kaima_card_uart_write_framed("KAIMA_HWUART_V5_BOOT\r\n");

	for (uint8_t i = 0; i < KAIMA_CARD_LED_COUNT; i++) {
		kaima_card_set_led((enum kaima_card_led)i, true);
		k_msleep(KAIMA_CARD_BOOT_STEP_MS);
		kaima_card_set_led((enum kaima_card_led)i, false);
	}

	while (1) {
		int64_t now_ms = k_uptime_get();

		if (now_ms >= next_adc_ms && device_is_ready(kaima_card_adc_dev)) {
			int ret10;
			int ret14;

			k_mutex_lock(&adc_lock, K_FOREVER);
			ret10 = kaima_card_adc_read(NRF_SAADC_AIN3, &kaima_card_ana10_sample);
			ret14 = kaima_card_adc_read(NRF_SAADC_AIN1, &kaima_card_ana14_sample);
			kaima_card_restore_piezo_adc();
			k_mutex_unlock(&adc_lock);

			kaima_card_ana10_ok = (ret10 == 0);
			kaima_card_ana14_ok = (ret14 == 0);
			kaima_card_ana10_low = kaima_card_ana10_ok &&
				(kaima_card_ana10_sample < (int16_t)KAIMA_CARD_ANA_LOW_RAW);
			kaima_card_ana10_high = kaima_card_ana10_ok &&
				(kaima_card_ana10_sample > (int16_t)KAIMA_CARD_ANA_HIGH_RAW);
			kaima_card_ana14_low = kaima_card_ana14_ok &&
				(kaima_card_ana14_sample < (int16_t)KAIMA_CARD_ANA_LOW_RAW);
			kaima_card_ana14_high = kaima_card_ana14_ok &&
				(kaima_card_ana14_sample > (int16_t)KAIMA_CARD_ANA_HIGH_RAW);
			next_adc_ms = now_ms + KAIMA_CARD_ADC_PERIOD_MS;
		}

		if (now_ms >= next_uart_ms) {
			unsigned char rx_char;

			while (device_is_ready(kaima_card_uart_dev) &&
			       uart_poll_in(kaima_card_uart_dev, &rx_char) == 0) {
				kaima_card_uart_rx_count++;
				kaima_card_pulse(KAIMA_LED_UART_TX);
			}

			snprintk(uart_line, sizeof(uart_line),
				"KAIMA_V5 %u A10=%d A14=%d low=%u/%u high=%u/%u TX=%u RX=%u\r\n",
				(unsigned int)uart_counter++,
				(int)kaima_card_ana10_sample,
				(int)kaima_card_ana14_sample,
				kaima_card_ana10_low ? 1u : 0u,
				kaima_card_ana14_low ? 1u : 0u,
				kaima_card_ana10_high ? 1u : 0u,
				kaima_card_ana14_high ? 1u : 0u,
				(unsigned int)kaima_card_uart_tx_count,
				(unsigned int)kaima_card_uart_rx_count);
			kaima_card_uart_write_framed(uart_line);
			LOG_INF("Kaima UART status: ready=%u tx=%u drop=%u rx=%u",
				kaima_card_uart_ready ? 1u : 0u,
				(unsigned int)kaima_card_uart_tx_count,
				(unsigned int)kaima_card_uart_drop_count,
				(unsigned int)kaima_card_uart_rx_count);
			next_uart_ms = now_ms + KAIMA_CARD_UART_PERIOD_MS;
		}

		if (now_ms >= next_heartbeat_ms) {
			heartbeat_on = !heartbeat_on;
			next_heartbeat_ms = now_ms + KAIMA_CARD_HEARTBEAT_MS;
		}

		kaima_card_set_led(KAIMA_LED_SYSTEM, heartbeat_on);
		kaima_card_set_led(KAIMA_LED_SPI_TX, now_ms < kaima_card_pulse_until[KAIMA_LED_SPI_TX]);
		kaima_card_set_led(KAIMA_LED_SPI_RX, now_ms < kaima_card_pulse_until[KAIMA_LED_SPI_RX]);
		kaima_card_set_led(KAIMA_LED_STREAM, sensor_stream_enabled);
		kaima_card_set_led(KAIMA_LED_LOOPBACK, loopback_enabled);
		kaima_card_set_led(KAIMA_LED_MIC, kaima_card_mic_ok);
		kaima_card_set_led(KAIMA_LED_PIEZO, kaima_card_piezo_ok);
		kaima_card_set_led(KAIMA_LED_ANA10, kaima_card_ana10_high);
		kaima_card_set_led(KAIMA_LED_ANA14, kaima_card_ana14_high);
		kaima_card_set_led(KAIMA_LED_UART_TX, now_ms < kaima_card_pulse_until[KAIMA_LED_UART_TX]);
		kaima_card_set_led(KAIMA_LED_BLE0, kaima_card_ana10_low);
		kaima_card_set_led(KAIMA_LED_BLE1, kaima_card_ana14_low);

		k_msleep(20);
	}
}

K_THREAD_DEFINE(kaima_card_tid, KAIMA_CARD_STACK_SIZE, kaima_card_thread,
	NULL, NULL, NULL, KAIMA_CARD_PRIORITY, 0, 0);

#endif /* KAIMA_DEBUG_CARD */

#define FFT_SIZE 256u
#define FFT_BINS (FFT_SIZE / 2u)

static float fft_re[FFT_SIZE];
static float fft_im[FFT_SIZE];
static float fft_mag[FFT_BINS];

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
	int64_t range = (int64_t)max_capture - (int64_t)min_capture;
	int64_t scaled = (range > 0) ? (((int64_t)clamped - (int64_t)min_capture) * 255) / range : 127;
	return (uint8_t)clamp_i32(scaled, 0, 255);
}

static uint8_t map_u8_to_plot_x(uint8_t u8_value)
{
	/* 120 is the zero axis center. 0..119 represent negative, 121..240 positive. */
	int32_t centered = (int32_t)u8_value - 128;
	int32_t x = 120 + (centered * 120) / 127;
	return (uint8_t)clamp_i32(x, 0, 240);
}

static uint8_t scale_capture_to_plot(int32_t sample, int32_t min_capture, int32_t max_capture)
{
	int64_t range = (int64_t)max_capture - (int64_t)min_capture;
	int64_t scaled = (range > 0) ? (((int64_t)sample - (int64_t)min_capture) * 240) / range : 120;
	return (uint8_t)clamp_i32((int32_t)scaled, 0, 240);
}

static void scale_capture_ring_snapshot(const int32_t *capture,
					uint8_t *cached_raw,
					uint8_t *cached_plot,
					uint8_t *raw,
					uint8_t *plot,
					int32_t fixed_min,
					int32_t fixed_max,
					int32_t autoscale_min_span)
{
	int32_t min_capture = fixed_min;
	int32_t max_capture = fixed_max;

	if (sensor_auto_scale_enabled) {
		min_capture = capture[0];
		max_capture = capture[0];
		for (uint16_t i = 1; i < SENSOR_RING_LEN; i++) {
			if (capture[i] < min_capture) {
				min_capture = capture[i];
			}
			if (capture[i] > max_capture) {
				max_capture = capture[i];
			}
		}

		if (autoscale_min_span > 0 && (max_capture - min_capture) < autoscale_min_span) {
			int32_t center = min_capture + ((max_capture - min_capture) / 2);
			int32_t half_span = autoscale_min_span / 2;

			min_capture = center - half_span;
			max_capture = min_capture + autoscale_min_span;

			if (min_capture < fixed_min) {
				min_capture = fixed_min;
				max_capture = fixed_min + autoscale_min_span;
			}
			if (max_capture > fixed_max) {
				max_capture = fixed_max;
				min_capture = fixed_max - autoscale_min_span;
			}
		}
	}

	for (uint16_t i = 0; i < SENSOR_RING_LEN; i++) {
		cached_raw[i] = scale_capture_to_u8(capture[i], min_capture, max_capture);
		cached_plot[i] = scale_capture_to_plot(capture[i], min_capture, max_capture);
	}

	memcpy(raw, cached_raw, SENSOR_RING_LEN);
	memcpy(plot, cached_plot, SENSOR_RING_LEN);
}

static void mic_store_sample_locked(int32_t sample)
{
	mic_ring_capture[mic_write_index] = sample;
	mic_write_index = (uint16_t)((mic_write_index + 1u) % SENSOR_RING_LEN);
}

static void mic_init_ring_centered(void)
{
	k_mutex_lock(&mic_ring_lock, K_FOREVER);
	memset(mic_ring_capture, 0, sizeof(mic_ring_capture));
	memset(mic_ring_raw, 127, sizeof(mic_ring_raw));
	memset(mic_ring_plot, 120, sizeof(mic_ring_plot));
	mic_write_index = 0;
	k_mutex_unlock(&mic_ring_lock);
}

static void mic_copy_ring_snapshot(uint8_t *raw, uint8_t *plot, uint16_t *write_index)
{
	k_mutex_lock(&mic_ring_lock, K_FOREVER);
	scale_capture_ring_snapshot(mic_ring_capture, mic_ring_raw, mic_ring_plot, raw, plot,
				    -8388608, 8388607, 0);
	*write_index = mic_write_index;
	k_mutex_unlock(&mic_ring_lock);
}

static void mic_capture_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	const struct device *tdm_dev = MIC_TDM_DEV;

	mic_init_ring_centered();

	if (!device_is_ready(tdm_dev)) {
		LOG_ERR("TDM device not ready; sensor 0 will stay centered");
		return;
	}

	struct i2s_config cfg = {
		.word_size = MIC_BIT_DEPTH,
		.channels = MIC_CHANNELS,
		.format = I2S_FMT_DATA_FORMAT_I2S,
		.options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER,
		.frame_clk_freq = MIC_SAMPLE_RATE,
		.mem_slab = &mic_rx_slab,
		.block_size = MIC_BLOCK_SIZE,
		.timeout = 2000,
	};

	int ret = i2s_configure(tdm_dev, I2S_DIR_RX, &cfg);
	if (ret < 0) {
		LOG_ERR("i2s_configure RX failed: %d", ret);
#ifdef KAIMA_DEBUG_CARD
		kaima_card_mic_ok = false;
#endif
		return;
	}

	ret = i2s_trigger(tdm_dev, I2S_DIR_RX, I2S_TRIGGER_START);
	if (ret < 0) {
		LOG_ERR("i2s_trigger START failed: %d", ret);
#ifdef KAIMA_DEBUG_CARD
		kaima_card_mic_ok = false;
#endif
		return;
	}

#ifdef KAIMA_DEBUG_CARD
	kaima_card_mic_ok = true;
#endif

	LOG_INF("ICS43434 capture started: %u Hz, %u-bit, %u channels, block=%u bytes",
		(unsigned int)MIC_SAMPLE_RATE,
		(unsigned int)MIC_BIT_DEPTH,
		(unsigned int)MIC_CHANNELS,
		(unsigned int)MIC_BLOCK_SIZE);

	while (1) {
		void *rx_block;
		size_t rx_size;

		ret = i2s_read(tdm_dev, &rx_block, &rx_size);
		if (ret < 0) {
			LOG_ERR("i2s_read error: %d", ret);
#ifdef KAIMA_DEBUG_CARD
			kaima_card_mic_ok = false;
#endif
			k_msleep(10);
			continue;
		}
#ifdef KAIMA_DEBUG_CARD
		kaima_card_mic_ok = true;
#endif

		int32_t *samples = (int32_t *)rx_block;
		uint32_t sample_count = (uint32_t)(rx_size / sizeof(int32_t));

		k_mutex_lock(&mic_ring_lock, K_FOREVER);
		for (uint32_t i = 0; i < sample_count; i += MIC_CHANNELS) {
			mic_store_sample_locked(samples[i] >> 8);
		}
		k_mutex_unlock(&mic_ring_lock);

		k_mem_slab_free(&mic_rx_slab, rx_block);
	}
}

K_THREAD_DEFINE(mic_capture_tid, MIC_THREAD_STACK_SIZE, mic_capture_thread,
	NULL, NULL, NULL, MIC_THREAD_PRIORITY, 0, 0);

static void piezo_store_sample_locked(int32_t sample)
{
	piezo_ring_capture[piezo_write_index] = sample;
	piezo_write_index = (uint16_t)((piezo_write_index + 1u) % SENSOR_RING_LEN);
}

static void piezo_init_ring_centered(void)
{
	k_mutex_lock(&piezo_ring_lock, K_FOREVER);
	memset(piezo_ring_capture, 0, sizeof(piezo_ring_capture));
	memset(piezo_ring_raw, 127, sizeof(piezo_ring_raw));
	memset(piezo_ring_plot, 120, sizeof(piezo_ring_plot));
	piezo_write_index = 0;
	k_mutex_unlock(&piezo_ring_lock);
}

static void piezo_copy_ring_snapshot(uint8_t *raw, uint8_t *plot, uint16_t *write_index)
{
	k_mutex_lock(&piezo_ring_lock, K_FOREVER);
	scale_capture_ring_snapshot(piezo_ring_capture, piezo_ring_raw, piezo_ring_plot, raw, plot,
				    0, (1 << PIEZO_ADC_RESOLUTION) - 1, 0);
	*write_index = piezo_write_index;
	k_mutex_unlock(&piezo_ring_lock);
}

static void piezo_adc_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	const struct device *adc_dev = PIEZO_ADC_DEV;

	piezo_init_ring_centered();

	if (!device_is_ready(adc_dev)) {
		LOG_ERR("ADC device not ready; sensor 1 will stay centered");
		return;
	}

	struct adc_channel_cfg ch_cfg = {
		.gain = ADC_GAIN_1_4,
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME_DEFAULT,
		.channel_id = PIEZO_ADC_CHANNEL,
		.input_positive = NRF_SAADC_AIN6,
	};
	struct adc_sequence sequence = {
		.channels = BIT(PIEZO_ADC_CHANNEL),
		.buffer = &piezo_sample_buffer,
		.buffer_size = sizeof(piezo_sample_buffer),
		.resolution = PIEZO_ADC_RESOLUTION,
	};

	int ret = adc_channel_setup(adc_dev, &ch_cfg);
	if (ret < 0) {
		LOG_ERR("adc_channel_setup failed: %d", ret);
#ifdef KAIMA_DEBUG_CARD
		kaima_card_piezo_ok = false;
#endif
		return;
	}

	LOG_INF("Piezo ADC capture started: channel=%u input=AIN6/P1.04 resolution=%u period=%u ms",
		(unsigned int)PIEZO_ADC_CHANNEL,
		(unsigned int)PIEZO_ADC_RESOLUTION,
		(unsigned int)PIEZO_ADC_SAMPLE_PERIOD_MS);

	while (1) {
		k_mutex_lock(&adc_lock, K_FOREVER);
		ret = adc_read(adc_dev, &sequence);
		k_mutex_unlock(&adc_lock);
		if (ret < 0) {
			LOG_ERR("adc_read failed: %d", ret);
#ifdef KAIMA_DEBUG_CARD
			kaima_card_piezo_ok = false;
#endif
			k_msleep(PIEZO_ADC_SAMPLE_PERIOD_MS);
			continue;
		}
#ifdef KAIMA_DEBUG_CARD
		kaima_card_piezo_ok = true;
#endif

		k_mutex_lock(&piezo_ring_lock, K_FOREVER);
		piezo_store_sample_locked((int32_t)piezo_sample_buffer);
		k_mutex_unlock(&piezo_ring_lock);

		k_msleep(PIEZO_ADC_SAMPLE_PERIOD_MS);
	}
}

K_THREAD_DEFINE(piezo_adc_tid, PIEZO_ADC_THREAD_STACK_SIZE, piezo_adc_thread,
	NULL, NULL, NULL, PIEZO_ADC_THREAD_PRIORITY, 0, 0);

static void ana31_store_sample_locked(int32_t sample)
{
	ana31_ring_capture[ana31_write_index] = sample;
	ana31_write_index = (uint16_t)((ana31_write_index + 1u) % SENSOR_RING_LEN);
}

static void ana31_init_ring_centered(void)
{
	k_mutex_lock(&ana31_ring_lock, K_FOREVER);
	memset(ana31_ring_capture, 0, sizeof(ana31_ring_capture));
	memset(ana31_ring_raw, 127, sizeof(ana31_ring_raw));
	memset(ana31_ring_plot, 120, sizeof(ana31_ring_plot));
	ana31_write_index = 0;
	k_mutex_unlock(&ana31_ring_lock);
}

static void ana31_copy_ring_snapshot(uint8_t *raw, uint8_t *plot, uint16_t *write_index)
{
	k_mutex_lock(&ana31_ring_lock, K_FOREVER);
	scale_capture_ring_snapshot(ana31_ring_capture, ana31_ring_raw, ana31_ring_plot, raw, plot,
				    ANA31_ELECTRET_FIXED_MIN_RAW, ANA31_ELECTRET_FIXED_MAX_RAW,
				    ANA31_ELECTRET_AUTOSCALE_MIN_SPAN_RAW);
	*write_index = ana31_write_index;
	k_mutex_unlock(&ana31_ring_lock);
}

static void ana31_adc_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	const struct device *adc_dev = PIEZO_ADC_DEV;

	ana31_init_ring_centered();

	if (!device_is_ready(adc_dev)) {
		LOG_ERR("ADC device not ready; sensor 2 will stay centered");
		return;
	}

	struct adc_channel_cfg ch_cfg = {
		.gain = ADC_GAIN_1_4,
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME_DEFAULT,
		.channel_id = ANA31_ADC_CHANNEL,
		.input_positive = NRF_SAADC_AIN1,
	};
	struct adc_sequence sequence = {
		.channels = BIT(ANA31_ADC_CHANNEL),
		.buffer = &ana31_sample_buffer,
		.buffer_size = sizeof(ana31_sample_buffer),
		.resolution = ANA31_ADC_RESOLUTION,
	};

	k_mutex_lock(&adc_lock, K_FOREVER);
	int ret = adc_channel_setup(adc_dev, &ch_cfg);
	k_mutex_unlock(&adc_lock);
	if (ret < 0) {
		LOG_ERR("ana31 adc_channel_setup failed: %d", ret);
		return;
	}

	LOG_INF("ANA31 ADC capture started: channel=%u input=AIN1/P1.31 resolution=%u period=%u ms",
		(unsigned int)ANA31_ADC_CHANNEL,
		(unsigned int)ANA31_ADC_RESOLUTION,
		(unsigned int)ANA31_ADC_SAMPLE_PERIOD_MS);

	while (1) {
		k_mutex_lock(&adc_lock, K_FOREVER);
		ret = adc_read(adc_dev, &sequence);
		k_mutex_unlock(&adc_lock);
		if (ret < 0) {
			LOG_ERR("ana31 adc_read failed: %d", ret);
			k_msleep(ANA31_ADC_SAMPLE_PERIOD_MS);
			continue;
		}

		k_mutex_lock(&ana31_ring_lock, K_FOREVER);
		ana31_store_sample_locked((int32_t)ana31_sample_buffer);
		k_mutex_unlock(&ana31_ring_lock);

		k_msleep(ANA31_ADC_SAMPLE_PERIOD_MS);
	}
}

K_THREAD_DEFINE(ana31_adc_tid, ANA31_ADC_THREAD_STACK_SIZE, ana31_adc_thread,
	NULL, NULL, NULL, ANA31_ADC_THREAD_PRIORITY, 0, 0);

static int32_t sensor_sim_capture(uint8_t sensor_id, uint32_t sample_index)
{
	float t = (float)sample_index;

	switch (sensor_id) {
	case 1: {
		/* 10-bit triangular wave: [-512 .. +511]. */
		int32_t phase = (int32_t)(sample_index % 160u);
		int32_t tri = (phase < 80) ? (phase * 13 - 520) : ((159 - phase) * 13 - 520);
		return clamp_i32(tri, -512, 511);
	}
	case 2: {
		/* 16-bit-like mixed signal: [-32768 .. +32767]. */
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
		/* 12-bit sine: [-2048 .. +2047]. */
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
	sensor_phase_3 = 0.0f;
	sensor_phase_4 = 0.0f;
	memset(sensor_ring_raw, 127, sizeof(sensor_ring_raw));
	memset(sensor_ring_plot, 120, sizeof(sensor_ring_plot));
	sensor_write_index = 0;
	sensor_next_fill_ms = 0;
}

/* In-place radix-2 Cooley-Tukey DIT FFT.  n must be a power of two. */
static void fft_radix2(float *re, float *im, uint16_t n)
{
	/* Bit-reversal permutation */
	for (uint16_t i = 1, j = 0; i < n; i++) {
		uint16_t bit = n >> 1;
		for (; j & bit; bit >>= 1)
			j ^= bit;
		j ^= bit;
		if (i < j) {
			float t;
			t = re[i]; re[i] = re[j]; re[j] = t;
			t = im[i]; im[i] = im[j]; im[j] = t;
		}
	}
	/* Butterfly stages */
	for (uint16_t len = 2; len <= n; len <<= 1) {
		float ang = -TWO_PI / (float)len;
		float w_re = cosf(ang), w_im = sinf(ang);
		for (uint16_t i = 0; i < n; i += len) {
			float tw_re = 1.0f, tw_im = 0.0f;
			for (uint16_t k = 0; k < (len >> 1u); k++) {
				uint16_t a = i + k;
				uint16_t b = i + k + (len >> 1u);
				float u_re = re[a], u_im = im[a];
				float v_re = re[b] * tw_re - im[b] * tw_im;
				float v_im = re[b] * tw_im + im[b] * tw_re;
				re[a] = u_re + v_re;
				im[a] = u_im + v_im;
				re[b] = u_re - v_re;
				im[b] = u_im - v_im;
				float new_tw = tw_re * w_re - tw_im * w_im;
				tw_im = tw_re * w_im + tw_im * w_re;
				tw_re = new_tw;
			}
		}
	}
}

static void sensor_fill_ring_once(void)
{
	if (selected_sensor == 0u && !sensor_fft_enabled) {
		mic_copy_ring_snapshot(sensor_ring_raw, sensor_ring_plot, &sensor_write_index);
		return;
	}
	if (selected_sensor == 1u && !sensor_fft_enabled) {
		piezo_copy_ring_snapshot(sensor_ring_raw, sensor_ring_plot, &sensor_write_index);
		return;
	}
	if (selected_sensor == 2u && !sensor_fft_enabled) {
		ana31_copy_ring_snapshot(sensor_ring_raw, sensor_ring_plot, &sensor_write_index);
		return;
	}

	if (sensor_fft_enabled) {
		/* FFT path: Hanning-windowed 256-point FFT → magnitude spectrum */
		if (selected_sensor == 0u || selected_sensor == 1u || selected_sensor == 2u) {
			uint16_t write_index;
			if (selected_sensor == 0u) {
				mic_copy_ring_snapshot(sensor_ring_raw, sensor_ring_plot, &write_index);
			} else if (selected_sensor == 1u) {
				piezo_copy_ring_snapshot(sensor_ring_raw, sensor_ring_plot, &write_index);
			} else {
				ana31_copy_ring_snapshot(sensor_ring_raw, sensor_ring_plot, &write_index);
			}
			for (uint16_t i = 0; i < FFT_SIZE; i++) {
				uint16_t idx = (uint16_t)((write_index + i) % SENSOR_RING_LEN);
				int32_t raw = ((int32_t)sensor_ring_raw[idx] - 128) * 256;
				float w = 0.5f * (1.0f - cosf(TWO_PI * (float)i / (float)(FFT_SIZE - 1u)));
				fft_re[i] = (float)raw * w;
				fft_im[i] = 0.0f;
			}
		} else {
			for (uint16_t i = 0; i < FFT_SIZE; i++) {
				int32_t raw = sensor_sim_capture(selected_sensor, sensor_sample_counter++);
				float w = 0.5f * (1.0f - cosf(TWO_PI * (float)i / (float)(FFT_SIZE - 1u)));
				fft_re[i] = (float)raw * w;
				fft_im[i] = 0.0f;
			}
		}

		fft_radix2(fft_re, fft_im, FFT_SIZE);

		float max_mag = 1.0f;
		for (uint16_t b = 0; b < FFT_BINS; b++) {
			float r = fft_re[b + 1u], c = fft_im[b + 1u];
			fft_mag[b] = sqrtf(r * r + c * c);
			if (fft_mag[b] > max_mag) {
				max_mag = fft_mag[b];
			}
		}

		for (uint16_t i = 0; i < SENSOR_RING_LEN; i++) {
			uint16_t bin = (uint16_t)((uint32_t)i * FFT_BINS / SENSOR_RING_LEN);
			float norm = fft_mag[bin] / max_mag;
			sensor_ring_raw[i]  = (uint8_t)(norm * 255.0f);
			sensor_ring_plot[i] = (uint8_t)(norm * 240.0f);
		}
		sensor_write_index = 0u;
	} else {
		/* Raw waveform path: time-domain samples, centered at 120 */
		for (uint16_t i = 0; i < SENSOR_RING_LEN; i++) {
			int32_t capture = sensor_sim_capture(selected_sensor, sensor_sample_counter++);
			uint8_t scaled_u8;
			switch (selected_sensor) {
			case 1: scaled_u8 = scale_capture_to_u8(capture, -512, 511); break;
			case 2: scaled_u8 = scale_capture_to_u8(capture, -32768, 32767); break;
			case 0:
			default: scaled_u8 = scale_capture_to_u8(capture, -2048, 2047); break;
			}
			sensor_ring_raw[sensor_write_index]  = scaled_u8;
			sensor_ring_plot[sensor_write_index] = map_u8_to_plot_x(scaled_u8);
			sensor_write_index = (uint16_t)((sensor_write_index + 1u) % SENSOR_RING_LEN);
		}
	}
}

static void process_command_frame(const uint8_t *rx, size_t len)
{
	if (!rx || len == 0u) {
		return;
	}
	if (loopback_enabled && rx[0] != CMD_LOOPBACK_MODE) {
		return;
	}

	switch (rx[0]) {
	case CMD_LOOPBACK_MODE:
		loopback_enabled = (len >= 2u) ? (rx[1] != 0u) : false;
		if (loopback_enabled) {
			sensor_stream_enabled = false;
		}
		break;
	case CMD_STREAM_START:
		loopback_enabled = false;
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
		loopback_enabled = false;
		if (len >= 2u) {
			apply_sensor_selection(rx[1]);
		}
		break;
	case CMD_STREAM_FETCH:
		loopback_enabled = false;
		/* Keep sensor selection sticky during streaming: master can resend desired sensor in every fetch. */
		if (len >= 2u) {
			apply_sensor_selection(rx[1]);
		}
		break;
	case CMD_SET_MODE:
		loopback_enabled = false;
		sensor_fft_enabled = (len >= 2u) ? (rx[1] != 0u) : false;
		sensor_next_fill_ms = 0;
		break;
	case CMD_AUTO_SCALE:
		loopback_enabled = false;
		sensor_auto_scale_enabled = (len >= 2u) ? (rx[1] != 0u) : true;
		sensor_next_fill_ms = 0;
		break;
	default:
		break;
	}
}

static void prepare_loopback_response(void)
{
	memcpy(tx_buffer, rx_buffer, sizeof(tx_buffer));
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
	loopback_enabled = false;
	sensor_auto_scale_enabled = true;
	sensor_sequence = 0;
	selected_sensor = 0;
	sensor_write_index = 0;
	sensor_sample_counter = 0;
	sensor_next_fill_ms = 0;
	sensor_phase_0 = 0.0f;
	sensor_phase_1 = 0.0f;
	sensor_phase_2 = 0.0f;
	sensor_phase_3 = 0.0f;
	sensor_phase_4 = 0.0f;
	prepare_tx_packet();

	LOG_INF("BM20_C SPIS scope ready (production, packet=%u bytes, samples=%u, Ts=%u ms, loopback cmd=0x%02X, autoscale cmd=0x%02X)",
		(unsigned int)SENSOR_PACKET_BYTES,
		(unsigned int)SENSOR_RING_LEN,
		(unsigned int)SENSOR_TS_MS,
		(unsigned int)CMD_LOOPBACK_MODE,
		(unsigned int)CMD_AUTO_SCALE);

	uint32_t transfer_count = 0;
	while (1) {
		int ret = spi_transceive(spis_dev, &spis_config, &tx_set, &rx_set);
		if (ret < 0) {
			LOG_ERR("spi_transceive failed: %d", ret);
			k_msleep(10);
			continue;
		}
#ifdef KAIMA_DEBUG_CARD
		kaima_card_pulse(KAIMA_LED_SPI_RX);
#endif

		process_command_frame(rx_buffer, sizeof(rx_buffer));
		if (loopback_enabled) {
			prepare_loopback_response();
		} else {
			prepare_tx_packet();
		}
#ifdef KAIMA_DEBUG_CARD
		kaima_card_pulse(KAIMA_LED_SPI_TX);
#endif

		transfer_count++;
		if ((transfer_count % 500u) == 0u) {
			LOG_INF("xfer=%u mode=%s stream=%u autoscale=%u seq=%u sensor=%u write=%u",
				(unsigned int)transfer_count,
				loopback_enabled ? "loopback" : "scope",
				sensor_stream_enabled ? 1u : 0u,
				sensor_auto_scale_enabled ? 1u : 0u,
				(unsigned int)sensor_sequence,
				(unsigned int)selected_sensor,
				(unsigned int)sensor_write_index);
		}
	}

	return 0;
}

#endif /* NRF_BENCH_MODE */
