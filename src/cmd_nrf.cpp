#include "cmd_nrf.h"
#include "command.h"
#include "cmd_lcd.h"
#include "cmd_cam.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
//
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
//
#include "DEV_Config.h"
#include "LCD_2in.h"
#include "GUI_Paint.h"
#include "cam.h"

// ─── Global definitions ───────────────────────────────────────────────────────

bool nrf_spi_initialized = false;           // True after nrf_spi_prepare_bus() succeeds at least once
uint32_t nrf_spi_actual_hz = 0;             // Baud rate actually achieved by spi_init() for NRF transfers
// Pico SPI0 pins shared with the LCD bus.  Wire each to the nRF54L15 SPIS00 dedicated P2 pin shown:
//   Pico GPIO22 (I2C1_SDA) → nRF P2.10  CSN
//   Pico GPIO18 (LCD_CLK)  → nRF P2.06  SCK   (dedicated HSSPI clock pin, up to 32 MHz)
//   Pico GPIO19 (LCD_MOSI) → nRF P2.09  SDO   (nRF SDO = data out of nRF = Pico MOSI)
//   Pico GPIO20 (LCD_RST)  → nRF P2.08  SDI   (nRF SDI = data into nRF = Pico MISO echo)
// SPIS00 (P2 port) supports up to 32 MHz; SPIS20/21/30 (P0/P1) are limited to 8 MHz.
const uint nrf_spi_cs_pin = I2C1_SDA;       // GPIO22 → nRF P2.10 CSN
const uint nrf_spi_sck_pin = LCD_CLK_PIN;   // GPIO18 → nRF P2.06 SCK
const uint nrf_spi_mosi_pin = LCD_MOSI_PIN; // GPIO19 → nRF P2.09 SDO
const uint nrf_spi_miso_pin = LCD_RST_PIN;  // GPIO20 → nRF P2.08 SDI (echo path back to Pico)
uint32_t nrf_spi_cs_setup_us = 15;          // CS-assert-to-first-SCK delay; tune down to 2 µs with `nrf timing setup 2` for HSSPI
uint32_t nrf_spi_cs_hold_us = 4;            // Last-SCK-to-CS-deassert hold; tune down to 2 µs with `nrf timing hold 2` for HSSPI
uint32_t nrf_spi_frame_gap_us = 200;        // Inter-frame idle gap; tune down to 50 µs with `nrf timing gap 50` for HSSPI
const uint32_t nrf_event_min_interval_ms = 250;  // Debounce window: ignores NRF button events closer than this
const uint32_t mode_b_fail_block_ms = 1200;      // After a Mode-B SPI failure, suppress NRF events for this duration
const uint32_t mode_b_retry_safe_hz = 16000000;  // HSSPI safe-fallback baud for one-time all-0xFF recovery retry (was 6 MHz for SPIS20; raised to 16 MHz for SPIS00)
uint32_t nrf_validation_rows_per_chunk = 4;      // Number of camera rows transferred per NRF SPI chunk (tunable)
uint8_t nrf_validation_rx_chunk[LCD_2IN_WIDTH * 2 * NRF_VALIDATION_ROWS_MAX];   // Receive buffer for one NRF chunk
uint8_t nrf_validation_dummy_chunk[LCD_2IN_WIDTH * 2 * NRF_VALIDATION_ROWS_MAX]; // All-zero payload for the Phase-B flush transfer
mode_b_transfer_ctx_t mode_b_ctx = {0};
const uint8_t *mode_b_stream_locked_frame = NULL;  // Frame pointer locked at the start of a Mode-B transfer; prevents mid-frame pointer flip when camera DMA swaps buffers
int spi_tx_dma_ch = -1;             // DMA channel for SPI TX (claimed on first use; -1 = not yet claimed)
int spi_rx_dma_ch = -1;             // DMA channel for SPI RX drain (claimed on first use; -1 = not yet claimed)
volatile uint16_t spi_rx_dummy;     // Discard sink for SPI RX DMA; prevents the 4-deep RX FIFO overflowing and stalling TX

// ─── NRF FFT / scope-stream globals ──────────────────────────────────────────
// (These are internal to cmd_nrf.cpp; only the public API symbols appear in cmd_nrf.h)

static bool nrf_fft_stream_active = false;                // True while RP master periodically polls scope packets and redraws the LCD graph
static bool nrf_fft_drop_first_packet = false;            // True immediately after START to ignore one stale pipelined slave response
static bool nrf_fft_period_override = false;              // True when user forced period via command instead of Ts+1ms
static uint32_t nrf_fft_poll_period_ms = 41;              // Graph refresh period in ms (defaults to Ts+1)
static absolute_time_t nrf_fft_next_poll_time;            // Next time a FETCH command should be sent
static uint8_t nrf_fft_selected_sensor = 0;               // Sensor selected on BM20_C (0..2)
static uint8_t nrf_fft_last_seq = 0;                      // Last accepted scope packet sequence number
static uint8_t nrf_fft_last_sensor = 0;                   // Sensor ID reported by the nRF in the latest valid packet
static uint16_t nrf_fft_last_ts_ms = 0;                   // Last Ts reported by the nRF
static uint16_t nrf_fft_last_write_index = 0;             // Circular-buffer write index echoed by the nRF
static uint32_t nrf_fft_frames_ok = 0;                    // Count of valid scope frames received since stream start
static uint32_t nrf_fft_frames_bad = 0;                   // Count of invalid/corrupt scope frames since stream start
static uint32_t nrf_fft_bad_magic = 0;                    // Bad frames: magic bytes wrong (byte 0 or 1)
static uint32_t nrf_fft_bad_version = 0;                  // Bad frames: version byte wrong (byte 2)
static uint32_t nrf_fft_bad_count = 0;                    // Bad frames: sample_count field wrong (bytes 8-9)
static uint32_t nrf_fft_bad_cksum = 0;                    // Bad frames: XOR checksum not zero
static uint32_t nrf_fft_bad_stream_off = 0;               // Bad frames: stream_enabled bit (byte 3 bit0) is 0
static uint32_t nrf_fft_bad_spi = 0;                      // Bad frames: SPI transfer itself failed
static uint8_t  nrf_fft_last_bad_hdr[NRF_SCOPE_HEADER_BYTES]; // First 13 bytes of the last failed frame
static uint32_t nrf_fft_sensor_mismatch_frames = 0;       // Consecutive valid frames whose reported sensor != requested sensor
static uint32_t nrf_fft_resync_count = 0;                 // Number of automatic START re-sync attempts issued by RP
static bool nrf_stream_fft_enabled = false;               // When false, nRF sends raw waveform; when true, nRF computes FFT
static bool nrf_stream_auto_scale_enabled = true;          // When true, hardware sensors scale each 320-sample window by min/max
static uint8_t nrf_fft_last_bins[NRF_SCOPE_SAMPLE_COUNT]; // Latest mapped X points (0..240), one per LCD row
static uint8_t nrf_fft_tx_frame[NRF_SCOPE_PACKET_BYTES];  // RP->nRF control frame (START/STOP/FETCH/SENSOR)
static uint8_t nrf_fft_rx_frame[NRF_SCOPE_PACKET_BYTES];  // nRF->RP scope packet received on each SPI transaction

// ─── Forward declarations ─────────────────────────────────────────────────────

static void nrf_fft_stream_task(void);
static void nrf_fft_stop_internal(bool notify_slave, bool clear_screen);
static void nrf_fft_render_graph(void);

// ─── NRF SPI bus functions ────────────────────────────────────────────────────

/**
 * @brief Configure the shared SPI bus for NRF communication at the requested baud rate.
 *
 * Reclaims LCD_CLK/MOSI/MISO/RST pins as SPI, configures I2C1_SDA as the NRF CS
 * GPIO output (deasserted), and initialises SPI at Mode 0. The LCD must not be
 * accessed while NRF owns these pins.
 */
bool nrf_spi_prepare_bus(uint32_t requested_hz) {
    if (requested_hz == 0) requested_hz = 15000000;  // Guard: caller passed zero; default to 15 MHz (max confirmed reliable)

    // Keep the LCD chip-select deasserted while NRF owns the bus.
    DEV_Digital_Write(LCD_CS_PIN, 1);

    // nrf_spi_cs_pin == I2C1_SDA.  This pin is normally pulled up as I2C SDA.
    // Reinitialise it as a plain GPIO output so we can drive NRF CS manually.
    gpio_init(nrf_spi_cs_pin);
    gpio_set_dir(nrf_spi_cs_pin, GPIO_OUT);
    gpio_put(nrf_spi_cs_pin, 1);  // Deasserted (active-low CS)

    // Reclaim the shared LCD/NRF pins as SPI functions.
    // LCD driver uses these same pins; they must be restored by shared_enter_lcd_active() before any LCD command.
    gpio_set_function(nrf_spi_sck_pin, GPIO_FUNC_SPI);
    gpio_set_function(nrf_spi_mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(nrf_spi_miso_pin, GPIO_FUNC_SPI);

    // Re-init SPI at the requested baud; spi_init() returns the actual achieved rate.
    nrf_spi_actual_hz = spi_init(SPI_PORT, requested_hz);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
    nrf_spi_initialized = true;
    shared_pin_state_set_nrf_transaction();  // Notify command.cpp that NRF now owns the shared pins
    return true;
}

/**
 * @brief Discard any leftover bytes in the SPI RX FIFO before starting a new transfer.
 *
 * Stale bytes from a previous transaction would corrupt the RX data of the next transfer
 * if not cleared first.
 */
void nrf_spi_drain_rx_fifo(void) {    while (spi_is_readable(SPI_PORT)) {
        (void)spi_get_hw(SPI_PORT)->dr;
    }
}

/**
 * @brief Perform a full-duplex SPI transaction with the NRF, including CS timing and bus ownership.
 *
 * Sequence: take bus ownership → drain stale RX bytes → assert CS → transfer → hold CS → deassert CS → inter-frame gap.
 * CS setup/hold/gap times are tunable via nrf_spi_cs_setup_us / cs_hold_us / frame_gap_us.
 *
 * @param tx  Bytes to send.
 * @param rx  Destination for bytes received during the same transfer.
 * @param len Number of bytes to exchange.
 * @return true if spi_write_read_blocking returned exactly len bytes, false otherwise.
 */
bool nrf_spi_transfer_bytes(const uint8_t *tx, uint8_t *rx, size_t len) {
    if (!tx || !rx || len == 0) return false;

    // 1) Take shared bus ownership for NRF transfer.
    shared_enter_nrf_transaction();
    // 2) Drop stale RX bytes from prior transfers.
    nrf_spi_drain_rx_fifo();
    // 3) Assert CS and honor setup timing.
    gpio_put(nrf_spi_cs_pin, 0);
    busy_wait_us_32(nrf_spi_cs_setup_us);

    // 4) Perform full-duplex SPI transaction.
    int ret = spi_write_read_blocking(SPI_PORT, tx, rx, len);

    // 5) Hold CS then release and keep inter-frame gap.
    //    Apply the same floor as the bench: 400 µs + 2 µs/byte covers nRF SPIS re-arm
    //    overhead regardless of frame size. nrf_spi_frame_gap_us is honoured if larger.
    uint32_t min_gap = 400u + (uint32_t)(len * 2u);
    uint32_t gap = (nrf_spi_frame_gap_us > min_gap) ? nrf_spi_frame_gap_us : min_gap;
    busy_wait_us_32(nrf_spi_cs_hold_us);
    gpio_put(nrf_spi_cs_pin, 1);
    busy_wait_us_32(gap);
    return ret >= 0 && (size_t)ret == len;
}

/**
 * @brief Check that lcd_image can be safely used as the Mode-B frame assembly buffer.
 */
bool ensure_mode_b_frame_raw(size_t needed_bytes) {
    return (lcd_image != NULL) && (needed_bytes != 0) && (needed_bytes <= lcd_image_bytes);
}

/**
 * @brief Mark the Mode-B transfer context inactive so the buffer is no longer considered live.
 */
void free_mode_b_frame_raw(void) {    mode_b_ctx.active = false;
}

/**
 * @brief Return true only if every byte in buf equals value.
 *
 * Used to detect all-0xFF NRF responses, which indicate a disconnected MISO line
 * or a slave that hasn't asserted CS yet (SPI bus floating high).
 */
bool bytes_all_value(const uint8_t *buf, size_t len, uint8_t value) {    for (size_t i = 0; i < len; ++i) {
        if (buf[i] != value) return false;
    }
    return true;
}

/**
 * @brief Clear all fields of mode_b_ctx to their initial values so the next Mode-B frame starts from a known baseline.
 */
void reset_mode_b_transfer_ctx(void) {
    // Reset every field so the next mode-B frame starts from a known baseline.
    mode_b_ctx.active = false;
    mode_b_ctx.frame = NULL;
    mode_b_ctx.first_transfer = true;
    mode_b_ctx.row = 0;
    mode_b_ctx.prev_row = 0;
    mode_b_ctx.prev_row_count = 0;
    mode_b_ctx.prev_byte_count = 0;
    mode_b_ctx.prev_src = NULL;
    memset(mode_b_ctx.prev_tx_fingerprint, 0, sizeof(mode_b_ctx.prev_tx_fingerprint));
}

// ─── NRF command handlers ─────────────────────────────────────────────────────

/**
 * @brief Configure the shared SPI bus for NRF SPIS00 at the given baud rate (default 32 MHz → ~30 MHz actual on RP2350 @ 150 MHz) and print the pin mapping.
 */
void run_nrf_spi_init(size_t argc, const char *argv[]) {
    uint32_t requested_hz = 15000000;  // Confirmed max reliable via bench sweep (Mode 1, all frame sizes 1–4096 B pass 100%)
    if (argc > 1) {
        printf("Usage: nrf_spi_init [baud_hz]\n");
        return;
    }
    if (argc == 1 && !parse_u32_arg(argv[0], &requested_hz)) {
        printf("Invalid baud_hz: %s\n", argv[0]);
        return;
    }
    if (lcd_cam_stream_active) {
        printf("Stop lcd_cam_stream before nrf_spi_init to avoid live SPI reconfiguration.\n");
        return;
    }
    if (nrf_fft_stream_active) {
        printf("Stop nrf stream before nrf_spi_init.\n");
        return;
    }

    nrf_spi_prepare_bus(requested_hz);
    nrf_event_poll_next_time = get_absolute_time();
    printf("NRF SPI ready: SCK=%u MOSI=%u MISO=%u CS=%u actual=%lu Hz\n",
           nrf_spi_sck_pin, nrf_spi_mosi_pin, nrf_spi_miso_pin, nrf_spi_cs_pin,
           (unsigned long)nrf_spi_actual_hz);
    printf("Timing defaults: gap=%lu us  setup=%lu us  hold=%lu us  chunk=%lu rows\n",
           (unsigned long)nrf_spi_frame_gap_us, (unsigned long)nrf_spi_cs_setup_us,
           (unsigned long)nrf_spi_cs_hold_us, (unsigned long)nrf_validation_rows_per_chunk);
}

/**
 * @brief Print NRF SPI init state, pin mapping, and current actual baud rate.
 */
void run_nrf_spi_status(size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;
    printf("NRF SPI initialized: %s\n", nrf_spi_initialized ? "yes" : "no");
    printf("Pins: SCK=%u MOSI=%u MISO=%u CS=%u\n",
           nrf_spi_sck_pin, nrf_spi_mosi_pin, nrf_spi_miso_pin, nrf_spi_cs_pin);
    printf("Actual baud: %lu Hz\n", (unsigned long)nrf_spi_actual_hz);
}

static bool nrf_set_slave_loopback(bool enable) {
    if (!nrf_spi_initialized) {
        nrf_spi_prepare_bus(15000000);
    }

    uint8_t tx[2] = {0};
    uint8_t rx[2] = {0};
	tx[0] = NRF_SCOPE_CMD_LOOPBACK;
	tx[1] = enable ? 1u : 0u;

	return nrf_spi_transfer_bytes(tx, rx, sizeof(tx));
}

void run_nrf_loopback(size_t argc, const char *argv[]) {
    if (argc != 1 ||
        (strcmp(argv[0], "start") != 0 && strcmp(argv[0], "stop") != 0 &&
         strcmp(argv[0], "on") != 0 && strcmp(argv[0], "off") != 0)) {
        printf("Usage: nrf loopback <start|stop>\n");
        printf("  start: BM20_C echoes each received frame on the next SPI transaction\n");
        printf("  stop:  BM20_C returns to production scope packets for nrf stream\n");
        return;
    }
    if (!nrf_spi_initialized) {
        nrf_spi_prepare_bus(15000000);
    }
    if (nrf_fft_stream_active) {
        printf("Stop nrf stream before changing loopback mode.\n");
        return;
    }

    bool enable = (strcmp(argv[0], "start") == 0 || strcmp(argv[0], "on") == 0);

    if (!nrf_set_slave_loopback(enable)) {
        printf("Failed to send BM20_C loopback command.\n");
        return;
    }

    printf("BM20_C loopback %s. %s\n",
           enable ? "enabled" : "disabled",
           enable ? "Diagnostic SPI commands will echo previous frames until stopped." : "nrf stream can be used again.");
}

/**
 * @brief Perform one manual full-duplex SPI transfer with hex byte arguments and print the TX and RX bytes.
 */
void run_nrf_spi_xfer(size_t argc, const char *argv[]) {
    if (argc < 1 || argc > 256) {
        printf("Usage: nrf_spi_xfer <byte0_hex> [byte1_hex] ... [byte255_hex]\n");
        return;
    }
    if (!nrf_spi_initialized) {
        nrf_spi_prepare_bus(15000000);
    }

    uint8_t tx[256];
    uint8_t rx[256];
    size_t n = 0;
    if (!parse_hex_tokens_bytes(argc, argv, tx, sizeof(tx), &n)) {
        printf("Invalid byte list. Use hex bytes like: nrf_spi_xfer 9F 00 00 00\n");
        return;
    }

    if (!nrf_set_slave_loopback(true)) {
        printf("Failed to enable BM20_C loopback mode. Is the updated BM20_C firmware flashed?\n");
        return;
    }

    /* Zephyr SPIS replies with the buffer prepared before this CS assertion.
     * Prime once so production loopback mode can echo tx on the checked transfer. */
    nrf_spi_drain_rx_fifo();
    gpio_put(nrf_spi_cs_pin, 0);
    busy_wait_us_32(nrf_spi_cs_setup_us);  // Give slave CS setup time before first SCK edge.
    (void)spi_write_read_blocking(SPI_PORT, tx, rx, (size_t)n);
    busy_wait_us_32(nrf_spi_cs_hold_us);  // Hold CS a moment after last edge.
    gpio_put(nrf_spi_cs_pin, 1);
    busy_wait_us_32(nrf_spi_frame_gap_us);  // Small gap between frames.

    nrf_spi_drain_rx_fifo();
    gpio_put(nrf_spi_cs_pin, 0);
    busy_wait_us_32(nrf_spi_cs_setup_us);
    int ret = spi_write_read_blocking(SPI_PORT, tx, rx, (size_t)n);
    busy_wait_us_32(nrf_spi_cs_hold_us);  // Hold CS a moment after last edge.
    gpio_put(nrf_spi_cs_pin, 1);
    busy_wait_us_32(nrf_spi_frame_gap_us);  // Small gap between frames.

    (void)nrf_set_slave_loopback(false);

    if (ret < 0 || (size_t)ret != n) {
        printf("SPI transfer failed (%d)\n", ret);
        printf("FAILED\n");
        return;
    }

    printf("TX: ");
    print_hex_bytes(tx, n);
    printf("\nRX: ");
    print_hex_bytes(rx, n);
    printf("\n%s\n", memcmp(rx, tx, n) == 0 ? "PASSED" : "FAILED");
}

/**
 * @brief Sweep SPI baud rates from start_hz to stop_hz, measuring pass rate and throughput at each step against an expected echo pattern.
 */
void run_nrf_spi_sweep(size_t argc, const char *argv[]) {
    bool loopback_active = false;
    bool aborted = false;
    uint32_t restore_hz = nrf_spi_actual_hz;

    if (argc != 6) {
        printf("Usage: nrf_spi_sweep <start_hz> <stop_hz> <step_hz> <loops> <tx_hex> <expect_hex>\n");
        printf("Example: nrf_spi_sweep 1000000 32000000 1000000 200 A55A010203 5AA5AABBCC\n");
        return;
    }

    uint32_t start_hz = 0, stop_hz = 0, step_hz = 0, loops = 0;
    if (!parse_u32_arg(argv[0], &start_hz) ||
        !parse_u32_arg(argv[1], &stop_hz) ||
        !parse_u32_arg(argv[2], &step_hz) ||
        !parse_u32_arg(argv[3], &loops)) {
        printf("Invalid numeric argument\n");
        return;
    }

    if (start_hz == 0 || stop_hz < start_hz || step_hz == 0 || loops == 0) {
        printf("Invalid range/loops values\n");
        return;
    }

    uint8_t tx[256], expect[256], rx[256];
    size_t tx_len = 0, expect_len = 0;
    if (!parse_hex_string_bytes(argv[4], tx, sizeof(tx), &tx_len)) {
        printf("Invalid tx_hex string (must be even-length hex)\n");
        return;
    }
    if (!parse_hex_string_bytes(argv[5], expect, sizeof(expect), &expect_len)) {
        printf("Invalid expect_hex string (must be even-length hex)\n");
        return;
    }
    if (tx_len != expect_len) {
        printf("tx_hex and expect_hex must have same byte length\n");
        return;
    }
    printf("Sweeping SPI speed from %lu to %lu Hz step %lu, loops=%lu, frame=%u bytes\n",
           (unsigned long)start_hz, (unsigned long)stop_hz, (unsigned long)step_hz,
           (unsigned long)loops, (unsigned)tx_len);
    printf("Using CS setup/hold/gap: %lu/%lu/%lu us\n",
           (unsigned long)nrf_spi_cs_setup_us,
           (unsigned long)nrf_spi_cs_hold_us,
           (unsigned long)nrf_spi_frame_gap_us);
    stdio_flush();
    printf("Enabling BM20_C loopback...\n");
    stdio_flush();
    if (!nrf_set_slave_loopback(true)) {
        printf("Failed to enable BM20_C loopback mode. Is the updated BM20_C firmware flashed?\n");
        return;
    }
    loopback_active = true;
    if (restore_hz == 0) {
        restore_hz = start_hz;
    }
    printf("Press Enter to abort sweep.\n");

    // Iterate over each requested baud point.
    for (uint32_t hz = start_hz; hz <= stop_hz; hz += step_hz) {
        nrf_spi_prepare_bus(hz);
        printf("req=%lu actual=%lu ... ", (unsigned long)hz, (unsigned long)nrf_spi_actual_hz);
        stdio_flush();
        // One warm-up transfer discarded before measurement: ensures the SPI FIFO and slave
        // state are stable after the baud-rate change so the first timed frame is representative.
        (void)nrf_spi_transfer_bytes(tx, rx, tx_len);

        uint32_t ok = 0;   // Transfers whose RX exactly matched expect[]
        uint32_t fail = 0; // Transfers that returned a wrong length or wrong data
        uint64_t t0 = time_us_64();
        // Timed measurement loop: send 'loops' frames and count pass/fail.
        for (uint32_t i = 0; i < loops; ++i) {
            if (die) {
                aborted = true;
                break;
            }
            if (nrf_spi_transfer_bytes(tx, rx, tx_len) && memcmp(rx, expect, tx_len) == 0) {
                ok++;
            } else {
                fail++;
            }
            if ((loops >= 1000u) && ((i + 1u) % 1000u == 0u)) {
                printf(".");
                stdio_flush();
            }
        }
        uint64_t dt = time_us_64() - t0;
        if (loops >= 1000u) {
            printf("\n");
        }
        // Compute throughput: total bytes / elapsed seconds, cast to uint32.
        // The cast is safe because tx_len * loops can't overflow uint64 for practical sweep sizes.
        uint32_t throughput_bps = 0;
        if (dt > 0) {
            throughput_bps = (uint32_t)(((uint64_t)tx_len * loops * 1000000ULL) / dt);
        }

        printf("ok=%lu fail=%lu throughput=%lu B/s\n",
               (unsigned long)ok, (unsigned long)fail, (unsigned long)throughput_bps);

        if (aborted) {
            break;
        }
        // Guard against uint32 overflow: if incrementing hz would wrap past stop_hz, stop now.
        if (hz > stop_hz - step_hz) break;
    }

    nrf_spi_prepare_bus(restore_hz);
    if (loopback_active) {
        (void)nrf_set_slave_loopback(false);
    }
    if (aborted) {
        die = false;
        printf("Sweep aborted. Restored SPI baud to actual=%lu Hz\n", (unsigned long)nrf_spi_actual_hz);
    } else {
        printf("Sweep done. Restored SPI baud to actual=%lu Hz\n", (unsigned long)nrf_spi_actual_hz);
    }
}

#ifdef NRF_BENCH_ENABLED
/**
 * @brief Throughput benchmark: send <frame_bytes> of an auto-generated pattern for <loops> iterations,
 *        validate the echo, and report ok/fail counts and throughput in B/s.
 *
 * Avoids the 512-char hex string limitation of nrf_spi_sweep for large frames.
 * Pattern: tx[i] = i & 0xFF (incrementing 00..FF repeating).
 * After a warm-up transfer that primes the nRF's echo buffer, each timed transfer
 * receives the previously-sent pattern back and validates it byte-for-byte.
 */
void run_nrf_spi_bench(size_t argc, const char *argv[]) {
    if (argc != 2) {
        printf("Usage: nrf bench <frame_bytes> <loops>\n");
        printf("  frame_bytes: 1..4096  loops: 1..10000\n");
        printf("Example: nrf bench 256 50\n");
        return;
    }
    uint32_t frame_bytes = 0, loops = 0;
    if (!parse_u32_arg(argv[0], &frame_bytes) || !parse_u32_arg(argv[1], &loops)) {
        printf("Invalid arguments\n");
        return;
    }
    if (frame_bytes < 1 || frame_bytes > 4096) {
        printf("frame_bytes must be 1..4096\n");
        return;
    }
    if (loops < 1 || loops > 10000) {
        printf("loops must be 1..10000\n");
        return;
    }
    if (!nrf_spi_initialized) {
        nrf_spi_prepare_bus(15000000);
    }
    if (!nrf_set_slave_loopback(true)) {
        printf("Failed to enable BM20_C loopback mode. Is the updated BM20_C firmware flashed?\n");
        return;
    }

    /* For large frames the nRF SPIS driver needs more re-arm time.
     * Fixed floor of 400 µs covers Zephyr SPIS driver overhead regardless of frame size,
     * plus 2 µs/byte for DMA proportional cost. 32 B → 464 µs, 256 B → 912 µs, 4096 B → 8592 µs.
     * Without the floor, small-to-medium frames (e.g. 32 B → 160 µs) fell below the 350 µs
     * re-arm minimum and SPIS output TXD.MAXCNT=0 → all-zero transfers. */
    uint32_t effective_gap = nrf_spi_frame_gap_us;
    uint32_t scaled_gap = 400u + frame_bytes * 2u;
    if (scaled_gap > effective_gap) {
        effective_gap = scaled_gap;
        printf("NOTE: gap auto-scaled to %lu us for %lu B frame (current gap=%lu us is too small).\n"
               "      Run `nrf timing gap %lu` to keep this as default, or bench will auto-scale.\n",
               (unsigned long)effective_gap, (unsigned long)frame_bytes,
               (unsigned long)nrf_spi_frame_gap_us, (unsigned long)effective_gap);
    }

    /* Static to avoid blowing the stack with 4 KB frames. */
    static uint8_t bench_tx[4096];
    static uint8_t bench_rx[4096];

    for (uint32_t i = 0; i < frame_bytes; i++) {
        bench_tx[i] = (uint8_t)(i & 0xFF);
    }

    /* Warm-up: prime the nRF's tx_buffer with our pattern. */
    nrf_spi_drain_rx_fifo();
    gpio_put(nrf_spi_cs_pin, 0);
    busy_wait_us_32(nrf_spi_cs_setup_us);
    (void)spi_write_read_blocking(SPI_PORT, bench_tx, bench_rx, frame_bytes);
    busy_wait_us_32(nrf_spi_cs_hold_us);
    gpio_put(nrf_spi_cs_pin, 1);
    busy_wait_us_32(effective_gap);

    uint32_t ok = 0, fail = 0;
    bool printed_diag = false;
    uint64_t t0 = time_us_64();

    for (uint32_t i = 0; i < loops; i++) {
        nrf_spi_drain_rx_fifo();
        gpio_put(nrf_spi_cs_pin, 0);
        busy_wait_us_32(nrf_spi_cs_setup_us);
        int ret = spi_write_read_blocking(SPI_PORT, bench_tx, bench_rx, frame_bytes);
        busy_wait_us_32(nrf_spi_cs_hold_us);
        gpio_put(nrf_spi_cs_pin, 1);
        busy_wait_us_32(effective_gap);

        if (ret == (int)frame_bytes && memcmp(bench_rx, bench_tx, frame_bytes) == 0) {
            ok++;
        } else {
            fail++;
            /* Print first 16 bytes of the first failure to diagnose:
             *   All 0xFF → nRF not armed (gap still too small)
             *   5A/A5 pattern → nRF sent its initial fill, warm-up didn't take
             *   Shifted/partial match → bit alignment or framing error */
            if (!printed_diag) {
                printed_diag = true;
                printf("FAIL diag (frame %lu): ret=%d\n  TX[0..15]:", (unsigned long)i, ret);
                for (size_t d = 0; d < 16 && d < frame_bytes; d++) printf(" %02X", bench_tx[d]);
                printf("\n  RX[0..15]:");
                for (size_t d = 0; d < 16 && d < frame_bytes; d++) printf(" %02X", bench_rx[d]);
                printf("\n");
            }
        }
    }

    uint64_t dt = time_us_64() - t0;
    uint32_t throughput = (dt > 0) ? (uint32_t)(((uint64_t)frame_bytes * loops * 1000000ULL) / dt) : 0;

    printf("bench: %lu B/frame  %lu loops  actual=%lu Hz\n",
           (unsigned long)frame_bytes, (unsigned long)loops, (unsigned long)nrf_spi_actual_hz);
    printf("timing: setup=%lu us  hold=%lu us  gap=%lu us (effective)\n",
           (unsigned long)nrf_spi_cs_setup_us,
           (unsigned long)nrf_spi_cs_hold_us,
           (unsigned long)effective_gap);
    printf("ok=%lu fail=%lu throughput=%lu B/s (%lu KB/s)\n",
           (unsigned long)ok, (unsigned long)fail,
           (unsigned long)throughput, (unsigned long)(throughput / 1024));
    (void)nrf_set_slave_loopback(false);
}
#endif /* NRF_BENCH_ENABLED */

#ifdef NRF_BENCH_ENABLED
/**
 * @brief Run SPI diagnostics: Phase 1 tests all four CPOL/CPHA modes with CS high (loopback), Phase 2 pulses CS low once and prints the slave response.
 */
void run_nrf_spi_diag(size_t argc, const char *argv[]) {
    uint32_t loops = 16;
    if (argc > 1) {
        printf("Usage: nrf_spi_diag [loops]\n");
        return;
    }
    if (argc == 1 && !parse_u32_arg(argv[0], &loops)) {
        printf("Invalid loops value: %s\n", argv[0]);
        return;
    }    if (loops == 0) loops = 1;
    if (loops > 1000) loops = 1000;

    if (!nrf_spi_initialized) {
        nrf_spi_prepare_bus(15000000);
    }

    static const uint8_t tx[8] = {0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6, 0x07, 0x18};
    uint8_t rx[8];
    printf("NRF SPI diag at %lu Hz, loops=%lu\n",
           (unsigned long)nrf_spi_actual_hz, (unsigned long)loops);
    printf("Loopback phase: keep CS high (disconnect/tri-state external MISO source for pure test)\n");
    printf("Mode sweep (CPOL/CPHA):\n");

    struct spi_mode_case {
        spi_cpol_t cpol;
        spi_cpha_t cpha;
        const char *name;
    };
    static const struct spi_mode_case modes[] = {
        {SPI_CPOL_0, SPI_CPHA_0, "mode0"},
        {SPI_CPOL_0, SPI_CPHA_1, "mode1"},
        {SPI_CPOL_1, SPI_CPHA_0, "mode2"},
        {SPI_CPOL_1, SPI_CPHA_1, "mode3"},
    };

    gpio_put(nrf_spi_cs_pin, 1);
    busy_wait_us_32(5);

    for (size_t m = 0; m < count_of(modes); ++m) {
        spi_set_format(SPI_PORT, 8, modes[m].cpol, modes[m].cpha, SPI_MSB_FIRST);
        uint32_t ok = 0;
        uint32_t fail = 0;

        for (uint32_t i = 0; i < loops; ++i) {
            memset(rx, 0, sizeof(rx));
            nrf_spi_drain_rx_fifo();
            int ret = spi_write_read_blocking(SPI_PORT, tx, rx, sizeof(tx));
            if (ret == (int)sizeof(tx) && memcmp(rx, tx, sizeof(tx)) == 0) {
                ok++;
            } else {
                fail++;
                if (fail <= 2) {
                    printf("%s fail #%lu ret=%d TX=", modes[m].name, (unsigned long)fail, ret);
                    print_hex_bytes(tx, sizeof(tx));
                    printf(" RX=");
                    print_hex_bytes(rx, sizeof(rx));
                    printf("\n");
                }
            }
            busy_wait_us_32(5);
        }
        printf("%s summary: ok=%lu fail=%lu\n",
               modes[m].name, (unsigned long)ok, (unsigned long)fail);
    }

    // Restore operational mode (Mode 1: CPOL=0, CPHA=1).
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

    printf("Slave phase: pulse CS low once and print RX\n");
    memset(rx, 0, sizeof(rx));
    nrf_spi_drain_rx_fifo();
    gpio_put(nrf_spi_cs_pin, 0);
    busy_wait_us_32(nrf_spi_cs_setup_us);
    int ret = spi_write_read_blocking(SPI_PORT, tx, rx, sizeof(tx));
    busy_wait_us_32(nrf_spi_cs_hold_us);
    gpio_put(nrf_spi_cs_pin, 1);
    printf("slave ret=%d TX=", ret);
    print_hex_bytes(tx, sizeof(tx));
    printf(" RX=");
    print_hex_bytes(rx, sizeof(rx));
    printf("\n");
}
#endif /* NRF_BENCH_ENABLED */

// ─── NRF FFT / scope helpers ──────────────────────────────────────────────────

uint8_t nrf_fft_xor_checksum(const uint8_t *buf, size_t len) {
    uint8_t x = 0;
    for (size_t i = 0; i < len; ++i) {
        x ^= buf[i];
    }
    return x;
}

bool nrf_fft_send_command(uint8_t cmd, uint8_t sensor_id) {
	if (!nrf_spi_initialized) return false;
	memset(nrf_fft_tx_frame, 0, sizeof(nrf_fft_tx_frame));
    memset(nrf_fft_rx_frame, 0, sizeof(nrf_fft_rx_frame));
    nrf_fft_tx_frame[0] = cmd;
	nrf_fft_tx_frame[1] = sensor_id;
	return nrf_spi_transfer_bytes(nrf_fft_tx_frame, nrf_fft_rx_frame, sizeof(nrf_fft_tx_frame));
}

static bool nrf_stream_send_autoscale(void) {
    if (!nrf_spi_initialized) return false;
    memset(nrf_fft_tx_frame, 0, sizeof(nrf_fft_tx_frame));
    memset(nrf_fft_rx_frame, 0, sizeof(nrf_fft_rx_frame));
    nrf_fft_tx_frame[0] = NRF_SCOPE_CMD_AUTOSCALE;
    nrf_fft_tx_frame[1] = nrf_stream_auto_scale_enabled ? 1u : 0u;
    return nrf_spi_transfer_bytes(nrf_fft_tx_frame, nrf_fft_rx_frame, sizeof(nrf_fft_tx_frame));
}

static inline void nrf_scope_fb_set_pixel(uint16_t x, uint16_t y, uint16_t color) {
    if (!lcd_image) return;
    if (x >= LCD_2IN_WIDTH || y >= LCD_2IN_HEIGHT) return;

    /* In vertical scan mode, panel column 0 is physically at the right edge. */
    uint16_t fb_x = (lcd_scan_dir == VERTICAL) ? (uint16_t)(LCD_2IN_WIDTH - 1u - x) : x;
    size_t off = ((size_t)y * LCD_2IN_WIDTH + fb_x) * 2u;
    uint8_t *fb = (uint8_t *)lcd_image;
    fb[off] = (uint8_t)((color >> 8) & 0xFFu);
    fb[off + 1u] = (uint8_t)(color & 0xFFu);
}

static void nrf_fft_render_graph(void) {
    if (!lcd_initialized || !lcd_image || lcd_scan_dir != VERTICAL) return;

    Paint_Clear(BLACK);

    if (nrf_stream_fft_enabled) {
        /* FFT spectrum: vertical frequency axis at x=0, horizontal bars per bin */
        for (uint16_t y = 0; y < LCD_2IN_HEIGHT; ++y) {
            nrf_scope_fb_set_pixel(0, y, WHITE);
        }
        for (uint16_t row = 0; row < NRF_SCOPE_SAMPLE_COUNT; ++row) {
            uint16_t bar = nrf_fft_last_bins[row];
            if (bar >= LCD_2IN_WIDTH) bar = LCD_2IN_WIDTH - 1;
            for (uint16_t x = 1; x <= bar; ++x) {
                nrf_scope_fb_set_pixel(x, row, CYAN);
            }
        }
    } else {
        /* Raw waveform: vertical amplitude axis at x=120, connected dot-per-row */
        for (uint16_t y = 0; y < LCD_2IN_HEIGHT; ++y) {
            nrf_scope_fb_set_pixel(120, y, WHITE);
        }
        for (uint16_t x = 0; x < LCD_2IN_WIDTH; x += 2u) {
            nrf_scope_fb_set_pixel(x, (uint16_t)(LCD_2IN_HEIGHT / 2u), GRAY);
        }
        uint16_t prev_x = 120;
        for (uint16_t row = 0; row < NRF_SCOPE_SAMPLE_COUNT; ++row) {
            uint16_t x = nrf_fft_last_bins[row];
            if (x >= LCD_2IN_WIDTH) x = LCD_2IN_WIDTH - 1u;
            uint16_t lo = (x < prev_x) ? x : prev_x;
            uint16_t hi = (x > prev_x) ? x : prev_x;
            for (uint16_t xx = lo; xx <= hi; ++xx) {
                nrf_scope_fb_set_pixel(xx, row, CYAN);
            }
            prev_x = x;
        }
    }

    shared_enter_lcd_active();
    lcd_display_rows_from_framebuffer(0, LCD_2IN.HEIGHT);
    lcd_note_displayed_frame();
}

static void nrf_fft_stop_internal(bool notify_slave, bool clear_screen) {
    if (notify_slave && nrf_spi_initialized) {
        (void)nrf_fft_send_command(NRF_SCOPE_CMD_STOP, nrf_fft_selected_sensor);
    }

    nrf_fft_stream_active = false;
    nrf_fft_drop_first_packet = false;
    nrf_fft_period_override = false;
    nrf_fft_last_ts_ms = 0;
    nrf_fft_last_write_index = 0;
    nrf_fft_sensor_mismatch_frames = 0;
    memset(nrf_fft_last_bins, 0, sizeof(nrf_fft_last_bins));

    if (clear_screen && lcd_initialized && lcd_image) {
        Paint_Clear(BLACK);
        shared_enter_lcd_active();
        lcd_display_rows_from_framebuffer(0, LCD_2IN.HEIGHT);
        lcd_note_displayed_frame();
    }
}

static void nrf_fft_stream_task(void) {
    if (!nrf_fft_stream_active) return;
    if (!nrf_spi_initialized) {
        nrf_fft_frames_bad++;
        return;
    }
    if (!lcd_initialized || !lcd_image || lcd_scan_dir != VERTICAL) {
        nrf_fft_stop_internal(true, false);
        printf("\nNRF FFT stream stopped: LCD not ready (need lcd_init in vertical mode)\n");
        print_prompt();
        return;
    }
    if (!time_reached(nrf_fft_next_poll_time)) return;
    nrf_fft_next_poll_time = delayed_by_ms(get_absolute_time(), nrf_fft_poll_period_ms);

    memset(nrf_fft_tx_frame, 0, sizeof(nrf_fft_tx_frame));
    memset(nrf_fft_rx_frame, 0, sizeof(nrf_fft_rx_frame));
    nrf_fft_tx_frame[0] = NRF_SCOPE_CMD_FETCH;
    nrf_fft_tx_frame[1] = nrf_fft_selected_sensor;

    if (!nrf_spi_transfer_bytes(nrf_fft_tx_frame, nrf_fft_rx_frame, sizeof(nrf_fft_tx_frame))) {
        nrf_fft_frames_bad++;
        nrf_fft_bad_spi++;
        return;
    }

    if (nrf_fft_drop_first_packet) {
        nrf_fft_drop_first_packet = false;
        return;
    }

    uint16_t sample_count = (uint16_t)nrf_fft_rx_frame[8] | ((uint16_t)nrf_fft_rx_frame[9] << 8);
    if (nrf_fft_rx_frame[0] != NRF_SCOPE_MAGIC0 ||
        nrf_fft_rx_frame[1] != NRF_SCOPE_MAGIC1 ||
        nrf_fft_rx_frame[2] != NRF_SCOPE_VERSION ||
        sample_count != NRF_SCOPE_SAMPLE_COUNT ||
        nrf_fft_xor_checksum(nrf_fft_rx_frame, sizeof(nrf_fft_rx_frame)) != 0) {
        memcpy(nrf_fft_last_bad_hdr, nrf_fft_rx_frame, NRF_SCOPE_HEADER_BYTES);
        if (nrf_fft_rx_frame[0] != NRF_SCOPE_MAGIC0 || nrf_fft_rx_frame[1] != NRF_SCOPE_MAGIC1)
            nrf_fft_bad_magic++;
        else if (nrf_fft_rx_frame[2] != NRF_SCOPE_VERSION)
            nrf_fft_bad_version++;
        else if (sample_count != NRF_SCOPE_SAMPLE_COUNT)
            nrf_fft_bad_count++;
        else
            nrf_fft_bad_cksum++;
        nrf_fft_frames_bad++;
        return;
    }

    if ((nrf_fft_rx_frame[3] & 0x01u) == 0u) {
        memcpy(nrf_fft_last_bad_hdr, nrf_fft_rx_frame, NRF_SCOPE_HEADER_BYTES);
        nrf_fft_bad_stream_off++;
        nrf_fft_frames_bad++;
        return;
    }

    nrf_fft_last_sensor = nrf_fft_rx_frame[5];
    if (nrf_fft_last_sensor != nrf_fft_selected_sensor) {
        // Do not render mismatched data; request a stream re-sync after a few consecutive mismatches.
        nrf_fft_sensor_mismatch_frames++;
        if (nrf_fft_sensor_mismatch_frames >= 3u) {
            if (nrf_fft_send_command(NRF_SCOPE_CMD_START, nrf_fft_selected_sensor)) {
                nrf_fft_resync_count++;
                nrf_fft_drop_first_packet = true;
                nrf_fft_next_poll_time = get_absolute_time();
            } else {
                nrf_fft_frames_bad++;
            }
            nrf_fft_sensor_mismatch_frames = 0;
        }
        return;
    }
    nrf_fft_sensor_mismatch_frames = 0;
    nrf_fft_last_ts_ms = (uint16_t)nrf_fft_rx_frame[6] | ((uint16_t)nrf_fft_rx_frame[7] << 8);
    nrf_fft_last_write_index = (uint16_t)nrf_fft_rx_frame[10] | ((uint16_t)nrf_fft_rx_frame[11] << 8);
    if (!nrf_fft_period_override && nrf_fft_last_ts_ms > 0) {
        nrf_fft_poll_period_ms = (uint32_t)nrf_fft_last_ts_ms + 1u;
    }

    memcpy(nrf_fft_last_bins, &nrf_fft_rx_frame[NRF_SCOPE_HEADER_BYTES], NRF_SCOPE_SAMPLE_COUNT);
    nrf_fft_last_seq = nrf_fft_rx_frame[4];
    nrf_fft_frames_ok++;
    nrf_fft_render_graph();
}

static void run_nrf_stream(size_t argc, const char *argv[]) {
    if (argc == 0 || strcmp(argv[0], "help") == 0) {
        printf("Usage: nrf stream start [period_ms]\n");
        printf("       nrf stream stop\n");
        printf("       nrf stream status\n");
		printf("       nrf stream sensor <0|1|2>\n");
		printf("       nrf stream period <20..1000>\n");
		printf("       nrf stream autoscale <on|off|status> - hardware sensor scaling (currently %s)\n",
		       nrf_stream_auto_scale_enabled ? "on" : "off");
		printf("       nrf stream fft           - toggle FFT on/off (currently %s)\n",
		       nrf_stream_fft_enabled ? "on" : "off");
		return;
    }

    const char *sub = argv[0];
    if (strcmp(sub, "start") == 0) {
        uint32_t period_ms = 0;
        if (argc > 2) {
            printf("Usage: nrf stream start [period_ms]\n");
            return;
        }
        if (argc == 2) {
            if (!parse_u32_arg(argv[1], &period_ms) || period_ms < 20 || period_ms > 1000) {
                printf("Invalid period: %s (use 20..1000 ms)\n", argv[1]);
                return;
            }
            nrf_fft_poll_period_ms = period_ms;
            nrf_fft_period_override = true;
        } else {
            nrf_fft_period_override = false;
        }
        if (!lcd_initialized || !lcd_image) {
            printf("LCD not initialized. Run lcd init first.\n");
            return;
        }
        if (lcd_scan_dir != VERTICAL) {
            printf("nrf stream requires vertical orientation. Run: lcd orient vertical\n");
            return;
        }
        if (cam_snap_pending || lcd_load_pending || lcd_cam_snap_pending || lcd_cam_stream_active) {
            printf("Stop current camera/LCD background activity before starting nrf stream.\n");
            return;
        }
        lcd_con_stop();
        const uint32_t nrf_fft_stream_safe_hz = 8000000;
        if (!nrf_spi_initialized || nrf_spi_actual_hz > nrf_fft_stream_safe_hz) {
            nrf_spi_prepare_bus(nrf_fft_stream_safe_hz);
        }

        nrf_fft_frames_ok = 0;
        nrf_fft_frames_bad = 0;
        nrf_fft_bad_magic = 0;
        nrf_fft_bad_version = 0;
        nrf_fft_bad_count = 0;
        nrf_fft_bad_cksum = 0;
        nrf_fft_bad_stream_off = 0;
        nrf_fft_bad_spi = 0;
        memset(nrf_fft_last_bad_hdr, 0, sizeof(nrf_fft_last_bad_hdr));
        nrf_fft_sensor_mismatch_frames = 0;
        nrf_fft_resync_count = 0;
        nrf_fft_last_seq = 0;
        nrf_fft_last_sensor = nrf_fft_selected_sensor;
        nrf_fft_last_ts_ms = 0;
        nrf_fft_last_write_index = 0;
        memset(nrf_fft_last_bins, 0, sizeof(nrf_fft_last_bins));

		if (!nrf_stream_send_autoscale()) {
			printf("Warning: failed to send autoscale mode to nRF.\n");
		}

		if (!nrf_fft_send_command(NRF_SCOPE_CMD_START, nrf_fft_selected_sensor)) {
			printf("Failed to start NRF stream (SPI transfer failed)\n");
			return;
		}

        nrf_fft_drop_first_packet = true;
        nrf_fft_stream_active = true;
        nrf_fft_next_poll_time = get_absolute_time();
        if (nrf_fft_period_override) {
			printf("NRF stream started: sensor=%u period=%lu ms (override) fft=%s autoscale=%s\n",
			       nrf_fft_selected_sensor, (unsigned long)nrf_fft_poll_period_ms,
			       nrf_stream_fft_enabled ? "on" : "off",
			       nrf_stream_auto_scale_enabled ? "on" : "off");
		} else {
			printf("NRF stream started: sensor=%u period=Ts+1ms (auto) fft=%s autoscale=%s\n",
			       nrf_fft_selected_sensor,
			       nrf_stream_fft_enabled ? "on" : "off",
			       nrf_stream_auto_scale_enabled ? "on" : "off");
		}
		return;
	}

    if (strcmp(sub, "stop") == 0) {
        if (!nrf_fft_stream_active) {
            printf("NRF stream is not running.\n");
            return;
        }
        nrf_fft_stop_internal(true, true);
        printf("NRF stream stopped.\n");
        return;
    }

    if (strcmp(sub, "status") == 0) {
		printf("NRF stream: %s\n", nrf_fft_stream_active ? "running" : "stopped");
		printf("FFT: %s\n", nrf_stream_fft_enabled ? "on" : "off");
		printf("Autoscale: %s\n", nrf_stream_auto_scale_enabled ? "on" : "off");
		printf("Selected sensor: %u\n", nrf_fft_selected_sensor);
        printf("Last sensor: %u\n", nrf_fft_last_sensor);
        printf("Ts (last): %u ms\n", (unsigned)nrf_fft_last_ts_ms);
        printf("Period: %lu ms\n", (unsigned long)nrf_fft_poll_period_ms);
        printf("Period mode: %s\n", nrf_fft_period_override ? "manual override" : "Ts+1ms auto");
        printf("Write index (last): %u\n", nrf_fft_last_write_index);
        printf("Frames: ok=%lu bad=%lu\n",
               (unsigned long)nrf_fft_frames_ok,
               (unsigned long)nrf_fft_frames_bad);
        if (nrf_fft_frames_bad > 0) {
            printf("  bad breakdown: magic=%lu ver=%lu count=%lu cksum=%lu stream_off=%lu spi=%lu\n",
                   (unsigned long)nrf_fft_bad_magic,
                   (unsigned long)nrf_fft_bad_version,
                   (unsigned long)nrf_fft_bad_count,
                   (unsigned long)nrf_fft_bad_cksum,
                   (unsigned long)nrf_fft_bad_stream_off,
                   (unsigned long)nrf_fft_bad_spi);
            printf("  last bad hdr: ");
            for (int _i = 0; _i < (int)NRF_SCOPE_HEADER_BYTES; _i++) {
                printf("%02X ", nrf_fft_last_bad_hdr[_i]);
            }
            printf("\n");
            printf("  expect hdr:   %02X %02X %02X xx xx xx xx xx %02X %02X xx xx xx\n",
                   NRF_SCOPE_MAGIC0, NRF_SCOPE_MAGIC1, NRF_SCOPE_VERSION,
                   (unsigned)(NRF_SCOPE_SAMPLE_COUNT & 0xFF),
                   (unsigned)(NRF_SCOPE_SAMPLE_COUNT >> 8));
        }
        printf("Sensor mismatches (consecutive current): %lu\n",
               (unsigned long)nrf_fft_sensor_mismatch_frames);
        printf("Automatic stream resync count: %lu\n",
               (unsigned long)nrf_fft_resync_count);
        printf("Last seq=%u\n", nrf_fft_last_seq);
        return;
    }

    if (strcmp(sub, "sensor") == 0) {
        if (argc != 2) {
            printf("Usage: nrf stream sensor <0|1|2>\n");
            return;
        }
        uint32_t sid = 0;
        if (!parse_u32_arg(argv[1], &sid) || sid > 2u) {
            printf("Invalid sensor: %s (use 0..2)\n", argv[1]);
            return;
        }
        nrf_fft_selected_sensor = (uint8_t)sid;
        if (nrf_spi_initialized) {
            uint8_t cmd = nrf_fft_stream_active ? NRF_SCOPE_CMD_START : NRF_SCOPE_CMD_SENSOR;
            if (!nrf_fft_send_command(cmd, nrf_fft_selected_sensor)) {
                printf("Warning: failed to send sensor update over SPI.\n");
            }
        }
        if (nrf_fft_stream_active) {
            nrf_fft_drop_first_packet = true;
            nrf_fft_sensor_mismatch_frames = 0;
            nrf_fft_next_poll_time = get_absolute_time();
        }
        printf("NRF sensor set to %u\n", nrf_fft_selected_sensor);
        return;
    }

    if (strcmp(sub, "period") == 0) {
        if (argc != 2) {
            printf("Usage: nrf stream period <20..1000>\n");
            return;
        }
        uint32_t ms = 0;
        if (!parse_u32_arg(argv[1], &ms) || ms < 20 || ms > 1000) {
            printf("Invalid period: %s (use 20..1000 ms)\n", argv[1]);
            return;
        }
        nrf_fft_poll_period_ms = ms;
        nrf_fft_period_override = true;
        printf("NRF stream period set to %lu ms\n", (unsigned long)nrf_fft_poll_period_ms);
        return;
    }

	if (strcmp(sub, "fft") == 0) {
		nrf_stream_fft_enabled = !nrf_stream_fft_enabled;
        if (nrf_spi_initialized) {
            memset(nrf_fft_tx_frame, 0, sizeof(nrf_fft_tx_frame));
            memset(nrf_fft_rx_frame, 0, sizeof(nrf_fft_rx_frame));
            nrf_fft_tx_frame[0] = NRF_SCOPE_CMD_SET_MODE;
            nrf_fft_tx_frame[1] = nrf_stream_fft_enabled ? 1u : 0u;
            if (!nrf_spi_transfer_bytes(nrf_fft_tx_frame, nrf_fft_rx_frame, sizeof(nrf_fft_tx_frame))) {
                printf("Warning: failed to send FFT mode change to nRF.\n");
            }
            if (nrf_fft_stream_active) {
                nrf_fft_drop_first_packet = true;
                nrf_fft_next_poll_time = get_absolute_time();
            }
        }
		printf("FFT %s\n", nrf_stream_fft_enabled ? "on" : "off");
		return;
	}

	if (strcmp(sub, "autoscale") == 0) {
		if (argc != 2 ||
		    (strcmp(argv[1], "on") != 0 && strcmp(argv[1], "off") != 0 && strcmp(argv[1], "status") != 0)) {
			printf("Usage: nrf stream autoscale <on|off|status>\n");
			return;
		}
		if (strcmp(argv[1], "status") == 0) {
			printf("Autoscale %s\n", nrf_stream_auto_scale_enabled ? "on" : "off");
			return;
		}
		nrf_stream_auto_scale_enabled = (strcmp(argv[1], "on") == 0);
		if (nrf_spi_initialized) {
			if (!nrf_stream_send_autoscale()) {
				printf("Warning: failed to send autoscale mode to nRF.\n");
			}
			if (nrf_fft_stream_active) {
				nrf_fft_drop_first_packet = true;
				nrf_fft_next_poll_time = get_absolute_time();
			}
		}
		printf("Autoscale %s\n", nrf_stream_auto_scale_enabled ? "on" : "off");
		return;
	}

	printf("Unknown nrf stream subcommand: %s\n", sub);
    printf("Try: nrf stream help\n");
}

/**
 * @brief Show or tune Mode-B SPI timing: displays current CS setup/hold/gap/chunk values and an FPS estimate, or sets one parameter.
 */
void run_nrf_timing(size_t argc, const char *argv[]) {
    if (argc == 0) {
        uint32_t chunk_bytes = cam_width * 2u * nrf_validation_rows_per_chunk;
        uint32_t n_chunks = (cam_height + nrf_validation_rows_per_chunk - 1u) / nrf_validation_rows_per_chunk;
        uint32_t n_transfers = n_chunks + 1u;  // +1 for the flush
        uint32_t overhead_us = nrf_spi_cs_setup_us + nrf_spi_cs_hold_us + nrf_spi_frame_gap_us;
        printf("cs_setup=%lu us  cs_hold=%lu us  gap=%lu us  chunk=%lu rows (%lu bytes)\n",
               (unsigned long)nrf_spi_cs_setup_us,
               (unsigned long)nrf_spi_cs_hold_us,
               (unsigned long)nrf_spi_frame_gap_us,
               (unsigned long)nrf_validation_rows_per_chunk,
               (unsigned long)chunk_bytes);
        printf("chunk_rows_max=%u  transfers/frame=%lu  per_transfer_overhead=%lu us\n",
               NRF_VALIDATION_ROWS_MAX,
               (unsigned long)n_transfers,
               (unsigned long)overhead_us);
        if (nrf_spi_actual_hz > 0) {
            uint64_t data_us = ((uint64_t)chunk_bytes * 8u * 1000000u) / nrf_spi_actual_hz;
            uint64_t frame_us = (uint64_t)n_transfers * (data_us + overhead_us);
            uint32_t fps10 = (frame_us > 0) ? (uint32_t)(10000000u / frame_us) : 0;
            printf("SPI=%lu Hz  data/transfer=%lu us  frame=%lu ms  est. FPS=%lu.%lu\n",
                   (unsigned long)nrf_spi_actual_hz,
                   (unsigned long)data_us,
                   (unsigned long)(frame_us / 1000u),
                   (unsigned long)(fps10 / 10u),
                   (unsigned long)(fps10 % 10u));
        } else {
            printf("SPI not initialized (run nrf_spi_init first)\n");
        }
        return;
    }
    if (argc != 2) {
        printf("Usage: nrf_timing                (show current settings + FPS estimate)\n");
        printf("       nrf_timing gap   <us>     (inter-frame gap, e.g. 50)\n");
        printf("       nrf_timing setup <us>     (CS setup before first SCK, e.g. 10)\n");
        printf("       nrf_timing hold  <us>     (CS hold after last SCK, e.g. 2)\n");
        printf("       nrf_timing chunk <rows>   (rows per transfer, 1..%u)\n", NRF_VALIDATION_ROWS_MAX);
        return;
    }
    uint32_t val;
    if (!parse_u32_arg(argv[1], &val)) {
        printf("Invalid value: %s\n", argv[1]);
        return;
    }
    if (strcmp(argv[0], "gap") == 0) {
        uint32_t old = nrf_spi_frame_gap_us;
        nrf_spi_frame_gap_us = val;
        printf("gap: %lu -> %lu us\n", (unsigned long)old, (unsigned long)nrf_spi_frame_gap_us);
    } else if (strcmp(argv[0], "setup") == 0) {
        uint32_t old = nrf_spi_cs_setup_us;
        nrf_spi_cs_setup_us = val;
        printf("cs_setup: %lu -> %lu us\n", (unsigned long)old, (unsigned long)nrf_spi_cs_setup_us);
    } else if (strcmp(argv[0], "hold") == 0) {
        uint32_t old = nrf_spi_cs_hold_us;
        nrf_spi_cs_hold_us = val;
        printf("cs_hold: %lu -> %lu us\n", (unsigned long)old, (unsigned long)nrf_spi_cs_hold_us);
    } else if (strcmp(argv[0], "chunk") == 0) {
        if (val == 0 || val > NRF_VALIDATION_ROWS_MAX) {
            printf("chunk must be 1..%u\n", NRF_VALIDATION_ROWS_MAX);
            return;
        }
        uint32_t old = nrf_validation_rows_per_chunk;
        nrf_validation_rows_per_chunk = val;
        printf("rows_per_chunk: %lu -> %lu\n", (unsigned long)old, (unsigned long)nrf_validation_rows_per_chunk);
    } else {
        printf("Unknown key '%s'. Use: gap | setup | hold | chunk\n", argv[0]);
    }
}

// ─── Public accessors used by command.cpp ────────────────────────────────────

bool nrf_fft_stream_is_active(void) {
    return nrf_fft_stream_active;
}

void nrf_fft_stop_public(bool notify_slave, bool clear_screen) {
    nrf_fft_stop_internal(notify_slave, clear_screen);
}

void nrf_fft_stream_task_tick(void) {
    nrf_fft_stream_task();
}

void run_nrf_stream_public(size_t argc, const char *argv[]) {
    run_nrf_stream(argc, argv);
}
