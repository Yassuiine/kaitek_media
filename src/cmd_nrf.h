#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
//
#include "pico/stdlib.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NRF_BENCH_ENABLED  // Compile with this defined to enable the SPI slave test mode that logs transfer stats and button events; undefine for production build without logging and NRF event handling
#ifndef ENABLE_PATH_AB_LOGIC
#define ENABLE_PATH_AB_LOGIC 1
#endif

#define NRF_VALIDATION_ROWS_MAX 8  // Hard upper bound on rows_per_chunk; sets nrf_validation_rx_chunk size

// Scope protocol constants
#define NRF_SCOPE_SAMPLE_COUNT 320u
#define NRF_SCOPE_HEADER_BYTES 13u
#define NRF_SCOPE_PACKET_BYTES (NRF_SCOPE_HEADER_BYTES + NRF_SCOPE_SAMPLE_COUNT)
#define NRF_SCOPE_MAGIC0 0x53u /* 'S' */
#define NRF_SCOPE_MAGIC1 0x42u /* 'B' */
#define NRF_SCOPE_VERSION 1u
#define NRF_SCOPE_CMD_START    0xB0u
#define NRF_SCOPE_CMD_STOP     0xB1u
#define NRF_SCOPE_CMD_FETCH    0xB2u
#define NRF_SCOPE_CMD_SENSOR   0xB3u
#define NRF_SCOPE_CMD_SET_MODE 0xB4u  /* payload byte: 0=raw waveform, 1=FFT */

// Per-frame transfer context for the Mode-B NRF relay path.
// Because Mode-B sends one chunk per scheduler tick, this struct persists transfer
// progress across multiple calls to process_background_tasks().
typedef struct {
    bool active;                   // True while a frame transfer is in progress
    const uint8_t *frame;          // Pointer to the locked camera frame being relayed
    bool first_transfer;           // True until the first chunk has been sent (pipeline not yet primed)
    uint32_t row;                  // Next camera row to transmit in Phase A
    uint32_t prev_row;             // Row index of the most recently transmitted chunk
    uint32_t prev_row_count;       // Row count of the most recently transmitted chunk
    size_t prev_byte_count;        // Byte count of the most recently transmitted chunk
    const uint8_t *prev_src;       // Source pointer of the most recently transmitted chunk
    uint8_t prev_tx_fingerprint[16]; // First 16 bytes of the previously sent chunk; used to validate the echo
} mode_b_transfer_ctx_t;

// Globals (extern declarations)
extern bool nrf_spi_initialized;
extern uint32_t nrf_spi_actual_hz;
extern const uint nrf_spi_cs_pin;
extern const uint nrf_spi_sck_pin;
extern const uint nrf_spi_mosi_pin;
extern const uint nrf_spi_miso_pin;
extern uint32_t nrf_spi_cs_setup_us;
extern uint32_t nrf_spi_cs_hold_us;
extern uint32_t nrf_spi_frame_gap_us;
extern const uint32_t nrf_event_min_interval_ms;
extern const uint32_t mode_b_fail_block_ms;
extern const uint32_t mode_b_retry_safe_hz;
extern uint32_t nrf_validation_rows_per_chunk;
extern uint8_t nrf_validation_rx_chunk[];
extern uint8_t nrf_validation_dummy_chunk[];
extern mode_b_transfer_ctx_t mode_b_ctx;
extern const uint8_t *mode_b_stream_locked_frame;
extern int spi_tx_dma_ch;
extern int spi_rx_dma_ch;
extern volatile uint16_t spi_rx_dummy;

// Function declarations
bool nrf_spi_prepare_bus(uint32_t requested_hz);
void nrf_spi_drain_rx_fifo(void);
bool nrf_spi_transfer_bytes(const uint8_t *tx, uint8_t *rx, size_t len);

bool ensure_mode_b_frame_raw(size_t needed_bytes);
void free_mode_b_frame_raw(void);
bool bytes_all_value(const uint8_t *buf, size_t len, uint8_t value);
void reset_mode_b_transfer_ctx(void);

void run_nrf_spi_init(size_t argc, const char *argv[]);
void run_nrf_spi_status(size_t argc, const char *argv[]);
void run_nrf_spi_xfer(size_t argc, const char *argv[]);
void run_nrf_spi_sweep(size_t argc, const char *argv[]);
#ifdef NRF_BENCH_ENABLED
void run_nrf_spi_bench(size_t argc, const char *argv[]);
void run_nrf_spi_diag(size_t argc, const char *argv[]);
#endif
void run_nrf_timing(size_t argc, const char *argv[]);

uint8_t nrf_fft_xor_checksum(const uint8_t *buf, size_t len);
bool nrf_fft_send_command(uint8_t cmd, uint8_t sensor_id);

// Public accessors for NRF FFT stream state (internal state lives in cmd_nrf.cpp)
bool nrf_fft_stream_is_active(void);
void nrf_fft_stop_public(bool notify_slave, bool clear_screen);
void nrf_fft_stream_task_tick(void);
void run_nrf_stream_public(size_t argc, const char *argv[]);

#ifdef __cplusplus
}
#endif
