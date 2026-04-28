#include <assert.h>
#include <ctype.h>
#include <malloc.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
//
#include "hardware/adc.h"
#include "hardware/clocks.h" 
#include "pico/aon_timer.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "pico/time.h"
//
#include "f_util.h"
#include "crash.h"
#include "hw_config.h"
#include "my_debug.h"
#include "my_rtc.h"
#include "sd_card.h"
#include "tests.h"
#include "cam.h"
#include "ov5640.h"
//
#include "diskio.h" /* Declarations of disk functions */
//
#include "command.h"
#include "DEV_Config.h"
#include "LCD_2in.h"
#include "GUI_Paint.h"

#define NRF_BENCH_ENABLED  // Compile with this defined to enable the SPI slave test mode that logs transfer stats and button events; undefine for production build without logging and NRF event handling

static char *saveptr;                    // strtok_r re-entrant parse position; shared across all tokenisation in process_cmd
static volatile bool die;                // Set by chars_available_callback on Enter to abort a running loop command
static char cmd_buffer[256];            // In-progress command line being edited; modified in-place by strtok_r on Enter
static size_t cmd_ix;                   // Logical end of cmd_buffer: number of characters typed (excluding NUL)
static size_t cmd_cursor;               // Cursor insert position within cmd_buffer; 0 = home, cmd_ix = end
static const char *PROMPT_STR = "Kaitek> ";  // Shell prompt string printed before every new input line
static const size_t CMD_HISTORY_MAX = 32;    // Maximum number of history entries retained in the ring
static char cmd_history[32][256];            // Circular ring of previous command lines (oldest overwritten when full)
static size_t cmd_history_count;             // Number of valid entries currently stored in cmd_history (0..CMD_HISTORY_MAX)
static int cmd_history_nav_index = -1;       // Index into cmd_history for Up/Down navigation; -1 = not in history mode
static char cmd_history_edit_backup[256];    // Snapshot of the in-progress edit saved before Up-arrow enters history mode
static size_t cmd_history_edit_backup_len;   // Length of the saved edit backup (chars, excluding NUL)
static bool cmd_history_backup_valid;        // True when cmd_history_edit_backup contains a valid saved line
static bool tab_was_last_key;                // True after a Tab that left the token unchanged (arms second-Tab list display)

static bool cam_snap_pending;                   // True while an async cam snap operation is in progress
static absolute_time_t cam_snap_deadline;       // Absolute time after which the frame-wait phase times out
static char cam_snap_path[256];                 // Destination file path for the current cam snap operation
static FIL cam_snap_file;                       // FatFS file object for the output file during cam snap
static bool cam_snap_file_open;                 // True when cam_snap_file is open and must be closed on finish/abort
static uint32_t cam_snap_write_offset;          // Byte offset into cam_ptr already written to the file
static UINT cam_snap_total_written;             // Cumulative bytes successfully written across all chunks so far
static absolute_time_t cam_snap_next_step_time; // Earliest time the state machine should execute its next step
static bool last_input_was_cr;                  // Tracks whether the previous byte was CR to suppress the following LF (CRLF â†' CR)

static bool lcd_initialized = false;               // True after lcd_init succeeds; guards all LCD command handlers
static bool lcd_con_active = false;                // True while the LCD console output driver is registered
static uint8_t lcd_con_ansi_state = 0;             // 0=normal, 1=seen ESC, 2=in CSI sequence (strip until final letter)
static int  lcd_con_x = 0;                        // Cursor column in pixels
static int  lcd_con_y = 0;                        // Cursor row in pixels
static uint16_t lcd_con_fg = 0xFFFF;              // Console foreground colour (default white)
static uint16_t lcd_con_bg = 0x0000;              // Console background colour (default black)
static const sFONT *lcd_con_font = &Font8;        // Console font (Font8 = 8×8 px → 40 cols × 60 rows at 320×480)
static int lcd_con_width  = 0;                    // Panel pixel width cached at lcdcon start
static int lcd_con_height = 0;                    // Panel pixel height cached at lcdcon start
static uint8_t lcd_backlight = 0;                  // Current PWM backlight level in percent (0 = off, 100 = full)
static UBYTE lcd_scan_dir = VERTICAL;              // Current panel scan direction: VERTICAL (portrait) or HORIZONTAL (landscape)
static UWORD *lcd_image = NULL;                    // Heap-allocated framebuffer (WIDTH×HEIGHT RGB565 words); NULL before lcd_init
static size_t lcd_image_bytes = 0;                 // Size of lcd_image in bytes (= WIDTH × HEIGHT × 2)
static const uint32_t lcd_spi_hz = 62500000;       // Requested SPI baud for the LCD; actual rate set by spi_init()
static uint32_t lcd_display_fps = 0;               // Rolling one-second display frame rate (frames per second)
static uint32_t lcd_display_frames_total = 0;      // Total frames pushed to the LCD panel since last lcd_init
static uint32_t lcd_display_frames_window = 0;     // Frame count in the current FPS measurement window
static uint64_t lcd_display_window_start_us = 0;   // Timestamp (us) when the current FPS window started

static bool nrf_spi_initialized = false;           // True after nrf_spi_prepare_bus() succeeds at least once
static uint32_t nrf_spi_actual_hz = 0;             // Baud rate actually achieved by spi_init() for NRF transfers
// Pico SPI0 pins shared with the LCD bus.  Wire each to the nRF54L15 SPIS00 dedicated P2 pin shown:
//   Pico GPIO22 (I2C1_SDA) → nRF P2.10  CSN
//   Pico GPIO18 (LCD_CLK)  → nRF P2.06  SCK   (dedicated HSSPI clock pin, up to 32 MHz)
//   Pico GPIO19 (LCD_MOSI) → nRF P2.09  SDO   (nRF SDO = data out of nRF = Pico MOSI)
//   Pico GPIO20 (LCD_RST)  → nRF P2.08  SDI   (nRF SDI = data into nRF = Pico MISO echo)
// SPIS00 (P2 port) supports up to 32 MHz; SPIS20/21/30 (P0/P1) are limited to 8 MHz.
static const uint nrf_spi_cs_pin = I2C1_SDA;       // GPIO22 → nRF P2.10 CSN
static const uint nrf_spi_sck_pin = LCD_CLK_PIN;   // GPIO18 → nRF P2.06 SCK
static const uint nrf_spi_mosi_pin = LCD_MOSI_PIN; // GPIO19 → nRF P2.09 SDO
static const uint nrf_spi_miso_pin = LCD_RST_PIN;  // GPIO20 → nRF P2.08 SDI (echo path back to Pico)
static uint32_t nrf_spi_cs_setup_us = 15;          // CS-assert-to-first-SCK delay; tune down to 2 µs with `nrf timing setup 2` for HSSPI
static uint32_t nrf_spi_cs_hold_us = 4;            // Last-SCK-to-CS-deassert hold; tune down to 2 µs with `nrf timing hold 2` for HSSPI
static uint32_t nrf_spi_frame_gap_us = 200;        // Inter-frame idle gap; tune down to 50 µs with `nrf timing gap 50` for HSSPI
static const uint32_t nrf_event_min_interval_ms = 250;  // Debounce window: ignores NRF button events closer than this
static const uint32_t mode_b_fail_block_ms = 1200;      // After a Mode-B SPI failure, suppress NRF events for this duration
static const uint32_t mode_b_retry_safe_hz = 16000000;  // HSSPI safe-fallback baud for one-time all-0xFF recovery retry (was 6 MHz for SPIS20; raised to 16 MHz for SPIS00)
static uint32_t nrf_validation_rows_per_chunk = 4;      // Number of camera rows transferred per NRF SPI chunk (tunable)
#define NRF_VALIDATION_ROWS_MAX 8                        // Hard upper bound on rows_per_chunk; sets nrf_validation_rx_chunk size
static uint8_t nrf_validation_rx_chunk[LCD_2IN_WIDTH * 2 * NRF_VALIDATION_ROWS_MAX];   // Receive buffer for one NRF chunk
static uint8_t nrf_validation_dummy_chunk[LCD_2IN_WIDTH * 2 * NRF_VALIDATION_ROWS_MAX]; // All-zero payload for the Phase-B flush transfer

// Mode-B assembles echoed chunks directly into lcd_image interpreted as bytes.
// This avoids a second full-frame static buffer and saves ~153 KB of RAM.

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
static mode_b_transfer_ctx_t mode_b_ctx = {0};

static bool touch_initialized = false;              // True after touch_init_controller() confirms the CST816D chip ID
static bool touch_press_latched = false;            // True while a press is held; prevents repeated trigger events
static bool cam_mirror_enabled = false;             // Tracks the current OV5640 horizontal mirror state for display
static uint32_t touch_event_count = 0;             // Total number of debounced touch events that triggered a mirror toggle
static uint32_t nrf_event_count = 0;               // Total number of NRF button events that triggered an action
static absolute_time_t touch_next_poll_time;        // Earliest time to re-sample the Touch_INT_PIN
static absolute_time_t nrf_event_poll_next_time;    // Earliest time to send the next standalone NRF poll transfer
static absolute_time_t nrf_event_block_until;       // Block NRF event processing until this time (debounce / fail-recovery)
static const uint8_t touch_addr = 0x15;             // I2C address of the CST816D touch controller on I2C0
static const uint8_t touch_reg_chip_id = 0xA7;      // CST816D register that returns the chip identification byte
static const uint8_t touch_expected_chip_id = 0xB6; // Expected chip-ID value confirming a genuine CST816D is present
static const uint32_t touch_poll_interval_ms = 8;   // How often to sample Touch_INT_PIN (ms); ~125 Hz
static const uint32_t touch_debounce_ms = 180;      // Minimum gap between two consecutive touch events (ms)
static const uint32_t nrf_event_poll_interval_ms = 35; // Interval between standalone NRF button-poll transfers in Mode A (ms)
static absolute_time_t touch_debounce_deadline;     // Earliest time a new touch event will be accepted after the last one

/**
 * @brief Forward declarations for helpers used before full definitions.
 */
static bool nrf_spi_prepare_bus(uint32_t requested_hz);
static void nrf_spi_drain_rx_fifo(void);
static void print_prompt(void);
static void lcd_note_displayed_frame(void);
static void lcd_display_rows_from_framebuffer(uint32_t start_row, uint32_t row_count);
static void lcd_display_raw_rows(uint32_t start_row, uint32_t row_count, const uint8_t *raw_rows);
static void lcd_copy_raw_rows_to_framebuffer(uint32_t start_row, uint32_t row_count, const uint8_t *raw_rows);
static void shared_enter_cam_prog(void);
static void shared_leave_cam_prog(void);
static void reset_mode_b_transfer_ctx(void);
static void lcd_con_putchar(char c);

// Dual-output printf: always writes to UART/USB via vprintf, and also mirrors
// to the LCD console when lcd_con_active is set. #define below replaces all
// printf calls in this TU so the LCD terminal sees every shell output line.
__attribute__((format(printf, 1, 2)))
static int kaitek_printf(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    int n = vprintf(fmt, ap);
    va_end(ap);
    if (lcd_con_active) {
        static char buf[512];
        va_list ap2;
        va_start(ap2, fmt);
        int len = vsnprintf(buf, sizeof(buf), fmt, ap2);
        va_end(ap2);
        int limit = (len < (int)sizeof(buf)) ? len : (int)sizeof(buf) - 1;
        for (int i = 0; i < limit; i++)
            lcd_con_putchar(buf[i]);
    }
    return n;
}
#define printf kaitek_printf


/**
 * @brief Asynchronous snapshot-to-file phases (`cam snap`).
 */
enum cam_snap_state_t {
    CAM_SNAP_IDLE = 0,
    CAM_SNAP_WAIT_FRAME,
    CAM_SNAP_OPEN_FILE,
    CAM_SNAP_WRITE_FILE,
    CAM_SNAP_CLOSE_FILE,
};

/**
 * @brief Asynchronous LCD image load/unload phases.
 */
enum lcd_load_state_t{
    LCD_LOAD_IDLE = 0,
    LCD_LOAD_OPEN_FILE,
    LCD_LOAD_READ_FILE,
    LCD_LOAD_CLOSE_FILE,
    LCD_LOAD_DISPLAY,
};

/**
 * @brief One-shot camera frame to LCD snapshot phases.
 */
enum lcd_cam_snap_state_t {
    LCD_CAM_SNAP_IDLE = 0,
    LCD_CAM_SNAP_WAIT_FRAME,
    LCD_CAM_SNAP_DISPLAY_ROWS,
};

/**
 * @brief Continuous camera-to-LCD stream phases.
 */
enum lcd_cam_stream_state_t {
    LCD_CAM_STREAM_IDLE = 0,
    LCD_CAM_STREAM_WAIT_FRAME,
    LCD_CAM_STREAM_DISPLAY_ROWS,
};

/**
 * @brief High-level processing pipeline mode selector.
 */
enum pipeline_mode_t {
    PIPE_MODE_A_TOUCH = 0,   // CAM -> RP RAM -> LCD
    PIPE_MODE_B_NRF,         // CAM -> RP RAM -> NRF -> RP -> LCD (validation path)
};

/**
 * @brief Origin of mode-switch request for diagnostics and arbitration.
 */
enum pipeline_mode_request_source_t {
    PIPE_REQ_NONE = 0,
    PIPE_REQ_TOUCH,
    PIPE_REQ_NRF,
    PIPE_REQ_SHELL,
};

/**
 * @brief Logical ownership state of shared pins.
 */
enum shared_pin_state_t {
    SHARED_PIN_LCD_ACTIVE = 0,
    SHARED_PIN_NRF_TRANSACTION,
    SHARED_PIN_CAM_PROG,
};

static lcd_load_state_t lcd_load_state = LCD_LOAD_IDLE; // Current phase of the non-blocking LCD load/display state machine
static bool lcd_load_pending = false;                    // True while any lcd_load state machine phase is active
static FIL lcd_load_file;                                // FatFS file object for the image being loaded from SD
static bool lcd_load_file_open = false;                  // True when lcd_load_file is open and must be closed on finish/abort
static char lcd_load_path[256];                          // File path of the image currently being loaded
static char lcd_load_status_msg[128];                    // Optional status message printed on load completion (empty = default msg)
static UINT lcd_load_offset = 0;                         // Total bytes read from the file so far (tracks progress)
static uint32_t lcd_load_display_row = 0;                // Next row to send to the LCD panel (advances in lcd_display_row_chunk steps)
static absolute_time_t lcd_load_next_step_time;          // Earliest time the lcd_load state machine should execute its next step
static const uint32_t lcd_display_row_chunk = 16;        // Number of rows per background-task tick for lcd_load (SD read + LCD push)
static uint8_t lcd_row_buffer[LCD_2IN_WIDTH * 2 * lcd_display_row_chunk];  // Scratch buffer for one chunk of rows read from SD

static int spi_tx_dma_ch = -1;             // DMA channel for SPI TX (claimed on first use; -1 = not yet claimed)
static int spi_rx_dma_ch = -1;             // DMA channel for SPI RX drain (claimed on first use; -1 = not yet claimed)
static volatile uint16_t spi_rx_dummy;     // Discard sink for SPI RX DMA; prevents the 4-deep RX FIFO overflowing and stalling TX

static lcd_cam_snap_state_t lcd_cam_snap_state = LCD_CAM_SNAP_IDLE; // Current phase of the one-shot cam-to-LCD snapshot state machine
static bool lcd_cam_snap_pending = false;                            // True while an lcd_cam_snap operation is in progress
static uint32_t lcd_cam_snap_row = 0;                                // Next row to push to the LCD in the DISPLAY_ROWS phase
static absolute_time_t lcd_cam_snap_deadline;                        // Absolute time after which the frame-wait phase times out
static absolute_time_t lcd_cam_snap_next_step_time;                  // Earliest time for the next state machine step

static lcd_cam_stream_state_t lcd_cam_stream_state = LCD_CAM_STREAM_IDLE; // Current phase of the continuous camera stream state machine
static bool lcd_cam_stream_active = false;                                 // True while lcd_cam_stream is running
static uint32_t lcd_cam_stream_row = 0;                                    // Next row to process in the DISPLAY_ROWS phase
static absolute_time_t lcd_cam_stream_deadline;                            // Absolute time after which the frame-wait phase times out
static absolute_time_t lcd_cam_stream_next_step_time;                      // Earliest time for the next stream state machine step
static const uint8_t *mode_b_stream_locked_frame = NULL;  // Frame pointer locked at the start of a Mode-B transfer; prevents mid-frame pointer flip when camera DMA swaps buffers
static pipeline_mode_t pipeline_mode = PIPE_MODE_A_TOUCH;                     // Currently active pipeline mode
static pipeline_mode_t pipeline_mode_pending = PIPE_MODE_A_TOUCH;             // Target mode for a deferred switch request
static pipeline_mode_request_source_t pipeline_mode_pending_source = PIPE_REQ_NONE; // Originator of the pending switch request (for logging)
static bool pipeline_mode_switch_pending = false;                              // True when a mode switch has been requested but not yet applied
static bool pipeline_validation_mode_b_enabled = false;                        // When true, streaming uses the full NRF echo-validation path
static shared_pin_state_t shared_pin_state = SHARED_PIN_LCD_ACTIVE;            // Current logical owner of the shared SPI/I2C pins

static cam_snap_state_t cam_snap_state;       // Current phase of the cam snap state machine
static const UINT cam_snap_chunk_size = 512;  // Bytes written per background-task tick (matches one SD block for efficiency)

bool logger_enabled;                // When true, process_background_tasks() periodically logs a temperature sample
const uint32_t period = 1000;       // Logger sampling interval in milliseconds
absolute_time_t next_log_time;      // Absolute time when the next logger sample should be taken

#pragma GCC diagnostic ignored "-Wunused-function"
#ifdef NDEBUG 
#  pragma GCC diagnostic ignored "-Wunused-variable"
#  pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

/**
 * @brief Print "Missing argument" to signal the user didn't provide enough args.
 */
static void missing_argument_msg() {
    printf("Missing argument\n");
}
/**
 * @brief Print "Unexpected argument: <s>" to signal the user passed too many args.
 */
static void extra_argument_msg(const char *s) {
    printf("Unexpected argument: %s\n", s);
}
/**
 * @brief Validate that exactly `expected` arguments were provided; print an error and return false otherwise.
 *
 * @param argc Actual argument count.
 * @param argv Argument vector; argv[expected] is reported if too many args were given.
 * @param expected Required argument count.
 * @return true if argc == expected, false otherwise.
 */
static bool expect_argc(const size_t argc, const char *argv[], const size_t expected) {
    if (argc < expected) {
        missing_argument_msg();
        return false;
    }
    if (argc > expected) {
        extra_argument_msg(argv[expected]);
        return false;
    }
    return true;
}

/**
 * @brief Parse a decimal uint32 from a shell argument string; rejects empty, non-numeric, and trailing-garbage input.
 *
 * @param s   Null-terminated string to parse (e.g. "32000000").
 * @param out Receives the parsed value on success.
 * @return true on clean parse, false on NULL pointer, empty string, or any non-digit character.
 */
static bool parse_u32_arg(const char *s, uint32_t *out) {
    if (!s || !out) return false;
    char *end = NULL;
    unsigned long v = strtoul(s, &end, 10);
    // strtoul returns start pointer when no digits were consumed;
    // *end != '\0' means trailing garbage like "42abc".
    if (end == s || *end != '\0') return false;
    *out = (uint32_t)v;
    return true;
}

/**
 * @brief Convert a single hex character to its 4-bit integer value.
 *
 * Accepts '0'-'9', 'a'-'f', 'A'-'F'.  Returns -1 for any other character.
 */
static int hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    return -1;
}

/**
 * @brief Decode a contiguous hex string (e.g. "A5FF0102") into raw bytes.
 *
 * Requires an even-length, non-empty string whose decoded byte count fits in out_cap.
 * An odd-length hex string can't represent whole bytes; an odd count is therefore rejected.
 *
 * @param hex     Null-terminated hex string (case-insensitive).
 * @param out     Destination buffer.
 * @param out_cap Capacity of out in bytes.
 * @param out_len Receives the decoded byte count on success.
 * @return true on success, false on NULL, empty/odd-length string, invalid chars, or overflow.
 */
static bool parse_hex_string_bytes(const char *hex, uint8_t *out, size_t out_cap, size_t *out_len) {
    if (!hex || !out || !out_len) return false;
    size_t n = strlen(hex);
    // An empty or odd-length hex string can't represent a whole number of bytes.
    if (n == 0 || (n % 2u) != 0) return false;
    size_t bytes = n / 2u;
    if (bytes > out_cap) return false;

    for (size_t i = 0; i < bytes; ++i) {
        int hi = hex_nibble(hex[i * 2]);
        int lo = hex_nibble(hex[i * 2 + 1]);
        if (hi < 0 || lo < 0) return false;
        out[i] = (uint8_t)((hi << 4) | lo);
    }

    *out_len = bytes;
    return true;
}

/**
 * @brief Parse a list of space-separated hex byte tokens into a raw byte array.
 *
 * Each argv[i] must be a valid hex byte value in [0x00, 0xFF] with no extra characters.
 * Used by commands like `nrf xfer A5 FF 01` where bytes arrive as individual tokens.
 *
 * @param argc    Number of tokens (= expected output byte count).
 * @param argv    Array of token strings (each one a hex byte, e.g. "A5").
 * @param out     Destination buffer.
 * @param out_cap Capacity of out in bytes.
 * @param out_len Receives the parsed byte count on success.
 * @return true on success, false on NULL, zero tokens, overflow, or any invalid token.
 */
static bool parse_hex_tokens_bytes(const size_t argc, const char *argv[], uint8_t *out, size_t out_cap, size_t *out_len) {
    if (!out || !out_len) return false;
    if (argc == 0 || argc > out_cap) return false;

    for (size_t i = 0; i < argc; ++i) {
        char *end = NULL;
        unsigned long v = strtoul(argv[i], &end, 16);
        if (end == argv[i] || *end != '\0' || v > 0xFFu) {
            return false;
        }
        out[i] = (uint8_t)v;
    }
    *out_len = argc;
    return true;
}

/**
 * @brief Print a byte array as uppercase hex with single spaces between bytes (no trailing space or newline).
 */
static void print_hex_bytes(const uint8_t *buf, size_t len) {    for (size_t i = 0; i < len; ++i) {
        printf("%02X", buf[i]);
        if (i + 1 < len) printf(" ");
    }
}

static const char *pipeline_mode_name(const pipeline_mode_t mode) {
    return (mode == PIPE_MODE_B_NRF) ? "B (CAM->NRF->RP->LCD)" : "A (CAM->RP->LCD)";
}

static const char *pipeline_request_source_name(const pipeline_mode_request_source_t src) {
    switch (src) {
    case PIPE_REQ_TOUCH: return "touch";
    case PIPE_REQ_NRF: return "nrf";
    case PIPE_REQ_SHELL: return "shell";
    case PIPE_REQ_NONE:
    default: return "none";
    }
}

static const char *shared_pin_state_name(const shared_pin_state_t state) {
    switch (state) {
    case SHARED_PIN_NRF_TRANSACTION: return "NRF_TRANSACTION";
    case SHARED_PIN_CAM_PROG: return "CAM_PROG";
    case SHARED_PIN_LCD_ACTIVE:
    default: return "LCD_ACTIVE";
    }
}

/**
 * @brief Give the shared SPI bus back to the LCD driver.
 *
 * NRF transactions reconfigure SPI clock rate and GPIO functions.
 * This deasserts the NRF CS, restores the LCD SPI clock rate and pin functions,
 * then deasserts the LCD CS so the next LCD command starts from a clean state.
 */
static void shared_enter_lcd_active(void) {
    if (nrf_spi_initialized) {
        // cam_prog may have switched this pin back to I2C; force it to GPIO output first.
        gpio_init(nrf_spi_cs_pin);
        gpio_set_dir(nrf_spi_cs_pin, GPIO_OUT);
        gpio_put(nrf_spi_cs_pin, 1);
    }
    // Always restore SPI clock/data pin ownership and LCD SPI mode before LCD accesses.
    if (shared_pin_state != SHARED_PIN_LCD_ACTIVE) {
        // NRF may have left SPI at Mode 1 and/or a lower clock rate.
        (void)spi_init(SPI_PORT, lcd_spi_hz);
        gpio_set_function(LCD_CLK_PIN, GPIO_FUNC_SPI);
        gpio_set_function(LCD_MOSI_PIN, GPIO_FUNC_SPI);
        spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    }
    if (lcd_initialized) {
        DEV_Digital_Write(LCD_CS_PIN, 1);
    }
    shared_pin_state = SHARED_PIN_LCD_ACTIVE;
}

/**
 * @brief Configure shared SPI pins for an NRF transaction, deassert the LCD CS.
 *
 * Calling this before any NRF transfer ensures the LCD is not accidentally
 * selected and that SPI clock/mode match what the NRF expects.
 */
static void shared_enter_nrf_transaction(void) {
    if (shared_pin_state != SHARED_PIN_NRF_TRANSACTION || !nrf_spi_initialized) {
        nrf_spi_prepare_bus(nrf_spi_actual_hz ? nrf_spi_actual_hz : 15000000);
    }
    DEV_Digital_Write(LCD_CS_PIN, 1);
    shared_pin_state = SHARED_PIN_NRF_TRANSACTION;
}

/**
 * @brief Reclaim the I2C1 bus for OV5640 register access.
 *
 * nrf_spi_prepare_bus() repurposes I2C1_SDA as a GPIO output (NRF CS).
 * Before writing any camera register via i2c1, this must restore I2C1_SDA
 * to I2C function and re-enable its pull-up.
 */
static void shared_enter_cam_prog(void) {
    if (nrf_spi_initialized) {
        gpio_put(nrf_spi_cs_pin, 1);
        // nrf_spi_prepare_bus() reconfigured I2C1_SDA (== nrf_spi_cs_pin) to GPIO output.
        // Restore it to I2C function so camera register writes via i2c1 work correctly.
        gpio_set_function(I2C1_SDA, GPIO_FUNC_I2C);
        gpio_pull_up(I2C1_SDA);
    }
    shared_pin_state = SHARED_PIN_CAM_PROG;
}

/**
 * @brief Release the I2C1 bus after camera register access and restore the correct SPI owner.
 *
 * Returns ownership to NRF (Mode B active) or LCD (all other cases).
 */
static void shared_leave_cam_prog(void) {
    if (pipeline_mode == PIPE_MODE_B_NRF && pipeline_validation_mode_b_enabled) {
        shared_enter_nrf_transaction();
    } else {
        shared_enter_lcd_active();
    }
}

static void pipeline_request_mode(const pipeline_mode_t next_mode,
                                  const pipeline_mode_request_source_t source) {
    if (!lcd_cam_stream_active) {
        pipeline_mode = next_mode;
        pipeline_mode_switch_pending = false;
        pipeline_mode_pending_source = PIPE_REQ_NONE;
        return;
    }

    pipeline_mode_pending = next_mode;
    pipeline_mode_pending_source = source;
    pipeline_mode_switch_pending = true;
}

/**
 * @brief Apply a queued pipeline mode switch at the next safe moment.
 *
 * Mode switches requested during an active stream are deferred to a frame boundary
 * to avoid changing shared-pin ownership mid-transfer and corrupting the current frame.
 *
 */
static void pipeline_apply_pending_mode_if_safe(void) {
    // No request => nothing to do.
    if (!pipeline_mode_switch_pending) return;

    // If stream is idle, switch immediately.
    if (!lcd_cam_stream_active) {
        pipeline_mode = pipeline_mode_pending;
        pipeline_mode_switch_pending = false;
        pipeline_mode_pending_source = PIPE_REQ_NONE;
        return;
    }

    // While streaming, only switch at frame boundary to avoid mid-transfer pin ownership flips.
    if (lcd_cam_stream_state != LCD_CAM_STREAM_WAIT_FRAME) return;

    pipeline_mode_t old_mode = pipeline_mode;
    pipeline_mode = pipeline_mode_pending;
    // Leaving mode B invalidates partially assembled mode-B transfer state.
    if (old_mode == PIPE_MODE_B_NRF && pipeline_mode != PIPE_MODE_B_NRF) {
        reset_mode_b_transfer_ctx();
    }
    printf("\nPipeline switched to mode %s (requested by %s)\n",
           pipeline_mode_name(pipeline_mode),
           pipeline_request_source_name(pipeline_mode_pending_source));
    pipeline_mode_switch_pending = false;
    pipeline_mode_pending_source = PIPE_REQ_NONE;
    print_prompt();
}

static void pipeline_force_mode_now(const pipeline_mode_t mode,
                                    const pipeline_mode_request_source_t source) {
    (void)source;
    pipeline_mode_t old_mode = pipeline_mode;
    pipeline_mode = mode;
    pipeline_mode_pending = mode;
    pipeline_mode_switch_pending = false;
    pipeline_mode_pending_source = PIPE_REQ_NONE;
    if (old_mode == PIPE_MODE_B_NRF && mode != PIPE_MODE_B_NRF) {
        reset_mode_b_transfer_ctx();
    }
}

/**
 * @brief Toggle the OV5640 horizontal mirror bit and update cam_mirror_enabled.
 *
 * Must take/release I2C1 ownership around the register write because I2C1_SDA
 * is shared with the NRF CS GPIO.
 *
 */
static void cam_apply_mirror_toggle(void) {    shared_enter_cam_prog();
    cam_mirror_enabled = !cam_mirror_enabled;
    OV5640_WR_Reg(i2c1, 0x3C, TIMING_TC_REG21, cam_mirror_enabled ? 0x06 : 0x00);
    shared_leave_cam_prog();
}

/**
 * @brief Process an NRF button event bitmask received during a Mode-B frame transfer.
 *
 * A non-zero mask forces the pipeline into Mode B and executes the mapped action
 * (bit 0 = camera mirror toggle). Debounced by nrf_event_min_interval_ms to prevent
 * rapid repeat triggers from a single physical button press.
 *
 */
static void handle_nrf_button_event_mask(const uint8_t mask) {
    // Empty payload means no actionable event.
    if (mask == 0) return;
    // Debounce/guard repeated button notifications.
    if (!time_reached(nrf_event_block_until)) return;
    nrf_event_block_until = delayed_by_ms(get_absolute_time(), nrf_event_min_interval_ms);

    // Any NRF-origin event forces pipeline mode B.
    pipeline_force_mode_now(PIPE_MODE_B_NRF, PIPE_REQ_NRF);
    nrf_event_count++;

    // Validation mapping: bit0 toggles camera mirror.
    if (mask & 0x01) {
        cam_apply_mirror_toggle();
        printf("\nNRF event: mirror %s, mode B forced\n", cam_mirror_enabled ? "ON" : "OFF");
    } else {
        printf("\nNRF event mask=0x%02X, mode B forced\n", mask);
    }
    print_prompt();
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
static bool nrf_spi_transfer_bytes(const uint8_t *tx, uint8_t *rx, size_t len) {
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
static bool ensure_mode_b_frame_raw(size_t needed_bytes) {
    return (lcd_image != NULL) && (needed_bytes != 0) && (needed_bytes <= lcd_image_bytes);
}

/**
 * @brief Mark the Mode-B transfer context inactive so the buffer is no longer considered live.
 */
static void free_mode_b_frame_raw(void) {    mode_b_ctx.active = false;
}

/**
 * @brief Return true only if every byte in buf equals value.
 *
 * Used to detect all-0xFF NRF responses, which indicate a disconnected MISO line
 * or a slave that hasn't asserted CS yet (SPI bus floating high).
 */
static bool bytes_all_value(const uint8_t *buf, size_t len, uint8_t value) {    for (size_t i = 0; i < len; ++i) {
        if (buf[i] != value) return false;
    }
    return true;
}

/**
 * @brief Clear all fields of mode_b_ctx to their initial values so the next Mode-B frame starts from a known baseline.
 */
static void reset_mode_b_transfer_ctx(void) {
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

/**
 * @brief Send one camera frame through the NRF SPI loopback chunk-by-chunk, validate the echo, then push to LCD.
 *
 * Must be called repeatedly from the scheduler until it returns non-zero.
 * The NRF echoes each received chunk one transfer later (one-transfer pipeline delay), so each call
 * validates the echoed bytes against a fingerprint saved from the previous chunk.
 * On completion the validated frame is assembled in mode_b_frame_raw and displayed on the LCD.
 *
 * @return -1 on link error (all-0xFF, echo mismatch, or SPI failure), 0 still transferring, 1 frame displayed.
 */
static int pipeline_mode_b_transfer_frame_via_nrf_to_lcd(const uint8_t *frame) {
    if (!frame || !lcd_image) return -1;
    size_t frame_bytes = (size_t)cam_width * cam_height * 2u;
    if (!ensure_mode_b_frame_raw(frame_bytes)) {
        return -1;
    }
    uint8_t *mode_b_frame_raw = (uint8_t *)lcd_image;

    // Initialize/refresh transfer context on first entry for this frame pointer.
    if (!mode_b_ctx.active || mode_b_ctx.frame != frame) {
        reset_mode_b_transfer_ctx();
        mode_b_ctx.active = true;
        mode_b_ctx.frame = frame;
    }

    // Phase A: transfer one chunk and (after first pass) consume previous returned chunk.
    if (mode_b_ctx.row < cam_height) {
        uint32_t row = mode_b_ctx.row;
        uint32_t row_count = cam_height - row;
        if (row_count > nrf_validation_rows_per_chunk) {
            row_count = nrf_validation_rows_per_chunk;
        }
        size_t byte_count = (size_t)cam_width * 2u * row_count;
        if (byte_count > sizeof(nrf_validation_rx_chunk)) {
            // chunk size was set beyond NRF_VALIDATION_ROWS_MAX â€” reset and fail safely
            reset_mode_b_transfer_ctx();
            return -1;
        }
        const uint8_t *tx = frame + ((size_t)row * cam_width * 2u);

        // Snapshot a fingerprint now. Camera DMA can overwrite live frame memory later.
        uint8_t current_fp[16];
        memcpy(current_fp, tx, byte_count < 16u ? byte_count : 16u);

        // Send chunk to NRF and receive returned bytes.
        if (!nrf_spi_transfer_bytes(tx, nrf_validation_rx_chunk, byte_count)) {
            reset_mode_b_transfer_ctx();
            return -1;
        }

        // All-0xFF: nRF MISO high — slave not yet armed or recovering from I2C/CS disturbance.
        // On row 0 only: wait 2 ms for the nRF SPIS to re-arm and retry once at same speed.
        if (bytes_all_value(nrf_validation_rx_chunk, byte_count, 0xFF)) {
            if (row == 0) {
                busy_wait_us_32(2000);
                if (!nrf_spi_transfer_bytes(tx, nrf_validation_rx_chunk, byte_count) ||
                    bytes_all_value(nrf_validation_rx_chunk, byte_count, 0xFF)) {
                    printf("\nMode-B transfer got all-0xFF chunk at row 0 (SPI link issue)\n");
                    reset_mode_b_transfer_ctx();
                    return -1;
                }
                printf("\nMode-B row0 retry recovered (nRF re-armed)\n");
            } else {
                printf("\nMode-B transfer got all-0xFF chunk at row %lu (SPI link issue)\n",
                       (unsigned long)row);
                reset_mode_b_transfer_ctx();
                return -1;
            }
        }

        // Returned bytes correspond to previous chunk once the pipeline is primed.
        if (!mode_b_ctx.first_transfer) {
            // Optional side-band event marker from NRF in first two bytes.
            if (nrf_validation_rx_chunk[0] == 0xA5 && mode_b_ctx.prev_byte_count >= 2u) {
                uint8_t mask = nrf_validation_rx_chunk[1];
                nrf_validation_rx_chunk[0] = mode_b_ctx.prev_tx_fingerprint[0];
                nrf_validation_rx_chunk[1] = mode_b_ctx.prev_tx_fingerprint[1];
                if (mask != 0) {
                    handle_nrf_button_event_mask(mask);
                }
            }

            // Verify returned data matches the previous transmitted chunk fingerprint.
            size_t fp_cmp = mode_b_ctx.prev_byte_count < 16u ? mode_b_ctx.prev_byte_count : 16u;
            if (memcmp(nrf_validation_rx_chunk, mode_b_ctx.prev_tx_fingerprint, fp_cmp) != 0) {
                printf("\nMode-B echo mismatch at row %lu\n"
                       "  expected: %02X %02X %02X %02X  %02X %02X %02X %02X\n"
                       "  received: %02X %02X %02X %02X  %02X %02X %02X %02X\n",
                       (unsigned long)mode_b_ctx.prev_row,
                       mode_b_ctx.prev_tx_fingerprint[0], mode_b_ctx.prev_tx_fingerprint[1],
                       mode_b_ctx.prev_tx_fingerprint[2], mode_b_ctx.prev_tx_fingerprint[3],
                       mode_b_ctx.prev_tx_fingerprint[4], mode_b_ctx.prev_tx_fingerprint[5],
                       mode_b_ctx.prev_tx_fingerprint[6], mode_b_ctx.prev_tx_fingerprint[7],
                       nrf_validation_rx_chunk[0], nrf_validation_rx_chunk[1],
                       nrf_validation_rx_chunk[2], nrf_validation_rx_chunk[3],
                       nrf_validation_rx_chunk[4], nrf_validation_rx_chunk[5],
                       nrf_validation_rx_chunk[6], nrf_validation_rx_chunk[7]);
                reset_mode_b_transfer_ctx();
                return -1;
            }

            // Commit validated returned chunk into full-frame relay buffer.
            size_t prev_offset = (size_t)mode_b_ctx.prev_row * cam_width * 2u;
            memcpy(mode_b_frame_raw + prev_offset, nrf_validation_rx_chunk, mode_b_ctx.prev_byte_count);
        } else {
            // First transfer only primes the pipeline (no previous chunk to consume yet).
            mode_b_ctx.first_transfer = false;
        }

        // Advance chunk state for next scheduler tick.
        mode_b_ctx.prev_row = row;
        mode_b_ctx.prev_row_count = row_count;
        mode_b_ctx.prev_byte_count = byte_count;
        mode_b_ctx.prev_src = tx;
        memcpy(mode_b_ctx.prev_tx_fingerprint, current_fp, sizeof(current_fp));
        mode_b_ctx.row += row_count;
        return 0;
    }

    if (mode_b_ctx.first_transfer) {
        reset_mode_b_transfer_ctx();
        return -1;
    }

    // Phase B (flush): ask NRF to return the final delayed chunk.
    memset(nrf_validation_dummy_chunk, 0, sizeof(nrf_validation_dummy_chunk));
    if (!nrf_spi_transfer_bytes(nrf_validation_dummy_chunk, nrf_validation_rx_chunk, mode_b_ctx.prev_byte_count)) {
        reset_mode_b_transfer_ctx();
        return -1;
    }
    if (bytes_all_value(nrf_validation_rx_chunk, mode_b_ctx.prev_byte_count, 0xFF)) {
        printf("\nMode-B flush transfer got all-0xFF chunk (SPI link issue)\n");
        reset_mode_b_transfer_ctx();
        return -1;
    }

    if (nrf_validation_rx_chunk[0] == 0xA5 && mode_b_ctx.prev_byte_count >= 2u) {
        uint8_t mask = nrf_validation_rx_chunk[1];
        nrf_validation_rx_chunk[0] = mode_b_ctx.prev_tx_fingerprint[0];
        nrf_validation_rx_chunk[1] = mode_b_ctx.prev_tx_fingerprint[1];
        if (mask != 0) {
            handle_nrf_button_event_mask(mask);
        }
    }

    // Echo check against saved fingerprint (flush mirrors the per-chunk protocol).
    size_t flush_fp_cmp = mode_b_ctx.prev_byte_count < 16u ? mode_b_ctx.prev_byte_count : 16u;
    if (memcmp(nrf_validation_rx_chunk, mode_b_ctx.prev_tx_fingerprint, flush_fp_cmp) != 0) {
        printf("\nMode-B flush echo mismatch (MISO open or SPI error)\n");
        reset_mode_b_transfer_ctx();
        return -1;
    }

    size_t prev_offset = (size_t)mode_b_ctx.prev_row * cam_width * 2u;
    memcpy(mode_b_frame_raw + prev_offset, nrf_validation_rx_chunk, mode_b_ctx.prev_byte_count);

    shared_enter_lcd_active();
    lcd_display_raw_rows(0, cam_height, mode_b_frame_raw);
    reset_mode_b_transfer_ctx();
    return 1;
}

/**
 * @brief Read a single register from the CST816D touch controller over I2C0.
 *
 * Uses a repeated-start write (register address) followed by a read (1 byte).
 * Returns false if either the write or the read NAKs.
 */
static bool touch_read_reg(uint8_t reg, uint8_t *value) {
    if (!value) return false;
    int wr = i2c_write_blocking(I2C_PORT, touch_addr, &reg, 1, true);
    if (wr != 1) return false;
    int rd = i2c_read_blocking(I2C_PORT, touch_addr, value, 1, false);
    return rd == 1;
}

/**
 * @brief Initialise I2C0 and verify the CST816D touch chip is present by reading its chip-ID register.
 *
 * Hardware reset is intentionally skipped: Touch_RST_PIN is shared with LCD_RST/NRF MISO
 * on this board, so toggling it would corrupt an active LCD or NRF session.
 * The chip-ID read is sufficient to confirm the controller is powered and responsive.
 */
static bool touch_init_controller(void) {
    // Touch uses I2C0 on DEV_SDA/SCL; avoid hard reset because Touch_RST_PIN
    // is shared with LCD_RST/MISO on this board's wiring.
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(DEV_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(DEV_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(DEV_SDA_PIN);
    gpio_pull_up(DEV_SCL_PIN);

    gpio_init(Touch_INT_PIN);
    gpio_set_dir(Touch_INT_PIN, GPIO_IN);
    gpio_pull_up(Touch_INT_PIN);

    uint8_t chip = 0;
    if (!touch_read_reg(touch_reg_chip_id, &chip)) {
        touch_initialized = false;
        return false;
    }

    touch_initialized = (chip == touch_expected_chip_id);
    touch_press_latched = false;
    touch_debounce_deadline = get_absolute_time();
    touch_next_poll_time = get_absolute_time();
    return touch_initialized;
}

/**
 * @brief Poll the touch INT pin and, on a debounced press, force Mode A and toggle camera mirror.
 *
 * Called every frame from process_background_tasks(). Guards prevent running when
 * touch is not initialized, when no stream is active, or when the debounce window has not elapsed.
 *
 */
static void touch_handle_mirror_event_if_needed(void) {
    if (!touch_initialized) return;
    if (!lcd_cam_stream_active) return;
    if (!time_reached(touch_next_poll_time)) return;
    touch_next_poll_time = delayed_by_ms(get_absolute_time(), touch_poll_interval_ms);

    bool pressed = (gpio_get(Touch_INT_PIN) == 0);
    if (!pressed) {
        touch_press_latched = false;
        return;
    }
    if (touch_press_latched) return;
    if (!time_reached(touch_debounce_deadline)) return;
    touch_press_latched = true;
    touch_debounce_deadline = delayed_by_ms(get_absolute_time(), touch_debounce_ms);

    // Touch event => force Mode A immediately then apply one transformation (mirror toggle).
    pipeline_force_mode_now(PIPE_MODE_A_TOUCH, PIPE_REQ_TOUCH);
    cam_apply_mirror_toggle();
    touch_event_count++;
    printf("\nTouch event: mirror %s, mode A requested\n", cam_mirror_enabled ? "ON" : "OFF");
    print_prompt();
}

/**
 * @brief Periodically poll the NRF for a button event when not in Mode-B streaming.
 *
 * In Mode-B streaming, button events arrive embedded in frame-transfer echoes and are
 * handled by pipeline_mode_b_transfer_frame_via_nrf_to_lcd(). This function handles
 * the Mode-A case where no frame transfer is running and we need an explicit poll.
 *
 */
static void nrf_poll_button_event_if_needed(void) {
    if (!nrf_spi_initialized) return;
    if (!lcd_cam_stream_active) return;
    // Skip standalone NRF polling during Mode-B streaming — events arrive embedded in transfer echoes instead.
    if (pipeline_mode == PIPE_MODE_B_NRF && pipeline_validation_mode_b_enabled) return;
    if (!time_reached(nrf_event_block_until)) return;
    if (!time_reached(nrf_event_poll_next_time)) return;
    nrf_event_poll_next_time = delayed_by_ms(get_absolute_time(), nrf_event_poll_interval_ms);

    uint8_t tx[2] = {0x00, 0x00};
    uint8_t rx[2] = {0x00, 0x00};
    if (!nrf_spi_transfer_bytes(tx, rx, sizeof(tx))) {
        return;
    }

    if (rx[0] == 0xA5 && rx[1] != 0x00 && rx[1] != 0xFF) {
        handle_nrf_button_event_mask(rx[1]);
    }
}

const char *chk_dflt_log_drv(const size_t argc, const char *argv[]) {
    if (argc > 1) {
        extra_argument_msg(argv[1]);
        return NULL;
    }
    if (!argc) {
        if (1 == sd_get_num()) {
            return sd_get_drive_prefix(sd_get_by_num(0));
        } else {
            printf("Missing argument: Specify logical drive\n");
            return NULL;
        }        
    }
    return argv[0];
}
/**
 * @brief Print the current wall-clock date, time, and day-of-year from the AON RTC.
 */
static void run_date(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;

    char buf[128] = {0};
    time_t epoch_secs = time(NULL);
    if (epoch_secs < 1) {
        printf("RTC not running\n");
        return;
    }
    struct tm *ptm = localtime(&epoch_secs);
    assert(ptm);
    size_t n = strftime(buf, sizeof(buf), "%c", ptm);
    assert(n);
    printf("%s\n", buf);
    strftime(buf, sizeof(buf), "%j",
             ptm);  // The day of the year as a decimal number (range
                    // 001 to 366).
    printf("Day of year: %s\n", buf);
}
/**
 * @brief Set the AON RTC from six user-supplied fields (day month year hour min sec).
 */
static void run_setrtc(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 6)) return;

    int date = atoi(argv[0]);
    int month = atoi(argv[1]);
    int year = atoi(argv[2]) + 2000;
    int hour = atoi(argv[3]);
    int min = atoi(argv[4]);
    int sec = atoi(argv[5]);

    struct tm t = {
        // tm_sec	int	seconds after the minute	0-61*
        .tm_sec = sec,
        // tm_min	int	minutes after the hour	0-59
        .tm_min = min,
        // tm_hour	int	hours since midnight	0-23
        .tm_hour = hour,
        // tm_mday	int	day of the month	1-31
        .tm_mday = date,
        // tm_mon	int	months since January	0-11
        .tm_mon = month - 1,
        // tm_year	int	years since 1900
        .tm_year = year - 1900,
        // tm_wday	int	days since Sunday	0-6
        .tm_wday = 0,
        // tm_yday	int	days since January 1	0-365
        .tm_yday = 0,
        // tm_isdst	int	Daylight Saving Time flag
        .tm_isdst = 0
    };
    /* The values of the members tm_wday and tm_yday of timeptr are ignored, and the values of
       the other members are interpreted even if out of their valid ranges */
    time_t epoch_secs = mktime(&t);
    if (-1 == epoch_secs) {
        printf("The passed in datetime was invalid\n");
        return;
    }
    struct timespec ts = {.tv_sec = epoch_secs, .tv_nsec = 0};
    aon_timer_set_time(&ts);
}
static char const *fs_type_string(int fs_type) {
    switch (fs_type) {
        case FS_FAT12:
            return "FAT12";
        case FS_FAT16:
            return "FAT16";
        case FS_FAT32:
            return "FAT32";
        case FS_EXFAT:
            return "EXFAT";
        default:
            return "Unknown";
    }
}
static const char *sd_if_string(sd_if_t type) {
    switch (type) {
        case SD_IF_SPI:
            return "SPI";
        case SD_IF_SDIO:
            return "SDIO";
        default:
            return "NONE";
    }
}
static const char *sd_card_type_string(card_type_t type) {
    switch (type) {
        case SDCARD_NONE:
            return "None";
        case SDCARD_V1:
            return "SDv1";
        case SDCARD_V2:
            return "SDv2";
        case SDCARD_V2HC:
            return "SDv2HC";
        default:
            return "Unknown";
    }
}
/**
 * @brief Print SD card identity, FatFS volume geometry, free space, and layout details for a logical drive.
 */
static void run_info(const size_t argc, const char *argv[]) {
    const char *arg = chk_dflt_log_drv(argc, argv);
    if (!arg)
        return;
    sd_card_t *sd_card_p = sd_get_by_drive_prefix(arg);
    if (!sd_card_p) {
        printf("Unknown logical drive id: \"%s\"\n", arg);
        return;
    }
    int ds = sd_card_p->init(sd_card_p);
    if (STA_NODISK & ds || STA_NOINIT & ds) {
        printf("SD card initialization failed\n");
        return;
    }
    printf("Drive %s interface=%s card_type=%s status=0x%02x sectors=%lu\n",
           arg, sd_if_string(sd_card_p->type), sd_card_type_string(sd_card_p->state.card_type),
           sd_card_p->state.m_Status, (unsigned long)sd_card_p->state.sectors);
    // Card IDendtification register. 128 buts wide.
    cidDmp(sd_card_p, printf);
    // Card-Specific Data register. 128 bits wide.
    csdDmp(sd_card_p, printf);
    
    // SD Status
    size_t au_size_bytes;
    bool ok = sd_allocation_unit(sd_card_p, &au_size_bytes);
    if (ok)
        printf("\nSD card Allocation Unit (AU_SIZE) or \"segment\": %zu bytes (%zu sectors)\n", 
            au_size_bytes, au_size_bytes / sd_block_size);
    
    if (!sd_card_p->state.mounted) {
        printf("Drive \"%s\" is not mounted\n", argv[0]);
        return;
    }

    /* Get volume information and free clusters of drive */
    FATFS *fs_p = &sd_card_p->state.fatfs;
    if (!fs_p) {
        printf("Unknown logical drive id: \"%s\"\n", arg);
        return;
    }
    DWORD fre_clust, fre_sect, tot_sect;
    FRESULT fr = f_getfree(arg, &fre_clust, &fs_p);
    if (FR_OK != fr) {
        printf("f_getfree error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    /* Get total sectors and free sectors */
    tot_sect = (fs_p->n_fatent - 2) * fs_p->csize;
    fre_sect = fre_clust * fs_p->csize;
    /* Print the free space (assuming 512 bytes/sector) */
    printf("\n%10lu KiB (%lu MiB) total drive space.\n%10lu KiB (%lu MiB) available.\n",
           tot_sect / 2, tot_sect / 2 / 1024,
           fre_sect / 2, fre_sect / 2 / 1024);

#if FF_USE_LABEL
    // Report label:
    TCHAR buf[34] = {};/* [OUT] Volume label */
    DWORD vsn;         /* [OUT] Volume serial number */
    fr = f_getlabel(arg, buf, &vsn);
    if (FR_OK != fr) {
        printf("f_getlabel error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    } else {
        printf("\nVolume label: %s\nVolume serial number: %lu\n", buf, vsn);
    }
#endif

    // Report format
    printf("\nFilesystem type: %s\n", fs_type_string(fs_p->fs_type));

    // Report Partition Starting Offset
    // uint64_t offs = fs_p->volbase;
    // printf("Partition Starting Offset: %llu sectors (%llu bytes)\n",
    //         offs, offs * sd_block_size);
	printf("Volume base sector: %llu\n", fs_p->volbase);		
	printf("FAT base sector: %llu\n", fs_p->fatbase);		
	printf("Root directory base sector (FAT12/16) or cluster (FAT32/exFAT): %llu\n", fs_p->dirbase);		 
	printf("Data base sector: %llu\n", fs_p->database);		

    // Report cluster size ("allocation unit")
    printf("FAT Cluster size (\"allocation unit\"): %d sectors (%llu bytes)\n",
           fs_p->csize,
           (uint64_t)sd_card_p->state.fatfs.csize * FF_MAX_SS);
}
/**
 * @brief Format a logical drive with FAT/exFAT, aligning the partition to the SD card's erase unit.
 */
static void run_format(const size_t argc, const char *argv[]) {
    const char *arg = chk_dflt_log_drv(argc, argv);
    if (!arg)
        return;
    sd_card_t *sd_card_p = sd_get_by_drive_prefix(arg);
    if (!sd_card_p) {
        printf("Unknown logical drive id: \"%s\"\n", arg);
        return;
    }
    int ds;
    for (int attempt = 1; attempt <= 3; ++attempt) {
        if (attempt > 1)
            sleep_ms(2000);
        ds = sd_card_p->init(sd_card_p);
        if (!(STA_NODISK & ds) && !(STA_NOINIT & ds))
            break;
        printf("SD init attempt %d failed (0x%02x), retrying...\n", attempt, ds);
    }
    if (STA_NODISK & ds || STA_NOINIT & ds) {
        printf("SD card initialization failed, status=0x%02x\n", ds);
        return;
    }
    printf("Drive %s interface=%s card_type=%s status=0x%02x sectors=%lu\n",
           arg, sd_if_string(sd_card_p->type), sd_card_type_string(sd_card_p->state.card_type),
           sd_card_p->state.m_Status, (unsigned long)sd_card_p->state.sectors);
    
    /* I haven't been able to find a way to obtain the layout produced
    by the SD Association's "SD Memory Card Formatter"
    (https://www.sdcard.org/downloads/formatter/).

    SD Memory Card Formatter:
    Volume base sector: 8192
    FAT base sector: 8790
    Root directory base sector (FAT12/16) or cluster (FAT32/exFAT): 2
    Data base sector: 16384
    FAT Cluster size ("allocation unit"): 64 sectors (32768 bytes)

    f_mkfs:
    Volume base sector: 63
    FAT base sector: 594
    Root directory base sector (FAT12/16) or cluster (FAT32/exFAT): 2
    Data base sector: 8192
    FAT Cluster size ("allocation unit"): 64 sectors (32768 bytes)    
    */

    /* Attempt to align partition to SD card segment (AU) */
    size_t au_size_bytes;
    bool ok = sd_allocation_unit(sd_card_p, &au_size_bytes);
    if (!ok || !au_size_bytes)
        au_size_bytes = 4194304; // Default to 4 MiB
    UINT n_align = au_size_bytes / sd_block_size;

    MKFS_PARM opt = {
        FM_ANY,  /* Format option (FM_FAT, FM_FAT32, FM_EXFAT and FM_SFD) */
        2,       /* Number of FATs */
        n_align, /* Data area alignment (sector) */
        0,       /* Number of root directory entries */
        0        /* Cluster size (byte) */
    };
    /* Format the drive */
    printf("Formatting %s (this may take a while)...\n", arg);
    FRESULT fr = f_mkfs(arg, &opt, 0, 32 * 1024);  // 32 KB: ~32x fewer disk_write calls vs FF_MAX_SS*2
    if (FR_OK != fr) {
        printf("f_mkfs error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }

    /* Re-init the card — f_mkfs leaves it in a post-write idle state that
     * confuses a subsequent cold init in run_mount. */
    sd_card_p->state.m_Status |= STA_NOINIT;
    sd_card_p->state.mounted = false;
    ds = sd_card_p->init(sd_card_p);
    if (STA_NODISK & ds || STA_NOINIT & ds) {
        printf("Re-init after format failed (0x%02x)\n", ds);
        return;
    }

    FATFS *fs_p = &sd_card_p->state.fatfs;
    fr = f_mount(fs_p, arg, 1);
    if (FR_OK != fr) {
        printf("Mount after format failed: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    sd_card_p->state.mounted = true;

#if FF_USE_LABEL
    TCHAR label[32];
    snprintf(label, sizeof(label), "%sFatFS", arg);
    f_setlabel(label);
#endif
    printf("Format and mount complete.\n");
}
/**
 * @brief Initialise an SD card and mount its FatFS volume, making it accessible for file commands.
 */
static void run_mount(const size_t argc, const char *argv[]) {
    const char *arg = chk_dflt_log_drv(argc, argv);
    if (!arg)
        return;
    sd_card_t *sd_card_p = sd_get_by_drive_prefix(arg);
    if (!sd_card_p) {
        printf("Unknown logical drive id: \"%s\"\n", arg);
        return;
    }
    int ds = sd_card_p->init(sd_card_p);
    if (STA_NODISK & ds || STA_NOINIT & ds) {
        printf("SD card initialization failed, status=0x%02x\n", ds);
        return;
    }
    printf("Drive %s interface=%s card_type=%s status=0x%02x sectors=%lu\n",
           arg, sd_if_string(sd_card_p->type), sd_card_type_string(sd_card_p->state.card_type),
           sd_card_p->state.m_Status, (unsigned long)sd_card_p->state.sectors);
    fflush(stdout);
    stdio_flush();
    printf("f_mount: begin\n");
    fflush(stdout);
    stdio_flush();
    FATFS *fs_p = &sd_card_p->state.fatfs;
    FRESULT fr = f_mount(fs_p, arg, 1);
    printf("f_mount: returned %s (%d)\n", FRESULT_str(fr), fr);
    fflush(stdout);
    stdio_flush();
    if (FR_OK != fr) {
        return;
    }
    sd_card_p->state.mounted = true;
}
/**
 * @brief Unmount a FatFS volume and mark the SD card as uninitialized so it is re-probed on next access.
 */
static void run_unmount(const size_t argc, const char *argv[]) {
    const char *arg = chk_dflt_log_drv(argc, argv);
    if (!arg)
        return;

    sd_card_t *sd_card_p = sd_get_by_drive_prefix(arg);
    if (!sd_card_p) {
        printf("Unknown logical drive id: \"%s\"\n", arg);
        return;
    }    
    FRESULT fr = f_unmount(arg);
    if (FR_OK != fr) {
        printf("f_unmount error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    sd_card_p->state.mounted = false;
    sd_card_p->state.m_Status |= STA_NOINIT;  // in case medium is removed
}
/**
 * @brief Set the FatFS current logical drive (e.g. "0:") so subsequent relative paths resolve to it.
 */
static void run_chdrive(const size_t argc, const char *argv[]) {
    const char *arg = chk_dflt_log_drv(argc, argv);
    if (!arg)
        return;

    FRESULT fr = f_chdrive(arg);
    if (FR_OK != fr) printf("f_chdrive error: %s (%d)\n", FRESULT_str(fr), fr);
}
/**
 * @brief Change the FatFS current working directory to the given path.
 */
static void run_cd(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;

    FRESULT fr = f_chdir(argv[0]);
    if (FR_OK != fr) printf("f_chdir error: %s (%d)\n", FRESULT_str(fr), fr);
}
/**
 * @brief Create a new directory at the given FatFS path.
 */
static void run_mkdir(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;

    FRESULT fr = f_mkdir(argv[0]);
    if (FR_OK != fr) printf("f_mkdir error: %s (%d)\n", FRESULT_str(fr), fr);
}
/**
 * @brief List files and subdirectories in the given path, or in the current working directory if omitted.
 */
static void run_ls(const size_t argc, const char *argv[]) {
    if (argc > 1) {
        extra_argument_msg(argv[1]);
        return;
    }
    if (argc)
        ls(argv[0]);
    else
        ls("");
}
/**
 * @brief Print the FatFS current working directory path.
 */
static void run_pwd(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;

    char buf[512];
    FRESULT fr = f_getcwd(buf, sizeof buf);
    if (FR_OK != fr)
        printf("f_getcwd error: %s (%d)\n", FRESULT_str(fr), fr);
    else
        printf("%s", buf);
}
/**
 * @brief Print the contents of a FatFS file to stdout, reading line by line.
 */
static void run_cat(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;

    FIL fil;
    FRESULT fr = f_open(&fil, argv[0], FA_READ);
    if (FR_OK != fr) {
        printf("f_open error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    char buf[256];
    while (f_gets(buf, sizeof buf, &fil)) {
        printf("%s", buf);
    }
    fr = f_close(&fil);
    if (FR_OK != fr) printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
}
/**
 * @brief Copy a file on the FatFS volume, reading up to 32 KiB at a time to limit heap use.
 */
static void run_cp(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 2)) return;

    FIL fsrc, fdst;    /* File objects */
    FRESULT fr;        /* FatFs function common result code */
    UINT br, bw;       /* File read/write count */

    /* Open source file on the drive 1 */
    fr = f_open(&fsrc, argv[0], FA_READ);
    if (FR_OK != fr) {
        printf("f_open error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    /* Create destination file on the drive 0 */
    fr = f_open(&fdst, argv[1], FA_WRITE | FA_CREATE_ALWAYS);
    if (FR_OK != fr) {
        printf("f_open error: %s (%d)\n", FRESULT_str(fr), fr);
        f_close(&fsrc);        
        return;
    }
    /* Copy source to destination */
    FSIZE_t buffer_sz = f_size(&fsrc);
    if (buffer_sz > 32768)
        buffer_sz = 32768;
    /* File copy buffer */
    BYTE *buffer = (BYTE *)malloc(buffer_sz);
    if (!buffer) {
        printf("malloc(%llu) failed\n", buffer_sz);
        f_close(&fdst);
        f_close(&fsrc);
        return;
    }
    for (;;) {
        fr = f_read(&fsrc, buffer, buffer_sz, &br); /* Read a chunk of data from the source file */
        if (FR_OK != fr) printf("f_read error: %s (%d)\n", FRESULT_str(fr), fr);
        if (br == 0) break;                   /* error or eof */
        fr = f_write(&fdst, buffer, br, &bw); /* Write it to the destination file */
        if (FR_OK != fr) printf("f_write error: %s (%d)\n", FRESULT_str(fr), fr);
        if (bw < br) break; /* error or disk full */
    }
    free(buffer);
    /* Close open files */
    fr = f_close(&fsrc);
    if (FR_OK != fr) printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
    fr = f_close(&fdst);
    if (FR_OK != fr) printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
}
/**
 * @brief Rename or move a file or directory on the FatFS volume.
 */
static void run_mv(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 2)) return;

    FRESULT fr = f_rename(argv[0], argv[1]);
    if (FR_OK != fr) printf("f_rename error: %s (%d)\n", FRESULT_str(fr), fr);
}
/**
 * @brief Run the destructive low-level I/O driver test on a physical drive number; SD must be reformatted after.
 */
static void run_lliot(const size_t argc, const char *argv[]) {
    const char *arg = chk_dflt_log_drv(argc, argv);
    if (!arg)
        return;

    size_t pnum = 0;
    pnum = strtoul(arg, NULL, 0);
    lliot(pnum);
}
/**
 * @brief Write a large random-data file of a given size and seed, then verify it; useful for SD stress testing.
 */
static void run_big_file_test(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 3)) return;

    const char *pcPathName = argv[0];
    size_t size = strtoul(argv[1], 0, 0);
    uint32_t seed = atoi(argv[2]);
    big_file_test(pcPathName, size, seed);
}
/**
 * @brief Recursively delete a directory and all its contents from the FatFS volume.
 *
 */
static void del_node(const char *path) {
    FILINFO fno;
    char buff[256];
    /* Directory to be deleted */
    strlcpy(buff, path, sizeof(buff));
    /* Delete the directory */
    FRESULT fr = delete_node(buff, sizeof buff / sizeof buff[0], &fno);
    /* Check the result */
    if (fr) {
        printf("Failed to delete the directory %s. ", path);
        printf("%s error: %s (%d)\n", __func__, FRESULT_str(fr), fr);
    }
}
/**
 * @brief Recursively delete the given directory path and all its contents.
 */
static void run_del_node(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;
    del_node(argv[0]);
}
/**
 * @brief Remove a file, empty directory (-d), or directory tree (-r) from the FatFS volume.
 */
static void run_rm(const size_t argc, const char *argv[]) {
    if (argc < 1) {
        missing_argument_msg();
        return;
    }
    if (argc > 2) {
        extra_argument_msg(argv[2]);
        return;
    }
    if (2 == argc) {
        if (0 == strcmp("-r", argv[0])) {
            del_node(argv[1]);
        } else if (0 == strcmp("-d", argv[0])) {
            FRESULT fr = f_unlink(argv[1]);
            if (FR_OK != fr) printf("f_unlink error: %s (%d)\n", FRESULT_str(fr), fr);
        } else {
            EMSG_PRINTF("Unknown option: %s\n", argv[0]);
        }
    } else {
        FRESULT fr = f_unlink(argv[0]);
        if (FR_OK != fr) printf("f_unlink error: %s (%d)\n", FRESULT_str(fr), fr);
    }
}
/**
 * @brief Run the simple FatFS filesystem self-test suite.
 */
static void run_simple(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;

    simple();
}
/**
 * @brief Run the binary write/read throughput benchmark on a logical drive.
 */
static void run_bench(const size_t argc, const char *argv[]) {
    const char *arg = chk_dflt_log_drv(argc, argv);
    if (!arg)
        return;

    bench(arg);
}
/**
 * @brief Create the /cdef fake mount-point and populate it with FatFS example files.
 */
static void run_cdef(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;

    f_mkdir("/cdef");  // fake mountpoint
    vCreateAndVerifyExampleFiles("/cdef");
}
/**
 * @brief Run the stdio-with-current-working-directory test against /cdef (run cdef first).
 */
static void run_swcwdt(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;

    vStdioWithCWDTest("/cdef");
}
/**
 * @brief Repeatedly run cdef + swcwdt in a loop until Enter is pressed (stress test).
 */
static void run_loop_swcwdt(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;

    die = false;
    do {
        f_chdir("/");
        del_node("/cdef");
        f_mkdir("/cdef");  // fake mountpoint
        vCreateAndVerifyExampleFiles("/cdef");
        vStdioWithCWDTest("/cdef");
    } while (!die);
}
/**
 * @brief Enable the periodic temperature logger: initialises the ADC, turns on the internal sensor, and sets the next sample time.
 */
static void run_start_logger(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;
    adc_init();
    adc_set_temp_sensor_enabled(true);
    logger_enabled = true;
    next_log_time = delayed_by_ms(get_absolute_time(), period);
}
/**
 * @brief Disable the periodic temperature logger by clearing logger_enabled.
 */
static void run_stop_logger(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;

    logger_enabled = false;
}
/**
 * @brief Print heap memory statistics (total, used, free) via malloc_stats().
 */
static void run_mem_stats(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;
 
    // extern ptrdiff_t __StackTop, __StackBottom, __StackOneTop, __StackOneBottom;
    // printf("__StackTop - __StackBottom = %zu\n", __StackTop - __StackBottom);
    // printf("__StackOneTop - __StackOneBottom = %zu\n", __StackOneTop - __StackOneBottom);

    malloc_stats();
}

/* Derived from pico-examples/clocks/hello_48MHz/hello_48MHz.c */
/**
 * @brief Count and print all measurable RP2350 clock frequencies using the hardware frequency counter.
 */
static void run_measure_freqs(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;

    uint f_pll_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
    uint f_pll_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_USB_CLKSRC_PRIMARY);
    uint f_rosc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC);
    uint f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
    uint f_clk_peri = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_PERI);
    uint f_clk_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_USB);
    uint f_clk_adc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_ADC);
#if PICO_RP2040
    uint f_clk_rtc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_RTC);
#endif

    printf("pll_sys  = %dkHz\n", f_pll_sys);
    printf("pll_usb  = %dkHz\n", f_pll_usb);
    printf("rosc     = %dkHz\n", f_rosc);
    printf("clk_sys  = %dkHz\treported  = %lukHz\n", f_clk_sys, clock_get_hz(clk_sys) / KHZ);
    printf("clk_peri = %dkHz\treported  = %lukHz\n", f_clk_peri, clock_get_hz(clk_peri) / KHZ);
    printf("clk_usb  = %dkHz\treported  = %lukHz\n", f_clk_usb, clock_get_hz(clk_usb) / KHZ);
    printf("clk_adc  = %dkHz\treported  = %lukHz\n", f_clk_adc, clock_get_hz(clk_adc) / KHZ);
#if PICO_RP2040
    printf("clk_rtc  = %dkHz\treported  = %lukHz\n", f_clk_rtc, clock_get_hz(clk_rtc) / KHZ);
#endif

    // Can't measure clk_ref / xosc as it is the ref
}
/**
 * @brief Switch the system clock to 48 MHz and reinitialise the default UART.
 */
static void run_set_sys_clock_48mhz(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;

    set_sys_clock_48mhz();
    setup_default_uart();
}
/**
 * @brief Change the system clock to a user-supplied kHz frequency and re-source clk_peri from clk_sys.
 */
static void run_set_sys_clock_khz(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;

    int khz = atoi(argv[0]);

    bool configured = set_sys_clock_khz(khz, false);
    if (!configured) {
        printf("Not possible. Clock not configured.\n");
        return;
    }
    /*
    By default, when reconfiguring the system clock PLL settings after runtime initialization,
    the peripheral clock is switched to the 48MHz USB clock to ensure continuity of peripheral operation.
    There seems to be a problem with running the SPI 2.4 times faster than the system clock,
    even at the same SPI baud rate.
    Anyway, for now, reconfiguring the peripheral clock to the system clock at its new frequency works OK.
    */
    bool ok = clock_configure(clk_peri,
                              0,
                              CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
                              clock_get_hz(clk_sys),
                              clock_get_hz(clk_sys));
    assert(ok);

    setup_default_uart();
}
/**
 * @brief Drive the given GPIO pin high (init, direction out, set 1).
 */
static void set(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;

    int gp = atoi(argv[0]);

    gpio_init(gp);
    gpio_set_dir(gp, GPIO_OUT);
    gpio_put(gp, 1);
}
/**
 * @brief Drive the given GPIO pin low (init, direction out, set 0).
 */
static void clr(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;

    int gp = atoi(argv[0]);

    gpio_init(gp);
    gpio_set_dir(gp, GPIO_OUT);
    gpio_put(gp, 0);
}
/**
 * @brief Invoke the development test routine my_test() (used for ad-hoc in-tree testing).
 */
static void run_test(const size_t argc, const char *argv[]) {
        if (!expect_argc(argc, argv, 0)) return;

    extern bool my_test();
    my_test();
}

static void run_help(const size_t argc, const char *argv[]);

/**
 * @brief Print the shell prompt string and flush stdout.
 *
 */
static void print_prompt(void) {
    printf("%s", PROMPT_STR);
    stdio_flush();
}

/**
 * @brief Reset command line state.
 *
 */
static void reset_command_line_state(void) {    cmd_ix = 0;
    cmd_cursor = 0;
    cmd_buffer[0] = '\0';
    cmd_history_nav_index = -1;
    cmd_history_backup_valid = false;
    cmd_history_edit_backup_len = 0;
    cmd_history_edit_backup[0] = '\0';
    tab_was_last_key = false;
}

/**
 * @brief Redraw the current command line in-place using CR + ANSI erase-to-EOL, preserving cursor position.
 *
 */
static void redraw_command_line(void) {
    printf("\r%s%s", PROMPT_STR, cmd_buffer);
    printf("\x1b[K");
    size_t tail = cmd_ix - cmd_cursor;
    if (tail > 0) {
        printf("\x1b[%uD", (unsigned)tail);
    }
    stdio_flush();
}

/**
 * @brief Append a command line to the history ring, silently dropping duplicates of the most recent entry.
 *
 * When the ring is full (CMD_HISTORY_MAX entries), the oldest entry is evicted to make room.
 *
 */
static void add_command_to_history(const char *line) {
    if (!line || !line[0]) return;
    if (cmd_history_count > 0 &&
        strcmp(cmd_history[cmd_history_count - 1], line) == 0) {
        return;
    }

    if (cmd_history_count < CMD_HISTORY_MAX) {
        strlcpy(cmd_history[cmd_history_count], line, sizeof(cmd_history[0]));
        cmd_history_count++;
        return;
    }

    memmove(&cmd_history[0], &cmd_history[1], sizeof(cmd_history[0]) * (CMD_HISTORY_MAX - 1));
    strlcpy(cmd_history[CMD_HISTORY_MAX - 1], line, sizeof(cmd_history[0]));
}

/**
 * @brief Snapshot the current in-progress command line before history navigation overwrites it.
 *
 * Allows Down-arrow at the bottom of history to restore the line the user was typing.
 *
 */
static void load_current_edit_backup(void) {
    strlcpy(cmd_history_edit_backup, cmd_buffer, sizeof(cmd_history_edit_backup));
    cmd_history_edit_backup_len = cmd_ix;
    cmd_history_backup_valid = true;
}

/**
 * @brief Replace the command buffer with text and move the cursor to the end.
 *
 */
static void set_command_line_text(const char *text) {
    if (!text) text = "";
    strlcpy(cmd_buffer, text, sizeof(cmd_buffer));
    cmd_ix = strlen(cmd_buffer);
    cmd_cursor = cmd_ix;
}

/**
 * @brief Move one step back in history (older), snapshotting the in-progress edit on first use.
 *
 */
static void history_nav_up(void) {
    if (cmd_history_count == 0) return;

    if (cmd_history_nav_index < 0) {
        load_current_edit_backup();
        cmd_history_nav_index = (int)cmd_history_count - 1;
    } else if (cmd_history_nav_index > 0) {
        cmd_history_nav_index--;
    }

    set_command_line_text(cmd_history[cmd_history_nav_index]);
    redraw_command_line();
}

/**
 * @brief Move one step forward in history (newer); restores the saved in-progress edit at the bottom.
 *
 */
static void history_nav_down(void) {
    if (cmd_history_nav_index < 0) return;

    if (cmd_history_nav_index < (int)cmd_history_count - 1) {
        cmd_history_nav_index++;
        set_command_line_text(cmd_history[cmd_history_nav_index]);
    } else {
        cmd_history_nav_index = -1;
        if (cmd_history_backup_valid) {
            set_command_line_text(cmd_history_edit_backup);
        } else {
            set_command_line_text("");
        }
    }
    redraw_command_line();
}

/**
 * @brief Exit history navigation mode without restoring the edit backup (called on any typed character).
 *
 */
static void leave_history_navigation_mode(void) {
    if (cmd_history_nav_index >= 0) {
        cmd_history_nav_index = -1;
        cmd_history_backup_valid = false;
    }
}

/**
 * @brief Delete the character to the left of the cursor (Backspace / ^H).
 *
 */
static void handle_backspace(void) {
    if (cmd_cursor == 0) return;
    leave_history_navigation_mode();
    memmove(&cmd_buffer[cmd_cursor - 1], &cmd_buffer[cmd_cursor], cmd_ix - cmd_cursor + 1);
    cmd_cursor--;
    cmd_ix--;
    redraw_command_line();
}

/**
 * @brief Delete the character under the cursor (Delete key / ^D).
 *
 */
static void handle_delete_at_cursor(void) {
    if (cmd_cursor >= cmd_ix) return;
    leave_history_navigation_mode();
    memmove(&cmd_buffer[cmd_cursor], &cmd_buffer[cmd_cursor + 1], cmd_ix - cmd_cursor);
    cmd_ix--;
    redraw_command_line();
}

/**
 * @brief Insert a printable character at the cursor, shifting the rest of the line right.
 *
 */
static void handle_insert_char(char ch) {
    if (cmd_ix >= sizeof(cmd_buffer) - 1) return;
    leave_history_navigation_mode();
    memmove(&cmd_buffer[cmd_cursor + 1], &cmd_buffer[cmd_cursor], cmd_ix - cmd_cursor + 1);
    cmd_buffer[cmd_cursor] = ch;
    cmd_cursor++;
    cmd_ix++;
    redraw_command_line();
}

/**
 * @brief Erase from the beginning of the line to the cursor (^U).
 *
 */
static void handle_kill_to_bol(void) {
    if (cmd_cursor == 0) return;
    leave_history_navigation_mode();
    memmove(&cmd_buffer[0], &cmd_buffer[cmd_cursor], cmd_ix - cmd_cursor + 1);
    cmd_ix -= cmd_cursor;
    cmd_cursor = 0;
    redraw_command_line();
}

/**
 * @brief Erase from the cursor to the end of the line (^K).
 *
 */
static void handle_kill_to_eol(void) {
    if (cmd_cursor >= cmd_ix) return;
    leave_history_navigation_mode();
    cmd_buffer[cmd_cursor] = '\0';
    cmd_ix = cmd_cursor;
    redraw_command_line();
}

/**
 * @brief Move the cursor to the start of the line (Home / ^A).
 *
 */
static void handle_move_home(void) {
    if (cmd_cursor == 0) return;
    cmd_cursor = 0;
    redraw_command_line();
}

/**
 * @brief Move the cursor to the end of the line (End / ^E).
 *
 */
static void handle_move_end(void) {
    if (cmd_cursor == cmd_ix) return;
    cmd_cursor = cmd_ix;
    redraw_command_line();
}

/**
 * @brief Insert a multi-character string at the cursor (e.g. tab-completion suffix), clamping to buffer capacity.
 *
 */
static void insert_text_at_cursor(const char *text) {
    if (!text || !text[0]) return;

    size_t add = strlen(text);
    if (cmd_ix + add >= sizeof(cmd_buffer)) {
        add = (sizeof(cmd_buffer) - 1) - cmd_ix;
    }    if (add == 0) return;

    leave_history_navigation_mode();
    memmove(&cmd_buffer[cmd_cursor + add], &cmd_buffer[cmd_cursor], cmd_ix - cmd_cursor + 1);
    memcpy(&cmd_buffer[cmd_cursor], text, add);
    cmd_ix += add;
    cmd_cursor += add;
    redraw_command_line();
}

/**
 * @brief Allocate the camera frame buffer if not already done.
 *
 * The buffer holds two frames (cam_ful_size * 2 bytes) for double-buffering.
 * Allocation is deferred so commands that don't use the camera don't consume the RAM.
 * @return true if the buffer is ready, false if malloc failed.
 */
static bool ensure_cam_buffer_allocated() {
    if (cam_ptr) return true;

    cam_ptr = (uint8_t *)malloc(cam_ful_size * 2);
    if (!cam_ptr) {
        printf("malloc failed for frame buffer (%lu bytes)\n",
               (unsigned long)(cam_ful_size * 2));
        return false;
    }
    memset(cam_ptr, 0, cam_ful_size * 2);
    return true;
}

/**
 * @brief Write the camera frame buffer to a file as raw RGB565 bytes.
 *
 * Creates or overwrites the file at path.  Writes exactly cam_ful_size * 2 bytes
 * (little-endian row-major RGB565).
 * @return true only if both f_write and f_close succeeded without error.
 */
static bool save_cam_buffer_to_file(const char *path) {
    FIL fil;
    FRESULT fr = f_open(&fil, path, FA_WRITE | FA_CREATE_ALWAYS);
    if (FR_OK != fr) {
        printf("f_open error: %s (%d)\n", FRESULT_str(fr), fr);
        return false;
    }

    UINT bw = 0;
    fr = f_write(&fil, cam_ptr, cam_ful_size * 2, &bw);
    if (FR_OK != fr) {
        printf("f_write error: %s (%d)\n", FRESULT_str(fr), fr);
    } else {
        printf("Wrote %u bytes to %s\n", bw, path);
    }

    FRESULT close_fr = f_close(&fil);
    if (FR_OK != close_fr) {
        printf("f_close error: %s (%d)\n", FRESULT_str(close_fr), close_fr);
    }

    return FR_OK == fr && FR_OK == close_fr;
}

/**
 * @brief Block until a camera frame is ready or the timeout expires, then free the camera on timeout.
 *
 * On timeout, frees the camera DMA resources and prints a message so the caller
 * doesn't need to handle cleanup.
 * @return true if a frame arrived within timeout_ms, false if it timed out.
 */
static bool wait_for_cam_frame_with_timeout(uint32_t timeout_ms) {
    if (cam_wait_for_frame(timeout_ms)) {
        return true;
    }

    free_cam();
    printf("Camera capture timed out after %lu ms\n", (unsigned long)timeout_ms);
    return false;
}

/**
 * @brief Clean up after a cam snap operation: close the file, free the camera, and reset all state.
 *
 * Called both on success (after the last chunk write) and on error paths.
 *
 */
static void finish_cam_snap(void) {
    if (cam_snap_file_open) {
        FRESULT close_fr = f_close(&cam_snap_file);
        if (FR_OK != close_fr) {
            printf("f_close error: %s (%d)\n", FRESULT_str(close_fr), close_fr);
        }
        cam_snap_file_open = false;
    }

    free_cam();
    buffer_ready = false;

    cam_snap_pending = false;
    cam_snap_state = CAM_SNAP_IDLE;
    cam_snap_path[0] = '\0';
    cam_snap_write_offset = 0;
    cam_snap_total_written = 0;
}

/**
 * @brief Read one OV5640 register (hex address) over I2C1 and print its value.
 */
static void run_cam_rreg(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;

    uint16_t reg = (uint16_t)strtol(argv[0], NULL, 16);
    uint8_t val = OV5640_RD_Reg(i2c1, 0x3C, reg);
    printf("reg 0x%04X = 0x%02X (%u)\n", reg, val, val);
}


/**
 * @brief Set the camera XCLK frequency via PWM and print the requested vs actual kHz.
 */
static void run_cam_xclk(const size_t argc, const char *argv[]){
    if (argc > 1) {
        printf("Usage: cam_xclk [freq_khz]  (default 24000)\n");
        return;
    }

    uint32_t requested_khz = 24000;
    if (argc == 1) {
        requested_khz = (uint32_t)atoi(argv[0]);
    }

    uint32_t actual_khz = set_pwm_freq_kHz(requested_khz, PIN_PWM);
    printf("XCLK requested: %lu kHz, actual: %lu kHz\n",
           (unsigned long)requested_khz,
           (unsigned long)actual_khz);
}

/**
 * @brief Initialise the i2c1 peripheral at 100 kHz and configure its SDA/SCL GPIO pins.
 */
static void init_i2c(){
    i2c_inst_t * i2c = i2c1;
    // Initialize I2C port at 100 kHz
    i2c_init(i2c, 100 * 1000);

    // Initialize I2C pins
    gpio_set_function(I2C1_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C1_SDA);
    gpio_pull_up(I2C1_SCL);
}

/**
 * @brief Initialise the I2C1 bus for camera SCCB communication (calls init_i2c()).
 */
static void run_cam_i2c_init(const size_t argc, const char *argv[]){
    if(!expect_argc(argc,argv,0)) return;
    
    init_i2c();
}

/**
 * @brief Read and print the OV5640 chip ID from registers 0x300A/0x300B over I2C1.
 */
static void run_cam_id(const size_t argc, const char *argv[]){
    if (!expect_argc(argc, argv, 0)) return;
    shared_enter_cam_prog();
    uint16_t reg;
    reg=OV5640_RD_Reg(i2c1,0x3C,0X300A);
	reg<<=8;
	reg|=OV5640_RD_Reg(i2c1,0x3C,0X300B);
    shared_leave_cam_prog();
    printf("ID: %d \r\n",reg);
}

/**
 * @brief Apply the original OV5640 default sensor init sequence via sccb_init() (resets size, PLL, color format).
 */
static void run_cam_defaults(const size_t argc, const char *argv[]){
    if (!expect_argc(argc, argv, 0)) return;
    // Match the original Waveshare 02-CAM flow:
    // sccb_init() does the I2C setup, applies the default table with embedded
    // delays, and then programs size, image options, PLL and colorspace.
    shared_enter_cam_prog();
    sccb_init(I2C1_SDA, I2C1_SCL);
    shared_leave_cam_prog();
    printf("Camera default sensor init applied\n");
}

/**
 * @brief Change the camera output resolution: stops the pipeline, reallocates the frame buffer, and writes the new size to the OV5640.
 */
static void run_cam_size(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 2)) return;

    uint32_t w = (uint32_t)atoi(argv[0]);
    uint32_t h = (uint32_t)atoi(argv[1]);

    // Take I2C1 ownership before writing camera registers (I2C1_SDA is shared with NRF CS).
    shared_enter_cam_prog();
    // Re-assert I2C pin mux each time in case a previous shared-pin state changed GPIO mode.
    init_i2c();

    // Stop any running capture pipeline so DMA doesn't write into the buffer we are about to free.
    free_cam();
    free(cam_ptr);
    cam_ptr = NULL;

    // Update the global resolution used by all cam.c / command.cpp callers.
    cam_width    = w;
    cam_height   = h;
    cam_ful_size = w * h;

    // Write the new output window to the OV5640 (X_OUTPUT_SIZE_H encodes both W and H in one call).
    uint8_t wr = OV5640_WR_Reg_2(i2c1, 0x3C, X_OUTPUT_SIZE_H, w, h);
    if (wr == 0xFF) {
        shared_leave_cam_prog();
        printf("cam size failed: SCCB write timed out (check CAM_SDA/CAM_SCL wiring or shared-pin state)\n");
        return;
    }

    // Allocate the new double-sized buffer (cam_ful_size pixels × 2 bytes/pixel × 2 for double-buffering).
    cam_ptr = (uint8_t *)malloc(cam_ful_size * 2);
    if (!cam_ptr) {
        shared_leave_cam_prog();
        printf("malloc failed for %lu x %lu\n", w, h);
        return;
    }

    shared_leave_cam_prog();
    printf("Size set to %lu x %lu\n", w, h);
    printf("Run cam dma then cam start to restart capture, or run cam snap directly\n");
}

/**
 * @brief Write the vertical flip register (TIMING_TC_REG20) of the OV5640 (0=normal, 6=flipped).
 */
static void run_cam_flip(const size_t argc, const char* argv[]){
    if(!expect_argc(argc,argv,1)) return;
    uint16_t flip = (uint16_t) atoi(argv[0]);
    shared_enter_cam_prog();
    OV5640_WR_Reg(i2c1, 0x3C, TIMING_TC_REG20, flip);
    shared_leave_cam_prog();
    printf("Flip set to %u\n", flip);
}

/**
 * @brief Write the horizontal mirror register (TIMING_TC_REG21) of the OV5640 (0=normal, 6=mirrored).
 */
static void run_cam_mirror(const size_t argc, const char *argv[]){
    if(!expect_argc(argc,argv,1)) return;
    uint16_t mirror = (uint16_t) atoi(argv[0]);
    shared_enter_cam_prog();
    OV5640_WR_Reg(i2c1, 0x3C, TIMING_TC_REG21, mirror);
    shared_leave_cam_prog();
    printf("Mirror set to %u\n", mirror);
}

/**
 * @brief Write the SC_PLL_CONTRL_2 multiplier register to adjust the OV5640 pixel clock speed.
 */
static void run_cam_pll(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;

    uint8_t multiplier = (uint8_t)atoi(argv[0]);
    shared_enter_cam_prog();
    OV5640_WR_Reg(i2c1, 0x3C, SC_PLL_CONTRL_2, multiplier);
    shared_leave_cam_prog();
    printf("PLL multiplier set to %u\n", multiplier);
}

/**
 * @brief Set the OV5640 output color format to either rgb565 or yuv422 by writing FORMAT_CTRL/FORMAT_CTRL00.
 */
static void run_cam_format(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;

    shared_enter_cam_prog();
    if (strcmp(argv[0], "rgb565") == 0) {
        OV5640_WR_Reg(i2c1, 0x3C, FORMAT_CTRL,  0x01);
        OV5640_WR_Reg(i2c1, 0x3C, FORMAT_CTRL00, 0x61);
        shared_leave_cam_prog();
        printf("Format set to RGB565\n");
    } else if (strcmp(argv[0], "yuv422") == 0) {
        OV5640_WR_Reg(i2c1, 0x3C, FORMAT_CTRL,  0x00);
        OV5640_WR_Reg(i2c1, 0x3C, FORMAT_CTRL00, 0x00);
        shared_leave_cam_prog();
        printf("Format set to YUV422\n");
    } else {
        shared_leave_cam_prog();
        printf("Unknown format: %s (use rgb565 or yuv422)\n", argv[0]);
    }
}

/**
 * @brief Allocate (or reallocate) the raw camera frame buffer to cam_ful_size * 2 bytes.
 */
static void run_cam_alloc(const size_t argc, const char *argv[]){
    if(!expect_argc(argc,argv,0)) return;
    if(cam_ptr) free(cam_ptr);
    cam_ptr = (uint8_t *)malloc(cam_ful_size * 2);
}

/**
 * @brief Enable IRQ-driven capture and configure the DMA channel and buffer for the camera pipeline.
 */
static void run_cam_dma(const size_t argc, const char *argv[]){
    if(!expect_argc(argc,argv,0)) return;
    cam_set_use_irq(true);
    config_cam_buffer();
}

/**
 * @brief Enable continuous capture mode, enable IRQ, and start the camera PIO/DMA pipeline.
 */
static void run_cam_start(const size_t argc, const char *argv[]){
    if(!expect_argc(argc,argv,0)) return;
    cam_set_continuous(true);
    cam_set_use_irq(true);
    start_cam();
}

/**
 * @brief Write one OV5640 register (hex address and value) over I2C1 and confirm the write.
 */
static void run_cam_wreg(const size_t argc, const char *argv[]){
    if(!expect_argc(argc,argv,2)) return;
    uint16_t reg = (uint16_t)strtol(argv[0], NULL, 16);
    uint8_t val = (uint8_t)strtol(argv[1], NULL, 16);
    shared_enter_cam_prog();
    OV5640_WR_Reg(i2c1,0x3C,reg,val);
    shared_leave_cam_prog();
    printf("reg 0x%04X = 0x%02X (%u)\n", reg, val, val);
}

/**
 * @brief Stop the camera DMA pipeline and release its resources (calls free_cam()).
 */
static void run_cam_stop(const size_t argc, const char *argv[]){
    if(!expect_argc(argc,argv,0)) return ;
    free_cam();
}

/**
 * @brief Print a summary of camera state: resolution, buffer pointers, stream flags, FPS, and mirror status.
 */
static void run_cam_status(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;
    printf("Resolution:      %lux%lu  (%lu pixels)\n",
           (unsigned long)cam_width, (unsigned long)cam_height, (unsigned long)cam_ful_size);
    printf("Frame buffer:    %s  (ptr=%p)\n",
           cam_ptr ? "allocated" : "not allocated", (void *)cam_ptr);
    printf("Double buffer:   %s  (ptr1=%p)\n",
           cam_ptr1 ? "active" : "inactive", (void *)cam_ptr1);
    printf("Display ptr:     %p\n", (void *)cam_display_ptr);
    printf("Buffer ready:    %s\n", buffer_ready ? "yes" : "no");
    printf("Stream active:   %s\n", lcd_cam_stream_active ? "yes" : "no");
    printf("Mirror:          %s\n", cam_mirror_enabled ? "on" : "off");
    printf("Capture FPS:     %lu\n", (unsigned long)cam_get_capture_fps());
    printf("Capture total:   %lu frames\n", (unsigned long)cam_get_capture_frames_total());
    printf("Display FPS:     %lu\n", (unsigned long)lcd_display_fps);
    printf("Display total:   %lu frames\n", (unsigned long)lcd_display_frames_total);
}

/**
 * @brief Capture one frame synchronously (blocking wait up to 2 s) and save it to the given file path.
 */
static void run_cam_capture(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;

    // wait for a fresh complete frame
    buffer_ready = false;
    printf("Waiting for frame...\n");
    if (!wait_for_cam_frame_with_timeout(2000)) return;
    buffer_ready = false;  // clear before we use the buffer

    save_cam_buffer_to_file(argv[0]);
}

/**
 * @brief Arm an asynchronous single-frame capture: configure the camera for one-shot mode, then hand off to
 *        the background state machine which opens the file, writes chunks, and closes it without blocking the shell.
 */
static void run_cam_snap(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;
    if (!ensure_cam_buffer_allocated()) return;
    if (lcd_load_pending || lcd_cam_snap_pending || lcd_cam_stream_active) {
        printf("Another camera or LCD operation is already in progress.\n");
        return;
    }
    if (cam_snap_pending) {
        printf("A camera snapshot is already in progress\n");
        return;
    }

    strlcpy(cam_snap_path, argv[0], sizeof(cam_snap_path));

    printf("Capturing frame...\n");
    fflush(stdout);
    stdio_flush();

    // Defensive reset: after heavy stream mode switching, DMA/PIO state may still be
    // mid-transition for one loop tick. Force a clean one-shot baseline here so cam_snap
    // cannot inherit a stale busy channel/SM state and appear to hang at "armed".
    cam_ptr1 = NULL;
    free_cam();

    // Verify VSYNC (GP8) is actually toggling before arming the PIO.
    // If the camera is not free-running, cam_wait_for_frame will hang forever.
    {
        const uint vsync_pin = CAM_BASE_PIN + 8;
        gpio_init(vsync_pin);
        gpio_set_dir(vsync_pin, GPIO_IN);
        int transitions = 0;
        bool prev = gpio_get(vsync_pin);
        uint64_t deadline_us = time_us_64() + 100000;  // 100 ms ≈ 3 frames at 30 fps
        while (time_us_64() < deadline_us) {
            bool cur = gpio_get(vsync_pin);
            if (cur != prev) { transitions++; prev = cur; }
        }
        // Restore pin to PIO0 so picampinos_program_init can use it.
        gpio_set_function(vsync_pin, GPIO_FUNC_PIO0);
        printf("VSYNC check (GP%u): %d transitions in 100 ms\n", vsync_pin, transitions);
        if (transitions == 0) {
            printf("VSYNC missing — re-initialising camera sensor...\n");
            shared_enter_cam_prog();
            sccb_init(I2C1_SDA, I2C1_SCL);
            shared_leave_cam_prog();
            // Re-check after reinit — give the sensor one frame to stabilise.
            sleep_ms(50);
            gpio_init(vsync_pin);
            gpio_set_dir(vsync_pin, GPIO_IN);
            transitions = 0;
            prev = gpio_get(vsync_pin);
            deadline_us = time_us_64() + 100000;
            while (time_us_64() < deadline_us) {
                bool cur = gpio_get(vsync_pin);
                if (cur != prev) { transitions++; prev = cur; }
            }
            gpio_set_function(vsync_pin, GPIO_FUNC_PIO0);
            if (transitions == 0) {
                printf("ERROR: camera still not responding after reinit\n");
                print_prompt();
                return;
            }
            printf("Camera recovered — VSYNC restored (%d transitions)\n", transitions);
        }
    }

    // One-shot mode: disable continuous capture and IRQ so the DMA stops after a single frame.
    cam_set_continuous(false);
    cam_set_use_irq(false);
    buffer_ready = false;
    config_cam_buffer();
    start_cam();  // Begin PIO + DMA capture; process_background_tasks() will observe buffer_ready

    // Arm the background state machine.
    cam_snap_deadline = make_timeout_time_ms(2000);
    cam_snap_next_step_time = get_absolute_time();
    cam_snap_pending = true;
    cam_snap_state = CAM_SNAP_WAIT_FRAME;
    cam_snap_write_offset = 0;
    cam_snap_total_written = 0;
}

/**
 * @brief Initialise the LCD panel: align clk_peri, start SPI at 62.5 MHz, run the panel init sequence,
 *        allocate the framebuffer, clear to white, attempt touch controller init, and set up NRF poll timer.
 */
static void run_lcd_init(const size_t argc, const char *argv[]) {
    if (argc > 1) {
        printf("Usage: lcd_init [vertical|horizontal]\n");
        return;
    }

    if (argc == 1) {
        if (strcmp(argv[0], "vertical") == 0) {
            lcd_scan_dir = VERTICAL;
        } else if (strcmp(argv[0], "horizontal") == 0) {
            lcd_scan_dir = HORIZONTAL;
        } else {
            printf("Invalid argument: %s (use vertical or horizontal)\n", argv[0]);
            return;
        }
    }

    // If NRF borrowed shared SPI pins earlier, deterministically restore LCD bus ownership first.
    shared_enter_lcd_active();

    // Configure RST, DC, and CS as GPIO outputs before touching the panel.
    DEV_GPIO_Mode(LCD_RST_PIN, 1);
    DEV_GPIO_Mode(LCD_DC_PIN, 1);
    DEV_GPIO_Mode(LCD_CS_PIN, 1);

    // Keep CS deasserted and DC low (command mode) during setup.
    DEV_Digital_Write(LCD_CS_PIN, 1);
    DEV_Digital_Write(LCD_DC_PIN, 0);

    // clk_peri must be sourced from clk_sys (not the USB PLL) so SPI can reach the
    // lcd_spi_hz target without being constrained by the 48 MHz USB PLL ceiling.
    // Sourcing from clk_sys also keeps SPI timing stable if clk_sys is later changed.
    uint32_t sys_hz = clock_get_hz(clk_sys);
    bool ok = clock_configure(
        clk_peri,
        0,
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
        sys_hz,
        sys_hz
    );
    if (!ok) {
        printf("Failed to align clk_peri with clk_sys\n");
        return;
    }

    // Initialise SPI at the requested LCD baud and assign SPI function to the clock/data pins.
    uint actual = spi_init(SPI_PORT, lcd_spi_hz);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    printf("LCD SPI actual: %u Hz\n", actual);
    gpio_set_function(LCD_CLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(LCD_MOSI_PIN, GPIO_FUNC_SPI);

    // Initialise backlight PWM (start at 0 / off) and run the panel initialisation sequence.
    DEV_PWM_Init();
    DEV_SET_PWM(0);
    LCD_2IN_Init(lcd_scan_dir);

    // Free any previous framebuffer and Mode-B relay buffer before reallocating.
    if (lcd_image) {
        free(lcd_image);
        lcd_image = NULL;
        lcd_image_bytes = 0;
    }
    free_mode_b_frame_raw();

    lcd_image_bytes = LCD_2IN.WIDTH * LCD_2IN.HEIGHT * 2;
    lcd_image = (UWORD *)malloc(lcd_image_bytes);
    if (!lcd_image) {
        printf("Failed to allocate LCD framebuffer\n");
        lcd_initialized = false;
        lcd_backlight = 0;
        return;
    }

    // Attach the GUI paint library to the freshly allocated buffer and clear to white.
    Paint_NewImage((UBYTE *)lcd_image, LCD_2IN.WIDTH, LCD_2IN.HEIGHT, 0, WHITE);
    Paint_SetScale(65);  // 65536-colour (16-bit RGB565) palette
    Paint_Clear(WHITE);
    LCD_2IN_Display(lcd_image);  // Push the white frame to the panel so it is immediately visible

    lcd_backlight = 0;
    lcd_initialized = true;
    // Attempt touch controller init (non-fatal: board may not have the CST816D fitted).
    if (touch_init_controller()) {
        printf("Touch initialized (CST816D)\n");
    } else {
        printf("Touch init failed (CST816D not detected)\n");
    }
    // Reset the NRF event poll timer so nrf_poll_button_event_if_needed() fires immediately.
    nrf_event_poll_next_time = get_absolute_time();
    printf("LCD initialized (%s)\n", lcd_scan_dir == VERTICAL ? "vertical" : "horizontal");
}

/**
 * @brief Set the LCD backlight PWM duty cycle (0–100%) and store it in lcd_backlight.
 */
static void run_lcd_bl(const size_t argc, const char *argv[]){
    if(!expect_argc(argc,argv,1)) return;
    if(!lcd_initialized){
        printf("LCD not initialized. Run lcd_init first.\n");
        return;
    }
    char *end = NULL;
    long value = strtol(argv[0], &end, 10);
    if (end == argv[0] || *end != '\0') {
        printf("Invalid brightness: %s (use 0-100)\n", argv[0]);
        return;
    }
    if (value < 0 || value > 100) {
        printf("Brightness out of range: %ld (use 0-100)\n", value);
        return;
    }

    DEV_SET_PWM((UWORD)value);
    lcd_backlight  = (uint8_t)value;

    printf("LCD backlight set to %u%%\n", (uint8_t)value);
}

/**
 * @brief Fill the entire LCD panel with a solid hex RGB565 color.
 */
static void run_lcd_clear(const size_t argc , const char *argv[]){
    if(!expect_argc(argc,argv,1)) return;
    if(!lcd_initialized){
        printf("LCD not initialized. Run lcd_init first.\n");
        return;
    }
    char *end = NULL;
    long value = strtoul(argv[0], &end, 16);
    if (end == argv[0] || *end != '\0') {
        printf("Invalid color: %s (use hex RGB565, e.g. ffff for white)\n", argv[0]);
        return;
    }

    if(value < 0 || value > 0xFFFF){
        printf("Color out of range: %ld (use hex RGB565, e.g. ffff for white)\n", value);
        return;
    }

    LCD_2IN_Clear((UWORD)value);
    printf("LCD cleared with color 0x%04X\n", (uint16_t)value);
}

/**
 * @brief Draw a single pixel at (x, y) with the given hex RGB565 color directly on the LCD panel.
 */
static void run_lcd_pixel(const size_t argc, const char *argv[]){
    if(!expect_argc(argc,argv,3)) return;
    if(!lcd_initialized){
        printf("LCD not initialized. Run lcd_init first.\n");
        return;
    }
    char *end = NULL;
    long x = strtol(argv[0], &end, 10);
    if (end == argv[0] || *end != '\0') {
        printf("Invalid x coordinate: %s (use decimal, e.g. 10)\n", argv[0]);
        return;
    }
    long y = strtol(argv[1], &end, 10);
    if (end == argv[1] || *end != '\0') {
        printf("Invalid y coordinate: %s (use decimal, e.g. 10)\n", argv[1]);
        return;
    }
    unsigned long color = strtoul(argv[2], &end, 16);
    if (end == argv[2] || *end != '\0') {
        printf("Invalid color: %s (use hex RGB565, e.g. ffff for white)\n", argv[2]);
        return;
    }
    int lcd_width = (lcd_scan_dir == VERTICAL) ? LCD_2IN_WIDTH : LCD_2IN_HEIGHT;
    int lcd_height = (lcd_scan_dir == VERTICAL) ? LCD_2IN_HEIGHT : LCD_2IN_WIDTH;
    if(x < 0 || x >= lcd_width || y < 0 || y >= lcd_height){
        printf("Coordinates out of range: (%lu, %lu) (LCD size is %dx%d)\n", x, y, lcd_width, lcd_height);
        return;
    }
    if(color > 0xFFFF){
        printf("Color out of range: %lu (use hex RGB565, e.g. ffff for white)\n", color);
        return;
    }

    LCD_2IN_DisplayPoint((UWORD)x, (UWORD)y, (UWORD)color);
    printf("Drew pixel at (%ld, %ld) with color 0x%04X\n", x, y, (uint16_t)color);
}

/**
 * @brief Fill a rectangle on the LCD panel with a solid color, byte-swapping to panel byte order and writing in 64-pixel chunks.
 */
static void run_lcd_fillrect(const size_t argc, const char *argv[]){
    if(!expect_argc(argc,argv,5)) return;
    if(!lcd_initialized){
        printf("LCD not initialized. Run lcd_init first.\n");
        return;
    }
    char *end = NULL;
    long x = strtol(argv[0], &end, 10);
    if (end == argv[0] || *end != '\0') {
        printf("Invalid x coordinate: %s (use decimal, e.g. 10)\n", argv[0]);
        return;
    }
    long y = strtol(argv[1], &end, 10);
    if (end == argv[1] || *end != '\0') {
        printf("Invalid y coordinate: %s (use decimal, e.g. 10)\n", argv[1]);
        return;
    }
    long w = strtol(argv[2], &end, 10);
    if (end == argv[2] || *end != '\0') {
        printf("Invalid width: %s (use decimal, e.g. 50)\n", argv[2]);
        return;
    }
    long h = strtol(argv[3], &end, 10);
    if (end == argv[3] || *end != '\0') {   
        printf("Invalid height: %s (use decimal, e.g. 50)\n", argv[3]);
        return;
    }
    unsigned long color = strtoul(argv[4], &end, 16);
    if (end == argv[4] || *end != '\0') {
        printf("Invalid color: %s (use hex RGB565, e.g. ffff for white)\n", argv[4]);
        return;
    }
    int lcd_width = (lcd_scan_dir == VERTICAL) ? LCD_2IN_WIDTH : LCD_2IN_HEIGHT;
    int lcd_height = (lcd_scan_dir == VERTICAL) ? LCD_2IN_HEIGHT : LCD_2IN_WIDTH;
    if(x < 0 || x >= lcd_width || x + w > lcd_width || y < 0 || y >= lcd_height || y + h > lcd_height){
        printf("Coordinates out of range: (%ld, %ld) (LCD size is %dx%d)\n", x, y, lcd_width, lcd_height);

        return;
    }
    if (w <= 0 || h <= 0) {
        printf("Width and height must be > 0\n");
        return;
    }

    if(color > 0xFFFF){
        printf("Color out of range: %lu (use hex RGB565, e.g. ffff for white)\n", color);
        return;
    }
    // The host supplies color as a little-endian uint16 (e.g. 0xF800 = red in RGB565).
    // The LCD panel expects big-endian (high byte first on the wire), so swap the bytes once here.
    UWORD color_be = (color >> 8) | (color << 8);
    // Pre-fill a 64-pixel tile with the byte-swapped color for efficient SPI writes.
    UWORD line_buf[64];

    for (int i = 0; i < 64; i++) {
        line_buf[i] = color_be;
    }

    // Set the LCD window to the target rectangle and enter RAM-write mode.
    LCD_2IN_SetWindows((uint16_t)x, (uint16_t)y, (uint16_t)(x + w), (uint16_t)(y + h));
    DEV_Digital_Write(LCD_DC_PIN, 1);
    DEV_Digital_Write(LCD_CS_PIN, 0);

    // Write the rectangle pixel-by-pixel in 64-pixel chunks to avoid large stack buffers.
    uint32_t pixels = (uint32_t)w * (uint32_t)h;
    while (pixels > 0) {
        uint32_t chunk = (pixels > 64) ? 64 : pixels;
        DEV_SPI_Write_nByte((uint8_t *)line_buf, chunk * 2);
        pixels -= chunk;
    }

    DEV_Digital_Write(LCD_CS_PIN, 1);


    printf("Filled rectangle at (%ld, %ld), size %ldx%ld, color 0x%04X\n",
       x, y, w, h, (uint16_t)color);

}

/* ── LCD console ─────────────────────────────────────────────────────────────
 * Adds an LCD output driver to pico stdio so all shell I/O (echo + command
 * output) is mirrored to the LCD panel as a text terminal.  USB/UART output
 * stays active, so you can still observe the terminal remotely.
 *
 * Uses the same MX-mirror rendering path as run_lcd_text.
 * Scrolling is "clear & wrap" — no framebuffer redraw needed.
 * ─────────────────────────────────────────────────────────────────────────── */

static void lcd_con_draw_char(char c, int x, int y) {
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    const sFONT *font = lcd_con_font;
    uint32_t bpr = (font->Width + 7u) / 8u;
    const uint8_t *cd = font->table + (size_t)((uint8_t)c - ' ') * bpr * font->Height;

    // VERTICAL: physical col 0 = right edge, so mirror the x address and reverse pixels.
    // HORIZONTAL (MV=1): physical col 0 = left edge, so use direct addressing.
    bool mirror = (lcd_scan_dir == VERTICAL);
    uint16_t x_addr = mirror ? (uint16_t)(lcd_con_width - x - (int)font->Width)
                             : (uint16_t)x;
    LCD_2IN_SetWindows(x_addr, (uint16_t)y,
                       (uint16_t)(x_addr + font->Width),
                       (uint16_t)(y + font->Height));
    DEV_Digital_Write(LCD_DC_PIN, 1);
    DEV_Digital_Write(LCD_CS_PIN, 0);

    uint16_t fg_be = (uint16_t)((lcd_con_fg >> 8) | (lcd_con_fg << 8));
    uint16_t bg_be = (uint16_t)((lcd_con_bg >> 8) | (lcd_con_bg << 8));
    uint16_t row_buf[20];

    for (uint16_t row = 0; row < font->Height; row++) {
        for (uint16_t col = 0; col < font->Width; col++) {
            uint8_t byte = cd[row * bpr + col / 8u];
            uint16_t dst = mirror ? (uint16_t)(font->Width - 1u - col) : col;
            row_buf[dst] = (byte & (0x80u >> (col % 8u))) ? fg_be : bg_be;
        }
        DEV_SPI_Write_nByte((uint8_t *)row_buf, font->Width * 2u);
    }
    DEV_Digital_Write(LCD_CS_PIN, 1);
}

static void lcd_con_scroll(void) {
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    LCD_2IN_Clear(lcd_con_bg);
    lcd_con_x = 0;
    lcd_con_y = 0;
}

static void lcd_con_putchar(char c) {
    if (!lcd_con_active || !lcd_initialized || !lcd_con_width) return;

    // Strip ANSI/VT escape sequences so line-editing sequences (erase-to-EOL,
    // cursor-back, etc.) don't appear as literal '[', 'K', digits on the LCD.
    if ((unsigned char)c == 0x1b) { lcd_con_ansi_state = 1; return; }
    if (lcd_con_ansi_state == 1) {
        lcd_con_ansi_state = (c == '[') ? 2 : 0;  // '[' → CSI sequence; anything else → 2-char seq, consumed
        return;
    }
    if (lcd_con_ansi_state == 2) {
        if (c == 'K') {
            // Erase to end of line: fill remaining cells on current row with background.
            lcd_con_ansi_state = 0;
            int cw = (int)lcd_con_font->Width;
            int save_x = lcd_con_x;
            while (lcd_con_x + cw <= lcd_con_width) {
                lcd_con_draw_char(' ', lcd_con_x, lcd_con_y);
                lcd_con_x += cw;
            }
            lcd_con_x = save_x;
            return;
        }
        if (isalpha((unsigned char)c) || c == '~') lcd_con_ansi_state = 0;
        return;
    }

    const sFONT *font = lcd_con_font;
    int ch = (int)font->Height;
    int cw = (int)font->Width;

    if (c == '\n') {
        lcd_con_x  = 0;
        lcd_con_y += ch;
        if (lcd_con_y + ch > lcd_con_height) lcd_con_scroll();
        return;
    }
    if (c == '\r') { lcd_con_x = 0; return; }
    if (c == '\b') {
        if (lcd_con_x >= cw) lcd_con_x -= cw;
        lcd_con_draw_char(' ', lcd_con_x, lcd_con_y);
        return;
    }
    if ((unsigned char)c < 32 || (unsigned char)c > 126) return;

    if (lcd_con_x + cw > lcd_con_width) {
        lcd_con_x  = 0;
        lcd_con_y += ch;
        if (lcd_con_y + ch > lcd_con_height) lcd_con_scroll();
    }
    lcd_con_draw_char(c, lcd_con_x, lcd_con_y);
    lcd_con_x += cw;
}

static void lcd_con_stop(void) {
    if (!lcd_con_active) return;
    lcd_con_active = false;
}

static void lcd_con_start(uint16_t fg, uint16_t bg, const sFONT *font) {
    if (!lcd_initialized) { printf("LCD not initialized.\n"); return; }
    lcd_con_stop();
    lcd_con_fg     = fg;
    lcd_con_bg     = bg;
    lcd_con_font   = font;
    lcd_con_width  = (lcd_scan_dir == VERTICAL) ? LCD_2IN_WIDTH  : LCD_2IN_HEIGHT;
    lcd_con_height = (lcd_scan_dir == VERTICAL) ? LCD_2IN_HEIGHT : LCD_2IN_WIDTH;
    lcd_con_x = 0;
    lcd_con_y = 0;
    shared_enter_lcd_active();
    LCD_2IN_Clear(bg);
    lcd_con_active = true;
}

static void run_lcdcon(const size_t argc, const char *argv[]) {
    if (argc > 0 && strcmp(argv[0], "-s") == 0) {
        if (!lcd_con_active) { printf("LCD console is not active.\n"); return; }
        lcd_con_stop();
        printf("LCD console stopped.\n");
        return;
    }

    const sFONT *font = &Font8;
    uint16_t fg = 0xFFFF, bg = 0x0000;

    if (argc >= 1) {
        int sz = atoi(argv[0]);
        switch (sz) {
            case  8: font = &Font8;  break;
            case 12: font = &Font12; break;
            case 16: font = &Font16; break;
            case 20: font = &Font20; break;
            case 24: font = &Font24; break;
            default: printf("Invalid size %d — use 8/12/16/20/24\n", sz); return;
        }
    }
    if (argc >= 2) fg = (uint16_t)strtoul(argv[1], NULL, 16);
    if (argc >= 3) bg = (uint16_t)strtoul(argv[2], NULL, 16);

    lcd_con_start(fg, bg, font);
}

/**
 * @brief Render a text string directly to the LCD panel via LCD_2IN_SetWindows + SPI.
 * Bypasses the Paint framebuffer so the coordinate system matches all other drawing commands.
 * Syntax: lcd text <x> <y> <size> <fg_hex> <bg_hex> <word> [word ...]
 * Size selects the font height: 8, 12, 16, 20, or 24.
 */
static void run_lcd_text(const size_t argc, const char *argv[]) {
    if (argc < 6) {
        printf("Usage: lcd text <x> <y> <size> <fg_hex> <bg_hex> <message...>\n");
        printf("  size: 8 | 12 | 16 | 20 | 24\n");
        printf("  e.g.: lcd text 0 0 16 ffff 0000 Hello world\n");
        return;
    }
    if (!lcd_initialized) {
        printf("LCD not initialized. Run lcd_init first.\n");
        return;
    }

    char *end;
    long x = strtol(argv[0], &end, 10);
    if (end == argv[0] || *end != '\0') { printf("Invalid x: %s\n", argv[0]); return; }
    long y = strtol(argv[1], &end, 10);
    if (end == argv[1] || *end != '\0') { printf("Invalid y: %s\n", argv[1]); return; }
    long size = strtol(argv[2], &end, 10);
    if (end == argv[2] || *end != '\0') { printf("Invalid size: %s\n", argv[2]); return; }
    unsigned long fg = strtoul(argv[3], &end, 16);
    if (end == argv[3] || *end != '\0') { printf("Invalid fg color: %s\n", argv[3]); return; }
    unsigned long bg = strtoul(argv[4], &end, 16);
    if (end == argv[4] || *end != '\0') { printf("Invalid bg color: %s\n", argv[4]); return; }

    sFONT *font;
    switch (size) {
        case  8: font = &Font8;  break;
        case 12: font = &Font12; break;
        case 16: font = &Font16; break;
        case 20: font = &Font20; break;
        case 24: font = &Font24; break;
        default:
            printf("Invalid size %ld — use 8, 12, 16, 20, or 24.\n", size);
            return;
    }

    /* Join remaining argv words into one string separated by spaces. */
    char msg[256];
    size_t pos = 0;
    for (size_t i = 5; i < argc && pos < sizeof(msg) - 1; i++) {
        if (i > 5 && pos < sizeof(msg) - 1) msg[pos++] = ' ';
        size_t len = strlen(argv[i]);
        if (pos + len >= sizeof(msg)) len = sizeof(msg) - 1 - pos;
        memcpy(msg + pos, argv[i], len);
        pos += len;
    }
    msg[pos] = '\0';

    int lcd_width  = (lcd_scan_dir == VERTICAL) ? LCD_2IN_WIDTH  : LCD_2IN_HEIGHT;
    int lcd_height = (lcd_scan_dir == VERTICAL) ? LCD_2IN_HEIGHT : LCD_2IN_WIDTH;

    /* ILI9488 expects big-endian RGB565 on the wire — byte-swap once here. */
    uint16_t fg_be = (uint16_t)((fg >> 8) | (fg << 8));
    uint16_t bg_be = (uint16_t)((bg >> 8) | (bg << 8));

    /* Pixel row buffer — max font width is 17 (Font24). */
    uint16_t row_buf[20];

    uint32_t bytes_per_row = (font->Width + 7u) / 8u;

    uint16_t cur_x = (uint16_t)x;
    uint16_t cur_y = (uint16_t)y;

    for (const char *p = msg; *p; p++) {
        uint8_t ch = (uint8_t)*p;
        if (ch < 32 || ch > 126) ch = '?';

        if ((int)(cur_x + font->Width) > lcd_width) {
            cur_x = (uint16_t)x;
            cur_y += (uint16_t)font->Height;
        }
        if ((int)(cur_y + font->Height) > lcd_height) break;

        const uint8_t *char_data = font->table + (size_t)(ch - ' ') * bytes_per_row * font->Height;

        // VERTICAL: physical col 0 = right edge → mirror address and reverse pixels.
        // HORIZONTAL (MV=1): physical col 0 = left edge → direct address, forward pixels.
        bool mirror = (lcd_scan_dir == VERTICAL);
        uint16_t x_addr = mirror ? (uint16_t)(lcd_width - (int)cur_x - (int)font->Width)
                                 : cur_x;
        LCD_2IN_SetWindows(x_addr, cur_y,
                           (uint16_t)(x_addr + font->Width),
                           (uint16_t)(cur_y + font->Height));
        DEV_Digital_Write(LCD_DC_PIN, 1);
        DEV_Digital_Write(LCD_CS_PIN, 0);

        for (uint16_t row = 0; row < font->Height; row++) {
            for (uint16_t col = 0; col < font->Width; col++) {
                uint8_t byte = char_data[row * bytes_per_row + col / 8u];
                uint16_t dst = mirror ? (uint16_t)(font->Width - 1u - col) : col;
                row_buf[dst] = (byte & (0x80u >> (col % 8u))) ? fg_be : bg_be;
            }
            DEV_SPI_Write_nByte((uint8_t *)row_buf, font->Width * 2u);
        }

        DEV_Digital_Write(LCD_CS_PIN, 1);
        cur_x += (uint16_t)font->Width;
    }

    printf("Text drawn at (%ld,%ld) size=%ld fg=0x%04lX bg=0x%04lX: \"%s\"\n",
           x, y, size, fg, bg, msg);
}

/**
 * @brief Change the LCD scan direction between vertical (portrait) and horizontal (landscape) without reinitialising.
 */
static void run_lcd_set_orientation(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;
    if (!lcd_initialized) {
        printf("LCD not initialized. Run lcd_init first.\n");
        return;
    }

    UBYTE new_scan_dir;
    if (strcmp(argv[0], "vertical") == 0) {
        new_scan_dir = VERTICAL;
    } else if (strcmp(argv[0], "horizontal") == 0) {
        new_scan_dir = HORIZONTAL;
    } else {
        printf("Invalid argument: %s (use vertical or horizontal)\n", argv[0]);
        return;
    }

    if (new_scan_dir == lcd_scan_dir) {
        printf("LCD orientation already set to %s\n",
               lcd_scan_dir == VERTICAL ? "vertical" : "horizontal");
        return;
    }

    LCD_2IN_SetAttributes(new_scan_dir);
    lcd_scan_dir = new_scan_dir;

    if (lcd_con_active) {
        lcd_con_width  = (lcd_scan_dir == VERTICAL) ? LCD_2IN_WIDTH  : LCD_2IN_HEIGHT;
        lcd_con_height = (lcd_scan_dir == VERTICAL) ? LCD_2IN_HEIGHT : LCD_2IN_WIDTH;
        spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
        LCD_2IN_Clear(lcd_con_bg);
        lcd_con_x = 0;
        lcd_con_y = 0;
    }

    printf("LCD orientation set to %s\n",
           lcd_scan_dir == VERTICAL ? "vertical" : "horizontal");
}

/**
 * @brief Clean up after an LCD image load: close the file if open and reset all load-state variables.
 *
 */
static void finish_lcd_load(void) {
    if (lcd_load_file_open) {
        f_close(&lcd_load_file);
        lcd_load_file_open = false;
    }
    lcd_load_pending = false;
    lcd_load_state = LCD_LOAD_IDLE;
    lcd_load_offset = 0;
    lcd_load_display_row = 0;
    lcd_load_path[0] = '\0';
    lcd_load_status_msg[0] = '\0';
}

/**
 * @brief Abort an in-progress LCD image load with an error message, then print the prompt.
 *
 */
static void fail_lcd_load(const char *msg) {
    printf("\n%s\n", msg);
    finish_lcd_load();
    print_prompt();
}

/**
 * @brief Begin a non-blocking push of the existing lcd_image framebuffer to the screen.
 *
 * Used after in-memory operations (e.g. Paint_Clear) that modify lcd_image in place;
 * skips file I/O and goes directly to the DISPLAY phase of the state machine.
 * @return true if the push was enqueued, false if another LCD operation is already running.
 */
static bool start_lcd_framebuffer_display(const char *status_msg) {
    if (lcd_load_pending) {
        printf("An LCD image operation is already in progress.\n");
        return false;
    }

    lcd_load_pending = true;
    lcd_load_state = LCD_LOAD_DISPLAY;
    lcd_load_file_open = false;
    lcd_load_offset = 0;
    lcd_load_display_row = 0;
    lcd_load_next_step_time = get_absolute_time();
    lcd_load_path[0] = '\0';
    if (status_msg) {
        strlcpy(lcd_load_status_msg, status_msg, sizeof(lcd_load_status_msg));
    } else {
        lcd_load_status_msg[0] = '\0';
    }
    return true;
}

/**
 * @brief Update the display FPS counter whenever a frame is pushed to the LCD.
 *
 * Uses a 1-second rolling window: counts frames within the window, then
 * calculates fps = frames / elapsed_us * 1e6 and resets the window.
 *
 */
static void lcd_note_displayed_frame(void) {
    uint64_t now_us = time_us_64();
    if (lcd_display_window_start_us == 0) {
        lcd_display_window_start_us = now_us;
    }

    lcd_display_frames_total++;
    lcd_display_frames_window++;

    uint64_t elapsed_us = now_us - lcd_display_window_start_us;
    if (elapsed_us >= 1000000ULL) {
        lcd_display_fps = (uint32_t)((lcd_display_frames_window * 1000000ULL) / elapsed_us);
        lcd_display_frames_window = 0;
        lcd_display_window_start_us = now_us;
    }
}

/**
 * @brief Push a range of rows from the lcd_image framebuffer to the LCD panel via SPI.
 *
 * @param start_row First row to transfer (0 = top of screen).
 * @param row_count Number of rows to transfer.
 */
static void lcd_display_rows_from_framebuffer(uint32_t start_row, uint32_t row_count) {    LCD_2IN_SetWindows(0, (UWORD)start_row, LCD_2IN.WIDTH, (UWORD)(start_row + row_count));
    DEV_Digital_Write(LCD_DC_PIN, 1);
    DEV_Digital_Write(LCD_CS_PIN, 0);
    for (uint32_t row = start_row; row < start_row + row_count; ++row) {
        DEV_SPI_Write_nByte((UBYTE *)lcd_image + (row * LCD_2IN.WIDTH * 2), LCD_2IN.WIDTH * 2);
    }
    DEV_Digital_Write(LCD_CS_PIN, 1);
}

/**
 * @brief Byte-swap each RGB565 pixel from camera byte order to LCD panel byte order.
 *
 * Camera DMA produces [LOW_BYTE, HIGH_BYTE] per pixel (little-endian).
 * The LCD controller expects [HIGH_BYTE, LOW_BYTE] per pixel (big-endian).
 * This must be applied before writing raw camera data to lcd_image.
 *
 * @param src         Source pixel data in camera/little-endian order (2 bytes/pixel).
 * @param dst         Destination buffer in panel/big-endian order (2 bytes/pixel).
 * @param pixel_count Number of pixels to convert.
 */
static void lcd_convert_raw_rgb565_to_panel_bytes(const uint8_t *src, uint8_t *dst, uint32_t pixel_count) {    for (uint32_t i = 0; i < pixel_count; ++i) {
        dst[i * 2] = src[i * 2 + 1];
        dst[i * 2 + 1] = src[i * 2];
    }
}

/**
 * @brief Push a complete camera frame to the LCD using DMA for maximum throughput.
 *
 * Uses 16-bit SPI DMA mode to avoid per-byte CPU overhead. The camera DMA stores pixels
 * in little-endian byte order; 16-bit SPI mode transmits HIGH byte first automatically,
 * producing correct RGB565 on the wire without a CPU-side byte swap.
 * Only acts when start_row == 0; subsequent chunk calls for the same frame are no-ops
 * because the full frame was already sent on the first call.
 *
 * @param start_row  Must be 0 to trigger the transfer; non-zero calls are silently skipped.
 * @param row_count  Ignored (full frame always sent).
 * @param raw_rows   Pointer to the camera pixel buffer (camera byte order, full frame).
 */
static void lcd_display_raw_rows(uint32_t start_row, uint32_t row_count, const uint8_t *raw_rows) {    (void)row_count;
    if (start_row > 0) {
        // The full frame was sent on start_row==0. Remaining chunk calls are no-ops.
        return;
    }

    // Claim DMA channels once, on first call.
    if (spi_tx_dma_ch < 0) {
        spi_tx_dma_ch = (int)dma_claim_unused_channel(true);
        spi_rx_dma_ch = (int)dma_claim_unused_channel(true);
    }

    // Set window for the whole frame and enter RAM-write mode.
    LCD_2IN_SetWindows(0, 0, LCD_2IN.WIDTH, LCD_2IN.HEIGHT);
    DEV_Digital_Write(LCD_DC_PIN, 1);
    DEV_Digital_Write(LCD_CS_PIN, 0);

    // Switch SPI to 16-bit mode.
    // The camera DMA stores each pixel as [LOW_BYTE, HIGH_BYTE] in memory (little-endian).
    // In 16-bit SPI mode the hardware reads the 16-bit word (0xHIGH_LOW) and sends
    // MSB-first â†’ HIGH byte then LOW byte on the wire: correct RGB565, no CPU swap needed.
    spi_set_format(SPI_PORT, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    // TX DMA: raw_rows â†’ SPI TX FIFO (full frame, 16-bit words, DMA-paced)
    dma_channel_config tx_cfg = dma_channel_get_default_config((uint)spi_tx_dma_ch);
    channel_config_set_transfer_data_size(&tx_cfg, DMA_SIZE_16);
    channel_config_set_dreq(&tx_cfg, spi_get_dreq(SPI_PORT, true));
    channel_config_set_read_increment(&tx_cfg, true);
    channel_config_set_write_increment(&tx_cfg, false);
    dma_channel_configure((uint)spi_tx_dma_ch, &tx_cfg,
                          &spi_get_hw(SPI_PORT)->dr, raw_rows, cam_ful_size, false);

    // RX DMA: drain SPI RX FIFO â†’ spi_rx_dummy.
    // Without this the 4-deep RX FIFO overflows, which stalls the TX FIFO and kills throughput.
    dma_channel_config rx_cfg = dma_channel_get_default_config((uint)spi_rx_dma_ch);
    channel_config_set_transfer_data_size(&rx_cfg, DMA_SIZE_16);
    channel_config_set_dreq(&rx_cfg, spi_get_dreq(SPI_PORT, false));
    channel_config_set_read_increment(&rx_cfg, false);
    channel_config_set_write_increment(&rx_cfg, false);
    dma_channel_configure((uint)spi_rx_dma_ch, &rx_cfg,
                          (void *)&spi_rx_dummy, &spi_get_hw(SPI_PORT)->dr, cam_ful_size, false);

    // Fire TX and RX simultaneously, then wait for TX to drain.
    dma_start_channel_mask((1u << (uint)spi_tx_dma_ch) | (1u << (uint)spi_rx_dma_ch));
    dma_channel_wait_for_finish_blocking((uint)spi_tx_dma_ch);

    // Wait for the SPI shift register to finish clocking out the last word,
    // then restore 8-bit mode (for subsequent LCD commands) and release CS.
    while (spi_get_hw(SPI_PORT)->sr & SPI_SSPSR_BSY_BITS) tight_loop_contents();
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    DEV_Digital_Write(LCD_CS_PIN, 1);
    lcd_note_displayed_frame();
}

/**
 * @brief Copy raw camera rows into the lcd_image framebuffer with byte-order conversion.
 *
 * Applies lcd_convert_raw_rgb565_to_panel_bytes per row so that subsequent
 * lcd_display_rows_from_framebuffer calls send correct panel byte order.
 *
 * @param start_row First destination row in lcd_image.
 * @param row_count Number of rows to copy.
 * @param raw_rows Source buffer of raw camera pixel data (little-endian RGB565).
 */
static void lcd_copy_raw_rows_to_framebuffer(uint32_t start_row, uint32_t row_count, const uint8_t *raw_rows) {
    for (uint32_t row = 0; row < row_count; ++row) {
        uint8_t *dst = (uint8_t *)lcd_image + ((start_row + row) * LCD_2IN_WIDTH * 2);
        const uint8_t *src = raw_rows + (row * LCD_2IN_WIDTH * 2);
        lcd_convert_raw_rgb565_to_panel_bytes(src, dst, LCD_2IN_WIDTH);
    }
}

/**
 * @brief Verify that both the LCD and camera are ready for a camera-to-LCD operation.
 *
 * Checks: LCD initialized, lcd_image framebuffer allocated, camera frame buffer allocated,
 * LCD in vertical orientation (required by the current DMA path), and camera resolution
 * matches LCD dimensions (240×320). Prints a descriptive error and returns false on the
 * first failing check.
 *
 * @param cmd_name Name of the calling command, used in error messages.
 * @return true if all preconditions are satisfied, false otherwise.
 */
static bool ensure_lcd_camera_ready(const char *cmd_name) {
    if (!lcd_initialized) {
        printf("LCD not initialized. Run lcd_init first.\n");
        return false;
    }

    if (!lcd_image) {
        printf("LCD framebuffer not allocated. Run lcd_init first.\n");
        return false;
    }

    if (!ensure_cam_buffer_allocated()) {
        return false;
    }

    if (lcd_scan_dir != VERTICAL) {
        printf("%s currently supports vertical LCD orientation only.\n", cmd_name);
        return false;
    }

    if (cam_width != LCD_2IN_WIDTH || cam_height != LCD_2IN_HEIGHT) {
        printf("%s requires camera size 240x320.\n", cmd_name);
        return false;
    }

    return true;
}

/**
 * @brief Reset all lcd_cam_snap state after a snapshot completes or is aborted.
 *
 * Releases the camera frame buffer, clears the pending flag, and returns the
 * state machine to idle so the next lcd_cam_snap command can start cleanly.
 */
static void finish_lcd_cam_snap(void) {
    free_cam();
    buffer_ready = false;
    lcd_cam_snap_pending = false;
    lcd_cam_snap_state = LCD_CAM_SNAP_IDLE;
    lcd_cam_snap_row = 0;
}

/**
 * @brief Tear down an active lcd_cam_stream session and release all shared resources.
 *
 * Deasserts both LCD and NRF chip-selects (in case streaming was interrupted mid-frame),
 * restores shared_pin_state to LCD ownership, releases the camera frame buffer and any
 * Mode-B raw frame buffer, then resets the stream state machine to idle.
 */
static void finish_lcd_cam_stream(void) {    DEV_Digital_Write(LCD_CS_PIN, 1);  // Safety: deassert CS if stopped mid-frame
    if (nrf_spi_initialized) {
        gpio_put(nrf_spi_cs_pin, 1);
    }
    shared_pin_state = SHARED_PIN_LCD_ACTIVE;
    cam_ptr1 = NULL;                   // Release the framebuffer back to LCD use
    free_cam();
    buffer_ready = false;
    lcd_cam_stream_active = false;
    lcd_cam_stream_state = LCD_CAM_STREAM_IDLE;
    lcd_cam_stream_row = 0;
    mode_b_stream_locked_frame = NULL;
    free_mode_b_frame_raw();
}

/**
 * @brief Arm an asynchronous one-shot camera capture and display: configures camera for single-shot mode,
 *        then hands off to the background state machine which waits for a frame and pushes rows to the LCD.
 */
static void run_lcd_cam_snap(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;
    lcd_con_stop();
    if (!ensure_lcd_camera_ready("lcd_cam_snap")) return;
    if (cam_snap_pending || lcd_load_pending || lcd_cam_snap_pending || lcd_cam_stream_active) {
        printf("Another camera or LCD operation is already in progress.\n");
        return;
    }

    cam_set_continuous(false);
    cam_set_use_irq(false);
    buffer_ready = false;
    config_cam_buffer();
    start_cam();

    lcd_cam_snap_pending = true;
    lcd_cam_snap_state = LCD_CAM_SNAP_WAIT_FRAME;
    lcd_cam_snap_row = 0;
    lcd_cam_snap_deadline = make_timeout_time_ms(2000);
    lcd_cam_snap_next_step_time = get_absolute_time();

    printf("Capturing and displaying camera frame in background...\n");
}

/**
 * @brief Begin a non-blocking load of a raw 240×320 RGB565 file from SD into the LCD framebuffer and panel.
 */
static void run_lcd_load_image(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;
    lcd_con_stop();

    if (!lcd_initialized) {
        printf("LCD not initialized. Run lcd_init first.\n");
        return;
    }

    if (!lcd_image) {
        printf("LCD framebuffer not allocated. Run lcd_init first.\n");
        return;
    }

    if (lcd_scan_dir != VERTICAL) {
        printf("lcd_load_image currently supports vertical LCD orientation only.\n");
        return;
    }

    if (lcd_load_pending || lcd_cam_snap_pending || lcd_cam_stream_active) {
        printf("Another LCD image or stream operation is already in progress.\n");
        return;
    }

    strlcpy(lcd_load_path, argv[0], sizeof(lcd_load_path));
    lcd_load_offset = 0;
    lcd_load_display_row = 0;
    lcd_load_pending = true;
    lcd_load_file_open = false;
    lcd_load_state = LCD_LOAD_OPEN_FILE;
    lcd_load_next_step_time = get_absolute_time();

    printf("Loading image in background...\n");
}

/**
 * @brief Clear the framebuffer to black and enqueue a non-blocking push to the LCD panel.
 */
static void run_lcd_unload_image(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;

    if (!lcd_initialized || !lcd_image) {
        printf("LCD not initialized. Run lcd_init first.\n");
        return;
    }

    if (lcd_cam_stream_active) {
        printf("Stop lcd_cam_stream before unloading the image.\n");
        return;
    }

    Paint_Clear(BLACK);
    if (!start_lcd_framebuffer_display("LCD image unloaded and screen cleared to black")) {
        return;
    }

    printf("Clearing LCD in background...\n");
}

/**
 * @brief Start or stop the continuous camera-to-LCD background stream; with "stop" tears down the stream and clears the screen.
 */
static void run_lcd_cam_stream(const size_t argc, const char *argv[]) {
    if (argc > 1) {
        printf("Usage: lcd_cam_stream [stop]\n");
        return;
    }

    if (argc == 1) {
        if (strcmp(argv[0], "stop") != 0) {
            printf("Unexpected argument: %s\n", argv[0]);
            return;
        }

        if (!lcd_cam_stream_active) {
            printf("LCD camera stream is not running.\n");
            return;
        }

        finish_lcd_cam_stream();
        Paint_Clear(BLACK);
        if (!start_lcd_framebuffer_display("LCD camera stream stopped and screen cleared to black")) {
            return;
        }

        printf("Stopping LCD camera stream...\n");
        return;
    }
    lcd_con_stop();
    if (!ensure_lcd_camera_ready("lcd_cam_stream")) return;
    if (cam_snap_pending || lcd_load_pending || lcd_cam_snap_pending || lcd_cam_stream_active) {
        printf("Another camera or LCD operation is already in progress.\n");
        return;
    }

    // Point cam_ptr1 at lcd_image so camera DMA can double-buffer into the LCD framebuffer directly.
    // The stream display path reads cam_display_ptr (which alternates between cam_ptr and cam_ptr1)
    // so the LCD always shows the most recently completed frame.
    cam_ptr1 = (uint8_t *)lcd_image;
    cam_set_continuous(true);  // Keep capturing frames without stopping after the first one
    cam_set_use_irq(true);     // Use IRQ/DMA path (non-blocking)
    buffer_ready = false;
    reset_mode_b_transfer_ctx();     // Ensure no stale Mode-B state from a previous session
    mode_b_stream_locked_frame = NULL;
    config_cam_buffer();
    start_cam();

    lcd_cam_stream_active = true;
    lcd_cam_stream_state = LCD_CAM_STREAM_WAIT_FRAME;
    lcd_cam_stream_row = 0;
    lcd_cam_stream_deadline = make_timeout_time_ms(2000);
    lcd_cam_stream_next_step_time = get_absolute_time();

    printf("Starting LCD camera stream in background (%s)...\n", pipeline_mode_name(pipeline_mode));
}

/**
 * @brief Print LCD init state, orientation, backlight level, and logical pixel dimensions.
 */
static void run_lcd_status(const size_t argc, const char *argv[]){
    if(!expect_argc(argc, argv, 0)) return;
    printf("LCD initialized: %s. ", lcd_initialized ? "yes" : "no");
    printf("Orientation: %s. ", lcd_scan_dir == VERTICAL ? "vertical" : "horizontal");
    printf("Backlight brightness: %u%%.\n", lcd_backlight);
    printf("Size: %dx%d.\n", (lcd_scan_dir == VERTICAL) ? LCD_2IN_WIDTH : LCD_2IN_HEIGHT,
           (lcd_scan_dir == VERTICAL) ? LCD_2IN_HEIGHT : LCD_2IN_WIDTH);
}

/**
 * @brief Print the rolling capture FPS, display FPS, and cumulative frame counts.
 */
static void run_lcd_fps(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;

    printf("Capture FPS: %lu\n", (unsigned long)cam_get_capture_fps());
    printf("Display FPS: %lu\n", (unsigned long)lcd_display_fps);
    printf("Captured frames total: %lu\n", (unsigned long)cam_get_capture_frames_total());
    printf("Displayed frames total: %lu\n", (unsigned long)lcd_display_frames_total);
    printf("LCD stream active: %s\n", lcd_cam_stream_active ? "yes" : "no");
}

/**
 * @brief Print touch controller init state, event counters, and current mirror setting.
 */
static void run_touch_status(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;
    printf("Touch initialized: %s\n", touch_initialized ? "yes" : "no");
    printf("Touch events handled: %lu\n", (unsigned long)touch_event_count);
    printf("NRF events handled: %lu\n", (unsigned long)nrf_event_count);
    printf("Mirror state: %s\n", cam_mirror_enabled ? "ON" : "OFF");
}

/**
 * @brief Resolve a user-supplied string to a pipeline_mode_t value.
 *
 * Accepts "a", "A", or "touch" for PIPE_MODE_A_TOUCH (direct CAM→LCD path),
 * and "b", "B", or "nrf" for PIPE_MODE_B_NRF (CAM→NRF echo→LCD validation path).
 *
 * @param arg String token from the command line.
 * @param mode_out Receives the resolved mode on success; unchanged on failure.
 * @return true if arg matched a known mode, false if unrecognized.
 */
static bool parse_pipeline_mode_arg(const char *arg, pipeline_mode_t *mode_out) {
    if (!arg || !mode_out) return false;
    if (strcmp(arg, "a") == 0 || strcmp(arg, "A") == 0 || strcmp(arg, "touch") == 0) {
        *mode_out = PIPE_MODE_A_TOUCH;
        return true;
    }
    if (strcmp(arg, "b") == 0 || strcmp(arg, "B") == 0 || strcmp(arg, "nrf") == 0) {
        *mode_out = PIPE_MODE_B_NRF;
        return true;
    }
    return false;
}

/**
 * @brief Print active pipeline mode, validation flag, shared-pin owner, and any pending mode switch.
 */
static void run_sm_status(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;

    printf("Pipeline mode: %s\n", pipeline_mode_name(pipeline_mode));
    printf("Validation mode-B path enabled: %s\n", pipeline_validation_mode_b_enabled ? "yes" : "no");
    printf("Shared pin state: %s\n", shared_pin_state_name(shared_pin_state));
    printf("Pending mode switch: %s\n", pipeline_mode_switch_pending ? "yes" : "no");
    if (pipeline_mode_switch_pending) {
        printf("Pending target: %s (source=%s)\n",
               pipeline_mode_name(pipeline_mode_pending),
               pipeline_request_source_name(pipeline_mode_pending_source));
    }
}

/**
 * @brief Request a pipeline mode switch (a/touch = Mode A, b/nrf = Mode B); deferred to frame boundary if streaming.
 */
static void run_sm_mode(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;

    pipeline_mode_t next_mode;
    if (!parse_pipeline_mode_arg(argv[0], &next_mode)) {
        printf("Invalid mode: %s (use a|b|touch|nrf)\n", argv[0]);
        return;
    }

    pipeline_request_mode(next_mode, PIPE_REQ_SHELL);
    if (!pipeline_mode_switch_pending) {
        printf("Pipeline mode set to %s\n", pipeline_mode_name(pipeline_mode));
    } else {
        printf("Pipeline mode switch to %s queued\n", pipeline_mode_name(next_mode));
    }
}

/**
 * @brief Inject a synthetic touch or NRF event to trigger a pipeline mode request as if raised by hardware.
 */
static void run_sm_event(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;

    if (strcmp(argv[0], "touch") == 0) {
        pipeline_request_mode(PIPE_MODE_A_TOUCH, PIPE_REQ_TOUCH);
    } else if (strcmp(argv[0], "nrf") == 0) {
        pipeline_request_mode(PIPE_MODE_B_NRF, PIPE_REQ_NRF);
    } else {
        printf("Invalid event: %s (use touch|nrf)\n", argv[0]);
        return;
    }

    if (!pipeline_mode_switch_pending) {
        printf("Event applied. Active pipeline mode: %s\n", pipeline_mode_name(pipeline_mode));
    } else {
        printf("Event accepted. Pending pipeline mode: %s\n", pipeline_mode_name(pipeline_mode_pending));
    }
}

/**
 * @brief Enable or disable the full NRF echo-validation path for Mode-B streaming (on/off).
 */
static void run_sm_validation(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;

    if (strcmp(argv[0], "on") == 0) {
        pipeline_validation_mode_b_enabled = true;
    } else if (strcmp(argv[0], "off") == 0) {
        pipeline_validation_mode_b_enabled = false;
    } else {
        printf("Invalid argument: %s (use on|off)\n", argv[0]);
        return;
    }

    printf("Validation mode-B full-frame path is now %s\n",
           pipeline_validation_mode_b_enabled ? "enabled" : "disabled");
}

/**
 * @brief Configure the shared SPI bus for NRF communication at the requested baud rate.
 *
 * Reclaims LCD_CLK/MOSI/MISO/RST pins as SPI, configures I2C1_SDA as the NRF CS
 * GPIO output (deasserted), and initialises SPI at Mode 0. The LCD must not be
 * accessed while NRF owns these pins.
 */
static bool nrf_spi_prepare_bus(uint32_t requested_hz) {
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
    shared_pin_state = SHARED_PIN_NRF_TRANSACTION;
    return true;
}

/**
 * @brief Discard any leftover bytes in the SPI RX FIFO before starting a new transfer.
 *
 * Stale bytes from a previous transaction would corrupt the RX data of the next transfer
 * if not cleared first.
 */
static void nrf_spi_drain_rx_fifo(void) {    while (spi_is_readable(SPI_PORT)) {
        (void)spi_get_hw(SPI_PORT)->dr;
    }
}

/**
 * @brief Configure the shared SPI bus for NRF SPIS00 at the given baud rate (default 32 MHz → ~30 MHz actual on RP2350 @ 150 MHz) and print the pin mapping.
 */
static void run_nrf_spi_init(const size_t argc, const char *argv[]) {
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
static void run_nrf_spi_status(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;
    printf("NRF SPI initialized: %s\n", nrf_spi_initialized ? "yes" : "no");
    printf("Pins: SCK=%u MOSI=%u MISO=%u CS=%u\n",
           nrf_spi_sck_pin, nrf_spi_mosi_pin, nrf_spi_miso_pin, nrf_spi_cs_pin);
    printf("Actual baud: %lu Hz\n", (unsigned long)nrf_spi_actual_hz);
}

/**
 * @brief Perform one manual full-duplex SPI transfer with hex byte arguments and print the TX and RX bytes.
 */
static void run_nrf_spi_xfer(const size_t argc, const char *argv[]) {
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

    nrf_spi_drain_rx_fifo();
    gpio_put(nrf_spi_cs_pin, 0);
    busy_wait_us_32(nrf_spi_cs_setup_us);  // Give slave CS setup time before first SCK edge.
    int ret = spi_write_read_blocking(SPI_PORT, tx, rx, (size_t)n);
    busy_wait_us_32(nrf_spi_cs_hold_us);  // Hold CS a moment after last edge.
    gpio_put(nrf_spi_cs_pin, 1);
    busy_wait_us_32(nrf_spi_frame_gap_us);  // Small gap between frames.

    if (ret < 0 || (size_t)ret != n) {
        printf("SPI transfer failed (%d)\n", ret);
        printf("FAILED\n");
        return;
    }

    printf("TX: ");
    print_hex_bytes(tx, n);
    printf("\nRX: ");
    print_hex_bytes(rx, n);
    printf("\nPASSED\n");
}

/**
 * @brief Sweep SPI baud rates from start_hz to stop_hz, measuring pass rate and throughput at each step against an expected echo pattern.
 */
static void run_nrf_spi_sweep(const size_t argc, const char *argv[]) {
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
    uint32_t restore_hz = nrf_spi_actual_hz;
    if (restore_hz == 0) {
        restore_hz = start_hz;
    }

    // Iterate over each requested baud point.
    for (uint32_t hz = start_hz; hz <= stop_hz; hz += step_hz) {
        nrf_spi_prepare_bus(hz);
        // One warm-up transfer discarded before measurement: ensures the SPI FIFO and slave
        // state are stable after the baud-rate change so the first timed frame is representative.
        nrf_spi_drain_rx_fifo();
        gpio_put(nrf_spi_cs_pin, 0);
        busy_wait_us_32(nrf_spi_cs_setup_us);
        (void)spi_write_read_blocking(SPI_PORT, tx, rx, tx_len);
        busy_wait_us_32(nrf_spi_cs_hold_us);
        gpio_put(nrf_spi_cs_pin, 1);
        busy_wait_us_32(nrf_spi_frame_gap_us);

        uint32_t ok = 0;   // Transfers whose RX exactly matched expect[]
        uint32_t fail = 0; // Transfers that returned a wrong length or wrong data
        uint64_t t0 = time_us_64();
        // Timed measurement loop: send 'loops' frames and count pass/fail.
        for (uint32_t i = 0; i < loops; ++i) {
            nrf_spi_drain_rx_fifo();
            gpio_put(nrf_spi_cs_pin, 0);
            busy_wait_us_32(nrf_spi_cs_setup_us);
            int ret = spi_write_read_blocking(SPI_PORT, tx, rx, tx_len);
            busy_wait_us_32(nrf_spi_cs_hold_us);
            gpio_put(nrf_spi_cs_pin, 1);
            busy_wait_us_32(nrf_spi_frame_gap_us);

            if (ret >= 0 && (size_t)ret == tx_len && memcmp(rx, expect, tx_len) == 0) {
                ok++;
            } else {
                fail++;
            }
        }
        uint64_t dt = time_us_64() - t0;
        // Compute throughput: total bytes / elapsed seconds, cast to uint32.
        // The cast is safe because tx_len * loops can't overflow uint64 for practical sweep sizes.
        uint32_t throughput_bps = 0;
        if (dt > 0) {
            throughput_bps = (uint32_t)(((uint64_t)tx_len * loops * 1000000ULL) / dt);
        }

        printf("req=%lu actual=%lu ok=%lu fail=%lu throughput=%lu B/s\n",
               (unsigned long)hz, (unsigned long)nrf_spi_actual_hz,
               (unsigned long)ok, (unsigned long)fail, (unsigned long)throughput_bps);

        // Guard against uint32 overflow: if incrementing hz would wrap past stop_hz, stop now.
        if (hz > stop_hz - step_hz) break;
    }

    nrf_spi_prepare_bus(restore_hz);
    printf("Sweep done. Restored SPI baud to actual=%lu Hz\n", (unsigned long)nrf_spi_actual_hz);
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
static void run_nrf_spi_bench(const size_t argc, const char *argv[]) {
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
}
#endif /* NRF_BENCH_ENABLED */

#ifdef NRF_BENCH_ENABLED
/**
 * @brief Run SPI diagnostics: Phase 1 tests all four CPOL/CPHA modes with CS high (loopback), Phase 2 pulses CS low once and prints the slave response.
 */
static void run_nrf_spi_diag(const size_t argc, const char *argv[]) {
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

/**
 * @brief Show or tune Mode-B SPI timing: displays current CS setup/hold/gap/chunk values and an FPS estimate, or sets one parameter.
 */
static void run_nrf_timing(const size_t argc, const char *argv[]) {
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

/**
 * @brief Print cam group help.
 *
 */
static void print_cam_group_help(void) {
    printf("cam <subcommand> [args]\n");
    printf("Subcommands: xclk, i2c, id, defaults, size, flip, mirror, pll, format,\n");
    printf("             alloc, dma, start, stop, status, rreg, wreg, snap\n");
}

/**
 * @brief Print lcd group help.
 *
 */
static void print_lcd_group_help(void) {
    printf("lcd <subcommand> [args]\n");
    printf("Subcommands: init, bl, clear, pixel, fillrect, text, orient, snap,\n");
    printf("             load, unload, stream, status, fps\n");
    printf("Examples: lcd stream, lcd stream -s, lcd load 0:/cam/test0.rgb565\n");
}

/**
 * @brief Print sm group help.
 *
 */
static void print_sm_group_help(void) {
    printf("sm <subcommand> [args]\n");
    printf("Subcommands: status, mode, event, validation\n");
}

/**
 * @brief Print nrf group help.
 *
 */
static void print_nrf_group_help(void) {
    printf("nrf <subcommand> [args]\n");
#ifdef NRF_BENCH_ENABLED
    printf("Subcommands: init, status, xfer, sweep, bench, diag, timing\n");
#else
    printf("Subcommands: init, status, xfer, timing\n");
#endif
}

static const char *const cam_group_subcmds[] = {
    "xclk", "i2c", "id", "defaults", "size", "flip", "mirror", "pll",
    "format", "alloc", "dma", "start", "stop", "status", "rreg", "wreg",
    "snap", "capture"
};
static const char *const lcd_group_subcmds[] = {
    "init", "bl", "clear", "pixel", "fillrect", "text", "orient", "orientation",
    "snap", "load", "unload", "stream", "status", "fps"
};
static const char *const sm_group_subcmds[] = {
    "status", "mode", "event", "validation",
};
static const char *const nrf_group_subcmds[] = {
    "init", "status", "xfer", "sweep",
#ifdef NRF_BENCH_ENABLED
    "bench", "diag",
#endif
    "timing",
};

static bool get_group_subcommands(const char *group,
                                  const char *const **subcmds_out,
                                  size_t *count_out) {
    if (!group || !subcmds_out || !count_out) return false;
    if (strcmp(group, "cam") == 0) {
        *subcmds_out = cam_group_subcmds;
        *count_out = count_of(cam_group_subcmds);
        return true;
    }
    if (strcmp(group, "lcd") == 0) {
        *subcmds_out = lcd_group_subcmds;
        *count_out = count_of(lcd_group_subcmds);
        return true;
    }
    if (strcmp(group, "sm") == 0) {
        *subcmds_out = sm_group_subcmds;
        *count_out = count_of(sm_group_subcmds);
        return true;
    }
    if (strcmp(group, "nrf") == 0) {
        *subcmds_out = nrf_group_subcmds;
        *count_out = count_of(nrf_group_subcmds);
        return true;
    }
    return false;
}

/**
 * @brief Dispatch the "cam <subcommand>" group: routes argv[0] to the corresponding run_cam_* handler.
 */
static void run_cam_group(const size_t argc, const char *argv[]) {
    if (argc == 0 || strcmp(argv[0], "help") == 0) {
        print_cam_group_help();
        return;
    }

    const char *sub = argv[0];
    size_t sub_argc = argc - 1;
    const char **sub_argv = argv + 1;
    if (strcmp(sub, "xclk") == 0) run_cam_xclk(sub_argc, sub_argv);    else if (strcmp(sub, "i2c") == 0) run_cam_i2c_init(sub_argc, sub_argv);    else if (strcmp(sub, "id") == 0) run_cam_id(sub_argc, sub_argv);    else if (strcmp(sub, "defaults") == 0) run_cam_defaults(sub_argc, sub_argv);    else if (strcmp(sub, "size") == 0) run_cam_size(sub_argc, sub_argv);    else if (strcmp(sub, "flip") == 0) run_cam_flip(sub_argc, sub_argv);    else if (strcmp(sub, "mirror") == 0) run_cam_mirror(sub_argc, sub_argv);    else if (strcmp(sub, "pll") == 0) run_cam_pll(sub_argc, sub_argv);    else if (strcmp(sub, "format") == 0) run_cam_format(sub_argc, sub_argv);    else if (strcmp(sub, "alloc") == 0) run_cam_alloc(sub_argc, sub_argv);    else if (strcmp(sub, "dma") == 0) run_cam_dma(sub_argc, sub_argv);    else if (strcmp(sub, "start") == 0) run_cam_start(sub_argc, sub_argv);    else if (strcmp(sub, "stop") == 0) run_cam_stop(sub_argc, sub_argv);    else if (strcmp(sub, "status") == 0) run_cam_status(sub_argc, sub_argv);    else if (strcmp(sub, "rreg") == 0) run_cam_rreg(sub_argc, sub_argv);    else if (strcmp(sub, "wreg") == 0) run_cam_wreg(sub_argc, sub_argv);    else if (strcmp(sub, "snap") == 0) run_cam_snap(sub_argc, sub_argv);    else if (strcmp(sub, "capture") == 0) run_cam_snap(sub_argc, sub_argv); // alias
    else {
        printf("Unknown cam subcommand: %s\n", sub);
        print_cam_group_help();
    }
}

/**
 * @brief Dispatch the "lcd <subcommand>" group: routes argv[0] to the corresponding run_lcd_* handler.
 */
static void run_lcd_group(const size_t argc, const char *argv[]) {
    if (argc == 0 || strcmp(argv[0], "help") == 0) {
        print_lcd_group_help();
        return;
    }

    const char *sub = argv[0];
    size_t sub_argc = argc - 1;
    const char **sub_argv = argv + 1;
    if (strcmp(sub, "init") == 0) run_lcd_init(sub_argc, sub_argv);    else if (strcmp(sub, "bl") == 0) run_lcd_bl(sub_argc, sub_argv);    else if (strcmp(sub, "clear") == 0) run_lcd_clear(sub_argc, sub_argv);    else if (strcmp(sub, "pixel") == 0) run_lcd_pixel(sub_argc, sub_argv);    else if (strcmp(sub, "fillrect") == 0) run_lcd_fillrect(sub_argc, sub_argv);    else if (strcmp(sub, "text") == 0) run_lcd_text(sub_argc, sub_argv);
    else if (strcmp(sub, "orient") == 0 || strcmp(sub, "orientation") == 0) run_lcd_set_orientation(sub_argc, sub_argv);    else if (strcmp(sub, "snap") == 0) run_lcd_cam_snap(sub_argc, sub_argv);    else if (strcmp(sub, "load") == 0) run_lcd_load_image(sub_argc, sub_argv);    else if (strcmp(sub, "unload") == 0) run_lcd_unload_image(sub_argc, sub_argv);
    else if (strcmp(sub, "stream") == 0) {
        if (sub_argc >= 1 && (strcmp(sub_argv[0], "-s") == 0 || strcmp(sub_argv[0], "--stop") == 0)) {
            const char *stop_argv[] = {"stop"};
            run_lcd_cam_stream(1, stop_argv);
        } else {
            run_lcd_cam_stream(sub_argc, sub_argv);
        }
    } else if (strcmp(sub, "status") == 0) run_lcd_status(sub_argc, sub_argv);    else if (strcmp(sub, "fps") == 0) run_lcd_fps(sub_argc, sub_argv);
    else {
        printf("Unknown lcd subcommand: %s\n", sub);
        print_lcd_group_help();
    }
}

/**
 * @brief Dispatch the "sm <subcommand>" group: routes argv[0] to the corresponding run_sm_* handler.
 */
static void run_sm_group(const size_t argc, const char *argv[]) {
    if (argc == 0 || strcmp(argv[0], "help") == 0) {
        print_sm_group_help();
        return;
    }

    const char *sub = argv[0];
    size_t sub_argc = argc - 1;
    const char **sub_argv = argv + 1;
    if (strcmp(sub, "status") == 0) run_sm_status(sub_argc, sub_argv);
    else if (strcmp(sub, "mode") == 0) run_sm_mode(sub_argc, sub_argv);
    else if (strcmp(sub, "event") == 0) run_sm_event(sub_argc, sub_argv);
    else if (strcmp(sub, "validation") == 0) run_sm_validation(sub_argc, sub_argv);
    else {
        printf("Unknown sm subcommand: %s\n", sub);
        print_sm_group_help();
    }
}

/**
 * @brief Dispatch the "nrf <subcommand>" group: routes argv[0] to the corresponding run_nrf_* handler.
 */
static void run_nrf_group(const size_t argc, const char *argv[]) {
    if (argc == 0 || strcmp(argv[0], "help") == 0) {
        print_nrf_group_help();
        return;
    }

    const char *sub = argv[0];
    size_t sub_argc = argc - 1;
    const char **sub_argv = argv + 1;
    if (strcmp(sub, "init") == 0) run_nrf_spi_init(sub_argc, sub_argv);
    else if (strcmp(sub, "status") == 0) run_nrf_spi_status(sub_argc, sub_argv);
    else if (strcmp(sub, "xfer") == 0) run_nrf_spi_xfer(sub_argc, sub_argv);
    else if (strcmp(sub, "sweep") == 0) run_nrf_spi_sweep(sub_argc, sub_argv);
#ifdef NRF_BENCH_ENABLED
    else if (strcmp(sub, "bench") == 0) run_nrf_spi_bench(sub_argc, sub_argv);
    else if (strcmp(sub, "diag") == 0) run_nrf_spi_diag(sub_argc, sub_argv);
#endif
    else if (strcmp(sub, "timing") == 0) run_nrf_timing(sub_argc, sub_argv);
    else {
        printf("Unknown nrf subcommand: %s\n", sub);
        print_nrf_group_help();
    }
}

static void run_alias_dotdot(const size_t argc, const char *argv[]) {
    (void)argc; (void)argv;
    const char *args[] = {".."};
    run_cd(1, args);
}

static void run_alias_dotdotdot(const size_t argc, const char *argv[]) {
    (void)argc; (void)argv;
    const char *args[] = {"../.."};
    run_cd(1, args);
}

typedef void (*p_fn_t)(const size_t argc, const char *argv[]);
typedef struct {
    char const *const command;
    p_fn_t const function;
    char const *const help;
} cmd_def_t;

static cmd_def_t cmds[] = {
    {"setrtc", run_setrtc,
     "setrtc <DD> <MM> <YY> <hh> <mm> <ss>:\n"
     " Set Real Time Clock\n"
     " Parameters: new date (DD MM YY) new time in 24-hour format "
     "(hh mm ss)\n"
     "\te.g.:setrtc 16 3 21 0 4 0"},
    {"date", run_date, "date:\n Print current date and time"},
    {"format", run_format,
     "format [<drive#:>]:\n"
     " Creates an FAT/exFAT volume on the logical drive.\n"
     "\te.g.: format 0:"},
    {"mount", run_mount,
     "mount [<drive#:>]:\n"
     " Register the work area of the volume\n"
     "\te.g.: mount 0:"},
    {"unmount", run_unmount,
     "unmount <drive#:>:\n"
     " Unregister the work area of the volume"},
    {"chdrive", run_chdrive,
     "chdrive <drive#:>:\n"
     " Changes the current directory of the logical drive.\n"
     " <path> Specifies the directory to be set as current directory.\n"
     "\te.g.: chdrive 1:"},
    {"info", run_info, 
    "info [<drive#:>]:\n"
      " Print information about an SD card"},
    {"cd", run_cd,
     "cd <path>:\n"
     " Changes the current directory of the logical drive.\n"
     " <path> Specifies the directory to be set as current directory.\n"
     "\te.g.: cd /dir1"},
    {"mkdir", run_mkdir,
     "mkdir <path>:\n"
     " Make a new directory.\n"
     " <path> Specifies the name of the directory to be created.\n"
     "\te.g.: mkdir /dir1"},
    // {"del_node", run_del_node,
    //  "del_node <path>:\n"
    //  "  Remove directory and all of its contents.\n"
    //  "  <path> Specifies the name of the directory to be deleted.\n"
    //  "\te.g.: del_node /dir1"},
    {"rm", run_rm,
     "rm [options] <pathname>:\n"
     " Removes (deletes) a file or directory\n"
     " <pathname> Specifies the path to the file or directory to be removed\n"
     " Options:\n"
     " -d Remove an empty directory\n"
     " -r Recursively remove a directory and its contents"},
    {"cp", run_cp,
     "cp <source file> <dest file>:\n"
     " Copies <source file> to <dest file>"},
    {"mv", run_mv,
     "mv <source file> <dest file>:\n"
     " Moves (renames) <source file> to <dest file>"},
    {"pwd", run_pwd,
     "pwd:\n"
     " Print Working Directory"},
    {"ls", run_ls, "ls [pathname]:\n List directory"},
    {"ll",  run_ls,              "ll: Alias for ls"},
    {"..",  run_alias_dotdot,    "..: Change to parent directory"},
    {"...", run_alias_dotdotdot, "...: Change up two directories"},
    // {"dir", run_ls, "dir:\n List directory"},
    {"cat", run_cat, "cat <filename>:\n Type file contents"},
    {"simple", run_simple, "simple:\n Run simple FS tests"},
    {"lliot", run_lliot,
     "lliot <physical drive#>:\n !DESTRUCTIVE! Low Level I/O Driver Test\n"
     "The SD card will need to be reformatted after this test.\n"
     "\te.g.: lliot 1"},
    {"bench", run_bench, "bench <drive#:>:\n A simple binary write/read benchmark"},
    {"big_file_test", run_big_file_test,
     "big_file_test <pathname> <size in MiB> <seed>:\n"
     " Writes random data to file <pathname>.\n"
     " Specify <size in MiB> in units of mebibytes (2^20, or 1024*1024 bytes)\n"
     "\te.g.: big_file_test 0:/bf 1 1\n"
     "\tor: big_file_test 1:big3G-3 3072 3"},
    {"bft", run_big_file_test,"bft: Alias for big_file_test"},
    {"cdef", run_cdef,
     "cdef:\n Create Disk and Example Files\n"
     " Expects card to be already formatted and mounted"},
    {"swcwdt", run_swcwdt,
     "swcwdt:\n Stdio With CWD Test\n"
     "Expects card to be already formatted and mounted.\n"
     "Note: run cdef first!"},
    {"loop_swcwdt", run_loop_swcwdt,
     "loop_swcwdt:\n Run Create Disk and Example Files and Stdio With CWD "
     "Test in a loop.\n"
     "Expects card to be already formatted and mounted.\n"
     "Note: Hit Enter key to quit."},
    {"start_logger", run_start_logger,
     "start_logger:\n"
     " Start Data Log Demo"},
    {"stop_logger", run_stop_logger,
     "stop_logger:\n"
     " Stop Data Log Demo"},
    {"mem-stats", run_mem_stats,
     "mem-stats:\n"
     " Print memory statistics"},
    {"cam", run_cam_group,
     "cam <subcommand> [args]:\n"
     " Camera command group.\n"
     " Use \"cam help\" for subcommands."},
    {"lcd", run_lcd_group,
     "lcd <subcommand> [args]:\n"
     " LCD command group.\n"
     " Use \"lcd help\" for subcommands."},
    {"sm", run_sm_group,
     "sm <subcommand> [args]:\n"
     " State-machine command group.\n"
     " Use \"sm help\" for subcommands."},
    {"nrf", run_nrf_group,
     "nrf <subcommand> [args]:\n"
     " NRF-SPI command group.\n"
     " Use \"nrf help\" for subcommands."},
    {"sm_status", run_sm_status,
     "sm_status:\n"
     " Show state-machine status (mode, pending switch, shared pin owner)."},
    {"sm_mode", run_sm_mode,
     "sm_mode <a|b|touch|nrf>:\n"
     " Request pipeline mode switch.\n"
     " a/touch -> CAM->RP->LCD, b/nrf -> CAM->NRF->RP->LCD."},
    {"sm_event", run_sm_event,
     "sm_event <touch|nrf>:\n"
     " Inject mode-switch event as if raised by touch panel or NRF button."},
    {"sm_validation", run_sm_validation,
     "sm_validation <on|off>:\n"
     " Enable/disable validation-only full-frame Path-B transfer via NRF."},
    {"touch_status", run_touch_status,
     "touch_status:\n"
     " Print touch init state and mirror event counters."},
    {"nrf_spi_init", run_nrf_spi_init,
     "nrf_spi_init [baud_hz]:\n"
     " Configure shared SPI pins for NRF SPIS00 (default 15 MHz, confirmed max reliable).\n"
     " Requires nRF54L15 wired on P2.06/P2.08/P2.09/P2.10 (HSSPI dedicated pins)."},
    {"nrf_spi_status", run_nrf_spi_status,
     "nrf_spi_status:\n"
     " Show NRF SPI pin mapping and current baud rate."},
    {"nrf_spi_xfer", run_nrf_spi_xfer,
     "nrf_spi_xfer <byte_hex...>:\n"
     " Perform one SPI transaction and print TX/RX bytes.\n"
     "\te.g.: nrf_spi_xfer 9F 00 00 00"},
    {"nrf_spi_sweep", run_nrf_spi_sweep,
     "nrf_spi_sweep <start_hz> <stop_hz> <step_hz> <loops> <tx_hex> <expect_hex>:\n"
     " Sweep SPI rates and report pass/fail against expected response.\n"
     "\te.g.: nrf_spi_sweep 1000000 32000000 1000000 200 A55A0102 5AA5AABB"},
#ifdef NRF_BENCH_ENABLED
    {"nrf_spi_bench", run_nrf_spi_bench,
     "nrf_spi_bench <frame_bytes> <loops>:\n"
     " Throughput benchmark with auto-generated pattern (no hex typing).\n"
     " Reports ok/fail counts and throughput in B/s and KB/s.\n"
     "\te.g.: nrf_spi_bench 256 500"},
    {"nrf_spi_diag", run_nrf_spi_diag,
     "nrf_spi_diag [loops]:\n"
     " Run SPI diagnostics.\n"
     " Phase1 keeps CS high for pure MOSI->MISO loopback checks.\n"
     " Phase2 pulses CS low once and prints slave RX sample."},
#endif
    {"nrf_timing", run_nrf_timing,
     "nrf_timing [gap|setup|hold|chunk <val>]:\n"
     " Show or tune Mode-B transfer timing.\n"
     " No args: print current values + FPS estimate.\n"
     " nrf_timing gap   <us>   - inter-frame gap  (default 200, HSSPI: try 50)\n"
     " nrf_timing setup <us>   - CS setup before SCK (default 15,  HSSPI: try 2)\n"
     " nrf_timing hold  <us>   - CS hold after SCK   (default 4,   HSSPI: try 2)\n"
     " nrf_timing chunk <rows> - rows per transfer   (default 4,   HSSPI: try 8)"},
    {"cam_xclk", run_cam_xclk,
     "cam_xclk [<freq_khz>]:\n"
     " Set camera XCLK frequency. Default 24000\n"
     "\te.g.: cam_xclk 24000"},
    {"cam_i2c", run_cam_i2c_init,
     "cam_i2c:\n"
     " Initialise I2C1 bus for camera SCCB"},
    {"cam_id", run_cam_id,
     "cam_id:\n"
     " Read and print OV5640 chip ID"},
    {"cam_defaults", run_cam_defaults,
     "cam_defaults:\n"
     " Apply the original OV5640 default sensor init sequence"},
    {"cam_size", run_cam_size,
     "cam_size <w> <h>:\n"
     " Set camera output resolution\n"
     "\te.g.: cam_size 240 320"},
    {"cam_flip", run_cam_flip,
     "cam_flip <val>:\n"
     " Set vertical flip (0=normal, 6=flipped)\n"
     "\te.g.: cam_flip 0"},
    {"cam_mirror", run_cam_mirror,
     "cam_mirror <val>:\n"
     " Set horizontal mirror (0=normal, 6=mirrored)\n"
     "\te.g.: cam_mirror 0"},
    {"cam_pll", run_cam_pll,
     "cam_pll <multiplier>:\n"
     " Set PLL multiplier. Default 11 (~25 fps at 240x320).\n"
     "\te.g.: cam_pll 11"},
    {"cam_format", run_cam_format,
     "cam_format <rgb565|yuv422>:\n"
     " Set camera output color format\n"
     "\te.g.: cam_format rgb565"},
    {"cam_alloc", run_cam_alloc,
     "cam_alloc:\n"
     " Allocate frame buffer in RAM"},
    {"cam_dma", run_cam_dma,
     "cam_dma:\n"
     " Configure DMA channel and IRQ handler"},
    {"cam_start", run_cam_start,
     "cam_start:\n"
     " Load PIO program and start capture"},
    {"cam_rreg", run_cam_rreg,
     "cam_rreg <reg>:\n"
     " Read one OV5640 register (hex address)\n"
     "\te.g.: cam_rreg 501f"},
    {"cam_wreg", run_cam_wreg,
     "cam_wreg <reg> <val>:\n"
     " Write one OV5640 register (hex address and value)\n"
     "\te.g.: cam_wreg 501f 01"},
    {"cam_capture", run_cam_capture,
     "cam_capture <filename>:\n"
     " Legacy alias of cam_snap.\n"
     " Prefer: cam snap <filename>"},
    {"cam_snap", run_cam_snap,
     "cam_snap <filename>:\n"
     " Capture one frame, restore UART, and save to SD\n"
     "\te.g.: cam_snap 0:/photo.bin"},
    {"cam_stop", run_cam_stop,
     "cam_stop:\n"
     " Stop camera DMA and pipeline"},
    {"cam_status", run_cam_status,
     "cam_status:\n"
     " Print camera state: resolution, buffers, FPS, stream and mirror flags"},
    {"lcd_init", run_lcd_init,
     "lcd_init [vertical|horizontal]:\n"
     " Initialise LCD and set scan direction (default vertical)"},
    {"lcd_bl", run_lcd_bl,
     "lcd_bl <0-100>:\n"
     " Set LCD backlight brightness in percent\n"
     "\te.g.: lcd_bl 50"},
    {"lcd_clear", run_lcd_clear,
     "lcd_clear <color>:\n"
     " Clear LCD with specified color (hex RGB565, e.g. ffff for white)"},
    {"lcd_pixel", run_lcd_pixel,
     "lcd_pixel <x> <y> <color>:\n"
     " Set a pixel on the LCD to the specified color (hex RGB565, e.g. ffff for white)\n"
     "\te.g.: lcd_pixel 100 100 ffff"},
    {"lcd_fillrect", run_lcd_fillrect,
     "lcd_fillrect <x> <y> <w> <h> <color>:\n"
     " Fill a rectangle on the LCD with the specified color (hex RGB565, e.g. ffff for white)\n"
     "\te.g.: lcd_fillrect 50 50 100 100 ffff"},
    {"lcd_text", run_lcd_text,
     "lcd_text <x> <y> <size> <fg_hex> <bg_hex> <message...>:\n"
     " Draw a text string on the LCD. size: 8|12|16|20|24\n"
     "\te.g.: lcd_text 0 0 16 ffff 0000 Hello world"},
    {"lcd_set_orientation", run_lcd_set_orientation,
     "lcd_set_orientation <vertical|horizontal>:\n"
     " Set the scan direction of the LCD (default vertical)"},
    {"lcd_cam_snap", run_lcd_cam_snap,
     "lcd_cam_snap:\n"
     " Capture one camera frame and display it on the LCD in background."},
    {"lcd_load_image", run_lcd_load_image,
     "lcd_load_image <file>:\n"
     " Load a raw 240x320 RGB565 image from SD and display it in background."},
    {"lcd_unload_image", run_lcd_unload_image,
     "lcd_unload_image:\n"
     " Clear the LCD image buffer and display black in background."},
    {"lcd_cam_stream", run_lcd_cam_stream,
     "lcd_cam_stream [stop]:\n"
     " Start or stop continuous camera-to-LCD preview in background."},
    {"lcdcon", run_lcdcon,
     "lcdcon [<size>] [<fg_hex>] [<bg_hex>]:\n"
     " Mirror all shell I/O (typed input + command output) to the LCD as a text terminal.\n"
     " size: font height in px — 8 (default, 40x60), 12, 16, 20, 24\n"
     " fg/bg: RGB565 hex colours (default: ffff / 0000  i.e. white on black)\n"
     " lcdcon -s : stop and return to UART-only output\n"
     " Auto-stops when lcd_load_image, lcd_cam_snap or lcd_cam_stream is started.\n"
     "\te.g.: lcdcon\n"
     "\te.g.: lcdcon 12 07e0 0000   (green on black, Font12)\n"
     "\te.g.: lcdcon -s"},
    {"lcd_status", run_lcd_status,
     "lcd_status:\n"
     " Display the current status of the LCD."},
    {"lcd_fps", run_lcd_fps,
     "lcd_fps:\n"
     " Display rolling camera capture FPS and LCD display FPS."},


    // // Clocks testing:
    // {"set_sys_clock_48mhz", run_set_sys_clock_48mhz,
    //  "set_sys_clock_48mhz:\n"
    //  " Set the system clock to 48MHz"},
    // {"set_sys_clock_khz", run_set_sys_clock_khz,
    //  "set_sys_clock_khz <khz>:\n"
    //  " Set the system clock system clock frequency in khz."},
    // {"measure_freqs", run_measure_freqs,
    //  "measure_freqs:\n"
    //  " Count the RP2040 clock frequencies and report."},
    // {"clr", clr, "clr <gpio #>: clear a GPIO"},
    // {"set", set, "set <gpio #>: set a GPIO"},
    // {"test", run_test, "test:\n"
    //  " Development test"},
    {"help", run_help,
     "help:\n"
     " Shows this command help."}
};
/**
 * @brief Print the help string for every registered command, one per line.
 */
static void run_help(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;

    for (size_t i = 0; i < count_of(cmds); ++i) {
        printf("%s\n\n", cmds[i].help);
    }
    fflush(stdout);
    stdio_flush();
}

/**
 * @brief Complete the token under the cursor using the registered command table or group subcommand lists.
 *
 * Case 1 (no space before cursor): matches against top-level command names.
 * Case 2 (space before cursor, first token is a group): matches against that group's subcommand list.
 * In both cases: single match appends the suffix and a trailing space; multiple matches prints the list;
 * exact match with multiple prefix-siblings inserts a space instead of listing ambiguous completions.
 */
static void handle_tab_completion(void) {
    if (cmd_cursor == 0) return;

    // Find the position of the first space before the cursor to determine which token we are completing.
    size_t first_space = cmd_cursor;
    for (size_t i = 0; i < cmd_cursor; ++i) {
        if (isspace((unsigned char)cmd_buffer[i])) {
            first_space = i;
            break;
        }
    }

    // Case 1: no space found before cursor — we are completing the command name (first token).
    if (first_space == cmd_cursor) {
        char prefix[sizeof(cmd_buffer)];
        memcpy(prefix, cmd_buffer, cmd_cursor);
        prefix[cmd_cursor] = '\0';
        size_t prefix_len = cmd_cursor;

        // Scan every registered command for entries whose name starts with the typed prefix.
        size_t match_idx[count_of(cmds)];
        size_t match_count = 0;
        bool has_exact = false;
        for (size_t i = 0; i < count_of(cmds); ++i) {
            if (strncmp(cmds[i].command, prefix, prefix_len) == 0) {
                match_idx[match_count++] = i;
                if (strcmp(cmds[i].command, prefix) == 0) {
                    has_exact = true;  // The typed text already names a command exactly.
                }
            }
        }

        if (match_count == 0) {
            printf("\a");  // Bell: no match found.
            stdio_flush();
            tab_was_last_key = false;
            return;
        }

        if (match_count == 1) {
            // Unique match: fill in the remainder of the command name.
            const char *only = cmds[match_idx[0]].command;
            if (cmd_cursor == cmd_ix && strlen(only) > prefix_len) {
                insert_text_at_cursor(only + prefix_len);
            }
            // Append a space so the user can immediately type the first argument.
            if (cmd_cursor == cmd_ix &&
                cmd_ix < sizeof(cmd_buffer) - 1 &&
                (cmd_ix == 0 || cmd_buffer[cmd_ix - 1] != ' ')) {
                insert_text_at_cursor(" ");
            }
            tab_was_last_key = false;
            return;
        }

        // Multiple matches but the typed text is already an exact command name:
        // prefer inserting a space over listing a potentially large set of similarly-prefixed aliases.
        if (has_exact && cmd_cursor == cmd_ix &&
            cmd_ix < sizeof(cmd_buffer) - 1 &&
            (cmd_ix == 0 || cmd_buffer[cmd_ix - 1] != ' ')) {
            insert_text_at_cursor(" ");
            tab_was_last_key = false;
            return;
        }

        // First Tab: extend to the longest common prefix of all matches.
        size_t lcp_len = strlen(cmds[match_idx[0]].command);
        for (size_t m = 1; m < match_count; ++m) {
            const char *s = cmds[match_idx[m]].command;
            size_t i = prefix_len;
            while (i < lcp_len && s[i] == cmds[match_idx[0]].command[i]) ++i;
            lcp_len = i;
        }
        if (lcp_len > prefix_len && cmd_cursor == cmd_ix) {
            char suffix[32];
            size_t suffix_len = lcp_len - prefix_len;
            if (suffix_len >= sizeof(suffix)) suffix_len = sizeof(suffix) - 1;
            memcpy(suffix, cmds[match_idx[0]].command + prefix_len, suffix_len);
            suffix[suffix_len] = '\0';
            insert_text_at_cursor(suffix);
            tab_was_last_key = false;
            return;
        }
        // Nothing to extend: second Tab on the same token prints the list.
        if (tab_was_last_key) {
            printf("\n");
            for (size_t m = 0; m < match_count; ++m) {
                printf("%s  ", cmds[match_idx[m]].command);
            }
            printf("\n");
            redraw_command_line();
            tab_was_last_key = false;
        } else {
            tab_was_last_key = true;
            stdio_flush();
        }
        return;
    }

    // Case 2: space found — first token is a group command; complete the second token (subcommand).
    char group[32];
    size_t group_len = first_space;
    if (group_len == 0 || group_len >= sizeof(group)) return;
    memcpy(group, cmd_buffer, group_len);
    group[group_len] = '\0';

    const char *const *subcmds = NULL;
    size_t subcmd_count = 0;
    if (!get_group_subcommands(group, &subcmds, &subcmd_count)) {
        return; // Not a grouped command; no subcommand completion available.
    }

    // Skip whitespace after the group name to find where the subcommand token starts.
    size_t sub_start = first_space;
    while (sub_start < cmd_cursor && isspace((unsigned char)cmd_buffer[sub_start])) {
        sub_start++;
    }

    // If there is already another space between sub_start and cursor, we are in arg3+ territory;
    // only complete the second token, not further arguments.
    for (size_t i = sub_start; i < cmd_cursor; ++i) {
        if (isspace((unsigned char)cmd_buffer[i])) {
            return;
        }
    }

    // Extract the partial subcommand token typed so far.
    char sub_prefix[sizeof(cmd_buffer)];
    size_t sub_prefix_len = cmd_cursor - sub_start;
    if (sub_prefix_len >= sizeof(sub_prefix)) return;
    memcpy(sub_prefix, &cmd_buffer[sub_start], sub_prefix_len);
    sub_prefix[sub_prefix_len] = '\0';

    // Match partial subcommand against the group's subcommand list.
    size_t match_idx[32];
    size_t match_count = 0;
    bool has_exact = false;
    for (size_t i = 0; i < subcmd_count; ++i) {
        if (strncmp(subcmds[i], sub_prefix, sub_prefix_len) == 0) {
            if (match_count < count_of(match_idx)) {
                match_idx[match_count++] = i;
            }
            if (strcmp(subcmds[i], sub_prefix) == 0) {
                has_exact = true;
            }
        }
    }

    if (match_count == 0) {
        printf("\a");  // Bell: no subcommand match.
        stdio_flush();
        tab_was_last_key = false;
        return;
    }

    if (match_count == 1) {
        // Unique subcommand match: fill suffix and add trailing space.
        const char *only = subcmds[match_idx[0]];
        if (cmd_cursor == cmd_ix && strlen(only) > sub_prefix_len) {
            insert_text_at_cursor(only + sub_prefix_len);
        }
        if (cmd_cursor == cmd_ix &&
            cmd_ix < sizeof(cmd_buffer) - 1 &&
            (cmd_ix == 0 || cmd_buffer[cmd_ix - 1] != ' ')) {
            insert_text_at_cursor(" ");
        }
        tab_was_last_key = false;
        return;
    }
    // Exact subcommand name typed but siblings exist: prefer space to avoid listing them.
    if (has_exact && cmd_cursor == cmd_ix &&
        cmd_ix < sizeof(cmd_buffer) - 1 &&
        (cmd_ix == 0 || cmd_buffer[cmd_ix - 1] != ' ')) {
        insert_text_at_cursor(" ");
        tab_was_last_key = false;
        return;
    }

    // First Tab: extend to the longest common prefix of all subcommand matches.
    size_t lcp_len = strlen(subcmds[match_idx[0]]);
    for (size_t m = 1; m < match_count; ++m) {
        const char *s = subcmds[match_idx[m]];
        size_t i = sub_prefix_len;
        while (i < lcp_len && s[i] == subcmds[match_idx[0]][i]) ++i;
        lcp_len = i;
    }
    if (lcp_len > sub_prefix_len && cmd_cursor == cmd_ix) {
        char suffix[32];
        size_t suffix_len = lcp_len - sub_prefix_len;
        if (suffix_len >= sizeof(suffix)) suffix_len = sizeof(suffix) - 1;
        memcpy(suffix, subcmds[match_idx[0]] + sub_prefix_len, suffix_len);
        suffix[suffix_len] = '\0';
        insert_text_at_cursor(suffix);
        tab_was_last_key = false;
        return;
    }
    // Nothing to extend: second Tab on the same token prints the list.
    if (tab_was_last_key) {
        printf("\n");
        for (size_t m = 0; m < match_count; ++m) {
            printf("%s  ", subcmds[match_idx[m]]);
        }
        printf("\n");
        redraw_command_line();
        tab_was_last_key = false;
    } else {
        tab_was_last_key = true;
        stdio_flush();
    }
}

// Break command
/**
 * @brief stdio chars-available IRQ callback used to interrupt long-running command handlers.
 *
 * Registered via stdio_set_chars_available_callback while a command handler is running.
 * Ctrl-C triggers a system reset, Esc hits a breakpoint (debug aid), and CR sets the
 * `die` flag so the handler can abort cleanly on the next iteration.
 */
static void chars_available_callback(void *ptr) {    (void)ptr;   
    int cRxedChar = getchar_timeout_us(0);
    switch (cRxedChar) {
    case 3: // Ctrl-C
        SYSTEM_RESET();
        break;
    case 27: // Esc
        __BKPT();
        break;
    case '\r':
        die = true;
    }
}

/**
 * @brief Tokenize a completed command line and dispatch to the matching handler.
 *
 * Splits cmd on spaces with strtok_r: the first token is the command name, subsequent
 * tokens become argv[0..argc-1] (argv[0] is the first *argument*, not the command name).
 * Looks up the command name in the cmds table; registers the break-key callback around
 * the handler call so Ctrl-C / CR can abort long-running operations.
 *
 * @param cmd_sz Capacity of the cmd buffer (used for bounds assertions).
 * @param cmd    Mutable command-line buffer; modified in-place by strtok_r.
 */
static void process_cmd(size_t cmd_sz, char *cmd) {
    // Command dispatcher: tokenize input, resolve handler, and execute with argc/argv.
    assert(cmd);
    assert(cmd[0]);

    // Token 1 is the command name.
    char *cmdn = strtok_r(cmd, " ", &saveptr);
    if (cmdn) {
        assert(cmdn < cmd + cmd_sz);

        // argv[0] is the first argument after the command name (not the command name itself).
        size_t argc = 0;
        // 10 slots is generous for every command in this shell; overflow is rejected by the dispatcher.
        const char *argv[10] = {0};
        const char *arg_p;
        do {
            // Remaining tokens become argv[0..argc-1].
            arg_p = strtok_r(NULL, " ", &saveptr);
            if (arg_p) {
                assert(arg_p < cmd + cmd_sz);
                if (argc >= count_of(argv)) {
                    extra_argument_msg(arg_p);
                    return;
                }
                argv[argc++] = arg_p;
            }
        } while (arg_p);

        size_t i;
        for (i = 0; i < count_of(cmds); ++i) {
            if (0 == strcmp(cmds[i].command, cmdn)) {
                
                // Allow long-running handlers to observe break keys via callback.
                stdio_set_chars_available_callback(chars_available_callback, NULL);
                // Dispatch command handler.
                (*cmds[i].function)(argc, argv);
                stdio_set_chars_available_callback(NULL, NULL);

                break;
            }
        }
        if (count_of(cmds) == i) printf("Command \"%s\" not found\n", cmdn);
    }
}

/**
 * @brief Process one input byte from stdio: run the ANSI escape FSM, handle readline shortcuts,
 *        insert/delete characters, and dispatch the command on CR/LF.
 */
void process_stdio(int cRxedChar) {
    // ANSI escape sequence FSM states.
    // Transitions: NONE -> ESC_SEEN (on ESC) -> ESC_CSI (on '[') -> action (on final byte)
    // ESC_CSI_3 handles the two-byte Delete sequence: ESC [ 3 ~
    enum esc_state_t {
        ESC_NONE = 0,
        ESC_SEEN,    // Received ESC (0x1B); waiting for '['
        ESC_CSI,     // Received ESC '['; waiting for the final byte
        ESC_CSI_3,   // Received ESC '[' '3'; waiting for '~' (Delete key)
    };
    static enum esc_state_t esc_state = ESC_NONE;

    // Reject multi-byte UTF-8 and NUL; this shell only handles 7-bit ASCII.
    if (!(0 < cRxedChar && cRxedChar <= 0x7F)) {
        return;
    }
    if (cRxedChar != '\t') tab_was_last_key = false;

    if (esc_state != ESC_NONE) {
        // We are mid-sequence in the ANSI escape FSM; route byte accordingly.
        switch (esc_state) {
        case ESC_SEEN:
            // Waiting for CSI introducer '['; anything else aborts the sequence.
            if (cRxedChar == '[') {
                esc_state = ESC_CSI;
            } else {
                esc_state = ESC_NONE;
            }
            return;
        case ESC_CSI:
            // Received ESC '['; the next byte selects the action.
            switch (cRxedChar) {
            case 'A':  // Up arrow: older history entry
                history_nav_up();
                break;
            case 'B':  // Down arrow: newer history entry
                history_nav_down();
                break;
            case 'C':  // Right arrow: move cursor one character right
                if (cmd_cursor < cmd_ix) {
                    cmd_cursor++;
                    redraw_command_line();
                }
                break;
            case 'D':  // Left arrow: move cursor one character left
                if (cmd_cursor > 0) {
                    cmd_cursor--;
                    redraw_command_line();
                }
                break;
            case 'H':  // Home: move cursor to beginning of line
                cmd_cursor = 0;
                redraw_command_line();
                break;
            case 'F':  // End: move cursor to end of line
                cmd_cursor = cmd_ix;
                redraw_command_line();
                break;
            case '3':  // Start of Delete key sequence (ESC [ 3 ~): wait for the '~'
                esc_state = ESC_CSI_3;
                return;
            default:
                break;
            }
            esc_state = ESC_NONE;
            return;
        case ESC_CSI_3:
            // Complete the Delete sequence: ESC [ 3 ~
            if (cRxedChar == '~') {
                handle_delete_at_cursor();
            }
            esc_state = ESC_NONE;
            return;
        case ESC_NONE:
        default:
            break;
        }
    }
    if (cRxedChar == 27) {  // ESC: begin ANSI escape sequence
        esc_state = ESC_SEEN;
        return;
    }

    // Readline-compatible editing shortcuts (Emacs bindings).
    if (cRxedChar == 1) {   // Ctrl+A: move to start of line
        handle_move_home();
        return;
    }    if (cRxedChar == 5) {   // Ctrl+E: move to end of line
        handle_move_end();
        return;
    }    if (cRxedChar == 11) {  // Ctrl+K: kill from cursor to end of line
        handle_kill_to_eol();
        return;
    }    if (cRxedChar == 21) {  // Ctrl+U: kill from start of line to cursor
        handle_kill_to_bol();
        return;
    }
    // Discard any other non-printable byte that is not one of the handled specials below.
    if (!isprint(cRxedChar) && '\r' != cRxedChar && '\n' != cRxedChar &&
        '\t' != cRxedChar &&
        '\b' != cRxedChar && cRxedChar != 127) {
        return;
    }

    // CRLF handling: Windows/serial terminals send CR then LF; suppress the LF so we don't
    // process two Enter events from one keypress.
    if (cRxedChar == '\n' && last_input_was_cr) {
        last_input_was_cr = false;
        return;
    }
    if (cRxedChar == '\r' || cRxedChar == '\n') {
        last_input_was_cr = (cRxedChar == '\r');
        printf("%c", cRxedChar);  // echo Enter
        stdio_flush();
        // Print a blank line so command output starts on a fresh line below the prompt.
        printf("%c", '\n');
        stdio_flush();

        if (!cmd_buffer[0]) {  // Empty line: just reprint the prompt and do nothing
            reset_command_line_state();
            print_prompt();
            return;
        }

        // Save to history before strtok_r mutates cmd_buffer during dispatch.
        add_command_to_history(cmd_buffer);

        // Tokenize and dispatch the completed command line.
        process_cmd(sizeof cmd_buffer, cmd_buffer);

        // Clear all edit state for the next command line.
        reset_command_line_state();
        memset(cmd_buffer, 0, sizeof cmd_buffer);
        printf("\n");
        print_prompt();
    } else {  // Printable character, Tab, or Backspace/Delete
        last_input_was_cr = false;
        if (cRxedChar == '\t') {
            handle_tab_completion();
        } else if (cRxedChar == '\b' || cRxedChar == (char)127) {
            handle_backspace();
        } else {
            handle_insert_char((char)cRxedChar);
        }
    }
}

/**
 * @brief Internal helper: command input in progress.
 *
 * @return `true` on success, `false` otherwise.
 */
bool command_input_in_progress(void) {
    return cmd_ix > 0;
}

/**
 * @brief Run all non-blocking background state machines once per main-loop iteration.
 *
 * Called from the main loop as frequently as possible.  Each state machine checks its
 * own pending flag and time guard before doing any work, so back-to-back calls are cheap
 * when nothing is due.  The four machines are:
 *   1. cam_snap        — single-frame capture to SD file
 *   2. lcd_cam_snap    — single-frame capture to LCD display
 *   3. lcd_cam_stream  — continuous camera-to-LCD preview (Mode A or Mode B)
 *   4. lcd_load        — raw image file load from SD to LCD
 */
void process_background_tasks(void) {
    // Run lightweight per-frame housekeeping before the heavier state machines.
    // These are cheap: pipeline_apply checks a flag, touch/nrf poll check timestamps.
    pipeline_apply_pending_mode_if_safe();
    touch_handle_mirror_event_if_needed();
    nrf_poll_button_event_if_needed();

    // --- State machine 1: cam_snap (single frame -> SD file) ---
    // Phases: WAIT_FRAME -> OPEN_FILE -> WRITE_FILE (repeated) -> CLOSE_FILE
    if (cam_snap_pending && time_reached(cam_snap_next_step_time)) {
        switch (cam_snap_state) {
        case CAM_SNAP_WAIT_FRAME: {
            // Poll cam_wait_for_frame() non-blocking (timeout=0); advance when DMA signals a complete frame.
            if (cam_wait_for_frame(0)) {
                cam_snap_state = CAM_SNAP_OPEN_FILE;
                cam_snap_next_step_time = get_absolute_time();
                return;
            }
            // Timeout guard: if the camera hasn't produced a frame within 2 s, abort.
            if (time_reached(cam_snap_deadline)) {
                printf("\ncam snap: TIMEOUT — DMA busy=%d buffer_ready=%d\n",
                       cam_get_dma_busy(), buffer_ready);
                finish_cam_snap();
                print_prompt();
                return;
            }
            // Progress: print DMA state every 500 ms so the user can see activity.
            static absolute_time_t cam_snap_next_diag = {0};
            if (time_reached(cam_snap_next_diag)) {
                cam_snap_next_diag = delayed_by_ms(get_absolute_time(), 500);
                printf("\ncam snap: waiting for frame (DMA busy=%d buffer_ready=%d)\n",
                       cam_get_dma_busy(), buffer_ready);
                print_prompt();
            }
            return;
        }

        case CAM_SNAP_OPEN_FILE: {
            // Auto-create parent directory if it doesn't exist.
            {
                char dir_buf[sizeof(cam_snap_path)];
                strncpy(dir_buf, cam_snap_path, sizeof(dir_buf) - 1);
                dir_buf[sizeof(dir_buf) - 1] = '\0';
                char *last_slash = strrchr(dir_buf, '/');
                if (last_slash && last_slash != dir_buf) {
                    *last_slash = '\0';
                    FRESULT mfr = f_mkdir(dir_buf);
                    if (mfr != FR_OK && mfr != FR_EXIST) {
                        printf("\nf_mkdir error: %s (%d)\n", FRESULT_str(mfr), mfr);
                        finish_cam_snap();
                        print_prompt();
                        return;
                    }
                }
            }
            // Create the destination file; abort the snap on any FatFS error.
            FRESULT fr = f_open(&cam_snap_file, cam_snap_path, FA_WRITE | FA_CREATE_ALWAYS);
            if (FR_OK != fr) {
                printf("\nf_open error: %s (%d)\n", FRESULT_str(fr), fr);
                finish_cam_snap();
                print_prompt();
                return;
            }
            cam_snap_file_open = true;
            cam_snap_state = CAM_SNAP_WRITE_FILE;
            cam_snap_next_step_time = delayed_by_ms(get_absolute_time(), 1);
            return;
        }

        case CAM_SNAP_WRITE_FILE: {
            // Write one chunk (cam_snap_chunk_size bytes) per tick to keep the main loop responsive.
            uint32_t total_bytes = cam_ful_size * 2;
            if (cam_snap_write_offset >= total_bytes) {
                // All data written; proceed to close the file.
                cam_snap_state = CAM_SNAP_CLOSE_FILE;
                return;
            }

            UINT chunk = cam_snap_chunk_size;
            uint32_t remaining = total_bytes - cam_snap_write_offset;
            if (remaining < chunk) {
                chunk = (UINT)remaining;  // Last partial chunk
            }

            UINT bw = 0;
            FRESULT fr = f_write(&cam_snap_file, cam_ptr + cam_snap_write_offset, chunk, &bw);
            if (FR_OK != fr || bw != chunk) {
                if (FR_OK != fr) {
                    printf("\nf_write error: %s (%d)\n", FRESULT_str(fr), fr);
                } else {
                    printf("\nShort write: %u of %u bytes\n", bw, chunk);
                }
                finish_cam_snap();
                print_prompt();
                return;
            }

            cam_snap_write_offset += bw;
            cam_snap_total_written += bw;
            cam_snap_next_step_time = get_absolute_time();
            return;
        }

        case CAM_SNAP_CLOSE_FILE: {
            // Copy path and byte count before finish_cam_snap() resets them.
            char completed_path[sizeof(cam_snap_path)];
            strlcpy(completed_path, cam_snap_path, sizeof(completed_path));
            UINT completed_written = cam_snap_total_written;
            finish_cam_snap();
            printf("\nWrote %u bytes to %s\n", completed_written, completed_path);
            print_prompt();
            return;
        }

        case CAM_SNAP_IDLE:
        default:
            break;
        }
    }

    // --- State machine 2: lcd_cam_snap (single frame -> LCD) ---
    // Phases: WAIT_FRAME -> DISPLAY_ROWS (repeated until all rows sent)
    if (lcd_cam_snap_pending && time_reached(lcd_cam_snap_next_step_time)) {
        switch (lcd_cam_snap_state) {
        case LCD_CAM_SNAP_WAIT_FRAME:
            // Poll for a complete frame; advance once the DMA buffer is ready.
            if (cam_wait_for_frame(0)) {
                lcd_cam_snap_state = LCD_CAM_SNAP_DISPLAY_ROWS;
                lcd_cam_snap_row = 0;
                lcd_cam_snap_next_step_time = delayed_by_ms(get_absolute_time(), 1);
                return;
            }
            // Abort on timeout (2 s).
            if (time_reached(lcd_cam_snap_deadline)) {
                finish_lcd_cam_snap();
                printf("\nLCD camera snapshot timed out after 2000 ms\n");
                print_prompt();
            }
            return;

        case LCD_CAM_SNAP_DISPLAY_ROWS: {
            // Push one chunk of rows per tick using DMA; lcd_display_raw_rows() handles the full frame
            // on the first call (start_row == 0) and is a no-op for subsequent calls.
            if (lcd_cam_snap_row >= cam_height) {
                finish_lcd_cam_snap();
                printf("\nCaptured and displayed camera frame on LCD\n");
                print_prompt();
                return;
            }

            uint32_t row_count = cam_height - lcd_cam_snap_row;
            if (row_count > lcd_display_row_chunk) {
                row_count = lcd_display_row_chunk;
            }

            lcd_display_raw_rows(lcd_cam_snap_row,
                                 row_count,
                                 cam_ptr + (lcd_cam_snap_row * cam_width * 2));
            lcd_cam_snap_row += row_count;
            lcd_cam_snap_next_step_time = delayed_by_ms(get_absolute_time(), 1);
            return;
        }

        case LCD_CAM_SNAP_IDLE:
        default:
            break;
        }
    }

    // --- State machine 3: lcd_cam_stream (continuous camera -> LCD) ---
    // Phases: WAIT_FRAME -> DISPLAY_ROWS (back to WAIT_FRAME each frame)
    if (lcd_cam_stream_active && time_reached(lcd_cam_stream_next_step_time)) {
        switch (lcd_cam_stream_state) {
        case LCD_CAM_STREAM_WAIT_FRAME:
            // Non-blocking poll for a fresh camera frame from DMA.
            if (cam_wait_for_frame(0)) {
                lcd_cam_stream_state = LCD_CAM_STREAM_DISPLAY_ROWS;
                lcd_cam_stream_row = 0;
                lcd_cam_stream_next_step_time = get_absolute_time();  // No inter-tick delay in display phase
                return;
            }
            // Per-frame timeout: if camera stops producing frames (e.g. PIO stall), stop the stream.
            if (time_reached(lcd_cam_stream_deadline)) {
                finish_lcd_cam_stream();
                printf("\nLCD camera stream timed out after 2000 ms\n");
                print_prompt();
            }
            return;

        case LCD_CAM_STREAM_DISPLAY_ROWS: {
            if (lcd_cam_stream_row >= cam_height) {
                // All rows for this frame have been sent; re-arm the camera and wait for the next frame.
                buffer_ready = false;
                config_cam_buffer();
                start_cam();
                lcd_cam_stream_state = LCD_CAM_STREAM_WAIT_FRAME;
                lcd_cam_stream_deadline = make_timeout_time_ms(2000);
                lcd_cam_stream_next_step_time = get_absolute_time();
                return;
            }

            uint32_t row_count = cam_height - lcd_cam_stream_row;
            if (row_count > lcd_display_row_chunk) {
                row_count = lcd_display_row_chunk;
            }

            if (pipeline_mode == PIPE_MODE_B_NRF && pipeline_validation_mode_b_enabled) {
                // Mode B: relay the frame through the NRF loopback path one chunk at a time.
                // Lock cam_display_ptr at the start of each new frame transfer.
                // cam_display_ptr flips every ~40ms (camera DMA), but Mode B needs
                // ~80ms to relay one frame chunk-by-chunk. Without locking, the pointer
                // changes mid-transfer and mode_b_ctx.frame != frame resets it to row 0.
                if (!mode_b_ctx.active) {
                    mode_b_stream_locked_frame = cam_display_ptr;
                }
                int st = pipeline_mode_b_transfer_frame_via_nrf_to_lcd(mode_b_stream_locked_frame);
                if (st < 0) {
                    // NRF SPI error (all-0xFF or echo mismatch): abandon this frame,
                    // fall back to Mode A, and block NRF events for mode_b_fail_block_ms.
                    reset_mode_b_transfer_ctx();
                    mode_b_stream_locked_frame = NULL;
                    pipeline_force_mode_now(PIPE_MODE_A_TOUCH, PIPE_REQ_SHELL);
                    nrf_event_block_until = delayed_by_ms(get_absolute_time(), mode_b_fail_block_ms);
                    printf("\nMode-B transfer failed (NRF SPI). Falling back to mode A.\n");
                    print_prompt();
                    lcd_cam_stream_row = cam_height;  // Skip remaining rows; restart at frame boundary
                    lcd_cam_stream_next_step_time = get_absolute_time();
                    return;
                }
                if (st == 0) {
                    // Still transferring this frame (chunks remain); schedule next tick.
                    lcd_cam_stream_next_step_time = delayed_by_ms(get_absolute_time(), 1);
                    return;
                }
                // st == 1: Mode-B frame fully validated and displayed; fall through to advance row counter.
                mode_b_stream_locked_frame = NULL;
            } else {
                // Mode A: direct DMA push of camera data to LCD (no NRF relay).
                // If a stale Mode-B context exists (e.g. from a mid-stream mode switch), discard it.
                if (mode_b_ctx.active) {
                    reset_mode_b_transfer_ctx();
                }
                shared_enter_lcd_active();
                lcd_display_raw_rows(lcd_cam_stream_row,
                                     row_count,
                                     cam_display_ptr + (lcd_cam_stream_row * cam_width * 2));
            }
            // lcd_display_raw_rows() sends the full frame on row 0 via DMA; mark all rows done.
            lcd_cam_stream_row = cam_height;
            lcd_cam_stream_next_step_time = get_absolute_time();
            return;
        }

        case LCD_CAM_STREAM_IDLE:
        default:
            break;
        }
    }

    // --- State machine 4: lcd_load (SD image file -> framebuffer -> LCD) ---
    // Early-exit when no load is pending or the next step is not yet due.
    if (!lcd_load_pending) {
        return;
    }

    if (!time_reached(lcd_load_next_step_time)) {
        return;
    }

    switch (lcd_load_state) {
    case LCD_LOAD_OPEN_FILE: {
        // Open the image file and validate its size before reading.
        FRESULT fr = f_open(&lcd_load_file, lcd_load_path, FA_READ);
        if (FR_OK != fr) {
            printf("\nf_open error: %s (%d)\n", FRESULT_str(fr), fr);
            finish_lcd_load();
            print_prompt();
            return;
        }

        lcd_load_file_open = true;

        // Reject files that don't exactly match the framebuffer size (wrong resolution or format).
        if (f_size(&lcd_load_file) != lcd_image_bytes) {
            printf("\nImage file size mismatch: expected %u bytes\n", (unsigned)lcd_image_bytes);
            finish_lcd_load();
            print_prompt();
            return;
        }

        lcd_load_state = LCD_LOAD_READ_FILE;
        lcd_load_next_step_time = delayed_by_ms(get_absolute_time(), 1);
        return;
    }

    case LCD_LOAD_READ_FILE: {
        // Read lcd_display_row_chunk rows per tick: copy into framebuffer (with byte swap), push to LCD.
        if (lcd_load_display_row >= LCD_2IN.HEIGHT) {
            // All rows read and displayed; proceed to close the file.
            lcd_load_state = LCD_LOAD_CLOSE_FILE;
            return;
        }

        uint32_t row_count = LCD_2IN.HEIGHT - lcd_load_display_row;
        if (row_count > lcd_display_row_chunk) {
            row_count = lcd_display_row_chunk;
        }

        UINT chunk = (UINT)(LCD_2IN.WIDTH * 2 * row_count);

        UINT br = 0;
        FRESULT fr = f_read(&lcd_load_file, lcd_row_buffer, chunk, &br);
        if (FR_OK != fr || br != chunk) {
            if (FR_OK != fr) {
                printf("\nf_read error: %s (%d)\n", FRESULT_str(fr), fr);
            } else {
                printf("\nShort read: %u of %u bytes\n", br, chunk);
            }
            finish_lcd_load();
            print_prompt();
            return;
        }

        // Convert from file (camera little-endian) byte order to panel big-endian, then push to hardware.
        lcd_copy_raw_rows_to_framebuffer(lcd_load_display_row, row_count, lcd_row_buffer);
        lcd_display_rows_from_framebuffer(lcd_load_display_row, row_count);
        lcd_load_display_row += row_count;
        lcd_load_offset += br;
        lcd_load_next_step_time = delayed_by_ms(get_absolute_time(), 1);
        return;
    }

    case LCD_LOAD_CLOSE_FILE: {
        // Close the file and report completion (or a custom status message if one was set).
        FRESULT fr = f_close(&lcd_load_file);
        lcd_load_file_open = false;
        if (FR_OK != fr) {
            printf("\nf_close error: %s (%d)\n", FRESULT_str(fr), fr);
            finish_lcd_load();
            print_prompt();
            return;
        }

        if (lcd_load_status_msg[0]) {
            printf("\n%s\n", lcd_load_status_msg);
        } else {
            printf("\nLoaded and displayed %s\n", lcd_load_path);
        }
        finish_lcd_load();
        print_prompt();
        return;
    }

    case LCD_LOAD_DISPLAY: {
        // Push the existing framebuffer content to the LCD panel row-by-row without reading from SD.
        // Used after in-memory operations (Paint_Clear, etc.) that modify lcd_image directly.
        if (lcd_load_display_row >= LCD_2IN.HEIGHT) {
            if (lcd_load_status_msg[0]) {
                printf("\n%s\n", lcd_load_status_msg);
            } else {
                printf("\nLCD framebuffer displayed\n");
            }
            finish_lcd_load();
            print_prompt();
            return;
        }

        uint32_t row_count = LCD_2IN.HEIGHT - lcd_load_display_row;
        if (row_count > lcd_display_row_chunk) {
            row_count = lcd_display_row_chunk;
        }

        lcd_display_rows_from_framebuffer(lcd_load_display_row, row_count);
        lcd_load_display_row += row_count;
        lcd_load_next_step_time = delayed_by_ms(get_absolute_time(), 1);
        return;
    }

    case LCD_LOAD_IDLE:
    default:
        return;
    }
}

