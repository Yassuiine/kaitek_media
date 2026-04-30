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
#include "cmd_sd.h"
#include "cmd_cam.h"
#include "cmd_lcd.h"
#include "cmd_nrf.h"
#include "DEV_Config.h"
#include "LCD_2in.h"
#include "GUI_Paint.h"

#define NRF_BENCH_ENABLED  // Compile with this defined to enable the SPI slave test mode that logs transfer stats and button events; undefine for production build without logging and NRF event handling
#ifndef ENABLE_PATH_AB_LOGIC
#define ENABLE_PATH_AB_LOGIC 1
#endif

static char *saveptr;                    // strtok_r re-entrant parse position; shared across all tokenisation in process_cmd
volatile bool die;                // Set by chars_available_callback on Enter to abort a running loop command
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

// CAM snap globals defined in cmd_cam.cpp (extern via cmd_cam.h).
// LCD display globals defined in cmd_lcd.cpp (extern via cmd_lcd.h).
static bool last_input_was_cr;                  // Tracks whether the previous byte was CR to suppress the following LF (CRLF → CR)

// NRF SPI globals and FFT stream globals are defined in cmd_nrf.cpp.
// Extern declarations are in cmd_nrf.h (included above).

static bool touch_initialized = false;              // True after touch_init_controller() confirms the CST816D chip ID
static bool touch_press_latched = false;            // True while a press is held; prevents repeated trigger events
// cam_mirror_enabled defined in cmd_cam.cpp (extern via cmd_cam.h)
static uint32_t touch_event_count = 0;             // Total number of debounced touch events that triggered a mirror toggle
static uint32_t nrf_event_count = 0;               // Total number of NRF button events that triggered an action
static absolute_time_t touch_next_poll_time;        // Earliest time to re-sample the Touch_INT_PIN
absolute_time_t nrf_event_poll_next_time;    // Earliest time to send the next standalone NRF poll transfer
absolute_time_t nrf_event_block_until;       // Block NRF event processing until this time (debounce / fail-recovery)
static const uint8_t touch_addr = 0x15;             // I2C address of the CST816D touch controller on I2C0
static const uint8_t touch_reg_chip_id = 0xA7;      // CST816D register that returns the chip identification byte
static const uint8_t touch_expected_chip_id = 0xB6; // Expected chip-ID value confirming a genuine CST816D is present
static const uint32_t touch_poll_interval_ms = 8;   // How often to sample Touch_INT_PIN (ms); ~125 Hz
static const uint32_t touch_debounce_ms = 180;      // Minimum gap between two consecutive touch events (ms)
static const uint32_t nrf_event_poll_interval_ms = 35; // Interval between standalone NRF button-poll transfers in Mode A (ms)
static absolute_time_t touch_debounce_deadline;     // Earliest time a new touch event will be accepted after the last one

/**
 * @brief Forward declarations for helpers used before full definitions.
 * NRF and LCD functions are declared in cmd_nrf.h and cmd_lcd.h (included above).
 * lcd_con_putchar is declared in cmd_lcd.h.
 */
void print_prompt(void) {
    printf("%s", PROMPT_STR);
    stdio_flush();
}

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


// cam_snap_state_t defined in cmd_cam.h.
// lcd_load_state_t defined in cmd_lcd.h.

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

// lcd_load_state machine globals defined in cmd_lcd.cpp (extern via cmd_lcd.h).
// spi_tx_dma_ch, spi_rx_dma_ch, spi_rx_dummy defined in cmd_nrf.cpp (extern via cmd_nrf.h).

static lcd_cam_snap_state_t lcd_cam_snap_state = LCD_CAM_SNAP_IDLE; // Current phase of the one-shot cam-to-LCD snapshot state machine
bool lcd_cam_snap_pending = false;                            // True while an lcd_cam_snap operation is in progress
static uint32_t lcd_cam_snap_row = 0;                                // Next row to push to the LCD in the DISPLAY_ROWS phase
static absolute_time_t lcd_cam_snap_deadline;                        // Absolute time after which the frame-wait phase times out
static absolute_time_t lcd_cam_snap_next_step_time;                  // Earliest time for the next state machine step

static lcd_cam_stream_state_t lcd_cam_stream_state = LCD_CAM_STREAM_IDLE; // Current phase of the continuous camera stream state machine
bool lcd_cam_stream_active = false;                                 // True while lcd_cam_stream is running
static uint32_t lcd_cam_stream_row = 0;                                    // Next row to process in the DISPLAY_ROWS phase
static absolute_time_t lcd_cam_stream_deadline;                            // Absolute time after which the frame-wait phase times out
static absolute_time_t lcd_cam_stream_next_step_time;                      // Earliest time for the next stream state machine step
// mode_b_stream_locked_frame defined in cmd_nrf.cpp (extern via cmd_nrf.h)
static pipeline_mode_t pipeline_mode = PIPE_MODE_A_TOUCH;                     // Currently active pipeline mode
static pipeline_mode_t pipeline_mode_pending = PIPE_MODE_A_TOUCH;             // Target mode for a deferred switch request
static pipeline_mode_request_source_t pipeline_mode_pending_source = PIPE_REQ_NONE; // Originator of the pending switch request (for logging)
static bool pipeline_mode_switch_pending = false;                              // True when a mode switch has been requested but not yet applied
static bool pipeline_validation_mode_b_enabled = false;                        // When true, streaming uses the full NRF echo-validation path
static shared_pin_state_t shared_pin_state = SHARED_PIN_LCD_ACTIVE;            // Current logical owner of the shared SPI/I2C pins

// cam_snap_state and cam_snap_chunk_size defined in cmd_cam.cpp (extern via cmd_cam.h).
// logger_enabled, period, next_log_time defined in cmd_sd.cpp (extern via cmd_sd.h).

#pragma GCC diagnostic ignored "-Wunused-function"
#ifdef NDEBUG 
#  pragma GCC diagnostic ignored "-Wunused-variable"
#  pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

/**
 * @brief Print "Missing argument" to signal the user didn't provide enough args.
 */
void missing_argument_msg(void) {
    printf("Missing argument\n");
}
/**
 * @brief Print "Unexpected argument: <s>" to signal the user passed too many args.
 */
void extra_argument_msg(const char *s) {
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
bool expect_argc(const size_t argc, const char *argv[], const size_t expected) {
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
bool parse_u32_arg(const char *s, uint32_t *out) {
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
static int hex_nibble(char c) {  // internal helper; not exposed in command.h
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
bool parse_hex_string_bytes(const char *hex, uint8_t *out, size_t out_cap, size_t *out_len) {
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
bool parse_hex_tokens_bytes(const size_t argc, const char *argv[], uint8_t *out, size_t out_cap, size_t *out_len) {
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
void print_hex_bytes(const uint8_t *buf, size_t len) {    for (size_t i = 0; i < len; ++i) {
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
void shared_enter_lcd_active(void) {
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
void shared_enter_nrf_transaction(void) {
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
void shared_enter_cam_prog(void) {
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
void shared_leave_cam_prog(void) {
    if (pipeline_mode == PIPE_MODE_B_NRF && pipeline_validation_mode_b_enabled) {
        shared_enter_nrf_transaction();
    } else {
        shared_enter_lcd_active();
    }
}

/**
 * @brief Set shared_pin_state to NRF_TRANSACTION without re-initialising the bus.
 *
 * Called from cmd_nrf.cpp's nrf_spi_prepare_bus() which does the actual SPI init;
 * this function only updates the logical ownership flag in command.cpp.
 */
void shared_pin_state_set_nrf_transaction(void) {
    shared_pin_state = SHARED_PIN_NRF_TRANSACTION;
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

// nrf_spi_transfer_bytes, ensure_mode_b_frame_raw, free_mode_b_frame_raw,
// bytes_all_value, reset_mode_b_transfer_ctx moved to cmd_nrf.cpp.

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
        if (byte_count > ((size_t)(LCD_2IN_WIDTH * 2 * NRF_VALIDATION_ROWS_MAX))) {
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
    memset(nrf_validation_dummy_chunk, 0, ((size_t)(LCD_2IN_WIDTH * 2 * NRF_VALIDATION_ROWS_MAX)));
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
bool touch_init_controller(void) {
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
    if (nrf_fft_stream_is_active()) return;
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

// SD/CAM/LCD functions moved to cmd_sd.cpp, cmd_cam.cpp, cmd_lcd.cpp.

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
 */
static void finish_lcd_cam_stream(void) {
    DEV_Digital_Write(LCD_CS_PIN, 1);  // Safety: deassert CS if stopped mid-frame
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

static void run_lcd_cam_snap(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;
    if (nrf_fft_stream_is_active()) {
        printf("Stop nrf stream before lcd snap.\n");
        return;
    }
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
 * @brief Start or stop the continuous camera-to-LCD background stream.
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
    if (nrf_fft_stream_is_active()) {
        printf("Stop nrf stream before lcd stream.\n");
        return;
    }
    lcd_con_stop();
    if (!ensure_lcd_camera_ready("lcd_cam_stream")) return;
    if (cam_snap_pending || lcd_load_pending || lcd_cam_snap_pending || lcd_cam_stream_active) {
        printf("Another camera or LCD operation is already in progress.\n");
        return;
    }

    cam_ptr1 = (uint8_t *)lcd_image;
    cam_set_continuous(true);
    cam_set_use_irq(true);
    buffer_ready = false;
    reset_mode_b_transfer_ctx();
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
#if !ENABLE_PATH_AB_LOGIC
    printf("Path A/B logic is disabled at compile time.\n");
    return;
#endif

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
#if !ENABLE_PATH_AB_LOGIC
    printf("Path A/B logic is disabled at compile time.\n");
    return;
#endif

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
#if !ENABLE_PATH_AB_LOGIC
    printf("Path A/B logic is disabled at compile time.\n");
    return;
#endif

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
#if !ENABLE_PATH_AB_LOGIC
    printf("Path A/B logic is disabled at compile time.\n");
    return;
#endif

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

// NRF SPI functions (nrf_spi_prepare_bus through run_nrf_timing) moved to cmd_nrf.cpp.
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
#if !ENABLE_PATH_AB_LOGIC
    printf("sm: Path A/B logic disabled at compile time.\n");
    return;
#endif
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
    printf("Subcommands: init, status, xfer, sweep, bench, diag, timing, fft\n");
#else
    printf("Subcommands: init, status, xfer, timing, fft\n");
#endif
    printf("For realtime sensor graph control: nrf stream help\n");
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
    "timing", "stream",
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
#if !ENABLE_PATH_AB_LOGIC
    (void)argc;
    (void)argv;
    printf("sm: Path A/B logic disabled at compile time.\n");
    return;
#else
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
#endif
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
    if (nrf_fft_stream_is_active() &&
        (strcmp(sub, "init") == 0 ||
         strcmp(sub, "xfer") == 0 ||
         strcmp(sub, "sweep") == 0
#ifdef NRF_BENCH_ENABLED
         || strcmp(sub, "bench") == 0 ||
         strcmp(sub, "diag") == 0
#endif
        )) {
        printf("Stop nrf stream before running nrf %s.\n", sub);
        return;
    }
    if (strcmp(sub, "init") == 0) run_nrf_spi_init(sub_argc, sub_argv);
    else if (strcmp(sub, "status") == 0) run_nrf_spi_status(sub_argc, sub_argv);
    else if (strcmp(sub, "xfer") == 0) run_nrf_spi_xfer(sub_argc, sub_argv);
    else if (strcmp(sub, "sweep") == 0) run_nrf_spi_sweep(sub_argc, sub_argv);
#ifdef NRF_BENCH_ENABLED
    else if (strcmp(sub, "bench") == 0) run_nrf_spi_bench(sub_argc, sub_argv);
    else if (strcmp(sub, "diag") == 0) run_nrf_spi_diag(sub_argc, sub_argv);
#endif
    else if (strcmp(sub, "timing") == 0) run_nrf_timing(sub_argc, sub_argv);
    else if (strcmp(sub, "stream") == 0) run_nrf_stream_public(sub_argc, sub_argv);
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
    {"nrf_stream", run_nrf_stream_public,
     "nrf_stream <start|stop|status|sensor|period|fft> [args]:\n"
     " Realtime sensor streaming from BM20_C to RP2350 over SPI with LCD graph.\n"
     " start [period_ms] starts 320-point spectrum/waveform rendering on the LCD.\n"
     " stop  halts polling and sends STOP to the slave.\n"
     " status prints counters/Ts/sequence and FFT state.\n"
     " sensor <0|1|2> selects simulated sensor source.\n"
     " period <20..1000> sets refresh period override (default: Ts+1 ms).\n"
     " fft   toggles FFT on/off (FFT=on: frequency spectrum; FFT=off: waveform)."},
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
 * Plus a periodic SPI poll task for nrf fft stream mode (command-driven FFT graph).
 */
void process_background_tasks(void) {
    // Run lightweight per-frame housekeeping before the heavier state machines.
#if ENABLE_PATH_AB_LOGIC
    // Path-A/B mode arbitration + event sources.
    pipeline_apply_pending_mode_if_safe();
    touch_handle_mirror_event_if_needed();
#endif
    nrf_fft_stream_task_tick();
#if ENABLE_PATH_AB_LOGIC
    nrf_poll_button_event_if_needed();
#endif

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

            #if ENABLE_PATH_AB_LOGIC
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
            #endif
                // Mode A: direct DMA push of camera data to LCD (no NRF relay).
                // If a stale Mode-B context exists (e.g. from a mid-stream mode switch), discard it.
                if (mode_b_ctx.active) {
                    reset_mode_b_transfer_ctx();
                }
                shared_enter_lcd_active();
                lcd_display_raw_rows(lcd_cam_stream_row,
                                     row_count,
                                     cam_display_ptr + (lcd_cam_stream_row * cam_width * 2));
            #if ENABLE_PATH_AB_LOGIC
            }
            #endif
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

