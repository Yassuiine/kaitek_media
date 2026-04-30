#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
//
#include "pico/stdlib.h"

#ifdef __cplusplus
extern "C" {
#endif

// ─── Globals moved to feature files ──────────────────────────────────────────
// logger_enabled / period / next_log_time  → cmd_sd.h
// NRF globals                              → cmd_nrf.h
// LCD globals                              → cmd_lcd.h
// CAM globals                              → cmd_cam.h

// ─── Cross-feature state (defined in command.cpp) ────────────────────────────
extern bool lcd_cam_snap_pending;    // True while a one-shot cam-to-LCD snapshot is in progress
extern bool lcd_cam_stream_active;   // True while continuous camera-to-LCD streaming is running
extern absolute_time_t nrf_event_poll_next_time;  // Next time standalone NRF poll is allowed
extern absolute_time_t nrf_event_block_until;     // Block NRF events until this time (debounce)

// ─── Shell abort flag ────────────────────────────────────────────────────────
extern volatile bool die;   // Set by chars_available_callback on Enter; used to abort running loop commands

// ─── Shell helpers (non-static, usable from feature TUs) ─────────────────────
void missing_argument_msg(void);
void extra_argument_msg(const char *s);
bool expect_argc(size_t argc, const char *argv[], size_t expected);
bool parse_u32_arg(const char *s, uint32_t *out);
bool parse_hex_string_bytes(const char *hex, uint8_t *out, size_t out_cap, size_t *out_len);
bool parse_hex_tokens_bytes(size_t argc, const char *argv[], uint8_t *out, size_t out_cap, size_t *out_len);
void print_hex_bytes(const uint8_t *buf, size_t len);
void print_prompt(void);

// ─── Shared-pin / pipeline helpers (non-static, usable from feature TUs) ─────
void shared_enter_lcd_active(void);
void shared_enter_nrf_transaction(void);
void shared_enter_cam_prog(void);
void shared_leave_cam_prog(void);
void shared_pin_state_set_nrf_transaction(void);  // Set shared_pin_state = NRF_TRANSACTION without re-init

// ─── Touch controller ─────────────────────────────────────────────────────────
bool touch_init_controller(void);

// ─── Main entry points ───────────────────────────────────────────────────────
void process_stdio(int cRxedChar);
void process_background_tasks(void);
bool command_input_in_progress(void);

#ifdef __cplusplus
}
#endif
