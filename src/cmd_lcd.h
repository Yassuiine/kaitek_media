#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
//
#include "pico/stdlib.h"
#include "ff.h"
#include "fonts.h"
#include "LCD_2in.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Asynchronous LCD image load/unload phases.
 */
typedef enum {
    LCD_LOAD_IDLE = 0,
    LCD_LOAD_OPEN_FILE,
    LCD_LOAD_READ_FILE,
    LCD_LOAD_CLOSE_FILE,
    LCD_LOAD_DISPLAY,
} lcd_load_state_t;

// Globals (extern declarations)
extern bool lcd_initialized;
extern bool lcd_con_active;
extern uint8_t lcd_con_ansi_state;
extern int lcd_con_x;
extern int lcd_con_y;
extern uint16_t lcd_con_fg;
extern uint16_t lcd_con_bg;
extern const sFONT *lcd_con_font;
extern int lcd_con_width;
extern int lcd_con_height;
extern uint8_t lcd_backlight;
extern UBYTE lcd_scan_dir;
extern UWORD *lcd_image;
extern size_t lcd_image_bytes;
extern const uint32_t lcd_spi_hz;
extern uint32_t lcd_display_fps;
extern uint32_t lcd_display_frames_total;
extern uint32_t lcd_display_frames_window;
extern uint64_t lcd_display_window_start_us;
extern lcd_load_state_t lcd_load_state;
extern bool lcd_load_pending;
extern FIL lcd_load_file;
extern bool lcd_load_file_open;
extern char lcd_load_path[256];
extern char lcd_load_status_msg[128];
extern UINT lcd_load_offset;
extern uint32_t lcd_load_display_row;
extern absolute_time_t lcd_load_next_step_time;
extern const uint32_t lcd_display_row_chunk;
extern uint8_t lcd_row_buffer[];

// Function declarations
void run_lcd_init(size_t argc, const char *argv[]);
void run_lcd_bl(size_t argc, const char *argv[]);
void run_lcd_clear(size_t argc, const char *argv[]);
void run_lcd_pixel(size_t argc, const char *argv[]);
void run_lcd_fillrect(size_t argc, const char *argv[]);
void run_lcd_text(size_t argc, const char *argv[]);
void run_lcd_set_orientation(size_t argc, const char *argv[]);

void lcd_con_draw_char(char c, int x, int y);
void lcd_con_scroll(void);
void lcd_con_putchar(char c);
void lcd_con_stop(void);
void lcd_con_start(uint16_t fg, uint16_t bg, const sFONT *font);
void run_lcdcon(size_t argc, const char *argv[]);

void finish_lcd_load(void);
void fail_lcd_load(const char *msg);
bool start_lcd_framebuffer_display(const char *status_msg);
void lcd_note_displayed_frame(void);

void lcd_convert_raw_rgb565_to_panel_bytes(const uint8_t *src, uint8_t *dst, uint32_t pixel_count);
void lcd_copy_raw_rows_to_framebuffer(uint32_t start_row, uint32_t row_count, const uint8_t *raw_rows);
void lcd_display_rows_from_framebuffer(uint32_t start_row, uint32_t row_count);
void lcd_display_raw_rows(uint32_t start_row, uint32_t row_count, const uint8_t *raw_rows);

void run_lcd_load_image(size_t argc, const char *argv[]);
void run_lcd_unload_image(size_t argc, const char *argv[]);
void run_lcd_status(size_t argc, const char *argv[]);
void run_lcd_fps(size_t argc, const char *argv[]);

#ifdef __cplusplus
}
#endif
