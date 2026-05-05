#include "cmd_lcd.h"
#include "command.h"
#include "cmd_nrf.h"
//
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
//
#include "pico/stdlib.h"
#include "hardware/dma.h"
//
#include "cam.h"
#include "DEV_Config.h"
#include "LCD_2in.h"
#include "GUI_Paint.h"
#include "f_util.h"

// Global variable definitions
bool lcd_initialized = false;               // True after lcd_init succeeds; guards all LCD command handlers
bool lcd_con_active = false;                // True while the LCD console output driver is registered
uint8_t lcd_con_ansi_state = 0;             // 0=normal, 1=seen ESC, 2=in CSI sequence (strip until final letter)
int  lcd_con_x = 0;                        // Cursor column in pixels
int  lcd_con_y = 0;                        // Cursor row in pixels
uint16_t lcd_con_fg = 0xFFFF;              // Console foreground colour (default white)
uint16_t lcd_con_bg = 0x0000;              // Console background colour (default black)
const sFONT *lcd_con_font = &Font8;        // Console font (Font8 = 8×8 px → 40 cols × 60 rows at 320×480)
int lcd_con_width  = 0;                    // Panel pixel width cached at lcdcon start
int lcd_con_height = 0;                    // Panel pixel height cached at lcdcon start
uint8_t lcd_backlight = 0;                 // Current PWM backlight level in percent (0 = off, 100 = full)
UBYTE lcd_scan_dir = VERTICAL;             // Current panel scan direction: VERTICAL (portrait) or HORIZONTAL (landscape)
UWORD *lcd_image = NULL;                   // Heap-allocated framebuffer (WIDTH×HEIGHT RGB565 words); NULL before lcd_init
size_t lcd_image_bytes = 0;                // Size of lcd_image in bytes (= WIDTH × HEIGHT × 2)
const uint32_t lcd_spi_hz = 150000000;      // Requested SPI baud for the LCD; actual rate set by spi_init()
uint32_t lcd_display_fps = 0;              // Rolling one-second display frame rate (frames per second)
uint32_t lcd_display_frames_total = 0;     // Total frames pushed to the LCD panel since last lcd_init
uint32_t lcd_display_frames_window = 0;    // Frame count in the current FPS measurement window
uint64_t lcd_display_window_start_us = 0;  // Timestamp (us) when the current FPS window started
lcd_load_state_t lcd_load_state = LCD_LOAD_IDLE; // Current phase of the non-blocking LCD load/display state machine
bool lcd_load_pending = false;             // True while any lcd_load state machine phase is active
FIL lcd_load_file;                         // FatFS file object for the image being loaded from SD
bool lcd_load_file_open = false;           // True when lcd_load_file is open and must be closed on finish/abort
char lcd_load_path[256];                   // File path of the image currently being loaded
char lcd_load_status_msg[128];             // Optional status message printed on load completion (empty = default msg)
UINT lcd_load_offset = 0;                  // Total bytes read from the file so far (tracks progress)
uint32_t lcd_load_display_row = 0;         // Next row to send to the LCD panel (advances in lcd_display_row_chunk steps)
absolute_time_t lcd_load_next_step_time;   // Earliest time the lcd_load state machine should execute its next step
const uint32_t lcd_display_row_chunk = 16; // Number of rows per background-task tick for lcd_load (SD read + LCD push)
uint8_t lcd_row_buffer[LCD_2IN_WIDTH * 2 * 16];  // Scratch buffer for one chunk of rows read from SD (lcd_display_row_chunk=16)

/**
 * @brief Initialise the LCD panel: align clk_peri, start SPI at 62.5 MHz, run the panel init sequence,
 *        allocate the framebuffer, clear to white, attempt touch controller init, and set up NRF poll timer.
 */
void run_lcd_init(const size_t argc, const char *argv[]) {
    if (argc > 1) {
        printf("Usage: lcd_init [vertical|horizontal]\n");
        return;
    }

    if (nrf_fft_stream_is_active()) {
        nrf_fft_stop_public(true, false);
        printf("Stopped nrf stream before LCD reinitialization.\n");
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
void run_lcd_bl(const size_t argc, const char *argv[]){
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
void run_lcd_clear(const size_t argc , const char *argv[]){
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
void run_lcd_pixel(const size_t argc, const char *argv[]){
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
void run_lcd_fillrect(const size_t argc, const char *argv[]){
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

void lcd_con_draw_char(char c, int x, int y) {
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

void lcd_con_scroll(void) {
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    LCD_2IN_Clear(lcd_con_bg);
    lcd_con_x = 0;
    lcd_con_y = 0;
}

void lcd_con_putchar(char c) {
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

void lcd_con_stop(void) {
    if (!lcd_con_active) return;
    lcd_con_active = false;
}

void lcd_con_start(uint16_t fg, uint16_t bg, const sFONT *font) {
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

void run_lcdcon(const size_t argc, const char *argv[]) {
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
void run_lcd_text(const size_t argc, const char *argv[]) {
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
void run_lcd_set_orientation(const size_t argc, const char *argv[]) {
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
void finish_lcd_load(void) {
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
void fail_lcd_load(const char *msg) {
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
bool start_lcd_framebuffer_display(const char *status_msg) {
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
void lcd_note_displayed_frame(void) {
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
void lcd_display_rows_from_framebuffer(uint32_t start_row, uint32_t row_count) {
    LCD_2IN_SetWindows(0, (UWORD)start_row, LCD_2IN.WIDTH, (UWORD)(start_row + row_count));
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
void lcd_convert_raw_rgb565_to_panel_bytes(const uint8_t *src, uint8_t *dst, uint32_t pixel_count) {
    for (uint32_t i = 0; i < pixel_count; ++i) {
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
void lcd_display_raw_rows(uint32_t start_row, uint32_t row_count, const uint8_t *raw_rows) {
    (void)row_count;
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
    // MSB-first → HIGH byte then LOW byte on the wire: correct RGB565, no CPU swap needed.
    spi_set_format(SPI_PORT, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    // TX DMA: raw_rows → SPI TX FIFO (full frame, 16-bit words, DMA-paced)
    dma_channel_config tx_cfg = dma_channel_get_default_config((uint)spi_tx_dma_ch);
    channel_config_set_transfer_data_size(&tx_cfg, DMA_SIZE_16);
    channel_config_set_dreq(&tx_cfg, spi_get_dreq(SPI_PORT, true));
    channel_config_set_read_increment(&tx_cfg, true);
    channel_config_set_write_increment(&tx_cfg, false);
    dma_channel_configure((uint)spi_tx_dma_ch, &tx_cfg,
                          &spi_get_hw(SPI_PORT)->dr, raw_rows, cam_ful_size, false);

    // RX DMA: drain SPI RX FIFO → spi_rx_dummy.
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
void lcd_copy_raw_rows_to_framebuffer(uint32_t start_row, uint32_t row_count, const uint8_t *raw_rows) {
    for (uint32_t row = 0; row < row_count; ++row) {
        uint8_t *dst = (uint8_t *)lcd_image + ((start_row + row) * LCD_2IN_WIDTH * 2);
        const uint8_t *src = raw_rows + (row * LCD_2IN_WIDTH * 2);
        lcd_convert_raw_rgb565_to_panel_bytes(src, dst, LCD_2IN_WIDTH);
    }
}

/**
 * @brief Begin a non-blocking load of a raw 240×320 RGB565 file from SD into the LCD framebuffer and panel.
 */
void run_lcd_load_image(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;
    if (nrf_fft_stream_is_active()) {
        printf("Stop nrf stream before lcd load.\n");
        return;
    }
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
void run_lcd_unload_image(const size_t argc, const char *argv[]) {
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
 * @brief Print LCD init state, orientation, backlight level, and logical pixel dimensions.
 */
void run_lcd_status(const size_t argc, const char *argv[]){
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
void run_lcd_fps(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;

    printf("Capture FPS: %lu\n", (unsigned long)cam_get_capture_fps());
    printf("Display FPS: %lu\n", (unsigned long)lcd_display_fps);
    printf("Captured frames total: %lu\n", (unsigned long)cam_get_capture_frames_total());
    printf("Displayed frames total: %lu\n", (unsigned long)lcd_display_frames_total);
    printf("LCD stream active: %s\n", lcd_cam_stream_active ? "yes" : "no");
}
