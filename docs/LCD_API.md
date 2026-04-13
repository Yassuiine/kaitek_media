# LCD API

This document summarizes the LCD stack used by the vendor `01-LCD` example in the
`RP2350-Touch-LCD-2` package and explains the role of each API layer.

## Source Reference

The LCD implementation described here comes from:

- `_cmp_lcd/RP2350-Touch-LCD-2/C/01-LCD/lib/Config/DEV_Config.c`
- `_cmp_lcd/RP2350-Touch-LCD-2/C/01-LCD/lib/Config/DEV_Config.h`
- `_cmp_lcd/RP2350-Touch-LCD-2/C/01-LCD/lib/LCD/LCD_2in.c`
- `_cmp_lcd/RP2350-Touch-LCD-2/C/01-LCD/lib/LCD/LCD_2in.h`
- `_cmp_lcd/RP2350-Touch-LCD-2/C/01-LCD/examples/LCD_2in_test.c`

## Architecture

The LCD stack has two core layers:

1. `DEV_Config`
   Board support layer for GPIO, SPI, PWM, I2C, delays, and reset/backlight pins.
2. `LCD_2in`
   LCD panel driver that sends commands and RGB565 pixel data over SPI.

Optional higher layers:

- `GUI_Paint`
  RAM framebuffer drawing helpers.
- `Fonts`
  Text rendering data.

## Hardware Mapping

From `DEV_Config.h`:

- `SPI_PORT = spi0`
- `LCD_RST_PIN = 20`
- `LCD_DC_PIN = 16`
- `LCD_BL_PIN = 15`
- `LCD_CS_PIN = 17`
- `LCD_CLK_PIN = 18`
- `LCD_MOSI_PIN = 19`
- `I2C_PORT = i2c0`
- `DEV_SDA_PIN = 12`
- `DEV_SCL_PIN = 13`

The LCD is write-only in this driver:

- control lines: `RST`, `DC`, `CS`
- pixel transport: `SPI0 CLK + MOSI`
- backlight: PWM on `LCD_BL_PIN`

## Data Format

The panel is configured for:

- resolution: `240 x 320`
- pixel format: `RGB565`

This is programmed in `LCD_2IN_InitReg()`:

- command `0x3A`
- data `0x05`

## Initialization Flow

The vendor example uses this order:

1. `DEV_Module_Init()`
2. `LCD_2IN_Init(HORIZONTAL or VERTICAL)`
3. `LCD_2IN_Clear(WHITE)`
4. `DEV_SET_PWM(100)`

### `DEV_Module_Init()`

This function prepares the board side:

- raises core voltage
- sets system clock to `150 MHz`
- calls `stdio_init_all()`
- initializes LCD GPIOs
- initializes ADC
- initializes SPI0
- initializes PWM for backlight
- initializes I2C0

Important for your shell project:

- this function does more than LCD-only setup
- do not reuse it blindly inside a shell command
- split it into LCD-only helpers if needed

## Core LCD APIs

### `void LCD_2IN_Init(UBYTE Scan_dir)`

Initializes the LCD panel.

Internally it does:

1. hardware reset
2. scan-direction / orientation setup
3. LCD register initialization sequence

`Scan_dir`:

- `HORIZONTAL`
- `VERTICAL`

### `void LCD_2IN_Clear(UWORD Color)`

Fills the entire display with a single RGB565 color.

Behavior:

- prepares a one-line color buffer
- sets the full LCD window
- streams repeated line data over SPI

### `void LCD_2IN_Display(UWORD *Image)`

Pushes a full framebuffer to the LCD.

Expected buffer:

- size: `LCD_2IN_WIDTH * LCD_2IN_HEIGHT * 2` bytes
- format: RGB565

Behavior:

- sets full-screen window
- writes the image row-by-row over SPI

### `void LCD_2IN_SetWindows(UWORD Xstart, UWORD Ystart, UWORD Xend, UWORD Yend)`

Sets the active draw window using:

- `0x2A` column address set
- `0x2B` row address set
- `0x2C` memory write

### `void LCD_2IN_DisplayWindows(UWORD Xstart, UWORD Ystart, UWORD Xend, UWORD Yend, UWORD *Image)`

Updates only a rectangular region of the framebuffer.

Useful for:

- partial redraws
- touch markers
- small HUD updates

### `void LCD_2IN_DisplayPoint(UWORD X, UWORD Y, UWORD Color)`

Writes a single RGB565 pixel.

### `void DEV_SET_PWM(uint8_t Value)`

Sets backlight PWM duty cycle.

Expected range:

- `0..100`

## Orientation Handling

Orientation is selected in `LCD_2IN_SetAttributes()`.

- `VERTICAL`
  - `WIDTH = 240`
  - `HEIGHT = 320`
  - memory access register set to `0x08`
- `HORIZONTAL`
  - dimensions are swapped for drawing
  - memory access register changes accordingly

For camera preview, `VERTICAL` is the safest starting point because the vendor
`02-CAM` example uses `LCD_2IN_Init(VERTICAL)`.

## Typical Bring-Up Example

```c
DEV_Module_Init();
LCD_2IN_Init(VERTICAL);
LCD_2IN_Clear(WHITE);
DEV_SET_PWM(100);
```

## Minimal Shell-Level Commands

If you expose the LCD through your shell, the most atomic commands are:

- `lcd_init [vertical|horizontal]`
- `lcd_bl <0-100>`
- `lcd_clear <hex565>`
- `lcd_status`

Useful second-wave commands:

- `lcd_pixel <x> <y> <hex565>`
- `lcd_fillrect <x> <y> <w> <h> <hex565>`
- `lcd_cam_show`

## Integration Notes For `03-FatFs`

Before copying the LCD code directly, check pin conflicts with your existing project.

In your current tree:

- camera uses `GP0..GP10`, `GP11`, `GP22`, `GP23`
- SD and SDIO already use several pins in `config/hw_config.c`

The vendor LCD example also uses:

- `GP17`, `GP18`, `GP19`, `GP20`
- `GP12`, `GP13`

So the code structure can be reused directly, but the pin map likely needs to be
adapted for your final combined build.
