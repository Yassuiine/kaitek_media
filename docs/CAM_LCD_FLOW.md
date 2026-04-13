# Camera To LCD Flow

This document summarizes how the vendor `02-CAM` example connects the camera
pipeline to the LCD and which APIs participate in that flow.

## Source Reference

The behavior described here comes from:

- `_cmp_lcd/RP2350-Touch-LCD-2/C/02-CAM/examples/LCD_2in_test.c`
- `_cmp_lcd/RP2350-Touch-LCD-2/C/02-CAM/lib/CAM/cam.c`
- `_cmp_lcd/RP2350-Touch-LCD-2/C/02-CAM/lib/CAM/cam.h`
- `_cmp_lcd/RP2350-Touch-LCD-2/C/02-CAM/lib/LCD/LCD_2in.c`
- `_cmp_lcd/RP2350-Touch-LCD-2/C/02-CAM/lib/GUI/GUI_Paint.c`

## Goal

The `02-CAM` demo does not wire the camera directly to the LCD controller.
Instead it uses a software pipeline:

1. camera captures a frame into RAM
2. software maps that frame into an LCD framebuffer
3. LCD driver pushes that framebuffer over SPI

## Data Path

The frame path is:

`OV5640 -> PIO -> DMA -> cam_ptr -> Paint_DrawImage(...) -> BlackImage -> LCD_2IN_Display(...)`

Key buffers:

- `cam_ptr`
  camera DMA buffer
- `BlackImage`
  LCD framebuffer used by `GUI_Paint`

Both are effectively RGB565 frame buffers for the demo flow.

## Main Demo Sequence

The vendor example does this:

```c
DEV_Module_Init();
DEV_SET_PWM(0);

LCD_2IN_Init(VERTICAL);
LCD_2IN_Clear(WHITE);
DEV_SET_PWM(100);

BlackImage = malloc(LCD_2IN_HEIGHT * LCD_2IN_WIDTH * 2);
Paint_NewImage((UBYTE *)BlackImage, LCD_2IN.WIDTH, LCD_2IN.HEIGHT, 0, WHITE);
Paint_SetScale(65);
Paint_Clear(WHITE);

init_cam();
config_cam_buffer();
start_cam();

while (1) {
    if (buffer_ready) {
        buffer_ready = false;
        Paint_DrawImage(cam_ptr, 0, 0, LCD_2IN.WIDTH, LCD_2IN.HEIGHT);
        LCD_2IN_Display((UBYTE *)BlackImage);
    }
}
```

## Camera-Side APIs Used

### `init_cam()`

Initializes the camera side:

- XCLK generation
- SCCB / I2C configuration
- sensor default setup
- camera frame buffer allocation

### `config_cam_buffer()`

Configures PIO/DMA capture into the camera buffer.

### `start_cam()`

Starts frame capture.

### `buffer_ready`

Global flag set when a full frame has been captured.

### `cam_ptr`

Pointer to the frame data captured by the camera DMA path.

## LCD-Side APIs Used

### `Paint_NewImage(...)`

Initializes a RAM framebuffer object for drawing.

### `Paint_DrawImage(...)`

Copies image data into the GUI framebuffer.

In this demo, it is used to move the latest camera frame into the LCD image buffer.

### `LCD_2IN_Display(...)`

Transfers the framebuffer to the LCD over SPI.

## Why `GUI_Paint` Is Used

The vendor flow uses `GUI_Paint` as a bridge layer.

That gives them:

- a consistent framebuffer object
- optional drawing primitives and text overlays
- a place to combine camera image + UI if desired

For your own project, this means:

- if you want to follow the vendor approach closely, copy `GUI_Paint`
- if you want the leanest possible path, you can write a direct `cam_ptr -> LCD`
  upload function and skip `GUI_Paint`

## Practical Interpretation

There are two integration strategies.

### Strategy A: Vendor-Style

Use:

- `DEV_Config`
- `LCD_2in`
- `GUI_Paint`

Flow:

1. camera captures into `cam_ptr`
2. `Paint_DrawImage(cam_ptr, ...)`
3. `LCD_2IN_Display(...)`

This is the easier path if you want to stay close to the example.

### Strategy B: Direct Camera To LCD

Use:

- `DEV_Config`
- `LCD_2in`

Flow:

1. camera captures into `cam_ptr`
2. custom function pushes `cam_ptr` directly to LCD

This avoids the paint layer, but you will need to handle image layout carefully.

## Recommended Shell Commands

If you expose the camera-to-LCD path through the shell, the most useful commands are:

- `lcd_init [vertical|horizontal]`
- `lcd_bl <0-100>`
- `lcd_clear <hex565>`
- `lcd_cam_show`

Possible next commands:

- `lcd_cam_stream_start`
- `lcd_cam_stream_stop`

For a first implementation, `lcd_cam_show` is enough:

- capture one frame
- push it to the LCD
- return to the shell

## Suggested Shell Flow

Minimal one-shot preview flow:

```text
lcd_init vertical
lcd_bl 100
cam_xclk 24000
cam_defaults
lcd_cam_show
```

## Integration Notes For Your Current Project

When porting this into your shell project:

- keep LCD initialization separate from global board initialization
- avoid calling `stdio_init_all()` from an LCD shell command
- avoid blindly reapplying system clock settings from the vendor `DEV_Module_Init()`
- verify pin conflicts with existing SD and camera wiring before finalizing the LCD pin map

## First Milestone

The safest staged plan is:

1. bring up LCD with `lcd_init`, `lcd_bl`, and `lcd_clear`
2. confirm a solid color screen works
3. add `lcd_cam_show`
4. only then consider continuous camera preview
