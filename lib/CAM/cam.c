/*****************************************************************************
* | File      	:   cam.c
* | Author      :   Waveshare team
* | Function    :   CAM function interface
* | Info        :
*----------------
* |	This version:   V1.0
* | Date        :   2025-03-13
* | Info        :   
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of theex Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "cam.h"

// init PIO
static PIO pio_cam = pio0;

// statemachine's pointer
static uint32_t sm_cam = 0; // CAMERA's state machines

// dma channels
static uint32_t DMA_CAM_RD_CH;
static int cam_program_offset = -1;
static bool cam_dma_claimed = false;
static volatile bool cam_continuous = true;
static bool cam_use_irq = true;
static bool cam_sm_running = false;
static volatile bool cam_seen_dma_busy = false;
static volatile uint32_t cam_capture_fps = 0;
static volatile uint32_t cam_capture_frames_total = 0;
static uint32_t cam_capture_frames_window = 0;
static uint64_t cam_capture_window_start_us = 0;

// private functions and buffers
uint8_t *cam_ptr;          // current DMA capture buffer
uint8_t *cam_ptr1 = NULL;  // alternate buffer for double-buffering (set externally)
uint8_t *cam_display_ptr;  // most recently completed frame — safe to read at any time

// flag
volatile bool buffer_ready = false;

uint32_t cam_width    = 240;
uint32_t cam_height   = 320;
uint32_t cam_ful_size = 240 * 320;  // updated whenever size changes

static void cam_note_captured_frame(void)
{
    uint64_t now_us = time_us_64();
    if (cam_capture_window_start_us == 0) {
        cam_capture_window_start_us = now_us;
    }

    cam_capture_frames_total++;
    cam_capture_frames_window++;

    uint64_t elapsed_us = now_us - cam_capture_window_start_us;
    if (elapsed_us >= 1000000ULL) {
        cam_capture_fps = (uint32_t)((cam_capture_frames_window * 1000000ULL) / elapsed_us);
        cam_capture_frames_window = 0;
        cam_capture_window_start_us = now_us;
    }
}


/********************************************************************************
function:   Camera initialization
parameter:
********************************************************************************/
void init_cam()
{
    // Initialize CAMERA
    set_pwm_freq_kHz(24000, PIN_PWM);
    sleep_ms(50);
    sccb_init(I2C1_SDA, I2C1_SCL); // sda,scl=(gp26,gp27). see 'sccb_if.c' and 'cam.h'
    sleep_ms(50);

    // buffer of camera data is LCD_2IN_WIDTH * LCD_2IN_HEIGHT * 2 bytes (RGB565 = 16 bits = 2 bytes)
    cam_ptr = (uint8_t *)malloc(cam_ful_size * 2);
    if (cam_ptr) {
        memset(cam_ptr, 0, cam_ful_size * 2);
    } else {
        printf("ERROR: camera buffer allocation failed\n");
    }
    cam_display_ptr = cam_ptr;  // single-buffer default: display == capture
}

/********************************************************************************
function:   Configuring DMA
parameter:
********************************************************************************/
void config_cam_buffer()
{
    if (!cam_ptr) {
        printf("ERROR: camera buffer not allocated\n");
        return;
    }

    // In continuous mode cam_handler restarts DMA immediately after each frame.
    // Reconfiguring while it's running would abort the in-progress capture.
    if (cam_dma_claimed && dma_channel_is_busy(DMA_CAM_RD_CH)) {
        return;
    }

    // init DMA
    if (!cam_dma_claimed) {
        DMA_CAM_RD_CH = dma_claim_unused_channel(true);
        cam_dma_claimed = true;
        printf("DMA_CH= %lu\n", (unsigned long)DMA_CAM_RD_CH);
    }
    
    // Disable IRQ
    irq_set_enabled(DMA_IRQ_0, false);

    // Configure DMA Channel 0
    dma_channel_config c0 = get_cam_config(pio_cam, sm_cam, DMA_CAM_RD_CH);
    channel_config_set_transfer_data_size(&c0, DMA_SIZE_16);

    dma_channel_configure(DMA_CAM_RD_CH, &c0,
                          cam_ptr,               // Destination pointer
                          &pio_cam->rxf[sm_cam], // Source pointer
                          cam_ful_size,          // Number of transfers
                          false                  // Don't Start yet
    );

    // IRQ settings
    if (cam_use_irq) {
        dma_hw->ints0 = 1u << DMA_CAM_RD_CH; // Clear any stale pending IRQ for this channel.
        dma_channel_set_irq0_enabled(DMA_CAM_RD_CH, true);
        irq_set_exclusive_handler(DMA_IRQ_0, cam_handler);
        irq_set_enabled(DMA_IRQ_0, true);
    } else {
        dma_channel_set_irq0_enabled(DMA_CAM_RD_CH, false);
    }
    cam_seen_dma_busy = false;
    buffer_ready = false;
    dma_channel_start(DMA_CAM_RD_CH);
    // Mark that DMA was armed so cam_wait_for_frame correctly detects completion
    // even if DMA finishes before the first poll (e.g. VSYNC fires immediately).
    cam_seen_dma_busy = true;
}

/********************************************************************************
function:   Start the camera
parameter:
********************************************************************************/
void start_cam()
{
    if (cam_program_offset < 0) {
        cam_program_offset = (int)pio_add_program(pio_cam, &picampinos_program);
    }

    if (cam_sm_running) {
        // The PIO program loops back to 'wait 1 gpio VSYNC' after every frame.
        // Resetting the SM would throw away that natural sync and force a full
        // VSYNC wait (up to ~33 ms) before the next capture can start.
        // Just re-arm the DMA; the SM will catch the next VSYNC on its own.
        return;
    }

    picampinos_program_init(pio_cam, sm_cam, (uint32_t)cam_program_offset, CAM_BASE_PIN, 11); // VSYNC,HREF,PCLK,D[2:9] : total 11 pins

    // Enable the state machine and clear the FIFO
    pio_sm_set_enabled(pio_cam, sm_cam, false);
    pio_sm_clear_fifos(pio_cam, sm_cam);
    pio_sm_restart(pio_cam, sm_cam);
    pio_sm_set_enabled(pio_cam, sm_cam, true);

    // Setting the X and Y registers
    pio_sm_put_blocking(pio_cam, sm_cam, 0);                  // X=0 : reserved
    pio_sm_put_blocking(pio_cam, sm_cam, (cam_ful_size - 1)); // Y: total words in an image

    cam_sm_running = true;
}

/********************************************************************************
function:   Reading camera data
parameter:
********************************************************************************/
void read_cam_data_blocking(uint8_t *buffer, size_t length)
{
    size_t index = 0;
    while (index < length)
    {
        if (!pio_sm_is_rx_fifo_empty(pio_cam, sm_cam))
        {
            uint16_t dat = pio_sm_get(pio_cam, sm_cam);
            buffer[index++] = (dat >> 8) & 0xFF; 
            buffer[index++] = dat & 0xFF;
        }
        else
        {
            tight_loop_contents(); // wait for data
        }
    }
}

/********************************************************************************
function:   DMA interrupt processing function
parameter:
********************************************************************************/
void cam_handler()
{
    uint32_t triggered_dma = dma_hw->ints0;

    if (triggered_dma & (1u << DMA_CAM_RD_CH))
    {
        dma_hw->ints0 = 1u << DMA_CAM_RD_CH;  // clear interrupt first

        if (cam_continuous && cam_ptr1 != NULL) {
            // Double-buffer ping-pong: the buffer DMA just wrote is now safe to
            // display; swap it to cam_display_ptr and immediately start capturing
            // the next frame into the other buffer so display and capture overlap.
            uint8_t *just_completed = cam_ptr;
            cam_ptr          = cam_ptr1;        // next capture target
            cam_ptr1         = just_completed;  // keep symmetric for next swap
            cam_display_ptr  = just_completed;  // expose completed frame to display
            dma_channel_set_write_addr(DMA_CAM_RD_CH, cam_ptr, true);
        }

        buffer_ready = true;
        cam_note_captured_frame();
    }
}

/********************************************************************************
function:   Release the camera
parameter:
********************************************************************************/
void free_cam()
{
    // Disable IRQ settings
    irq_set_enabled(DMA_IRQ_0, false);
    if (cam_dma_claimed) {
        dma_channel_set_irq0_enabled(DMA_CAM_RD_CH, false);
        dma_channel_abort(DMA_CAM_RD_CH);
        // Ensure abort has propagated before any subsequent re-config/restart.
        // Without this, a fast stop->start sequence can observe the channel as still
        // busy and skip re-arming, which makes one-shot capture appear stuck.
        absolute_time_t dma_abort_deadline = make_timeout_time_ms(5);
        while (dma_channel_is_busy(DMA_CAM_RD_CH) && !time_reached(dma_abort_deadline)) {
            tight_loop_contents();
        }
    }
    pio_sm_set_enabled(pio_cam, sm_cam, false);
    pio_sm_clear_fifos(pio_cam, sm_cam);
    pio_sm_restart(pio_cam, sm_cam);
    cam_sm_running = false;
    cam_seen_dma_busy = false;
    buffer_ready = false;
}

void cam_set_continuous(bool enabled)
{
    cam_continuous = enabled;
}

void cam_set_use_irq(bool enabled)
{
    cam_use_irq = enabled;
}

bool cam_wait_for_frame(uint32_t timeout_ms)
{
    if (!cam_dma_claimed) {
        return false;
    }

    // In IRQ mode, completion is signaled by cam_handler via buffer_ready.
    // In continuous mode the IRQ may restart DMA before this function runs,
    // so buffer_ready is the only stable completion indicator.
    if (buffer_ready) {
        return true;
    }

    // For IRQ-driven capture there is nothing else to poll.
    if (cam_use_irq) {
        return false;
    }

    // Non-IRQ one-shot path: poll DMA busy state.
    // Important guard:
    // If DMA never actually started (or was aborted before starting), busy is false
    // from the very first check. That does NOT mean a frame is complete.
    bool busy = dma_channel_is_busy(DMA_CAM_RD_CH);
    if (busy) {
        cam_seen_dma_busy = true;
    } else if (!cam_seen_dma_busy) {
        // Not busy yet and we have never seen this transfer become busy -> no frame.
        return false;
    }

    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    while (dma_channel_is_busy(DMA_CAM_RD_CH)) {
        if (time_reached(deadline)) {
            return false;
        }
        tight_loop_contents();
    }

    // Non-continuous path: cam_ptr is the single capture buffer.
    cam_seen_dma_busy = false;
    cam_display_ptr = cam_ptr;
    buffer_ready = true;
    cam_note_captured_frame();
    return true;
}

uint32_t cam_get_capture_fps(void)
{
    return cam_capture_fps;
}

uint32_t cam_get_capture_frames_total(void)
{
    return cam_capture_frames_total;
}

bool cam_get_dma_busy(void)
{
    if (!cam_dma_claimed) return false;
    return dma_channel_is_busy(DMA_CAM_RD_CH);
}

/********************************************************************************
function:   Camera dma config
parameter:
********************************************************************************/
dma_channel_config get_cam_config(PIO pio, uint32_t sm, uint32_t dma_chan)
{
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));
    return c;
}

/********************************************************************************
function:   Configure PWM to provide clock for the camera
parameter:
********************************************************************************/
uint32_t set_pwm_freq_kHz(uint32_t freq_khz, uint8_t gpio_num)
{
    if (freq_khz < 6000) freq_khz = 6000;
    if (freq_khz > 27000) freq_khz = 27000;

    uint32_t system_clk_hz = clock_get_hz(clk_sys);
    uint32_t target_hz = freq_khz * 1000u;

    // Keep TOP small to maximize reachable frequency while preserving 50% duty.
    const uint32_t top = 4u;
    const uint32_t denom = (top + 1u) * target_hz;
    uint32_t div16 = (uint32_t)(((uint64_t)system_clk_hz * 16u + (denom / 2u)) / denom);

    // PWM divider limits in 4.4 fixed-point: [1.0, 255 + 15/16]
    if (div16 < 16u) div16 = 16u;
    if (div16 > 4095u) div16 = 4095u;

    uint8_t div_int = (uint8_t)(div16 >> 4);
    uint8_t div_frac = (uint8_t)(div16 & 0x0F);

    gpio_set_function(gpio_num, GPIO_FUNC_PWM);
    uint32_t pwm0_slice_num = pwm_gpio_to_slice_num(gpio_num);

    pwm_config pwm_slice_config = pwm_get_default_config();
    pwm_config_set_wrap(&pwm_slice_config, top);
    pwm_config_set_clkdiv_int_frac(&pwm_slice_config, div_int, div_frac);
    pwm_init(pwm0_slice_num, &pwm_slice_config, true);
    pwm_set_gpio_level(gpio_num, (top + 1u) / 2u); // ~50% duty

    uint32_t actual_hz = (uint32_t)(((uint64_t)system_clk_hz * 16u) / ((top + 1u) * div16));
    return actual_hz / 1000u;
}
