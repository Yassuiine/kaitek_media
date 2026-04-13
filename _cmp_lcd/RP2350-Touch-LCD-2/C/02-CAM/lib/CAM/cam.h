/*****************************************************************************
* | File      	:   cam.h
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
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "pico/binary_info.h"
#include "ov5640.h"
#include "DEV_Config.h"
#include "LCD_2in.h"
#include "picampinos.pio.h"

extern uint8_t *cam_ptr;  
extern uint8_t *cam_ptr1; 
extern volatile bool buffer_ready;

#define USE_100BASE_FX (false)

#define SYS_CLK_IN_KHZ (150000) // 192000 ~ 250000 (if you use sfp, SYS_CLK_KHZ must be just 250000)
#define CAM_BASE_PIN (0)        // GP1  (camera module needs 11pin)
#define PIN_PWM (11)            // GP11 (camera's xclk(24MHz))

// interfaces
// SCCB IF
#define I2C1_SDA (22)
#define I2C1_SCL (23)

// camera buffer size
// 240x320, RGB565 picture needs 240x320x2 bytes of buffers.
#define CAM_FUL_SIZE (LCD_2IN_HEIGHT * LCD_2IN_WIDTH)                 

// high layer APIs
void init_cam();
void config_cam_buffer();
void start_cam();
void free_cam();
void set_pwm_freq_kHz(uint32_t freq_khz, uint8_t gpio_num);
void read_cam_data_blocking(uint8_t *buffer, size_t length);
dma_channel_config get_cam_config(PIO pio, uint32_t sm, uint32_t dma_chan);
void cam_handler();
