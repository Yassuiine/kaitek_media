/*****************************************************************************
* | File      	:   ov5640.c
* | Author      :   Waveshare team
* | Function    :   OV5640 configuration function interface
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
#include "pico/stdlib.h"
#include "ov5640.h"

// Define the default register settings for the OV5640 sensor
const uint16_t sensor_default_regs[][2] = {
    {SYSTEM_CTROL0, 0x82},  // software reset
    {0XFFFF, 10}, // delay 10ms
    {SYSTEM_CTROL0, 0x42},  // power down

    //enable pll
    {0x3103, 0x13},

    //io direction
    {0x3017, 0xff},
    {0x3018, 0xff},

    {DRIVE_CAPABILITY, 0xc3},
    {CLOCK_POL_CONTROL, 0x21},

    {0x4713, 0x02},//jpg mode select

    {ISP_CONTROL_01, 0x83}, // turn color matrix, awb and SDE

    //sys reset
    {0x3000, 0x20}, // reset MCU
    {0XFFFF, 10}, // delay 10ms
    {0x3002, 0x1c},

    //clock enable
    {0x3004, 0xff},
    {0x3006, 0xc3},

    //isp control
    {0x5000, 0xa7},
    {ISP_CONTROL_01, 0xa3},//+scaling?
    {0x5003, 0x08},//special_effect

    //unknown
    {0x370c, 0x02},//!!IMPORTANT
    {0x3634, 0x40},//!!IMPORTANT

    //AEC/AGC
    {0x3a02, 0x03},
    {0x3a03, 0xd8},
    {0x3a08, 0x01},
    {0x3a09, 0x27},
    {0x3a0a, 0x00},
    {0x3a0b, 0xf6},
    {0x3a0d, 0x04},
    {0x3a0e, 0x03},
    {0x3a0f, 0x30},//ae_level
    {0x3a10, 0x28},//ae_level
    {0x3a11, 0x60},//ae_level
    {0x3a13, 0x43},
    {0x3a14, 0x03},
    {0x3a15, 0xd8},
    {0x3a18, 0x00},//gainceiling
    {0x3a19, 0xf8},//gainceiling
    {0x3a1b, 0x30},//ae_level
    {0x3a1e, 0x26},//ae_level
    {0x3a1f, 0x14},//ae_level

    //vcm debug
    {0x3600, 0x08},
    {0x3601, 0x33},

    //50/60Hz
    {0x3c01, 0xa4},
    {0x3c04, 0x28},
    {0x3c05, 0x98},
    {0x3c06, 0x00},
    {0x3c07, 0x08},
    {0x3c08, 0x00},
    {0x3c09, 0x1c},
    {0x3c0a, 0x9c},
    {0x3c0b, 0x40},

    {0x460c, 0x22},//disable jpeg footer

    //BLC
    {0x4001, 0x02},
    {0x4004, 0x02},

    //AWB
    {0x5180, 0xff},
    {0x5181, 0xf2},
    {0x5182, 0x00},
    {0x5183, 0x14},
    {0x5184, 0x25},
    {0x5185, 0x24},
    {0x5186, 0x09},
    {0x5187, 0x09},
    {0x5188, 0x09},
    {0x5189, 0x75},
    {0x518a, 0x54},
    {0x518b, 0xe0},
    {0x518c, 0xb2},
    {0x518d, 0x42},
    {0x518e, 0x3d},
    {0x518f, 0x56},
    {0x5190, 0x46},
    {0x5191, 0xf8},
    {0x5192, 0x04},
    {0x5193, 0x70},
    {0x5194, 0xf0},
    {0x5195, 0xf0},
    {0x5196, 0x03},
    {0x5197, 0x01},
    {0x5198, 0x04},
    {0x5199, 0x12},
    {0x519a, 0x04},
    {0x519b, 0x00},
    {0x519c, 0x06},
    {0x519d, 0x82},
    {0x519e, 0x38},

    //color matrix (Saturation)
    {0x5381, 0x1e},
    {0x5382, 0x5b},
    {0x5383, 0x08},
    {0x5384, 0x0a},
    {0x5385, 0x7e},
    {0x5386, 0x88},
    {0x5387, 0x7c},
    {0x5388, 0x6c},
    {0x5389, 0x10},
    {0x538a, 0x01},
    {0x538b, 0x98},

    //CIP control (Sharpness)
    {0x5300, 0x10},//sharpness
    {0x5301, 0x10},//sharpness
    {0x5302, 0x18},//sharpness
    {0x5303, 0x19},//sharpness
    {0x5304, 0x10},
    {0x5305, 0x10},
    {0x5306, 0x08},//denoise
    {0x5307, 0x16},
    {0x5308, 0x40},
    {0x5309, 0x10},//sharpness
    {0x530a, 0x10},//sharpness
    {0x530b, 0x04},//sharpness
    {0x530c, 0x06},//sharpness

    //GAMMA
    {0x5480, 0x01},
    {0x5481, 0x00},
    {0x5482, 0x1e},
    {0x5483, 0x3b},
    {0x5484, 0x58},
    {0x5485, 0x66},
    {0x5486, 0x71},
    {0x5487, 0x7d},
    {0x5488, 0x83},
    {0x5489, 0x8f},
    {0x548a, 0x98},
    {0x548b, 0xa6},
    {0x548c, 0xb8},
    {0x548d, 0xca},
    {0x548e, 0xd7},
    {0x548f, 0xe3},
    {0x5490, 0x1d},

    //Special Digital Effects (SDE) (UV adjust)
    {0x5580, 0x04},//enable brightness and contrast
    {0x5583, 0x40},//special_effect
    {0x5584, 0x10},//special_effect
    {0x5586, 0x20},//contrast
    {0x5587, 0x00},//brightness
    {0x5588, 0x01},//brightness
    {0x5589, 0x10},
    {0x558a, 0x00},
    {0x558b, 0xf8},
    {0x501d, 0x40},// enable manual offset of contrast

    //power on
    {0x3008, 0x02},

    //50Hz
    {0x3c00, 0x04},
    
    {0XFFFF, 300},
    {0x0000, 0x00}, // tail
};

/********************************************************************************
 * Function Definitions
 */
/********************************************************************************
function:   Camera register initialization
parameter:
********************************************************************************/
void sccb_init(const uint32_t sda_pin, const uint32_t scl_pin)
{
    uint8_t CAM_ADDR = 0x3c; // default: OV5640

    i2c_inst_t * i2c = i2c1;

    // Initialize I2C port at 100 kHz
    i2c_init(i2c, 100 * 1000);

    // Initialize I2C pins
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);

    uint16_t reg;
    reg=OV5640_RD_Reg(i2c,CAM_ADDR,0X300A);
	reg<<=8;
	reg|=OV5640_RD_Reg(i2c,CAM_ADDR,0X300B);
    printf("ID: %d \r\n",reg);

    for(uint16_t i=0; i<sizeof(sensor_default_regs)/4; i++)
	{
		OV5640_WR_Reg(i2c,CAM_ADDR,sensor_default_regs[i][0],sensor_default_regs[i][1]);
	} 
    sleep_ms(50);

    // set_size_and_colorspace 320x240
    OV5640_WR_Reg_2(i2c,CAM_ADDR,X_ADDR_ST_H,352,26);
    OV5640_WR_Reg_2(i2c,CAM_ADDR,X_ADDR_END_H,1792,1946);
    OV5640_WR_Reg_2(i2c,CAM_ADDR,X_OUTPUT_SIZE_H,240,320); //1440 1920
    OV5640_WR_Reg_2(i2c,CAM_ADDR,X_TOTAL_SIZE_H,2592,1944);
    OV5640_WR_Reg_2(i2c,CAM_ADDR,X_OFFSET_H,16,14);

    uint8_t dat;
    dat = OV5640_RD_Reg(i2c,CAM_ADDR,0x5001) | 0x20;
    OV5640_WR_Reg(i2c,CAM_ADDR,0x5001, dat);
    sleep_ms(50);

    // set_image_options
    OV5640_WR_Reg(i2c,CAM_ADDR,TIMING_TC_REG20, 0x01);
    OV5640_WR_Reg(i2c,CAM_ADDR,TIMING_TC_REG21, 0X00);
    OV5640_WR_Reg(i2c,CAM_ADDR,0x4514, 0xAA);
    OV5640_WR_Reg(i2c,CAM_ADDR,0x4520, 0x0B);
    OV5640_WR_Reg(i2c,CAM_ADDR,X_INCREMENT, 0x31);
    OV5640_WR_Reg(i2c,CAM_ADDR,Y_INCREMENT, 0x31);
    sleep_ms(50);

    // set_pll
    OV5640_WR_Reg(i2c,CAM_ADDR,SC_PLL_CONTRL_5, 0x00);
    OV5640_WR_Reg(i2c,CAM_ADDR,SC_PLL_CONTRL_0, 0x1A); 
    OV5640_WR_Reg(i2c,CAM_ADDR,SC_PLL_CONTRL_1, 0x11); 
    OV5640_WR_Reg(i2c,CAM_ADDR,SC_PLL_CONTRL_2, 11 & 0xFF); 
    OV5640_WR_Reg(i2c,CAM_ADDR,SC_PLL_CONTRL_3, 0x01); 
    OV5640_WR_Reg(i2c,CAM_ADDR,SYSTEM_ROOT_DIVIDER, 0x16); 
    OV5640_WR_Reg(i2c,CAM_ADDR,PCLK_RATIO, 0x04); 
    OV5640_WR_Reg(i2c,CAM_ADDR,VFIFO_CTRL0C, 0x22);
    OV5640_WR_Reg(i2c,CAM_ADDR,SCCB_SYSTEM_CTRL1, 0x13);
    sleep_ms(50);

    // set_colorspace
    OV5640_WR_Reg(i2c,CAM_ADDR,FORMAT_CTRL,0x01);
    OV5640_WR_Reg(i2c,CAM_ADDR,FORMAT_CTRL00,0x61);
    sleep_ms(50);
}

/********************************************************************************
function:   Read 1 byte from the specified register
parameter:
********************************************************************************/
uint8_t OV5640_RD_Reg(i2c_inst_t *i2c,
                  const uint8_t addr,
                  uint16_t reg)
{
    uint8_t val = 0;
    uint8_t reg_high = reg >> 8;
    uint8_t reg_low = reg & 0xFF;

    uint8_t reg_data[2] = {reg_high, reg_low};
    i2c_write_blocking(i2c, addr, reg_data, 2, true);
    i2c_read_blocking(i2c, addr, &val, 1, false);  
    return val;
}

/********************************************************************************
function:   Write 1 byte to the specified register
parameter:
********************************************************************************/
uint8_t OV5640_WR_Reg(i2c_inst_t *i2c,
                  const uint8_t addr,
                  uint16_t reg,
                  uint8_t data)
{
    uint8_t msg[3];
    msg[0] = reg >> 8;
    msg[1] = reg & 0xFF;
    msg[2] = data;

    uint8_t ret=0;
    ret=i2c_write_blocking(i2c, addr, msg, 3, false);
    return ret;
}

/********************************************************************************
function:   Write 4 byte to the specified register
parameter:
********************************************************************************/
uint8_t OV5640_WR_Reg_2(i2c_inst_t *i2c,
                  const uint8_t addr,
                  uint16_t reg,
                  uint16_t data1,
                  uint16_t data2)
{
    uint8_t msg[3];
    msg[0] = reg >> 8;
    msg[1] = reg & 0xFF;
    msg[2] = data1 >> 8;
    uint8_t ret=0;
    ret=i2c_write_blocking(i2c, addr, msg, 3, false);

    reg+=1;
    msg[0] = reg >> 8;
    msg[1] = reg & 0xFF;
    msg[2] = data1 & 0xFF;
    ret=i2c_write_blocking(i2c, addr, msg, 3, false);

    reg+=1;
    msg[0] = reg >> 8;
    msg[1] = reg & 0xFF;
    msg[2] = data2 >> 8;
    ret=i2c_write_blocking(i2c, addr, msg, 3, false);

    reg+=1;
    msg[0] = reg >> 8;
    msg[1] = reg & 0xFF;
    msg[2] = data2 & 0xFF;
    ret=i2c_write_blocking(i2c, addr, msg, 3, false);
    return ret;
}
