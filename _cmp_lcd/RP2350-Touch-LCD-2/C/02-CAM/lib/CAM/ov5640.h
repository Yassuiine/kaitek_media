/*****************************************************************************
* | File      	:   ov5640.h
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
#ifndef _OV5640_H
#define _OV5640_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

/*******************************************************************************
 * OV5640 register definitions.
/*******************************************************************************/
/* Reset for Individual Block */
#define SYSTEM_RESET02  0x3002  // (0: enable block; 1: reset block)
                                // Bit[7]: Reset VFIFO
                                // Bit[5]: Reset format
                                // Bit[4]: Reset JFIFO
                                // Bit[3]: Reset SFIFO
                                // Bit[2]: Reset JPG
                                // Bit[1]: Reset format MUX
                                // Bit[0]: Reset average

#define CLOCK_ENABLE02  0x3006  // (0: disable clock; 1: enable clock)
                                // Bit[7]: Enable PSRAM clock
                                // Bit[6]: Enable FMT clock
                                // Bit[5]: Enable JPEG 2x clock
                                // Bit[3]: Enable JPEG clock
                                // Bit[1]: Enable format MUX clock
                                // Bit[0]: Enable average clock

/* system control registers */
#define SYSTEM_CTROL0   0x3008  // Bit[7]: Software reset 
                                // Bit[6]: Software power down 
                                // Bit[5]: Reserved 
                                // Bit[4]: SRB clock SYNC enable 
                                // Bit[3]: Isolation suspend select 
                                // Bit[2:0]: Not used
                                
#define SC_PLL_CONTRL_0 0x3034  // Bit[6:4]: PLL charge pump control
                                // Bit[3:0]: MIPI bit mode
                                // 0x8: 8-bit mode
                                // 0xA: 10-bit mode
#define SC_PLL_CONTRL_1 0x3035  // Bit[7:4]: System clock divider Slow down all clocks
                                // Bit[3:0]: Scale divider for MIPI MIPI PCLK/SERCLK can be slowed down
#define SC_PLL_CONTRL_2 0x3036  // Bit[7:0]: PLL multiplier (4~252) Can be any integer for 4~127 and only even integer for 128~252
#define SC_PLL_CONTRL_3 0x3037  // Bit[4]: PLL root divider
                                //          0: Bypass
                                //          1: Divided by 2
                                // Bit[3:0]: PLL pre-divider
                                //          1, 2, 3, 4, 6, 8
#define SC_PLL_CONTRL_5 0x3039  // Bit[7]: PLL bypass


#define DRIVE_CAPABILITY 0x302c // Bit[7:6]:
                                //          00: 1x
                                //          01: 2x
                                //          10: 3x
                                //          11: 4x

#define SC_PLLS_CTRL0    0x303a // Bit[7]: PLLS bypass
#define SC_PLLS_CTRL1    0x303b // Bit[4:0]: PLLS multiplier
#define SC_PLLS_CTRL2    0x303c // Bit[6:4]: PLLS charge pump control
                                // Bit[3:0]: PLLS system divider
#define SC_PLLS_CTRL3    0x303d // Bit[5:4]: PLLS pre-divider
                                //          00: 1
                                //          01: 1.5
                                //          10: 2
                                //          11: 3
                                // Bit[2]: PLLS root-divider - 1
                                // Bit[1:0]: PLLS seld5
                                //          00: 1
                                //          01: 1
                                //          10: 2
                                //          11: 2.5

/* Pad Clock Divider for SCCB Clock */
#define SYSTEM_ROOT_DIVIDER 0x3108  // Bit[7:2]: Debug mode Changing this value is not allowed
                                    // Bit[1]: Select system input clock
                                    //         0: From pad clock
                                    //         1: From PLL
                                    // Bit[0]: Debug mode Changing this value is not allowed    

/* PLL Clock Select */
#define SCCB_SYSTEM_CTRL1 0x3103    // Bit[7:6]: Debug mode

/* AEC/AGC control functions */
#define AEC_PK_MANUAL       0x3503  // AEC Manual Mode Control
                                    // Bit[7:6]: Reserved
                                    // Bit[5]: Gain delay option
                                    //         Valid when 0x3503[4]=1’b0
                                    //         0: Delay one frame latch
                                    //         1: One frame latch
                                    // Bit[4:2]: Reserved
                                    // Bit[1]: AGC manual
                                    //         0: Auto enable
                                    //         1: Manual enable
                                    // Bit[0]: AEC manual
                                    //         0: Auto enable
                                    //         1: Manual enable

//gain = {0x350A[1:0], 0x350B[7:0]} / 16


#define X_ADDR_ST_H     0x3800 //Bit[3:0]: X address start[11:8]
#define X_ADDR_ST_L     0x3801 //Bit[7:0]: X address start[7:0]
#define Y_ADDR_ST_H     0x3802 //Bit[2:0]: Y address start[10:8]
#define Y_ADDR_ST_L     0x3803 //Bit[7:0]: Y address start[7:0]
#define X_ADDR_END_H    0x3804 //Bit[3:0]: X address end[11:8]
#define X_ADDR_END_L    0x3805 //Bit[7:0]:
#define Y_ADDR_END_H    0x3806 //Bit[2:0]: Y address end[10:8]
#define Y_ADDR_END_L    0x3807 //Bit[7:0]:
// Size after scaling
#define X_OUTPUT_SIZE_H 0x3808 //Bit[3:0]: DVP output horizontal width[11:8]
#define X_OUTPUT_SIZE_L 0x3809 //Bit[7:0]:
#define Y_OUTPUT_SIZE_H 0x380a //Bit[2:0]: DVP output vertical height[10:8]
#define Y_OUTPUT_SIZE_L 0x380b //Bit[7:0]:
#define X_TOTAL_SIZE_H  0x380c //Bit[3:0]: Total horizontal size[11:8]
#define X_TOTAL_SIZE_L  0x380d //Bit[7:0]:
#define Y_TOTAL_SIZE_H  0x380e //Bit[7:0]: Total vertical size[15:8]
#define Y_TOTAL_SIZE_L  0x380f //Bit[7:0]:
#define X_OFFSET_H      0x3810 //Bit[3:0]: ISP horizontal offset[11:8]
#define X_OFFSET_L      0x3811 //Bit[7:0]:
#define Y_OFFSET_H      0x3812 //Bit[2:0]: ISP vertical offset[10:8]
#define Y_OFFSET_L      0x3813 //Bit[7:0]:
#define X_INCREMENT     0x3814 //Bit[7:4]: Horizontal odd subsample increment
                               //Bit[3:0]: Horizontal even subsample increment
#define Y_INCREMENT     0x3815 //Bit[7:4]: Vertical odd subsample increment
                               //Bit[3:0]: Vertical even subsample increment
// Size before scaling
//#define X_INPUT_SIZE    (X_ADDR_END - X_ADDR_ST + 1 - (2 * X_OFFSET))
//#define Y_INPUT_SIZE    (Y_ADDR_END - Y_ADDR_ST + 1 - (2 * Y_OFFSET))

/* mirror and flip registers */
#define TIMING_TC_REG20 0x3820  // Timing Control Register
                                // Bit[2:1]: Vertical flip enable
                                //         00: Normal
                                //         11: Vertical flip
                                // Bit[0]: Vertical binning enable
#define TIMING_TC_REG21 0x3821  // Timing Control Register
                                // Bit[5]: Compression Enable
                                // Bit[2:1]: Horizontal mirror enable
                                //         00: Normal
                                //         11: Horizontal mirror
                                // Bit[0]: Horizontal binning enable

#define PCLK_RATIO       0x3824 // Bit[4:0]: PCLK ratio manual

/* frame control registers */
#define FRAME_CTRL01    0x4201  // Control Passed Frame Number When both ON and OFF number set to 0x00,frame control is in bypass mode
                                // Bit[7:4]: Not used
                                // Bit[3:0]: Frame ON number
#define FRAME_CTRL02    0x4202  // Control Masked Frame Number When both ON and OFF number set to 0x00,frame control is in bypass mode
                                // Bit[7:4]: Not used
                                // BIT[3:0]: Frame OFF number

/* format control registers */
#define FORMAT_CTRL00   0x4300

#define CLOCK_POL_CONTROL 0x4740// Bit[5]: PCLK polarity 0: active low
                                //          1: active high
                                // Bit[3]: Gate PCLK under VSYNC
                                // Bit[2]: Gate PCLK under HREF
                                // Bit[1]: HREF polarity
                                //          0: active low
                                //          1: active high
                                // Bit[0] VSYNC polarity
                                //          0: active low
                                //          1: active high

#define ISP_CONTROL_01   0x5001 // Bit[5]: Scale enable
                                //          0: Disable
                                //          1: Enable

/* output format control registers */
#define FORMAT_CTRL     0x501F  // Format select
                                // Bit[2:0]:
                                //  000: YUV422
                                //  001: RGB
                                //  010: Dither
                                //  011: RAW after DPC
                                //  101: RAW after CIP

/* ISP top control registers */
#define PRE_ISP_TEST_SETTING_1  0x503D  // Bit[7]: Test enable
                                        //         0: Test disable
                                        //         1: Color bar enable
                                        // Bit[6]: Rolling
                                        // Bit[5]: Transparent
                                        // Bit[4]: Square black and white
                                        // Bit[3:2]: Color bar style
                                        //         00: Standard 8 color bar
                                        //         01: Gradual change at vertical mode 1
                                        //         10: Gradual change at horizontal
                                        //         11: Gradual change at vertical mode 2
                                        // Bit[1:0]: Test select
                                        //         00: Color bar
                                        //         01: Random data
                                        //         10: Square data
                                        //         11: Black image

//exposure = {0x3500[3:0], 0x3501[7:0], 0x3502[7:0]} / 16 × tROW

#define SCALE_CTRL_1     0x5601 // Bit[6:4]: HDIV RW
                                //          DCW scale times
                                //          000: DCW 1 time
                                //          001: DCW 2 times
                                //          010: DCW 4 times
                                //          100: DCW 8 times
                                //          101: DCW 16 times
                                //          Others: DCW 16 times
                                // Bit[2:0]: VDIV RW
                                //          DCW scale times
                                //          000: DCW 1 time
                                //          001: DCW 2 times
                                //          010: DCW 4 times
                                //          100: DCW 8 times
                                //          101: DCW 16 times
                                //          Others: DCW 16 times

#define SCALE_CTRL_2     0x5602 // X_SCALE High Bits
#define SCALE_CTRL_3     0x5603 // X_SCALE Low Bits
#define SCALE_CTRL_4     0x5604 // Y_SCALE High Bits
#define SCALE_CTRL_5     0x5605 // Y_SCALE Low Bits
#define SCALE_CTRL_6     0x5606 // Bit[3:0]: V Offset

#define VFIFO_CTRL0C     0x460C // Bit[1]: PCLK manual enable
                                //          0: Auto
                                //          1: Manual by PCLK_RATIO

#define VFIFO_X_SIZE_H   0x4602
#define VFIFO_X_SIZE_L   0x4603
#define VFIFO_Y_SIZE_H   0x4604
#define VFIFO_Y_SIZE_L   0x4605

#define COMPRESSION_CTRL00 0x4400 //
#define COMPRESSION_CTRL01 0x4401 //
#define COMPRESSION_CTRL02 0x4402 //
#define COMPRESSION_CTRL03 0x4403 //
#define COMPRESSION_CTRL04 0x4404 //
#define COMPRESSION_CTRL05 0x4405 //
#define COMPRESSION_CTRL06 0x4406 //
#define COMPRESSION_CTRL07 0x4407 // Bit[5:0]: QS
#define COMPRESSION_ISI_CTRL 0x4408 //
#define COMPRESSION_CTRL09 0x4409 //
#define COMPRESSION_CTRL0a 0x440a //
#define COMPRESSION_CTRL0b 0x440b //
#define COMPRESSION_CTRL0c 0x440c //
#define COMPRESSION_CTRL0d 0x440d //
#define COMPRESSION_CTRL0E 0x440e //

/**
 * @brief register value
 */
#define TEST_COLOR_BAR  0xC0    /* Enable Color Bar roling Test */

#define AEC_PK_MANUAL_AGC_MANUALEN  0x02    /* Enable AGC Manual enable */
#define AEC_PK_MANUAL_AEC_MANUALEN  0x01    /* Enable AEC Manual enable */

#define TIMING_TC_REG20_VFLIP   0x06 /* Vertical flip enable */
#define TIMING_TC_REG21_HMIRROR 0x06 /* Horizontal mirror enable */

/*******************************************************************************
 * Function Declarations
/*******************************************************************************/

void sccb_init(const uint32_t sda_pin,
               const uint32_t scl_pin);

int32_t reg_write(i2c_inst_t *i2c,
                  const uint8_t addr,
                  uint8_t *buf,
                  const uint8_t nbytes);

int32_t reg_read(i2c_inst_t *i2c,
                 const uint8_t addr,
                 uint8_t *buf,
                 const uint8_t nbytes);

uint8_t OV5640_WR_Reg(i2c_inst_t *i2c,
                  const uint8_t addr,
                  uint16_t reg,
                  uint8_t data);

uint8_t OV5640_WR_Reg_2(i2c_inst_t *i2c,
                  const uint8_t addr,
                  uint16_t reg,
                  uint16_t data1,
                  uint16_t data2);

uint8_t OV5640_RD_Reg(i2c_inst_t *i2c,
                  const uint8_t addr,
                  uint16_t reg);

extern const uint16_t sensor_default_regs[][2];

#endif // OV5640_H

