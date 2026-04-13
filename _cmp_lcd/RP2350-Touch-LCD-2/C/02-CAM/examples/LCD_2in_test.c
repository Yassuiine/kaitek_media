/*****************************************************************************
* | File      	:   LCD_2in_test.c
* | Author      :   Waveshare team
* | Function    :   2inch LCD test demo
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
# copies of the Software, and to permit persons to  whom the Software is
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
#
******************************************************************************/
#include "LCD_Test.h"
#include "LCD_2in.h"
#include "DEV_Config.h"
#include "cam.h"

int LCD_2in_test(void)
{
    if(DEV_Module_Init()!=0){
        return -1;
    }
    DEV_SET_PWM(0);

    /* LCD Init */
    printf("2inch LCD demo...\r\n");
    LCD_2IN_Init(VERTICAL);
    LCD_2IN_Clear(WHITE);
    DEV_SET_PWM(100);

    UDOUBLE Imagesize = LCD_2IN_HEIGHT*LCD_2IN_WIDTH*2;
    UWORD *BlackImage;
    if((BlackImage = (UWORD *)malloc(Imagesize)) == NULL) {
        printf("Failed to apply for black memory...\r\n");
        exit(0);
    }

    /* Create a new image cache named IMAGE_RGB and fill it with white */
    Paint_NewImage((UBYTE *)BlackImage,LCD_2IN.WIDTH,LCD_2IN.HEIGHT, 0, WHITE);
    Paint_SetScale(65);
    Paint_Clear(WHITE);
    sleep_ms(10);

    init_cam();          // initialize camera
    config_cam_buffer(); // config buffer
    start_cam();         // start streaming

    while(1)
    {
        if (buffer_ready)
        {
            buffer_ready = false;  // Reset the flag
            Paint_DrawImage(cam_ptr, 0, 0, LCD_2IN.WIDTH, LCD_2IN.HEIGHT);
            LCD_2IN_Display((UBYTE *)BlackImage);
        }
    }
}
