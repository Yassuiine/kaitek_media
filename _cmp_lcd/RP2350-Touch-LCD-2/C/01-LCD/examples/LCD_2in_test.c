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
#include "QMI8658.h"
#include "CST816D.h"

uint8_t flag = 0;
uint8_t i2c_lock = 0;
#define I2C_LOCK() i2c_lock = 1
#define I2C_UNLOCK() i2c_lock = 0

void Touch_INT_callback(uint gpio, uint32_t events);

int LCD_2in_test(void)
{
    if(DEV_Module_Init()!=0){
        return -1;
    }

    /* LCD Init */
    printf("2inch LCD demo...\r\n");
    LCD_2IN_Init(HORIZONTAL);
    LCD_2IN_Clear(WHITE);
    DEV_SET_PWM(100);

    UDOUBLE Imagesize = LCD_2IN_HEIGHT*LCD_2IN_WIDTH*2;
    UWORD *BlackImage;
    if((BlackImage = (UWORD *)malloc(Imagesize)) == NULL) {
        printf("Failed to apply for black memory...\r\n");
        exit(0);
    }

    /*1.Create a new image cache named IMAGE_RGB and fill it with white*/
    Paint_NewImage((UBYTE *)BlackImage, LCD_2IN.WIDTH, LCD_2IN.HEIGHT, 0, WHITE);
    Paint_SetScale(65);
    Paint_Clear(WHITE);
    Paint_SetRotate(ROTATE_0);
    sleep_ms(10);

    /* GUI */
    printf("drawing...\r\n");
    /*2.Drawing on the image*/
#if 1
    Paint_DrawPoint(2,1, BLACK, DOT_PIXEL_1X1,  DOT_FILL_RIGHTUP);
    Paint_DrawPoint(2,6, BLACK, DOT_PIXEL_2X2,  DOT_FILL_RIGHTUP);
    Paint_DrawPoint(2,11, BLACK, DOT_PIXEL_3X3, DOT_FILL_RIGHTUP);
    Paint_DrawPoint(2,16, BLACK, DOT_PIXEL_4X4, DOT_FILL_RIGHTUP);
    Paint_DrawPoint(2,21, BLACK, DOT_PIXEL_5X5, DOT_FILL_RIGHTUP);
    Paint_DrawLine( 10,  5, 40, 35, MAGENTA, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    Paint_DrawLine( 10, 35, 40,  5, MAGENTA, DOT_PIXEL_2X2, LINE_STYLE_SOLID);

    Paint_DrawLine( 80,  20, 110, 20, CYAN, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
    Paint_DrawLine( 95,   5,  95, 35, CYAN, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);

    Paint_DrawRectangle(10, 5, 40, 35, RED, DOT_PIXEL_2X2,DRAW_FILL_EMPTY);
    Paint_DrawRectangle(45, 5, 75, 35, BLUE, DOT_PIXEL_2X2,DRAW_FILL_FULL);

    Paint_DrawCircle(95, 20, 15, GREEN, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    Paint_DrawCircle(130, 20, 15, GREEN, DOT_PIXEL_1X1, DRAW_FILL_FULL);

    Paint_DrawNum (50, 40 , 9.87654321, &Font20, 5, WHITE, BLACK);
    Paint_DrawString_EN(1, 40, "ABC", &Font20, 0x000f, 0xfff0);
    Paint_DrawString_CN(1,60, "欢迎使用",  &Font24CN, WHITE, BLUE);
    Paint_DrawString_EN(1, 100, "WaveShare", &Font16, RED, WHITE); 

    /*3.Refresh the picture in RAM to LCD*/
    LCD_2IN_Display(BlackImage);
    DEV_Delay_ms(1000);
#endif

#if 1
    Paint_DrawImage(gImage_2inch_1,0,0,LCD_2IN.WIDTH,LCD_2IN.HEIGHT);
    LCD_2IN_Display(BlackImage);
    DEV_Delay_ms(1000);
#endif
    
#if 1
    float acc[3], gyro[3];
    unsigned int tim_count = 0;
    const float conversion_factor = 3.3f / (1 << 12) * 3;
    QMI8658_init();
    CST816D_init(CST816D_Gesture_Mode);
    DEV_KEY_Config(Touch_INT_PIN);
    DEV_IRQ_SET(Touch_INT_PIN, GPIO_IRQ_EDGE_RISE, &Touch_INT_callback);
    
    Paint_Clear(WHITE);
    Paint_DrawRectangle(0, 00, 320, 49, 0XF410, DOT_PIXEL_2X2, DRAW_FILL_FULL);
    Paint_DrawRectangle(0, 49, 320, 120, 0X4F30, DOT_PIXEL_2X2, DRAW_FILL_FULL);
    Paint_DrawRectangle(0, 120, 320, 195, 0XAD55, DOT_PIXEL_2X2, DRAW_FILL_FULL);
    Paint_DrawRectangle(0, 195, 320, 240, 0X2595, DOT_PIXEL_2X2, DRAW_FILL_FULL);
    
    Paint_DrawString_EN(45, 30, "LongPress Quit", &Font16, BLACK, 0XF410);
    Paint_DrawString_EN(45, 50, "ACC_X = " , &Font16, BLACK, 0X4F30);
    Paint_DrawString_EN(45, 75, "ACC_Y = " , &Font16, BLACK, 0X4F30);
    Paint_DrawString_EN(45, 100, "ACC_Z = ", &Font16, BLACK, 0X4F30);
    Paint_DrawString_EN(45, 125, "GYR_X = ", &Font16, BLACK, 0XAD55);
    Paint_DrawString_EN(45, 150, "GYR_Y = ", &Font16, BLACK, 0XAD55);
    Paint_DrawString_EN(45, 175, "GYR_Z = ", &Font16, BLACK, 0XAD55);
    Paint_DrawString_EN(45, 200, "BAT(V)=" , &Font16, BLACK, 0X2595);
    LCD_2IN_Display(BlackImage);
    while (true)
    {
        uint16_t result = adc_read();
        printf("Raw value: 0x%03x, voltage: %f V\n", result, result * conversion_factor);
        Paint_Clear(WHITE);
        while(i2c_lock);
        I2C_LOCK();
        QMI8658_read_xyz(acc, gyro, &tim_count);
        I2C_UNLOCK();
        printf("acc_x   = %4.3fmg , acc_y  = %4.3fmg , acc_z  = %4.3fmg\r\n", acc[0], acc[1], acc[2]);
        printf("gyro_x  = %4.3fdps, gyro_y = %4.3fdps, gyro_z = %4.3fdps\r\n", gyro[0], gyro[1], gyro[2]);
        printf("tim_count = %d\r\n", tim_count);
        
        Paint_DrawRectangle(130, 49,  220, 120, 0X4F30, DOT_PIXEL_2X2, DRAW_FILL_FULL);
        Paint_DrawRectangle(130, 120, 220, 195, 0XAD55, DOT_PIXEL_2X2, DRAW_FILL_FULL);
        Paint_DrawRectangle(130, 195, 220, 240, 0X2595, DOT_PIXEL_2X2, DRAW_FILL_FULL);
        Paint_DrawNum(130, 50, acc[0], &Font16, 2, BLACK , 0X4F30);
        Paint_DrawNum(130, 75, acc[1], &Font16, 2, BLACK , 0X4F30);
        Paint_DrawNum(130, 100, acc[2], &Font16, 2, BLACK, 0X4F30);
        Paint_DrawNum(130, 125, gyro[0], &Font16, 2, BLACK, 0XAD55);
        Paint_DrawNum(130, 150, gyro[1], &Font16, 2, BLACK, 0XAD55);
        Paint_DrawNum(130, 175, gyro[2], &Font16, 2, BLACK, 0XAD55);
        Paint_DrawNum(130, 200, result * conversion_factor, &Font16, 2, BLACK, 0X2595);
        LCD_2IN_DisplayWindows(130, 50, 220, 200, BlackImage);
        LCD_2IN_DisplayWindows(130, 200, 220, 220, BlackImage);
        DEV_Delay_ms(100);
        if (flag == 1)
        {
            flag = 0;
            break;
        }
    }
#endif

#if 1
    while(i2c_lock);
    I2C_LOCK();
    CST816D_init(CST816D_Point_Mode);
    I2C_UNLOCK();
    Paint_Clear(WHITE);
    LCD_2IN_Display(BlackImage);
    Paint_DrawRectangle(0, 0, 320, 47, 0X2595, DOT_PIXEL_2X2, DRAW_FILL_FULL);
    Paint_DrawString_EN(100, 30, "Touch test", &Font16, BLACK, 0X2595);
    LCD_2IN_Display(BlackImage);
    while (true)
    {
        if (flag)
        {
            while(i2c_lock);
            I2C_LOCK();
            CST816D_Get_Point();
            I2C_UNLOCK();
            Paint_DrawPoint(Touch_CTS816.x_point, Touch_CTS816.y_point, BLACK, DOT_PIXEL_5X5, DOT_FILL_RIGHTUP);
            LCD_2IN_DisplayWindows(Touch_CTS816.x_point - 1, Touch_CTS816.y_point - 1, Touch_CTS816.x_point + 5, Touch_CTS816.y_point + 5, BlackImage);
            // printf("X:%d Y:%d\r\n", Touch_CTS816.x_point, Touch_CTS816.y_point);
            flag = 0;
        }
        __asm__ volatile("nop");
    }
#endif

    /* Module Exit */
    free(BlackImage);
    BlackImage = NULL;
    
    DEV_Module_Exit();
}

void Touch_INT_callback(uint gpio, uint32_t events)
{
    if (gpio == Touch_INT_PIN)
    {
        if (Touch_CTS816.mode == CST816D_Gesture_Mode)
        {
            while(i2c_lock);
            I2C_LOCK();
            uint8_t gesture = CST816D_Get_Gesture();
            I2C_UNLOCK();
            
            if (gesture == CST816D_Gesture_Long_Press)
            {
                flag = 1;
            }
        }
        else
        {
            flag = 1;
        }
    }
}
