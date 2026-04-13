/*****************************************************************************
 * | File      	:   CST816D.c
 * | Author      :   Waveshare team
 * | Function    :   Hardware underlying interface
 * | Info        :
 *                Used to shield the underlying layers of each master
 *                and enhance portability
 *----------------
 * |	This version:   V1.0
* | Date        :   2025-03-13
 * | Info        :   Basic version
 *
 ******************************************************************************/
#include "CST816D.h"
CST816D Touch_CTS816;

void CST816D_I2C_Write(uint8_t reg, uint8_t value)
{
    DEV_I2C_Write_Byte(CST816_ADDR, reg, value);
}
uint8_t CST816D_I2C_Read(uint8_t reg)
{
    uint8_t res;
    res = DEV_I2C_Read_Byte(CST816_ADDR, reg);
    return res;
}

uint8_t CST816D_WhoAmI()
{
    if (CST816D_I2C_Read(CST816_ChipID) == 0xB6)
        return true;
    else
        return false;
}

void CST816D_Reset()
{
    DEV_Digital_Write(Touch_RST_PIN, 0);
    DEV_Delay_ms(100);
    DEV_Digital_Write(Touch_RST_PIN, 1);
    DEV_Delay_ms(100);
}

uint8_t CST816D_Read_Revision()
{
    return CST816D_I2C_Read(CST816_FwVersion);
}

void CST816D_Wake_up()
{
    DEV_Digital_Write(Touch_RST_PIN, 0);
    DEV_Delay_ms(10);
    DEV_Digital_Write(Touch_RST_PIN, 1);
    DEV_Delay_ms(50);
    CST816D_I2C_Write(CST816_DisAutoSleep, 0x01);
}

void CST816D_Stop_Sleep()
{
    CST816D_I2C_Write(CST816_DisAutoSleep, 0x01);
}

void CST816D_Set_Mode(uint8_t mode)
{
    if (mode == CST816D_Point_Mode)
    {
        CST816D_I2C_Write(CST816_IrqCtl, 0x41);    
        Touch_CTS816.x_point = 0;
        Touch_CTS816.y_point = 0;
        Touch_CTS816.mode = mode;
    }
    else if (mode == CST816D_Gesture_Mode)
    {
        CST816D_I2C_Write(CST816_IrqCtl, 0X11);
        CST816D_I2C_Write(CST816_MotionMask, 0x01);
        Touch_CTS816.x_point = 0;
        Touch_CTS816.y_point = 0;
        Touch_CTS816.mode = mode;
    }
    else
    {
        CST816D_I2C_Write(CST816_IrqCtl, 0X71);
    }
        
}

uint8_t CST816D_init(uint8_t mode)
{
    uint8_t bRet, Rev;
    CST816D_Reset();

    bRet = CST816D_WhoAmI();
    if (bRet)
    {
        printf("Success:Detected CST816D.\r\n");
        Rev = CST816D_Read_Revision();
        printf("CST816D Revision = %d\r\n", Rev);
        CST816D_Stop_Sleep();
    }
    else
    {
        printf("Error: Not Detected CST816D.\r\n");
        return false;
    }

    CST816D_Set_Mode(mode);
    Touch_CTS816.x_point = 0;
    Touch_CTS816.y_point = 0;
    CST816D_I2C_Write(CST816_IrqPluseWidth, 0x01);
    CST816D_I2C_Write(CST816_NorScanPer, 0x01);
    
    Touch_CTS816.mode = mode;

    return true;
}

CST816D CST816D_Get_Point()
{
    uint8_t x_point_h, x_point_l, y_point_h, y_point_l;
    uint16_t x_point, y_point;
    // CST816D_Wake_up();

    x_point_h = CST816D_I2C_Read(CST816_XposH);
    x_point_l = CST816D_I2C_Read(CST816_XposL);
    y_point_h = CST816D_I2C_Read(CST816_YposH);
    y_point_l = CST816D_I2C_Read(CST816_YposL);

    x_point = ((x_point_h & 0x0f) << 8) + x_point_l;
    y_point = ((y_point_h & 0x0f) << 8) + y_point_l;

    Touch_CTS816.x_point = y_point;
    Touch_CTS816.y_point = 240 - x_point;

    return Touch_CTS816;
}
uint8_t CST816D_Get_Gesture(void)
{
    uint8_t gesture;
    gesture=CST816D_I2C_Read(CST816_GestureID);
    return gesture;
}
