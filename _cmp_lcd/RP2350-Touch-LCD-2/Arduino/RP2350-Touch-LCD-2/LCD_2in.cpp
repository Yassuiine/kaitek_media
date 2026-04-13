/*****************************************************************************
* | File      	:   LCD_2in.cpp
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
#include "LCD_2in.h"
#include "DEV_Config.h"

#include <stdlib.h>		//itoa()
#include <stdio.h>

LCD_2IN_ATTRIBUTES LCD_2IN;


/******************************************************************************
function :	Hardware reset
parameter:
******************************************************************************/
static void LCD_2IN_Reset(void)
{
    DEV_Digital_Write(LCD_RST_PIN, 1);
    DEV_Delay_ms(100);
    DEV_Digital_Write(LCD_RST_PIN, 0);
    DEV_Delay_ms(100);
    DEV_Digital_Write(LCD_RST_PIN, 1);
    DEV_Delay_ms(100);
}

/******************************************************************************
function :	send command
parameter:
     Reg : Command register
******************************************************************************/
static void LCD_2IN_SendCommand(UBYTE Reg)
{
    DEV_Digital_Write(LCD_DC_PIN, 0);
    DEV_Digital_Write(LCD_CS_PIN, 0);
    DEV_SPI_WriteByte(Reg);
    DEV_Digital_Write(LCD_CS_PIN, 1);
}

/******************************************************************************
function :	send data
parameter:
    Data : Write data
******************************************************************************/
static void LCD_2IN_SendData_8Bit(UBYTE Data)
{
    DEV_Digital_Write(LCD_DC_PIN, 1);
    DEV_Digital_Write(LCD_CS_PIN, 0);
    DEV_SPI_WriteByte(Data);
    DEV_Digital_Write(LCD_CS_PIN, 1);
}

/******************************************************************************
function :	send data
parameter:
    Data : Write data
******************************************************************************/
static void LCD_2IN_SendData_16Bit(UWORD Data)
{
    DEV_Digital_Write(LCD_DC_PIN, 1);
    DEV_Digital_Write(LCD_CS_PIN, 0);
    DEV_SPI_WriteByte((Data >> 8) & 0xFF);
    DEV_SPI_WriteByte(Data & 0xFF);
    DEV_Digital_Write(LCD_CS_PIN, 1);
}

/******************************************************************************
function :	Initialize the lcd register
parameter:
******************************************************************************/
static void LCD_2IN_InitReg(void)
{
    LCD_2IN_SendCommand(0x3A);     
    LCD_2IN_SendData_8Bit(0x05);   //LCD_2IN_SendData_8Bit(0x66);

    LCD_2IN_SendCommand(0xF0);     // Command Set Control
    LCD_2IN_SendData_8Bit(0xC3);   

    LCD_2IN_SendCommand(0xF0);     
    LCD_2IN_SendData_8Bit(0x96);   

    LCD_2IN_SendCommand(0xB4);     
    LCD_2IN_SendData_8Bit(0x01);   

    LCD_2IN_SendCommand(0xB7);     
    LCD_2IN_SendData_8Bit(0xC6);   

    LCD_2IN_SendCommand(0xC0);     
    LCD_2IN_SendData_8Bit(0x80);   
    LCD_2IN_SendData_8Bit(0x45);   

    LCD_2IN_SendCommand(0xC1);     
    LCD_2IN_SendData_8Bit(0x13);   //18  //00

    LCD_2IN_SendCommand(0xC2);     
    LCD_2IN_SendData_8Bit(0xA7);   

    LCD_2IN_SendCommand(0xC5);     
    LCD_2IN_SendData_8Bit(0x0A);   

    LCD_2IN_SendCommand(0xE8);     
    LCD_2IN_SendData_8Bit(0x40);
    LCD_2IN_SendData_8Bit(0x8A);
    LCD_2IN_SendData_8Bit(0x00);
    LCD_2IN_SendData_8Bit(0x00);
    LCD_2IN_SendData_8Bit(0x29);
    LCD_2IN_SendData_8Bit(0x19);
    LCD_2IN_SendData_8Bit(0xA5);
    LCD_2IN_SendData_8Bit(0x33);

    LCD_2IN_SendCommand(0xE0);
    LCD_2IN_SendData_8Bit(0xD0);
    LCD_2IN_SendData_8Bit(0x08);
    LCD_2IN_SendData_8Bit(0x0F);
    LCD_2IN_SendData_8Bit(0x06);
    LCD_2IN_SendData_8Bit(0x06);
    LCD_2IN_SendData_8Bit(0x33);
    LCD_2IN_SendData_8Bit(0x30);
    LCD_2IN_SendData_8Bit(0x33);
    LCD_2IN_SendData_8Bit(0x47);
    LCD_2IN_SendData_8Bit(0x17);
    LCD_2IN_SendData_8Bit(0x13);
    LCD_2IN_SendData_8Bit(0x13);
    LCD_2IN_SendData_8Bit(0x2B);
    LCD_2IN_SendData_8Bit(0x31);

    LCD_2IN_SendCommand(0xE1);
    LCD_2IN_SendData_8Bit(0xD0);
    LCD_2IN_SendData_8Bit(0x0A);
    LCD_2IN_SendData_8Bit(0x11);
    LCD_2IN_SendData_8Bit(0x0B);
    LCD_2IN_SendData_8Bit(0x09);
    LCD_2IN_SendData_8Bit(0x07);
    LCD_2IN_SendData_8Bit(0x2F);
    LCD_2IN_SendData_8Bit(0x33);
    LCD_2IN_SendData_8Bit(0x47);
    LCD_2IN_SendData_8Bit(0x38);
    LCD_2IN_SendData_8Bit(0x15);
    LCD_2IN_SendData_8Bit(0x16);
    LCD_2IN_SendData_8Bit(0x2C);
    LCD_2IN_SendData_8Bit(0x32);

    LCD_2IN_SendCommand(0xF0);     
    LCD_2IN_SendData_8Bit(0x3C);   

    LCD_2IN_SendCommand(0xF0);     
    LCD_2IN_SendData_8Bit(0x69);   

    DEV_Delay_ms(120);                

    LCD_2IN_SendCommand(0x21);     
	
    LCD_2IN_SendCommand(0x29); 
}

/********************************************************************************
function:	Set the resolution and scanning method of the screen
parameter:
		Scan_dir:   Scan direction
********************************************************************************/
void LCD_2IN_SetAttributes(UBYTE Scan_dir)
{
    //Get the screen scan direction
    LCD_2IN.SCAN_DIR = Scan_dir;
    UBYTE MemoryAccessReg = 0x00;

    //Get GRAM and LCD width and height
    if(Scan_dir == HORIZONTAL) {
        LCD_2IN.HEIGHT	= LCD_2IN_WIDTH;
        LCD_2IN.WIDTH   = LCD_2IN_HEIGHT;
        MemoryAccessReg = 0X28;
    } else {
        LCD_2IN.HEIGHT	= LCD_2IN_HEIGHT;       
        LCD_2IN.WIDTH   = LCD_2IN_WIDTH;
        MemoryAccessReg = 0X08;
    }

    // Set the read / write scan direction of the frame memory
    LCD_2IN_SendCommand(0x11); 
    sleep_ms(120); 
    LCD_2IN_SendCommand(0x36); //MX, MY, RGB mode
    LCD_2IN_SendData_8Bit(MemoryAccessReg);	//0x08 set RGB
}

/********************************************************************************
function :	Initialize the lcd
parameter:
********************************************************************************/
void LCD_2IN_Init(UBYTE Scan_dir)
{
    //Hardware reset
    LCD_2IN_Reset();

    //Set the resolution and scanning method of the screen
    LCD_2IN_SetAttributes(Scan_dir);
    
    //Set the initialization register
    LCD_2IN_InitReg();
}

/********************************************************************************
function:	Sets the start position and size of the display area
parameter:
		Xstart 	:   X direction Start coordinates
		Ystart  :   Y direction Start coordinates
		Xend    :   X direction end coordinates
		Yend    :   Y direction end coordinates
********************************************************************************/
void LCD_2IN_SetWindows(UWORD Xstart, UWORD Ystart, UWORD Xend, UWORD Yend)
{
    //set the X coordinates
    LCD_2IN_SendCommand(0x2A);
    LCD_2IN_SendData_8Bit(Xstart >> 8);
    LCD_2IN_SendData_8Bit(Xstart & 0xff);
	LCD_2IN_SendData_8Bit((Xend - 1) >> 8);
    LCD_2IN_SendData_8Bit((Xend - 1) & 0xFF);

    //set the Y coordinates
    LCD_2IN_SendCommand(0x2B);
    LCD_2IN_SendData_8Bit(Ystart >> 8);
	LCD_2IN_SendData_8Bit(Ystart & 0xff);
	LCD_2IN_SendData_8Bit((Yend - 1) >> 8);
    LCD_2IN_SendData_8Bit((Yend - 1) & 0xff);

    LCD_2IN_SendCommand(0X2C);
}

/******************************************************************************
function :	Clear screen
parameter:
******************************************************************************/
void LCD_2IN_Clear(UWORD Color)
{
    UWORD i;
	UWORD image[LCD_2IN.HEIGHT];
	for(i=0;i<LCD_2IN.HEIGHT;i++){
		image[i] = Color>>8 | (Color&0xff)<<8;
	}
	UBYTE *p = (UBYTE *)(image);
	LCD_2IN_SetWindows(0, 0, LCD_2IN.WIDTH,LCD_2IN.HEIGHT);
	DEV_Digital_Write(LCD_DC_PIN, 1);
	DEV_Digital_Write(LCD_CS_PIN, 0);
	for(i = 0; i < LCD_2IN.WIDTH; i++){
		DEV_SPI_Write_nByte(p,LCD_2IN.HEIGHT*2);
	}
    DEV_Digital_Write(LCD_CS_PIN, 1);
}

/******************************************************************************
function :	Sends the image buffer in RAM to displays
parameter:
******************************************************************************/
void LCD_2IN_Display(UWORD *Image)
{
    UWORD j;
    LCD_2IN_SetWindows(0, 0, LCD_2IN.WIDTH, LCD_2IN.HEIGHT);
    // LCD_2IN_SetWindows(0, 0, LCD_2IN.HEIGHT, LCD_2IN.WIDTH);
    DEV_Digital_Write(LCD_DC_PIN, 1);
    DEV_Digital_Write(LCD_CS_PIN, 0);
    for (j = 0; j < LCD_2IN.WIDTH; j++) {
        DEV_SPI_Write_nByte((UBYTE *)Image+LCD_2IN.HEIGHT*2*j,LCD_2IN.HEIGHT*2);
    }
    DEV_Digital_Write(LCD_CS_PIN, 1);
    LCD_2IN_SendCommand(0x29);
}

void LCD_2IN_DisplayWindows(UWORD Xstart, UWORD Ystart, UWORD Xend, UWORD Yend, UWORD *Image)
{
    // display
    UDOUBLE Addr = 0;

    UWORD j;
    LCD_2IN_SetWindows(Xstart, Ystart, Xend , Yend);
    DEV_Digital_Write(LCD_DC_PIN, 1);
    DEV_Digital_Write(LCD_CS_PIN, 0);
    for (j = Ystart; j < Yend - 1; j++) {
        Addr = Xstart + j * LCD_2IN.WIDTH ;
        DEV_SPI_Write_nByte((uint8_t *)&Image[Addr], (Xend-Xstart)*2);
    }
    DEV_Digital_Write(LCD_CS_PIN, 1);
}

void LCD_2IN_DisplayPoint(UWORD X, UWORD Y, UWORD Color)
{
    LCD_2IN_SetWindows(X,Y,X,Y);
    LCD_2IN_SendData_16Bit(Color);
}

void  Handler_2IN_LCD(int signo)
{
    //System Exit
    printf("\r\nHandler:Program stop\r\n");     
    DEV_Module_Exit();
	exit(0);
}
