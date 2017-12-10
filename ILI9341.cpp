#include "ILI9341.h"

// LCD module inital
int LCD_Inital(void)
{
 if(!bcm2835_init()){ //若BCM2835庫初始化失敗則返回錯誤碼1 
   return 1;
 }

 if(!GPIO_Inital()){ //若BCM2835 GPIO初始化失敗則返回錯誤碼2 
   return 2;
 }

 if(!SPI_Inital()){ //若BCM2835 SPI初始化失敗則返回錯誤碼3
   return 3;
 }

 LCD_SetBacklight(HIGH); //點亮LCD背光
 LCD_HW_Reset(); //重置LCD
 delay(50); //延時50ms

 if(!ILI9341_Inital()){
   LCD_SetBacklight(LOW); //若LCD初始化失敗則關閉背光
   return 4; // 返回錯誤碼4
 } 
 
 delay(300); //延時300ms
 
 return 0;
}

// BCM2835 函式庫初始化
int BCM2835_Inital()
{
 if(!bcm2835_init()){
   return 0;
 }
 
 return 1;
}

// 關閉 BCM2835 庫
void BCM2835_End()
{
 LCD_SetBacklight(LOW); //關閉LCD背光
 bcm2835_spi_end(); //結束SPI
 bcm2835_close();
}

// GPIO 初始化 設定接腳輸出入模式
int GPIO_Inital(void)
{
 bcm2835_gpio_fsel(PIN_LCD_RESET, BCM2835_GPIO_FSEL_OUTP);
 bcm2835_gpio_fsel(PIN_LCD_DC, BCM2835_GPIO_FSEL_OUTP);
 bcm2835_gpio_fsel(PIN_LCD_LED, BCM2835_GPIO_FSEL_OUTP);

 bcm2835_gpio_fsel(PIN_GPIO_SW1, BCM2835_GPIO_FSEL_INPT);
 bcm2835_gpio_fsel(PIN_GPIO_SW2, BCM2835_GPIO_FSEL_INPT);
 bcm2835_gpio_fsel(PIN_GPIO_LED_R, BCM2835_GPIO_FSEL_OUTP);
 bcm2835_gpio_fsel(PIN_GPIO_LED_G, BCM2835_GPIO_FSEL_OUTP);

 return 1;
}

// SPI 初始化
int SPI_Inital(void)
{
 if(!bcm2835_spi_begin()){
   return 0;
 }
 
 bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // 最高位元先行 
 bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // 模式0
 bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_16);    // 256MHz/DIVIDER = 16 MHz for LCD SCLK
 bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // 使用CE0_N
 bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      // 低電位方式驅動CE0_N

 return 1;
}

// 重置LCD模組
void LCD_SetReset(uint8_t state)
{
 bcm2835_gpio_write(PIN_LCD_RESET, state);
}

// 設定LCD背光 0關閉 1開啟 
void LCD_SetBacklight(uint8_t state)
{
 bcm2835_gpio_write(PIN_LCD_LED, state);
}

// 設定LCD讀寫模式
// LOW(0x0): 命令模式; HIGH(0x1): 資料模式
void LCD_SetMode(uint8_t state)
{
 bcm2835_gpio_write(PIN_LCD_DC, state);
}

// 設定LCD為垂直(240x320)顯示模式
void LCD_SetVerticalDisplay(void){
 ILI9341_WriteCommand(0x36);	// Memory Access Control 
 ILI9341_WritePara(0x48);	// Row Address Order(MY), Column Address Order(MX), Top-Down refresh order, BGR Color Filter
 ILI9341_SetWindow(0, 239, 0, 319);
}

// 設定LCD為水平(320x240)顯示模式
void LCD_SetHorizontalDisplay(void){
 ILI9341_WriteCommand(0x36);	// Memory Access Control 
 ILI9341_WritePara(0x28);	// Row Address Order(MY), Column Address Order(MX), Top-Down refresh order, BGR Color Filter
 ILI9341_SetWindow(0, 319, 0, 239);
}

// LCD硬體重置
void LCD_HW_Reset(void)
{
 LCD_SetReset(HIGH);
 delay(5);
 LCD_SetReset(LOW);
 delay(20);
 LCD_SetReset(HIGH);
 delay(150);
}

// 寫入 1 byte 命令到LCD模組
void ILI9341_WriteCommand(char cmd)
{
 char buf = cmd;

 LCD_SetMode(LOW); // set LCD module to command mode
 bcm2835_spi_transfern(&buf,1); // write to SPI without read back
}

// 寫入 1 byte 資料到LCD模組
void ILI9341_WritePara(char para)
{
 char buf = para;

 LCD_SetMode(HIGH); // set LCD module to data mode
 bcm2835_spi_transfern(&buf,1); // write to SPI without read back
}

// 連績寫入 n byte 資料到LCD模組
void ILI9341_WriteData(char *buf, uint32_t len)
{ 
 LCD_SetMode(HIGH); // set data mode
 bcm2835_spi_transfern(buf,len); // write to SPI without read back	
}


// 轉換OPENCV BGR(24bit)資料格式變為RGB565(16bit)並傳輸到LCD模組
// BGR24bit:[B7][B6][B5][B4][B3][B2][B1][B0]|[G7][G6][G5][G4][G3][G2][G1][G0]|[R7][R6][R5][R4][R3][R2][R1][R0]
// RGB16bit:[R7][R6][R5][R4][R3][G7][G6][G5]|[G4][G3][G2][B7][B6][B5][B4][B3]|
void ILI9341_WriteLineBGR2RGB565(char *buf, uint32_t len)
{ 
 char* pRGB565 = new char[len*2];
 int posS = 0;
 int posT = 0;

 for(int i=0; i<len; i++){
   posS += 3;
   posT += 2;
   pRGB565[ posT ] = (buf[posS+2] & 0xF8) | (buf[posS+1]>>5);
   pRGB565[posT+1] = ((buf[posS+1] & 0x1C) << 3) | (buf[posS]>>3);
 }

 LCD_SetMode(HIGH); // set data mode
 bcm2835_spi_transfern(pRGB565,len*2); // write to SPI without read back	
}

// Set Display Window
void ILI9341_SetWindow(uint16_t sx, uint16_t ex, uint16_t sy, uint16_t ey)
{
 ILI9341_WriteCommand(0x2A);	// Column Address Set (X Axis)
 ILI9341_WritePara(sx >> 8);	// Start Column High Byte
 ILI9341_WritePara(sx);	        // Start Column Low Byte
 ILI9341_WritePara(ex >> 8);	// End Column High Byte
 ILI9341_WritePara(ex);	        // End Column Low Byte

 ILI9341_WriteCommand(0x2B);	// Page Address Set (Y Axis)
 ILI9341_WritePara(sy >> 8);	// Start Page High Byte
 ILI9341_WritePara(sy);	        // Start Page Low Byte
 ILI9341_WritePara(ey >> 8);	// End Page High Byte
 ILI9341_WritePara(ey);	        // End Page Low Byte

 ILI9341_WriteCommand(0x2C);	// Memory Write
}

// LCD module inital
int ILI9341_Inital()
{
 ILI9341_WriteCommand(0xCB);	// Power Control A  
 ILI9341_WritePara(0x39); 
 ILI9341_WritePara(0x2C); 
 ILI9341_WritePara(0x00); 
 ILI9341_WritePara(0x34);	// vcore control, 1.6v
 ILI9341_WritePara(0x02); 	// ddvdh control, 5.6v

 ILI9341_WriteCommand(0xCF);	// Power Control B  
 ILI9341_WritePara(0x00); 
 ILI9341_WritePara(0XC1);	// power control 
 ILI9341_WritePara(0X30); 	// discharge path enable, enable ESD protection
 
 ILI9341_WriteCommand(0xE8);	// Driver Timing Control A  
 ILI9341_WritePara(0x85);	// get driver non-overlap timing control, default + 1 unit 
 ILI9341_WritePara(0x00); 	// EQ and CR timing control, both default-1unit
 ILI9341_WritePara(0x78); 	// pre-charge timing, default - 2 unit
 
 ILI9341_WriteCommand(0xEA);	// Driver Timing Control B
 ILI9341_WritePara(0x00);	// gate driver timing control 
 ILI9341_WritePara(0x00);	  
 
 ILI9341_WriteCommand(0xED);	// Power on Sequence Control
 ILI9341_WritePara(0x64);	// CP1 soft start keep 1 frame, CP23 soft start keep 3 frame
 ILI9341_WritePara(0x03); 	// Power on sequence control, En_vcl 1st frame enable, En_ddvdh 4th frame enable 
 ILI9341_WritePara(0X12);	// Power on sequence control, En_vgh 2nd frame enable, En_vgl 3rd frame enable 
 ILI9341_WritePara(0X81);	// DDVDH enhance mode, enable 

 ILI9341_WriteCommand(0xF7);	// Pump Ratio Control  
 ILI9341_WritePara(0x20);	// Ratio control, DDVDH = 2xVCI
  
 ILI9341_WriteCommand(0xC0);	// Power control 1
 ILI9341_WritePara(0x23);	// Set the GVDD level, 4.60v
 
 ILI9341_WriteCommand(0xC1);	// Power control 2
 ILI9341_WritePara(0x10);	// Sets the factor used in the  step-up circuits, DDVDH=VCI*2, VGH=VCI*7, VGL = -VCI*4 
 
 ILI9341_WriteCommand(0xC5);	// VCOM Control 
 ILI9341_WritePara(0x3e);	// Set VCOMH voltage, 4.200v
 ILI9341_WritePara(0x28);	// Set VCOML voltage, -0.700v 
 
 ILI9341_WriteCommand(0xC7);	// VCOM control2 
 ILI9341_WritePara(0x86);	// Set the CVOM offset voltage, VCOMH = VMH-58, VCOML = VML-58
 
 ILI9341_WriteCommand(0x36);	// Memory Access Control 
 ILI9341_WritePara(0x48);	// Row Address Order(MY), Column Address Order(MX), Top-Down refresh order, BGR Color Filter

 ILI9341_WriteCommand(0x3A);	// COLMOD: Pixel Format Set    
 ILI9341_WritePara(0x55);	// DPI and DBI are 16 bits / pixel

 ILI9341_WriteCommand(0xB1);	// Frame Rate Control (In Normal Mode)  
 ILI9341_WritePara(0x00);	// DIVA setting, fosc / 1
 ILI9341_WritePara(0x18);	// RTNA setting, Frame rate 79 Hz 
 
 ILI9341_WriteCommand(0xB6);	// Display Function Control 
 ILI9341_WritePara(0x08);	// Non-display area interval scan, determine source in partial display mode 
 ILI9341_WritePara(0x82);  	// REV normally white, GS SM G1->G320, SS S1-S720, ISC 5 frames 85ms
 ILI9341_WritePara(0x27);  	// LCD driver line 320 lines
 
 ILI9341_WriteCommand(0xF2);	// Enable 3G 
 ILI9341_WritePara(0x00);	// disable 3 gamma control
 
 ILI9341_WriteCommand(0x26);	// Gamma Set 
 ILI9341_WritePara(0x01);	// Gamma curve 1 (G2.2)
 
 ILI9341_WriteCommand(0xE0);	// Positive Gamma Correction 
 ILI9341_WritePara(0x0F);	 
 ILI9341_WritePara(0x31); 
 ILI9341_WritePara(0x2B); 
 ILI9341_WritePara(0x0C); 
 ILI9341_WritePara(0x0E); 
 ILI9341_WritePara(0x08); 
 ILI9341_WritePara(0x4E); 
 ILI9341_WritePara(0xF1); 
 ILI9341_WritePara(0x37); 
 ILI9341_WritePara(0x07); 
 ILI9341_WritePara(0x10); 
 ILI9341_WritePara(0x03); 
 ILI9341_WritePara(0x0E); 
 ILI9341_WritePara(0x09); 
 ILI9341_WritePara(0x00); 

 ILI9341_WriteCommand(0XE1);    // Negative Gamma Correction
 ILI9341_WritePara(0x00); 
 ILI9341_WritePara(0x0E); 
 ILI9341_WritePara(0x14); 
 ILI9341_WritePara(0x03); 
 ILI9341_WritePara(0x11); 
 ILI9341_WritePara(0x07); 
 ILI9341_WritePara(0x31); 
 ILI9341_WritePara(0xC1); 
 ILI9341_WritePara(0x48); 
 ILI9341_WritePara(0x08); 
 ILI9341_WritePara(0x0F); 
 ILI9341_WritePara(0x0C); 
 ILI9341_WritePara(0x31); 
 ILI9341_WritePara(0x36); 
 ILI9341_WritePara(0x0F); 

 ILI9341_Wakeup();

 return 1;
}

// LCD module exit sleep mode
void ILI9341_Wakeup(void)
{
 ILI9341_WriteCommand(0x11);	// Sleep Out
 delay(120);			// delay 120ms, must be > 5 ms
 ILI9341_WriteCommand(0x29);	// Display On
 ILI9341_WriteCommand(0x2C);	// Memory Write 
}

//LCD module into sleep mode
void ILI9341_Sleep(void)
{
 ILI9341_WriteCommand(0x28);	// Display Off
 delay(20);                     // delay 20ms
 ILI9341_WriteCommand(0x10);	// Enter Sleep Mode
}

