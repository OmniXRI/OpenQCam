#include <bcm2835.h>

#define PIN_LCD_RESET RPI_V2_GPIO_P1_11
#define PIN_LCD_DC    RPI_V2_GPIO_P1_13
#define PIN_LCD_LED   RPI_V2_GPIO_P1_15
//#define PIN_LCD_MOSI  PRI_V2_GPIO_P1_19
//#define PIN_LCD_MISO  PRI_V2_GPIO_P1_21
//#define PIN_LCD_SCLK  PRI_V2_GPIO_P1_23
//#define PIN_LCD_CS    PRI_V2_GPIO_P1_24

#define PIN_GPIO_SW1 RPI_V2_GPIO_P1_40
#define PIN_GPIO_SW2 RPI_V2_GPIO_P1_37
#define PIN_GPIO_LED_R RPI_V2_GPIO_P1_35
#define PIN_GPIO_LED_G RPI_V2_GPIO_P1_33

int LCD_Inital(void);
int bcm2835_init(void);
void BCM2835_End(void);
int GPIO_Inital(void);
int SPI_Inital(void);
void LCD_SetReset(uint8_t);
void LCD_SetBacklight(uint8_t);
void LCD_SetMode(uint8_t);
void LCD_SetVerticalDisplay(void);
void LCD_SetHorizontalDisplay(void);
void LCD_HW_Reset(void);
void ILI9341_WriteCommand(char);
void ILI9341_WritePara(char);
void ILI9341_WriteData(char*,uint32_t);
void ILI9341_WriteLineBGR2RGB565(char*, uint32_t);
void ILI9341_SetWindow(uint16_t,uint16_t,uint16_t,uint16_t);
int  ILI9341_Inital(void);
void ILI9341_Wakeup(void);
void ILI9341_Sleep(void); 