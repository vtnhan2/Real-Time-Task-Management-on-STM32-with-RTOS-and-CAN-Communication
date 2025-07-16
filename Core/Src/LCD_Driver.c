/*****************************************************************************
* | File      	:		LCD_Driver.c
* | Author      :   Waveshare team
* | Function    :   LCD_Driver driver of ST7789 & HX8347
* | Info        :
*----------------
* |	This version:   V1.2
* | Date        :   2019-08-16
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

#include "LCD_Driver.h"
#include "LCD_lib.h"
#include "math.h"
#include <stdlib.h>

/******************************************************************************
function :	initial SPI1
parameter:
******************************************************************************/
//void spi_init(void)
//{
//
//	SPI_InitTypeDef SPI_InitStructure;
//	GPIO_InitTypeDef GPIO_InitStruct;
//
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOA,ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
//
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1);
// 	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
//
//
//	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
//	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//	GPIO_InitStruct.GPIO_Pin =GPIO_Pin_6| GPIO_Pin_7;
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
//	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//	SPI_I2S_DeInit(SPI1);
//	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
//	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
//	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
//	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
//	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
//	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
//	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
//	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
//	SPI_Init(SPI1, &SPI_InitStructure);
//
//	SPI_Cmd(SPI1, ENABLE);
//
//}
//
///******************************************************************************
//function :	SPI1 send data
//parameter:
//send_char:	sned data
//******************************************************************************/
//uint8_t spi1_communication(uint8_t send_char)
//{
//	uint8_t chRetry = 0;
//	uint8_t chTemp = 0;
//
//	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) {
//		if (++ chRetry > 200) {
//			return 0;
//		}
//	}
//
//	SPI_I2S_SendData(SPI1, send_char);
//
//	chRetry=0;
//	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) {
//		if (++ chRetry > 200) {
//			return 0;
//		}
//	}
//
//	chTemp = SPI_I2S_ReceiveData(SPI1);
//
//	/* Wait until the BSY flag is set */
//	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) != RESET) {
//
//	}
//
//	return chTemp;
//}

extern SPI_HandleTypeDef hspi1;
uint8_t spi1_communication(uint8_t send_char)
{
    uint8_t receive_char = 0;
    HAL_StatusTypeDef status;

    status = HAL_SPI_TransmitReceive(&hspi1, &send_char, &receive_char, 1, 10);  // timeout 10ms

    if (status == HAL_OK) {
        return receive_char;
    } else {
        return 0;  // lỗi
    }
}

/******************************************************************************
function :	write eight bits' data to LCD
parameter:
  chByte : send data
  chCmd  : command or data
******************************************************************************/
void lcd_write_byte(uint8_t chByte, uint8_t chCmd)
{
    if (chCmd) {
        LCD_DC_H();
    } else {
        LCD_DC_L();
    }

    LCD_CS_L();
    spi1_communication(chByte);
    LCD_CS_H();
}

/******************************************************************************
function :	write sixteen bits' data to LCD
parameter:
  chByte : send data
  chCmd  : command or data
******************************************************************************/
void lcd_write_word(uint16_t hwData)
{
    LCD_DC_H();
    LCD_CS_L();
    spi1_communication(hwData >> 8);
    spi1_communication(hwData & 0xFF);
    LCD_CS_H();
}

/******************************************************************************
function :	write data to LCD register
parameter:
     chByte : send data
		 chCmd  : command or data
******************************************************************************/
void lcd_write_command(uint8_t chRegister, uint8_t chValue)
{
	lcd_write_byte(chRegister, LCD_CMD);
	lcd_write_byte(chValue, LCD_DATA);
}

/********************************************************************************
Function Name  : initials lcd control pin
			parameter:
********************************************************************************/
//void lcd_ctrl_port_init(void)
//{
//	GPIO_InitTypeDef GPIO_InitStruct;
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
//
//	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_6 | GPIO_Pin_2;
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//	GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//}

/******************************************************************************
Function Name  : delay
			parameter: ms
******************************************************************************/
void delay_ms(uint32_t ms)
{
	uint32_t j=5000;
	for(;ms>2;ms--)
		for(;j>2;j--){

		}
}

/******************************************************************************
Function Name  : initials lcd control pin
			parameter:
******************************************************************************/
void lcd_init(void)
{
  //lcd_ctrl_port_init();
	LCD_RST_H();
  //Sspi_init();

	LCD_CS_H();
	LCD_BKL_H();
#ifdef 	ST7789_DEVICE
	LCD_RST_H();
	delay_ms(5);
	LCD_RST_L();
	delay_ms(5);
	LCD_RST_H();
	delay_ms(5);
	LCD_CS_H();
#endif

#ifdef HX8347D_DEVICE
	lcd_write_command(0xEA,0x00);
	lcd_write_command(0xEB,0x20);
	lcd_write_command(0xEC,0x0C);
	lcd_write_command(0xED,0xC4);
	lcd_write_command(0xE8,0x38);
	lcd_write_command(0xE9,0x10);
	lcd_write_command(0xF1,0x01);
	lcd_write_command(0xF2,0x10);
	lcd_write_command(0x40,0x01);
	lcd_write_command(0x41,0x00);
	lcd_write_command(0x42,0x00);
	lcd_write_command(0x43,0x10);
	lcd_write_command(0x44,0x0E);
	lcd_write_command(0x45,0x24);
	lcd_write_command(0x46,0x04);
	lcd_write_command(0x47,0x50);
	lcd_write_command(0x48,0x02);
	lcd_write_command(0x49,0x13);
	lcd_write_command(0x4A,0x19);
	lcd_write_command(0x4B,0x19);
	lcd_write_command(0x4C,0x16);
	lcd_write_command(0x50,0x1B);
	lcd_write_command(0x51,0x31);
	lcd_write_command(0x52,0x2F);
	lcd_write_command(0x53,0x3F);
	lcd_write_command(0x54,0x3F);
	lcd_write_command(0x55,0x3E);
	lcd_write_command(0x56,0x2F);
	lcd_write_command(0x57,0x7B);
	lcd_write_command(0x58,0x09);
	lcd_write_command(0x59,0x06);
	lcd_write_command(0x5A,0x06);
	lcd_write_command(0x5B,0x0C);
	lcd_write_command(0x5C,0x1D);
	lcd_write_command(0x5D,0xCC);
	lcd_write_command(0x1B,0x1B);
	lcd_write_command(0x1A,0x01);
	lcd_write_command(0x24,0x2F);
	lcd_write_command(0x25,0x57);
	lcd_write_command(0x23,0x88);
	lcd_write_command(0x18,0x34);
	lcd_write_command(0x19,0x01);
	lcd_write_command(0x01,0x00);
	lcd_write_command(0x1F,0x88);
	lcd_write_command(0x1F,0x80);
	lcd_write_command(0x1F,0x90);
	lcd_write_command(0x1F,0xD0);
	lcd_write_command(0x17,0x05);
	lcd_write_command(0x36,0x02);
	lcd_write_command(0x28,0x38);
	lcd_write_command(0x28,0x3F);
	lcd_write_command(0x16,0x18);
	lcd_write_command(0x02,0x00);
	lcd_write_command(0x03,0x00);
	lcd_write_command(0x04,0x00);
	lcd_write_command(0x05,0xEF);
	lcd_write_command(0x06,0x00);
	lcd_write_command(0x07,0x00);
	lcd_write_command(0x08,0x01);
	lcd_write_command(0x09,0x3F);
	lcd_write_byte(0x20, LCD_CMD);


#elif defined ST7789_DEVICE
	lcd_write_byte(0x11,LCD_CMD);
	delay_ms(10);
	lcd_write_command(0x36,0x00);
	lcd_write_command(0x3a,0x05);
	lcd_write_byte(0xb2,LCD_CMD);
	lcd_write_byte(0x0c,LCD_DATA);
	lcd_write_byte(0x0c,LCD_DATA);
	lcd_write_byte(0x00,LCD_DATA);
	lcd_write_byte(0x33,LCD_DATA);
	lcd_write_byte(0x33,LCD_DATA);
	lcd_write_command(0xb7,0x35);
	lcd_write_command(0xbb,0x28);
	lcd_write_command(0xc0,0x3c);
	lcd_write_command(0xc2,0x01);
	lcd_write_command(0xc3,0x0b);
	lcd_write_command(0xc4,0x20);
	lcd_write_command(0xc6,0x0f);
	lcd_write_byte(0xD0,LCD_CMD);
	lcd_write_byte(0xa4,LCD_DATA);
	lcd_write_byte(0xa1,LCD_DATA);
	lcd_write_byte(0xe0,LCD_CMD);
	lcd_write_byte(0xd0,LCD_DATA);
	lcd_write_byte(0x01,LCD_DATA);
	lcd_write_byte(0x08,LCD_DATA);
	lcd_write_byte(0x0f,LCD_DATA);
	lcd_write_byte(0x11,LCD_DATA);
	lcd_write_byte(0x2a,LCD_DATA);
	lcd_write_byte(0x36,LCD_DATA);
	lcd_write_byte(0x55,LCD_DATA);
	lcd_write_byte(0x44,LCD_DATA);
	lcd_write_byte(0x3a,LCD_DATA);
	lcd_write_byte(0x0b,LCD_DATA);
	lcd_write_byte(0x06,LCD_DATA);
	lcd_write_byte(0x11,LCD_DATA);
	lcd_write_byte(0x20,LCD_DATA);
	lcd_write_byte(0xe1,LCD_CMD);
	lcd_write_byte(0xd0,LCD_DATA);
	lcd_write_byte(0x01,LCD_DATA);
	lcd_write_byte(0x07,LCD_DATA);
	lcd_write_byte(0x0a,LCD_DATA);
	lcd_write_byte(0x0b,LCD_DATA);
	lcd_write_byte(0x18,LCD_DATA);
	lcd_write_byte(0x34,LCD_DATA);
	lcd_write_byte(0x43,LCD_DATA);
	lcd_write_byte(0x4a,LCD_DATA);
	lcd_write_byte(0x2b,LCD_DATA);
	lcd_write_byte(0x1b,LCD_DATA);
	lcd_write_byte(0x1c,LCD_DATA);
	lcd_write_byte(0x22,LCD_DATA);
	lcd_write_byte(0x1f,LCD_DATA);
	//lcd_write_byte(0x21, LCD_CMD);
	lcd_write_byte(0x29,LCD_CMD);
	lcd_write_command(0x51,0xff);
	lcd_write_command(0x55,0xB0);
#endif

	lcd_clear_screen(BLACK);
}

/******************************************************************************
Function Name  : clear lcd screen
			parameter:
				hwColor: background color
******************************************************************************/
//void lcd_clear_screen(uint16_t hwColor)
//{
//	uint32_t i, wCount = LCD_WIDTH;
//	wCount *= LCD_HEIGHT;
//
//#ifdef HX8347D_DEVICE
//	lcd_set_cursor(0, 0);
//	lcd_write_byte(0x22, LCD_CMD);
//#elif defined ST7789_DEVICE
//	lcd_write_byte(0x2A,LCD_CMD);
//	lcd_write_byte(0x00,LCD_DATA);
//	lcd_write_byte(0x00,LCD_DATA);
//	lcd_write_byte(0x00,LCD_DATA);
//	lcd_write_byte((LCD_WIDTH-1)&0xff,LCD_DATA);
//	lcd_write_byte(0x2B,LCD_CMD);
//	lcd_write_byte(0x00,LCD_DATA);
//	lcd_write_byte(0x00,LCD_DATA);
//	lcd_write_byte(((LCD_HEIGHT-1)>>8)&0xff,LCD_DATA);
//	lcd_write_byte((LCD_HEIGHT-1)&0xff,LCD_DATA);
//	lcd_write_byte(0x2C,LCD_CMD);
//#endif
//	LCD_CS_L();
//	LCD_DC_H();
//	for(i=0;i<wCount;i++){
//#ifdef HX8347D_DEVICE
////		spi1_communication((uint8_t)(hwColor&0xff));
////		spi1_communication(hwColor>>8);
//
//		spi1_communication(hwColor>>8);
//		spi1_communication((uint8_t)(hwColor&0xff));
//#elif defined ST7789_DEVICE
//		spi1_communication(hwColor>>8);
//		spi1_communication((uint8_t)(hwColor&0xff));
//#endif
//	}
//	LCD_CS_H();
//}

void lcd_clear_screen(uint16_t hwColor)
{
    uint32_t wCount = LCD_WIDTH * LCD_HEIGHT;
    uint8_t color_buf[512]; // Smaller buffer, e.g., 256 pixels (512 bytes)
    uint16_t buf_size = sizeof(color_buf) / 2;

    // Fill small buffer
    for (uint16_t i = 0; i < buf_size * 2; i += 2) {
        color_buf[i] = hwColor >> 8;
        color_buf[i + 1] = hwColor & 0xFF;
    }

#ifdef HX8347D_DEVICE
    lcd_set_cursor(0, 0);
    lcd_write_byte(0x22, LCD_CMD);
#elif defined ST7789_DEVICE
    lcd_write_byte(0x2A, LCD_CMD);
    lcd_write_byte(0x00, LCD_DATA);
    lcd_write_byte(0x00, LCD_DATA);
    lcd_write_byte(0x00, LCD_DATA);
    lcd_write_byte((LCD_WIDTH - 1) & 0xFF, LCD_DATA);
    lcd_write_byte(0x2B, LCD_CMD);
    lcd_write_byte(0x00, LCD_DATA);
    lcd_write_byte(0x00, LCD_DATA);
    lcd_write_byte(((LCD_HEIGHT - 1) >> 8) & 0xFF, LCD_DATA);
    lcd_write_byte((LCD_HEIGHT - 1) & 0xFF, LCD_DATA);
    lcd_write_byte(0x2C, LCD_CMD);
#endif

    LCD_CS_L();
    LCD_DC_H();
    // Send buffer multiple times to cover entire screen
    for (uint32_t i = 0; i < wCount; i += buf_size) {
        uint16_t remaining = (wCount - i) > buf_size ? buf_size : (wCount - i);
        HAL_SPI_Transmit(&hspi1, color_buf, remaining * 2, HAL_MAX_DELAY);
    }
    LCD_CS_H();
}


/******************************************************************************
Function Name  : set lcd cursor
			parameter:
				 hwXpos: x axis position
				 hwYpos: y axis position
******************************************************************************/
void lcd_set_cursor(uint16_t hwXpos, uint16_t hwYpos)
{
	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}
#ifdef HX8347D_DEVICE
	lcd_write_command(0x02, hwXpos >> 8);
	lcd_write_command(0x03, hwXpos & 0xFF);
	lcd_write_command(0x06, hwYpos >> 8);
	lcd_write_command(0x07, hwYpos & 0xFF);
#elif defined ST7789_DEVICE
	lcd_write_byte(0x2A,LCD_CMD);
	lcd_write_byte(0x00,LCD_DATA);
	lcd_write_byte(hwXpos,LCD_DATA);
	lcd_write_byte(0x2B,LCD_CMD);
	lcd_write_byte((hwYpos>>8)&0xff,LCD_DATA);
	lcd_write_byte(hwYpos&0xff,LCD_DATA);
#endif
}

/******************************************************************************
Function Name  : lcd display char
			parameter:
				 hwXpos: x axis position
				 hwYpos: y axis position
				  chChr: display character
				 chSize: character size
			  hwColor: character color
******************************************************************************/
void lcd_display_char(	 uint16_t hwXpos,
                         uint16_t hwYpos,
                         uint8_t chChr,
                         uint8_t chSize,
                         uint16_t hwColor)
{
	uint8_t i, j, chTemp;
	uint16_t hwYpos0 = hwYpos, hwColorVal = 0;

	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}
#ifdef HX8347D_DEVICE

#elif defined ST7789_DEVICE
	lcd_write_byte(0x2A,LCD_CMD);
	lcd_write_byte(0x00,LCD_DATA);
	lcd_write_byte(hwXpos,LCD_DATA);
	lcd_write_byte((hwXpos) >> 8,LCD_DATA);
	lcd_write_byte((hwXpos) & 0xFF,LCD_DATA);

	lcd_write_byte(0x2B,LCD_CMD);
	lcd_write_byte(hwYpos >> 8,LCD_DATA);
	lcd_write_byte(hwYpos & 0xFF,LCD_DATA);
	lcd_write_byte((hwYpos) >> 8,LCD_DATA);
	lcd_write_byte((hwYpos) & 0xFF,LCD_DATA);
	lcd_write_byte(0x2C, LCD_CMD);
#endif
    for (i = 0; i < chSize; i ++) {
				if (FONT_1206 == chSize) {
					chTemp = c_chFont1206[chChr - 0x20][i];
				}
				else if (FONT_1608 == chSize) {
					chTemp = c_chFont1608[chChr - 0x20][i];
				}
        for (j = 0; j < 8; j ++) {
					if (chTemp & 0x80) {
						hwColorVal = hwColor;
						lcd_draw_dot(hwXpos, hwYpos, hwColorVal);
					}
					chTemp <<= 1;
					hwYpos ++;
					if ((hwYpos - hwYpos0) == chSize) {
						hwYpos = hwYpos0;
						hwXpos ++;
						break;
					}
				}
    }
}
/******************************************************************************
Function Name  : lcd display string
			parameter:
				 hwXpos: x axis position
				 hwYpos: y axis position
		  pchString: display string
				 chSize: string size
			  hwColor: string color
******************************************************************************/
void lcd_display_string(	uint16_t hwXpos,uint16_t hwYpos,
													const uint8_t *pchString,
													uint8_t chSize,uint16_t hwColor)
{

	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}

    while (*pchString != '\0') {
        if (hwXpos > (LCD_WIDTH - chSize / 2)) {
					hwXpos = 0;
					hwYpos += chSize;
					if (hwYpos > (LCD_HEIGHT - chSize)) {
						hwYpos = hwXpos = 0;
						lcd_clear_screen(0x00);
					}
				}

        lcd_display_char(hwXpos, hwYpos, (uint8_t)*pchString, chSize, hwColor);
        hwXpos += chSize / 2;
        pchString ++;
    }
}

/******************************************************************************
Function Name  : lcd display chinese character
			parameter:
						 gb:	which character in GB2312[][32]
	  color_front:	character color
				 hwXpos: x axis position
				 hwYpos: y axis position
******************************************************************************/
void lcd_display_GB2312(  uint8_t gb, uint16_t color_front,
													uint16_t postion_x,uint16_t postion_y )
{
	uint8_t i, j,chTemp;
	uint16_t hwYpos0 = postion_y, hwColorVal = 0;

	if (postion_x >= LCD_WIDTH || postion_y >= LCD_HEIGHT) {
		return;
	}

	for (i = 0; i < 32; i++) {
		chTemp = GB2312[gb][i];
		for (j = 0; j < 8; j++) {
			if (chTemp & 0x80) {
					hwColorVal = color_front;
				if(i<15)
					lcd_draw_dot(postion_x, postion_y, hwColorVal);
				else
					lcd_draw_dot(postion_x-16, postion_y+8, hwColorVal);
			}
			chTemp <<= 1;
			postion_y ++;
			if ((postion_y - hwYpos0) == 8) {
				postion_y = hwYpos0;
				postion_x ++;
				break;
			}
		}
	}
}

/******************************************************************************
Function Name  : lcd draw a dot
			parameter:
				 hwXpos: x axis position
				 hwYpos: y axis position
				hwColor:	dot color
******************************************************************************/
void lcd_draw_dot(uint16_t hwXpos, uint16_t hwYpos, uint16_t hwColor)
{
	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}
	lcd_set_cursor(hwXpos, hwYpos);
#ifdef HX8347D_DEVICE
	lcd_write_byte(0x22, LCD_CMD);
#elif defined ST7789_DEVICE
	lcd_write_byte(0x2C, LCD_CMD);
#endif
	lcd_write_word(hwColor);

}

/******************************************************************************
Function Name  : lcd draw a big dot
			parameter:
	  color_front:	dot color
		  	 hwXpos: x axis position
				h wYpos: y axis position
******************************************************************************/
void lcd_draw_bigdot(uint32_t color_front,
                     uint32_t x,uint32_t y )
{
    lcd_draw_dot(color_front,x,y);
    lcd_draw_dot(color_front,x,y+1);
    lcd_draw_dot(color_front,x,y-1);

    lcd_draw_dot(color_front,x+1,y);
    lcd_draw_dot(color_front,x+1,y+1);
    lcd_draw_dot(color_front,x+1,y-1);

    lcd_draw_dot(color_front,x-1,y);
    lcd_draw_dot(color_front,x-1,y+1);
    lcd_draw_dot(color_front,x-1,y-1);

}

/******************************************************************************
Function Name  : calculate N power
			parameter:
						 m :	valure
					   n :  exponent
******************************************************************************/
static uint32_t _pow(uint8_t m, uint8_t n)
{
	uint32_t result = 1;

	while(n --) result *= m;
	return result;
}

/******************************************************************************
Function Name  : lcd display number
			parameter:
				 hwXpos: x axis position
				 hwYpos: y axis position
				  chNum: number
				  chLen: number length
				 chSize: number size
				hwColor: number color
******************************************************************************/
void lcd_display_num(			uint16_t hwXpos,  uint16_t hwYpos,
                          uint32_t chNum,  uint8_t chLen,
                          uint8_t chSize,  uint16_t hwColor)
{
	uint8_t i;
	uint8_t chTemp, chShow = 0;

	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}

	for(i = 0; i < chLen; i ++) {
		chTemp = (chNum / _pow(10, chLen - i - 1)) % 10;
		if(chShow == 0 && i < (chLen - 1)) {
			if(chTemp == 0) {
				lcd_display_char(hwXpos + (chSize / 2) * i, hwYpos, ' ', chSize, hwColor);
				continue;
			} else {
				chShow = 1;
			}
		}
	 	lcd_display_char(hwXpos + (chSize / 2) * i, hwYpos, chTemp + '0', chSize, hwColor);
	}
}

/******************************************************************************
Function Name  : lcd draw a line
			parameter:
				hwXpos0: x axis start position
				hwYpos0: y axis start position
				hwXpos1: x axis end position
				hwYpos1: y axis end position
				hwColor: line color
******************************************************************************/
void lcd_draw_line(		uint16_t hwXpos0, uint16_t hwYpos0,
                      uint16_t hwXpos1, uint16_t hwYpos1,
                      uint16_t hwColor)
{
	int x = hwXpos1 - hwXpos0;
	int y = hwYpos1 - hwYpos0;
	int dx = abs(x), sx = hwXpos0 < hwXpos1 ? 1 : -1;
	int dy = -abs(y), sy = hwYpos0 < hwYpos1 ? 1 : -1;
	int err = dx + dy, e2;

	if (hwXpos0 >= LCD_WIDTH || hwYpos0 >= LCD_HEIGHT || hwXpos1 >= LCD_WIDTH || hwYpos1 >= LCD_HEIGHT) {
		return;
	}

    for (;;){
        lcd_draw_dot(hwXpos0, hwYpos0 , hwColor);
        e2 = 2 * err;
        if (e2 >= dy) {
            if (hwXpos0 == hwXpos1) break;
            err += dy; hwXpos0 += sx;
        }
        if (e2 <= dx) {
            if (hwYpos0 == hwYpos1) break;
            err += dx; hwYpos0 += sy;
        }
    }
}

/******************************************************************************
Function Name  : lcd draw a circle
			parameter:
				hwXpos: x axis  position
				hwYpos: y axis  position
			 hwRadius: circle radius
				hwColor: cirlce color
******************************************************************************/
void lcd_draw_circle(		uint16_t hwXpos, uint16_t hwYpos,
                        uint16_t hwRadius,uint16_t hwColor)
{
	int x = -hwRadius, y = 0, err = 2 - 2 * hwRadius, e2;

	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}

    do {
        lcd_draw_dot(hwXpos - x, hwYpos + y, hwColor);
        lcd_draw_dot(hwXpos + x, hwYpos + y, hwColor);
        lcd_draw_dot(hwXpos + x, hwYpos - y, hwColor);
        lcd_draw_dot(hwXpos - x, hwYpos - y, hwColor);
        e2 = err;
        if (e2 <= y) {
            err += ++ y * 2 + 1;
            if(-x == y && e2 <= x) e2 = 0;
        }
        if(e2 > x) err += ++ x * 2 + 1;
    } while(x <= 0);
}

/******************************************************************************
Function Name  :  fill a rectangle on lcd
			parameter:
				 hwXpos: x axis  position
				 hwYpos: y axis  position
			  hwWidth: rectangle width
			 hwHeight: rectangle height
			  hwColor: rectangle color
******************************************************************************/
void lcd_fill_rect(uint16_t hwXpos,
                   uint16_t hwYpos, uint16_t hwWidth,
                   uint16_t hwHeight,uint16_t hwColor)
{
	uint16_t i, j;

	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}

	for(i = 0; i < hwHeight; i ++){
		for(j = 0; j < hwWidth; j ++){
			lcd_draw_dot(hwXpos + j, hwYpos + i, hwColor);
		}
	}
}

/******************************************************************************
Function Name  : draw a vertical line at the specified position on lcd
			parameter:
				 hwXpos: x axis  position
				 hwYpos: y axis  position
			 hwHeight: line height
			  hwColor: vertical linc color
******************************************************************************/
void lcd_draw_v_line(		uint16_t hwXpos,uint16_t hwYpos,
                        uint16_t hwHeight,uint16_t hwColor)
{
	uint16_t i, y1 = MIN(hwYpos + hwHeight, LCD_HEIGHT - 1);

	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}

    for (i = hwYpos; i < y1; i ++) {
        lcd_draw_dot(hwXpos, i, hwColor);
    }
}

/******************************************************************************
Function Name  : draw a horizonal line at the specified position on lcd
			parameter:
				 hwXpos: x axis  position
				 hwYpos: y axis  position
			  hwWidth: line width
			  hwColor: horizonal linc color
******************************************************************************/
void lcd_draw_h_line(		uint16_t hwXpos, uint16_t hwYpos,
                        uint16_t hwWidth,uint16_t hwColor)
{
	uint16_t i, x1 = MIN(hwXpos + hwWidth, LCD_WIDTH - 1);

	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}

    for (i = hwXpos; i < x1; i ++) {
        lcd_draw_dot(i, hwYpos, hwColor);
    }
}

/******************************************************************************
Function Name  : draw a rectangle on lcd
			parameter:
				 hwXpos: x axis  position
				 hwYpos: y axis  position
			  hwWidth: rectangle width
			 hwHeight: rectangle height
			  hwColor: rectangle color
******************************************************************************/
void lcd_draw_rect(		uint16_t hwXpos,
                      uint16_t hwYpos,uint16_t hwWidth,
                      uint16_t hwHeight,uint16_t hwColor)
{
	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}

	lcd_draw_h_line(hwXpos, hwYpos, hwWidth, hwColor);
	lcd_draw_h_line(hwXpos, hwYpos + hwHeight, hwWidth, hwColor);
	lcd_draw_v_line(hwXpos, hwYpos, hwHeight, hwColor);
	lcd_draw_v_line(hwXpos + hwWidth, hwYpos, hwHeight + 1, hwColor);
}

/******************************************************************************
Function Name  : clear rectangle on lcd
			parameter:
				 hwXpos: x axis  position
				 hwYpos: y axis  position
			  hwXpos1: rectangle width
			  hwYpos1: rectangle height
	  color_front: rectangle color
******************************************************************************/
void lcd_clear_Rect(	uint32_t color_front,
											uint32_t hwXpos,uint32_t hwYpos,
											uint32_t hwXpos1,uint32_t hwYpos1)
{
	uint16_t i, j;

	if (hwXpos1 >= LCD_WIDTH || hwYpos1 >= LCD_HEIGHT) {
		return;
	}

	for(i = 0; i < hwYpos1-hwYpos+1; i ++){
		for(j = 0; j < hwXpos1-hwXpos+1; j ++){
			lcd_draw_dot(hwXpos + j, hwYpos + i, color_front);
		}
	}
}



/*-------------------------------END OF FILE-------------------------------*/
