/*
 * LCD12864_rom_enable.h
 *
 *  Created on: 2018��8��14��
 *      Author: rebeater
 */
#define TARGET_IS_TM4C123_RA1
#ifndef LCD12864_ROM_ENABLE_H_
#define LCD12864_ROM_ENABLE_H_

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"

#include "driverlib/rom.h"

// sclk
#define LCD12864_SCLK_PORT GPIO_PORTJ_BASE
#define LCD12864_SCLK_PIN GPIO_PIN_0
// SDA
#define LCD12864_SDA_PORT GPIO_PORTJ_BASE
#define LCD12864_SDA_PIN GPIO_PIN_1
// rs
#define LCD12864_RS_PORT GPIO_PORTB_BASE
#define LCD12864_RS_PIN GPIO_PIN_3
// reset
#define LCD12864_RST_PORT GPIO_PORTJ_BASE
#define LCD12864_RST_PIN GPIO_PIN_2
// cs
#define LCD12864_CS_PORT GPIO_PORTB_BASE
#define LCD12864_CS_PIN GPIO_PIN_2

/*
 * rom pins
 */
// ROM out
#define LCD12864_ROM_OUT_PORT GPIO_PORTG_BASE
#define LCD12864_ROM_OUT_PIN GPIO_PIN_2
// ROM In
#define LCD12864_ROM_IN_PORT GPIO_PORTG_BASE
#define LCD12864_ROM_IN_PIN GPIO_PIN_3
// ROM SCK
#define LCD12864_ROM_SCK_PORT GPIO_PORTG_BASE
#define LCD12864_ROM_SCK_PIN GPIO_PIN_1
// ROM CS
#define LCD12864_ROM_CS_PORT GPIO_PORTG_BASE
#define LCD12864_ROM_CS_PIN GPIO_PIN_0

#define LCD12864_CLR_SCLK ROM_GPIOPinWrite(LCD12864_SCLK_PORT, LCD12864_SCLK_PIN, ~LCD12864_SCLK_PIN)
#define LCD12864_SET_SCLK ROM_GPIOPinWrite(LCD12864_SCLK_PORT, LCD12864_SCLK_PIN, LCD12864_SCLK_PIN)

#define LCD12864_CLR_SDA ROM_GPIOPinWrite(LCD12864_SDA_PORT, LCD12864_SDA_PIN, ~LCD12864_SDA_PIN)
#define LCD12864_SET_SDA ROM_GPIOPinWrite(LCD12864_SDA_PORT, LCD12864_SDA_PIN, LCD12864_SDA_PIN)

#define LCD12864_CLR_RS ROM_GPIOPinWrite(LCD12864_RS_PORT, LCD12864_RS_PIN, ~LCD12864_RS_PIN)
#define LCD12864_SET_RS ROM_GPIOPinWrite(LCD12864_RS_PORT, LCD12864_RS_PIN, LCD12864_RS_PIN)

#define LCD12864_CLR_RST ROM_GPIOPinWrite(LCD12864_RST_PORT, LCD12864_RST_PIN, ~LCD12864_RST_PIN)
#define LCD12864_SET_RST ROM_GPIOPinWrite(LCD12864_RST_PORT, LCD12864_RST_PIN, LCD12864_RST_PIN)

#define LCD12864_CLR_CS ROM_GPIOPinWrite(LCD12864_CS_PORT, LCD12864_CS_PIN, ~LCD12864_CS_PIN)
#define LCD12864_SET_CS ROM_GPIOPinWrite(LCD12864_CS_PORT, LCD12864_CS_PIN, LCD12864_CS_PIN)

#define LCD12864_CLR_ROM_IN ROM_GPIOPinWrite(LCD12864_ROM_IN_PORT, LCD12864_ROM_IN_PIN, ~LCD12864_ROM_IN_PIN)
#define LCD12864_SET_ROM_IN ROM_GPIOPinWrite(LCD12864_ROM_IN_PORT, LCD12864_ROM_IN_PIN, LCD12864_ROM_IN_PIN)

#define LCD12864_CLR_ROM_SCK ROM_GPIOPinWrite(LCD12864_ROM_SCK_PORT, LCD12864_ROM_SCK_PIN, ~LCD12864_ROM_SCK_PIN)
#define LCD12864_SET_ROM_SCK ROM_GPIOPinWrite(LCD12864_ROM_SCK_PORT, LCD12864_ROM_SCK_PIN, LCD12864_ROM_SCK_PIN)

#define LCD12864_CLR_ROM_CS ROM_GPIOPinWrite(LCD12864_ROM_CS_PORT, LCD12864_ROM_CS_PIN, ~LCD12864_ROM_CS_PIN)
#define LCD12864_SET_ROM_CS ROM_GPIOPinWrite(LCD12864_ROM_CS_PORT, LCD12864_ROM_CS_PIN, LCD12864_ROM_CS_PIN)

#define LCD12864_GET_BIT_ROM_OUT ROM_GPIOPinRead(LCD12864_ROM_OUT_PORT, LCD12864_ROM_OUT_PIN)

void lcd12864_init(void);
void DispClear();
void DispClear_line(uint8_t x, uint8_t y, uint8_t len);

void Display_128x64(uint8_t dp[1024]);
void Disp_GB2312_string_16x16(uint8_t x, uint8_t y, uint8_t *text, bool style);

void DispChar5x8(uint8_t x, uint8_t y, char str, bool style);
void DispString5x8(uint8_t x, uint8_t y, char *str, bool style);

// ��ʾ�����ַ�
void DispExtern16x16(uint8_t x, uint8_t y, uint8_t *str, bool style);
// ��ʾС����
void DispNumber5x8(uint8_t page, uint8_t column, uint32_t data, uint8_t len, bool style);
void DispNumber8x16(uint8_t page, uint8_t column, uint32_t data, uint8_t len, bool style);

void DispFloat5x8(uint8_t page, uint8_t column, float dat, bool style, uint8_t len1, uint8_t len2);
void DispFloat8x16(uint8_t page, uint8_t column, float dat, bool style, uint8_t len1, uint8_t len2);

// ������ͼ
void display_vertical_line(uint8_t y, uint8_t len, bool style);
void DispLineChart(uint8_t x, uint8_t y, char *data, bool style);

void DispFloat(int row, int col, float data, int len1, int len2);

#endif /* LCD12864_ROM_ENABLE_H_ */
