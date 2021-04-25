/**************************************************************  
 * This file is part of the project
 * Copyright (c) 2020 Alexander Krapivniy (a.krapivniy@gmail.com)
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************/

#include "ssd1306.h"
#include "stm8l.h"
#include "i2c.h"
#include "fonts.h"

#define ssd1306_font_width  8
#define ssd1306_font_height  8

/* internal declarations: */
#define SSD_COMMAND_MODE      0x00
#define SSD_DATA_MODE         0x40
#define SSD_INVERSE_DISPLAY   0xA7
#define SSD_DISPLAY_OFF       0xAE
#define SSD_DISPLAY_ON        0xAF
#define SSD_SET_CONTRAST      0x81
#define SSD_EXTERNAL_VCC      0x01
#define SSD_INTERNAL_VCC      0x02
#define SSD_DEACTIVATE_SCROLL 0x2E

#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON        0xA5
#define SSD1306_NORMAL_DISPLAY      0xA6

#define SSD1306_SETDISPLAYOFFSET    0xD3
#define SSD1306_SETCOMPINS          0xDA
#define SSD1306_SETVCOMDETECT       0xDB
#define SSD1306_SETDISPLAYCLOCKDIV  0xD5
#define SSD1306_SETPRECHARGE        0xD9
#define SSD1306_SETMULTIPLEX        0xA8
#define SSD1306_SETLOWCOLUMN        0x00
#define SSD1306_SETHIGHCOLUMN       0x10
#define SSD1306_SETSTARTLINE        0x40
#define SSD1306_MEMORYMODE          0x20
#define SSD1306_COMSCANINC          0xC0
#define SSD1306_COMSCANDEC          0xC8
#define SSD1306_SEGREMAP            0xA0
#define SSD1306_CHARGEPUMP          0x8D

#define SSD1306_WIDTH  128
#define SSD1306_HEIGHT  32
#define SSD1306_BUFFER_SIZE    (SSD1306_WIDTH * SSD1306_HEIGHT / 8)

static uint8_t ssd1306_buffer[SSD1306_BUFFER_SIZE];

void ssd1306_update(uint8_t part);
void ssd1306_clear();

static void ssd1306_cmd1(uint8_t c)
{
	i2c_write_addr8_data8(SSD1306_I2CADDR, SSD_COMMAND_MODE, c);
}

static void ssd1306_cmd2(uint8_t c0, uint8_t c1)
{
	uint8_t buff[3] = {SSD_COMMAND_MODE, c0, c1};
	i2c_write_data(SSD1306_I2CADDR, buff, 3);
}

static void ssd1306_cmd3(uint8_t c0, uint8_t c1, uint8_t c2)
{
	uint8_t buff[4] = {SSD_COMMAND_MODE, c0, c1, c2};
	i2c_write_data(SSD1306_I2CADDR, buff, 4);
}

static void ssd1306_data(uint8_t *buf, int size)
{
	i2c_write_addr8_data(SSD1306_I2CADDR, SSD_DATA_MODE, buf, size);
}

#ifndef SSD1306_CONTRAST

void ssd1306_set_contrast(uint8_t contrast)
{
	ssd1306_cmd2(SSD_SET_CONTRAST, contrast);
}
#endif

/* public functions: */
void ssd1306_init(uint8_t contrast)
{
	ssd1306_cmd1(SSD_DISPLAY_OFF);
	ssd1306_cmd2(SSD1306_SETDISPLAYCLOCKDIV, 0x80);
	ssd1306_cmd2(SSD1306_SETDISPLAYOFFSET, 0x00);
	ssd1306_cmd1(SSD1306_SETSTARTLINE);
	ssd1306_cmd2(SSD1306_CHARGEPUMP, 0x14);
	ssd1306_cmd2(SSD1306_MEMORYMODE, 0x00);
#ifdef SSD1306_ROTATE180
	ssd1306_cmd1(SSD1306_SEGREMAP);
	ssd1306_cmd1(SSD1306_COMSCANINC);
#else
	ssd1306_cmd1(SSD1306_SEGREMAP | 0x1);
	ssd1306_cmd1(SSD1306_COMSCANDEC);
#endif

#ifdef SSD1306_TYPE32 
	ssd1306_cmd2(SSD1306_SETCOMPINS, 0x02);
	ssd1306_cmd3(0x22, 0, 3); // pages for 32 lines
	ssd1306_cmd2(SSD1306_SETMULTIPLEX, 32 - 1); //lines number
#else
	ssd1306_cmd2(SSD1306_SETCOMPINS, 0x12);
	ssd1306_cmd3(0x22, 0, 7); // pages for 64 lines
	ssd1306_cmd2(SSD1306_SETMULTIPLEX, 64 - 1); //lines number
#endif
	ssd1306_cmd2(SSD_SET_CONTRAST, contrast);

	ssd1306_cmd2(SSD1306_SETPRECHARGE, 0xF1); //
	ssd1306_cmd2(SSD1306_SETVCOMDETECT, 0x40);

	ssd1306_cmd2(SSD1306_SETVCOMDETECT, 0x40);

	ssd1306_cmd1(SSD1306_DISPLAYALLON_RESUME);
	ssd1306_cmd1(SSD1306_NORMAL_DISPLAY);

	ssd1306_cmd3(0x21, 0, 127);
	ssd1306_cmd1(SSD_DEACTIVATE_SCROLL);

	ssd1306_clear();
	ssd1306_update(0);
#ifndef SSD1306_TYPE32  
	ssd1306_update(1);
#endif

	ssd1306_cmd1(SSD_DISPLAY_ON);
}

void ssd1306_sleep()
{
	ssd1306_cmd1(SSD_DISPLAY_OFF);
}

void ssd1306_update(uint8_t part)
{
	uint8_t i;
	uint8_t *p = ssd1306_buffer;

#ifndef SSD1306_TYPE32  
	ssd1306_cmd3(0x22, 0 | (part << 2), 3 | (part << 2)); // pages for 64 lines
#else
	(void) part;
#endif
	for (i = 0; i < SSD1306_HEIGHT; i++) {
		ssd1306_data(p, (SSD1306_WIDTH / 8));
		p += (SSD1306_WIDTH / 8);
	}
}

void bzero(uint8_t *dst, uint16_t size)
{
	while (size--)
		dst[size] = 0;
}

void ssd1306_clear()
{
	bzero(ssd1306_buffer, SSD1306_BUFFER_SIZE);
}

void ssd1306_pixel(uint8_t x, uint8_t y, uint8_t color)
{
	uint8_t *p = ssd1306_buffer + (x + (y / 8) * SSD1306_WIDTH);
	if (color)
		*p |= 1 << (y & 0x7);
	else
		*p &= ~(1 << (y & 0x7));
}

void ssd1306_char(uint8_t x, uint8_t y, uint8_t c)
{
	const unsigned char *column;
	uint8_t xoffset, yoffset;
	uint8_t bit;

	column = ssd1306_font_table + ((c - 32) * ssd1306_font_width);
	// Render each column
	for (xoffset = 0; xoffset < ssd1306_font_width; xoffset++) {
		for (yoffset = 0; yoffset < (ssd1306_font_height + 1); yoffset++) {
			bit = column[xoffset] & (1 << yoffset); // Shift current row bit left
			ssd1306_pixel(x + xoffset, y + yoffset, bit);
		}
	}
}

uint8_t strlen(const char *str)
{
	const char *s;
	for (s = str; *s; ++s);
	return(s - str);
}

void ssd1306_string(uint8_t x, uint8_t y, const char *text)
{
	uint8_t i;
	uint8_t len = strlen(text);

	for (i = 0; i < len; i++) {
		ssd1306_char(x + (i * (ssd1306_font_width + 1)), y, text[i]);
	}
}

void ssd1306_clean_string(uint8_t y)
{
	// Render each column
	uint8_t yoffset;
	for (yoffset = y; yoffset < (y + ssd1306_font_height + 1); yoffset++) {
		bzero(ssd1306_buffer + ((yoffset / 8) * SSD1306_WIDTH), SSD1306_WIDTH / 8);
	}
}

void ssd1306_new_string(uint8_t x, uint8_t y, const char *text)
{
	ssd1306_clean_string(y);
	ssd1306_string(x, y, text);
}

#ifdef LINK_UNUSED
char const hex_chars[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

void ssd1306_hex(uint8_t x, uint8_t y, uint8_t value)
{
	ssd1306_char(x + (0 * (ssd1306_font_width + 1)), y, hex_chars[ (value >> 4) ]);
	ssd1306_char(x + (1 * (ssd1306_font_width + 1)), y, hex_chars[ (value & 0x0F) ]);
}
#endif