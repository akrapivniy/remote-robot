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

#ifndef __SSD1306_H__
#define __SSD1306_H__


#include <stdint.h>

#define SSD1306_I2CADDR  (0x3c<<1)
#define SSD1306_TYPE32 
#define SSD1306_CONTRAST 1


void ssd1306_init(uint8_t contrast);
void ssd1306_sleep();
void ssd1306_string(uint8_t x, uint8_t y, const char *text);
void ssd1306_new_string(uint8_t x, uint8_t y, const char *text);
void ssd1306_char(uint8_t x, uint8_t y, uint8_t c);
void ssd1306_line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void ssd1306_update(uint8_t part);
void ssd1306_clear();
void ssd1306_hex(uint8_t x, uint8_t y, uint8_t value);


#endif /* __SSD1306_H__ */

