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

#include <stm8l.h>
#include "nrf24.h"
#include "i2c.h"
#include "ssd1306.h"


uint8_t dir_status_timer = 0;
uint8_t servo_timer = 0;
uint16_t button_timer = 0;

void gpio_init()
{
	// Configure pins
	PA_DDR = 0x00;
	PA_CR1 = 0x0c;
	PA_CR2 = 0x0c;

	PB_DDR = 0x78;
	PB_CR1 = 0xFB;
	PB_CR2 = 0x7B;

	PC_DDR = 0x03;
	PC_CR1 = 0x73;
	PC_CR2 = 0x73;
	PC_ODR = 0x03;

	PD_DDR = 0x00;
	PD_CR1 = 0x01;
	PD_CR2 = 0x01;
}

uint8_t itoa(int16_t i, char b[])
{
	char* p = b;
	uint8_t size = 0;
	int16_t shifter = i;

	if (i < 0) {
		*p++ = '-';
		i *= -1;
		size++;
	}
	do { //Move to where representation ends
		++p;
		shifter = shifter / 10;
	} while (shifter);
	*p = '\0';
	do { //Move back, inserting digits as u go
		*--p = '0' + (i % 10);
		i = i / 10;
		size++;
	} while (i);
	return size;
}

char const hex_chars[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

void itoa_hex(uint8_t i, char b[])
{
	b[0] = hex_chars[ (i >> 4) ];
	b[1] = hex_chars[ (i & 0x0F) ];
	b[2] = 0;
}

uint8_t check_button = 0;

void tim4_init()
{
	set_bit(CLK_PCKENR1, CLK_PCKENR1_TIM4);

	TIM4_PSCR = 0x07; // Prescaler 
	TIM4_ARR = 0xff; // TIM4 overflow 

	set_bit(TIM4_IER, TIM4_IER_UIE); // Update interrupt enable
	TIM4_CR1 = 0x81;
	set_bit(TIM4_EGR, TIM4_EGR_UG); // Generate UEV (update event) to reload TIM4 and set the prescaler
}

void exti_init()
{
	EXTI_CR1 = 0xff;
	EXTI_CR2 = 0xff;
}

char *strinsert(char *dst, char *src, uint8_t pos)
{
	dst += pos;

	while (*src) {
		*dst = *src;
		dst++;
		src++;
	}
	return dst;
}


#define KEY_BACK 0x02
#define KEY_FRWD 0x80
#define KEY_LEFT 0x08
#define KEY_RIGHT 0x01
#define KEY_SERVO_DOWN 0x10
#define KEY_SERVO_UP 0x40
#define KEY_MODE 0x04

#define LEFT_FRWD(speed) (speed)
#define LEFT_BACK(speed) (0x04 | speed)
#define RIGHT_FRWD(speed) (speed<<3)
#define RIGHT_BACK(speed) (0x20 | (speed<<3))
#define CMD_SERVO(pos) (pos<<6)


#define RADIO_CHANNEL 80

int main()
{
	char disp_line0[25] = "Radio:       ";
	char disp_line1[25] = "Front:big    ";
	uint8_t servo_pos = 2;
	uint8_t command = CMD_SERVO(servo_pos);
	uint8_t zero = 0;
	uint8_t button = 0;
	uint8_t old_button = 0;
	int8_t tx_status = 0;
	uint8_t dir_status = 1;
	uint8_t radio_status = 0;
	uint8_t display_update = 0;
	uint8_t resend_button = 0;

	uint8_t channel = RADIO_CHANNEL;

	clear_bit(CLK_PCKENR2, CLK_PCKENR2_BROM);
	disable_interrupts();
	stm8_speed(0);

	gpio_init();
	exti_init();
	tim4_init();

	i2c_init();

	ssd1306_init(1);

	nrf24_init();
	if (nrf24_check()) {
		strinsert(disp_line0, "broken", 6);
		radio_status = 1;
	} else {
		strinsert(disp_line0, "ready ", 6);
		radio_status = 0;
	};

	ssd1306_new_string(1, 1, disp_line0);
	ssd1306_new_string(1, 11, disp_line1);
	ssd1306_update(0);
	enable_interrupts();

	nrf24_mode_tx(10, 0, channel, nrf24_speed_250k, nrf24_txpower_0dbm, nrf24_crc_2b, 0x1200 | RADIO_CHANNEL);

	do {
		nrf24_write_done();

		button = ~((PB_IDR & 0x03) | (PA_IDR & 0x0c) | (PC_IDR & 0x70) | (PD_IDR << 7));


		if (button_timer == 0) {
			button_timer = 10000;
			nrf24_write(&command, 1);
		}

		if (button == old_button) {
			continue;
		}
		old_button = button;
		command = 0;
		button_timer = 10000;

		if (button & KEY_FRWD) {
			if (button & KEY_LEFT)
				command = RIGHT_FRWD(3) | LEFT_FRWD(2);
			else if (button & KEY_RIGHT)
				command = RIGHT_FRWD(2) | LEFT_FRWD(3);
			else command = RIGHT_FRWD(3) | LEFT_FRWD(3);
		} else if (button & KEY_BACK) {
			if (button & KEY_LEFT)
				command = RIGHT_BACK(3) | LEFT_BACK(2);
			else if (button & KEY_RIGHT)
				command = RIGHT_BACK(2) | LEFT_BACK(3);
			else command = RIGHT_BACK(3) | LEFT_BACK(3);
		} else {
			if (button & KEY_LEFT)
				command = RIGHT_FRWD(1) | LEFT_BACK(1);
			else if (button & KEY_RIGHT)
				command = RIGHT_BACK(1) | LEFT_FRWD(1);
		}
		if (dir_status & 1)
			command ^= 0x24;

		if ((button & KEY_SERVO_UP) && (servo_timer == 0)) {
			if (servo_pos < 3) servo_pos++;
			servo_timer = 100;
		}
		if ((button & KEY_SERVO_DOWN) && (servo_timer == 0)) {
			if (servo_pos > 0) servo_pos--;
			servo_timer = 100;
		}
		command |= CMD_SERVO(servo_pos);

		nrf24_write(&command, 1);

		if (button & KEY_MODE) {
			if (!dir_status_timer) {
				dir_status++;
				if (dir_status & 1) {
					strinsert(disp_line1, "big  ", 6);
				} else {
					strinsert(disp_line1, "small", 6);
				}
				ssd1306_new_string(1, 11, disp_line1);
				dir_status_timer = 100;
				display_update = 1;
			}
		}

		if (display_update) {
			display_update = 0;
			ssd1306_update(0);
		}

	} while (1);
}

void exti0_isr(void) __interrupt(EXTI0_ISR)
{
	set_bit(EXTI_SR1, 0);
}

void exti1_isr(void) __interrupt(EXTI1_ISR)
{
	set_bit(EXTI_SR1, 1);
}

void exti2_isr(void) __interrupt(EXTI2_ISR)
{
	set_bit(EXTI_SR1, 2);
}

void exti3_isr(void) __interrupt(EXTI3_ISR)
{
	set_bit(EXTI_SR1, 3);
}

void exti4_isr(void) __interrupt(EXTI4_ISR)
{
	set_bit(EXTI_SR1, 4);

}

void exti5_isr(void) __interrupt(EXTI5_ISR)
{
	set_bit(EXTI_SR1, 5);
}

void exti6_isr(void) __interrupt(EXTI6_ISR)
{
	set_bit(EXTI_SR1, 6);

}

void tim4_isr(void) __interrupt(TIM4_ISR)
{
	if (dir_status_timer) dir_status_timer--;
	if (servo_timer) servo_timer--;
	if (button_timer) button_timer--;

	clear_bit(TIM4_SR, TIM4_SR_UIF);
}
