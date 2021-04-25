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

#include "stm8l.h"
#include "nrf24.h"

#define SERVO_MIN 1000
#define SERVO_MAX 2500

void gpio_init()
{
	// Configure pins
	PA_DDR = 0x0c;
	PA_CR1 = 0x0c;
	PA_CR2 = 0x0c;

	PB_DDR = 0x77;
	PB_CR1 = 0xF7;
	PB_CR2 = 0x77;

	PC_DDR = 0x53;
	PC_CR1 = 0x53;
	PC_CR2 = 0x53;

	PD_DDR = 0x01;
	PD_CR1 = 0x01;
	PD_CR2 = 0x01;
}

void set_servo(uint16_t value)
{
	if (value > SERVO_MAX) value = SERVO_MAX;
	if (value < SERVO_MIN) value = SERVO_MIN;
	TIM3_CCR1H = (uint16_t) value >> 8;
	TIM3_CCR1L = (uint16_t) value & 0xff;
}

void servo_tim3_init()
{
	set_bit(CLK_PCKENR1, CLK_PCKENR1_TIM3);
	TIM3_PSCR = 0x01; //prescaler to 1MHZ   4 for 8mhz  8 for 16mhz 

	TIM3_ARRH = 19999 >> 8; // 20000 uS = 50 hz 
	TIM3_ARRL = 19999 & 0xff;

	TIM3_CCMR1 = 0x68; // PWM mode 1, use preload register 
	TIM3_CCER1 = 0x01; // output enable, normal polarity 

	TIM3_CCR1H = 0;
	TIM3_CCR1L = 0;

	TIM3_BKR = 0x80;
	TIM3_CR1 = 0x81;
}

void servo_enable()
{
	PC_ODR |= 0x10;
}

void servo_disable()
{
	PC_ODR &= ~0x10;
}

void set_motor_left(uint8_t value)
{
	TIM2_CCR1L = value & 0xff;
}

void set_motor_right(uint8_t value)
{
	TIM2_CCR2L = value & 0xff;
}

void motor_tim2_init()
{
	set_bit(CLK_PCKENR1, CLK_PCKENR1_TIM2);

	TIM2_PSCR = 0x01; //prescaler to 1MHZ   4 for 8mhz  8 for 16mhz 

	TIM2_ARRH = 0; // 200 uS = 5000 hz 
	TIM2_ARRL = 199;

	TIM2_CCMR1 = 0x68; // PWM mode 1, use preload register 
	TIM2_CCMR2 = 0x68; // PWM mode 1, use preload register 
	TIM2_CCER1 = 0x11; // output enable, normal polarity 

	TIM2_CCR1H = 0;
	TIM2_CCR1L = 0;
	TIM2_CCR2H = 0;
	TIM2_CCR2L = 0;

	TIM2_BKR = 0x80;
	TIM2_CR1 = 0x81;

}

void set_motor(int8_t left, int8_t right)
{
	if (left > 0) {
		set_bits(PA_ODR, 0x0c, 0x04);
		left *= 2;
	} else if (left < 0) {
		set_bits(PA_ODR, 0x0c, 0x08);
		left = (-left) *2;
	} else if (left == 0) {
		set_bits(PA_ODR, 0x0c, 0x04);
	}

	if (right > 0) {
		set_bits(PC_ODR, 0x03, 0x01);
		right *= 2;
	} else if (right < 0) {
		set_bits(PC_ODR, 0x03, 0x02);
		right = -right * 2;
	} else if (right == 0) {
		clear_mask(PC_ODR, 0x03);
	}

	set_motor_left(left);
	set_motor_right(right);
}



#define SOFT_MAX 20
#define SOFT_STEP (256/SOFT_MAX)

struct soft_motor {
	int8_t values[SOFT_MAX];
	int8_t current;
	uint8_t ptr;
};


struct soft_motor lm;
struct soft_motor rm;

inline void soft_init()
{
	lm.current = 0;
	lm.ptr = SOFT_MAX;
	rm.current = 0;
	rm.ptr = SOFT_MAX;
}

inline void soft_fill_motor(int8_t value, struct soft_motor *motor)
{
	int8_t i, dir = 0, step = 0;


	if (value > motor->current) {
		dir = 1;
		step = (value - motor->current) / SOFT_STEP;
	} else if (value < motor->current) {
		dir = -1;
		step = (motor->current - value) / SOFT_STEP;
	}

	motor->values[0] = (motor->current);
	for (i = 1; i < SOFT_MAX; i++, step--) {
		if (step >= 1)
			motor->values[i] = motor->values[i - 1] + (SOFT_STEP * dir);
		else
			motor->values[i] = value;
	}
	motor->values[SOFT_MAX - 1] = value;
}

void soft_set_motors(int8_t left, int8_t right)
{
	soft_fill_motor(left, &lm);
	soft_fill_motor(right, &rm);
	lm.ptr = 0;
	rm.ptr = 0;
}

void soft_tick()
{
	if (lm.ptr < SOFT_MAX) {
		lm.current = lm.values[lm.ptr];
		lm.ptr++;
	}

	if (rm.ptr < SOFT_MAX) {
		rm.current = rm.values[rm.ptr];
		rm.ptr++;
	}
	set_motor(lm.current, rm.current);

}


uint16_t control_watchdog = 0;
uint16_t servo_timer = 0;

void tim4_init()
{
	set_bit(CLK_PCKENR1, CLK_PCKENR1_TIM4);

	TIM4_PSCR = 0x07; // Prescaler 
	TIM4_ARR = 0xff; // TIM4 overflow 

	set_bit(TIM4_IER, TIM4_IER_UIE); // Update interrupt enable
	TIM4_CR1 = 0x81;
	set_bit(TIM4_EGR, TIM4_EGR_UG); // Generate UEV (update event) to reload TIM4 and set the prescaler
}


#define RADIO_CHANNEL 80

int main()
{
	int8_t ret;
	uint8_t packet;
	uint8_t channel = RADIO_CHANNEL;

	int8_t lmotor = 0, rmotor = 0;
	int16_t servo = SERVO_MIN;
	int16_t cservo = servo;

	clear_bit(CLK_PCKENR2, CLK_PCKENR2_BROM);
	disable_interrupts();

	soft_init();
	gpio_init();
	PA_ODR |= 0x04;

	servo_tim3_init();
	motor_tim2_init();
	nrf24_init();
	if (nrf24_check())
		PA_ODR &= ~0x04;

	enable_interrupts();
	tim4_init();
	nrf24_mode_rx(1, 1, 1, channel, nrf24_speed_250k, nrf24_crc_2b, 0x1200 | RADIO_CHANNEL, nrf24_txpower_0dbm);

	set_motor(lmotor, rmotor);
	set_servo(servo);
	control_watchdog = 0xff;

	while (1) {
		if (control_watchdog == 0) {
			control_watchdog = 0xff;
			nrf24_mode_rx(1, 1, 1, channel, nrf24_speed_250k, nrf24_crc_2b, 0x1200 | RADIO_CHANNEL, nrf24_txpower_0dbm);
			soft_set_motors(0, 0);
			servo_disable();
		}
		if (servo_timer == 0)
			servo_disable();
		packet = 0;
		ret = nrf24_read(&packet, 1);
		if (ret != 1) continue;
		control_watchdog = 0xff;

		switch (packet & 0x03) {
		case 0: lmotor = 0;
			break;
		case 1: lmotor = 50;
			break;
		case 2: lmotor = 70;
			break;
		case 3: lmotor = 127;
			break;
		}
		if (packet & 0x04)
			lmotor = -lmotor;

		packet >>= 3;

		switch (packet & 0x03) {
		case 0: rmotor = 0;
			break;
		case 1: rmotor = 50;
			break;
		case 2: rmotor = 70;
			break;
		case 3: rmotor = 127;
			break;
		}
		if (packet & 0x04)
			rmotor = -rmotor;
		soft_set_motors(lmotor, rmotor);

		packet >>= 3;
		switch (packet & 0x03) {
		case 0: servo = SERVO_MIN;
			break;
		case 1: servo = SERVO_MIN + (SERVO_MAX - SERVO_MIN) / 3;
			break;
		case 2: servo = SERVO_MIN + (SERVO_MAX - SERVO_MIN) / 3 * 2;
			break;
		case 3: servo = SERVO_MAX;
			break;
		}

		if (cservo != servo) {
			servo_enable();
			set_servo(servo);
			servo_timer = 80;
			cservo = servo;
		}

	};
}

void tim4_isr(void) __interrupt(TIM4_ISR)
{
	if (control_watchdog) control_watchdog--;
	if (servo_timer) servo_timer--;
	soft_tick();
	clear_bit(TIM4_SR, TIM4_SR_UIF);
}
