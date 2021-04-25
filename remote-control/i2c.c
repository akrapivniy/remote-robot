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
#include "i2c.h"

void i2c_init()
{
	set_bit(CLK_PCKENR1, CLK_PCKENR1_I2C); // Enable I2C peripherial (PCKEN13)

	clear_bit(I2C1_CR1, I2C1_CR1_PE); // Disable I2C to configure TRISER
	I2C1_FREQR = 0x02; // 2MHz peripherial clock frequency (WTF is this?)
	I2C1_CCRH = 0x00; // Standard mode I2C, clear CCRH
	I2C1_CCRL = 0x00; // Clear CCRL

	I2C1_TRISER = 0x03; // Maximum rise time: = [1000ns/(1/input_clock.10e6)]+1 (3 for 2MHz CPU)


	I2C1_CCRH = 0x00;
	I2C1_CCRL = 0x0A; // 100kHz I2C on 2MHz CPU

	set_bit(I2C1_CR1, I2C1_CR1_PE); // Enable I2C
	set_bit(I2C1_CR2, I2C1_CR2_ACK); // Acknowledge enable
	I2C1_OARL = 0x00; // Own 7-bit address = 0 (don't care in master mode)
	I2C1_OARH = 0x00; // 7-bit addressing mode
	set_bit(I2C1_OARH, I2C1_OARH_ADDCONF); // ADDCONF bit must always be written as '1'

	return;
}


// Enable or disable I2C acknowledge feature

inline void i2c_set_ack(void)
{
	set_bit(I2C1_CR2, I2C1_CR2_ACK);
}

inline void i2c_set_ack_next(void)
{
	set_mask(I2C1_CR2, (1 << I2C1_CR2_ACK) | (1 << I2C1_CR2_POS));
}

inline void i2c_clear_ack(void)
{
	clear_mask(I2C1_CR2, (1 << I2C1_CR2_ACK) | (1 << I2C1_CR2_POS));
}

inline void i2c_send_start(void)
{
	set_bit(I2C1_CR2, I2C1_CR2_START);
}

inline void i2c_clear_start(void)
{
	clear_bit(I2C1_CR2, I2C1_CR2_START);
}

inline void i2c_send_stop(void)
{
	set_bit(I2C1_CR2, I2C1_CR2_STOP);
}

inline void i2c_clear_stop(void)
{
	clear_bit(I2C1_CR2, I2C1_CR2_STOP);
}

inline void i2c_send_addr_read(uint8_t addr)
{
	I2C1_DR = (addr) | 1;
}

inline void i2c_send_addr_write(uint8_t addr)
{
	I2C1_DR = addr;
}

int8_t i2c_waitfor(uint16_t event)
{
	uint16_t tm = 0xff;
	while ((((((uint16_t) I2C1_SR1) | (uint16_t) I2C1_SR3 << 8)) != event) && tm) tm--;
	if (tm == 0)
		return -1;
	else return 0;
}

inline void i2c_tx(uint8_t data)
{
	I2C1_DR = data;
}

inline uint8_t i2c_rx(void)
{
	return(I2C1_DR);
}

#define I2C_MASTER_MODE_SELECT               (0x0301)  // EV5: BUSY, MSL and SB flags
#define I2C_MASTER_TRANSMITTER_MODE_SELECTED (0x0782)  // EV6: BUSY, MSL, ADDR, TXE and TRA flags
#define I2C_MASTER_RECEIVER_MODE_SELECTED    (0x0302)  // EV6: BUSY, MSL and ADDR flags
#define I2C_MASTER_BYTE_RECEIVED             (0x0340)  // EV7: BUSY, MSL and RXNE flags
#define I2C_MASTER_BYTE_TRANSMITTING         (0x0780)  // EV8: TRA, BUSY, MSL, TXE flags
#define I2C_MASTER_BYTE_TRANSMITTED          (0x0784)  // EV8_2: TRA, BUSY, MSL, TXE and BTF flags

#ifdef LINK_UNUSED

uint8_t i2c_read_addr8_data8(uint8_t addr, uint8_t reg_addr)
{
	uint8_t data;

	i2c_set_ack();
	i2c_send_start();
	i2c_waitfor(I2C_MASTER_MODE_SELECT);
	i2c_send_addr_write(addr);
	i2c_waitfor(I2C_MASTER_TRANSMITTER_MODE_SELECTED);
	i2c_tx(reg_addr);
	i2c_waitfor(I2C_MASTER_BYTE_TRANSMITTED);
	i2c_send_start();
	i2c_waitfor(I2C_MASTER_MODE_SELECT);
	i2c_send_addr_read(addr);
	i2c_waitfor(I2C_MASTER_RECEIVER_MODE_SELECTED);
	i2c_clear_ack();
	i2c_waitfor(I2C_MASTER_BYTE_RECEIVED);
	data = i2c_rx();
	i2c_send_stop();
	return data;
}

uint16_t i2c_read_addr8_data16(uint8_t addr, uint8_t reg_addr)
{
	uint16_t data;

	i2c_set_ack();
	i2c_send_start();
	i2c_waitfor(I2C_MASTER_MODE_SELECT);
	i2c_send_addr_write(addr);
	i2c_waitfor(I2C_MASTER_TRANSMITTER_MODE_SELECTED);

	i2c_tx(reg_addr);
	i2c_waitfor(I2C_MASTER_BYTE_TRANSMITTED);
	i2c_send_start();
	i2c_waitfor(I2C_MASTER_MODE_SELECT);
	i2c_send_addr_read(addr);
	i2c_waitfor(I2C_MASTER_RECEIVER_MODE_SELECTED);
	i2c_waitfor(I2C_MASTER_BYTE_RECEIVED);
	data = i2c_rx() << 8;
	i2c_clear_ack();
	i2c_waitfor(I2C_MASTER_BYTE_RECEIVED);
	data |= i2c_rx();
	i2c_send_stop();
	return data;
}
#endif

uint8_t i2c_read_data(uint8_t addr, uint8_t *data, uint8_t size)
{
	uint8_t i = 0;

	i2c_set_ack();
	i2c_send_start();
	i2c_waitfor(I2C_MASTER_MODE_SELECT);
	i2c_send_addr_read(addr);
	i2c_waitfor(I2C_MASTER_RECEIVER_MODE_SELECTED);

	do {
		if (size == 1) i2c_clear_ack();
		i2c_waitfor(I2C_MASTER_BYTE_RECEIVED);
		data[i] = i2c_rx();
		i++;
		size--;
	} while (size);

	i2c_send_stop();
	return i;
}

int8_t i2c_write_addr8_data8(uint8_t addr, uint8_t reg_addr, uint8_t data)
{
	i2c_set_ack();
	i2c_send_start();
	i2c_waitfor(I2C_MASTER_MODE_SELECT);
	i2c_send_addr_write(addr);
	i2c_waitfor(I2C_MASTER_TRANSMITTER_MODE_SELECTED);

	i2c_tx(reg_addr);
	i2c_waitfor(I2C_MASTER_BYTE_TRANSMITTED);

	i2c_tx(data);
	i2c_waitfor(I2C_MASTER_BYTE_TRANSMITTED);
	i2c_send_stop();
	return(0);
}

int8_t i2c_write_data(uint8_t addr, uint8_t *data, uint8_t size)
{
	int i;

	i2c_set_ack();
	i2c_send_start();
	i2c_waitfor(I2C_MASTER_MODE_SELECT);
	i2c_send_addr_write(addr);
	i2c_waitfor(I2C_MASTER_TRANSMITTER_MODE_SELECTED);

	for (i = 0; i < size; i++) {
		i2c_tx(data[i]);
		i2c_waitfor(I2C_MASTER_BYTE_TRANSMITTED);
	}

	i2c_send_stop();

	return(0);
}

int8_t i2c_write_addr8_data(uint8_t addr, uint8_t reg_addr, uint8_t *data, uint8_t size)
{
	int i;

	i2c_set_ack();
	i2c_send_start();
	i2c_waitfor(I2C_MASTER_MODE_SELECT);
	i2c_send_addr_write(addr);
	i2c_waitfor(I2C_MASTER_TRANSMITTER_MODE_SELECTED);

	i2c_tx(reg_addr);
	i2c_waitfor(I2C_MASTER_BYTE_TRANSMITTED);

	for (i = 0; i < size; i++) {
		i2c_tx(data[i]);
		i2c_waitfor(I2C_MASTER_BYTE_TRANSMITTED);
	}
	i2c_send_stop();
	return(0);
}

int8_t i2c_ping(uint8_t addr)
{
	int8_t ret = 0;

	i2c_set_ack();
	i2c_send_start();
	i2c_waitfor(I2C_MASTER_MODE_SELECT);
	i2c_send_addr_write(addr);
	ret = i2c_waitfor(I2C_MASTER_TRANSMITTER_MODE_SELECTED);
	i2c_send_stop();
	return ret;
}
