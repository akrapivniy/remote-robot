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


uint8_t nrf24_status_reg = 0;

inline void tsleep(uint16_t tick)
{
	while (tick--) __asm__("nop");
}

uint8_t spi1_rw(uint8_t data)
{
	uint8_t rcv;

	SPI1_DR = data; // Send byte to SPI (TXE cleared)
	while (!check_bit(SPI1_SR, SPI1_SR_RXNE)); // Wait until byte is received
	rcv = SPI1_DR; // Read received byte (RXNE cleared)
	while (!check_bit(SPI1_SR, SPI1_SR_TXE)); // Wait for TXE flag --> transmit buffer is empty
	while (check_bit(SPI1_SR, SPI1_SR_BSY)); // Wait until the transmission is complete

	return rcv;
}

void nrf24_write_reg(uint8_t reg, uint8_t value)
{
	CHIP_SELECT();
	nrf24_status_reg = spi1_rw(nRF24_CMD_WREG | reg);
	spi1_rw(value);
	while (check_bit(SPI1_SR, SPI1_SR_BSY));
	CHIP_UNSELECT();
}

uint8_t nrf24_read_reg(uint8_t reg)
{
	uint8_t value;

	CHIP_SELECT();
	nrf24_status_reg = spi1_rw(reg & 0x1f); // Select register to read from
	value = spi1_rw(nRF24_CMD_NOP); // Read register value
	while (check_bit(SPI1_SR, SPI1_SR_BSY)); // Wait until the transmission is complete
	CHIP_UNSELECT();

	return value;
}

uint8_t nrf24_read_reg_status()
{
	uint8_t value;

	CHIP_SELECT();
	value = spi1_rw(0xff); // Select register to read from
	while (check_bit(SPI1_SR, SPI1_SR_BSY)); // Wait until the transmission is complete
	CHIP_UNSELECT();

	return value;
}

void nrf24_read_buf(uint8_t reg, uint8_t *buffer, uint8_t count)
{
	CHIP_SELECT();
	spi1_rw(reg); // Send buffer address
	SPI1_DR = reg; // Transmit first dummy byte (clears the TXE flag)
	while (--count) {
		while (!check_bit(SPI1_SR, SPI1_SR_TXE)); // Wait until TX buffer is empty
		while (!check_bit(SPI1_SR, SPI1_SR_RXNE)); // Wait while RX buffer is empty
		*buffer++ = SPI1_DR; // Read received byte into buffer (clears the RXNE flag)
		SPI1_DR = reg; // Transmit dummy byte
	}
	while (!check_bit(SPI1_SR, SPI1_SR_RXNE)); // Wait while RX buffer is empty
	*buffer++ = SPI1_DR; // Read last received byte
	while (!check_bit(SPI1_SR, SPI1_SR_TXE)); // Wait until TX buffer is empty
	while (check_bit(SPI1_SR, SPI1_SR_BSY)); // Wait until the transmission is complete
	tsleep(0x80);
	CHIP_UNSELECT();
	tsleep(0x80);

}

void nrf24_read_packet(uint8_t reg, uint8_t *buffer, uint8_t count)
{
	CHIP_SELECT();
	spi1_rw(reg); // Send buffer address
	SPI1_DR = reg; // Transmit first dummy byte (clears the TXE flag)
	while (--count) {
		while (!check_bit(SPI1_SR, SPI1_SR_TXE)); // Wait until TX buffer is empty
		while (!check_bit(SPI1_SR, SPI1_SR_RXNE)); // Wait while RX buffer is empty
		*buffer++ = SPI1_DR; // Read received byte into buffer (clears the RXNE flag)
		SPI1_DR = reg; // Transmit dummy byte
	}
	while (!check_bit(SPI1_SR, SPI1_SR_RXNE)); // Wait while RX buffer is empty
	*buffer++ = SPI1_DR; // Read last received byte
	while (!check_bit(SPI1_SR, SPI1_SR_TXE)); // Wait until TX buffer is empty
	while (check_bit(SPI1_SR, SPI1_SR_BSY)); // Wait until the transmission is complete
	tsleep(0x80);
	CHIP_UNSELECT();
	tsleep(0x80);

}

void nrf24_write_buf(uint8_t reg, uint8_t *buffer, uint8_t size)
{
	CHIP_SELECT();
	spi1_rw(nRF24_CMD_WREG | reg); // Send buffer address
	SPI1_DR = *buffer++; // Transmit first byte (clears the TXE the flag)
	while (--size) {
		while (!check_bit(SPI1_SR, SPI1_SR_TXE)); // Wait until TX buffer is empty
		while (!check_bit(SPI1_SR, SPI1_SR_RXNE)); // Wait while RX buffer is empty
		(void) SPI1_DR; // Clear the RXNE flag
		SPI1_DR = *buffer++; // Transmit byte
	}
	while (!check_bit(SPI1_SR, SPI1_SR_RXNE)); // Wait while RX buffer is empty
	(void) SPI1_DR; // Clear the RXNE flag
	while (!check_bit(SPI1_SR, SPI1_SR_TXE)); // Wait until TX buffer is empty
	while (check_bit(SPI1_SR, SPI1_SR_BSY)); // Wait until the transmission is complete
	tsleep(0x80);
	CHIP_UNSELECT();
	tsleep(0x80);
}

void nrf24_write_packet(uint8_t reg, uint8_t *buffer, uint8_t size)
{
	uint16_t d;

	CHIP_SELECT();
	for (d = 0xff; d; d--);
	spi1_rw(reg); // Send buffer address

	//	SPI1_DR = 0; // Transmit first byte (clears the TXE the flag)
	SPI1_DR = *buffer++; // Transmit byte

	while (--size) {
		while (!check_bit(SPI1_SR, SPI1_SR_TXE)); // Wait until TX buffer is empty
		while (!check_bit(SPI1_SR, SPI1_SR_RXNE)); // Wait while RX buffer is empty
		(void) SPI1_DR; // Clear the RXNE flag
		SPI1_DR = *buffer++; // Transmit byte
	}
	while (!check_bit(SPI1_SR, SPI1_SR_RXNE)); // Wait while RX buffer is empty
	(void) SPI1_DR; // Clear the RXNE flag
	while (!check_bit(SPI1_SR, SPI1_SR_TXE)); // Wait until TX buffer is empty
	while (check_bit(SPI1_SR, SPI1_SR_BSY)); // Wait until the transmission is complete
	tsleep(0x80);
	CHIP_UNSELECT();
	tsleep(0x80);


}

int8_t nrf24_check(void)
{
	uint8_t rxbuf[5];
	uint8_t *ptr = (uint8_t *) nRF24_TEST_ADDR;
	uint8_t i;

	nrf24_write_buf(nRF24_REG_TX_ADDR, ptr, 5); // Write fake TX address
	nrf24_read_buf(nRF24_REG_TX_ADDR, rxbuf, 5); // Read TX_ADDR register
	for (i = 0; i < 5; i++) if (rxbuf[i] != *ptr++) return -1;

	return 0;
}

void nrf24_set_rfchannel(uint8_t channel)
{
	nrf24_write_reg(nRF24_REG_RF_CH, channel);
}

void nrf24_set_txpower(uint8_t power)
{
	uint8_t reg;

	reg = nrf24_read_reg(nRF24_REG_RF_SETUP) & 0xf9;
	nrf24_write_reg(nRF24_REG_RF_SETUP, reg | power);
}

void nrf24_flush_tx(void)
{
	nrf24_write_reg(nRF24_CMD_FLUSH_TX, 0xFF);
}

void nrf24_flush_rx(void)
{
	nrf24_write_reg(nRF24_CMD_FLUSH_RX, 0xFF);
}

void nrf24_reset(void)
{
	nrf24_flush_rx();
	nrf24_flush_tx();
}

void nrf24_clear_intr(void)
{
	uint8_t reg;
	reg = nrf24_read_reg_status();
	nrf24_write_reg(nRF24_REG_STATUS, reg | 0x70);
}

int8_t nrf24_read(uint8_t *buffer, uint8_t size)
{
	uint8_t status;
	int8_t result = -1;

	status = nrf24_read_reg_status();
	result = ((status & 0x0e) >> 1);
	if (result < 6) {
		nrf24_read_packet(nRF24_CMD_R_RX_PAYLOAD, buffer, size);
		nrf24_write_reg(nRF24_REG_STATUS, status | 0x70);
		return result;
	}
	return 0;
}

void nrf24_status(uint8_t *r7, uint8_t *r8, uint8_t * r9)
{
	*r7 = 0xdd;
	*r8 = 0xdd;
	*r9 = 0xdd;
	*r7 = nrf24_read_reg(0x07);
	*r8 = nrf24_read_reg(0x08);
	*r9 = nrf24_read_reg(0x09);
}

// Put nRF24L01 in RX mode
// input:
//   PIPE - RX data pipe (nRF24_RX_PIPE[0..5])
//   PIPE_AA - auto acknowledgment for data pipe (nRF24_ENAA_P[0..5] or nRF24_ENAA_OFF)
//   RFChan - Frequency channel (0..127) (frequency = 2400 + RFChan [MHz])
//   DataRate - Set data rate (nRF24_DataRate_[250kbps,1Mbps,2Mbps])
//   CRCS - CRC encoding scheme (nRF24_CRC_[off | 1byte | 2byte])
//   RX_Addr - buffer with TX address
//   RX_Addr_Width - size of TX address (3..5 byte)
//   RX_PAYLOAD - receive buffer length
//   TXPower - RF output power for ACK packets (-18dBm, -12dBm, -6dBm, 0dBm)

void nrf24_mode_rx(uint8_t pipe, uint8_t pipe_size, uint8_t pipe_ack, uint8_t channel,
	enum nrf24_speed speed, enum nrf24_crc_mode crc, uint16_t addr,
	enum nrf24_txpower txpower)
{
	uint8_t rreg;

	CHIP_MODE_LO();
	nrf24_read_reg(nRF24_CMD_NOP); // Dummy read

	rreg = nrf24_read_reg(nRF24_REG_EN_AA);
	if (pipe_ack) {
		// Enable auto acknowledgment for given data pipe
		rreg |= (1 << (uint8_t) pipe);
	} else {
		// Disable auto acknowledgment for given data pipe
		rreg &= ~(1 << pipe);
	}
	nrf24_write_reg(nRF24_REG_EN_AA, rreg);

	rreg = nrf24_read_reg(nRF24_REG_EN_RXADDR);
	nrf24_write_reg(nRF24_REG_EN_RXADDR, rreg | (1 << (uint8_t) pipe)); // Enable given data pipe

	nrf24_write_reg(nRF24_REG_RX_PW(pipe), pipe_size); // Set RX payload length

	nrf24_write_reg(nRF24_REG_RF_SETUP, (uint8_t) speed | (uint8_t) txpower); // SETUP register
	nrf24_set_rfchannel(channel); // Set frequency channel 
	nrf24_write_reg(nRF24_REG_SETUP_AW, 0); // Set of address widths (common for all data pipes)
	nrf24_write_buf(nRF24_REG_RX_ADDR(pipe), (uint8_t *) & addr, 2); // Set static RX address for given data pipe
	nrf24_write_buf(nRF24_REG_TX_ADDR, (uint8_t *) & addr, 2); // Set static TX address

	nrf24_write_reg(nRF24_REG_CONFIG, (uint8_t) crc | 0x02 | 0x01); // Config register

	nrf24_flush_rx();
	nrf24_clear_intr();
	CHIP_MODE_HI();
}

int8_t nrf24_write(uint8_t *buffer, uint8_t size)
{
	uint8_t reg;
	uint16_t wait = nRF24_WAIT_TIMEOUT;

	// Release CE pin (in case if it still high)
	CHIP_MODE_LO();
	// Transfer data from specified buffer to the TX FIFO
	nrf24_write_packet(nRF24_CMD_W_TX_PAYLOAD, buffer, size);
	// CE pin high => Start transmit (must hold pin at least 10us)
	CHIP_MODE_HI();
	// Wait for IRQ from nRF24L01
	nrf24_wait_intr(wait);
	// Release CE pin
	CHIP_MODE_LO();
	if (!wait)
		return -1;

	//reg = nrf24_read_reg(nRF24_REG_STATUS);
	reg = nrf24_read_reg_status();
	nrf24_write_reg(nRF24_REG_STATUS, reg | 0x70);

	// Auto retransmit counter exceeds the programmed maximum limit
	if (reg & nRF24_MASK_MAX_RT) {
		nrf24_flush_tx();
		return -2;
	};
	// Transmit successful
	if (reg & nRF24_MASK_TX_DS)
		return 0;

	nrf24_flush_tx();
	nrf24_clear_intr();
	return -3;
}


// Put nRF24L01 in TX mode
// input:
//   RetrCnt - Auto retransmit count on fail of AA (1..15 or 0 for disable)
//   RetrDelay - Auto retransmit delay 250us+(0..15)*250us (0 = 250us, 15 = 4000us)
//   RFChan - Frequency channel (0..127) (frequency = 2400 + RFChan [MHz])
//   DataRate - Set data rate: nRF24_DataRate_1Mbps or nRF24_DataRate_2Mbps
//   TXPower - RF output power (-18dBm, -12dBm, -6dBm, 0dBm)
//   CRCS - CRC encoding scheme (nRF24_CRC_[off | 1byte | 2byte])
//   addr -  TX address

void nrf24_mode_tx(uint8_t retry, uint8_t retry_delay, uint8_t channel, enum nrf24_speed speed,
	enum nrf24_txpower txpower, enum nrf24_crc_mode crc, uint16_t addr)
{
	uint8_t rreg;

	CHIP_MODE_LO();
	nrf24_read_reg(0x00); // Dummy read
	nrf24_write_reg(nRF24_REG_SETUP_AW, 0); // Set address width
	nrf24_write_buf(nRF24_REG_TX_ADDR, (uint8_t *) & addr, 2); // Set static TX address
	nrf24_write_buf(nRF24_REG_RX_ADDR(0), (uint8_t *) & addr, 2); // Set static RX address for given data pipe
	nrf24_write_reg(nRF24_REG_EN_AA, 0x01);

	nrf24_write_reg(nRF24_REG_RF_SETUP, (uint8_t) speed | (uint8_t) txpower); // Setup register
	nrf24_write_reg(nRF24_REG_CONFIG, (uint8_t) crc | (uint8_t) 0x02); // Config register
	nrf24_set_rfchannel(channel); // Set frequency channel (OBSERVER_TX part PLOS_CNT will be cleared)
	rreg = nrf24_read_reg(nRF24_REG_EN_AA);
	nrf24_write_reg(nRF24_REG_SETUP_RETR, (retry_delay << 4) | (retry & 0x0f)); // Auto retransmit settings
	if (retry) {
		// Enable auto acknowledgment for data pipe 0
		rreg |= 0x01;
		// Static RX address of the PIPE0 must be same as TX address for auto ack
		nrf24_write_buf(nRF24_REG_RX_ADDR_P0, (uint8_t *) & addr, 4);
	} else {
		// Disable auto acknowledgment for data pipe 0
		rreg &= ~0x01;
	}
	nrf24_write_reg(nRF24_REG_EN_AA, rreg);
}

void nrf24_init()
{
	// Enable the SPI peripheral
	set_bit(CLK_PCKENR1, CLK_PCKENR1_SPI);
	// Configure the SPI
	//   - MSB first
	//   - Baud = Fsysclk/2
	//   - Master mode
	//   - CPOL = low
	//   - CPHA = 1st edge
	SPI1_CR1 = 0x34;
	//   - 2-line unidirectional data mode
	//   - full duplex
	//   - software slave management enabled
	//   - CRC generation enabled
	SPI1_CR2 = 0x03;
	// SPI enabled
	set_bit(SPI1_CR1, SPI1_CR1_SPE);

	CHIP_UNSELECT();
	CHIP_MODE_LO();
	nrf24_clear_intr();
}

