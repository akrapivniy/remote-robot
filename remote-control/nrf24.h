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

#ifndef __NRF24_H__
#define __NRF24_H__

#ifdef __cplusplus
extern "C" {
#endif

// Chip Enable Activates RX or TX mode

// Chip Select
#define CHIP_SELECT() PB_ODR &= ~0x10;
#define CHIP_UNSELECT() PB_ODR |= 0x10;
   
#define CHIP_MODE_HI() PB_ODR |= 0x08;
#define CHIP_MODE_LO() PB_ODR &= ~0x08;
    
#define nrf24_wait_intr(wait) while ((PB_IDR&0x04) && wait) wait--
    
// nRF24L0 commands
#define nRF24_CMD_RREG             0x00  // R_REGISTER -> Read command and status registers
#define nRF24_CMD_WREG             0x20  // W_REGISTER -> Write command and status registers
#define nRF24_CMD_R_RX_PAYLOAD     0x61  // R_RX_PAYLOAD -> Read RX payload
#define nRF24_CMD_W_TX_PAYLOAD     0xA0  // W_TX_PAYLOAD -> Write TX payload
#define nRF24_CMD_FLUSH_TX         0xE1  // FLUSH_TX -> Flush TX FIFO
#define nRF24_CMD_FLUSH_RX         0xE2  // FLUSH_RX -> Flush RX FIFO
#define nRF24_CMD_REUSE_TX_PL      0xE3  // REUSE_TX_PL -> Reuse last transmitted payload
#define nRF24_CMD_NOP              0xFF  // No operation (to read status register)

// nRF24L0 registers
#define nRF24_REG_CONFIG           0x00  // Configuration register
#define nRF24_REG_EN_AA            0x01  // Enable "Auto acknowledgment"
#define nRF24_REG_EN_RXADDR        0x02  // Enable RX addresses
#define nRF24_REG_SETUP_AW         0x03  // Setup of address widths
#define nRF24_REG_SETUP_RETR       0x04  // Setup of automatic retranslation
#define nRF24_REG_RF_CH            0x05  // RF channel
#define nRF24_REG_RF_SETUP         0x06  // RF setup register
#define nRF24_REG_STATUS           0x07  // Status register
#define nRF24_REG_OBSERVE_TX       0x08  // Transmit observe register
#define nRF24_REG_RPD              0x09  // Received power detector
#define nRF24_REG_RX_ADDR_P0       0x0A  // Receive address data pipe 0
#define nRF24_REG_RX_ADDR_P1       0x0B  // Receive address data pipe 1
#define nRF24_REG_RX_ADDR_P2       0x0C  // Receive address data pipe 2
#define nRF24_REG_RX_ADDR_P3       0x0D  // Receive address data pipe 3
#define nRF24_REG_RX_ADDR_P4       0x0E  // Receive address data pipe 4
#define nRF24_REG_RX_ADDR_P5       0x0F  // Receive address data pipe 5
#define nRF24_REG_TX_ADDR          0x10  // Transmit address
#define nRF24_REG_RX_PW_P0         0x11  // Number of bytes in RX payload id data pipe 0
#define nRF24_REG_RX_PW_P1         0x12  // Number of bytes in RX payload id data pipe 1
#define nRF24_REG_RX_PW_P2         0x13  // Number of bytes in RX payload id data pipe 2
#define nRF24_REG_RX_PW_P3         0x14  // Number of bytes in RX payload id data pipe 3
#define nRF24_REG_RX_PW_P4         0x15  // Number of bytes in RX payload id data pipe 4
#define nRF24_REG_RX_PW_P5         0x16  // Number of bytes in RX payload id data pipe 5
#define nRF24_REG_FIFO_STATUS      0x17  // FIFO status register
#define nRF24_REG_DYNPD            0x1C  // Enable dynamic payload length
#define nRF24_REG_FEATURE          0x1D  // Feature register

#define nRF24_REG_RX_ADDR(pipe) (nRF24_REG_RX_ADDR_P0+(pipe))
#define nRF24_REG_RX_PW(pipe) (nRF24_REG_RX_PW_P0+(pipe))
    
    
// nRF24L0 bits
#define nRF24_MASK_RX_DR           0x40  // Mask interrupt caused by RX_DR
#define nRF24_MASK_TX_DS           0x20  // Mask interrupt caused by TX_DS
#define nRF24_MASK_MAX_RT          0x10  // Mask interrupt caused by MAX_RT
#define nRF24_FIFO_RX_EMPTY        0x01  // RX FIFO empty flag
#define nRF24_FIFO_RX_FULL         0x02  // RX FIFO full flag

#define nRF24_TEST_ADDR         "nRF24"  // Fake address to test nRF24 presence

#define nRF24_WAIT_TIMEOUT       0xFFFF  // Timeout counter

    

// nRF24L01 data rate
enum nrf24_speed {
    nrf24_speed_250k = (uint8_t)0x20, // 250kbps data rate
    nrf24_speed_1m   = (uint8_t)0x00, // 1Mbps data rate
    nrf24_speed_2m   = (uint8_t)0x08  // 2Mbps data rate
};

// nRF24L01 RF output power in TX mode
enum nrf24_txpower {
    nrf24_txpower_18dbm = (uint8_t)0x00, // -18dBm
    nrf24_txpower_12dbm = (uint8_t)0x02, // -12dBm
    nrf24_txpower_6dbm  = (uint8_t)0x04, //  -6dBm
    nrf24_txpower_0dbm  = (uint8_t)0x06  //   0dBm
};
 
// nRF24L01 CRC encoding scheme
enum nrf24_crc_mode {
    nrf24_crc_off = (uint8_t)0x00, // CRC disabled
    nrf24_crc_1b = (uint8_t)0x08, // 1-byte CRC
    nrf24_crc_2b = (uint8_t)0x0c  // 2-byte CRC
};

    
    
void nrf24_init ();
void nrf24_mode_rx(uint8_t pipe, uint8_t pipe_size, uint8_t pipe_ack, uint8_t channel,
	enum nrf24_speed speed, enum nrf24_crc_mode crc, uint16_t addr,
	enum nrf24_txpower txpower);
void nrf24_mode_tx(uint8_t retry, uint8_t retry_delay, uint8_t channel, enum nrf24_speed speed,
	enum nrf24_txpower txpower, enum nrf24_crc_mode crc, uint16_t addr);
int8_t nrf24_write(uint8_t *buffer, uint8_t size);
int8_t nrf24_write_done();
int8_t nrf24_read(uint8_t *buffer, uint8_t size);
void nrf24_set_txpower(uint8_t power);
void nrf24_set_rfchannel(uint8_t channel);
int8_t nrf24_check(void);
void nrf24_status(uint8_t *r7, uint8_t *r8, uint8_t *r9);


#ifdef __cplusplus
}
#endif

#endif /* __NRF24_H__ */

