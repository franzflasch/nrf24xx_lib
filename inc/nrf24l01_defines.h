/*
 * nrf24l01_defines.h
 *
 *  Created on: Jul 9, 2015
 *      Author: fflasch
 */

#ifndef NRF24L01_DEFINES_H_
#define NRF24L01_DEFINES_H_

/* Memory Map */
#define NRF24_CONFIG      0x00
#define NRF24_EN_AA       0x01
#define NRF24_EN_RXADDR   0x02
#define NRF24_SETUP_AW    0x03
#define NRF24_SETUP_RETR  0x04
#define NRF24_RF_CH       0x05
#define NRF24_RF_SETUP    0x06
#define NRF24_STATUS      0x07
#define NRF24_OBSERVE_TX  0x08
#define NRF24_CD          0x09
#define NRF24_RX_ADDR_P0  0x0A
#define NRF24_RX_ADDR_P1  0x0B
#define NRF24_RX_ADDR_P2  0x0C
#define NRF24_RX_ADDR_P3  0x0D
#define NRF24_RX_ADDR_P4  0x0E
#define NRF24_RX_ADDR_P5  0x0F
#define NRF24_TX_ADDR     0x10
#define NRF24_RX_PW_P0    0x11
#define NRF24_RX_PW_P1    0x12
#define NRF24_RX_PW_P2    0x13
#define NRF24_RX_PW_P3    0x14
#define NRF24_RX_PW_P4    0x15
#define NRF24_RX_PW_P5    0x16
#define NRF24_FIFO_STATUS 0x17
#define NRF24_DYNPD       0x1C

/* Bit Mnemonics */

/* configuratio nregister */
#define NRF24_MASK_RX_DR  6
#define NRF24_MASK_TX_DS  5
#define NRF24_MASK_MAX_RT 4
#define NRF24_EN_CRC      3
#define NRF24_CRCO        2
#define NRF24_PWR_UP      1
#define NRF24_PRIM_RX     0

/* enable auto acknowledgment */
#define NRF24_ENAA_P5     5
#define NRF24_ENAA_P4     4
#define NRF24_ENAA_P3     3
#define NRF24_ENAA_P2     2
#define NRF24_ENAA_P1     1
#define NRF24_ENAA_P0     0

/* enable rx addresses */
#define NRF24_ERX_P5      5
#define NRF24_ERX_P4      4
#define NRF24_ERX_P3      3
#define NRF24_ERX_P2      2
#define NRF24_ERX_P1      1
#define NRF24_ERX_P0      0

/* setup of address width */
#define NRF24_AW          0 /* 2 bits */

/* setup of auto re-transmission */
#define NRF24_ARD         4 /* 4 bits */
#define NRF24_ARC         0 /* 4 bits */

/* RF setup register */
#define NRF24_RF_DR_LOW   5
#define NRF24_PLL_LOCK    4
#define NRF24_RF_DR       3
#define NRF24_RF_PWR      1 /* 2 bits */

/* general status register */
#define NRF24_RX_DR       6
#define NRF24_TX_DS       5
#define NRF24_MAX_RT      4
#define NRF24_RX_P_NO     1 /* 3 bits */
#define NRF24_TX_FULL     0

/* transmit observe register */
#define NRF24_PLOS_CNT    4 /* 4 bits */
#define NRF24_ARC_CNT     0 /* 4 bits */

/* fifo status */
#define NRF24_TX_REUSE    6
#define NRF24_FIFO_FULL   5
#define NRF24_TX_EMPTY    4
#define NRF24_RX_FULL     1
#define NRF24_RX_EMPTY    0

/* dynamic length */
#define NRF24_DPL_P0      0
#define NRF24_DPL_P1      1
#define NRF24_DPL_P2      2
#define NRF24_DPL_P3      3
#define NRF24_DPL_P4      4
#define NRF24_DPL_P5      5

/* Instruction Mnemonics */
#define NRF24_R_REGISTER    0x00 /* last 4 bits will indicate reg. address */
#define NRF24_W_REGISTER    0x20 /* last 4 bits will indicate reg. address */
#define NRF24_REGISTER_MASK 0x1F
#define NRF24_R_RX_PAYLOAD  0x61
#define NRF24_W_TX_PAYLOAD  0xA0
#define NRF24_FLUSH_TX      0xE1
#define NRF24_FLUSH_RX      0xE2
#define NRF24_REUSE_TX_PL   0xE3
#define NRF24_ACTIVATE      0x50
#define NRF24_R_RX_PL_WID   0x60
#define NRF24_NOP           0xFF

#endif /* NRF24L01_DEFINES_H_ */
