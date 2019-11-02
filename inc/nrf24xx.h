/*
    Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>

    Permission is hereby granted, free of charge, to any person 
    obtaining a copy of this software and associated documentation 
    files (the "Software"), to deal in the Software without 
    restriction, including without limitation the rights to use, copy, 
    modify, merge, publish, distribute, sublicense, and/or sell copies 
    of the Software, and to permit persons to whom the Software is 
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be 
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
    DEALINGS IN THE SOFTWARE.

    $Id$
*/

#ifndef NRF24XX_H
#define NRF24XX_H

#include <stdint.h>

/* LOWLEVEL */
#define NRF24_TRANSMISSON_OK  0
#define NRF24_MESSAGE_LOST    1
#define NRF24_INVALID_ARGUMENT  2


#define NRF24_SPI_MSG_MAX_LEN 32
#define NRF24_PAYLOAD_LEN 16

#define NRF24_ADDR_LEN 5

#define NRF24_MAX_RETRIES 50

#define NRF24_NR_TX_MESSAGES 2
#define NRF24_NR_RX_MESSAGES 2

#if (NRF24_ADDR_LEN>NRF24_PAYLOAD_LEN)
#error INVALID NRF24 Msg payload length!
#endif

#define NRF24_BUFFER_PAYLOAD_LEN (NRF24_PAYLOAD_LEN-NRF24_ADDR_LEN-3)

typedef struct nrf24xx_msg_s
{
  uint8_t addr_from[NRF24_ADDR_LEN];
  uint8_t payload_size;
  uint8_t msg_id;
  uint8_t rx_req;
  uint8_t msg_buffer[NRF24_BUFFER_PAYLOAD_LEN];

} nrf24xx_msg_t;

typedef union nrf24xx_msg_union_s
{
  uint8_t raw[NRF24_PAYLOAD_LEN];
  nrf24xx_msg_t msg;

} nrf24xx_msg_union_t;

typedef struct nrf24xx_s
{
   /* spi interface */
   void *spi;

   /* Storage to build the SPI message maxlen is 32 + Control byte = 33 */
   uint8_t spi_msg[NRF24_SPI_MSG_MAX_LEN+1];

   /* Function pointers to the corresponding SPI driver layer */
   uint8_t (*SPI_transfer_byte)(void *spi, uint8_t val);
   void (*SPI_transfer_msg)(void *spi, uint8_t *data_out, uint8_t *data_in, uint16_t len);

   /* Hardware specific */
   void (*NRF24XX_set_ce)(uint8_t val);

   /* Software layer specific */
   void (*NRF24XX_delay_func)(uint32_t val);

   /* execution time infos */
   uint8_t payload_len;
   uint8_t channel;

}nrf24xx_t;

uint8_t nrf24_read_register_byte(nrf24xx_t *nrf24, uint8_t reg);

void nrf24_drv_init(nrf24xx_t *nrf24,
          void *nrf24_spi,
          void *spi_transfer_byte,
          void *spi_transfer_msg,
          void *nrf24xx_set_ce,
          void *nrf24_delay_func);

void nrf24_config(nrf24xx_t *nrf24, uint8_t channel, uint8_t pay_length);

void nrf24_power_up_rx(nrf24xx_t *nrf24);

void nrf24_power_up_tx(nrf24xx_t *nrf24);

void nrf24_power_down(nrf24xx_t *nrf24);

void nrf24_rx_address(nrf24xx_t *nrf24, uint8_t * adr);

void nrf24_tx_address(nrf24xx_t *nrf24, uint8_t* adr);

uint8_t nrf24_data_ready(nrf24xx_t *nrf24);

void nrf24_get_data(nrf24xx_t *nrf24, uint8_t* data);

void nrf24_send(nrf24xx_t *nrf24, uint8_t* value);

uint8_t nrf24_is_sending(nrf24xx_t *nrf24);

uint8_t nrf24_get_status(nrf24xx_t *nrf24);

uint8_t nrf24_last_message_status(nrf24xx_t *nrf24);

void nrf24_clear_irqs(nrf24xx_t *nrf24);

#endif
