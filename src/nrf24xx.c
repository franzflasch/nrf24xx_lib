#include <nrf24xx.h>
#include <stdio.h>
#include <string.h>
#include <nrf24l01_defines.h>

//#define NRF24XX_DEBUG

#ifdef NRF24XX_DEBUG
#define NRF24XX_DEBUG_PRINT(fmt, args...)    printf(fmt, ## args)
#else
#define NRF24XX_DEBUG_PRINT(fmt, args...)    /* Don't do anything in release builds */
#endif

#define NRF24_LOW 0
#define NRF24_HIGH 1

#define nrf24_CONFIG ((1<<NRF24_EN_CRC) | (1<<NRF24_CRCO))

#define NRF24_SEND_DELAY_MS 300

/* It seems that all string functions in sdcc do not work properly so we use our own function */
static void local_memcpy(void *dest, void *src, size_t n)
{
    int i = 0;
   // Typecast src and dest addresses to (char *)
   uint8_t *csrc = (uint8_t *)src;
   uint8_t *cdest = (uint8_t *)dest;
 
   // Copy contents of src[] to dest[]
   for (i=0; i<n; i++)
       cdest[i] = csrc[i];
}

/* PRIVATE */
static uint8_t nrf24_transfer_spi_byte(nrf24xx_t *nrf24, uint8_t val)
{
   return nrf24->SPI_transfer_byte(nrf24->spi, val);
}

/* send and receive multiple bytes over SPI */
static void nrf24_transfer_spi(nrf24xx_t *nrf24, uint8_t* dataout, uint8_t* datain, uint8_t len)
{
    nrf24->SPI_transfer_msg(nrf24->spi, dataout, datain, len);
}

/* Clocks only one byte into the given nrf24 register */
static void nrf24_write_register(nrf24xx_t *nrf24, uint8_t reg, uint8_t value)
{
    nrf24->spi_msg[0] = (NRF24_W_REGISTER | (NRF24_REGISTER_MASK & reg));
    nrf24->spi_msg[1] = value;
    nrf24_transfer_spi(nrf24, nrf24->spi_msg, NULL, 2);
}

/* Read single register from nrf24 */
static uint8_t nrf24_read_register_byte(nrf24xx_t *nrf24, uint8_t reg)
{
    nrf24->spi_msg[0] = (NRF24_R_REGISTER | (NRF24_REGISTER_MASK & reg));
    nrf24_transfer_spi(nrf24, nrf24->spi_msg, nrf24->spi_msg, 2);
    return nrf24->spi_msg[1];
}

/* Write to a single register of nrf24 */
static void nrf24_write_register_burst(nrf24xx_t *nrf24, uint8_t reg, uint8_t *value, uint8_t len)
{
    nrf24->spi_msg[0] = (NRF24_W_REGISTER | (NRF24_REGISTER_MASK & reg));

    /* copy the message to the spi msg buffer */
    local_memcpy(&nrf24->spi_msg[1], value, len);
    nrf24_transfer_spi(nrf24, nrf24->spi_msg, NULL, len+1);
}



/* Checks if receive FIFO is empty or not */
static uint8_t nrf24_rx_fifo_empty(nrf24xx_t *nrf24)
{
    uint8_t fifoStatus;

    fifoStatus = nrf24_read_register_byte(nrf24, NRF24_FIFO_STATUS);

    //NRF24XX_DEBUG_PRINT("Fifo status %d\r\n", fifoStatus);
    //nrf24->NRF24XX_delay_func(100);
    return (fifoStatus & (1 << NRF24_RX_EMPTY));
}


/* PUBLIC */
void nrf24_drv_init(nrf24xx_t *nrf24,
                void *nrf24_spi,
                void *spi_transfer_byte,
                void *spi_transfer_msg,
                void *nrf24xx_set_ce,
                void *nrf24_delay_func)
{
    nrf24->spi = nrf24_spi;
    nrf24->SPI_transfer_byte = spi_transfer_byte;
    nrf24->SPI_transfer_msg = spi_transfer_msg;

    nrf24->NRF24XX_set_ce = nrf24xx_set_ce;
    nrf24->NRF24XX_set_ce(NRF24_LOW);

    nrf24->NRF24XX_delay_func = nrf24_delay_func;
}

/* configure the module */
void nrf24_config(nrf24xx_t *nrf24, uint8_t channel, uint8_t pay_length)
{
    /* Use static payload length ... */
    nrf24->payload_len = pay_length;

    // Set RF channel
    nrf24_write_register(nrf24, NRF24_RF_CH, channel);

    // Set length of incoming payload
    nrf24_write_register(nrf24, NRF24_RX_PW_P0, pay_length); // Auto-ACK pipe ...
    nrf24_write_register(nrf24, NRF24_RX_PW_P1, 0x00); // Data payload pipe
    nrf24_write_register(nrf24, NRF24_RX_PW_P2, 0x00); // Pipe not used
    nrf24_write_register(nrf24, NRF24_RX_PW_P3, 0x00); // Pipe not used
    nrf24_write_register(nrf24, NRF24_RX_PW_P4, 0x00); // Pipe not used
    nrf24_write_register(nrf24, NRF24_RX_PW_P5, 0x00); // Pipe not used

    // 250 kbps, TX gain: 0dbm
    nrf24_write_register(nrf24, NRF24_RF_SETUP, (1<<NRF24_RF_DR_LOW) | (0<<NRF24_RF_DR) | (0x03<<NRF24_RF_PWR));

    // CRC enable, 1 byte CRC length
    nrf24_write_register(nrf24, NRF24_CONFIG, nrf24_CONFIG);

    // Auto Acknowledgment
    nrf24_write_register(nrf24, NRF24_EN_AA,(1<<NRF24_ENAA_P0)|(0<<NRF24_ENAA_P1)|(0<<NRF24_ENAA_P2)|(0<<NRF24_ENAA_P3)|(0<<NRF24_ENAA_P4)|(0<<NRF24_ENAA_P5));

    // Enable RX addresses
    nrf24_write_register(nrf24, NRF24_EN_RXADDR,(1<<NRF24_ERX_P0)|(0<<NRF24_ERX_P1)|(0<<NRF24_ERX_P2)|(0<<NRF24_ERX_P3)|(0<<NRF24_ERX_P4)|(0<<NRF24_ERX_P5));

    // Auto retransmit delay: 1000 us and Up to 15 retransmit trials
    nrf24_write_register(nrf24, NRF24_SETUP_RETR, (0x04<<NRF24_ARD) | (0x0F<<NRF24_ARC));

    // Dynamic length configurations: No dynamic length
    nrf24_write_register(nrf24, NRF24_DYNPD,(0<<NRF24_DPL_P0)|(0<<NRF24_DPL_P1)|(0<<NRF24_DPL_P2)|(0<<NRF24_DPL_P3)|(0<<NRF24_DPL_P4)|(0<<NRF24_DPL_P5));

    // Start listening
    nrf24_power_up_rx(nrf24);
}

void nrf24_power_up_rx(nrf24xx_t *nrf24)
{
   nrf24_transfer_spi_byte(nrf24, NRF24_FLUSH_RX);
   nrf24_write_register(nrf24, NRF24_STATUS, (1<<NRF24_RX_DR) | (1<<NRF24_TX_DS) | (1<<NRF24_MAX_RT));

   nrf24->NRF24XX_set_ce(NRF24_LOW);
   nrf24_write_register(nrf24, NRF24_CONFIG, nrf24_CONFIG | ((1<<NRF24_PWR_UP) | (1<<NRF24_PRIM_RX)));
   nrf24->NRF24XX_set_ce(NRF24_HIGH);
}

void nrf24_power_up_tx(nrf24xx_t *nrf24)
{
    nrf24_write_register(nrf24, NRF24_STATUS, (1<<NRF24_RX_DR) | (1<<NRF24_TX_DS) | (1<<NRF24_MAX_RT));
    nrf24_write_register(nrf24, NRF24_CONFIG, nrf24_CONFIG | ((1<<NRF24_PWR_UP) | (0<<NRF24_PRIM_RX)));
}

void nrf24_power_down(nrf24xx_t *nrf24)
{
    nrf24->NRF24XX_set_ce(NRF24_LOW);
    nrf24_write_register(nrf24, NRF24_CONFIG, nrf24_CONFIG);
}

/* Set the RX address */
void nrf24_rx_address(nrf24xx_t *nrf24, uint8_t *adr)
{
    nrf24->NRF24XX_set_ce(NRF24_LOW);
    nrf24_write_register_burst(nrf24, NRF24_RX_ADDR_P0, adr, NRF24_ADDR_LEN);
    nrf24->NRF24XX_set_ce(NRF24_HIGH);
}

/* Set the TX address */
void nrf24_tx_address(nrf24xx_t *nrf24, uint8_t *adr)
{
    /* RX_ADDR_P0 must be set to the sending addr for auto ack to work. */
    nrf24_write_register_burst(nrf24, NRF24_RX_ADDR_P0, adr, NRF24_ADDR_LEN);
    nrf24_write_register_burst(nrf24, NRF24_TX_ADDR, adr, NRF24_ADDR_LEN);
}

/* Checks if data is available for reading */
/* Returns 1 if data is ready ... */
uint8_t nrf24_data_ready(nrf24xx_t *nrf24)
{
    // See note in getData() function - just checking RX_DR isn't good enough
    uint8_t status = nrf24_get_status(nrf24);

    // We can short circuit on RX_DR, but if it's not set, we still need
    // to check the FIFO for any pending packets
    if ( status & (1 << NRF24_RX_DR) )
    {
        return 1;
    }

    //return 0;
    return !nrf24_rx_fifo_empty(nrf24);
}

/* Reads payload bytes into data array */
void nrf24_get_data(nrf24xx_t *nrf24, uint8_t* data)
{
    nrf24->spi_msg[0] = NRF24_R_RX_PAYLOAD;
    memset(&nrf24->spi_msg[1], 0xff, nrf24->payload_len);
    nrf24_transfer_spi(nrf24, nrf24->spi_msg, nrf24->spi_msg, nrf24->payload_len+1);

    local_memcpy(data, &nrf24->spi_msg[1], nrf24->payload_len);

    /* Reset status register */
    nrf24_write_register(nrf24, NRF24_STATUS, (1<<NRF24_RX_DR));
}

// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
void nrf24_send(nrf24xx_t *nrf24, uint8_t* value)
{
    /* Go to Standby-I first */
    nrf24->NRF24XX_set_ce(NRF24_LOW);

    /* Set to transmitter mode , Power up if needed */
    nrf24_power_up_tx(nrf24);

    /* Do we really need to flush TX fifo each time ? */
    #if 1
        /* Write cmd to flush transmit FIFO */
        nrf24_transfer_spi_byte(nrf24, NRF24_FLUSH_TX);
    #endif

    nrf24->spi_msg[0] = NRF24_W_TX_PAYLOAD;

    /* copy the message to the spi msg buffer */
    local_memcpy(&nrf24->spi_msg[1], value, nrf24->payload_len);
    nrf24_transfer_spi(nrf24, nrf24->spi_msg, NULL, nrf24->payload_len+1);

    /* Start the transmission */
    nrf24->NRF24XX_set_ce(NRF24_HIGH);
}

uint8_t nrf24_is_sending(nrf24xx_t *nrf24)
{
    uint8_t status;

    /* read the current status */
    status = nrf24_get_status(nrf24);

    /* if sending successful (TX_DS) or max retries exceded (MAX_RT). */
    if((status & ((1 << NRF24_TX_DS) | (1 << NRF24_MAX_RT))))
    {
        return 0; /* false */
    }

    return 1; /* true */

}

uint8_t nrf24_get_status(nrf24xx_t *nrf24)
{
    uint8_t rv;
    rv = nrf24_transfer_spi_byte(nrf24, NRF24_NOP);
    return rv;
}

uint8_t nrf24_last_message_status(nrf24xx_t *nrf24)
{
    uint8_t rv;

    rv = nrf24_get_status(nrf24);

    /* Transmission went OK */
    if((rv & ((1 << NRF24_TX_DS))))
    {
        return NRF24_TRANSMISSON_OK;
    }
    /* Maximum retransmission count is reached */
    /* Last message probably went missing ... */
    else if((rv & ((1 << NRF24_MAX_RT))))
    {
        return NRF24_MESSAGE_LOST;
    }
    /* Probably still sending ... */
    else
    {
        return 0xFF;
    }
}

void nrf24_clear_irqs(nrf24xx_t *nrf24)
{
    nrf24_write_register(nrf24, NRF24_STATUS, (1<<NRF24_RX_DR) | (1<<NRF24_TX_DS) | (1<<NRF24_MAX_RT));
}
