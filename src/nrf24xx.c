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

#define nrf24_CONFIG ((1<<NRF24_EN_CRC) | (0<<NRF24_CRCO))

#define NRF24_SEND_DELAY_MS 300

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
    memcpy(&nrf24->spi_msg[1], value, len);
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
    nrf24_write_register(nrf24, NRF24_RX_PW_P0, 0x00); // Auto-ACK pipe ...
    nrf24_write_register(nrf24, NRF24_RX_PW_P1, pay_length); // Data payload pipe
//    nrf24_write_register(nrf24, NRF24_RX_PW_P2, 0x00); // Pipe not used
//    nrf24_write_register(nrf24, NRF24_RX_PW_P3, 0x00); // Pipe not used
//    nrf24_write_register(nrf24, NRF24_RX_PW_P4, 0x00); // Pipe not used
//    nrf24_write_register(nrf24, NRF24_RX_PW_P5, 0x00); // Pipe not used

    // 250 kbps, TX gain: 0dbm
    nrf24_write_register(nrf24, NRF24_RF_SETUP, (1<<NRF24_RF_DR_LOW) | (0<<NRF24_RF_DR) | (0x03<<NRF24_RF_PWR));

    // CRC enable, 1 byte CRC length
    nrf24_write_register(nrf24, NRF24_CONFIG, nrf24_CONFIG);

    // Auto Acknowledgment
    nrf24_write_register(nrf24, NRF24_EN_AA,(1<<NRF24_ENAA_P0)|(1<<NRF24_ENAA_P1)|(0<<NRF24_ENAA_P2)|(0<<NRF24_ENAA_P3)|(0<<NRF24_ENAA_P4)|(0<<NRF24_ENAA_P5));

    // Enable RX addresses
    nrf24_write_register(nrf24, NRF24_EN_RXADDR,(1<<NRF24_ERX_P0)|(1<<NRF24_ERX_P1)|(0<<NRF24_ERX_P2)|(0<<NRF24_ERX_P3)|(0<<NRF24_ERX_P4)|(0<<NRF24_ERX_P5));

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
    nrf24_write_register_burst(nrf24, NRF24_RX_ADDR_P1, adr, NRF24_ADDR_LEN);
    nrf24->NRF24XX_set_ce(NRF24_HIGH);
}

/* Set the TX address */
void nrf24_tx_address(nrf24xx_t *nrf24, uint8_t *adr)
{
    /* RX_ADDR_P0 must be set to the sending addr for auto ack to work. */
    nrf24_write_register_burst(nrf24, NRF24_RX_ADDR_P0, adr,NRF24_ADDR_LEN);
    nrf24_write_register_burst(nrf24, NRF24_TX_ADDR,adr, NRF24_ADDR_LEN);
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

    memcpy(data, &nrf24->spi_msg[1], nrf24->payload_len);

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
    memcpy(&nrf24->spi_msg[1], value, nrf24->payload_len);
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

uint8_t nrf24_send_message(nrf24xx_t *nrf24, uint8_t *buf, uint8_t buf_size)
{
    uint8_t retries = 0;
    uint8_t temp;
    uint8_t message[NRF24_PAYLOAD_LEN];

    memcpy(message, buf, buf_size);

    /* Automatically goes to TX mode */
    nrf24_send(nrf24, message);

    /* Wait for transmission to end */
    while(nrf24_is_sending(nrf24))
    {
        //NRF24XX_DEBUG_PRINT("Waiting for sending...\n\r");
        if(retries == NRF24_MAX_RETRIES)
            return NRF24_MESSAGE_LOST;
        
        nrf24->NRF24XX_delay_func(10);
        retries++;
    }

    /* Make analysis on last tranmission attempt */
    temp = nrf24_last_message_status(nrf24);
    if(temp == NRF24_TRANSMISSON_OK)
    {
        NRF24XX_DEBUG_PRINT(">OK\r\n");
        return NRF24_TRANSMISSON_OK;
    }
    else if(temp == NRF24_MESSAGE_LOST)
    {
        NRF24XX_DEBUG_PRINT(">Err\r\n");
        return NRF24_MESSAGE_LOST;
    }

    return NRF24_MESSAGE_LOST;
}

static void nrf24_encode_message(nrf24xx_msg_union_t *msg_to_send, uint8_t *rxaddress, uint8_t msg_id, uint8_t *buf, uint8_t buf_size, uint8_t rx_req)
{
    /* Encode message */
    memcpy(msg_to_send->msg.addr_from, rxaddress, NRF24_ADDR_LEN);

    if(buf != NULL)
        memcpy(msg_to_send->msg.msg_buffer, buf, ((buf_size > NRF24_BUFFER_PAYLOAD_LEN) ? NRF24_BUFFER_PAYLOAD_LEN : buf_size));

    msg_to_send->msg.payload_size = buf_size;
    msg_to_send->msg.msg_id = msg_id;
    msg_to_send->msg.rx_req = rx_req;
}

static void print_message_info(nrf24xx_msg_union_t *msg_to_send)
{
    uint8_t i = 0;

    NRF24XX_DEBUG_PRINT("Address from:");
    for(i=0;i<NRF24_ADDR_LEN;i++)
    {
        NRF24XX_DEBUG_PRINT(" %d", msg_to_send->msg.addr_from[i]);
    }
    NRF24XX_DEBUG_PRINT("\r\n");

    NRF24XX_DEBUG_PRINT("Message id: %d\r\n", msg_to_send->msg.msg_id);
    NRF24XX_DEBUG_PRINT("Payload size: %d\r\n", msg_to_send->msg.payload_size);

    NRF24XX_DEBUG_PRINT("Msg buf:");
    for(i=0;i<msg_to_send->msg.payload_size;i++)
    {
        NRF24XX_DEBUG_PRINT(" %d", msg_to_send->msg.msg_buffer[i]);
    }
    NRF24XX_DEBUG_PRINT("\r\n");

    NRF24XX_DEBUG_PRINT("rx req: %d\r\n", msg_to_send->msg.rx_req);
}

static void nrf24_prepare_and_send(nrf24xx_t *nrf24, uint8_t *ownaddress, nrf24xx_msg_union_t *msg_to_send, uint8_t *buf_tx, uint8_t buf_size)
{
    uint8_t j = 0;
    uint8_t how_many = 0;
    uint8_t rest = 0;

    /* packetize the buffer */
    how_many = (buf_size/NRF24_BUFFER_PAYLOAD_LEN)+1;

    if(buf_size>=NRF24_BUFFER_PAYLOAD_LEN)
    {
        rest = buf_size%NRF24_BUFFER_PAYLOAD_LEN;
        if(!rest)
        {
            rest=NRF24_BUFFER_PAYLOAD_LEN;
            how_many--;
        }
    }
    else
    {
        rest = buf_size;
    }

    NRF24XX_DEBUG_PRINT("bufsize: %d how_many: %d rest: %d\n", buf_size, how_many, rest);

    nrf24_encode_message(msg_to_send, ownaddress, how_many, NULL, buf_size, 0);

    nrf24->NRF24XX_delay_func(NRF24_SEND_DELAY_MS);
    nrf24_send_message(nrf24, msg_to_send->raw, NRF24_PAYLOAD_LEN);

    for(j=0;j<how_many;j++)
    {
        nrf24->NRF24XX_delay_func(NRF24_SEND_DELAY_MS);
        if(j==(how_many-1))
        {
            NRF24XX_DEBUG_PRINT("last encoding: %s with size %d\n", &buf_tx[j*NRF24_BUFFER_PAYLOAD_LEN], rest);
            nrf24_encode_message(msg_to_send, ownaddress, j, &buf_tx[j*NRF24_BUFFER_PAYLOAD_LEN], rest, 0);
        }
        else
        {
            NRF24XX_DEBUG_PRINT("encoding: %s with size %d\n", &buf_tx[j*NRF24_BUFFER_PAYLOAD_LEN], NRF24_BUFFER_PAYLOAD_LEN);
            nrf24_encode_message(msg_to_send, ownaddress, j, &buf_tx[j*NRF24_BUFFER_PAYLOAD_LEN], NRF24_BUFFER_PAYLOAD_LEN, 0);
        }
        nrf24_send_message(nrf24, msg_to_send->raw, NRF24_PAYLOAD_LEN);
        nrf24->NRF24XX_delay_func(NRF24_SEND_DELAY_MS/2);
    }
}

static uint16_t nrf24_prepare_and_receive(nrf24xx_t *nrf24, nrf24xx_msg_union_t *msg_to_send, uint8_t *buf_rx, uint8_t buf_size, uint8_t blocking_wait)
{
    uint8_t retries = 0;
    uint8_t j = 0;
    uint8_t how_many = 0;
    uint8_t buf_rx_max_packets = 0;

    how_many = msg_to_send->msg.msg_id;
    buf_rx_max_packets = (buf_size/NRF24_BUFFER_PAYLOAD_LEN)+1;
    if(!(buf_size%NRF24_BUFFER_PAYLOAD_LEN))
    {
        buf_rx_max_packets--;
    }

    for(j=0;j<how_many;j++)
    {
        nrf24_power_up_rx(nrf24);
        while(1)
        {
            if(nrf24_data_ready(nrf24))
            {
                /* Get the address */
                //nrf24_clear_irqs(nrf24l01);
                nrf24_get_data(nrf24, msg_to_send->raw);

                if(buf_rx_max_packets)
                {
                    buf_rx_max_packets--;
                    memcpy(&buf_rx[j*NRF24_BUFFER_PAYLOAD_LEN],
                            msg_to_send->msg.msg_buffer,
                            ((buf_size > msg_to_send->msg.payload_size) ? msg_to_send->msg.payload_size : buf_size));
                    NRF24XX_DEBUG_PRINT("y\n\r");
                }
                else
                {
                    NRF24XX_DEBUG_PRINT("n\n\r");
                }
                //print_message_info(&msg_to_send);
                break;
            }
            if(!blocking_wait)
            {
                if(retries == NRF24_MAX_RETRIES)
                    return NRF24_MESSAGE_LOST ;
                nrf24->NRF24XX_delay_func(20);
                retries++;
            }
        }
    }

    return NRF24_TRANSMISSON_OK;
}

uint8_t nrf24_master_ping_pong(nrf24xx_t *nrf24, uint8_t *rxaddress, uint8_t *buf_tx, uint8_t *buf_rx, uint8_t buf_size, uint8_t rx_req)
{
    uint8_t retries = 0;
    uint8_t how_many = 0;
    nrf24xx_msg_union_t msg_to_send;

    if(!rx_req)
    {
        nrf24_prepare_and_send(nrf24, rxaddress, &msg_to_send, buf_tx, buf_size);
    }
    else
    {
        NRF24XX_DEBUG_PRINT("bufsize: %d how_many: %d\n", buf_size, how_many);

        nrf24_encode_message(&msg_to_send, rxaddress, how_many, NULL, 0, 1);

        nrf24->NRF24XX_delay_func(NRF24_SEND_DELAY_MS);
        nrf24_send_message(nrf24, msg_to_send.raw, NRF24_PAYLOAD_LEN);

        /* Get some control information */
        nrf24_power_up_rx(nrf24);
        while(1)
        {
            if(nrf24_data_ready(nrf24))
            {
                //nrf24_clear_irqs(nrf24l01);
                nrf24_get_data(nrf24, msg_to_send.raw);
                print_message_info(&msg_to_send);
                break;
            }
            if(retries == NRF24_MAX_RETRIES)
                return NRF24_MESSAGE_LOST;
            nrf24->NRF24XX_delay_func(10);
            retries++;
        }

        nrf24_prepare_and_receive(nrf24, &msg_to_send, buf_rx, buf_size, 0);
    }

    return NRF24_TRANSMISSON_OK;
}

uint8_t nrf24_slave_ping_pong(nrf24xx_t *nrf24, uint8_t *rx_address, uint8_t *buf_tx, uint8_t *buf_rx, uint8_t buf_size)
{
    uint8_t j = 0;
    nrf24xx_msg_union_t msg_to_send;
    uint8_t how_many = 0;
    uint8_t rx_req = 0;
    uint8_t rest = 0;
    uint8_t payload_rx_message = 0;

    /* Get some control information */
    nrf24_power_up_rx(nrf24);
    while(1)
    {
        if(nrf24_data_ready(nrf24))
        {
            //nrf24_clear_irqs(nrf24l01);
            nrf24_get_data(nrf24, msg_to_send.raw);
            print_message_info(&msg_to_send);
            payload_rx_message = msg_to_send.msg.payload_size;
            break;
        }
    }

    nrf24_tx_address(nrf24, msg_to_send.msg.addr_from);

    rx_req = msg_to_send.msg.rx_req;

    if(!rx_req)
    {
        nrf24_prepare_and_receive(nrf24, &msg_to_send, buf_rx, buf_size, 1);
    }
    else
    {
        //FIXME: This produces a strange compiler error when using avr-gcc version 4.8.1: should be uncommented
        //nrf24_prepare_and_send(nrf24, rx_address, &msg_to_send, buf_tx, buf_size);

        /* packetize the buffer */
        how_many = (buf_size/NRF24_BUFFER_PAYLOAD_LEN)+1;

        if(buf_size>=NRF24_BUFFER_PAYLOAD_LEN)
        {
            rest = buf_size%NRF24_BUFFER_PAYLOAD_LEN;
            if(!rest)
            {
                rest=NRF24_BUFFER_PAYLOAD_LEN;
                how_many--;
            }
        }
        else
        {
            rest = buf_size;
        }

        nrf24_encode_message(&msg_to_send, rx_address, how_many, NULL, 0, rx_req);

        nrf24->NRF24XX_delay_func(NRF24_SEND_DELAY_MS);
        nrf24_send_message(nrf24, msg_to_send.raw, NRF24_PAYLOAD_LEN);

        for(j=0;j<how_many;j++)
        {
            nrf24->NRF24XX_delay_func(NRF24_SEND_DELAY_MS);
            if(j==(how_many-1))
            {
                NRF24XX_DEBUG_PRINT("last encoding: %s with size %d\n", &buf_tx[j*NRF24_BUFFER_PAYLOAD_LEN], rest);
                nrf24_encode_message(&msg_to_send, rx_address, j, &buf_tx[j*NRF24_BUFFER_PAYLOAD_LEN], rest, rx_req);
            }
            else
            {
                NRF24XX_DEBUG_PRINT("encoding: %s with size %d\n", &buf_tx[j*NRF24_BUFFER_PAYLOAD_LEN], NRF24_BUFFER_PAYLOAD_LEN);
                nrf24_encode_message(&msg_to_send, rx_address, j, &buf_tx[j*NRF24_BUFFER_PAYLOAD_LEN], NRF24_BUFFER_PAYLOAD_LEN, rx_req);
            }
            nrf24_send_message(nrf24, msg_to_send.raw, NRF24_PAYLOAD_LEN);
            nrf24->NRF24XX_delay_func(NRF24_SEND_DELAY_MS/2);
        }
    }

    return payload_rx_message;
}
