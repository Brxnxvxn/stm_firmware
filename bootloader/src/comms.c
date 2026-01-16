#include <string.h>

#include "comms.h"
#include "uart.h"
#include "crc.h"


#define PACKET_BUFFER_LENGTH   8U


static COMMS_STATE state = COMMS_PACKET_LENGTH;

static comms_packet_t temp_packet = {.length = 0, .data = {0}, .crc = 0 };
static comms_packet_t retx_packet = {.length = 0, .data = {0}, .crc = 0 };
static comms_packet_t ack_packet = {.length = 0, .data = {0}, .crc = 0 };
static comms_packet_t last_tx_packet = {.length = 0, .data = {0}, .crc = 0 };

static comms_packet_t packet_buf[PACKET_BUFFER_LENGTH];

static uint32_t packet_read_index = 0U;
static uint32_t packet_write_index = 0U;

void comms_setup()
{
    /* create retx packet */
    comms_create_single_byte_packet(&retx_packet, PACKET_RETX);

    /* create ack packet */
    comms_create_single_byte_packet(&ack_packet, PACKET_ACK);
}

bool comms_is_single_byte_packet(comms_packet_t* packet, uint8_t byte)
{
    if (packet->length != 1)
    {
        return false;
    }

    if (packet->data[0] != byte)
    {
        return false;
    }

    for (uint8_t i = 1; i < DATA_LENGTH_MAX; i++)
    {
        if(packet->data[i] != 0xFF)
        {
            return false;
        }
    }

    return true;

}

void comms_create_single_byte_packet(comms_packet_t* packet, uint8_t byte)
{
    memset(packet, 0xFF, sizeof(comms_packet_t));
    packet->length = 1U;
    packet->data[0] = byte;
    packet->crc = crc8((uint8_t*)packet, PACKET_LENGTH - CRC_BYTES);

}

void comms_update()
{   
    while(uart_data_available())
    {
        switch(state)
        {
            case COMMS_PACKET_LENGTH:
                temp_packet.length = uart_read_byte();
                state = COMMS_PACKET_DATA;
                break;

            case COMMS_PACKET_DATA:
                for (uint32_t i = 0; i < DATA_LENGTH_MAX; i++)
                {
                    temp_packet.data[i] = uart_read_byte();
                }
                state = COMMS_PACKET_CRC;
                break;

            case COMMS_PACKET_CRC:    
                /* read crc byte */
                temp_packet.crc = uart_read_byte();

                /* compute crc and check for match - write retx packet and move to length state if mismatch */
                if (temp_packet.crc != crc8((uint8_t*)&temp_packet, PACKET_LENGTH - CRC_BYTES))
                {
                    comms_write(&retx_packet);
                    state = COMMS_PACKET_LENGTH;
                    break;
                }

                /* if packet is a retx packet, write last packet and move to length state */
                if (comms_is_single_byte_packet(&temp_packet, PACKET_RETX))
                {
                    comms_write(&last_tx_packet);
                    state = COMMS_PACKET_LENGTH;
                    break;
                }

                /* if packet is an ack packet, do nothing and move to length state */
                if (comms_is_single_byte_packet(&temp_packet, PACKET_ACK))
                {
                    state = COMMS_PACKET_LENGTH;
                    break;
                }

                /* else add packet to buffer and break */

                /* check if next write index is equal to read index */
                uint32_t next_write_index = (packet_write_index + 1) % PACKET_BUFFER_LENGTH;

                if(next_write_index == packet_read_index)
                {
                    __asm__("BKPT #0");
                }

                memcpy(&packet_buf[packet_write_index], &temp_packet, sizeof(comms_packet_t));
                packet_write_index = next_write_index;
                comms_write(&ack_packet);

                state = COMMS_PACKET_LENGTH;
                break;
            
            default:
                state = COMMS_PACKET_LENGTH;
        }
    }
}

void comms_read(comms_packet_t* packet)
{
    memcpy(packet, &packet_buf[packet_read_index], sizeof(comms_packet_t));
    packet_read_index = (packet_read_index + 1) % PACKET_BUFFER_LENGTH;
}

void comms_write(comms_packet_t* packet)
{
    uart_write((uint8_t*)packet, PACKET_LENGTH);

    /* store last transmitted packet */
    memcpy(&last_tx_packet, packet, sizeof(comms_packet_t));
}

bool comms_packet_available(void) {
    return packet_read_index != packet_write_index;
}