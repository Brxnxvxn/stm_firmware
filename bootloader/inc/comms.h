#ifndef COMMS_H
#define COMMS_H

#include "common_defines.h"

#define DATA_LENGTH_MAX     16U
#define CRC_BYTES            1U
#define LENGTH_BYTES         1U
#define PACKET_LENGTH   DATA_LENGTH_MAX + CRC_BYTES + LENGTH_BYTES

#define PACKET_RETX         0x19U
#define PACKET_ACK          0x15U

typedef enum {
    COMMS_PACKET_LENGTH,
    COMMS_PACKET_DATA,
    COMMS_PACKET_CRC
} COMMS_STATE;

typedef struct comms_packet_s
{
    uint8_t length;
    uint8_t data[DATA_LENGTH_MAX];
    uint8_t crc;
} comms_packet_t;

void comms_setup(void);
void comms_update(void);
void comms_create_single_byte_packet(comms_packet_t* packet, uint8_t byte);
bool comms_is_single_byte_packet(comms_packet_t* packet, uint8_t byte);
void comms_read(comms_packet_t* packet);
void comms_write(comms_packet_t* packet);

#endif /* COMMS_H */