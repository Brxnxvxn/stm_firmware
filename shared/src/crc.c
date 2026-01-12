#include "crc.h"

uint8_t crc8(uint8_t* data, uint32_t len)
{
    uint8_t crc = 0U;
    uint8_t poly = 0x07U;

    for (uint32_t i = 0; i < len; i++)
    {
        crc ^= data[i];

        for (int n = 0; n < 8; n++) 
        {
            if(crc & 0x80)
            {
                crc = (crc << 1) ^ poly;
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;

}