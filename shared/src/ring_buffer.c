#include "ring_buffer.h"


void ring_buffer_init(ring_buffer_t* rb, uint8_t* buf, uint32_t size)
{
    rb->buf = buf;
    rb->size = size;
    rb->read_index = 0U;
    rb->write_index = 0U;
}

bool ring_buffer_read(ring_buffer_t* rb, uint8_t* byte)
{
    uint32_t local_read_index = rb->read_index;
    uint32_t local_write_index = rb->write_index;

    if(local_read_index == local_write_index)
        return false;
    
    *byte = rb->buf[local_read_index];

    rb->read_index = (local_read_index + 1) % rb->size;
    
    return true;
}

bool ring_buffer_write(ring_buffer_t* rb, uint8_t byte)
{
    uint32_t local_read_index = rb->read_index;
    uint32_t local_write_index = rb->write_index;

    uint32_t next_write_index = (local_write_index + 1) % rb->size;

    if(next_write_index == local_read_index)
    {
        return false;
    }

    rb->buf[local_write_index] = byte;
    
    rb->write_index = next_write_index;

    return true;
}

bool ring_buffer_empty(ring_buffer_t* rb)
{
    /* buffer is empty if read and write pointer are equal */
    if(rb->write_index == rb->read_index)
        return true;
    
    return false;
}