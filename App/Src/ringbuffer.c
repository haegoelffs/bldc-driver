/*
 * ringbuffer.c
 *
 *  Created on: Dec 26, 2017
 *      Author: simon
 */

#include <stdint.h>
#include "logger.h"
#include "ringbuffer.h"

/**StringRingbuffer allocStringRingbuffer(uint16_t bufferSize)
{
    StringRingbuffer *p;
    p = malloc(sizeof(*p) + bufferSize + 1); // alloc more space for the array (last position in struct)
    p->bufferSize = bufferSize;
}

unsigned char bufferIn(uint16_t byte)
{
    if((buffer.write + 1)%BUFFER_SIZE == buffer.read)
    {
        // buffer full
        return BUFFER_FAIL;
    }

    // write data in array
    buffer.data[buffer.write] = byte;

    // increase write pointer
    buffer.write = (buffer.write + 1)%BUFFER_SIZE;

    return BUFFER_SUCCESS;
}

unsigned char bufferOut(uint16_t *pByte)
{
    if (buffer.read == buffer.write)
    {
        // no element
        return BUFFER_FAIL;
    }

    *pByte = buffer.data[buffer.read];

    buffer.read = (buffer.read + 1)%BUFFER_SIZE;

    return BUFFER_SUCCESS;
}*/
