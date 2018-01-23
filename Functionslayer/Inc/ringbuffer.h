/*
 * ringbuffer.h
 *
 *  Created on: Dec 26, 2017
 *      Author: simon
 */

#ifndef INC_RINGBUFFER_H_
#define INC_RINGBUFFER_H_

#include <stdint.h>

#define BUFFER_FAIL     0
#define BUFFER_SUCCESS  1

#define BUFFER_SIZE 200

typedef struct
{
    int16_t data[BUFFER_SIZE][4];
    unsigned char read; // zeigt auf das Feld mit dem ältesten Inhalt
    unsigned char write; // zeigt immer auf leeres Feld
} BufferDriveData;

// source: https://stackoverflow.com/questions/246977/is-using-flexible-array-members-in-c-bad-practice
typedef struct
{
    uint16_t next_read;
    uint16_t next_write;
    uint16_t bufferSize;
    uint8_t data[];
} StringRingbuffer;

StringRingbuffer * allocStringRingbuffer(uint16_t bufferSize);

unsigned char bufferIn(BufferDriveData *buffer, int16_t value1, int16_t value2, int16_t value3, int16_t value4);

unsigned char bufferOut(BufferDriveData *buffer, int16_t *pValue1, int16_t *pValue2, int16_t *pValue3, int16_t *pValue4);

#endif /* INC_RINGBUFFER_H_ */
