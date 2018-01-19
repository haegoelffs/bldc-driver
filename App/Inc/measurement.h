/*
 * measurement.h
 *
 *  Created on: Dec 25, 2017
 *      Author: simon
 */

#ifndef INC_MEASUREMENT_H_
#define INC_MEASUREMENT_H_

#include <stdint.h>

void initMeasurement();

void register_newRotorPos_listener_ISR(void (*listener)(uint32_t));
void register_tooManyZeroCrossings_listener_ISR(void (*listener)(void));

#endif /* INC_MEASUREMENT_H_ */
