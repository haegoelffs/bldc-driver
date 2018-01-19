/*
 * drive.h
 *
 *  Created on: Dec 25, 2017
 *      Author: simon
 */

#ifndef INC_DRIVE_H_
#define INC_DRIVE_H_

#include <stdint.h>

void initDrive();
void proceedDrive();

void inform_newRotorPos(uint32_t time);
void inform_newRotationDirection(uint8_t direction);
void inform_tooManyZeroCrossings();

typedef enum {
            free_running,
            start_up,
            controlled_positive_torque,
			controlled_negative_torque
            } DriveState;
void changeState(DriveState newState);

#endif /* INC_DRIVE_H_ */
