/*
 * pwrControl.h
 *
 *  Created on: Apr 27, 2018
 *      Author: simon
 */

#ifndef INC_PWRCONTROL_H_
#define INC_PWRCONTROL_H_

#include <stdint.h>

#define MAX_POWER_LEVEL 255
#define OFF_POWER_LEVEL 0

void setPowerlevel(uint8_t powerLevel);

#endif /* INC_PWRCONTROL_H_ */
