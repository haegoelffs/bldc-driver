/*
 * pwrControl.c
 *
 *  Created on: Apr 27, 2018
 *      Author: simon
 */

#include "pwrControl.h"
#include "bufferedLogger.h"

#include "bldc_driver_functions.h"
#include "bldc_driver_HAL.h"

void setPowerlevel(uint8_t powerLevel){
	set_PWM_DutyCycle(1100);
}

