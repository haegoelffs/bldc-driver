/*
 * time_service.c
 *
 *  Created on: Dec 15, 2017
 *      Author: simon
 */

#include "bldc_driver_functions.h"
#include "bldc_driver_HAL.h"


uint32_t getTimestamp(){
	return getSystimeUs();
}

uint32_t calculateDeltaTime(uint32_t start_timestamp){
	return getSystimeUs() - start_timestamp;
}
