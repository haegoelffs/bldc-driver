/*
 * time_service.c
 *
 *  Created on: Dec 15, 2017
 *      Author: simon
 */

#include "bldc_driver_HAL.h"
#include "bldc_driver_functions.h"


/** Starts a new time measurement.
resolution = 1/(16e6/64) = 4us
max. time = 4us * 2¹⁶ = 262.2ms

Input:
timerOverflowCallback: called after the max. time
*/
void startTimeMeasurement(void (*timerOverflowCallback)(void)){

}

/** Returns a 1, if there is a running time measurement
*/
uint8_t isTimeMeasurementRunning(){

}

/** Stops the time measurement and returns the measured time.
 return: measured time in us.
*/
uint32_t stopTimeMeasurement(){

}

uint32_t getTime(){

}
