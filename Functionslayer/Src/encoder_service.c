/*
 * encoder_service.c
 *
 *  Created on: May 11, 2018
 *      Author: simon
 */

// =============== Includes ==============================================
#include "bldc_driver_functions.h"
#include "bldc_driver_HAL.h"
#include "bufferedLogger.h"
// =============== Defines ===============================================
#define UNREFERENCED -1

// =============== Variables =============================================
uint32_t referencePosition = 0;
uint32_t timestamp = 0;

// =============== Function pointers =====================================

// =============== Function declarations =================================
void handle_rotorInRefPos(void);
void handle_togglePin(void);

// =============== Functions =============================================
void initEncoderService(){
	initEncoder();
	enableIRQ_encoderSignalReferencePos(&handle_rotorInRefPos);
}

void setReferencePosition(uint32_t position){
	referencePosition = position;
}

// ISR's
void handle_togglePin(void){
	static uint8_t lastState = 0;

	lastState = !lastState;
	switch_encoderPositionPin(lastState);
}

void handle_rotorInRefPos(void){
	resetNrImpulses_encoderSignalA();

	if(timestamp != 0){
		uint32_t t_rot_us = calculateDeltaTime(timestamp)/7;
		/* calculate min. t_rot_us:
		 * max. rotations per minute = 4000
		 * -> max. mech. frequenzy: 4000/60 = 66.7 Hz
		 * -> max. electric frequenzy: 66.7*7 = 466.9 Hz
		 * -> min. t_rot_us: 1/466.9 = 2'142us
		 */
		if(t_rot_us > 2142 && t_rot_us <= 8388608){
			uint32_t t_cal_us = (referencePosition*t_rot_us)/360;
			/* check overflow:
			 * max. size referencePosition: 360 = 9bit
			 * max. size t_rot_us: 32bit - 9bit = 23bit --> 8'388'608us
			 */
			uint32_t t_delay_us = t_cal_us + 2.5*t_rot_us;

			delayedCallback_A(t_delay_us, &handle_togglePin);
		}
	}

	timestamp = getSystimeUs();
}
