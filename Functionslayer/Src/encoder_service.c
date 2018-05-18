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
#define NR_OF_TICKS_WHILE_180_DEG 357/2
#define MIN_OFFSET 5


// =============== Variables =============================================
volatile uint32_t offset = 50;
uint8_t idZeroCrossing = 0;

// =============== Function pointers =====================================

// =============== Function declarations =================================
void handle_rotorInRefPos(void);
void handle_180degRotated(void);

// =============== Functions =============================================
void initEncoderService(){
	initEncoder();
	enableIRQ_encoderSignalReferencePos(&handle_rotorInRefPos);
}

void setReferencePositionOffset(uint32_t newOffset){
	offset = newOffset;
}

void togglePin(){
	static uint8_t lastState = 0;

	lastState = !lastState;
	switch_encoderPositionPin(lastState);
}

// ISR's
void handle_rotorInRefPos(void){
	resetNrImpulses_encoderSignalA();
	if(waitForEncoderTicks(1000, &handle_180degRotated) != DELAYED_CALLBACK_REGISTERED){
		//log_msg("error");
	}else{
		//log_msg("gut");
	}
}

void handle_180degRotated(void){
	log_nrImpulsesEncoder(getNrImpulses_encoderSignalA());
	/*idZeroCrossing = (idZeroCrossing + 1) % 12;
	togglePin();

	if(idZeroCrossing < 12){
		uint32_t nrOfTicks = idZeroCrossing*NR_OF_TICKS_WHILE_180_DEG;
		waitForEncoderTicks(nrOfTicks, &handle_180degRotated);
	}*/
}
