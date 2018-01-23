/*
 * zerocrossing_service.c
 *
 *  Created on: Dec 22, 2017
 *      Author: simon
 */

#include "bldc_driver_functions.h"
#include "bldc_driver_HAL.h"
#include "logger.h"

static void (*pListener)(uint8_t, uint8_t);

void phaseA_Listener(uint8_t edge);
void phaseB_Listener(uint8_t edge);
void phaseC_Listener(uint8_t edge);

void initZeroCrossingService(){
	logINFO("zero crossing service initialized.");

	register_comperatorListener_phaseA(&phaseA_Listener);
	register_comperatorListener_phaseB(&phaseB_Listener);
	register_comperatorListener_phaseC(&phaseC_Listener);
}

/** Register the handed function as listener which is called when the voltage of one phase crosses zero
    Input:
    listener = function with parameter.
    phase:
    	--> phase = 'A': phase A
    	--> phase = 'B': phase B
    	--> phase = 'C': phase C
    edge:
        --> edge = 0: falling edge
        --> edge = 1: rising edge
**/
void registerZeroCrossingListener(void (*pListenerParam)(uint8_t, uint8_t)){
	pListener = pListenerParam;
}

void resetFilter(){
	enableCompA(1);
	enableCompB(1);
	enableCompC(1);
}

void enableZeroCrossingIRQ(uint8_t phase, uint8_t enable){
	switch(phase){
	case PHASE_A:
		enableCompA(enable);
		break;
	case PHASE_B:
		enableCompB(enable);
		break;
	case PHASE_C:
		enableCompC(enable);
		break;
	}
}

// zero crossing ISR handler
void phaseA_Listener(uint8_t edge){
	pListener(PHASE_A, edge);
}
void phaseB_Listener(uint8_t edge){
	pListener(PHASE_A, edge);
}
void phaseC_Listener(uint8_t edge){
	pListener(PHASE_A, edge);
}
