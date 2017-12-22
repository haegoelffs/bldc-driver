/*
 * zerocrossing_service.c
 *
 *  Created on: Dec 22, 2017
 *      Author: simon
 */

#include "bldc_driver_HAL.h"
#include "bldc_driver_functions.h"

static uint8_t lastZC_phase = 0;
static uint8_t lastZC_type = 0; // rising or sinking

static void (*listener)(char, char);

void initZeroCrossingService(){
	//register_comperatorListener_phaseA()
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
void registerZeroCrossingListener(void (*listenerParam)(char, char)){
	listener = listenerParam;
}


/** Enables the comperator for the phase A.
    Input:
    enable = 1: enable comperator
    enable = 0: disable comperator
**/
void setEnableCompA(char enable){

}
void setEnableCompB(char enable){

}
void setEnableCompC(char enable){

}
