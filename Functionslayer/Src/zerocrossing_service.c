/*
 * zerocrossing_service.c
 *
 *  Created on: Dec 22, 2017
 *      Author: simon
 */

#include "bldc_driver_functions.h"
#include "bldc_driver_HAL.h"
#include "bufferedLogger.h"

static void (*pListener)(volatile uint8_t, volatile uint8_t);

static void (*pListener_zeroCrossing_phaseA)(volatile uint8_t edge);
static void (*pListener_zeroCrossing_phaseB)(volatile uint8_t edge);
static void (*pListener_zeroCrossing_phaseC)(volatile uint8_t edge);

void phaseA_Listener(volatile uint8_t edge);
void phaseB_Listener(volatile uint8_t edge);
void phaseC_Listener(volatile uint8_t edge);

void initZeroCrossingService() {
	log_msg("zero crossing service initialized.");

	register_comperatorListener_phaseA(&phaseA_Listener);
	register_comperatorListener_phaseB(&phaseB_Listener);
	register_comperatorListener_phaseC(&phaseC_Listener);
}

void registerlistener_zeroCrossing_phaseA(void (*listener)(volatile uint8_t)){
	pListener_zeroCrossing_phaseA = listener;
}
void registerlistener_zeroCrossing_phaseB(void (*listener)(volatile uint8_t)){
	pListener_zeroCrossing_phaseB = listener;
}
void registerlistener_zeroCrossing_phaseC(void (*listener)(volatile uint8_t)){
	pListener_zeroCrossing_phaseC = listener;
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
void registerZeroCrossingListener(void (*pListenerParam)(volatile uint8_t, volatile uint8_t)) {
	pListener = pListenerParam;
}

void resetFilter() {
	enableCompA(1);
	enableCompB(1);
	enableCompC(1);
}

void enableZeroCrossingIRQ(uint8_t phase, uint8_t enable) {
	switch (phase) {
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

uint8_t readStatusOfZeroCrossingSignal(uint8_t phase) {
	switch (phase) {
	case PHASE_A:
		return read_signal_compA();
	case PHASE_B:
		return read_signal_compB();
	case PHASE_C:
		return read_signal_compC();
	}

	return 0;
}

// zero crossing ISR handler
void phaseA_Listener(volatile uint8_t edge) {
	//pListener(PHASE_A, edge);

	if(pListener_zeroCrossing_phaseA != 0){
		pListener_zeroCrossing_phaseA(edge);
	}
}
void phaseB_Listener(volatile uint8_t edge) {
	//pListener(PHASE_B, edge);

	if(pListener_zeroCrossing_phaseB != 0){
			pListener_zeroCrossing_phaseB(edge);
		}
}
void phaseC_Listener(volatile uint8_t edge) {
	//pListener(PHASE_C, edge);

	if(pListener_zeroCrossing_phaseC != 0){
			pListener_zeroCrossing_phaseC(edge);
		}
}
