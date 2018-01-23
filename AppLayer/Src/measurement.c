/*
 * measurement.c
 *
 *  Created on: Dec 24, 2017
 *      Author: simon
 */

#include "measurement.h"

#include "bldc_driver_functions.h"

// =============== Defines ===============================================
#define MAX_ZERO_CROSSINGS 5

// =============== Variables =============================================
static uint8_t zeroCrossingCounter = 0;
static uint32_t zeroCrossingTimestamp = 0;

// =============== Function pointers =====================================
void (*pNewRotorPositionMeasurement_listener_ISR)(uint32_t);
void (*pTooManyZeroCrossings_listener_ISR)(void);

// =============== Function declarations =================================
void handleZeroCrossing(uint8_t phase, uint8_t edge);
void handleNewSectionActive(uint8_t state);

// =============== Functions =============================================
void initMeasurement() {
	registerZeroCrossingListener(&handleZeroCrossing);
	registerSectionChangedListener(&handleNewSectionActive);
}

void register_newRotorPos_listener_ISR(void (*pListener)(uint32_t)) {
	pNewRotorPositionMeasurement_listener_ISR = pListener;
}

void register_tooManyZeroCrossings_listener_ISR(void (*listener)(void)){
	pTooManyZeroCrossings_listener_ISR = listener;
}

// listeners
void handleZeroCrossing(uint8_t phase, uint8_t edge) {
	switch (getActiveSection()) {
	case SECTION_0_ACTIVE:
		if (phase == PHASE_B && edge == RISING_EDGE) {
			zeroCrossingTimestamp = getTimestamp();
			zeroCrossingCounter++;
		}
		break;
	case SECTION_1_ACTIVE:
		if (phase == PHASE_A && edge == FALLING_EDGE) {
			zeroCrossingTimestamp = getTimestamp();
			zeroCrossingCounter++;
		}
		break;
	case SECTION_2_ACTIVE:
		if (phase == PHASE_C && edge == RISING_EDGE) {
			zeroCrossingTimestamp = getTimestamp();
			zeroCrossingCounter++;
		}
		break;
	case SECTION_3_ACTIVE:
		if (phase == PHASE_B && edge == FALLING_EDGE) {
			zeroCrossingTimestamp = getTimestamp();
			zeroCrossingCounter++;
		}
		break;
	case SECTION_4_ACTIVE:
		if (phase == PHASE_A && edge == RISING_EDGE) {
			zeroCrossingTimestamp = getTimestamp();
			zeroCrossingCounter++;
		}
		break;
	case SECTION_5_ACTIVE:
		if (phase == PHASE_C && edge == FALLING_EDGE) {
			zeroCrossingTimestamp = getTimestamp();
			zeroCrossingCounter++;
		}
		break;
	default:
		// run synchronisation
		break;
	}
}

void handleNewSectionActive(uint8_t section) {
	if (section != NO_SECTION_ACTIVE) {
		if (zeroCrossingCounter > MAX_ZERO_CROSSINGS) {
			// error!
			pTooManyZeroCrossings_listener_ISR();
		}
		if (zeroCrossingTimestamp > 0) {
			// valid measurement
			uint32_t deltaTime = calculateDeltaTime(zeroCrossingTimestamp);
			pNewRotorPositionMeasurement_listener_ISR(deltaTime);
		}

		zeroCrossingCounter = 0;
		zeroCrossingTimestamp = 0;

		// enable disable zero crossing interrupts
		enableZeroCrossingIRQ(PHASE_A, 0);
		enableZeroCrossingIRQ(PHASE_B, 0);
		enableZeroCrossingIRQ(PHASE_C, 0);

		switch (getActiveSection()) {
			case SECTION_0_ACTIVE:
			case SECTION_3_ACTIVE:
				enableZeroCrossingIRQ(PHASE_B, 1);
				break;
			case SECTION_1_ACTIVE:
			case SECTION_4_ACTIVE:
				enableZeroCrossingIRQ(PHASE_A, 1);
				break;
			case SECTION_2_ACTIVE:
			case SECTION_5_ACTIVE:
				enableZeroCrossingIRQ(PHASE_C, 1);
				break;
			default:
				// run synchronisation
				break;
			}
	}
}
