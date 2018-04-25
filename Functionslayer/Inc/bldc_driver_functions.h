/*
 * bldc_driver_functions.h
 *
 *  Created on: Nov 29, 2017
 *      Author: simon
 */

#ifndef INC_BLDC_DRIVER_FUNCTIONS_H_
#define INC_BLDC_DRIVER_FUNCTIONS_H_

#include <stdint.h>

//======================= INTERFACE SERVICE ==================================
void initInterfaceService();
void proceedInterfaceService();

uint8_t getDebouncedMainSwitchState();
uint8_t getDebouncedStateSwitchState();
uint32_t getUserInValue();

//========================= TIME ===================================
uint32_t getTimestamp();
uint32_t calculateDeltaTime(uint32_t start_timestamp);

//========================= PHASE CONTROLL ===================================
void initPhaseControllService();

#define START_SIN_APPROX_FORWARD 0
#define START_SIN_APPROX_BACKWARD 1
#define STOP_SIN_APPROX 2
void control3PhaseSinusApproximation(uint8_t start_stop_selecter);
void setSinusApproximation60DegTime(uint32_t t60Deg);
uint32_t getSinusApproximation60DegTime();

#define SECTION_0_ACTIVE 0
#define SECTION_1_ACTIVE 1
#define SECTION_2_ACTIVE 2
#define SECTION_3_ACTIVE 3
#define SECTION_4_ACTIVE 4
#define SECTION_5_ACTIVE 5
#define NO_SECTION_ACTIVE 6
/** Returns the active section.
 *  Return:
    state = 0: A heavyside, C lowside
    state = 1: B heavyside, C lowside
    state = 2: B heavyside, A lowside
    state = 3: C heavyside, A lowside
    state = 4: C heavyside, B lowside
    state = 5: A heavyside, B lowside
    state > 5: power off all channels
 */
uint8_t getActiveSection();

/** Returns the active phase control state.
 *  	Return = 0: sinus approximation is running forward
 *  	Return = 1: sinus approximation is running backward
 *  	Return = 2: sinus approximation is stopped
 */
uint8_t getPhasecontrolState();

void registerSectionChangedListener(void (*pListener)(uint8_t oldSection, uint8_t newSection));
void registerListener_sectionEnds_ISR(void (*pListener)(uint8_t section));

//========================= ZERO CROSSING ===================================
void initZeroCrossingService();

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
void registerZeroCrossingListener(void (*listener)(uint8_t, uint8_t));
#define FALLING_EDGE ZERO_CROSSING_SIGNAL_LOW
#define RISING_EDGE ZERO_CROSSING_SIGNAL_HIGH
#define ZERO_CROSSING_SIGNAL_LOW 0
#define ZERO_CROSSING_SIGNAL_HIGH 1
#define PHASE_A 'A'
#define PHASE_B 'B'
#define PHASE_C 'C'

void enableZeroCrossingIRQ(uint8_t phase, uint8_t enable);
void resetFilter();

uint8_t readStatusOfZeroCrossingSignal(uint8_t phase);

#endif /* INC_BLDC_DRIVER_FUNCTIONS_H_ */
