/*
 * bldc_driver_functions.h
 *
 *  Created on: Nov 29, 2017
 *      Author: simon
 */

#ifndef INC_BLDC_DRIVER_FUNCTIONS_H_
#define INC_BLDC_DRIVER_FUNCTIONS_H_

//======================= INTERFACE SERVICE ==================================
void initInterfaceService();
void proceedInterfaceService();

uint8_t getDebouncedMainSwitchState();
uint8_t getDebouncedStateSwitchState();
uint32_t getUserInValue();

//========================= TIME ===================================
/** Starts a new time measurement.
resolution = 1/(16e6/64) = 4us
max. time = 4us * 2¹⁶ = 262.2ms

Input:
timerOverflowCallback: called after the max. time
*/
void startTimeMeasurement(void (*timerOverflowCallback)(void));

/** Returns a 1, if there is a running time measurement
*/
uint8_t isTimeMeasurementRunning();

/** Stops the time measurement and returns the measured time.
 return: measured time in us.
*/
uint32_t stopTimeMeasurement();

uint32_t getTime();

//========================= PHASE CONTROLL ===================================
#define START_SIN_APPROX_FORWARD 0
#define START_SIN_APPROX_BACKWARD 1
#define STOP_SIN_APPROX 2
void control3PhaseSinusApproximation(uint8_t start_stop_selecter);
void setSinusApproximationPeriod(uint32_t period);

/** Returns the active state.
 *  Return:
    state = 0: A heavyside, C lowside
    state = 1: B heavyside, C lowside
    state = 2: B heavyside, A lowside
    state = 3: C heavyside, A lowside
    state = 4: C heavyside, B lowside
    state = 5: A heavyside, B lowside
    state > 5: power off all channels
 */
uint8_t getPhaseState();

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
void registerZeroCrossingListener(void (*listener)(char, char));


/** Enables the comperator for the phase A.
    Input:
    enable = 1: enable comperator
    enable = 0: disable comperator
**/
void setEnableCompA(char enable);
void setEnableCompB(char enable);
void setEnableCompC(char enable);
#endif /* INC_BLDC_DRIVER_FUNCTIONS_H_ */
