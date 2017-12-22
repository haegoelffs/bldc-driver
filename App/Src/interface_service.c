/*
 * interface_service.c
 *
 *  Created on: Nov 30, 2017
 *      Author: simon
 */

#include "bldc_driver_HAL.h"
#include "bldc_driver_functions.h"

#define DEBOUNCE_HYSTERESIS 10

uint8_t mainSwitchState = 0;
uint8_t stateSwitchState = 0;

uint32_t userInValue = 0;

void debounceMainSwitch() {
	static int8_t cnt = 0;

	if (read_MainButton()) {
		cnt++;
	} else {
		cnt--;
	}

	if (cnt <= 0) {
		cnt = 0;
		mainSwitchState = 0;
	} else if (cnt >= DEBOUNCE_HYSTERESIS) {
		cnt = DEBOUNCE_HYSTERESIS;
		mainSwitchState = 1;
	}
}
void debounceStateSwitch() {
	static int8_t cnt = 0;

	if (read_StateButton()) {
		cnt++;
	} else {
		cnt--;
	}

	if (cnt <= 0) {
		cnt = 0;
		stateSwitchState = 0;
	} else if (cnt >= DEBOUNCE_HYSTERESIS) {
		cnt = DEBOUNCE_HYSTERESIS;
		stateSwitchState = 1;
	}
}
void pollAnalogUserInput(){
	if(isMeasReady_userVolatgeMeas){
		userInValue = getLastMeas_userVolatgeMeas();
		start_userVolatgeMeas();
	}
}

void initInterfaceService() {
	start_userVolatgeMeas();
}

void proceedInterfaceService() {
	debounceMainSwitch();
	debounceStateSwitch();
	pollAnalogUserInput();
}

uint8_t getDebouncedMainSwitchState(){
	return mainSwitchState;
}

uint8_t getDebouncedStateSwitchState(){
	return stateSwitchState;
}

uint32_t getUserInValue(){
	return userInValue;
}
