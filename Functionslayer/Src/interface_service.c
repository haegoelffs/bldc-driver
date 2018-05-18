/*
 * interface_service.c
 *
 *  Created on: Nov 30, 2017
 *      Author: simon
 */

#include "bldc_driver_functions.h"
#include "bldc_driver_HAL.h"

#define DEBOUNCE_HYSTERESIS 10

uint8_t mainSwitchState = 0;
uint8_t stateSwitchState = 0;

uint32_t userInValue = 0;

void debounceMainSwitch() {
	//static int8_t cnt = 0;

	/*if (read_MainButton()) {
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
	}*/
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
void pollAnalogUserInput() {
	if (isMeasReady_userVolatgeMeas) {
		userInValue = getLastMeas_userVolatgeMeas();
		start_userVolatgeMeas();
	}
}

uint8_t getDebouncedMainSwitchState() {
	return mainSwitchState;
}

uint8_t getDebouncedStateSwitchState() {
	return stateSwitchState;
}

uint32_t getUserInValue() {
	return userInValue;
}

void flashNextLED() {
	static uint8_t led = 0;

	switch_StatusLED1(0);
	switch_StatusLED2(0);
	switch_StatusLED3(0);
	switch_StatusLED4(0);

	switch (led) {
	case 0:
		switch_StatusLED1(1);
		break;
	case 1:
		switch_StatusLED2(1);
		break;
	case 2:
		switch_StatusLED3(1);
		break;
	case 3:
		switch_StatusLED4(1);
		break;
	}

	led = (led + 1) % 4;
}

void readOutBridgeDriverPins(){
	// fault report indicator
	uint8_t nfault = read_NFault_BridgeDriver();

	// overcurrent and over temperature warning
	uint8_t noctw = read_NOCTW_BridgeDriver();

	// buck output voltage is low
	uint8_t pwrgd = read_PWRGD_BridgeDriver();

	switch_StatusLED1(!nfault);
	switch_StatusLED2(!noctw);
	switch_StatusLED3(!pwrgd);
}

void proceedInterfaceService() {
	debounceMainSwitch();
	debounceStateSwitch();
	pollAnalogUserInput();
}

void initInterfaceService() {

	start_userVolatgeMeas();

	flashNextLED();
	HAL_Delay(100);
	flashNextLED();
	HAL_Delay(100);
	flashNextLED();
	HAL_Delay(100);
	flashNextLED();
	HAL_Delay(100);

	switch_StatusLED4(0);
}
