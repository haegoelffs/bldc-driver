/*
 * flow_controll.c
 *
 *  Created on: Nov 30, 2017
 *      Author: simon
 */

#include "bldc_driver_HAL.h"
#include "bldc_driver_functions.h"
#include "logger.h"

uint8_t started = 0;

volatile uint32_t timestamp = 0;
volatile uint8_t isrCalled = 0;

void timertest() {
	timestamp = getSystimeUs() - timestamp;
	isrCalled = 1;
}

void startup() {
	/* Selbsthaltung Harwareseitig nicht realisiert, deshalb temporär auskommentiert.
	 if (!read_MainButton()) {
	 // no real startup
	 return;
	 };
	 */
	started = 1;
	switch_PowerLED(1);
	switch_MainSwitch(1);

	initUART();
	logINFO("Startup BLDC driver...");
	initSystime();
	logINFO("Systemtime initalized.");

	initAnalog();
	initInterfaceService();

	uint32_t period = 50000;
	logINFO("Start after will be called");
	timestamp = getSystimeUs();
	startAfterUs(period, &timertest);
	logINFO("Start after called");



	//set_PWM_DutyCycle(900);
	//enable_PWM_phaseA_HS(1);
	//enable_PWM_phaseA_LS(1);
	//enable_PWM_phaseB_HS(1);
	//enable_PWM_phaseB_LS(1);
	//enable_PWM_phaseC_HS(1);
	//enable_PWM_phaseC_LS(1);
	//logINFO("PWM enabled");

	//setSinusApproximationPeriod(50000);
	//control3PhaseSinusApproximation(START_SIN_APPROX_FORWARD);
}

void proceed() {
	if (!started) {
		return;
	}

	/*transmitStringOverUART("proceed");
	 writeNewLine();*/
	HAL_Delay(2000);
	logDEBUG("hartbeat");
	//logUnsignedDEBUG("timer", getElapsedTimeInUs());

	if(isrCalled){
			logINFO("ISR called");
			logUnsignedINFO("delta t", timestamp);
		}

	/*if (getDebouncedMainSwitchState()) {
	 // user want to shutdown
	 started = 0;
	 switch_PowerLED(0);
	 switch_MainSwitch(0);
	 return;
	 }*/

	proceedInterfaceService();
}

