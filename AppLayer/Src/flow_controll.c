/*
 * flow_controll.c
 *
 *  Created on: Nov 30, 2017
 *      Author: simon
 */

#include "bldc_driver_functions.h"
#include "bldc_driver_HAL.h"
#include "drive.h"
#include "logger.h"
#include "bufferedLogger.h"
#include "measurement.h"

uint8_t started = 0;
volatile uint32_t timestamp = 0;

void handle_measurementERROR_ISR(){

}

void startup() {
	/* Selbsthaltung nicht realisiert, deshalb temporär auskommentiert.
	 if (!read_MainButton()) {
	 // no real startup
	 return;
	 };
	 */
	started = 1;
	switch_PowerLED(1);
	switch_MainSwitch(1);

	// init hardware
	initUART();
	initBufferedLogger();

	log_msg("Startup BLDC driver...");

	initSystime();
	initAnalog();


	// init services
	initInterfaceService();
	initZeroCrossingService();
	initPhaseControllService();

	// init rest of software
	initDrive();
	initMeasurement();

	register_rotorPosMeas_listener_ISR(&inform_newRotorPos,
			&informRotorTooEarly,
			&informRotorTooLate);
	register_tooManyZeroCrossings_listener_ISR(&handle_measurementERROR_ISR);

	switch_Enable_BridgeDriver(1);

	set_PWM_DutyCycle(1100);
	changeState(start_up);
}

void proceed() {
	// handle invalid startup
	if (!started) {
		return;
	}

	// measure last cycletime
	/*uint32_t timestamp = getTimestamp();
	if(timestamp != 0){
		log_maxCycleTimeStatistics(1000000, calculateDeltaTime(timestamp));
	}*/

	log_writeBuffered();

	proceedDrive();
	proceedInterfaceService();
}

