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

void handle_measurementERROR_ISR(){
	switch_StatusLED4(1);

	//changeState(free_running);
}

volatile uint32_t timestamp = 0;
volatile uint8_t isrCalled = 0;

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
	log_msg("Systime initalized.");
	initAnalog();
	log_msg("ADC initalized.");


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

	/*spi_readStatusRegisters_BLOCKING();

	uint16_t reg1 = getLastStatusRegister1Value();
	logUnsignedDEBUG("reg1", reg1);
	uint16_t reg2 = getLastStatusRegister1Value();
	logUnsignedDEBUG("reg2", reg2);*/

	set_PWM_DutyCycle(1100);
	changeState(start_up);
}

void proceed() {
	// handle invalid startup
	if (!started) {
		return;
	}

	// measure last cycletime
	uint32_t timestamp = getTimestamp();
	if(timestamp != 0){
		log_maxCycleTimeStatistics(1000000, calculateDeltaTime(timestamp));
	}

	/*start_mainVoltageMeas();
	if (newDataAvailable_mainVolatgeMeas()) {
		volatile uint32_t voltage = getLastData_mainVoltageMeas();
		//logUnsignedDEBUG("mainVoltage", voltage);
		//logDEBUG("hallo");

		set_virtualGNDValue(voltage*0.36);
		enable_virtualGND(1);
	}*/

	/*start_userVolatgeMeas();
	if (newDataAvailable_userVolatgeMeas()) {
		uint32_t voltage = getLastMeas_userVolatgeMeas();
		logUnsignedDEBUG("user voltage", voltage);
	}*/

	log_writeBuffered();

	proceedDrive();
	proceedInterfaceService();
}

