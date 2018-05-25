/*
 * drive.c
 *
 *  Created on: Dec 25, 2017
 *      Author: simon
 */

#include "drive.h"

#include "drive_state.h"
#include "bufferedLogger.h"
#include "measurement.h"
#include "pwrControl.h"

#include "bldc_driver_functions.h"
#include "bldc_driver_HAL.h"

// =============== Defines =============================================
#define GRADIENT_DIVIDER 128
#define TIME_60DEG_SPEED_UP_START 10000
#define TIME_60DEG_SPEED_UP_END 1500

#define TIMING 0
#define MAX_T_60_DEG 5000

// _controller options____________________________________________________
#define P_DIVIDER 80
#define I_DIVIDER 60000
//#define D_DIVIDER 32768

// =============== Variables =============================================
DriveState activeState = off;
uint8_t started = 0;

static uint32_t time60deg;
static uint32_t timestamp_entryState;

// =============== Function Pointers =====================================

// =============== Function Declarations =================================

// =============== Functions =============================================
void initDrive() {
	log_msg("init drive...");
	changeState(off);
}

void proceedController(volatile uint32_t rotorpos) {
	volatile uint32_t targetTime = (time60deg * (30 - TIMING)) / 60;

	volatile int32_t fault = (targetTime - rotorpos);

	volatile int32_t controllerOut = (fault / P_DIVIDER);

#ifdef I_DIVIDER
	// integrator active
	volatile static int32_t fault_I;
	fault_I = fault_I + fault;
	controllerOut += fault_I / I_DIVIDER;
#endif
#ifdef D_DIVIDER
	// differentiator active
	static int32_t last_fault;
	controllerOut += (fault-last_fault)/D_DIVIDER;
#endif

	time60deg = time60deg - controllerOut;

	setSinusApproximation60DegTime(time60deg);

	if (time60deg > MAX_T_60_DEG) {
		// too slow
		control3PhaseSinusApproximation(STOP_SIN_APPROX);
		changeState(off);
	}

	//log_controllerParameterTuple_mr(5, time60deg, rotorpos, targetTime, controllerOut);
	log_controllerParameterTuple(time60deg, rotorpos, targetTime,
			controllerOut);
}

// statemachine & transitions
void entryState(DriveState state) {
	switch (state) {
	case off:
		log_msg("off state active");

		switch_Enable_BridgeDriver(0);
		enableMeasurement(0);
		setPowerlevel(OFF_POWER_LEVEL);
		control3PhaseSinusApproximation(STOP_SIN_APPROX);
		break;

	case stopped:
		log_msg("stopped state active");
		break;

	case start_up:
		log_msg("start up state active");

		time60deg = TIME_60DEG_SPEED_UP_START; //us

		setSinusApproximation60DegTime(time60deg);

		switch_Enable_BridgeDriver(1);
		setPowerlevel(MAX_POWER_LEVEL);
		control3PhaseSinusApproximation(START_SIN_APPROX_FORWARD);
		break;

	case synchronized:
		log_msg("synchronized state active");
		break;

	case controlled_negative_torque:
		log_msg("controlled negative torque state active");
		break;

	case controlled_positive_torque:
		log_msg("controlled positive torque state active");

		enableMeasurement(1);
		break;

	case calibrate_encoder:
		log_msg("calibrate encoder state active");
		setPowerLED_blinkingMode();
		initEncoderService();
		break;
	}
}

void exitState(DriveState state) {
	switch (state) {
	case off:
		break;
	case stopped:
		break;
	case start_up:
		break;
	case synchronized:
		break;
	case controlled_negative_torque:
		break;
	case controlled_positive_torque:
		enableMeasurement(0);
		break;
	case calibrate_encoder:

			break;
	}
}

void changeState(DriveState newState) {
	exitState(activeState);

	timestamp_entryState = getTimestamp();
	entryState(newState);

	activeState = newState;
}

// callbacks
void inform_newRotorPos(uint32_t time) {
	switch (activeState) {
	case controlled_positive_torque:
		//proceedController(time);
		//setSinusApproximation60DegTime(time60deg);

		/*if (time60deg > MAX_T_60_DEG) {
		 // too slow
		 control3PhaseSinusApproximation(STOP_SIN_APPROX);
		 changeState(off);
		 }*/
		break;
	default:
		// do nothing
		break;
	}
}
void informRotorTooEarly() {
	switch (activeState) {
	case controlled_positive_torque:
		proceedController(0);
		break;
	default:
		// do nothing
		break;
	}
}
void informRotorTooLate() {
	switch (activeState) {
	case controlled_positive_torque:
		proceedController(time60deg);
		break;
	default:
		// do nothing
		break;
	}
}
void inform_newRotationDirection(uint8_t direction) {

}

void inform_tooManyZeroCrossings() {
	switch (activeState) {
	case controlled_positive_torque:
		//changeState(off);
		break;
	default:
		// do nothing
		break;
	}
}

void startup() {
	/*if (!read_MainButton()) {
	 // no real startup
	 return;
	 };*/
	started = 1;

	//switch_PowerLED(1);

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
			&informRotorTooEarly, &informRotorTooLate);

	if(read_encoderCalibrate()){
		changeState(calibrate_encoder);
	} else if(read_encoderEnable()){

	} else{
		changeState(start_up);
	}
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



	switch (activeState) {
		case off:
			break;
		case stopped:
			break;
		case start_up:
			;
			uint32_t deltaTime = calculateDeltaTime(timestamp_entryState);
			time60deg = (TIME_60DEG_SPEED_UP_START - deltaTime / GRADIENT_DIVIDER);
			setSinusApproximation60DegTime(time60deg);

			log_time60Deg(time60deg);

			if (time60deg < TIME_60DEG_SPEED_UP_END) {
				changeState(controlled_positive_torque);
			}
			break;
		case synchronized:
			break;
		case controlled_negative_torque:
			break;
		case controlled_positive_torque:

			break;
		case calibrate_encoder:;
			uint32_t poti = getReferencePositionEncoder();
			log_unnamedUint(poti);
			setReferencePosition(poti);
			break;
	}




	proceedInterfaceService();

	log_writeBuffered();
}
