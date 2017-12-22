/*
 * phasecontrol_service.c
 *
 *  Created on: Dec 15, 2017
 *      Author: simon
 */

#include "bldc_driver_HAL.h"
#include "bldc_driver_functions.h"

static uint8_t activeState = STOP_SIN_APPROX;
static uint8_t phaseState = 6; // power off all
static uint32_t period = 0;
static uint32_t sixtyDegTime = 0;

/** Changes the output channels for the pwm.
 Input:
 state = 0: A heavyside, C lowside
 state = 1: B heavyside, C lowside
 state = 2: B heavyside, A lowside
 state = 3: C heavyside, A lowside
 state = 4: C heavyside, B lowside
 state = 5: A heavyside, B lowside
 state > 5: power off all channels
 **/
void changePhaseState(uint8_t state);
void switchPhases(uint8_t forward_backward_selecter);
void timerCallback();

void changePhaseState(uint8_t newPhaseState) {
	enable_PWM_phaseA_HS(0);
	enable_PWM_phaseB_HS(0);
	enable_PWM_phaseC_HS(0);
	enable_PWM_phaseA_LS(0);
	enable_PWM_phaseB_LS(0);
	enable_PWM_phaseC_LS(0);

	switch (newPhaseState) {
	case 0:
		enable_PWM_phaseA_HS(1);
		enable_PWM_phaseC_LS(1);
		break;

	case 1:
		enable_PWM_phaseB_HS(1);
		enable_PWM_phaseC_LS(1);
		break;

	case 2:
		enable_PWM_phaseB_HS(1);
		enable_PWM_phaseA_LS(1);
		break;

	case 3:
		enable_PWM_phaseC_HS(1);
		enable_PWM_phaseA_LS(1);
		break;

	case 4:
		enable_PWM_phaseC_HS(1);
		enable_PWM_phaseB_LS(1);
		break;

	case 5:
		enable_PWM_phaseA_HS(1);
		enable_PWM_phaseB_LS(1);
		break;

	default:
		break;
	}

	phaseState = newPhaseState;
}

void switchPhases(uint8_t forward_backward_selecter) {
	static uint8_t phasestate;

	switch (forward_backward_selecter) {
	case START_SIN_APPROX_FORWARD:
		phasestate = (phasestate + 1) % 6;
		changePhaseState(phasestate);
		break;

	case START_SIN_APPROX_BACKWARD:
		phasestate = (phasestate - 1) % 6;
		changePhaseState(phasestate);
		break;

	default:
		break;
	}
}

void control3PhaseSinusApproximation(uint8_t start_stop_selecter){
	if(start_stop_selecter != STOP_SIN_APPROX){
		startAfterUs(sixtyDegTime, &timerCallback);
	}
	activeState = start_stop_selecter;
}

void setSinusApproximationPeriod(uint32_t periodParam){
	period = periodParam;
	sixtyDegTime = periodParam/6;
}

void timerCallback(){
	if(activeState == START_SIN_APPROX_FORWARD ||
			activeState == START_SIN_APPROX_BACKWARD){
		switchPhases(activeState);
		startAfterUs(sixtyDegTime, &timerCallback);
	}
}
