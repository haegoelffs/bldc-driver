/*
 * drive.c
 *
 *  Created on: Dec 25, 2017
 *      Author: simon
 */

#include "drive.h"
#include "drive_state.h"
#include "bldc_driver_functions.h" // temp!!! --> weiter abstrahieren!

// =============== Variables =============================================
DriveState activeState = free_running;

// =============== Function Pointers =====================================


// =============== Function Declarations =================================
void handle_startUpStoped_callback(uint32_t time60deg);
void handle_controlledStoped_callback();
void handle_freeRunningStoped_callback();

// =============== Functions =============================================
void initDrive(){
	logDEBUG("init drive...");
	startFreeRunning(&handle_freeRunningStoped_callback);
}
void proceedDrive(){
	switch(activeState){
		case start_up:
			proceedStartUp();
			break;
		case free_running:
			stopFreeRunning();
			break;
		case controlled_negative_torque:
			break;
		case controlled_positive_torque:
			break;
		}
}

void inform_newRotorPos(uint32_t time){
	switch(activeState){
		case start_up:
			break;
		case free_running:
			break;
		case controlled_negative_torque:
			break;
		case controlled_positive_torque:
			informRotorPos_controlled(time);
			break;
		}
}
void inform_newRotationDirection(uint8_t direction){

}
void inform_tooManyZeroCrossings(){
	switch(activeState){
			case start_up:
				break;
			case free_running:
				break;
			case controlled_negative_torque:
				break;
			case controlled_positive_torque:
				stopControlled();
				break;
			}
}
void changeState(DriveState newState){
	switch(activeState){
	case start_up:
		break;
	case free_running:
		stopFreeRunning();
		break;
	case controlled_negative_torque:
		break;
	case controlled_positive_torque:
		stopControlled();
		break;
	}
	activeState = newState;
}

// callbacks
void handle_startUpStoped_callback(uint32_t time60deg){
	activeState = controlled_positive_torque;
	startControlled(time60deg, &handle_controlledStoped_callback);
}
void handle_controlledStoped_callback(uint32_t lastTime60deg){

}
void handle_freeRunningStoped_callback(){
	startStartUp(&handle_startUpStoped_callback);
}
