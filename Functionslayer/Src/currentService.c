/*
 * currentService.c
 *
 *  Created on: Jul 13, 2018
 *      Author: simon
 */

// =============== Includes ==============================================
#include <stdint.h>

#include "bldc_driver_functions.h"
#include "bldc_driver_HAL.h"
#include "bufferedLogger.h"
#include "utils.h"

// =============== Defines ===============================================
#define NR_MEASUREMENTS 20
#define VOLTAGE_OFFSET 0b111111111111/2 // 12bit ADC
#define MAX_VOLTAGE_IN_mV 3300/2
#define MAX_VALUE_ADC 0b111111111111 // 12 bit ADC

#define CURRENT_TO_VOLTAGE_RATIO 38

// =============== Variables =============================================
uint32_t pBuffer_A[NR_MEASUREMENTS];
uint32_t pBuffer_B[NR_MEASUREMENTS];

uint8_t callbackCnt;

uint32_t lastValue_A;
uint32_t lastValue_B;

static void (*listener_newCurrentData)(int32_t phaseA, int32_t phaseB, int32_t phaseC);

// =============== Function pointers =====================================

// =============== Function declarations =================================
void callback_phaseA(void);
void callback_phaseB(void);

void calculateCurrentOfPhases(void);

// =============== Functions =============================================
void initCurrentServiceService(){
	for(int loopCnt = 0; loopCnt < NR_MEASUREMENTS; loopCnt++){
		pBuffer_A[loopCnt] = 0;
		pBuffer_B[loopCnt] = 0;
	}

	registerListener_newMeasData_hallA(&callback_phaseA);
	registerListener_newMeasData_hallB(&callback_phaseB);
}

void startCurrentMeasurement(){
	callbackCnt = 0;
	start_phaseACurrentMeas_hall(NR_MEASUREMENTS, pBuffer_A);
	start_phaseBCurrentMeas_hall(NR_MEASUREMENTS, pBuffer_B);
}

void calculateCurrentOfPhases(){
	int32_t voltage_phaseA_mV = (lastValue_A - VOLTAGE_OFFSET);
	voltage_phaseA_mV = (voltage_phaseA_mV * MAX_VOLTAGE_IN_mV) / MAX_VALUE_ADC;

	int32_t voltage_phaseB_mV = (lastValue_B - VOLTAGE_OFFSET);
	voltage_phaseB_mV = (voltage_phaseB_mV * MAX_VOLTAGE_IN_mV) / MAX_VALUE_ADC;

	int32_t current_phaseA_mA = voltage_phaseA_mV * CURRENT_TO_VOLTAGE_RATIO;
	int32_t current_phaseB_mA = voltage_phaseB_mV * CURRENT_TO_VOLTAGE_RATIO;
	int32_t current_phaseC_mA = (current_phaseA_mA+current_phaseB_mA)*(-1);

	listener_newCurrentData(current_phaseA_mA, current_phaseB_mA, current_phaseC_mA);
}

void callback_phaseA(void){
	// calc median
	callbackCnt++;
	bubblesort(pBuffer_A, NR_MEASUREMENTS);
	lastValue_A = pBuffer_A[NR_MEASUREMENTS/2];

	if(callbackCnt == 2){
		calculateCurrentOfPhases();
		log_msg("haha");
	}
}

void callback_phaseB(void){
	// calc median
	callbackCnt++;
	bubblesort(pBuffer_B, NR_MEASUREMENTS);
	lastValue_B = pBuffer_B[NR_MEASUREMENTS/2];

	if(callbackCnt == 2){
		calculateCurrentOfPhases();
		log_msg("haha");
	}
}
