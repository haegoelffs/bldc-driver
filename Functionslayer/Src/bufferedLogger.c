// =============== Includes ==============================================
#include <stdint.h>
#include <stdlib.h>

#include "bldc_driver_functions.h"
#include "bldc_driver_HAL.h"
#include "ringbuffer.h"
#include "bufferedLogger.h"

// =============== Defines ===============================================

// ascii
#define STX 2
#define ETX 3

#define GROUP_SEPERATOR 29
#define RECORD_SEPERATOR 30

// types
#define TYPE_INT '1'
#define TYPE_STRING '2'

// names
#define NAME_GENERIC_MESSAGE '1'

#define NAME_CURRENT_PHASE_A_SHUNT '2'
#define NAME_CURRENT_PHASE_A_HALL '3'
#define NAME_CURRENT_PHASE_B_SHUNT '4'
#define NAME_CURRENT_PHASE_B_HALL '5'
#define NAME_CURRENT_SETPOINT '6'
#define NAME_CURRENT_CONTROL_OUTPUT '7'

#define NAME_VOLTAGE_PHASE_A '8'
#define NAME_VOLTAGE_PHASE_B '9'
#define NAME_VOLTAGE_PHASE_C 'A'

#define NAME_STATE_DRIVE 'B'
#define NAME_ROTATION_FREQUENZY 'C'

#define NAME_ROTORPOSITION 'D'
#define NAME_ROTORPOSITION_SETPOINT 'E'
#define NAME_ROTORPOSITION_CONTROL_OUTPUT 'F'

#define NAME_CYCLE_TIME 'G'

#define ZERO_CROSSINGS 'H'

// =============== Variables =============================================
Ringbuffer *pRingbuffer;

// =============== Function pointers =====================================

// =============== Function declarations =================================
void addUnsignedToRingbuffer(uint32_t number);
void addSignedToRingbuffer(int32_t number);
void addMsgToRingbuffer(char *pMsg);

// =============== Functions =============================================
void initBufferedLogger() {
	pRingbuffer = allocRingbuffer(2000);
}

//<STX>timestamp<GS>data_name<RS>data_type<RS>data<GS><ETX>
void log_msg(char *pMsg) {
#ifdef LOG_MSG
	bufferIn(pRingbuffer, (uint32_t) STX);
	addUnsignedToRingbuffer(getTimestamp());
	bufferIn(pRingbuffer, (uint32_t) GROUP_SEPERATOR);
	bufferIn(pRingbuffer, (uint32_t) NAME_GENERIC_MESSAGE);
	bufferIn(pRingbuffer, (uint32_t) RECORD_SEPERATOR);
	bufferIn(pRingbuffer, (uint32_t) TYPE_STRING);
	bufferIn(pRingbuffer, (uint32_t) RECORD_SEPERATOR);
	addMsgToRingbuffer(pMsg);
	bufferIn(pRingbuffer, (uint32_t) GROUP_SEPERATOR);
	bufferIn(pRingbuffer, (uint32_t) ETX);
#endif
}

void log_time60Deg(uint32_t t60Deg){
#ifdef LOG_TIME60DEG
	bufferIn(pRingbuffer, (uint32_t) STX);
	addUnsignedToRingbuffer(getTimestamp());
	bufferIn(pRingbuffer, (uint32_t) GROUP_SEPERATOR);
	bufferIn(pRingbuffer, (uint32_t) NAME_ROTATION_FREQUENZY);
	bufferIn(pRingbuffer, (uint32_t) RECORD_SEPERATOR);
	bufferIn(pRingbuffer, (uint32_t) TYPE_INT);
	bufferIn(pRingbuffer, (uint32_t) RECORD_SEPERATOR);
	addUnsignedToRingbuffer(t60Deg);
	bufferIn(pRingbuffer, (uint32_t) GROUP_SEPERATOR);
	bufferIn(pRingbuffer, (uint32_t) ETX);
#endif
}

void log_time60Deg_mr(uint32_t max_resolution_us, uint32_t t60Deg){
#ifdef LOG_TIME60DEG
	static uint32_t last_timestamp_us;
		uint32_t timestamp_us = getElapsedTimeInUs();
		if((timestamp_us - last_timestamp_us) >= max_resolution_us){
			last_timestamp_us = timestamp_us;
			log_time60Deg(t60Deg);
		}
#endif
}

void log_cycleTime(uint32_t cycleTime){
#ifdef LOG_TIME60DEG
	bufferIn(pRingbuffer, (uint32_t) STX);
	addUnsignedToRingbuffer(getTimestamp());
	bufferIn(pRingbuffer, (uint32_t) GROUP_SEPERATOR);
	bufferIn(pRingbuffer, (uint32_t) NAME_CYCLE_TIME);
	bufferIn(pRingbuffer, (uint32_t) RECORD_SEPERATOR);
	bufferIn(pRingbuffer, (uint32_t) TYPE_INT);
	bufferIn(pRingbuffer, (uint32_t) RECORD_SEPERATOR);
	addUnsignedToRingbuffer(cycleTime);
	bufferIn(pRingbuffer, (uint32_t) GROUP_SEPERATOR);
	bufferIn(pRingbuffer, (uint32_t) ETX);
#endif
}

void log_maxCycleTimeStatistics(uint32_t max_resolution_us, uint32_t cycleTime){
#ifdef LOG_CYCLETIME
	static uint32_t max_cycleTime;
	static uint32_t last_timestamp_us;

	if(cycleTime > max_cycleTime){
		max_cycleTime = cycleTime;
	}

	uint32_t timestamp_us = getElapsedTimeInUs();
	if((timestamp_us - last_timestamp_us) >= max_resolution_us){
		last_timestamp_us = timestamp_us;
		log_cycleTime(max_cycleTime);
		max_cycleTime = 0;
	}
#endif
}

// <STX>timestamp<GS>data_name<RS>data_type<RS>data<GS><ETX>
// <STX>6603375<GS>
//		C<RS>1<RS>5014<GS>
//		D<RS>1<RS>62<GS>
//		E<RS>1<RS>2495<GS>
//		F<RS>1<RS>4294967272<GS>
// <ETX>
void log_controllerParameterTuple(uint32_t t60Deg, uint32_t rotorpos, uint32_t rotorpos_setpoint, int32_t controller_out){
#ifdef LOG_CONTROLLER_PARAMETER_TUPLE
	bufferIn(pRingbuffer, (uint32_t) STX);
	addUnsignedToRingbuffer(getTimestamp());

	bufferIn(pRingbuffer, (uint32_t) GROUP_SEPERATOR);
	bufferIn(pRingbuffer, (uint32_t) NAME_ROTATION_FREQUENZY);
	bufferIn(pRingbuffer, (uint32_t) RECORD_SEPERATOR);
	bufferIn(pRingbuffer, (uint32_t) TYPE_INT);
	bufferIn(pRingbuffer, (uint32_t) RECORD_SEPERATOR);
	addUnsignedToRingbuffer(t60Deg);

	bufferIn(pRingbuffer, (uint32_t) GROUP_SEPERATOR);

	bufferIn(pRingbuffer, (uint32_t) NAME_ROTORPOSITION);
	bufferIn(pRingbuffer, (uint32_t) RECORD_SEPERATOR);
	bufferIn(pRingbuffer, (uint32_t) TYPE_INT);
	bufferIn(pRingbuffer, (uint32_t) RECORD_SEPERATOR);
	addUnsignedToRingbuffer(rotorpos);

	bufferIn(pRingbuffer, (uint32_t) GROUP_SEPERATOR);

	bufferIn(pRingbuffer, (uint32_t) NAME_ROTORPOSITION_SETPOINT);
	bufferIn(pRingbuffer, (uint32_t) RECORD_SEPERATOR);
	bufferIn(pRingbuffer, (uint32_t) TYPE_INT);
	bufferIn(pRingbuffer, (uint32_t) RECORD_SEPERATOR);
	addUnsignedToRingbuffer(rotorpos_setpoint);

	bufferIn(pRingbuffer, (uint32_t) GROUP_SEPERATOR);

	bufferIn(pRingbuffer, (uint32_t) NAME_ROTORPOSITION_CONTROL_OUTPUT);
	bufferIn(pRingbuffer, (uint32_t) RECORD_SEPERATOR);
	bufferIn(pRingbuffer, (uint32_t) TYPE_INT);
	bufferIn(pRingbuffer, (uint32_t) RECORD_SEPERATOR);
	addSignedToRingbuffer(controller_out);

	bufferIn(pRingbuffer, (uint32_t) GROUP_SEPERATOR);

	bufferIn(pRingbuffer, (uint32_t) ETX);
#endif
}

void log_controllerParameterTuple_mr(uint32_t nrIgnoredCalls, uint32_t t60Deg, uint32_t rotorpos, uint32_t rotorpos_setpoint, int32_t controller_out){
#ifdef LOG_CONTROLLER_PARAMETER_TUPLE
	static uint32_t cnt;
	cnt++;
	if(cnt >= nrIgnoredCalls){
		log_controllerParameterTuple(t60Deg, rotorpos, rotorpos_setpoint, controller_out);
		cnt = 0;
	}
#endif
}

void log_zeroCrossings(uint32_t buffer[]){
	bufferIn(pRingbuffer, (uint32_t) STX);
	addUnsignedToRingbuffer(getTimestamp());
	bufferIn(pRingbuffer, (uint32_t) GROUP_SEPERATOR);
	bufferIn(pRingbuffer, (uint32_t) ZERO_CROSSINGS);
	bufferIn(pRingbuffer, (uint32_t) RECORD_SEPERATOR);
	bufferIn(pRingbuffer, (uint32_t) TYPE_INT);
	bufferIn(pRingbuffer, (uint32_t) RECORD_SEPERATOR);

	uint32_t cnt = 0;
	while(1){
		if(buffer[cnt] != 0){
			addUnsignedToRingbuffer(buffer[cnt]);
			bufferIn(pRingbuffer, (uint32_t) RECORD_SEPERATOR);
			buffer[cnt] = 0;
			cnt++;
		}else{
			break;
		}
	}

	bufferIn(pRingbuffer, (uint32_t) GROUP_SEPERATOR);
	bufferIn(pRingbuffer, (uint32_t) ETX);
}

void log_writeBuffered() {
	int32_t data;
	int32_t *pData = &data;

	while (1) {
		if (bufferOut(pRingbuffer, pData) == BUFFER_EMPTY) {
			return;
		}
		transmitCharOverUART((char) *pData);
	}
}

// functions to add elements to buffer
void addUnsignedToRingbuffer(uint32_t number) {
	uint8_t tempBuffer[10]; // 2^32 = 10 digits
	uint32_t cnt = 0;

	uint32_t divider = 10;
	while (1) {
		switch (number % divider) {
		case 0:
			tempBuffer[cnt] = '0';
			break;
		case 1:
			tempBuffer[cnt] = '1';
			break;
		case 2:
			tempBuffer[cnt] = '2';
			break;
		case 3:
			tempBuffer[cnt] = '3';
			break;
		case 4:
			tempBuffer[cnt] = '4';
			break;
		case 5:
			tempBuffer[cnt] = '5';
			break;
		case 6:
			tempBuffer[cnt] = '6';
			break;
		case 7:
			tempBuffer[cnt] = '7';
			break;
		case 8:
			tempBuffer[cnt] = '8';
			break;
		case 9:
			tempBuffer[cnt] = '9';
			break;
		}

		number = number / divider;
		cnt++;

		if (number == 0) {
			break;
		}
	}

	// fill ringbuffer
	while(1){
		bufferIn(pRingbuffer, (uint32_t) tempBuffer[cnt-1]);
		cnt--;
		if(cnt == 0){
			return;
		}
	}
}

void addSignedToRingbuffer(int32_t number) {
	if (number < 0) {
		number = number * (-1);

		bufferIn(pRingbuffer, (uint32_t) '-');
	}

	addUnsignedToRingbuffer(number);
}

void addMsgToRingbuffer(char *pMsg) {
	while (1) {
		if (*pMsg == 0) {
			return;
		}

		bufferIn(pRingbuffer, (uint32_t) *pMsg);

		pMsg++;
	}
}
