// =============== Includes ==============================================
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

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
#define TYPE_EVENT '3'

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

#define ZERO_CROSSING_PHASE_A_RISING 'H'
#define ZERO_CROSSING_PHASE_B_RISING 'I'
#define ZERO_CROSSING_PHASE_C_RISING 'J'
#define ZERO_CROSSING_PHASE_A_FALLING 'K'
#define ZERO_CROSSING_PHASE_B_FALLING 'L'
#define ZERO_CROSSING_PHASE_C_FALLING 'M'

#define NAME_SECTION_0_ACTIVE 'N'
#define NAME_SECTION_1_ACTIVE 'O'
#define NAME_SECTION_2_ACTIVE 'P'
#define NAME_SECTION_3_ACTIVE 'Q'
#define NAME_SECTION_4_ACTIVE 'R'
#define NAME_SECTION_5_ACTIVE 'S'

#define NAME_NR_IMPULSES_ENCODER 'T'
#define NAME_UNNAMED 'U'

// =============== Variables =============================================
Ringbuffer *pRingbuffer;

// =============== Function pointers =====================================

// =============== Function declarations =================================
void addUnsignedToRingbuffer(uint32_t number);
void addSignedToRingbuffer(int32_t number);
void addMsgToRingbuffer(char *pMsg);
void addCharToRingbuffer(char data);

// =============== Functions =============================================
void initBufferedLogger() {
	pRingbuffer = allocRingbuffer(2000);
}

//<STX>timestamp<GS>data_name<RS>data_type<RS>data<GS><ETX>
void log_msg(char *pMsg) {
#ifdef LOG_MSG
	addCharToRingbuffer(STX);
	addUnsignedToRingbuffer(getTimestamp());

	addCharToRingbuffer(GROUP_SEPERATOR);
	addCharToRingbuffer(NAME_GENERIC_MESSAGE);
	addCharToRingbuffer(RECORD_SEPERATOR);
	addCharToRingbuffer(TYPE_STRING);
	addCharToRingbuffer(RECORD_SEPERATOR);
	addMsgToRingbuffer(pMsg);
	addCharToRingbuffer(GROUP_SEPERATOR);
	addCharToRingbuffer(ETX);
#endif
}

void log_namedUint(char *pName, uint32_t value, uint32_t size) {
	char pString[size];
	sprintf(pString, "%s:%lu", pName, value);

	log_msg(pString);
}

void log_time60Deg(uint32_t t60Deg) {
#ifdef LOG_TIME60DEG
	addCharToRingbuffer(STX);
	addUnsignedToRingbuffer(getTimestamp());
	addCharToRingbuffer(GROUP_SEPERATOR);
	addCharToRingbuffer(NAME_ROTATION_FREQUENZY);
	addCharToRingbuffer(RECORD_SEPERATOR);
	addCharToRingbuffer(TYPE_INT);
	addCharToRingbuffer(RECORD_SEPERATOR);
	addUnsignedToRingbuffer(t60Deg);
	addCharToRingbuffer(GROUP_SEPERATOR);
	addCharToRingbuffer(ETX);
#endif
}

void log_time60Deg_mr(uint32_t max_resolution_us, uint32_t t60Deg) {
#ifdef LOG_TIME60DEG
	static uint32_t last_timestamp_us;
	uint32_t timestamp_us = getElapsedTimeInUs();
	if((timestamp_us - last_timestamp_us) >= max_resolution_us) {
		last_timestamp_us = timestamp_us;
		log_time60Deg(t60Deg);
	}
#endif
}

void log_cycleTime(uint32_t cycleTime) {
#ifdef LOG_TIME60DEG
	addCharToRingbuffer(STX);
	addUnsignedToRingbuffer(getTimestamp());
	addCharToRingbuffer(GROUP_SEPERATOR);
	addCharToRingbuffer(NAME_CYCLE_TIME);
	addCharToRingbuffer(RECORD_SEPERATOR);
	addCharToRingbuffer(TYPE_INT);
	addCharToRingbuffer(RECORD_SEPERATOR);
	addUnsignedToRingbuffer(cycleTime);
	addCharToRingbuffer(GROUP_SEPERATOR);
	addCharToRingbuffer(ETX);
#endif
}

void log_maxCycleTimeStatistics(uint32_t max_resolution_us, uint32_t cycleTime) {
#ifdef LOG_CYCLETIME
	static uint32_t max_cycleTime;
	static uint32_t last_timestamp_us;

	if(cycleTime > max_cycleTime) {
		max_cycleTime = cycleTime;
	}

	uint32_t timestamp_us = getElapsedTimeInUs();
	if((timestamp_us - last_timestamp_us) >= max_resolution_us) {
		last_timestamp_us = timestamp_us;
		log_cycleTime(max_cycleTime);
		max_cycleTime = 0;
	}
#endif
}

void log_nrImpulsesEncoder(uint32_t nrImpulses) {
#ifdef LOG_NR_IMPULSES_ENCODER
	addCharToRingbuffer(STX);
	addUnsignedToRingbuffer(getTimestamp());
	addCharToRingbuffer(GROUP_SEPERATOR);
	addCharToRingbuffer(NAME_NR_IMPULSES_ENCODER);
	addCharToRingbuffer(RECORD_SEPERATOR);
	addCharToRingbuffer(TYPE_INT);
	addCharToRingbuffer(RECORD_SEPERATOR);
	addUnsignedToRingbuffer(nrImpulses);
	addCharToRingbuffer(GROUP_SEPERATOR);
	addCharToRingbuffer(ETX);
#endif
}

// <STX>timestamp<GS>data_name<RS>data_type<RS>data<GS><ETX>
// <STX>6603375<GS>
//		C<RS>1<RS>5014<GS>
//		D<RS>1<RS>62<GS>
//		E<RS>1<RS>2495<GS>
//		F<RS>1<RS>4294967272<GS>
// <ETX>
void log_controllerParameterTuple(uint32_t t60Deg, uint32_t rotorpos,
		uint32_t rotorpos_setpoint) {
#ifdef LOG_CONTROLLER_PARAMETER_TUPLE
	addCharToRingbuffer(STX);
	addUnsignedToRingbuffer(getTimestamp());

	/*addCharToRingbuffer(GROUP_SEPERATOR);
	 addCharToRingbuffer( NAME_ROTATION_FREQUENZY);
	 addCharToRingbuffer(RECORD_SEPERATOR);
	 addCharToRingbuffer(TYPE_INT);
	 addCharToRingbuffer(RECORD_SEPERATOR);
	 addUnsignedToRingbuffer(t60Deg);*/

	addCharToRingbuffer(GROUP_SEPERATOR);

	addCharToRingbuffer(NAME_ROTORPOSITION);
	addCharToRingbuffer(RECORD_SEPERATOR);
	addCharToRingbuffer(TYPE_INT);
	addCharToRingbuffer(RECORD_SEPERATOR);
	addUnsignedToRingbuffer(rotorpos);

	addCharToRingbuffer(GROUP_SEPERATOR);

	addCharToRingbuffer(NAME_ROTORPOSITION_SETPOINT);
	addCharToRingbuffer(RECORD_SEPERATOR);
	addCharToRingbuffer(TYPE_INT);
	addCharToRingbuffer(RECORD_SEPERATOR);
	addUnsignedToRingbuffer(rotorpos_setpoint);

	addCharToRingbuffer(GROUP_SEPERATOR);

	/*addCharToRingbuffer(NAME_ROTORPOSITION_CONTROL_OUTPUT);
	 addCharToRingbuffer(RECORD_SEPERATOR);
	 addCharToRingbuffer(TYPE_INT);
	 addCharToRingbuffer(RECORD_SEPERATOR);
	 addSignedToRingbuffer(controller_out);

	 addCharToRingbuffer(GROUP_SEPERATOR);*/

	addCharToRingbuffer(ETX);
#endif
}

void log_controllerParameterTuple_mr(uint32_t nrIgnoredCalls, uint32_t t60Deg,
		uint32_t rotorpos, uint32_t rotorpos_setpoint) {
#ifdef LOG_CONTROLLER_PARAMETER_TUPLE
	static uint32_t cnt;
	cnt++;
	if (cnt >= nrIgnoredCalls) {
		log_controllerParameterTuple(t60Deg, rotorpos, rotorpos_setpoint);
		cnt = 0;
	}
#endif
}

void log_unnamedUint(uint32_t data) {
#ifdef LOG_NR_IMPULSES_ENCODER
	addCharToRingbuffer(STX);
	addUnsignedToRingbuffer(getTimestamp());
	addCharToRingbuffer(GROUP_SEPERATOR);
	addCharToRingbuffer(NAME_UNNAMED);
	addCharToRingbuffer(RECORD_SEPERATOR);
	addCharToRingbuffer(TYPE_INT);
	addCharToRingbuffer(RECORD_SEPERATOR);
	addUnsignedToRingbuffer(data);
	addCharToRingbuffer(GROUP_SEPERATOR);
	addCharToRingbuffer(ETX);
#endif
}

// events
void log_zeroCrossing(uint8_t phase, uint32_t timestamp) {
#ifdef LOG_MEAS_ZEROCROSSING
	addCharToRingbuffer(STX);
	addUnsignedToRingbuffer(timestamp);

	addCharToRingbuffer(GROUP_SEPERATOR);
	addCharToRingbuffer(ZERO_CROSSING_PHASE_A_RISING);
	addCharToRingbuffer(RECORD_SEPERATOR);
	addCharToRingbuffer(TYPE_EVENT);

	addCharToRingbuffer(GROUP_SEPERATOR);
	addCharToRingbuffer(ETX);
#endif
}

void log_zeroCrossingPhaseA(uint32_t timestamp) {
#ifdef LOG_MEAS_ZEROCROSSING
	addCharToRingbuffer(STX);
	addUnsignedToRingbuffer(timestamp);

	addCharToRingbuffer(GROUP_SEPERATOR);
	addCharToRingbuffer(ZERO_CROSSING_PHASE_A_RISING);
	addCharToRingbuffer(RECORD_SEPERATOR);
	addCharToRingbuffer(TYPE_EVENT);

	addCharToRingbuffer(GROUP_SEPERATOR);
	addCharToRingbuffer(ETX);
#endif
}
void log_zeroCrossingPhaseB(uint32_t timestamp) {
#ifdef LOG_MEAS_ZEROCROSSING
	addCharToRingbuffer(STX);
	addUnsignedToRingbuffer(timestamp);

	addCharToRingbuffer(GROUP_SEPERATOR);
	addCharToRingbuffer(ZERO_CROSSING_PHASE_B_RISING);
	addCharToRingbuffer(RECORD_SEPERATOR);
	addCharToRingbuffer(TYPE_EVENT);

	addCharToRingbuffer(GROUP_SEPERATOR);
	addCharToRingbuffer(ETX);
#endif
}
void log_zeroCrossingPhaseC(uint32_t timestamp) {
#ifdef LOG_MEAS_ZEROCROSSING
	addCharToRingbuffer(STX);
	addUnsignedToRingbuffer(timestamp);

	addCharToRingbuffer(GROUP_SEPERATOR);
	addCharToRingbuffer(ZERO_CROSSING_PHASE_C_RISING);
	addCharToRingbuffer(RECORD_SEPERATOR);
	addCharToRingbuffer(TYPE_EVENT);

	addCharToRingbuffer(GROUP_SEPERATOR);
	addCharToRingbuffer(ETX);
#endif
}

void log_sectionActive(uint32_t timestamp, uint8_t section) {
#ifdef LOG_SECTION_ACTIVE
	char sectionName;

	switch (section) {
		case 0:
#ifdef LOG_SECTION_0_ACTIVE
		sectionName = NAME_SECTION_0_ACTIVE;
		break;
#else
		return;
#endif
		case 1:
#ifdef LOG_SECTION_1_ACTIVE
		sectionName = NAME_SECTION_1_ACTIVE;
		break;
#else
		return;
#endif
		case 2:
#ifdef LOG_SECTION_2_ACTIVE
		sectionName = NAME_SECTION_2_ACTIVE;
		break;
#else
		return;
#endif
		case 3:
#ifdef LOG_SECTION_3_ACTIVE
		sectionName = NAME_SECTION_3_ACTIVE;
		break;
#else
		return;
#endif
		case 4:
#ifdef LOG_SECTION_4_ACTIVE
		sectionName = NAME_SECTION_4_ACTIVE;
		break;
#else
		return;
#endif
		case 5:
#ifdef LOG_SECTION_5_ACTIVE
		sectionName = NAME_SECTION_5_ACTIVE;
		break;
#else
		return;
#endif
	}

	addCharToRingbuffer(STX);
	addUnsignedToRingbuffer(timestamp);

	addCharToRingbuffer(GROUP_SEPERATOR);
	addCharToRingbuffer(sectionName);
	addCharToRingbuffer(RECORD_SEPERATOR);
	addCharToRingbuffer(TYPE_EVENT);

	addCharToRingbuffer(GROUP_SEPERATOR);
	addCharToRingbuffer(ETX);
#endif
}

// handling of the logger
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
	while (1) {
		addCharToRingbuffer(tempBuffer[cnt - 1]);
		cnt--;
		if (cnt == 0) {
			return;
		}
	}
}

void addSignedToRingbuffer(int32_t number) {
	if (number < 0) {
		number = number * (-1);

		addCharToRingbuffer('-');
	}

	addUnsignedToRingbuffer(number);
}

void addMsgToRingbuffer(char *pMsg) {
	while (1) {
		if (*pMsg == 0) {
			return;
		}

		addCharToRingbuffer(*pMsg);

		pMsg++;
	}
}

void addCharToRingbuffer(char data) {
	if (bufferIn(pRingbuffer, (uint32_t) data) == BUFFER_OVERFLOW) {
		// reset and log
		bufferReset(pRingbuffer);
		log_msg("Logging buffer reseted because of a overflow");
	}
}
