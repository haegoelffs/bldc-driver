/*
 * bufferedLogger.h
 *
 *  Created on: Mar 23, 2018
 *      Author: simon
 */

#ifndef INC_BUFFEREDLOGGER_H_
#define INC_BUFFEREDLOGGER_H_

#include <stdint.h>

#define LOG_MSG
//#define LOG_NR_IMPULSES_ENCODER
#define LOG_CONTROLLER_PARAMETER_TUPLE
//#define LOG_TIME60DEG
//#define LOG_CYCLETIME
//#define LOG_MEAS_ZEROCROSSING

//#define LOG_SECTION_ACTIVE
//#define LOG_SECTION_0_ACTIVE
//#define LOG_SECTION_1_ACTIVE
//#define LOG_SECTION_2_ACTIVE
//#define LOG_SECTION_3_ACTIVE
//#define LOG_SECTION_4_ACTIVE
//#define LOG_SECTION_5_ACTIVE

void initBufferedLogger();

void log_msg(char msg[]);
void log_namedUint(char *pName, uint32_t value, uint32_t size);

void log_time60Deg(uint32_t t60Deg);
void log_time60Deg_mr(uint32_t max_resolution_us, uint32_t t60Deg);

void log_cycleTime(uint32_t cycleTime);
void log_maxCycleTimeStatistics(uint32_t max_resolution_us, uint32_t cycleTime);

void log_controllerParameterTuple(uint32_t t60Deg, uint32_t rotorpos, uint32_t rotorpos_setpoint);
void log_controllerParameterTuple_mr(uint32_t max_resolution_us, uint32_t t60Deg, uint32_t rotorpos, uint32_t rotorpos_setpoint);

void log_nrImpulsesEncoder(uint32_t nrImpulses);

void log_unsignedInt(uint32_t data);

void log_unnamedUint(uint32_t data);

// events
void log_zeroCrossingPhaseA(uint32_t timestamp);
void log_zeroCrossingPhaseB(uint32_t timestamp);
void log_zeroCrossingPhaseC(uint32_t timestamp);

void log_sectionActive(uint32_t timestamp, uint8_t section);

void log_writeBuffered();

#endif /* INC_BUFFEREDLOGGER_H_ */
