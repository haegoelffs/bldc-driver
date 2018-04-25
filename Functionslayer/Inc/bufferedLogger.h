/*
 * bufferedLogger.h
 *
 *  Created on: Mar 23, 2018
 *      Author: simon
 */

#ifndef INC_BUFFEREDLOGGER_H_
#define INC_BUFFEREDLOGGER_H_

#define LOG_MSG
#define LOG_CONTROLLER_PARAMETER_TUPLE
//#define LOG_TIME60DEG
//#define LOG_CYCLETIME

void initBufferedLogger();

void log_msg(char msg[]);

void log_time60Deg(uint32_t t60Deg);
void log_time60Deg_mr(uint32_t max_resolution_us, uint32_t t60Deg);

void log_cycleTime(uint32_t cycleTime);
void log_maxCycleTimeStatistics(uint32_t max_resolution_us, uint32_t cycleTime);

void log_controllerParameterTuple(uint32_t t60Deg, uint32_t rotorpos, uint32_t rotorpos_setpoint, int32_t controller_out);
void log_controllerParameterTuple_mr(uint32_t max_resolution_us, uint32_t t60Deg, uint32_t rotorpos, uint32_t rotorpos_setpoint, int32_t controller_out);


void log_writeBuffered();

#endif /* INC_BUFFEREDLOGGER_H_ */
