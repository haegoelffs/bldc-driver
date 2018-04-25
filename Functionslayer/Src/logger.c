/*
 * logger.c
 *
 *  Created on: Dec 16, 2017
 *      Author: simon
 */

#include "logger.h"
#include <stdio.h>
#include <string.h>
#include "bldc_driver_HAL.h"

#define MAX_SIZE_OF_STRING 255

#define DEBUG 3
#define INFO 2
#define ERROR 1
#define OFF 0

// ============================== Logger settings =========================================
#define LOG_LEVEL DEBUG
#define LOG_TIMESTAMP
// ========================================================================================

// utils
uint32_t getSizeOfString(char *pString) {
	// find lenght of string (zero terminated)
	uint32_t *pTemp = pString;
	uint32_t cnt = 0;
	while (1) {
		if (*pTemp == 0 || cnt >= MAX_SIZE_OF_STRING) {
			// end of string
			break;
		} else {
			// count up pointer to next string element
			pTemp++;
			cnt++;
		}
	}
	return cnt;
}
void logTimestamp() {
#ifdef LOG_TIMESTAMP
	uint32_t sysTimeUs = getSystimeUs();

	char str[10]; // 2^32 ~= 4.295e9 -> 10 digits
	sprintf(str, "%10u", sysTimeUs);

	char usStr[4];
	memcpy(usStr, str + 7, 3 * sizeof(char)); // Start: 0123'456>[789]
	usStr[3] = 0;
	// replace leading zeros with space
	if (usStr[0] == '0') {
		usStr[0] = ' ';
		if (usStr[1] == '0') {
			usStr[1] = ' ';
		}
	}

	char msStr[4];
	memcpy(msStr, str + 4, 3 * sizeof(char)); // Start: 0123>[456]'789
	msStr[3] = 0;
	// replace leading zeros with space
	if (msStr[0] == '0') {
		msStr[0] = ' ';
		if (msStr[1] == '0') {
			msStr[1] = ' ';
		}
	}

	char sStr[5];
	memcpy(sStr, str + 0, 4 * sizeof(char)); // Start: >[0123]'456'789
	sStr[4] = 0;

	transmitCharOverUART('[');
	transmitStringOverUART(sStr);
	transmitStringOverUART("s ");
	transmitStringOverUART(msStr);
	transmitStringOverUART("ms ");
	transmitStringOverUART(usStr);
	transmitStringOverUART("us");
	transmitCharOverUART(']');
#endif
}
void writeNewLine() {
	transmitCharOverUART('\r'); // return
	transmitCharOverUART('\n'); // new line
}

// logging functions
void logMsg(char* msg) {
	transmitStringOverUART(msg);
}

void logMsgLn(char *msg) {
	transmitStringOverUART(msg);
	writeNewLine();
}

void logUnsigned(char *name, uint32_t var) {
	transmitStringOverUART(name);
	transmitCharOverUART(':');

	char str[10]; // 2^32 ~= 4.295e9 -> 10 digits
	sprintf(str, "%u", var);
	transmitStringOverUART(str);
	writeNewLine();
}

void logSigned(char *name, uint32_t var) {
	transmitStringOverUART(name);
	transmitCharOverUART(':');

	char str[10]; // 2^32 ~= 4.295e9 -> 10 digits
	sprintf(str, "%i", var);
	transmitStringOverUART(str);
	writeNewLine();
}

void logERROR(char *msg){
#if LOG_LEVEL >= ERROR
	logTimestamp();
	transmitStringOverUART(" ERROR ");
	transmitStringOverUART(msg);
	writeNewLine();
#endif
}
void logINFO(char *msg){
#if LOG_LEVEL >= INFO
	logTimestamp();
	transmitStringOverUART(" INFO ");
	transmitStringOverUART(msg);
	writeNewLine();
#endif
}
void logDEBUG(char *msg){
#if LOG_LEVEL >= DEBUG
	logTimestamp();
	transmitStringOverUART(" DEBUG ");
	transmitStringOverUART(msg);
	writeNewLine();
#endif
}

void logUnsignedERROR(char *name, uint32_t var){
#if LOG_LEVEL >= ERROR
	logTimestamp();
	transmitStringOverUART(" ERROR ");
	logUnsigned(name, var);
#endif
}
void logSignedERROR(char *name, uint32_t var){
#if LOG_LEVEL >= ERROR
	logTimestamp();
	transmitStringOverUART(" ERROR ");
	logSigned(name, var);
#endif
}
void logUnsignedINFO(char *name, uint32_t var){
#if LOG_LEVEL >= INFO
	logTimestamp();
	transmitStringOverUART(" INFO ");
	logUnsigned(name, var);
#endif
}
void logSignedINFO(char *name, uint32_t var){
#if LOG_LEVEL >= INFO
	logTimestamp();
	transmitStringOverUART(" INFO ");
	logSigned(name, var);
#endif
}

void logUnsignedDEBUG(char *name, uint32_t var){
#if LOG_LEVEL >= DEBUG
	logTimestamp();
	transmitStringOverUART(" DEBUG ");
	logUnsigned(name, var);
#endif
}
void logSignedDEBUG(char *name, uint32_t var){
#if LOG_LEVEL >= DEBUG
	logTimestamp();
	transmitStringOverUART(" DEBUG ");
	logSigned(name, var);
#endif
}

