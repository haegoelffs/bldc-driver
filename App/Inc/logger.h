/*
 * logger.h
 *
 *  Created on: Dec 16, 2017
 *      Author: simon
 */

#ifndef INC_LOGGER_H_
#define INC_LOGGER_H_

void logERROR(char *msg);
void logINFO(char *msg);
void logDEBUG(char *msg);

void logUnsignedERROR(char *name, uint32_t var);
void logSignedERROR(char *name, uint32_t var);

void logUnsignedINFO(char *name, uint32_t var);
void logSignedINFO(char *name, uint32_t var);

void logUnsignedDEBUG(char *name, uint32_t var);
void logSignedDEBUG(char *name, uint32_t var);

// utils
void writeNewLine();
void logTimestamp();

#endif /* INC_LOGGER_H_ */
