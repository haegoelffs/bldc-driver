/*
 * bldc_driver_HAL.h
 *
 *  Created on: Nov 29, 2017
 *      Author: simon
 */

#ifndef INC_BLDC_DRIVER_HAL_H_
#define INC_BLDC_DRIVER_HAL_H_

#include <stdint.h>

//========================= PWM ===================================
void initPWM();

// timers
void initTimers();

/** Calls the handed function after the handed time
Parameter:
time_ us    = time in us. max value: 2¹⁶ * 4 = 262'144us.
fn          = callback. Called after the handed time.
*/
void startAfterUs(uint32_t time_us, void (*fn)(void));

/** Starts a new time measurement.
resolution = 1/(16e6/64) = 4us
max. time = 4us * 2¹⁶ = 262.2ms

Input:
timerOverflowCallback: called after the max. time
*/
void startTimeMeasurement(void (*timerOverflowCallback)(void));

/** Returns a 1, if there is a running time measurement
*/
uint8_t isTimeMeasurementRunning();

/** Stops the time measurement and returns the measured time.
 return: measured time in us.
*/
uint32_t stopTimeMeasurement();

uint32_t getTime();

//========================= ADC ==============================
void initAnalog();
int8_t startMeasureProcedure(char newPhaseToMeasure);
void registerMeasurementDataAvailableListener(void (*listener)(char phaseLastCurrentMeassure));

char readPhaseCurrnet(char phase); //value between 0 and 42, where 42 stands for 42 Ampére

int8_t getLastPhaseACurrent();
int8_t getLastPhaseBCurrent();

//========================= COMPERATORS ==============================
void initComp();

/** Register the handed function as listener which is called when the voltage of the phase A crosses the zero
    Input:
    listener = function with parameter edge.
        --> edge = 0: falling edge
        --> edge = 1: rising edge
**/
void registerVoltageZeroCrossingListenerPhaseA(void (*listener)(char));
/** Register the handed function as listener which is called when the voltage of the phase B crosses the zero
    Input:
    listener = function with parameter edge.
        --> edge = 0: falling edge
        --> edge = 1: rising edge
**/
void registerVoltageZeroCrossingListenerPhaseB(void (*listener)(char));
/** Register the handed function as listener which is called when the voltage of the phase C crosses the zero
    Input:
    listener = function with parameter edge.
        --> edge = 0: falling edge
        --> edge = 1: rising edge
**/
void registerVoltageZeroCrossingListenerPhaseC(void (*listener)(char));

/** Enables the comperator interrupt for the phase A.
    Input:
    enable = 1: enable comperator interrupt
    enable = 0: disable comperator interrupt
**/
void setEnableCompA(uint8_t enable);
/** Enables the comperator interrupt for the phase B.
    Input:
    enable = 1: enable comperator interrupt
    enable = 0: disable comperator interrupt
**/
void setEnableCompB(uint8_t enable);
/** Enables the comperator interrupt for the phase C.
    Input:
    enable = 1: enable comperator interrupt
    enable = 0: disable comperator interrupt
**/
void setEnableCompC(uint8_t enable);

//========================= GPIO'S ===================================
void initGPIOs();

// main switch
void switch_MainSwitch(uint8_t state);
uint8_t read_MainButton();

// bridge driver
void switch_Enable_BridgeDriver(uint8_t state);
void switch_DCCal_BridgeDriver(uint8_t state);
uint8_t read_NFault_BridgeDriver();
uint8_t read_NOCTW_BridgeDriver();
uint8_t read_PWRGD_BridgeDriver();

// leds
void switch_PowerLED(uint8_t state);
void switch_StatusLED1(uint8_t state);
void switch_StatusLED2(uint8_t state);
void switch_StatusLED3(uint8_t state);
void switch_StatusLED4(uint8_t state);

//========================= SYSTIME ===================================
void initSystime();
uint16_t getSystimeMs();

//========================= UART ===================================
void initUART();

void transmitStringOverUART(uint8_t *msg);

//========================= SPI ===================================
void initSPI();
void spi_readStatusRegisters_BLOCKING();
uint16_t getLastStatusRegister1Value();
uint16_t getLastStatusRegister2Value();

#endif /* INC_BLDC_DRIVER_HAL_H_ */
