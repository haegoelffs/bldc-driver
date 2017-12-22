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

void set_PWM_DutyCycle(uint16_t dutyCycle);

/** Enables the pwm output for the phase A
    Input:
    enable = 1: enable the pwm output
    enable = 0: disable the pwm output
**/
void enable_PWM_phaseA_HS(uint8_t enable);
void enable_PWM_phaseB_HS(uint8_t enable);
void enable_PWM_phaseC_HS(uint8_t enable);
void enable_PWM_phaseA_LS(uint8_t enable);
void enable_PWM_phaseB_LS(uint8_t enable);
void enable_PWM_phaseC_LS(uint8_t enable);


//========================= ADC ==============================
void initAnalog();

// listener
void registerListener_newMeasData_hallA_shuntB(void (*listener)(void));
void registerListener_newMeasData_hallB_shuntA(void (*listener)(void));

// shunt
int8_t start_phaseACurrentMeas_shunt();
int8_t start_phaseBCurrentMeas_shunt();

uint32_t getLastMeas_phaseACurrentMeas_shunt();
uint32_t getLastMeas_phaseBCurrentMeas_shunt();

// hall
int8_t start_phaseACurrentMeas_hall();
int8_t start_phaseBCurrentMeas_hall();

uint32_t getLastMeas_phaseACurrentMeas_hall();
uint32_t getLastMeas_phaseBCurrentMeas_hall();

// user voltage in
int8_t start_userVolatgeMeas();
uint32_t getLastMeas_userVolatgeMeas();
uint8_t isMeasReady_userVolatgeMeas();

//========================= COMPERATORS ==============================
void initComp();

/** Register the handed function as listener which is called when the voltage of the phase A crosses the zero
    Input:
    listener = function with parameter edge.
        --> edge = 0: falling edge
        --> edge = 1: rising edge
**/
void register_comperatorListener_phaseA(void (*listener)(uint8_t));
/** Register the handed function as listener which is called when the voltage of the phase B crosses the zero
    Input:
    listener = function with parameter edge.
        --> edge = 0: falling edge
        --> edge = 1: rising edge
**/
void register_comperatorListener_phaseB(void (*listener)(uint8_t));
/** Register the handed function as listener which is called when the voltage of the phase C crosses the zero
    Input:
    listener = function with parameter edge.
        --> edge = 0: falling edge
        --> edge = 1: rising edge
**/
void register_comperatorListener_phaseC(void (*listener)(uint8_t));

/** Enables the comperator interrupt for the phase A.
    Input:
    enable = 1: enable comperator interrupt
    enable = 0: disable comperator interrupt
**/
void enableCompA(uint8_t enable);
/** Enables the comperator interrupt for the phase B.
    Input:
    enable = 1: enable comperator interrupt
    enable = 0: disable comperator interrupt
**/
void enableCompB(uint8_t enable);
/** Enables the comperator interrupt for the phase C.
    Input:
    enable = 1: enable comperator interrupt
    enable = 0: disable comperator interrupt
**/
void enableCompC(uint8_t enable);

//========================= GPIO'S ===================================
void initGPIOs();

// main interface
void switch_MainSwitch(uint8_t state);
uint8_t read_MainButton();
uint8_t read_StateButton();

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
uint32_t getSystimeUs();

/** Calls the handed function after the handed time
Parameter:
time_ us    = time in us. max value: 2¹⁶ * 4 = 262'144us.
fn          = callback. Called after the handed time.
*/
void startAfterUs(uint32_t time_us, void (*fn)(void));
uint32_t getElapsedTimeInUs();


//========================= UART ===================================
void initUART();
void transmitStringOverUART(uint8_t *msg);

//========================= SPI ===================================
void initSPI();
void spi_readStatusRegisters_BLOCKING();
uint16_t getLastStatusRegister1Value();
uint16_t getLastStatusRegister2Value();

//====================== FLOW CONTROLL ============================
void startup();
void proceed();
void shutdown();

#endif /* INC_BLDC_DRIVER_HAL_H_ */
