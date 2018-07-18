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

#define MAX_PWM_DUTYCYCLE 1800
#define MIN_PWM_DUTYCYCLE 0
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
void registerListener_newMeasData_hallA(void (*listener)(void));
void registerListener_newMeasData_hallB(void (*listener)(void));

// hall
#define ADC_MEASUREMENT_STARTED 0
#define ADC_ERROR -1
int8_t start_phaseACurrentMeas_hall(uint32_t nr_measurements, uint32_t *pBuffer);
int8_t start_phaseBCurrentMeas_hall(uint32_t nr_measurements, uint32_t *pBuffer);

// user voltage in
int8_t start_userVolatgeMeas();
uint8_t isMeasReady_userVolatgeMeas();
uint8_t newDataAvailable_userVolatgeMeas();
uint32_t getLastMeas_userVolatgeMeas();


// main voltage
/* REFERENZ
 * Spannungsteiler 1/9 --> Max. Voltage 26V / 9 = 2.8V
 * Korrektur 3.3V/2.8 = 1.1423
*/
#define CORRECTION_FACTOR_MAIN_VOLTAGE 1.1423

int8_t start_mainVoltageMeas();
uint8_t isMeasReady_mainVolatgeMeas();
uint8_t newDataAvailable_mainVolatgeMeas();
uint32_t getLastData_mainVoltageMeas();


//========================= DAC ======================================
void enable_virtualGND(uint8_t enable);
void set_virtualGNDValue(uint32_t value);

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

uint8_t read_signal_compA();
uint8_t read_signal_compB();
uint8_t read_signal_compC();

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

#define DELAYED_CALLBACK_REGISTERED 1
#define DELAYED_CALLBACK_ERROR 2
#define DELAYED_CALLBACK_IS_BUSY 2
#define DELAYED_CALLBACK_IS_READY 3
uint8_t delayedCallback_A(uint32_t time_us, void (*listener)(void));
uint8_t delayedCallback_B(uint32_t time_us, void (*listener)(void));
uint8_t delayedCallback_C(uint32_t time_us, void (*listener)(void));
uint8_t delayedCallback_D(uint32_t time_us, void (*listener)(void));

uint8_t isBusy_delayedCallback_A();
uint8_t isBusy_delayedCallback_B();
uint8_t isBusy_delayedCallback_C();
uint8_t isBusy_delayedCallback_D();

void abort_delayedCallback_A();
void abort_delayedCallback_B();
void abort_delayedCallback_C();
void abort_delayedCallback_D();

void waitBLOCKING(uint32_t ms);

//========================= UART ===================================
void initUART();
void transmitCharOverUART(char data);
void transmitStringOverUART(char *msg);

//========================= SPI ===================================
// Bridge Driver:
#define ADDRESS_STATUS_REGISTER_1 0b0000
#define ADDRESS_STATUS_REGISTER_2 0b0001
#define ADDRESS_CONTROL_REGISTER_1 0b0010
#define ADDRESS_CONTROL_REGISTER_2 0b0011

/** Bridge Input Data
|15     |14     |13     |12     |11     |10     |9      |8      |7      |6      |5      |4      |3      |2      |1      |0      |
|R/W    |ADR3   |ADR2   |ADR1   |ADR0   |DATA10 |DATA9  |DATA8  |DATA6  |DATA5  |DATA4  |DATA3  |DATA2  |DATA2  |DATA1  |DATA0  |
**/
#define WRITE_COMMAND 0
#define READ_COMMAND 1

#define READ_STATUS_REGISTER_1_COMMAND_high ((READ_COMMAND<<7) | (ADDRESS_STATUS_REGISTER_1<<3))
#define READ_STATUS_REGISTER_2_COMMAND_high ((READ_COMMAND<<7) | (ADDRESS_STATUS_REGISTER_2<<3))

// Status register 1 data p.22
#define FAULT 10 // frame error in previous cycle
#define GVDD_UV 9 // gate drive supply undervoltage
#define PVDD_UV 8 // undervoltage protection
#define OTSD 7 // overtemp shutdown
#define OTW 6 // overtemp warning
#define FETHA_OC 5
#define FETLA_OC 4
#define FETHB_OC 3
#define FETLB_OC 2
#define FETHC_OC 1
#define FETLC_OC 0

void initSPI();
void spi_readStatusRegisters_BLOCKING();
uint16_t getLastStatusRegister1Value();
uint16_t getLastStatusRegister2Value();

//====================== FLOW CONTROLL ============================
void startup();
void proceed();
void shutdown();

//========================= ENCODER ==============================
void initEncoder();

void enableSingleIRQ_encoderSignalA(void (*listener)(void));

void enableIRQ_encoderSignalReferencePos(void (*listener)(void));
void disableIRQ_encoderSignalReferencePos();

void registerListener_rotated180Deg(void (*listener)(void));

uint8_t read_encoderSignalA();
uint8_t read_encoderSignalB();
uint8_t read_encoderEnable();
uint8_t read_encoderCalibrate();

uint32_t getNrImpulses_encoderSignalA();
void setNrImpulses_encoderSignalA(uint16_t nrImpulses);
void resetNrImpulses_encoderSignalA();

// calibration
#define MAX_CALIBRATION_POTI_VALUE 0b111111111111 // 12bit ADC
uint32_t measAnalog_encoderCalibrationPoti_BLOCKING();
void switch_encoderPositionPin(uint8_t state);

#endif /* INC_BLDC_DRIVER_HAL_H_ */
