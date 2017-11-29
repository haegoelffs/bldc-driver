/*
 * bldc_driver_HAL.c
 *
 *  Created on: Nov 29, 2017
 *      Author: simon
 */

#include "main.h"

#include "bldc_driver_HAL.h"
#include "bldc_driver_adapter.h"

// hal handles
static ADC_HandleTypeDef shuntA_hallB_ADC_handle;
static ADC_HandleTypeDef shuntB_ADC_handle;
static ADC_HandleTypeDef hallA_ADC_handle;
static ADC_HandleTypeDef user_ADC_handle;
static DAC_HandleTypeDef virtZero_DAC_handler;
static TIM_HandleTypeDef a_LS_PWM_handle;
static TIM_HandleTypeDef b_LS_PWM_handle;
static TIM_HandleTypeDef c_LS_PWM_handle;
static TIM_HandleTypeDef a_HS_PWM_handle;
static TIM_HandleTypeDef b_HS_PWM_handle;
static TIM_HandleTypeDef c_HS_PWM_handle;
static SPI_HandleTypeDef spi_handle;
static UART_HandleTypeDef uart_handle;

// comperator interrupt listeners
static void (*listenerPhaseA)(uint8_t edge);
static void (*listenerPhaseB)(uint8_t edge);
static void (*listenerPhaseC)(uint8_t edge);


void initBLDCDriver(
		ADC_HandleTypeDef *pShuntA_hallB_ADC_handle,
		ADC_HandleTypeDef *pShuntB_ADC_handle,
		ADC_HandleTypeDef *pHallA_ADC_handle,
		ADC_HandleTypeDef *pUser_ADC_handle,
		DAC_HandleTypeDef *pVirtZero_DAC_handler,
		TIM_HandleTypeDef *pA_LS_PWM_handle,
		TIM_HandleTypeDef *pB_LS_PWM_handle,
		TIM_HandleTypeDef *pC_LS_PWM_handle,
		TIM_HandleTypeDef *pA_HS_PWM_handle,
		TIM_HandleTypeDef *pB_HS_PWM_handle,
		TIM_HandleTypeDef *pC_HS_PWM_handle,
		SPI_HandleTypeDef *pSPI_handle,
		UART_HandleTypeDef *pUART_handle) {
	shuntA_hallB_ADC_handle = *pShuntA_hallB_ADC_handle;
	shuntB_ADC_handle = *pShuntB_ADC_handle;
	hallA_ADC_handle = *pHallA_ADC_handle;
	user_ADC_handle = *pUser_ADC_handle;
	virtZero_DAC_handler = *pVirtZero_DAC_handler;
	a_LS_PWM_handle = *pA_LS_PWM_handle;
	b_LS_PWM_handle = *pB_LS_PWM_handle;
	c_LS_PWM_handle = *pC_LS_PWM_handle;
	a_HS_PWM_handle = *pA_HS_PWM_handle;
	b_HS_PWM_handle = *pB_HS_PWM_handle;
	c_HS_PWM_handle = *pC_HS_PWM_handle;
	spi_handle = *pSPI_handle;
	uart_handle = *pUART_handle;
}
void startupBLDCDriver() {
	switch_PowerLED(1);
	switch_Enable_BridgeDriver(1);
}
void proceedBLDCDriver() {
}
void shutdownBLDCDriver() {
}

//========================= GPIO'S ===================================
void initGPIOs(){
}
// led's
void switch_PowerLED(uint8_t state) {
	HAL_GPIO_WritePin(DO_LED_1_GPIO_Port, DO_LED_1_Pin, state);
}
void switch_StatusLED1(uint8_t state) {
	HAL_GPIO_WritePin(DO_LED_2_GPIO_Port, DO_LED_2_Pin, state);
}
void switch_StatusLED2(uint8_t state) {
	HAL_GPIO_WritePin(DO_LED_3_GPIO_Port, DO_LED_3_Pin, state);
}
void switch_StatusLED3(uint8_t state) {
	HAL_GPIO_WritePin(DO_LED_4_GPIO_Port, DO_LED_4_Pin, state);
}
void switch_StatusLED4(uint8_t state) {
	HAL_GPIO_WritePin(DO_LED_5_GPIO_Port, DO_LED_5_Pin, state);
}
// bridge driver
void switch_Enable_BridgeDriver(uint8_t state) {
	HAL_GPIO_WritePin(DO_DRIVER_EN_GPIO_Port, DO_DRIVER_EN_Pin, state);
}
void switch_DCCal_BridgeDriver(uint8_t state) {
	HAL_GPIO_WritePin(DO_DRIVER_DC_CAL_GPIO_Port, DO_DRIVER_DC_CAL_Pin, state);
}
uint8_t read_NFault_BridgeDriver(){
	return HAL_GPIO_ReadPin(DI_DRIVER_NFAULT_GPIO_Port, DI_DRIVER_NFAULT_Pin);
}
uint8_t read_NOCTW_BridgeDriver(){
	return HAL_GPIO_ReadPin(DI_DRIVER_NOCTW_GPIO_Port, DI_DRIVER_NOCTW_Pin);
}
uint8_t read_PWRGD_BridgeDriver(){
	return HAL_GPIO_ReadPin(DI_DRIVER_PWRGD_GPIO_Port, DI_DRIVER_PWRGD_Pin);
}
// main switch
void switch_MainSwitch(uint8_t state){
	HAL_GPIO_WritePin(DO_MAIN_SWITCH_GPIO_Port, DO_MAIN_SWITCH_Pin, state);
}
uint8_t read_MainButton(){
	return HAL_GPIO_ReadPin(DI_MAIN_BUTTON_GPIO_Port, DI_MAIN_BUTTON_Pin);
}

//========================= UART ===================================
void initUART(){
}

void transmitStringOverUART(uint8_t *pMsg){
	// find lenght of string (zero terminated)
	uint8_t *pTemp = pMsg;
	uint8_t cnt = 0;
	while(1){
		if(*pTemp == 0 || cnt >= 255)
		{
			// end of string
			break;
		}
		else
		{
			// count up pointer to next string element
			pTemp++;
			cnt++;
		}
	}

	HAL_UART_Transmit(uart_handle, pMsg, cnt, 100);
}
void transmitCharOverUART(char data){
	HAL_UART_Transmit(uart_handle, &data, 1, 100);
}

//========================= COMPERATORS ==============================
void initComp(){

}

void registerVoltageZeroCrossingListenerPhaseA(void (*listener)(uint8_t)){
	listenerPhaseA = listener;
}
void registerVoltageZeroCrossingListenerPhaseB(void (*listener)(uint8_t)){
	listenerPhaseB = listener;
}
void registerVoltageZeroCrossingListenerPhaseC(void (*listener)(uint8_t)){
	listenerPhaseC = listener;
}

void setEnableCompA(uint8_t enable){

}
void setEnableCompB(uint8_t enable){

}
void setEnableCompC(uint8_t enable){

}
