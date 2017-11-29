/*
 * bldc_driver_adapter.h
 *
 *  Created on: Nov 29, 2017
 *      Author: simon
 */

#ifndef INC_BLDC_DRIVER_ADAPTER_H_
#define INC_BLDC_DRIVER_ADAPTER_H_

#include "stm32f3xx_hal.h"

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
		UART_HandleTypeDef *pUART_handle);
void startupBLDCDriver();
void proceedBLDCDriver();
void shutdownBLDCDriver();

#endif /* INC_BLDC_DRIVER_ADAPTER_H_ */
