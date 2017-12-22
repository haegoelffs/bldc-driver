/*
 * bldc_driver_adapter.h
 *
 *  Created on: Nov 29, 2017
 *      Author: simon
 */

#ifndef INC_BLDC_DRIVER_ADAPTER_H_
#define INC_BLDC_DRIVER_ADAPTER_H_

#include "stm32f3xx_hal.h"

void initBLDCDriver(ADC_HandleTypeDef *pShuntA_hallB_ADC_handle_param,
		ADC_HandleTypeDef *pShuntB_hallA_ADC_handle_param,
		ADC_HandleTypeDef *pUser_ADC_handle_param,
		DAC_HandleTypeDef *pVirtZero_DAC_handler_param,
		TIM_HandleTypeDef *pA_LS_HS_PWM_handle_param,
		TIM_HandleTypeDef *pB_LS_PWM_handle_param,
		TIM_HandleTypeDef *pC_LS_HS_PWM_handle_param,
		TIM_HandleTypeDef *pB_HS_PWM_handle_param,
		TIM_HandleTypeDef *pCallback_Timer_param,
		TIM_HandleTypeDef *pSystemtime_Timer_param,
		SPI_HandleTypeDef *pSPI_handle_param,
		UART_HandleTypeDef *pUART_handle_param);

void startupBLDCDriver();
void proceedBLDCDriver();

void phaseAComp_interrupt();
void phaseBComp_interrupt();
void phaseCComp_interrupt();

void hallB_shuntA_adc_interrupt();
void hallA_shuntB_adc_interrupt();

void callbackTimer_interrupt();

#endif /* INC_BLDC_DRIVER_ADAPTER_H_ */
