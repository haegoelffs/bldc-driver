/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define IR_COMP_C_old_Pin GPIO_PIN_14
#define IR_COMP_C_old_GPIO_Port GPIOC
#define IR_COMP_B_Pin GPIO_PIN_15
#define IR_COMP_B_GPIO_Port GPIOC
#define IR_COMP_B_EXTI_IRQn EXTI15_10_IRQn
#define IR_COMP_A_Pin GPIO_PIN_0
#define IR_COMP_A_GPIO_Port GPIOC
#define IR_COMP_A_EXTI_IRQn EXTI0_IRQn
#define ADC_MAIN_VOLTAGE_Pin GPIO_PIN_1
#define ADC_MAIN_VOLTAGE_GPIO_Port GPIOC
#define DO_MAIN_SWITCH_Pin GPIO_PIN_2
#define DO_MAIN_SWITCH_GPIO_Port GPIOC
#define DI_MAIN_BUTTON_Pin GPIO_PIN_3
#define DI_MAIN_BUTTON_GPIO_Port GPIOC
#define ADC_SHUNT_A_Pin GPIO_PIN_0
#define ADC_SHUNT_A_GPIO_Port GPIOA
#define DO_DRIVER_EN_Pin GPIO_PIN_1
#define DO_DRIVER_EN_GPIO_Port GPIOA
#define DO_DRIVER_DC_CAL_Pin GPIO_PIN_2
#define DO_DRIVER_DC_CAL_GPIO_Port GPIOA
#define ADC_HALL_B_Pin GPIO_PIN_3
#define ADC_HALL_B_GPIO_Port GPIOA
#define DAC_VIRT_ZERO_Pin GPIO_PIN_4
#define DAC_VIRT_ZERO_GPIO_Port GPIOA
#define SPI_SCK_Pin GPIO_PIN_5
#define SPI_SCK_GPIO_Port GPIOA
#define SPI_MISO_Pin GPIO_PIN_6
#define SPI_MISO_GPIO_Port GPIOA
#define SPI_MOSI_Pin GPIO_PIN_7
#define SPI_MOSI_GPIO_Port GPIOA
#define DI_DRIVER_PWRGD_Pin GPIO_PIN_4
#define DI_DRIVER_PWRGD_GPIO_Port GPIOC
#define ADC_USER_IN_old_Pin GPIO_PIN_5
#define ADC_USER_IN_old_GPIO_Port GPIOC
#define DI_DRIVER_NOCTW_Pin GPIO_PIN_0
#define DI_DRIVER_NOCTW_GPIO_Port GPIOB
#define ADC_SHUNT_B_old_Pin GPIO_PIN_1
#define ADC_SHUNT_B_old_GPIO_Port GPIOB
#define DI_DRIVER_NFAULT_Pin GPIO_PIN_2
#define DI_DRIVER_NFAULT_GPIO_Port GPIOB
#define UART_TX_Pin GPIO_PIN_10
#define UART_TX_GPIO_Port GPIOB
#define UART_RX_Pin GPIO_PIN_11
#define UART_RX_GPIO_Port GPIOB
#define ADC_HALL_A_Pin GPIO_PIN_12
#define ADC_HALL_A_GPIO_Port GPIOB
#define ADC_USER_IN_Pin GPIO_PIN_13
#define ADC_USER_IN_GPIO_Port GPIOB
#define ADC_SHUNT_B_Pin GPIO_PIN_14
#define ADC_SHUNT_B_GPIO_Port GPIOB
#define PWM_C_LS_old_Pin GPIO_PIN_15
#define PWM_C_LS_old_GPIO_Port GPIOB
#define PWM_C_HS_Pin GPIO_PIN_6
#define PWM_C_HS_GPIO_Port GPIOC
#define PWM_B_LS_Pin GPIO_PIN_7
#define PWM_B_LS_GPIO_Port GPIOC
#define PWM_C_LS_Pin GPIO_PIN_8
#define PWM_C_LS_GPIO_Port GPIOC
#define DO_LED_2_Pin GPIO_PIN_9
#define DO_LED_2_GPIO_Port GPIOC
#define DO_LED_3_Pin GPIO_PIN_8
#define DO_LED_3_GPIO_Port GPIOA
#define DO_LED_4_Pin GPIO_PIN_9
#define DO_LED_4_GPIO_Port GPIOA
#define DO_LED_5_Pin GPIO_PIN_10
#define DO_LED_5_GPIO_Port GPIOA
#define PWM_B_HS_Pin GPIO_PIN_11
#define PWM_B_HS_GPIO_Port GPIOA
#define PWM_A_LS_old_Pin GPIO_PIN_12
#define PWM_A_LS_old_GPIO_Port GPIOA
#define DO_SELECT_BRIDGE_DRIVER_Pin GPIO_PIN_15
#define DO_SELECT_BRIDGE_DRIVER_GPIO_Port GPIOA
#define DO_LED_1_Pin GPIO_PIN_10
#define DO_LED_1_GPIO_Port GPIOC
#define PWM_A_HS_Pin GPIO_PIN_6
#define PWM_A_HS_GPIO_Port GPIOB
#define PWM_A_LS_Pin GPIO_PIN_7
#define PWM_A_LS_GPIO_Port GPIOB
#define DI_USER_IN_Pin GPIO_PIN_8
#define DI_USER_IN_GPIO_Port GPIOB
#define IR_COMP_C_Pin GPIO_PIN_9
#define IR_COMP_C_GPIO_Port GPIOB
#define IR_COMP_C_EXTI_IRQn EXTI9_5_IRQn

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#define PWM_A_HS_channel TIM_CHANNEL_1 // TIM4
#define PWM_B_HS_channel TIM_CHANNEL_4 // TIM1
#define PWM_C_HS_channel TIM_CHANNEL_1 // TIM8
#define PWM_A_LS_channel TIM_CHANNEL_2 // TIM4
#define PWM_B_LS_channel TIM_CHANNEL_2	// TIM3
#define PWM_C_LS_channel TIM_CHANNEL_3 // TIM8
#define DAC_virtual_GND_channel DAC_CHANNEL_1
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
