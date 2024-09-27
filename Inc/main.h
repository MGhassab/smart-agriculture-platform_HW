/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define		DEVICE_TYPE								1				// 1 -> Simple			2 -> Abghiri			
																							

#if(DEVICE_TYPE == 1)												// Type : Simple    PCB : P200
#define   BUTTONS_NUM								2				// External BUTTONS number
#define   OUTPUTS_NUM								2				// outputs  number
#define		level control
#endif

#if(DEVICE_TYPE == 2)												// Type : Abghiri	  PCB : P200 			
#define   BUTTONS_NUM								3				// External BUTTONS number
#define   OUTPUTS_NUM								3				// outputs  number
#endif



/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void		DEBUG(char* input);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define flow1_EXIT_Pin GPIO_PIN_3
#define flow1_EXIT_GPIO_Port GPIOA
#define flow1_EXIT_EXTI_IRQn EXTI3_IRQn
#define flow2_EXIT_Pin GPIO_PIN_6
#define flow2_EXIT_GPIO_Port GPIOA
#define flow2_EXIT_EXTI_IRQn EXTI9_5_IRQn
#define NSS_Pin GPIO_PIN_12
#define NSS_GPIO_Port GPIOB
#define RST_Pin GPIO_PIN_9
#define RST_GPIO_Port GPIOD
#define SW3_Pin GPIO_PIN_10
#define SW3_GPIO_Port GPIOD
#define SW3_EXTI_IRQn EXTI15_10_IRQn
#define SW_Light1_Pin GPIO_PIN_2
#define SW_Light1_GPIO_Port GPIOG
#define SW_Light3_Pin GPIO_PIN_3
#define SW_Light3_GPIO_Port GPIOG
#define SW_Light2_Pin GPIO_PIN_4
#define SW_Light2_GPIO_Port GPIOG
#define SW1_Pin GPIO_PIN_8
#define SW1_GPIO_Port GPIOG
#define SW1_EXTI_IRQn EXTI9_5_IRQn
#define SW2_Pin GPIO_PIN_9
#define SW2_GPIO_Port GPIOC
#define SW2_EXTI_IRQn EXTI9_5_IRQn
#define SIM_PWR_Pin GPIO_PIN_8
#define SIM_PWR_GPIO_Port GPIOA
#define SIM_TX_Pin GPIO_PIN_9
#define SIM_TX_GPIO_Port GPIOA
#define SIM_RX_Pin GPIO_PIN_10
#define SIM_RX_GPIO_Port GPIOA
#define SIM_STATUS_Pin GPIO_PIN_11
#define SIM_STATUS_GPIO_Port GPIOA
#define SIM_NET_Pin GPIO_PIN_12
#define SIM_NET_GPIO_Port GPIOA
#define DEBUG_TX_Pin GPIO_PIN_10
#define DEBUG_TX_GPIO_Port GPIOC
#define DEBUG_RX_Pin GPIO_PIN_11
#define DEBUG_RX_GPIO_Port GPIOC
#define relay1_Pin GPIO_PIN_0
#define relay1_GPIO_Port GPIOD
#define relay2_Pin GPIO_PIN_1
#define relay2_GPIO_Port GPIOD
#define relay3_Pin GPIO_PIN_3
#define relay3_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
