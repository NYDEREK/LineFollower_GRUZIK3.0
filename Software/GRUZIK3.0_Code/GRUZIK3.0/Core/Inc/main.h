/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void motor_control (double pos_right, double pos_left);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LEDON_Pin GPIO_PIN_13
#define LEDON_GPIO_Port GPIOC
#define ENCODER_R_1_Pin GPIO_PIN_0
#define ENCODER_R_1_GPIO_Port GPIOC
#define ENCODER_R_2_Pin GPIO_PIN_1
#define ENCODER_R_2_GPIO_Port GPIOC
#define Motor_R_A_Pin GPIO_PIN_2
#define Motor_R_A_GPIO_Port GPIOC
#define Motor_R_B_Pin GPIO_PIN_3
#define Motor_R_B_GPIO_Port GPIOC
#define PWM_R_Pin GPIO_PIN_0
#define PWM_R_GPIO_Port GPIOA
#define LPUART1_TX_Pin GPIO_PIN_2
#define LPUART1_TX_GPIO_Port GPIOA
#define LPUART1_RX_Pin GPIO_PIN_3
#define LPUART1_RX_GPIO_Port GPIOA
#define SENSOR11_Pin GPIO_PIN_4
#define SENSOR11_GPIO_Port GPIOA
#define SENSOR12_Pin GPIO_PIN_5
#define SENSOR12_GPIO_Port GPIOA
#define LED_4_Pin GPIO_PIN_6
#define LED_4_GPIO_Port GPIOA
#define LED_3_Pin GPIO_PIN_7
#define LED_3_GPIO_Port GPIOA
#define LED_2_Pin GPIO_PIN_4
#define LED_2_GPIO_Port GPIOC
#define LED_1_Pin GPIO_PIN_5
#define LED_1_GPIO_Port GPIOC
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define Motor_STBY_Pin GPIO_PIN_6
#define Motor_STBY_GPIO_Port GPIOC
#define Motor_L_A_Pin GPIO_PIN_8
#define Motor_L_A_GPIO_Port GPIOA
#define Motor_L_B_Pin GPIO_PIN_9
#define Motor_L_B_GPIO_Port GPIOA
#define PWM_L_Pin GPIO_PIN_10
#define PWM_L_GPIO_Port GPIOA
#define ENCODER_L_1_Pin GPIO_PIN_11
#define ENCODER_L_1_GPIO_Port GPIOA
#define ENCODER_L_2_Pin GPIO_PIN_12
#define ENCODER_L_2_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define SENSOR10_Pin GPIO_PIN_15
#define SENSOR10_GPIO_Port GPIOA
#define SENSOR9_Pin GPIO_PIN_10
#define SENSOR9_GPIO_Port GPIOC
#define SENSOR8_Pin GPIO_PIN_11
#define SENSOR8_GPIO_Port GPIOC
#define SENSOR7_Pin GPIO_PIN_12
#define SENSOR7_GPIO_Port GPIOC
#define SENSOR6_Pin GPIO_PIN_2
#define SENSOR6_GPIO_Port GPIOD
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define SENSOR5_Pin GPIO_PIN_4
#define SENSOR5_GPIO_Port GPIOB
#define SENSOR4_Pin GPIO_PIN_5
#define SENSOR4_GPIO_Port GPIOB
#define SENSOR3_Pin GPIO_PIN_6
#define SENSOR3_GPIO_Port GPIOB
#define SENSOR2_Pin GPIO_PIN_7
#define SENSOR2_GPIO_Port GPIOB
#define SENSOR1_Pin GPIO_PIN_9
#define SENSOR1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
