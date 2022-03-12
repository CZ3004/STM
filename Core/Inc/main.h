/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OLED_SCL_Pin GPIO_PIN_5
#define OLED_SCL_GPIO_Port GPIOE
#define OLED_SDA_Pin GPIO_PIN_6
#define OLED_SDA_GPIO_Port GPIOE
#define MOTOR_AIN2_Pin GPIO_PIN_2
#define MOTOR_AIN2_GPIO_Port GPIOA
#define MOTOR_AIN1_Pin GPIO_PIN_3
#define MOTOR_AIN1_GPIO_Port GPIOA
#define MOTOR_BIN1_Pin GPIO_PIN_4
#define MOTOR_BIN1_GPIO_Port GPIOA
#define MOTOR_BIN2_Pin GPIO_PIN_5
#define MOTOR_BIN2_GPIO_Port GPIOA
#define ENC_B2_Pin GPIO_PIN_6
#define ENC_B2_GPIO_Port GPIOA
#define ENC_B1_Pin GPIO_PIN_7
#define ENC_B1_GPIO_Port GPIOA
#define OLED_RST_Pin GPIO_PIN_7
#define OLED_RST_GPIO_Port GPIOE
#define OLED_DC_Pin GPIO_PIN_8
#define OLED_DC_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_10
#define LED3_GPIO_Port GPIOE
#define SERVO_PWM_Pin GPIO_PIN_14
#define SERVO_PWM_GPIO_Port GPIOE
#define BTN_USER_Pin GPIO_PIN_8
#define BTN_USER_GPIO_Port GPIOD
#define MOTOR_PWMA_Pin GPIO_PIN_6
#define MOTOR_PWMA_GPIO_Port GPIOC
#define MOTOR_PWMB_Pin GPIO_PIN_7
#define MOTOR_PWMB_GPIO_Port GPIOC
#define ENC_A2_Pin GPIO_PIN_15
#define ENC_A2_GPIO_Port GPIOA
#define US_Output_Pin GPIO_PIN_4
#define US_Output_GPIO_Port GPIOD
#define ENC_A1_Pin GPIO_PIN_3
#define ENC_A1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
