/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>

#include "system_state.h"
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
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;

void app_support_init(void);
void app_delay_us(uint32_t microseconds);
bool taskReadDistanceSensor(uint8_t sensor_index, distance_data_t *distance_data);
warning_level_t taskPickWarningLevel(float distance_cm, system_status_t system_status);
void taskUpdateAlert(const system_state_t *combined_state,
                     const system_state_t *sensor1_state,
                     const system_state_t *sensor2_state);
void taskUpdateDisplay(const system_state_t *sensor1_state,
                       const system_state_t *sensor2_state);
void taskHandleEchoEdgeIsr(uint16_t gpio_pin);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Echo_Pin GPIO_PIN_2
#define Echo_GPIO_Port GPIOA
#define echo2_Pin GPIO_PIN_4
#define echo2_GPIO_Port GPIOA
#define led_Pin GPIO_PIN_5
#define led_GPIO_Port GPIOB
#define buzz_Pin GPIO_PIN_6
#define buzz_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* Trung ten voi code ung dung (PA1 = Trig1, PA3 = Trig2 — khop MX_GPIO_Init) */
#define TRig_Pin GPIO_PIN_1
#define TRig_GPIO_Port GPIOA
#define trig2_Pin GPIO_PIN_3
#define trig2_GPIO_Port GPIOA
/* LED2: doi neu CubeMX gan chan khac (hien khop PB4 = GPIO_PIN_4) */
#define led2_Pin GPIO_PIN_4
#define led2_GPIO_Port GPIOB
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
