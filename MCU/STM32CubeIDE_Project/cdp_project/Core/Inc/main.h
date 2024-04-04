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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "BMA456.h"
#include "Steppers.h"
#include "Quaternion.h"
#include "HC05.h"

// setup the includes for the EKF
#define Nsta 3
#define Mobs 3			// 3 state variables and 3 observations - ax, ay, az

#include "tiny_ekf_struct.h"
#include "tiny_ekf.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DATA_Ready_Pin GPIO_PIN_2
#define DATA_Ready_GPIO_Port GPIOE
#define CS_I2C_SPI_Pin GPIO_PIN_3
#define CS_I2C_SPI_GPIO_Port GPIOE
#define INT1_Pin GPIO_PIN_4
#define INT1_GPIO_Port GPIOE
#define INT2_Pin GPIO_PIN_5
#define INT2_GPIO_Port GPIOE
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define OTG_FS_PowerSwitchOn_Pin GPIO_PIN_0
#define OTG_FS_PowerSwitchOn_GPIO_Port GPIOC
#define TMC_2208_2_MS2_Pin GPIO_PIN_15
#define TMC_2208_2_MS2_GPIO_Port GPIOB
#define TMC_2208_DIR_2_Pin GPIO_PIN_8
#define TMC_2208_DIR_2_GPIO_Port GPIOD
#define TMC_2208_STEP_2_Pin GPIO_PIN_9
#define TMC_2208_STEP_2_GPIO_Port GPIOD
#define TMC_2208_EN_2_Pin GPIO_PIN_10
#define TMC_2208_EN_2_GPIO_Port GPIOD
#define TMC_2208_2_MS1_Pin GPIO_PIN_11
#define TMC_2208_2_MS1_GPIO_Port GPIOD
#define LD4_Pin GPIO_PIN_12
#define LD4_GPIO_Port GPIOD
#define LD3_Pin GPIO_PIN_13
#define LD3_GPIO_Port GPIOD
#define LD5_Pin GPIO_PIN_14
#define LD5_GPIO_Port GPIOD
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOD
#define TMC_2208_1_MS2_Pin GPIO_PIN_6
#define TMC_2208_1_MS2_GPIO_Port GPIOC
#define TMC2208_EN_1_Pin GPIO_PIN_7
#define TMC2208_EN_1_GPIO_Port GPIOC
#define TMC2208_DIR_1_Pin GPIO_PIN_8
#define TMC2208_DIR_1_GPIO_Port GPIOC
#define TMC2208_STEP_1_Pin GPIO_PIN_9
#define TMC2208_STEP_1_GPIO_Port GPIOC
#define TMC_2208_1_MS1_Pin GPIO_PIN_8
#define TMC_2208_1_MS1_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define HC05_AT_Pin GPIO_PIN_15
#define HC05_AT_GPIO_Port GPIOA
#define AMT103_2_B_Pin GPIO_PIN_10
#define AMT103_2_B_GPIO_Port GPIOC
#define AMT103_2_A_Pin GPIO_PIN_11
#define AMT103_2_A_GPIO_Port GPIOC
#define AMT103_1_B_Pin GPIO_PIN_12
#define AMT103_1_B_GPIO_Port GPIOC
#define AMT103_1_A_Pin GPIO_PIN_0
#define AMT103_1_A_GPIO_Port GPIOD
#define Audio_RST_Pin GPIO_PIN_4
#define Audio_RST_GPIO_Port GPIOD
#define OTG_FS_OverCurrent_Pin GPIO_PIN_5
#define OTG_FS_OverCurrent_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define I2C1_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_9
#define I2C1_SDA_GPIO_Port GPIOB
#define I2C_INTERRUPT_Pin GPIO_PIN_1
#define I2C_INTERRUPT_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
