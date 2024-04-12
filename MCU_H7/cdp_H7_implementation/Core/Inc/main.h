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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "BMA456.h"
#include "Steppers.h"
#include "Quaternion.h"
#include "HC05.h"

#define RX_BUFFER_SIZE 18 // RX buffer can hold up to 13 bytes

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
#define TMC_2208_1_MS1_Pin GPIO_PIN_2
#define TMC_2208_1_MS1_GPIO_Port GPIOE
#define TMC_2208_STEP_1_Pin GPIO_PIN_3
#define TMC_2208_STEP_1_GPIO_Port GPIOE
#define TMC_2208_1_MS2_Pin GPIO_PIN_4
#define TMC_2208_1_MS2_GPIO_Port GPIOE
#define TMC_2208_DIR_1_Pin GPIO_PIN_5
#define TMC_2208_DIR_1_GPIO_Port GPIOE
#define TMC_2208_EN_1_Pin GPIO_PIN_6
#define TMC_2208_EN_1_GPIO_Port GPIOE
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define TMC_2208_EN_2_Pin GPIO_PIN_8
#define TMC_2208_EN_2_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define LED_GREEN_Pin GPIO_PIN_0
#define LED_GREEN_GPIO_Port GPIOB
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_14
#define LED_RED_GPIO_Port GPIOB
#define STLK_VCP_RX_Pin GPIO_PIN_8
#define STLK_VCP_RX_GPIO_Port GPIOD
#define STLK_VCP_TX_Pin GPIO_PIN_9
#define STLK_VCP_TX_GPIO_Port GPIOD
#define USB_FS_PWR_EN_Pin GPIO_PIN_10
#define USB_FS_PWR_EN_GPIO_Port GPIOD
#define AMT103_1_B_Pin GPIO_PIN_2
#define AMT103_1_B_GPIO_Port GPIOG
#define AMT103_1_A_Pin GPIO_PIN_3
#define AMT103_1_A_GPIO_Port GPIOG
#define USB_FS_OVCR_Pin GPIO_PIN_7
#define USB_FS_OVCR_GPIO_Port GPIOG
#define USB_FS_VBUS_Pin GPIO_PIN_9
#define USB_FS_VBUS_GPIO_Port GPIOA
#define USB_FS_ID_Pin GPIO_PIN_10
#define USB_FS_ID_GPIO_Port GPIOA
#define USB_FS_DM_Pin GPIO_PIN_11
#define USB_FS_DM_GPIO_Port GPIOA
#define USB_FS_DP_Pin GPIO_PIN_12
#define USB_FS_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define AMT103_2_B_Pin GPIO_PIN_11
#define AMT103_2_B_GPIO_Port GPIOC
#define AMT103_2_A_Pin GPIO_PIN_12
#define AMT103_2_A_GPIO_Port GPIOC
#define TMC_2208_2_MS1_Pin GPIO_PIN_3
#define TMC_2208_2_MS1_GPIO_Port GPIOD
#define TMC_2208_2_MS2_Pin GPIO_PIN_4
#define TMC_2208_2_MS2_GPIO_Port GPIOD
#define TMC_2208_STEP_2_Pin GPIO_PIN_6
#define TMC_2208_STEP_2_GPIO_Port GPIOD
#define TMC_2208_DIR_2_Pin GPIO_PIN_7
#define TMC_2208_DIR_2_GPIO_Port GPIOD
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LED_YELLOW_Pin GPIO_PIN_1
#define LED_YELLOW_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
