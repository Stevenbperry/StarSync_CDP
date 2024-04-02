/**
  ******************************************************************************
  * @file           : HC05.c
  * @brief          : HC05 Bluetooth module implementation for CDP
  * @author			: Steven Perry
  ******************************************************************************
  *
  ******************************************************************************
  */

#include "main.h"


/**
  * @brief If user push putton is detected, go into pairing mode.
  * @param None
  * @retval None
  */
void HC05_pair(int* pairing_flag, UART_HandleTypeDef huart1){
	// sets the HC05 into AT mode
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	char data1[] = "STM32 in pairing mode!";
	int size = sizeof(data1);
	HAL_UART_Transmit(&huart1, (uint8_t*) &data1, size, 100);
	while (*pairing_flag == 0) {
		// wait
	}
	char data2[] = "STM32 leaving pairing mode!";
	size = sizeof(data2);
	HAL_UART_Transmit(&huart1, (uint8_t*) &data2, size, 100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

}
