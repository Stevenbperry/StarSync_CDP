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
	char pData[] = "STM32 in AT mode!";
	int Size = sizeof(pData);
	HAL_UART_Transmit(&huart1, (uint8_t*) &pData, Size, 100);
	while (*pairing_flag == 0) {
		// wait
	}
	// turns off AT mode
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

}
