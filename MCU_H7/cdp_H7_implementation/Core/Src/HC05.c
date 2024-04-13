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
  * @brief Processes the command received by a Bluetooth transmission.
  * @param char* command: The char array containing the command received
  * @param HC05_ModeStatus* status: The struct containing the mode the scope is in.
  * @param UART_HandleTypeDef* huart: The uart channel being used - should be huart1.
  * @retval None
  */
void HC05_ProcessCommand(char* command, Telescope_Status* status, UART_HandleTypeDef* huart) {
	char msg[100];
	int prev_mode = status->currentMode;
	if (command[0] == 'P') {
	    float altitude, azimuth;
	    int parsed = sscanf(command + 1, "%f,%f", &altitude, &azimuth); // Parses the string with a comma as the delimiter

	    if (parsed == 2) { // Check if both values were successfully parsed
	        status->currentMode = MODE_POINTING;  // Update the mode only if parsing is successful
	        status->altitude = altitude;
	        status->azimuth = azimuth;
	        char msg[100];  // Ensure your buffer is large enough to hold the entire message
	        sprintf(msg, "Updated to Altitude: %f, Azimuth: %f \r\n", altitude, azimuth);
	        HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg), 100);
	    } else {
	        char msg[100];  // Ensure your buffer is large enough to hold the entire message
	        sprintf(msg, "Failed to parse altitude and azimuth data. Previous reference vector unchanged.\r\n");
	        HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg), 100);
	    }
	}else if (command[0] == 'S') {
        status->currentMode = MODE_STANDBY;
    } else if (command[0] == 'C') {
        status->currentMode = MODE_CALIBRATION;
    } else if (command[0] == 'H') {
        status->currentMode = MODE_HEALTH_CHECK;
    } else {
		sprintf(msg, "Unknown mode change - no action taken\r\n");
		HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg), 100);
    }

    // For demonstration, send back the current mode as a confirmation
    if(prev_mode != status->currentMode){
		sprintf(msg, "Mode changed: %d\r\n", status->currentMode);
		HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg), 100);
    }
}


/**
  * @brief If user push putton is detected, go into pairing mode.
  * @param None
  * @retval None
  */
void HC05_pair(int* pairing_flag, UART_HandleTypeDef huart1){
	// sets the HC05 into AT mode
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	const char data1[] = "STM32 in pairing mode!";
	HAL_UART_Transmit(&huart1, (uint8_t*) data1, strlen(data1), 100);
	while (*pairing_flag == 0) {
		// wait
	}
	const char data2[] = "STM32 leaving pairing mode!";
	HAL_UART_Transmit(&huart1, (uint8_t*) data2, strlen(data2), 100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

}
