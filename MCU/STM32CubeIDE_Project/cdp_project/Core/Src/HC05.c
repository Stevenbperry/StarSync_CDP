/*
 * HC05.c
 *
 *  Created on: Mar 8, 2024
 *      Author: steve
 */

#include "main.h"

/**
  * @brief Checks the UART line if the HC05 triggers an interrupt.
  * @param UART_HandleTypeDef serial : the UART line being used (huart1)
  * @param uint8_t* data : pointer to array of data
  * @param uint8_t size : amount of data expected
  * @param int* flag : pointer to interrupt flag
  * @retval int : Integer representing the command received
  */
int HC05_Check(UART_HandleTypeDef serial, uint8_t* data, uint8_t size, int* flag){
	if(*flag == 1){
		*flag = 0;
		if (HAL_UART_Receive(&serial, data, size, 100) == HAL_OK){
			// #TODO: Implement bluetooth CMD logic
			if (strstr((char*) data, "send") != NULL) {
				return 1;
			} else if (strstr((char*) data, "EXAMPLE_CMD2") != NULL) {
				return 2;
				// repeat for however many commands there are
			} else {
				return 0; // return 0 if there was an invalid command, no response, etc
			}
		} else{
			return 0;
		}
		__HAL_UART_ENABLE_IT(&serial, UART_IT_RXNE);
	} else{
		return -1;	// return -1 if there was no bluetooth interrupt - this is most common
					// response
		// we will call HC05_Check every loop iteration
	}
}




