/*
 * HC05.h
 *
 *  Created on: Apr 2, 2024
 *      Author: steve
 */

#ifndef INC_HC05_H_
#define INC_HC05_H_


void HC05_ProcessCommand(char* command, Telescope_Status* status, UART_HandleTypeDef* huart);
void HC05_pair(int*, UART_HandleTypeDef);

#endif /* INC_HC05_H_ */
