/*
 * HC05.h
 *
 *  Created on: Apr 2, 2024
 *      Author: steve
 */

#ifndef INC_HC05_H_
#define INC_HC05_H_

typedef enum {
    MODE_POINTING,
    MODE_STANDBY,
    MODE_CALIBRATION,
    MODE_HEALTH_CHECK
} SystemMode;

typedef struct {
    SystemMode currentMode;
    float altitude;	// Altitude of object to point at
    float azimuth;	// Azimuth of object to point at
} HC05_ModeStatus;


void HC05_ProcessCommand(char* command, HC05_ModeStatus* status, UART_HandleTypeDef* huart);
void HC05_pair(int*, UART_HandleTypeDef);

#endif /* INC_HC05_H_ */
