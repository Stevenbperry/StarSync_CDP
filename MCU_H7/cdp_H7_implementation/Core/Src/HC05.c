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
	char msg[150];
	int prev_mode = status->currentMode;
	if (command[0] == 'P') {
	    float altitude, azimuth;
	    int parsed = sscanf(command + 1, "%f,%f", &altitude, &azimuth); // Parses the string with a comma as the delimiter

	    if (parsed == 2) { // Check if both values were successfully parsed
	        status->currentMode = MODE_POINTING;  // Update the mode only if parsing is successful
	        status->reference_altitude = altitude;
	        status->reference_azimuth = azimuth;
	        sprintf(msg, "Updated to Altitude: %f, Azimuth: %f \r\n", altitude, azimuth);
	        HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg), 100);
	    } else {
	        sprintf(msg, "Failed to parse altitude and azimuth data. Previous reference vector unchanged.\r\n");
	        HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg), 100);
	    }
	}else if (command[0] == 'S') {
        status->currentMode = MODE_STANDBY;
    } else if (command[0] == 'C') {
    	if (command[1] == 'A'){
        	// Calibrate accelerometers
            status->currentMode = MODE_CALIBRATION_A;

    	}else if (command[1] == 'M') {
    		// Calibrate magnetometers
            status->currentMode = MODE_CALIBRATION_M;

    	}else if (command[1] == 'F'){
    		// Update magnetometer constants (F for finished)
    		float H[3][3];  // Hard iron matrix
    		float S[3];     // Scale factors
    	    char* token;
    	    int index = 0;
    	    command += 2;

    	    // Get the first token
    	    token = strtok(command, ",");

    	    // Walk through other tokens
    	    while (token != NULL) {
    	        if (index < 9) {  // First 9 tokens are matrix entries
    	            int row = index / 3;
    	            int col = index % 3;
    	            sscanf(token, "%f", &H[row][col]);
    	        } else if (index < 12) {  // Next 3 tokens are scale factors
    	            sscanf(token, "%f", &S[index - 9]);
    	        }

    	        token = strtok(NULL, ",");
    	        index++;
    	    }
    	    memcpy(status->hard_iron, H, sizeof(status->hard_iron));
    	    memcpy(status->soft_iron, S, sizeof(status->soft_iron));
	        sprintf(msg, "Updated magnetometer calibration.\r\n");
	        HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg), 100);
    	} else{
	        sprintf(msg, "Invalid calibration request.\r\n");
	        HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg), 100);
    	}
    } else if (command[0] == 'H') {
        status->currentMode = MODE_HEALTH_CHECK;
    } else if (command[0] == 'M') {
    	float mag_x, mag_y, mag_z, dec;
	    int parsed = sscanf(command + 1, "%f,%f,%f,%f", &mag_x, &mag_y, &mag_z, &dec); // Parses the string with a comma as the delimiter

	    if (parsed == 4) { // Check if both values were successfully parsed
	        status->ref_mag_x = mag_x;
	        status->ref_mag_y = mag_y;
	        status->ref_mag_z = mag_z;
	        status->ref_dec = dec;
	        sprintf(msg, "Updated to X: %f, Y: %f Z: %f DEC: %f\r\n", mag_x, mag_y, mag_z, dec);
	        HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg), 100);
	    } else {
	        sprintf(msg, "Failed to parse magnetic field data. Previous reference vector unchanged.\r\n");
	        HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg), 100);
	    }
    } else {
		sprintf(msg, "Unknown mode change - no action taken\r\n");
		HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg), 100);
    }

    // For demonstration, send back the current mode as a confirmation
    if(prev_mode != status->currentMode){
		sprintf(msg, "Mode changed: %d\r\n", status->currentMode);
		HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg), 100);
    }else{
		sprintf(msg, "Mode unchanged.\r\n");
		HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg), 100);
    }
}
