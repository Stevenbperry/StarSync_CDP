/**
  ******************************************************************************
  * @file           : BMA456.c
  * @brief          : BMA456 accelerometer implementation for CDP
  * @author			: Steven Perry
  ******************************************************************************
  *
  ******************************************************************************
  */

#include "main.h"

void BMA456_Calibration(I2C_HandleTypeDef serial, double* xCal, double* yCal, double* zCal, ekf_t ekf){

  	int increment = 0;
	double accel[3];
	double filtered[6] = {0, 0, 0, 0, 0, 0};
  	while (increment < 10000) {
		increment++;
		int16_t accelX, accelY, accelZ;
		BMA456_ReadAccelData(&accelX, &accelY, &accelZ, serial);

		// Convert to double for calculation (assuming BMA456 scale is set for +-2g and 12-bit resolution)
		accel[0] = ((double)accelX / BMA456_FSR * 9.80665);
		accel[1] = ((double)accelY / BMA456_FSR * 9.80665);
		accel[2] = ((double)accelZ / BMA456_FSR * 9.80665);


		model(&ekf, accel);

		ekf_step(&ekf, accel);

		filtered[0] = ekf.x[0];
		filtered[1] = ekf.x[1];
		filtered[2] = ekf.x[2];
		filtered[3] = ekf.x[3];
		filtered[4] = ekf.x[4];
		filtered[5] = ekf.x[5];


  	}
  	*xCal = filtered[0] - 0;
  	*yCal = filtered[1] - 0;
  	*zCal = filtered[2] - 9.80556;
}

/*
 * @param I2C_HandleTypeDef serial: the serial line being used (like hi2c1)
 *
 */

void BMA456_Startup(I2C_HandleTypeDef serial){
	/* Starts up the accelerometer, sets to average 128 samples per output */
	HAL_Delay(15);
	BMA456_Write(0x7E, 0xB6, serial); // CMD: resets the device
	HAL_Delay(15); // wait 2 ms for sensor to start

	BMA456_Write(0x40, (0x07 << 4) | 0x03, serial); // ACC_CONF: sets to average 128 samples
	BMA456_Write(0x7D, 0x04, serial); // PWR_CTRL: enables accelerometer
	HAL_Delay(2); // wait 2 ms for sensor to start

	BMA456_Write(0x7C, 0x03, serial); // PWR_CONF: turns fifo on and low power mode on
	BMA456_Write(0x41, 0x00, serial); // ACC_RANGE: sets g-range to +-2g


}
/*
 * @param uint8_t* flag: variable to hold value of accelerometer error flags
 * @param I2C_HandleTypeDef serial: the serial line being used (like hi2c1)
 *
 */
void BMA456_ReadErrorFlag(uint8_t* flag, I2C_HandleTypeDef serial){
	// Reads error flag register
	uint8_t data[1] = {0};
    uint16_t size = sizeof(data);
	if(BMA456_Read(0x02, data, &size, serial) == HAL_OK){
		*flag = data[0];
	} else {
		*flag = 0xFF;
	}
}
/*
 * @param uint8_t* flag: variable to hold value of accelerometer data ready variable
 * @param I2C_HandleTypeDef serial: the serial line being used (like hi2c1)
 *
 * 	CURRENTLY UNUSED!!
 */
void BMA456_ReadStatus(uint8_t* flag, I2C_HandleTypeDef serial) {
	// Reads Data Ready (DRDY) flag - if it is 1, then can read accelerations
	uint8_t data[1] = {0};
    uint16_t size = sizeof(data);
	if(BMA456_Read(0x03, data, &size, serial) == HAL_OK) {
		*flag = (data[0] & 0x40) >> 7;
	}
}
/*
 * @param int16_t* accelX: acceleration in x-axis
 * @param int16_t* accelY: acceleration in y-axis
 * @param int16_t* accelZ: acceleration in z-axis
 * @param I2C_HandleTypeDef serial: the serial line being used (like hi2c1)
 *
 */
void BMA456_ReadAccelData(int16_t* accelX, int16_t* accelY, int16_t* accelZ,
		I2C_HandleTypeDef serial) {
    uint8_t data[6] = {0};
    uint16_t size = sizeof(data);
    if(BMA456_Read(0x12, data, &size, serial) == HAL_OK) {
        // Convert the data to 16-bit integers
        *accelX = (int16_t)(data[0] | data[1] << 8);
        *accelY = (int16_t)(data[2] | data[3] << 8);
        *accelZ = (int16_t)(data[4] | data[5] << 8);
    } else {
        Error_Handler();
    }
}
/*
 * @param uint8_t reg: the device register you are writing to
 * @param uint8_t data: the singular byte of data you want to write to the register
 * @param I2C_HandleTypeDef serial: the serial line being used (like hi2c1)
 *
 * It shouldn't be necessary to expand data to a larger variable, as any
 * register is only going to hold 8 bits.
 */
void BMA456_Write(uint8_t reg, uint8_t data, I2C_HandleTypeDef serial) {
	uint8_t buf[2] = {reg, data};
    HAL_I2C_Master_Transmit(&serial, BMA_456_1, (uint8_t*) &buf, 2, MAX_DEL);
}
/*
 * @param uint8_t reg: The device register you are writing to
 * @param uint8_t* data: A pointer to an array that, after this function is called, will
 * contain the data you are reading
 * @param uint16_t* size: A pointer to a value representing the number of bytes of
 * data you are expecting to read. Found by the sizeof() operator on the data array.
 * @param I2C_HandleTypeDef serial: the serial line being used (like hi2c1)
 *
 * It shouldn't be necessary to expand data to a larger variable, as any
 * register is only going to hold 8 bits.
 */
HAL_StatusTypeDef BMA456_Read(uint8_t reg, uint8_t* data, uint16_t* size,
		I2C_HandleTypeDef serial) {
	HAL_StatusTypeDef result;
    result = HAL_I2C_Master_Transmit(&serial, BMA_456_1, &reg, 1, MAX_DEL);
    if(result != HAL_OK) {
    	return result;
    }
    result = HAL_I2C_Master_Receive(&serial, BMA_456_1, data, *size, MAX_DEL);

    return result;
}



