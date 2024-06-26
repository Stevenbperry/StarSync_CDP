/*
 * BMA456.h
 *
 *  Created on: Mar 7, 2024
 *      Author: Steven Perry
 */

#ifndef INC_BMA456_H_
#define INC_BMA456_H_


#define BMA_456_1 0x18 << 1
#define MAX_DEL 1000
#define BMA456_FSR 16384 // full scale range at +-2g
#define BMA456_2_X_OFFSET 3.67749375
#define BMA456_2_Y_OFFSET 0.010175356
#define BMA456_2_Z_OFFSET -5.9748373438

void BMA456_Write(uint8_t, uint8_t, I2C_HandleTypeDef);
HAL_StatusTypeDef BMA456_Read(uint8_t, uint8_t*, uint16_t*, I2C_HandleTypeDef);
void BMA456_ReadAccelData(int16_t*, int16_t*, int16_t*, I2C_HandleTypeDef);
void BMA456_Startup(I2C_HandleTypeDef);
void BMA456_ReadErrorFlag(uint8_t*, I2C_HandleTypeDef);
void BMA456_ReadStatus(uint8_t*, I2C_HandleTypeDef);



#endif /* INC_BMA456_H_ */
