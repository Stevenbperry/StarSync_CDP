/**
 ******************************************************************************
 * @file    lis3mdl.c
 * @author  MCD Application Team
 * @brief   This file provides a set of functions needed to manage the LIS3MDL
 *          magnetometer devices
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lis3mdl.h"

extern I2C_HandleTypeDef hi2c1;

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Component
  * @{
  */

/** @defgroup LIS3MDL LIS3MDL
  * @{
  */

/** @defgroup LIS3MDL_Mag_Private_Variables LIS3MDL Mag Private Variables
  * @{
  */ 
MAGNETO_DrvTypeDef Lis3mdlMagDrv =
{
    .Init = LIS3MDL_MagInit,
    .DeInit = LIS3MDL_MagDeInit,
    .ReadID = LIS3MDL_MagReadID,
    .Reset = 0,
    .ConfigIT = 0,
    .EnableIT = 0,
    .DisableIT = 0,
    .ITStatus = 0,
    .ClearIT = 0,
    .FilterConfig = 0,
    .FilterCmd = 0,
    .GetXYZ = LIS3MDL_MagReadXYZ
};
/**
  * @}
  */ 


/** @defgroup LIS3MDL_Mag_Private_Functions LIS3MDL Mag Private Functions
  * @{
  */

void Magnetic_Calibration(UART_HandleTypeDef* huart){
	int increment = 0;
	char debugMsg[64];
	sprintf(debugMsg, "CM_BEGIN\n");
    HAL_UART_Transmit(huart, (uint8_t*)debugMsg, strlen(debugMsg), 100);
	while(increment <= 300){
		volatile float data[3];
		char buffer[64];
		LIS3MDL_MagReadXYZ((float*) &data);
        snprintf(buffer, sizeof(buffer), "%f,%f,%f\r\n", data[0], data[1], data[2]);
        HAL_UART_Transmit(huart, (uint8_t*)buffer, strlen(buffer), 100);
		HAL_Delay(100);
		increment++;
	}
	char null_term = '\0';
    HAL_UART_Transmit(huart, (uint8_t*) &null_term, 1, 100);
}

void SENSOR_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  HAL_I2C_Mem_Write(&hi2c1, Addr<<1, Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, 1000);
}

uint8_t SENSOR_IO_Read(uint8_t Addr, uint8_t Reg)
{
  uint8_t data;
  HAL_I2C_Mem_Read(&hi2c1, Addr<<1, Reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
  return data;
}

void SENSOR_IO_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length)
{
  HAL_I2C_Mem_Read(&hi2c1, Addr<<1, Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length, 1000);
}

void SENSOR_IO_WriteMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length)
{
  HAL_I2C_Mem_Write(&hi2c1, Addr<<1, Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length, 1000);
}

/**
  * @brief  Set LIS3MDL Magnetometer Initialization.
  * @param  LIS3MDL_InitStruct: pointer to a LIS3MDL_MagInitTypeDef structure 
  *         that contains the configuration setting for the LIS3MDL.
  */
void LIS3MDL_MagInit(MAGNETO_InitTypeDef LIS3MDL_InitStruct)
{  
  SENSOR_IO_Write(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG1, LIS3MDL_InitStruct.Register1);
  SENSOR_IO_Write(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG2, LIS3MDL_InitStruct.Register2);
  SENSOR_IO_Write(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG3, LIS3MDL_InitStruct.Register3);
  SENSOR_IO_Write(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG4, LIS3MDL_InitStruct.Register4);
  SENSOR_IO_Write(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG5, LIS3MDL_InitStruct.Register5);
}

/**
  * @brief  LIS3MDL Magnetometer De-initialization.
  */
void LIS3MDL_MagDeInit(void)
{
  uint8_t ctrl = 0x00;
  
  /* Read control register 1 value */
  ctrl = SENSOR_IO_Read(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG3);

  /* Clear Selection Mode bits */
  ctrl &= ~(LIS3MDL_MAG_SELECTION_MODE);

  /* Set Power down */
  ctrl |= LIS3MDL_MAG_POWERDOWN2_MODE;
  
  /* write back control register */
  SENSOR_IO_Write(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG3, ctrl);  
}

/**
  * @brief  Read LIS3MDL ID.
  * @retval ID 
  */
uint8_t LIS3MDL_MagReadID(void)
{
  /* IO interface initialization */
  /* Read value at Who am I register address */
  return (SENSOR_IO_Read(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_WHO_AM_I_REG));
}

/**
  * @brief  Set/Unset Magnetometer in low power mode.
  * @param  status 0 means disable Low Power Mode, otherwise Low Power Mode is enabled
  */
void LIS3MDL_MagLowPower(uint16_t status)
{  
  uint8_t ctrl = 0;
  
  /* Read control register 1 value */
  ctrl = SENSOR_IO_Read(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG3);

  /* Clear Low Power Mode bit */
  ctrl &= ~(0x20);

  /* Set Low Power Mode */
  if(status)
  {
    ctrl |= LIS3MDL_MAG_CONFIG_LOWPOWER_MODE;
  }else
  {
    ctrl |= LIS3MDL_MAG_CONFIG_NORMAL_MODE;
  }
  
  /* write back control register */
  SENSOR_IO_Write(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG3, ctrl);  
}

/**
  * @brief  Read X, Y & Z Magnetometer values 
  * @param  pData: Data out pointer
  */
void LIS3MDL_MagReadXYZ(float* pData)
{
  int16_t pnRawData[3];
  uint8_t ctrlm= 0;
  uint8_t buffer[6];
  uint8_t i = 0;
  float sensitivity = 0;
  
  /* Read the magnetometer control register content */
  ctrlm = SENSOR_IO_Read(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG2);
  
  /* Read output register X, Y & Z acceleration */
  SENSOR_IO_ReadMultiple(LIS3MDL_MAG_I2C_ADDRESS_HIGH, (LIS3MDL_MAG_OUTX_L | 0x80), buffer, 6);
  
  for(i=0; i<3; i++)
  {
    pnRawData[i]=((((uint16_t)buffer[2*i+1]) << 8) + (uint16_t)buffer[2*i]);
  }
  
  /* Normal mode */
  /* Switch the sensitivity value set in the CRTL_REG2 */
  switch(ctrlm & 0x60)
  {
  case LIS3MDL_MAG_FS_4_GA:
    sensitivity = LIS3MDL_MAG_SENSITIVITY_FOR_FS_4GA;
    break;
  case LIS3MDL_MAG_FS_8_GA:
    sensitivity = LIS3MDL_MAG_SENSITIVITY_FOR_FS_8GA;
    break;
  case LIS3MDL_MAG_FS_12_GA:
    sensitivity = LIS3MDL_MAG_SENSITIVITY_FOR_FS_12GA;
    break;
  case LIS3MDL_MAG_FS_16_GA:
    sensitivity = LIS3MDL_MAG_SENSITIVITY_FOR_FS_16GA;
    break;    
  }
  
  /* Obtain the mGauss value for the three axis */
  for(i=0; i<3; i++)
  {
    pData[i]= (pnRawData[i] * sensitivity);
  }
}


/**
  * @}
  */ 

/**
  * @}
  */
  
/**
  * @}
  */
  
/**
  * @}
  */
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
  
