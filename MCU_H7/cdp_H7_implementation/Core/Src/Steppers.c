/**
  ******************************************************************************
  * @file           : Steppers.c
  * @brief          : Using TMC2208 stepper driver to driver NEMA 23 motors for CDP
  * @author 		: Steven Perry
  ******************************************************************************
  *
  ******************************************************************************
  */



#include "main.h"

/*
 * Enables the TMC2208 driver
 */
void TMC2208_Enable(void){
	HAL_GPIO_WritePin(GPIOC, TMC_2208_EN_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, TMC_2208_EN_2_Pin, GPIO_PIN_RESET);
}
/*
 * Disables the TMC2208 driver
 */
void TMC2208_Disable(void){
	HAL_GPIO_WritePin(GPIOC, TMC_2208_EN_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, TMC_2208_EN_2_Pin, GPIO_PIN_SET);
}
/*
 * Sets the DIR pin for a stepper to low
 */
void TMC2208_Forward(int motor){
	if(motor==1){
		HAL_GPIO_WritePin(GPIOD, TMC_2208_DIR_1_Pin, GPIO_PIN_SET);
	}
	if(motor==2){
		HAL_GPIO_WritePin(GPIOD, TMC_2208_DIR_2_Pin, GPIO_PIN_SET);
	}
}
/*
 * Sets the DIR pin for a stepper to high
 */
void TMC2208_Backward(int motor){
	if(motor==1){
		HAL_GPIO_WritePin(GPIOD, TMC_2208_DIR_1_Pin, GPIO_PIN_RESET);
	}
	if(motor==2){
		HAL_GPIO_WritePin(GPIOD, TMC_2208_DIR_2_Pin, GPIO_PIN_RESET);
	}
}
/*
 * Steps a stepper motor
 */
void TMC2208_Step(int motor){
	if(motor == 1){
		HAL_GPIO_WritePin(GPIOD, TMC_2208_STEP_1_Pin, GPIO_PIN_SET);
		HAL_Delay(10);
		HAL_GPIO_WritePin(GPIOD, TMC_2208_STEP_1_Pin, GPIO_PIN_RESET);
		HAL_Delay(10);
	}
	if(motor == 2){
		HAL_GPIO_WritePin(GPIOD, TMC_2208_STEP_2_Pin, GPIO_PIN_SET);
		HAL_Delay(10);
		HAL_GPIO_WritePin(GPIOD, TMC_2208_STEP_2_Pin, GPIO_PIN_RESET);
		HAL_Delay(10);
	}
}
/*
 * Selects microstep configuration for either stepper motor
 */
void TMC2208_Microstep(int motor, int val){
	int ms1, ms2;
	if(motor == 2){
		ms1 = TMC_2208_2_MS1_Pin;
		ms2 = TMC_2208_2_MS2_Pin;
	}else{
		ms1 = TMC_2208_1_MS1_Pin;
		ms2 = TMC_2208_1_MS2_Pin;
	}
	switch(val){
		case 1:
			HAL_GPIO_WritePin(GPIOD, ms1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, ms2, GPIO_PIN_RESET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOD, ms1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, ms2, GPIO_PIN_SET);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOD, ms1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, ms2, GPIO_PIN_SET);
			break;
		default:
			HAL_GPIO_WritePin(GPIOD, ms1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, ms2, GPIO_PIN_RESET);
			break;
	}
}
