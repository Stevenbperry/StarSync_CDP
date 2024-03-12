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
void Enable_Steppers(void){
	HAL_GPIO_WritePin(GPIOC, TMC_2208_EN_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, TMC_2208_EN_2, GPIO_PIN_SET);
}
/*
 * Disables the TMC2208 driver
 */
void Disable_Steppers(void){
	HAL_GPIO_WritePin(GPIOC, TMC_2208_EN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, TMC_2208_EN_2, GPIO_PIN_RESET);
}
/*
 * Sets the DIR pin for the 1st stepper to high
 */
void Step1_Forward(void){
	HAL_GPIO_WritePin(GPIOC, TMC_2208_DIR_1, GPIO_PIN_SET);
}
/*
 * Sets the DIR pin for the 1st stepper to low
 */
void Step1_Backward(void){
	HAL_GPIO_WritePin(GPIOC, TMC_2208_DIR_1, GPIO_PIN_RESET);
}
/*
 * Sets the DIR pin for the 2nd stepper to high
 */
void Step2_Forward(void){
	HAL_GPIO_WritePin(GPIOD, TMC_2208_DIR_2, GPIO_PIN_SET);
}
/*
 * Sets the DIR pin for the 2nd stepper to high
 */
void Step2_Backward(void){
	HAL_GPIO_WritePin(GPIOD, TMC_2208_DIR_2, GPIO_PIN_RESET);
}
/*
 * Steps the 1st stepper motor
 */
void Step1(void){
	HAL_GPIO_WritePin(GPIOC, TMC_2208_STEP_1, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOC, TMC_2208_STEP_1, GPIO_PIN_RESET);
}
/*
 * Steps the 2nd stepper motor
 */
void Step2(void){
	HAL_GPIO_WritePin(GPIOD, TMC_2208_STEP_2, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOD, TMC_2208_STEP_2, GPIO_PIN_RESET);
}

