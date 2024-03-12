/*
 * Steppers.h
 *
 *  Created on: Mar 12, 2024
 *      Author: Steven Perry
 */

#ifndef INC_STEPPERS_H_
#define INC_STEPPERS_H_



// Stepper 1 is on GPIOC
#define TMC_2208_EN_1	0x80	// pin 7
#define TMC_2208_STEP_1	0x200	// pin 9
#define TMC_2208_DIR_1	0x100	// pin 8


// Stepper 2 is on GPIOD
#define TMC_2208_EN_2	0x400	// pin 10
#define TMC_2208_STEP_2	0x200	// pin 9
#define TMC_2208_DIR_2	0x100	// pin 8

void Enable_Steppers(void);
void Disable_Steppers(void);
void Step1_Forward(void);
void Step2_Forward(void);
void Step1_Backward(void);
void Step2_Backward(void);
void Step1(void);
void Step2(void);


#endif /* INC_STEPPERS_H_ */
