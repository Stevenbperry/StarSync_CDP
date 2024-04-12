/*
 * Steppers.h
 *
 *  Created on: Mar 12, 2024
 *      Author: Steven Perry
 */

#ifndef INC_STEPPERS_H_
#define INC_STEPPERS_H_

void TMC2208_Enable(void);
void TMC2208_Disable(void);
void TMC2208_Forward(int);
void TMC2208_Backward(int);
void TMC2208_Step(int);
void TMC2208_Microstep(int, int);


#endif /* INC_STEPPERS_H_ */
