/*
 * Steppers.h
 *
 *  Created on: Mar 12, 2024
 *      Author: Steven Perry
 */

#ifndef INC_STEPPERS_H_
#define INC_STEPPERS_H_

void Enable_Steppers(void);
void Disable_Steppers(void);
void Direction_Forward(int);
void Direction_Backward(int);
void Step(int);
void Microstep(int, int);


#endif /* INC_STEPPERS_H_ */
