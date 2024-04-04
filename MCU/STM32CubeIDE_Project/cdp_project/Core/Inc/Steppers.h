/*
 * Steppers.h
 *
 *  Created on: Mar 12, 2024
 *      Author: Steven Perry
 */

#ifndef INC_STEPPERS_H_
#define INC_STEPPERS_H_

void enable_steppers(void);
void disable_steppers(void);
void direction_forward(int);
void direction_backward(int);
void step(int);
void microstep(int, int);


#endif /* INC_STEPPERS_H_ */
