/*
 * Sevor.h
 *
 *  Created on: Apr 24, 2025
 *      Author: APC
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "main.h"

extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim9;

void Servo_SetAngle(uint16_t angle);



#endif /* INC_SERVO_H_ */
