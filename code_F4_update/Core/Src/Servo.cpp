/*
 * Servo.cpp
 *
 *  Created on: Apr 24, 2025
 *      Author: APC
 */
#include "Servo.h"

void Servo_SetAngle(uint16_t angle) {
    if(angle > 180) angle = 180;
    uint16_t pulse_length = 1000 + ((angle * 1000) / 180); // Từ 1000 đến 2000
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pulse_length);
}



