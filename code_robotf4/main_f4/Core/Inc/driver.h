/*
 * driver.h
 *
 *  Created on: Mar 19, 2025
 *      Author: APC
 */

#ifndef INC_DRIVER_H_
#define INC_DRIVER_H_

#include "stdbool.h"
#include "math.h"
#include "main.h"
#include "Mapping.h"


// Cấu trúc điều khiển động cơ
typedef struct {
    TIM_HandleTypeDef *htim;  // Timer driver PWM
    uint32_t Channel;         // Channel PWM
    GPIO_TypeDef *IN1_Port;   // Drain GPIO of IN1, example GPIO_B,D
    uint16_t IN1;             // Drain IN1
    GPIO_TypeDef *IN2_Port;   // Drain GPIO of IN2
    uint16_t IN2;             // Drain IN2
} Motor_t;
extern Motor_t motor0 ,motor1, motor2, motor3;

// Khai báo các hàm điều khiển động cơ
void Motor_Init(Motor_t *motor);
void Motor_SetSpeed(Motor_t *motor, int16_t speed);
void tien(uint16_t pwm );
void lui(uint16_t pwm );
void trai(uint16_t pwm );
void phai(uint16_t pwm );
void ps3_ctrl();
// function PID
int pid_cal(int set_point, int input,int constrain_p,  double pP, double pI, double pD);
void rundc0(int v, int vec);
void rundc1(int v, int vec);
void rundc2(int v, int vec);
void rundc3(int v, int vec);
void move(int angle_rotare, int v_rotare, int angle_tt, int v_tt , int soft_start , int soft_stopp  );
void ps3_move();
// handle speed
int32_t get_encoder(int32_t * value,int32_t * level_count,TIM_HandleTypeDef * htim,int16_t encoder_limit);


#endif /* INC_DRIVER_H_ */
