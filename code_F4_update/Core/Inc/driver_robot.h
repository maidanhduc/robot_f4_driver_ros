/*
 * driver_robot.h
 *
 *  Created on: Apr 24, 2025
 *      Author: APC
 */

#ifndef INC_DRIVER_ROBOT_H_
#define INC_DRIVER_ROBOT_H_

#include "main.h"
#include "stdbool.h"
#include "math.h"

extern TIM_HandleTypeDef htim5;

class Motor{
	private :
		TIM_HandleTypeDef *htim;  // Timer driver PWM
		uint32_t Channel;         // Channel PWM
		GPIO_TypeDef *IN1_Port;   // Drain GPIO of IN1, example GPIO_B,D
		uint16_t IN1;             // Drain IN1
		GPIO_TypeDef *IN2_Port;   // Drain GPIO of IN2
		uint16_t IN2;             // Drain IN2
	public :
		int16_t angle[2];

		Motor(TIM_HandleTypeDef *htim, uint32_t Channel, GPIO_TypeDef *IN1_Port, uint16_t IN1, GPIO_TypeDef *IN2_Port, uint16_t IN2,int16_t angle_val):
			htim(htim),Channel(Channel),IN1_Port(IN1_Port),IN1(IN1),IN2_Port(IN2_Port),IN2(IN2){
			angle[0] = angle_val;
			if(angle[0] > 0) angle[1] = angle[0] - 900;
			if(angle[0] < 0) angle[1] = angle[0] + 900;
		}
		void init_motor(){
			HAL_GPIO_WritePin(IN1_Port,IN1,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IN2_Port,IN2,GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(htim,Channel,0);
			HAL_TIM_PWM_Start(htim,Channel);
		}
		void goForward(uint32_t pwm){
			HAL_GPIO_WritePin(IN1_Port,IN1,GPIO_PIN_SET);
			HAL_GPIO_WritePin(IN2_Port,IN2,GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(htim,Channel,pwm);
		}
		void goReverse(uint32_t pwm){
			HAL_GPIO_WritePin(IN1_Port,IN1,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IN2_Port,IN2,GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(htim,Channel,pwm);
		}
		void stop(){
			HAL_GPIO_WritePin(IN1_Port,IN1,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IN2_Port,IN2,GPIO_PIN_RESET);
		}
};

extern Motor DC[4];
// cac ham co ban de test
void tien(uint16_t pwm );
void lui(uint16_t pwm );
void trai(uint16_t pwm );
void phai(uint16_t pwm );
void xoay_trai(uint16_t pwm);
void xoay_phai(uint16_t pwm);
//
void up(uint8_t v_rotate);
void down(uint8_t v_rotate);
void left(uint8_t v_rotate);
void right(uint8_t v_rotate);
int16_t constrains(int16_t value, int16_t down, int16_t up );
double fConstrain(double value, double down, double up);
void soft_speed(int16_t* variable,uint32_t * last_time, int16_t speed,uint32_t time_out,uint8_t delta_speed);
// function PID
void run_dc(uint8_t num_dc,int16_t speed);
void run_dc(uint8_t vec,uint8_t num_dc,uint8_t speed);
int pid_cal(int set_point, int input,int constrain_p,  double pP, double pI, double pD);
int16_t move(int16_t angle_set, int16_t speed_rot,int16_t angle_tt, int16_t speed_tt,uint32_t time_soft,uint8_t delta_speed);
int16_t move_direction(int16_t angle_set, int16_t speed_rot,int16_t angle_tt, int16_t speed_tt,uint32_t time_soft,uint8_t delta_speed);
void ps3_pid();
void ps3_nor();
void ps3_pid_double(int32_t pwm);
// handle speed
int32_t get_encoder(int32_t * value,int32_t * level_count,TIM_HandleTypeDef * htim,int16_t encoder_limit);

void test_DC();
void stop_all();

#endif /* INC_DRIVER_ROBOT_H_ */
