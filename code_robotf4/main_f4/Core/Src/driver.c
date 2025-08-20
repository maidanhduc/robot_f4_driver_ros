#include "driver.h"
#include "Uart.h"
#include <stdlib.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
//extern TIM_HandleTypeDef htim8;
//extern TIM_HandleTypeDef htim9;

#define SPEED_MAX 255
#define SPEED_MIN 0

//========================VARIABLE============================//
Motor_t motor0 = {&htim5, TIM_CHANNEL_1,GPIOB,GPIO_PIN_13,GPIOB,GPIO_PIN_12};
Motor_t motor1 = {&htim5, TIM_CHANNEL_2,GPIOB,GPIO_PIN_14,GPIOB,GPIO_PIN_15};
Motor_t motor2 = {&htim5, TIM_CHANNEL_3,GPIOD,GPIO_PIN_9,GPIOD,GPIO_PIN_8 };
Motor_t motor3 = {&htim5, TIM_CHANNEL_4,GPIOD,GPIO_PIN_10,GPIOD,GPIO_PIN_11};
uint32_t v=500;
uint16_t last_cnt[4] = {0};
uint32_t pwm =999;
uint16_t pwms = 500;
uint32_t speed_encoder[4] ;
volatile int32_t encoder_count[4] = {0};  // Lưu giá trị encoder thực tế
volatile int32_t overflow_count[4] = {-1,-1,-1,-1}; // Đếm số lần tràn (Underflow/Overflow)
// ===================== STRUCT FUNCTION=====================//
void Motor_Init(Motor_t *motor) {
    HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->Channel, 0); // set PWM = 0
    HAL_TIM_PWM_Start(motor->htim, motor->Channel); // B?t d?u PWM
}
void Motor_SetSpeed(Motor_t *motor, int16_t speed) {
    if (speed > 0) {
        HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2, GPIO_PIN_RESET);
    } else if (speed < 0) {
        HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2, GPIO_PIN_SET);
        speed = -speed; // transmit positive value
    } else {
        HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2, GPIO_PIN_RESET);
    }

    if (speed > 1000) speed = 1000; // speed limit
    __HAL_TIM_SET_COMPARE(motor->htim, motor->Channel, speed);
}



int32_t get_encoder(int32_t * value,int32_t * level_count,TIM_HandleTypeDef * htim,int16_t encoder_limit){
	int16_t count_current = __HAL_TIM_GET_COUNTER(htim);

	if((*value) > encoder_limit || (*value) < - encoder_limit){
		(*level_count) += count_current;
		__HAL_TIM_SET_COUNTER(htim,0);
		count_current = 0;
	}

	(*value) = (*level_count) + count_current;

	return (*value);
}
// "pid_controller"
int32_t constrains(int32_t value, int32_t down, int32_t up ){
	if(value >= up) return up;
	if(value < down) return down;
	return value;
}

double fConstrain(double value, double down, double up){
	if(value >= up) return up;
	if(value < down) return down;
	return value;
}
uint16_t speed = 0;
void Soft_start(uint16_t value,uint32_t cycle){ // chạy nhanh dần
	static uint32_t time = 0;
	if(speed < value){
		if(HAL_GetTick() - time >= cycle){
			speed++;
			time = HAL_GetTick();
		}
	}
}

void Soft_end(uint16_t value , uint32_t cycle){ // chạy chậm dần
	static uint32_t time = 0;
	if(speed > value){
		if(HAL_GetTick() - time >= cycle){
			speed--;
			time = HAL_GetTick();
		}
	}
}
// PID algorithm
uint16_t v_high,v_low;
double PI_const = 3.141592654;
int16_t angle_update = 0;
int pid_cal(int set_point, int input,int constrain_p,  double pP, double pI, double pD){   // goc quay, dau vao, toc do quay , 3 he so
  int output = 0;
  static long error_p, last_error_p;
  error_p = set_point - input;
  output = pP*error_p + pI*(error_p +  last_error_p) + pD*(error_p - last_error_p); // tỉ lệ ; tích phân ;  đạo hàm
  last_error_p = error_p;
  output = constrains(output, - constrain_p, constrain_p);
  return output;
}

void rundc0(int v, int vec){ v = abs(v);   if(vec == 1 ){ Motor_SetSpeed(&motor0,v) ;  }
	else if(vec == 0){  Motor_SetSpeed(&motor0,-v);}}
void rundc1(int v, int vec){ v = abs(v);   if(vec == 1 ){ Motor_SetSpeed(&motor1,v) ;  }
	else if(vec == 0){  Motor_SetSpeed(&motor1,-v);}}
void rundc2(int v, int vec){ v = abs(v);   if(vec == 1 ){ Motor_SetSpeed(&motor2,v) ;  }
	else if(vec == 0){  Motor_SetSpeed(&motor2,-v);}}
void rundc3(int v, int vec){ v = abs(v);   if(vec == 1 ){ Motor_SetSpeed(&motor3,v) ;  }
	else if(vec == 0){  Motor_SetSpeed(&motor3,-v);}}

void move(int angle_rotare, int v_rotare, int angle_tt, int v_tt , int soft_start , int soft_stopp  ){   // angle tt là góc * 10
  int angle_compass = compass();
  int real_v_rotare = pid_cal(angle_rotare/10,angle_compass/10, v_rotare, 0.5, 0.01, 0.04);
  int pwm0, pwm1, pwm2, pwm3;
  int vec0, vec1, vec2, vec3;
  pwm0 = real_v_rotare + v_tt*cos((angle_tt - angle_compass - 450)*(PI_const/1800));
  pwm1 = real_v_rotare + v_tt*cos((angle_tt - angle_compass - 1350)*(PI_const/1800));
  pwm2 = real_v_rotare + v_tt*cos((angle_tt - angle_compass - 2250)*(PI_const/1800));
  pwm3 = real_v_rotare + v_tt*cos((angle_tt - angle_compass - 3150)*(PI_const/1800));

  if(pwm0 >  0 ){ vec0 = 1; }else { vec0 = 0 ; }
  if(pwm1 >  0 ){ vec1 = 1; }else { vec1 = 0 ; }
  if(pwm2 >  0 ){ vec2 = 1; }else { vec2 = 0 ; }
  if(pwm3 >  0 ){ vec3 = 1; }else { vec3 = 0 ; }

  rundc0(pwm0, vec0);
  rundc1(pwm1, vec1);
  rundc2(pwm2, vec2);
  rundc3(pwm3, vec3);

}
void ps3_move(){

}
// basic move
void tien(uint16_t pwm ){
		Motor_SetSpeed(&motor0,pwm); // similar function analogWrite( );
		Motor_SetSpeed(&motor1,pwm);
		Motor_SetSpeed(&motor2,pwm);
		Motor_SetSpeed(&motor3,pwm);
}
void lui(uint16_t pwm){
		Motor_SetSpeed(&motor0,-pwm);
		Motor_SetSpeed(&motor1,-pwm);
		Motor_SetSpeed(&motor2,-pwm);
		Motor_SetSpeed(&motor3,-pwm);
}
void trai(uint16_t pwm){
		Motor_SetSpeed(&motor0,pwm);
		Motor_SetSpeed(&motor1,-pwm);
		Motor_SetSpeed(&motor2,-pwm);
		Motor_SetSpeed(&motor3,pwm);
}
void phai(uint16_t pwm){
		Motor_SetSpeed(&motor0,-pwm);
		Motor_SetSpeed(&motor1,pwm);
		Motor_SetSpeed(&motor2,pwm);
		Motor_SetSpeed(&motor3,-pwm);
}
void xoayphai(uint16_t pwm){
		Motor_SetSpeed(&motor0,-pwm);
		Motor_SetSpeed(&motor1,pwm);
		Motor_SetSpeed(&motor2,-pwm);
		Motor_SetSpeed(&motor3,pwm);
}
void xoaytrai(uint16_t pwm){
		Motor_SetSpeed(&motor0,pwm);
		Motor_SetSpeed(&motor1,-pwm);
		Motor_SetSpeed(&motor2,pwm);
		Motor_SetSpeed(&motor3,-pwm);
}
void ps3_ctrl(){
	get_uart();
	if (but_data == 1) {
		tien(v);
		//setServoAngle(0,&htim9,TIM_CHANNEL_2);
	}
	else if (but_data == 2){
		lui(v);
		//setServoAngle(90,&htim9,TIM_CHANNEL_2);
	}
	else if (but_data == 8){
		trai(v);
		//setServoAngle(180,&htim9,TIM_CHANNEL_2);
	}
	else if (but_data == 4){
		phai(v);
		//setServoAngle(45,&htim9,TIM_CHANNEL_2);
	}
	else if (but_data == 64){
		xoaytrai(v);
	}
	else if (but_data == 128){
		xoayphai(v);
	}
	else {
		tien(0);
	}
}



