/*
 * driver_robot.cpp
 *
 *  Created on: Apr 24, 2025
 *      Author: APC
 */
#include "driver_robot.h"
#include "Uart.h"

#define SPEED_MAX 255
#define SPEED_MIN 0
#define DEG_TO_RAD 0.017453292519943295769236907684886

int pwm0, pwm1, pwm2, pwm3;
int debug0,debug1,debug2,debug3;
int bug0,bug1,bug2,bug3;
int real_v_rotate;
int db_agrot;
int16_t constrains(int16_t value, int16_t down, int16_t up ){
	if(value >= up) return up;
	if(value < down) return down;
	return value;
}
double fConstrain(double value, double down, double up){
	if(value >= up) return up;
	if(value < down) return down;
	return value;
}
uint16_t speed = 300;
// PID algorithm
double PI_const = 3.141592654;
int16_t angle_update = 0;
void soft_speed(int16_t* variable,uint32_t * last_time, int16_t speed,uint32_t time_out,uint8_t delta_speed){ // tốc độ và thời gian khởi động
  if(HAL_GetTick() - (*last_time) >= time_out) {
    if((*variable) < speed){
    	(*variable) += delta_speed;
    	if((*variable) > speed) *variable = speed;
    }else if((*variable) > speed){
    	(*variable) -= delta_speed;
		if((*variable) < speed) *variable = speed;
    }else{
    	(*variable) = speed;
    }

    (*last_time) = HAL_GetTick();
  }
}
int16_t get_angle(int16_t angle){ // resolution(độ phân giải) : -1800 -> 1800
  angle = angle % 3600;
  if(angle >= 1800) return angle -= 3600;
  if(angle < -1800) return angle += 3600;
  return angle;

}
int16_t PID_rotate(int16_t angle_set,int16_t speed_rot){ // góc muốn đến , giới hạn tốc độ
  float kp, ki, kd;
  static int16_t error  = 0,last_error = 0;
  int16_t output = 0;

    error = get_angle(angle_set) - get_angle(compass());
    //error = angle_set - compass();

//    if(abs(error) >= 1000){
        kp = 0.65, ki = 0.35, kd = 0.01;
      //kp = 0.025, ki = 0.01,  kd = 0.02;
//    } else
//    if(abs(error) >= 400){
//      kp = 0.2, ki = 0.1, kd = 0.08;
//    } else{
//        kp = 0.2, ki = 0.1,  kd = 0.04;
//
//    }

    output = kp * error + ki * (error + last_error) + kd * (error - last_error);
    output = constrains(output, -speed_rot, speed_rot);
    //Serial.println(output);
    last_error = error;

  return output;
}


void run_dc(uint8_t num_dc,int16_t speed){
  if(speed >= 0){
    DC[num_dc].goForward(speed);
  }else{
    DC[num_dc].goReverse(abs(speed));
  }
}
void run_dc(uint8_t vec,uint8_t num_dc,uint8_t speed){
	if(vec == 1) DC[num_dc].goForward(speed);
	if(vec == 0) DC[num_dc].goReverse(abs(speed));
}
/*
int16_t move(int16_t angle_set, int16_t speed_rot,int16_t angle_tt, int16_t speed_tt,uint32_t time_soft,uint8_t delta_speed){
  // góc muốn đầu robot hướng tới , tốc độ quay, góc tịnh tiến, tốc độ tịnh tiến, thời gian để tăng giảm 1 giá trị speed

  static int16_t speed_run = 80; // khởi tạo giá trị luôn mà không sử dụng hàm soft_speed để tăng dần
  //soft_speed(&speed_run,&time_soft_speed,speed_tt,time_soft,delta_speed);
  //            speed   ->                   speed hướng tới
  int16_t pwm[4];
  float alpha[4];

  int16_t speed_rotate = PID_rotate(angle_set,speed_rot); // xử lí PID góc

  int16_t angle_diff = get_angle(angle_tt - compass());
  static uint8_t left_front, left_back, right_front, right_back, mode;

  // xử lí góc tịnh tiến của robot

  if(angle_diff <= -450 && angle_diff > -1350 ){
    left_front  =   0;  right_front =  3;
    left_back   =   1;  right_back  =  2;
    angle_diff += 900;

  }else if( angle_diff <= 450 && angle_diff > -450){
    left_front  =   1;  right_front =  0;
    left_back   =   2;  right_back  =  3;
    mode = 0;
  }else if( angle_diff > 450  && angle_diff <= 1350){
    left_front  =   2;  right_front =  1;
    left_back   =   3;  right_back  =  0;
    angle_diff -= 900;
    mode = 1;
  }else if(angle_diff > 1350 || angle_diff <= -1350 ){
    left_front  =   3;  right_front =  2;
    left_back   =   0;  right_back  =  1;
    if(angle_diff > 1350) angle_diff -= 1800;
    if(angle_diff <= -1350) angle_diff += 1800;

  }

  alpha[right_front]  = cos((DC[right_front].angle[mode] - angle_diff) * DEG_TO_RAD / 10);
  alpha[right_back]   = cos((DC[right_back].angle[mode]  - angle_diff) * DEG_TO_RAD / 10);
  alpha[left_front]   = cos((DC[left_front].angle[mode] - angle_diff + 1800) * DEG_TO_RAD / 10);
  alpha[left_back]    = cos((DC[left_back].angle[mode]  - angle_diff + 1800) * DEG_TO_RAD / 10);

  const double theta[2] = {cos(abs(DC[right_front].angle[mode] - DC[right_back].angle[mode]) * DEG_TO_RAD /10),
                          cos(abs(DC[left_front].angle[mode] -  DC[left_back].angle[mode]) *  DEG_TO_RAD /10)};


  const double gama[2] = {(alpha[right_front] * alpha[right_front] + alpha[right_back] * alpha[right_back] - 2 * alpha[right_front] * alpha[right_back] * theta[0]),
                   (alpha[left_front] * alpha[left_front] + alpha[left_back] * alpha[left_back] - 2 * alpha[left_front] * alpha[left_back] * theta[1])};
  //right robot

  // xử lí tốc độ

  pwm[right_front] =  (speed_rotate + speed_run * sqrt(2)) * (alpha[right_front] - alpha[right_back]  * theta[0]) / gama[0];
  pwm[right_back]  =  (speed_rotate + speed_run * sqrt(2)) * (alpha[right_back] -  alpha[right_front] * theta[0]) / gama[0];


  // left robot
  pwm[left_front]  =  (-speed_rotate + speed_run * sqrt(2)) * (alpha[left_front] - alpha[left_back]  * theta[1]) / gama[1];
  pwm[left_back]   =  (-speed_rotate + speed_run * sqrt(2)) * (alpha[left_back]  - alpha[left_front] * theta[1]) / gama[1];

#if 1

  pwm0 = pwm[right_front];
  pwm1 = pwm[right_back];
  pwm2 = pwm[left_front];
  pwm3 = pwm[left_back];

#endif


  run_dc(right_front,pwm[right_front]);
  run_dc(right_back ,pwm[right_back] );
  run_dc(left_front ,pwm[left_front] );
  run_dc(left_back  ,pwm[left_back]  );



  return speed_run;

}
*/
int16_t move_direction(int16_t angle_set, int16_t speed_rot,int16_t angle_tt, int16_t speed_tt,uint32_t time_soft,uint8_t delta_speed){

  static int16_t pwm[4];
  static int16_t speed_run = 0; // 0-360
  static uint32_t time_soft_speed = 0;
  soft_speed(&speed_run,&time_soft_speed,speed_tt,time_soft,delta_speed);
//										gioi han  time nhay	buoc nhay
  static int16_t speed_rotate = 0;
  speed_rotate = PID_rotate(angle_set,speed_rot);
  int16_t angle = compass();

  float alpha[4];

  for(uint8_t i = 0; i < 4; i++)
    alpha[i] = cos((DC[i].angle[0] - angle_tt + angle) * DEG_TO_RAD / 10);

 // const double theta[2] = {cos(abs(DC[0].angle[0] - DC[3].angle[0]) * DEG_TO_RAD /10), cos(abs(DC[1].angle[0] - DC[2].angle[0]) * DEG_TO_RAD /10)};

//  double gama[2] = {(alpha[0] * alpha[0] + alpha[3] * alpha[3] - 2 * alpha[0] * alpha[3] * theta[0]),
//                       (alpha[1] * alpha[1] + alpha[2] * alpha[2] - 2 * alpha[1] * alpha[2] * theta[1])};
#if 0
  //right robot : DC[0], DC[3]

  pwm[0] = speed_rotate + (speed_run * sqrt(2)) * (alpha[0] - alpha[3] * theta[0]) / gama[0];
  pwm[3] = speed_rotate + (speed_run * sqrt(2)) * (alpha[3] - alpha[0] * theta[0]) / gama[0];


  // left robot : DC[1] DC[2]
  pwm[1] =  speed_rotate - (speed_run * sqrt(2)) * (alpha[1] - alpha[2] * theta[1]) / gama[1];
  pwm[2] =  speed_rotate - (speed_run * sqrt(2)) * (alpha[2] - alpha[1] * theta[1]) / gama[1];
#endif

#if 1
  //right robot : DC[0], DC[3]

  pwm[0] = speed_rotate + speed_run  * alpha[0]; // căn 2 hết
  pwm[3] = speed_rotate + speed_run  * alpha[3];


  // left robot : DC[1] DC[2]
  pwm[1] =  speed_rotate - speed_run *  alpha[1];
  pwm[2] =  speed_rotate - speed_run *  alpha[2];

#endif


  pwm[0] = constrains(pwm[0], -255, 255);
  pwm[1] = constrains(pwm[1], -255, 255);
  pwm[2] = constrains(pwm[2], -255, 255);
  pwm[3] = constrains(pwm[3], -255, 255);

  run_dc(0,pwm[0]);
  run_dc(1,pwm[1]);
  run_dc(2,pwm[2]);
  run_dc(3,pwm[3]);
#if 1 // for debug

  pwm0 = pwm[0];
  pwm1 = pwm[1];
  pwm2 = pwm[2];
  pwm3 = pwm[3];
  debug0 = speed_run  * alpha[0];
  debug1 = speed_run  * alpha[1];
  debug2 = speed_run  * alpha[2];
  debug3 = speed_run  * alpha[3];
  real_v_rotate = speed_rotate;

#endif

  //run_dc(0,pwm[0]);
  //Debug("pwm[0] = %d pwm[1] = %d  pwm[2] = %d  pwm[3] = %d",pwm[0],pwm[1],pwm[2],pwm[3] );

//  Send_speed_UART(&huart3, DC[0].address_DC, pwm[0], DC[1].address_DC, pwm[1], DC[2].address_DC, pwm[2], DC[3].address_DC, pwm[3]);

  return speed_run;

}


void test_DC(){
	for(int i =0;i<4;i++){
		DC[i].goForward(255);
		HAL_Delay(1000);
	}
}

void stop_all(){
	for (uint8_t i =0;i<4;i++){
		DC[i].stop();
	}
}
void ps3_pid(){
	get_uart();
	static int16_t ag_rot = 0;
	static int16_t ag_tt = 0;

	//move_direction(ag_rot,70,ag_tt,100,1,1);
	  if(but_data ==  button_up){
		ag_tt = ag_rot;
		move_direction(ag_rot,250,ag_tt,400,10,10); // dùng như hàm move
		  // góc muốn đầu robot hướng tới , tốc độ quay, góc tịnh tiến, tốc độ tịnh tiến, thời gian để tăng giảm 1 giá trị speed

	  }else if(but_data == button_down){
		ag_tt = ag_rot + 1800;
		move_direction(ag_rot,250,ag_tt,400,10,10);

	  }else if(but_data == button_left){
		ag_tt = ag_rot + 900;
		move_direction(ag_rot,250,ag_tt,400,10,10);

	  }else if(but_data == button_right){
		ag_tt = ag_rot - 900;
		move_direction(ag_rot,250,ag_tt,400,10,10);
	  }
	   else if (but_data == button_R2){
		ag_tt = ag_rot - 450;
		move_direction(ag_rot,250,ag_tt,400,10,10);
	   }
	   else if (but_data == button_L2){
		ag_tt = ag_rot + 450;
		move_direction(ag_rot,250,ag_tt,400,10,10);
	   }
	   else if (but_data == button_L1){
		xoay_phai(250);
		}
	   else if (but_data == button_R1){
		xoay_trai(250);
		}
	   else {
		pwm0=0;pwm1=0;pwm2=0;pwm3=0;
		real_v_rotate = 0;
		stop_all();
		ag_rot = compass();
	}
}
// test PWM
void tien(uint16_t pwm ){
	DC[0].goForward(pwm);
	DC[1].goReverse(pwm);
	DC[2].goReverse(pwm);
	DC[3].goForward(pwm);
}
void lui(uint16_t pwm ){
	DC[0].goReverse(pwm);
	DC[1].goForward(pwm);
	DC[2].goForward(pwm);
	DC[3].goReverse(pwm);
}
void trai(uint16_t pwm){
	DC[0].goReverse(pwm);
	DC[1].goReverse(pwm);
	DC[2].goForward(pwm);
	DC[3].goForward(pwm);
}
void phai(uint16_t pwm){
	DC[2].goReverse(pwm);
	DC[3].goReverse(pwm);
	DC[0].goForward(pwm);
	DC[1].goForward(pwm);
}
void xoay_phai(uint16_t pwm){
	DC[0].goForward(pwm);
	DC[1].goForward(pwm);
	DC[2].goForward(pwm);
	DC[3].goForward(pwm);

}
void xoay_trai(uint16_t pwm){
	DC[0].goReverse(pwm);
	DC[1].goReverse(pwm);
	DC[2].goReverse(pwm);
	DC[3].goReverse(pwm);
}

void ps3_nor(){
	if (but_data == button_up){
		tien(250);
	}
	else if (but_data == button_down){
		lui(250);
	}
	else if (but_data == button_right){
		trai(250);
	}
	else if (but_data == button_left){
		phai(250);
	}
	else if (but_data == button_R1){
		xoay_phai(250);
	}
	else if (but_data == button_L1){
		xoay_trai(250);
	}
	else {
		stop_all();
		//angle_rot = compass();
	}
}

void ps3_pid_double(int32_t pwm){
	//get_uart();
	static int16_t ag_rot = 0;
	static int16_t ag_tt = 0;
	static uint32_t time_delay =0;
	//move_direction(ag_rot,70,ag_tt,100,1,1);

		ag_tt = ag_rot;
		move_direction(ag_rot,250,ag_tt,pwm,10,10); // dùng như hàm move
		  // góc muốn đầu robot hướng tới , tốc độ quay, góc tịnh tiến, tốc độ tịnh tiến, thời gian để tăng giảm 1 giá trị speed


	  /*else if(but_data == button_left){
		ag_tt = ag_rot + 900;
		move_direction(ag_rot,250,ag_tt,pwm,10,10);

	  }
	  else if(but_data == button_right){
		ag_tt = ag_rot - 900;
		move_direction(ag_rot,250,ag_tt,400,10,10);
	  }
	   else if (but_data == button_R2){
		ag_tt = ag_rot - 450;
		move_direction(ag_rot,250,ag_tt,400,10,10);
	   }
	   else if (but_data == button_L2){
		ag_tt = ag_rot + 450;
		move_direction(ag_rot,250,ag_tt,400,10,10);
	   }
	   else if (but_data == button_L1){
		xoay_phai(250);
		}
	   else if (but_data == button_R1){
		xoay_trai(250);
		}
		*/
		if (HAL_GetTick() - time_delay  >= 10000){
		time_delay = HAL_GetTick();

		ag_rot = compass();
		}
}





