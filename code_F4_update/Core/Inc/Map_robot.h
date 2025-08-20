/*
 * Map_robot.h
 *
 *  Created on: Apr 24, 2025
 *      Author: APC
 */

#ifndef INC_MAP_ROBOT_H_
#define INC_MAP_ROBOT_H_
#include <cstdint>
#include "main.h"
#include "math.h"

extern int count_flag;
extern int32_t cnt1,level_count1,speeden1,target_speed;
extern int32_t cnt2,level_count2,speeden2,really_speed;

int32_t get_encoder(int32_t * value,int32_t * level_count,TIM_HandleTypeDef * htim,int16_t encoder_limit);
int32_t get_speed(int32_t *cnt, int32_t *realspeed,int32_t * level_count,TIM_HandleTypeDef * htim,uint32_t period_ms, uint8_t id_encoder);
int32_t pid_speed(int32_t* real_speed, int32_t* target_speed,float kp, float ki, float kd);

/*
 * nhận vận tốc góc tử rap và pid chạy bám theo vận tốc đó
 * gửi dữ liệu qua uart cho rap
 */

#endif /* INC_MAP_ROBOT_H_ */
