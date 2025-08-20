/*
 * Map_robot.cpp
 *
 *  Created on: Apr 24, 2025
 *      Author: APC
 */
#include "Map_robot.h"
#include "driver_robot.h"

int32_t cnt1=0;
int32_t level_count1=0;
int32_t speeden1 =0;
int32_t cnt2=0;
int32_t level_count2=0;
int32_t speeden2 =0;

int32_t target_speed =0;
int32_t really_speed =0;

int32_t get_encoder(int32_t* value, int32_t* level_count, TIM_HandleTypeDef* htim, int16_t encoder_limit) {
    if (value == NULL || level_count == NULL || htim == NULL) {
        return 0; // hoặc xử lý lỗi phù hợp
    }  // không cần thiết

    int32_t count_current = (int32_t)__HAL_TIM_GET_COUNTER(htim);
    int32_t new_value = *level_count + count_current;

    if (new_value > encoder_limit || new_value < -encoder_limit) {
        *level_count += count_current;
        __HAL_TIM_SET_COUNTER(htim, 0);
        // count_current = 0; // Không cần thiết vì đã tính new_value trước đó
    }

    *value = *level_count + (int32_t)__HAL_TIM_GET_COUNTER(htim);
    return *value;
}
int32_t get_speed(int32_t *cnt, int32_t *realspeed, int32_t *level_count, TIM_HandleTypeDef *htim, uint32_t period_ms, uint8_t id_encoder) {
    if (!cnt || !realspeed || !level_count || !htim) {
        return 0; // Hoặc xử lý lỗi phù hợp
    }

    static uint32_t time_last[2] = {0};
    uint32_t current_tick = HAL_GetTick();
    // Xử lý tràn tick (tick overflow)
    if (current_tick - time_last[id_encoder] >= period_ms || time_last[id_encoder] > current_tick) {
        // Tính tốc độ (tránh chia nguyên nếu cần độ chính xác)
        *realspeed = (*cnt * 60) / 22500; // Hoặc nhân trước rồi chia sau

        // Reset các giá trị
        time_last[id_encoder] = current_tick;
        __HAL_TIM_SET_COUNTER(htim, 0);
        *cnt = 0;
        *level_count = 0; // Cân nhắc có nên reset level_count không?
    }

    return (*realspeed);
}

int32_t last_error = 0;
float integral = 0,derivative=0;
uint32_t last_time = 0;
int32_t error=0;
float i_term;
float p_term;
float d_term,output;
float dt;
int32_t pid_speed(int32_t* real_speed, int32_t* target_speed, float kp, float ki, float kd) {
//    static int32_t last_error = 0;
//    static float integral = 0;
//    static uint32_t last_time = 0;

    // Tính thời gian delta (dt) tự động
    uint32_t current_time = HAL_GetTick();
     dt = (current_time - last_time) / 1000.0f; // Chuyển sang giây
    if (dt <= 0) dt = 0.01f; // Tránh chia 0 (default: 10ms)

    // Tính sai số
     error = *target_speed - *real_speed;

//    if (error > 300 ){
//    	kp = 1; ki = 0.05; kd = 0.01;
//    }

    // Khâu P
     p_term = kp * error;

    // Khâu I (có anti-windup)
    integral += error * dt;
    integral = constrains(integral, -255.0f, 255.0f); // Giới hạn integral
    i_term = ki * integral;

    // Khâu D
    derivative = (error - last_error) / dt;
     d_term = kd * derivative;

    // Tổng hợp PID
     output = p_term + i_term + d_term;
    output = constrains(output, -360.0f, 360.0f); // Giới hạn đầu ra

    // Cập nhật giá trị trước đó
    last_error = error;
    last_time = current_time;

    return (int32_t)output;
}


