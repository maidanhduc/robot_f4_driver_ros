#include "6dot_servo.h"

void analog_Write(TIM_HandleTypeDef *htim , uint32_t channel , uint16_t pwm) {
	HAL_TIM_PWM_Start(htim,channel);
	__HAL_TIM_SetCompare(htim,channel,pwm);
}

void setServoAngle(uint8_t angle, TIM_HandleTypeDef *htim, uint32_t channel )
{
    // Giới hạn góc từ 0 - 180 độ
    if (angle > 180) angle = 180;

    // Chuyển đổi góc thành giá trị PWM (500us - 2000us)
    uint32_t pulse = 500 + ((angle * 2000) / 180);
    __HAL_TIM_SET_COMPARE(htim, channel, pulse);
}
