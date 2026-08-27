#include "buzzer.h"
#include "main.h"

extern TIM_HandleTypeDef htim12;

void Init_buzzer()
{
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
}

void Start_buzzer()
{
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 100);
}

void End_buzzer()
{
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
}