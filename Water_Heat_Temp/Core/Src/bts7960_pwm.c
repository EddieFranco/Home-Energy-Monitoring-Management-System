
#include "bts7960_pwm.h"

/* TIM3 is created by CubeMX */
extern TIM_HandleTypeDef htim3;

/* Clamp helper */
static inline float clampf(float x, float a, float b) {
    if (x < a) return a;
    if (x > b) return b;
    return x;
}

void BTS7960_PWM_Init(void)
{
    /* Start the PWM outputs that CubeMX configured */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);  // PB0 = TIM3_CH3
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);  // PB1 = TIM3_CH4

    /* Ensure known state */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
}

void BTS7960_SetDuty_CH3(float duty_percent)
{
    duty_percent = clampf(duty_percent, 0.0f, 100.0f);

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);   // e.g., 8399
    uint32_t ccr = (uint32_t)((duty_percent * (arr + 1U)) / 100.0f);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ccr);
}

void BTS7960_SetDuty_CH4(float duty_percent)
{
    duty_percent = clampf(duty_percent, 0.0f, 100.0f);

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
    uint32_t ccr = (uint32_t)((duty_percent * (arr + 1U)) / 100.0f);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ccr);
}


float BTS7960_GetDuty_CH3(void)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
    uint32_t ccr = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_3);

    if (arr == 0) return 0.0f;
    return ((float)ccr / (float)(arr + 1U)) * 100.0f; // percent
}

float BTS7960_GetDuty_CH4(void)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
    uint32_t ccr = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_4);

    if (arr == 0) return 0.0f;
    return ((float)ccr / (float)(arr + 1U)) * 100.0f; // percent
}
