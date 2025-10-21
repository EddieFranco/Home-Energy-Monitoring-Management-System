
#ifndef BTS7960_PWM_H
#define BTS7960_PWM_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * BTS7960 PWM driver for TIM3 CH3 (PB0) and CH4 (PB1).
 *
 * Requirements:
 *  - CubeMX must generate and initialize TIM3 for PWM on CH3 & CH4.
 *  - The global handle `htim3` must exist (extern here).
 *  - GPIO pins PB0 and PB1 must be configured as AF for TIM3_CH3/CH4.
 */

extern TIM_HandleTypeDef htim3;

/** Start PWM on TIM3 CH3 and CH4 and set both to 0% */
void BTS7960_PWM_Init(void);

/** Set duty (0..100 %) on CH3 (LPWM) */
void BTS7960_SetDuty_CH3(float duty_percent);

/** Set duty (0..100 %) on CH4 (RPWM) */
void BTS7960_SetDuty_CH4(float duty_percent);

/** Heater helper: one-direction heat (LPWM=duty, RPWM=0) */
static inline void BTS7960_SetHeaterPercent(float duty_percent) {
    BTS7960_SetDuty_CH3(duty_percent);
    BTS7960_SetDuty_CH4(0.0f);
}

/** Coast/Stop: both sides 0% */
static inline void BTS7960_Stop(void) {
    BTS7960_SetDuty_CH3(0.0f);
    BTS7960_SetDuty_CH4(0.0f);
}

/** Electronic brake: both sides 100% (optional use) */
static inline void BTS7960_Brake(void) {
    BTS7960_SetDuty_CH3(100.0f);
    BTS7960_SetDuty_CH4(100.0f);
}

float BTS7960_GetDuty_CH3(void);
float BTS7960_GetDuty_CH4(void);

#ifdef __cplusplus
}
#endif

#endif /* BTS7960_PWM_H */
