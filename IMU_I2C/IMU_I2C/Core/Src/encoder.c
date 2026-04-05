/*
 * encoder.c
 *
 * Created on: Mar 13, 2026
 * Author: Vedant
 */

#include "encoder.h"
#include "main.h"

/* ================================================================
   ROBOT GEOMETRY & ENCODER SPECS (N20 Motors)
   ================================================================ */
/* Calculate the exact pulses for ONE full rotation of the wheel. */
#define MOTOR_BASE_PPR  3.0f      // 2-pole magnetic disc
#define GEAR_RATIO      100.0f    // 100:1 gearbox
#define QUADRATURE_X4   4.0f      // STM32 counts all rising/falling edges on A and B

// Total pulses for exactly ONE spin of the wheel (800 pulses)
#define PULSES_PER_REV  (MOTOR_BASE_PPR * GEAR_RATIO * QUADRATURE_X4)

/* Physical wheel diameter */
#define WHEEL_DIAMETER  0.025f    // 28mm converted to meters
#define PI              3.14159265f

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

static int32_t left_total = 0;
static int32_t right_total = 0;

static uint16_t prev_left = 0;
static uint16_t prev_right = 0;

void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    Encoder_ResetBoth();
}

int32_t Encoder_GetLeft(void)
{
    return left_total;
}

int32_t Encoder_GetRight(void)
{
    return right_total;
}

float Encoder_GetDistanceLeft(void)
{
    // Math is cast to float to prevent integer truncation, then multiplied by 1000 for mm
    return (((float)left_total * PI * WHEEL_DIAMETER) / PULSES_PER_REV) * 1000.0f;
}

float Encoder_GetDistanceRight(void)
{
    // Math is cast to float to prevent integer truncation, then multiplied by 1000 for mm
    return (((float)right_total * PI * WHEEL_DIAMETER) / PULSES_PER_REV) * 1000.0f;
}

void Encoder_ResetLeft(void)
{
    /* Don't force hardware to 0, just sync the software variables! */
    left_total = 0;
    prev_left = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
}

void Encoder_ResetRight(void)
{
    /* Don't force hardware to 0, just sync the software variables! */
    right_total = 0;
    prev_right = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
}

void Encoder_ResetBoth(void)
{
    /* Don't force hardware to 0, just sync the software variables! */
    left_total = 0;
    right_total = 0;

    prev_left = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    prev_right = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
}

void Encoder_Update(void)
{
    /* Note: TIM2 is a 32-bit timer, TIM3 is a 16-bit timer on the G431.
       By reading both into uint16_t, we safely treat both as 16-bit timers.
       This is perfectly fine as long as this update function is called frequently.
    */
    uint16_t curr_left = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    uint16_t curr_right = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

    /* Signed 16-bit math automatically handles timer rollover! */
    int16_t diff_left = (int16_t)(curr_left - prev_left);
    int16_t diff_right = (int16_t)(curr_right - prev_right);

    left_total += diff_left;
    right_total += diff_right;

    prev_left = curr_left;
    prev_right = curr_right;
}
