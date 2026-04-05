/*
 * encoder.c
 *
 *  Created on: Mar 13, 2026
 *      Author: Vedant
 */


#include "encoder.h"
#include "main.h"

#define PULSES_PER_REV 3   // change according to your encoder
#define WHEEL_DIAMETER 0.044  // meters (example: 44mm)

#define PI 3.1415926


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

    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
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
    return (left_total * PI * WHEEL_DIAMETER) / PULSES_PER_REV;
}

float Encoder_GetDistanceRight(void)
{
    return (right_total * PI * WHEEL_DIAMETER) / PULSES_PER_REV;
}

void Encoder_ResetLeft(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
}

void Encoder_ResetRight(void)
{
    __HAL_TIM_SET_COUNTER(&htim3, 0);
}

void Encoder_ResetBoth(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    left_total = 0;
    right_total = 0;

    prev_left = 0;
    prev_right = 0;
}

void Encoder_Update(void)
{
    uint16_t curr_left = __HAL_TIM_GET_COUNTER(&htim2);
    uint16_t curr_right = __HAL_TIM_GET_COUNTER(&htim3);

    int16_t diff_left = curr_left - prev_left;
    int16_t diff_right = curr_right - prev_right;

    left_total += diff_left;
    right_total += diff_right;

    prev_left = curr_left;
    prev_right = curr_right;
}
