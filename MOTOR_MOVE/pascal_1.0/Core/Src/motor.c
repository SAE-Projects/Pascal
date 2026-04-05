/*
 * motor.c
 *
 *  Created on: Mar 13, 2026
 *      Author: Vedant
 */

#include "motor.h"


void Motor_Init(void)
{
    // Start PWM channels
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // PWMA
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // PWMB

    // Enable motor driver
    HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_SET);
}


/* Left Motor Control (Motor A) */
void Motor_SetLeftSpeed(int16_t speed)
{
    if(speed >= 0)
    {
        HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
    }
    else
    {
        HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_SET);

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -speed);
    }
}


/* Right Motor Control (Motor B) */
void Motor_SetRightSpeed(int16_t speed)
{
    if(speed >= 0)
    {
        HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_RESET);

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
    }
    else
    {
        HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_SET);

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -speed);
    }
}


/* Move Forward */
void Motor_Forward(int16_t speed)
{
    Motor_SetLeftSpeed(speed);
    Motor_SetRightSpeed(speed);
}


/* Move Backward */
void Motor_Backward(int16_t speed)
{
    Motor_SetLeftSpeed(-speed);
    Motor_SetRightSpeed(-speed);
}


/* Turn Left */
void Motor_Left(int16_t speed)
{
    Motor_SetLeftSpeed(-speed);
    Motor_SetRightSpeed(speed);
}


/* Turn Right */
void Motor_Right(int16_t speed)
{
    Motor_SetLeftSpeed(speed);
    Motor_SetRightSpeed(-speed);
}


/* Stop Motors */
void Motor_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
}
