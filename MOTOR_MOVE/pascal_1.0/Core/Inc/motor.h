/*
 * motor.h
 *
 *  Created on: Mar 13, 2026
 *      Author: Vedant
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "stm32g4xx_hal.h"

/* Direction Pins */

// Motor A (Left)
#define AIN1_PORT GPIOA
#define AIN1_PIN  GPIO_PIN_10

#define AIN2_PORT GPIOA
#define AIN2_PIN  GPIO_PIN_11

// Motor B (Right)
#define BIN1_PORT GPIOA
#define BIN1_PIN  GPIO_PIN_12

#define BIN2_PORT GPIOB
#define BIN2_PIN  GPIO_PIN_3

// Standby pin
#define STBY_PORT GPIOB
#define STBY_PIN  GPIO_PIN_4

// PWM Timer
extern TIM_HandleTypeDef htim1;


/* Function Prototypes */

void Motor_Init(void);

void Motor_SetLeftSpeed(int16_t speed);
void Motor_SetRightSpeed(int16_t speed);

void Motor_Forward(int16_t speed);
void Motor_Backward(int16_t speed);

void Motor_Left(int16_t speed);
void Motor_Right(int16_t speed);

void Motor_Stop(void);

#endif
