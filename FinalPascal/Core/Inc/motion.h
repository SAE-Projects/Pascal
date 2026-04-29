#ifndef MOTION_H
#define MOTION_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

/* --- HARVESTED HARDWARE MAP --- */
#define MOTOR_L_AIN1_PORT   GPIOA
#define MOTOR_L_AIN1_PIN    GPIO_PIN_10
#define MOTOR_L_AIN2_PORT   GPIOA
#define MOTOR_L_AIN2_PIN    GPIO_PIN_11
#define MOTOR_R_BIN1_PORT   GPIOA
#define MOTOR_R_BIN1_PIN    GPIO_PIN_12
#define MOTOR_R_BIN2_PORT   GPIOB
#define MOTOR_R_BIN2_PIN    GPIO_PIN_3
#define MOTOR_STBY_PORT     GPIOB
#define MOTOR_STBY_PIN      GPIO_PIN_4

/* --- HARVESTED GEOMETRY --- */
#define MOTOR_BASE_PPR      3.0f      // 2-pole magnetic disc
#define GEAR_RATIO          100.0f    // 100:1 gearbox
#define QUADRATURE_X4       4.0f
#define PULSES_PER_REV      (MOTOR_BASE_PPR * GEAR_RATIO * QUADRATURE_X4)
#define WHEEL_DIAMETER_MM   25.0f
#define PI                  3.14159265f
#define MM_PER_TICK         ((PI * WHEEL_DIAMETER_MM) / PULSES_PER_REV)

/* --- STATE MACHINE --- */
typedef enum {
    STATE_IDLE = 0,
    STATE_STRAIGHT,
    STATE_TURN_LEFT,
    STATE_TURN_RIGHT
} BotState_t;

/* --- SHARED BRAIN MEMORY --- */
typedef struct {
    // Current Sensor States
    float current_heading;      // Degrees
    float current_distance_mm;  // Total driven distance

    // Targets (Set by Floodfill)
    float target_heading;
    float target_distance_mm;
    int base_speed;             // 0 to 1000

    BotState_t state;
} Micromouse_t;

// --- EXPOSED GLOBALS ---
extern volatile Micromouse_t pascal;
extern float Kp_IR; // Added so you can tune IR Proportional from main.c
extern float Kd_IR; // Added so you can tune IR Derivative from main.c

// Core Functions
void Motion_Init(void);
void Update_Encoders(void);
void Run_Motion_Control_Loop(void);

#endif
