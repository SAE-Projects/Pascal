#ifndef MOTOR_H
#define MOTOR_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
   HARDWARE MAP — TB6612FNG on STM32G431CBU6
   ================================================================
   LEFT  motor  → PWMA = TIM1 CH1
                  AIN1 = PA10
                  AIN2 = PA11

   RIGHT motor  → PWMB = TIM1 CH2
                  BIN1 = PA12
                  BIN2 = PB3

   STBY         → must be HIGH to enable driver
                  set your STBY pin in Motor_Init()

   TIM1 ARR = 999  → PWM range 0–999
   PWM 0   = stop
   PWM 999 = full speed
   ================================================================ */

/* PWM limits — tune these for your motors */
#define MOTOR_PWM_MAX       999u    /* TIM1 ARR value               */
#define MOTOR_PWM_MIN       200u    /* minimum PWM to overcome stall */
#define MOTOR_PWM_TURN_MAX  700u    /* max PWM during a turn        */
#define MOTOR_PWM_TURN_MIN  150u    /* min PWM during a turn        */

/* ================================================================
   PID GAINS — tune these on your actual robot
   Start with only Kp, set Ki and Kd to 0.
   Increase Kp until turn is fast but overshoots.
   Add Kd to reduce overshoot.
   Add tiny Ki only if there is steady-state error.
   ================================================================ */
#define PID_KP   4.5f    /* proportional — main turning force  */
#define PID_KI   0.0f    /* integral     — corrects slow drift */
#define PID_KD   0.8f    /* derivative   — reduces overshoot   */

/* Turn done when error is within this many degrees */
#define TURN_TOLERANCE_DEG  2.0f

/* ================================================================
   DIRECTION PINS — GPIO
   ================================================================ */

/* LEFT motor direction (AIN1, AIN2 on PA10, PA11) */
#define MOTOR_L_AIN1_PORT   GPIOA
#define MOTOR_L_AIN1_PIN    GPIO_PIN_10
#define MOTOR_L_AIN2_PORT   GPIOA
#define MOTOR_L_AIN2_PIN    GPIO_PIN_11

/* RIGHT motor direction (BIN1, BIN2 on PA12, PB3) */
#define MOTOR_R_BIN1_PORT   GPIOA
#define MOTOR_R_BIN1_PIN    GPIO_PIN_12
#define MOTOR_R_BIN2_PORT   GPIOB
#define MOTOR_R_BIN2_PIN    GPIO_PIN_3

/* ================================================================
   DATA STRUCTURES
   ================================================================ */

/* PID state — one instance used for turn control */
typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float output;
} PID_t;

/* Turn direction */
typedef enum {
    TURN_RIGHT = 1,
    TURN_LEFT  = -1
} TurnDir_t;

/* ================================================================
   FUNCTION PROTOTYPES
   ================================================================ */

/* Initialisation */
void Motor_Init(TIM_HandleTypeDef *htim);

/* Raw motor control */
void Motor_Left   (int32_t speed);  /* -999 to +999, negative = reverse  */
void Motor_Right  (int32_t speed);  /* -999 to +999, negative = reverse  */
void Motor_Stop   (void);
void Motor_Forward(uint32_t time_ms, uint32_t speed); /* both fwd, timed */
void Motor_Backward(uint32_t time_ms, uint32_t speed);/* both rev, timed */

/* PID turn — blocks until turn is complete */
/* target_deg: degrees to turn (positive = right, negative = left) */
void Motor_TurnPID(float target_deg, float *heading_deg_ptr,
                   void (*imu_read_fn)(void),
                   void (*imu_update_fn)(void));

/* PID utility */
void  PID_Init  (PID_t *pid, float kp, float ki, float kd);
float PID_Update(PID_t *pid, float error, float dt);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
