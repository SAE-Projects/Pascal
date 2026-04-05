#ifndef MOTOR_H
#define MOTOR_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
   ROBOT GEOMETRY — 33mm tyre diameter, 50mm axle width
   ================================================================ */
#define TYRE_DIAMETER_MM    25.0f
#define AXLE_WIDTH_MM       50.0f
#define TYRE_CIRC_MM        (3.14159f * TYRE_DIAMETER_MM)
#define TURN_ARC_MM(deg)    (3.14159f * AXLE_WIDTH_MM * (deg) / 360.0f)

/* ================================================================
   HARDWARE MAP — TB6612FNG on STM32G431CBU6
   ================================================================ */
#define MOTOR_L_AIN1_PORT   GPIOA
#define MOTOR_L_AIN1_PIN    GPIO_PIN_10
#define MOTOR_L_AIN2_PORT   GPIOA
#define MOTOR_L_AIN2_PIN    GPIO_PIN_11
#define MOTOR_R_BIN1_PORT   GPIOA
#define MOTOR_R_BIN1_PIN    GPIO_PIN_12
#define MOTOR_R_BIN2_PORT   GPIOB
#define MOTOR_R_BIN2_PIN    GPIO_PIN_3

/* ================================================================
   PWM LIMITS & TURN SETTINGS
   ================================================================ */
#define MOTOR_PWM_MAX       1000u

#define TURN_SPEED      600u
#define TARGET_45R_DEG  45.755f
#define TARGET_45L_DEG -45.40f
#define TARGET_90R_DEG  90.55f
#define TARGET_90L_DEG -90.55f
#define TARGET_180R_DEG  180.0f
#define TARGET_180L_DEG -180.5f
#define TARGET_360R_DEG  360.0f
#define TARGET_360L_DEG -360.0f
#define TURN_TIMEOUT_MS  4000u

/* ================================================================
   FUNCTION PROTOTYPES
   ================================================================ */

void Motor_Init    (TIM_HandleTypeDef *htim);
void Motor_Left    (int32_t speed);
void Motor_Right   (int32_t speed);
void Motor_Stop    (void);
void Motor_Forward (uint32_t time_ms, uint32_t speed);
void Motor_Backward(uint32_t time_ms, uint32_t speed);

void Motor_DriveStraight_HDG(uint32_t time_ms,
                             uint32_t base_speed,
                             float *heading_deg_ptr,
                             void (*imu_read_fn)(void),
                             void (*imu_update_fn)(void));

void Motor_Turn_HDG(float target_deg,
                    float stop_threshold_deg,
                    float *heading_deg_ptr,
                    void (*imu_read_fn)(void),
                    void (*imu_update_fn)(void));
void Motor_Turn45R_HDG(float *heading_deg_ptr, void (*imu_read_fn)(void), void (*imu_update_fn)(void));
void Motor_Turn45L_HDG(float *heading_deg_ptr, void (*imu_read_fn)(void), void (*imu_update_fn)(void));
void Motor_Turn90R_HDG(float *heading_deg_ptr, void (*imu_read_fn)(void), void (*imu_update_fn)(void));
void Motor_Turn90L_HDG(float *heading_deg_ptr, void (*imu_read_fn)(void), void (*imu_update_fn)(void));
void Motor_Turn180R_HDG(float *heading_deg_ptr, void (*imu_read_fn)(void), void (*imu_update_fn)(void));
void Motor_Turn180L_HDG(float *heading_deg_ptr, void (*imu_read_fn)(void), void (*imu_update_fn)(void));
void Motor_Turn360R_HDG(float *heading_deg_ptr, void (*imu_read_fn)(void), void (*imu_update_fn)(void));
void Motor_Turn360L_HDG(float *heading_deg_ptr, void (*imu_read_fn)(void), void (*imu_update_fn)(void));

void Motor_DriveDistance_HDG(float target_distance_mm,
                             uint32_t max_speed,
                             float *heading_deg_ptr,
                             void (*imu_read_fn)(void),
                             void (*imu_update_fn)(void));

/* Smooth Arc Turn Prototypes */
void Motor_SetArcLeft(int32_t base_speed, float turn_ratio);
void Motor_SetArcRight(int32_t base_speed, float turn_ratio);
void Motor_ArcTurn_HDG(float target_angle_deg,
                       int32_t base_speed,
                       float turn_ratio,
                       uint8_t is_left,
                       uint8_t stop_at_end,
                       float *heading_deg_ptr,
                       void (*imu_read_fn)(void),
                       void (*imu_update_fn)(void));

/* Continuous Curve Turn Prototypes */
void Motor_Curve90R_Continuous(int32_t base_speed,
                               float turn_ratio,
                               float *heading_deg_ptr,
                               void (*imu_read_fn)(void),
                               void (*imu_update_fn)(void));

void Motor_Curve90L_Continuous(int32_t base_speed,
                               float turn_ratio,
                               float *heading_deg_ptr,
                               void (*imu_read_fn)(void),
                               void (*imu_update_fn)(void));

void Motor_Curve45R_Continuous(int32_t base_speed,
                               float turn_ratio,
                               float *heading_deg_ptr,
                               void (*imu_read_fn)(void),
                               void (*imu_update_fn)(void));

void Motor_Curve45L_Continuous(int32_t base_speed,
                               float turn_ratio,
                               float *heading_deg_ptr,
                               void (*imu_read_fn)(void),
                               void (*imu_update_fn)(void));
#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
