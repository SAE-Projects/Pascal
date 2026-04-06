#include "motor.h"
#include <math.h>

/* Internal handle — set by Motor_Init() */
static TIM_HandleTypeDef *_htim = NULL;

/* ================================================================
   MOTOR_INIT
   Starts TIM1 CH1 and CH2 PWM.
   Sets STBY pin HIGH to enable TB6612FNG.
   Call once at startup before using any motor function.
   ================================================================ */
void Motor_Init(TIM_HandleTypeDef *htim)
{
    _htim = htim;

    /* Start PWM on both channels */
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);   /* LEFT  motor PWMA */
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);   /* RIGHT motor PWMB */

    /* Set both motors to stop */
    Motor_Stop();

    /*
     * STBY PIN — you must set your STBY GPIO HIGH here.
     * Find which pin you connected STBY to and add:
     *
     *   HAL_GPIO_WritePin(GPIOX, GPIO_PIN_X, GPIO_PIN_SET);
     *
     * If STBY is tied directly to 3.3V on your board, skip this.
     * If STBY is LOW the motors will not move at all.
     */
    /* HAL_GPIO_WritePin(GPIOA, GPIO_PIN_X, GPIO_PIN_SET); */
}

/* ================================================================
   MOTOR_LEFT
   Controls left motor speed and direction.
   speed: -999 (full reverse) to +999 (full forward)
   ================================================================
   TB6612 truth table:
     AIN1=1 AIN2=0 → forward
     AIN1=0 AIN2=1 → reverse
     AIN1=0 AIN2=0 → coast (free spin)
     AIN1=1 AIN2=1 → brake
   ================================================================ */
void Motor_Left(int32_t speed)
{
    /* Clamp to valid range */
    if (speed >  (int32_t)MOTOR_PWM_MAX) { speed =  (int32_t)MOTOR_PWM_MAX; }
    if (speed < -(int32_t)MOTOR_PWM_MAX) { speed = -(int32_t)MOTOR_PWM_MAX; }

    if (speed >= 0)
    {
        /* Forward */
        HAL_GPIO_WritePin(MOTOR_L_AIN1_PORT, MOTOR_L_AIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_L_AIN2_PORT, MOTOR_L_AIN2_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_1, (uint32_t)speed);
    }
    else
    {
        /* Reverse */
        HAL_GPIO_WritePin(MOTOR_L_AIN1_PORT, MOTOR_L_AIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_L_AIN2_PORT, MOTOR_L_AIN2_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_1, (uint32_t)(-speed));
    }
}

/* ================================================================
   MOTOR_RIGHT
   Controls right motor speed and direction.
   speed: -999 (full reverse) to +999 (full forward)
   ================================================================ */
void Motor_Right(int32_t speed)
{
    if (speed >  (int32_t)MOTOR_PWM_MAX) { speed =  (int32_t)MOTOR_PWM_MAX; }
    if (speed < -(int32_t)MOTOR_PWM_MAX) { speed = -(int32_t)MOTOR_PWM_MAX; }

    if (speed >= 0)
    {
        /* Forward */
        HAL_GPIO_WritePin(MOTOR_R_BIN1_PORT, MOTOR_R_BIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_R_BIN2_PORT, MOTOR_R_BIN2_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_2, (uint32_t)speed);
    }
    else
    {
        /* Reverse */
        HAL_GPIO_WritePin(MOTOR_R_BIN1_PORT, MOTOR_R_BIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_R_BIN2_PORT, MOTOR_R_BIN2_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_2, (uint32_t)(-speed));
    }
}

/* ================================================================
   MOTOR_STOP
   Hard brake both motors (AIN1=AIN2=1 → TB6612 brake mode).
   Much faster stop than coast. Use this at end of every turn.
   ================================================================ */
void Motor_Stop(void)
{
    /* Brake left */
    HAL_GPIO_WritePin(MOTOR_L_AIN1_PORT, MOTOR_L_AIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_L_AIN2_PORT, MOTOR_L_AIN2_PIN, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_1, 0u);

    /* Brake right */
    HAL_GPIO_WritePin(MOTOR_R_BIN1_PORT, MOTOR_R_BIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_R_BIN2_PORT, MOTOR_R_BIN2_PIN, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_2, 0u);
}

/* ================================================================
   PID_INIT
   Initialise PID state. Call once before Motor_TurnPID().
   ================================================================ */
void PID_Init(PID_t *pid, float kp, float ki, float kd)
{
    pid->kp         = kp;
    pid->ki         = ki;
    pid->kd         = kd;
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->output     = 0.0f;
}

/* ================================================================
   PID_UPDATE
   Computes PID output for one control cycle.
   error: target - current  (degrees)
   dt:    time since last call (seconds)
   returns: motor speed command (positive or negative)
   ================================================================ */
float PID_Update(PID_t *pid, float error, float dt)
{
    /* Proportional */
    float p = pid->kp * error;

    /* Integral — accumulate error over time */
    pid->integral += error * dt;

    /* Clamp integral to prevent windup */
    if      (pid->integral >  200.0f) { pid->integral =  200.0f; }
    else if (pid->integral < -200.0f) { pid->integral = -200.0f; }

    float i = pid->ki * pid->integral;

    /* Derivative — rate of error change */
    float d = 0.0f;
    if (dt > 0.0f)
    {
        d = pid->kd * (error - pid->prev_error) / dt;
    }
    pid->prev_error = error;

    pid->output = p + i + d;
    return pid->output;
}

/* ================================================================
   MOTOR_FORWARD
   Drives both motors forward at the same speed for time_ms ms,
   then hard brakes.
   speed: 0–999 (PWM value)
   time_ms: how long to drive — tune this to match one maze cell.
   ================================================================ */
void Motor_Forward(uint32_t time_ms, uint32_t speed)
{
    if (speed > MOTOR_PWM_MAX) { speed = MOTOR_PWM_MAX; }

    /* Both forward */
    HAL_GPIO_WritePin(MOTOR_L_AIN1_PORT, MOTOR_L_AIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_L_AIN2_PORT, MOTOR_L_AIN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_BIN1_PORT, MOTOR_R_BIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_R_BIN2_PORT, MOTOR_R_BIN2_PIN, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_1, speed);
    __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_2, speed);

    HAL_Delay(time_ms);
    Motor_Stop();
}

/* ================================================================
   MOTOR_BACKWARD
   Drives both motors in reverse for time_ms ms, then brakes.
   ================================================================ */
void Motor_Backward(uint32_t time_ms, uint32_t speed)
{
    if (speed > MOTOR_PWM_MAX) { speed = MOTOR_PWM_MAX; }

    HAL_GPIO_WritePin(MOTOR_L_AIN1_PORT, MOTOR_L_AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_L_AIN2_PORT, MOTOR_L_AIN2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_R_BIN1_PORT, MOTOR_R_BIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_BIN2_PORT, MOTOR_R_BIN2_PIN, GPIO_PIN_SET);

    __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_1, speed);
    __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_2, speed);

    HAL_Delay(time_ms);
    Motor_Stop();
}


   /*Turns the robot by target_deg degrees using PID control.

   HOW IT WORKS:
     1. Record heading at start of turn
     2. Each loop: read IMU, calculate how far we have turned
     3. error = target - turned_so_far
     4. PID converts error → motor speed
     5. Left motor forward, right motor backward (or vice versa)
     6. Stop when error < TURN_TOLERANCE_DEG (2°)

   PARAMETERS:
     target_deg      positive = right turn, negative = left turn
     heading_deg_ptr pointer to your heading_deg variable in main.c
     imu_read_fn     pointer to your IMU_Read() function
     imu_update_fn   pointer to your IMU_UpdateHeading() function

   USAGE in main.c:
     Motor_TurnPID(90.0f,  &heading_deg, IMU_Read, IMU_UpdateHeading);
     Motor_TurnPID(-90.0f, &heading_deg, IMU_Read, IMU_UpdateHeading);
     Motor_TurnPID(180.0f, &heading_deg, IMU_Read, IMU_UpdateHeading);
   ================================================================ */

void Motor_TurnPID(float target_deg, float *heading_deg_ptr,
                   void (*imu_read_fn)(void),
                   void (*imu_update_fn)(void))
{
    PID_t pid;
    PID_Init(&pid, PID_KP, PID_KI, PID_KD);

    /* Record heading at start */
    float start_heading = *heading_deg_ptr;
    uint32_t last_time  = HAL_GetTick();

    /* Safety timeout — maximum time for any turn (ms)
       90° at minimum speed should never take more than 2 seconds */
    uint32_t timeout = HAL_GetTick() + 2000u;

    while (1)
    {
        /* 1. Read fresh IMU data */
        imu_read_fn();
        imu_update_fn();

        /* 2. Calculate how far we have turned so far */
        float turned = *heading_deg_ptr - start_heading;

        /* Unwrap angle — handle 0°/360° boundary crossing
           e.g. if start=350° and now=10°, turned = 20° not -340° */
        if      (turned >  180.0f) { turned -= 360.0f; }
        else if (turned < -180.0f) { turned += 360.0f; }

        /* 3. Error = how many degrees remain */
        float error = target_deg - turned;

        /* 4. Check if turn is complete */
        if (fabsf(error) <= TURN_TOLERANCE_DEG)
        {
            break;
        }

        /* 5. Safety timeout */
        if (HAL_GetTick() >= timeout)
        {
            break;
        }

        /* 6. Calculate dt */
        uint32_t now = HAL_GetTick();
        float dt_sec = (float)(now - last_time) / 1000.0f;
        if (dt_sec <= 0.0f) { dt_sec = 0.001f; }
        last_time = now;

        /* 7. PID output → motor speed */
        float pid_out = PID_Update(&pid, error, dt_sec);

        /* 8. Clamp speed to safe turn range */
        int32_t speed = (int32_t)pid_out;

        /* Apply minimum speed so motors actually move */
        if      (speed >  0 && speed <  (int32_t)MOTOR_PWM_TURN_MIN) { speed =  (int32_t)MOTOR_PWM_TURN_MIN; }
        else if (speed <  0 && speed > -(int32_t)MOTOR_PWM_TURN_MIN) { speed = -(int32_t)MOTOR_PWM_TURN_MIN; }

        /* Apply maximum speed limit */
        if      (speed >  (int32_t)MOTOR_PWM_TURN_MAX) { speed =  (int32_t)MOTOR_PWM_TURN_MAX; }
        else if (speed < -(int32_t)MOTOR_PWM_TURN_MAX) { speed = -(int32_t)MOTOR_PWM_TURN_MAX; }

        /* 9. Drive motors in opposite directions to turn
              Positive speed → turn right: left fwd, right rev
              Negative speed → turn left:  left rev, right fwd  */
        Motor_Left ( speed);
        Motor_Right(-speed);
    }

    /* Turn complete — hard brake */
    Motor_Stop();

    /* Short settle delay — lets robot stop vibrating */
    HAL_Delay(50u);
}
