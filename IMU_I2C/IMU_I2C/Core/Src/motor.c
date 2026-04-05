#include "motor.h"
#include "ism330dhcx_i2c.h"
#include "encoder.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

static TIM_HandleTypeDef *_htim = NULL;

extern ISM330_Phys_t  imu_phys;
extern uint32_t       last_tick;

/* Tell the compiler our microsecond timer exists elsewhere (in main.c) */
extern uint32_t DWT_GetMicros(void);

/* ================================================================
   MOTOR_INIT
   ================================================================ */
void Motor_Init(TIM_HandleTypeDef *htim)
{
    _htim = htim;

    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);   /* LEFT  motor */
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);   /* RIGHT motor */

    Motor_Stop();

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);   /* STBY HIGH */

    printf("[MTR] Init done. Tyre=33mm Axle=50mm\r\n");
}

/* ================================================================
   MOTOR_LEFT
   ================================================================ */
void Motor_Left(int32_t speed)
{
    if (speed >  (int32_t)MOTOR_PWM_MAX) { speed =  (int32_t)MOTOR_PWM_MAX; }
    if (speed < -(int32_t)MOTOR_PWM_MAX) { speed = -(int32_t)MOTOR_PWM_MAX; }

    if (speed >= 0)
    {
        HAL_GPIO_WritePin(MOTOR_L_AIN1_PORT, MOTOR_L_AIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_L_AIN2_PORT, MOTOR_L_AIN2_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_1, (uint32_t)speed);
    }
    else
    {
        HAL_GPIO_WritePin(MOTOR_L_AIN1_PORT, MOTOR_L_AIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_L_AIN2_PORT, MOTOR_L_AIN2_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_1, (uint32_t)(-speed));
    }
}

/* ================================================================
   MOTOR_RIGHT
   ================================================================ */
void Motor_Right(int32_t speed)
{
    if (speed >  (int32_t)MOTOR_PWM_MAX) { speed =  (int32_t)MOTOR_PWM_MAX; }
    if (speed < -(int32_t)MOTOR_PWM_MAX) { speed = -(int32_t)MOTOR_PWM_MAX; }

    if (speed >= 0)
    {
        HAL_GPIO_WritePin(MOTOR_R_BIN1_PORT, MOTOR_R_BIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_R_BIN2_PORT, MOTOR_R_BIN2_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_2, (uint32_t)speed);
    }
    else
    {
        HAL_GPIO_WritePin(MOTOR_R_BIN1_PORT, MOTOR_R_BIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_R_BIN2_PORT, MOTOR_R_BIN2_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_2, (uint32_t)(-speed));
    }
}

/* ================================================================
   MOTOR_STOP
   ================================================================ */
void Motor_Stop(void)
{
    HAL_GPIO_WritePin(MOTOR_L_AIN1_PORT, MOTOR_L_AIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_L_AIN2_PORT, MOTOR_L_AIN2_PIN, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_1, 0u);

    HAL_GPIO_WritePin(MOTOR_R_BIN1_PORT, MOTOR_R_BIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_R_BIN2_PORT, MOTOR_R_BIN2_PIN, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_2, 0u);
}

/* ================================================================
   MOTOR_FORWARD / BACKWARD
   (HAL_Delay is kept here because these are simple blocking moves
   where microsecond precision is not required)
   ================================================================ */
void Motor_Forward(uint32_t time_ms, uint32_t speed)
{
    if (speed > MOTOR_PWM_MAX) { speed = MOTOR_PWM_MAX; }
    printf("[MTR] FWD speed=%lu time=%lums\r\n", speed, time_ms);
    HAL_GPIO_WritePin(MOTOR_L_AIN1_PORT, MOTOR_L_AIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_L_AIN2_PORT, MOTOR_L_AIN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_BIN1_PORT, MOTOR_R_BIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_R_BIN2_PORT, MOTOR_R_BIN2_PIN, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_1, speed);
    __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_2, speed);
    HAL_Delay(time_ms);
    Motor_Stop();
    printf("[MTR] STOP\r\n");
}

void Motor_Backward(uint32_t time_ms, uint32_t speed)
{
    if (speed > MOTOR_PWM_MAX) { speed = MOTOR_PWM_MAX; }
    printf("[MTR] BWD speed=%lu time=%lums\r\n", speed, time_ms);
    HAL_GPIO_WritePin(MOTOR_L_AIN1_PORT, MOTOR_L_AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_L_AIN2_PORT, MOTOR_L_AIN2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_R_BIN1_PORT, MOTOR_R_BIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_BIN2_PORT, MOTOR_R_BIN2_PIN, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_1, speed);
    __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_2, speed);
    HAL_Delay(time_ms);
    Motor_Stop();
    printf("[MTR] STOP\r\n");
}

/* ================================================================
   UNIVERSAL MOTOR_TURN_HDG
   Positive target = Right Turn (Clockwise)
   Negative target = Left Turn (Counter-Clockwise)
   ================================================================ */
/* ================================================================
   UNIVERSAL MOTOR_TURN_HDG (Proportional Braking)
   Positive target = Right Turn (Clockwise)
   Negative target = Left Turn (Counter-Clockwise)
   ================================================================ */
/* ================================================================
   UNIVERSAL MOTOR_TURN_HDG (Proportional Braking + Momentum Offset)
   ================================================================ */
/* ================================================================
   UNIVERSAL MOTOR_TURN_HDG (Trapezoidal Velocity Profile)
   Slow start -> Max Speed -> Smooth Brake -> Momentum Offset
   ================================================================ */
/* ================================================================
   UNIVERSAL MOTOR_TURN_HDG (Unified PID Controller)
   Handles any target angle dynamically.
   ================================================================ */
/* ================================================================
   UNIVERSAL MOTOR_TURN_HDG (PID + Acceleration Ramp)
   Slow start, Fast middle, Perfect PID brake.
   ================================================================ */
void Motor_Turn_HDG(float target_deg, float stop_threshold_deg,
                    float *heading_deg_ptr,
                    void (*imu_read_fn)(void),
                    void (*imu_update_fn)(void))
{
    imu_read_fn();
    *heading_deg_ptr = 0.0f;
    last_tick = DWT_GetMicros();

    /* --- PID & RAMP TUNING VARIABLES --- */
    float Kp = 10.0f;   // Stronger pull to the target
    float Ki = 0.0f;    // Nudges the bot if it gets stuck
    float Kd = 0.2f;    // Softer braking so it doesn't stop short

    float min_turn_speed = 200.0f; // Minimum PWM to prevent stalling

    /* NEW: How fast to let the speed build up at the start */
    float K_accel = 7.0f;

    // Variables to track history for PID math
    float error = target_deg;
    float integral = 0.0f;
    uint32_t last_pid_time = DWT_GetMicros();

    uint32_t timeout_limit_us = ((fabsf(target_deg) > 100.0f) ? (TURN_TIMEOUT_MS * 2) : TURN_TIMEOUT_MS) * 1000u;
    uint32_t timeout_us = DWT_GetMicros() + timeout_limit_us;
    uint32_t t_print_us = DWT_GetMicros();

    printf("[MTR] PID+RAMP TURN Target=%.1f\r\n", (double)target_deg);

    while (1)
    {
        imu_read_fn();
        imu_update_fn();

        uint32_t now = DWT_GetMicros();
        float dt = (float)(now - last_pid_time) / 1000000.0f;
        if (dt <= 0.0001f) { dt = 0.0001f; }

        float current_hdg = *heading_deg_ptr;
        float abs_hdg = fabsf(current_hdg);

        /* 1. Calculate Error */
        error = target_deg - current_hdg;

        /* STOP CONDITION */
        if (fabsf(error) < 0.5f && fabsf(imu_phys.gz) < 5.0f)
        {
            printf("[MTR] TARGET HIT  hdg=%.2f\r\n", (double)current_hdg);
            break;
        }

        if (now >= timeout_us) {
            printf("[MTR] TIMEOUT\r\n");
            break;
        }

        /* --- THE PID MATH --- */
        float P = error * Kp;

        integral += error * dt;
        if (integral >  50.0f) integral =  50.0f;
        if (integral < -50.0f) integral = -50.0f;
        float I = integral * Ki;

        // Derivative using native gyro velocity
        float D = -imu_phys.gz * Kd;

        // Total Output
        float output = P + I + D;
        float abs_output = fabsf(output);

        /* --- THE ACCELERATION RAMP --- */
        /* Calculates a maximum allowed speed that starts at min_turn_speed
           and grows as the robot turns. */
        float max_allowed_speed = min_turn_speed + (abs_hdg * K_accel);

        /* If PID wants to go faster than the ramp allows, CHOP the speed down */
        if (abs_output > max_allowed_speed) {
            abs_output = max_allowed_speed;
        }

        /* --- FINAL SAFETY CLAMPS --- */
        // Don't stall at the very end
        if (fabsf(error) > 0.5f && abs_output < min_turn_speed) {
            abs_output = min_turn_speed;
        }

        // Don't exceed maximum hardware limit
        if (abs_output > (float)TURN_SPEED) {
            abs_output = (float)TURN_SPEED;
        }

        int32_t drive_speed = (int32_t)abs_output;

        // Handle Left vs Right
        if (output > 0.0f) {
            Motor_Left(drive_speed);
            Motor_Right(-drive_speed);
        } else {
            Motor_Left(-drive_speed);
            Motor_Right(drive_speed);
        }

        last_pid_time = now;

        /* Debug print every 80ms */
        if ((now - t_print_us) >= 80000u) {
            t_print_us = now;
            // printf("[MTR] hdg=%.1f  err=%.1f  spd=%ld\r\n", (double)current_hdg, (double)error, drive_speed);
        }
    }

    Motor_Stop();
    HAL_Delay(100u);

    float final = *heading_deg_ptr;
    printf("[MTR] SETTLED. Final=%.2f  Error=%.2f deg\r\n",
           (double)final, (double)(final - target_deg));
}

/* ================================================================
   MOTOR_DRIVESTRAIGHT_HDG
   Drives forward for a set time while actively using the gyro
   to correct any mechanical drift.
   ================================================================ */

void Motor_DriveStraight_HDG(uint32_t time_ms,
                             uint32_t base_speed,
                             float *heading_deg_ptr,
                             void (*imu_read_fn)(void),
                             void (*imu_update_fn)(void))
{
    /* 1. Flush stale data and reset heading */
    imu_read_fn();
    *heading_deg_ptr = 0.0f;
    last_tick = DWT_GetMicros();

    /* 2. Setup timing */
    uint32_t start_time_us = DWT_GetMicros();
    uint32_t duration_us   = time_ms * 1000u;
    uint32_t t_print_us    = start_time_us;

    /* 3. Proportional Gain (Kp)
       How aggressively it corrects drift.
       If it wiggles side-to-side, lower this number.
       If it still drifts, raise this number. */
    float kp = 12.0f;

    printf("[MTR] DRIVE STRAIGHT  speed=%lu  time=%lums\r\n", base_speed, time_ms);

    /* 4. Active Tracking Loop */
    while ((DWT_GetMicros() - start_time_us) < duration_us)
    {
        imu_read_fn();
        imu_update_fn();

        float current_heading = *heading_deg_ptr;

        /* Calculate the correction factor.
           If drifting right (positive heading), correction is positive. */
        float correction = current_heading * kp;

        /* Apply correction to motor speeds.
           To fix a right drift, left motor slows down, right motor speeds up. */
        int32_t left_speed  = (int32_t)base_speed - (int32_t)correction;
        int32_t right_speed = (int32_t)base_speed + (int32_t)correction;

        /* Drive the motors */
        Motor_Left(left_speed);
        Motor_Right(right_speed);

        /* Print debug info every 80ms */
        if ((DWT_GetMicros() - t_print_us) >= 80000u)
        {
            t_print_us = DWT_GetMicros();
            printf("[MTR] STR  hdg=%5.2f  L=%ld  R=%ld\r\n",
                   (double)current_heading, left_speed, right_speed);
        }
    }

    /* 5. Stop and Settle */
    Motor_Stop();
    HAL_Delay(80u);

    printf("[MTR] DONE STRAIGHT. Final drift: %.2f deg\r\n", (double)*heading_deg_ptr);
}



/* ================================================================
   CLEAN WRAPPER FUNCTIONS
   ================================================================ */
void Motor_Turn45R_HDG(float *hdg, void (*rd)(void), void (*upd)(void)) {
    Motor_Turn_HDG(TARGET_45R_DEG, 0.0f, hdg, rd, upd);
}

void Motor_Turn45L_HDG(float *hdg, void (*rd)(void), void (*upd)(void)) {
    Motor_Turn_HDG(TARGET_45L_DEG, 0.0f, hdg, rd, upd);
}

void Motor_Turn90R_HDG(float *hdg, void (*rd)(void), void (*upd)(void)) {
    Motor_Turn_HDG(TARGET_90R_DEG, 0.0f, hdg, rd, upd);
}

void Motor_Turn90L_HDG(float *hdg, void (*rd)(void), void (*upd)(void)) {
    Motor_Turn_HDG(TARGET_90L_DEG, 0.0f, hdg, rd, upd);
}

void Motor_Turn180R_HDG(float *hdg, void (*rd)(void), void (*upd)(void)) {
    Motor_Turn_HDG(TARGET_180R_DEG, 0.0f, hdg, rd, upd);
}

void Motor_Turn180L_HDG(float *hdg, void (*rd)(void), void (*upd)(void)) {
    Motor_Turn_HDG(TARGET_180L_DEG, 0.0f, hdg, rd, upd);
}

void Motor_Turn360R_HDG(float *hdg, void (*rd)(void), void (*upd)(void)) {
    Motor_Turn_HDG(TARGET_360R_DEG, 0.0f, hdg, rd, upd);
}

void Motor_Turn360L_HDG(float *hdg, void (*rd)(void), void (*upd)(void)) {
    Motor_Turn_HDG(TARGET_360L_DEG, 0.0f, hdg, rd, upd);
}
/* ================================================================
   MOTOR_DRIVEDISTANCE_HDG
   Drives perfectly straight for an exact distance (in millimeters).
   Uses Gyro for straightness, Encoders for distance, and
   proportional braking to prevent momentum overshoot.
   ================================================================ */
/* ================================================================
   MOTOR_DRIVEDISTANCE_HDG (Pro-Level Sensor Fusion)
   Encoders control throttle/braking. Gyro controls steering.
   Features an Acceleration Ramp to prevent tire slip.
   ================================================================ */
void Motor_DriveDistance_HDG(float target_distance_mm,
                             uint32_t max_speed,
                             float *heading_deg_ptr,
                             void (*imu_read_fn)(void),
                             void (*imu_update_fn)(void))
{
    /* 1. Reset all sensors for a clean slate */
    Encoder_ResetBoth();
    imu_read_fn();
    *heading_deg_ptr = 0.0f;

    uint32_t last_tick = DWT_GetMicros();
    uint32_t t_print_us = last_tick;

    /* 2. Handle Forward vs Backward Logic */
    uint8_t is_forward = (target_distance_mm >= 0.0f);
    float abs_target_mm = fabsf(target_distance_mm);

    /* 3. Distance Tuning Parameters */
    float kp_dist    = 4.0f;   /* How fast to brake at the end (Throttle down) */
    float k_accel    = 3.0f;   /* How fast to ramp up speed at the start */
    float min_speed  = 170.0f; /* Minimum PWM to prevent stalling */

    /* --- NEW: Steering PID Tuning Parameters --- */
    float kp_heading = 25.0f;  /* Proportional: Steer aggressively against immediate error */
    float ki_heading = 0.02f;   /* Integral: Accumulates over time to fix physical drift/bias */
    float kd_heading = 0.8f;  /* Derivative: Dampens oscillations to prevent fishtailing */

    float heading_integral = 0.0f;
    float heading_prev_error = 0.0f;
    float max_integral = 100.0f; /* Anti-windup: Max correction the I-term can apply */

    printf("[MTR] DRIVE %s target=%.1fmm max_speed=%lu\r\n",
           is_forward ? "FWD" : "BWD", (double)abs_target_mm, max_speed);

    /* 4. Active Tracking Loop */
    while (1)
    {
        /* Calculate Delta Time (dt) in seconds for PID math */
        uint32_t current_tick = DWT_GetMicros();
        float dt = (float)(current_tick - last_tick) / 1000000.0f;
        last_tick = current_tick;

        /* Prevent division by zero if loop runs too fast */
        if (dt <= 0.0001f) { dt = 0.0001f; }

        /* Update all sensors */
        imu_read_fn();
        imu_update_fn();
        Encoder_Update(); // MUST call this to read the hardware!

        /* Get current distance in millimeters from both wheels */
        float dist_l = Encoder_GetDistanceLeft();
        float dist_r = Encoder_GetDistanceRight();

        /* Use absolute values so math works for driving backward too */
        float current_dist_mm = fabsf((dist_l + dist_r) / 2.0f);

        /* Calculate how far we have left to go */
        float error_dist = abs_target_mm - current_dist_mm;

        /* STOP CONDITION: We reached or passed the target */
        if (error_dist <= 0.0f)
        {
            printf("[MTR] TARGET REACHED. Final Dist=%.1fmm\r\n", (double)current_dist_mm);
            break;
        }

        /* --- THROTTLE CONTROL (Encoders) --- */
        float base_speed = error_dist * kp_dist;

        float max_allowed_speed = min_speed + (current_dist_mm * k_accel);
        if (base_speed > max_allowed_speed) { base_speed = max_allowed_speed; }

        if (base_speed > (float)max_speed) { base_speed = (float)max_speed; }
        if (base_speed < min_speed)        { base_speed = min_speed; }

        /* --- STEERING CONTROL (Gyro PID) --- */
        float current_heading = *heading_deg_ptr;

        /* For a target of 0 degrees, the error is simply the current heading. */
        float p_term = current_heading * kp_heading;

        /* Integral term + Anti-windup clamping */
        heading_integral += current_heading * dt;
        if (heading_integral > max_integral)       { heading_integral = max_integral; }
        else if (heading_integral < -max_integral) { heading_integral = -max_integral; }
        float i_term = heading_integral * ki_heading;

        /* Derivative term */
        float d_term = ((current_heading - heading_prev_error) / dt) * kd_heading;
        heading_prev_error = current_heading;

        /* Total PID Correction */
        float correction = p_term + i_term + d_term;

        /* Apply steering correction */
        int32_t left_speed, right_speed;

        if (is_forward) {
            left_speed  = (int32_t)(base_speed - correction);
            right_speed = (int32_t)(base_speed + correction);
        } else {
            left_speed  = -(int32_t)(base_speed + correction);
            right_speed = -(int32_t)(base_speed - correction);
        }

        /* Drive the motors */
        Motor_Left(left_speed);
        Motor_Right(right_speed);

        /* Print real-time telemetry every 80ms */
        if ((current_tick - t_print_us) >= 80000u)
        {
            t_print_us = current_tick;
            printf("[MTR] Dist: %5.1f / %5.1f mm | Heading: %5.2f deg\r\n",
                   (double)current_dist_mm, (double)abs_target_mm, (double)current_heading);
        }
    }

    /* 5. Hard Brake and Settle */
//    Motor_Stop();
//    HAL_Delay(150u);

    /* Print final resting position */
    Encoder_Update();
    float final_dist = (Encoder_GetDistanceLeft() + Encoder_GetDistanceRight()) / 2.0f;
    printf("[MTR] DONE. Final Resting Dist: %.1fmm. Heading Error: %.2f deg\r\n",
           (double)final_dist, (double)*heading_deg_ptr);
}


/* ================================================================
   SMOOTH ARC TURNS (Differential Steering)
   ================================================================ */

/* Helper: Slows down the left wheel to pull the robot into a left arc. */
void Motor_SetArcLeft(int32_t base_speed, float turn_ratio)
{
    if (turn_ratio < 0.0f) turn_ratio = 0.0f;
    if (turn_ratio > 1.0f) turn_ratio = 1.0f;

    int32_t left_speed  = (int32_t)(base_speed * turn_ratio);
    int32_t right_speed = base_speed;

    Motor_Left(left_speed);
    Motor_Right(right_speed);
}

/* Helper: Slows down the right wheel to pull the robot into a right arc. */
void Motor_SetArcRight(int32_t base_speed, float turn_ratio)
{
    if (turn_ratio < 0.0f) turn_ratio = 0.0f;
    if (turn_ratio > 1.0f) turn_ratio = 1.0f;

    int32_t left_speed  = base_speed;
    int32_t right_speed = (int32_t)(base_speed * turn_ratio);

    Motor_Left(left_speed);
    Motor_Right(right_speed);
}

/* Main Arc Turn Function: Uses Gyro to track the angle */
void Motor_ArcTurn_HDG(float target_angle_deg,
                       int32_t base_speed,
                       float turn_ratio,
                       uint8_t is_left,
                       uint8_t stop_at_end,
                       float *heading_deg_ptr,
                       void (*imu_read_fn)(void),
                       void (*imu_update_fn)(void))
{
    /* 1. Reset the gyro so we track exactly how far we have turned */
    imu_read_fn();
    *heading_deg_ptr = 0.0f;

    float abs_target_angle = fabsf(target_angle_deg);

    printf("[MTR] ARC TURN %s Target=%.1f deg Speed=%ld Ratio=%.2f\r\n",
           is_left ? "LEFT" : "RIGHT", (double)abs_target_angle, base_speed, (double)turn_ratio);

    /* 2. Start the motors turning */
    if (is_left) {
        Motor_SetArcLeft(base_speed, turn_ratio);
    } else {
        Motor_SetArcRight(base_speed, turn_ratio);
    }

    /* 3. Monitoring Loop */
    while (1)
    {
        /* Update the IMU hardware */
        imu_read_fn();
        imu_update_fn();

        /* Get absolute angle traveled */
        float current_angle = fabsf(*heading_deg_ptr);

        /* STOP CONDITION: We reached or passed the target angle */
        if (current_angle >= abs_target_angle)
        {
            printf("[MTR] ARC TURN COMPLETE. Final Angle=%.1f deg\r\n", (double)*heading_deg_ptr);
            break;
        }
    }

    /* 4. Fluidity Control */
    if (stop_at_end)
    {
        Motor_Stop();
        HAL_Delay(150u);
    }
}


/* ================================================================
   CONTINUOUS 90-DEGREE CURVE TURNS
   Executes a 90-degree arc turn without braking. Once the target
   heading is reached, both motors immediately resume the base_speed.

   turn_ratio: 0.0 to 1.0.
               0.0 = Inner wheel stops (pivot turn).
               0.5 = Inner wheel moves at 50% speed (smooth, wide arc).
               0.8 = Inner wheel moves at 80% speed (very wide, gentle arc).
   ================================================================ */

void Motor_Curve90R_Continuous(int32_t base_speed,
                               float turn_ratio,
                               float *heading_deg_ptr,
                               void (*imu_read_fn)(void),
                               void (*imu_update_fn)(void))
{
    // 1. Execute the right arc turn.
    // is_left = 0, stop_at_end = 0
    Motor_ArcTurn_HDG(TARGET_90R_DEG, base_speed, turn_ratio, 0u, 0u,
                      heading_deg_ptr, imu_read_fn, imu_update_fn);

    // 2. The millisecond the turn is complete, slam both motors back to base_speed
    // so it continues driving straight without losing momentum.
    Motor_Left(base_speed);
    Motor_Right(base_speed);

    printf("[MTR] CURVE 90R COMPLETE. Resuming straight at speed: %ld\r\n", base_speed);
}

void Motor_Curve90L_Continuous(int32_t base_speed,
                               float turn_ratio,
                               float *heading_deg_ptr,
                               void (*imu_read_fn)(void),
                               void (*imu_update_fn)(void))
{
    // 1. Execute the left arc turn.
    // is_left = 1, stop_at_end = 0
    // Note: Motor_ArcTurn_HDG automatically takes the absolute value of TARGET_90L_DEG
    Motor_ArcTurn_HDG(TARGET_90L_DEG, base_speed, turn_ratio, 1u, 0u,
                      heading_deg_ptr, imu_read_fn, imu_update_fn);

    // 2. Immediately resume straight forward motion.
    Motor_Left(base_speed);
    Motor_Right(base_speed);

    printf("[MTR] CURVE 90L COMPLETE. Resuming straight at speed: %ld\r\n", base_speed);
}

/* ================================================================
   CONTINUOUS 45-DEGREE CURVE TURNS
   Executes a 45-degree arc turn without braking. Once the target
   heading is reached, both motors immediately resume the base_speed.
   ================================================================ */

void Motor_Curve45R_Continuous(int32_t base_speed,
                               float turn_ratio,
                               float *heading_deg_ptr,
                               void (*imu_read_fn)(void),
                               void (*imu_update_fn)(void))
{
    // 1. Execute the right arc turn for 45 degrees.
    // is_left = 0, stop_at_end = 0
    Motor_ArcTurn_HDG(TARGET_45R_DEG, base_speed, turn_ratio, 0u, 0u,
                      heading_deg_ptr, imu_read_fn, imu_update_fn);

    // 2. Immediately resume straight forward motion.
    Motor_Left(base_speed);
    Motor_Right(base_speed);

    printf("[MTR] CURVE 45R COMPLETE. Resuming straight at speed: %ld\r\n", base_speed);
}

void Motor_Curve45L_Continuous(int32_t base_speed,
                               float turn_ratio,
                               float *heading_deg_ptr,
                               void (*imu_read_fn)(void),
                               void (*imu_update_fn)(void))
{
    // 1. Execute the left arc turn for 45 degrees.
    // is_left = 1, stop_at_end = 0
    Motor_ArcTurn_HDG(TARGET_45L_DEG, base_speed, turn_ratio, 1u, 0u,
                      heading_deg_ptr, imu_read_fn, imu_update_fn);

    // 2. Immediately resume straight forward motion.
    Motor_Left(base_speed);
    Motor_Right(base_speed);

    printf("[MTR] CURVE 45L COMPLETE. Resuming straight at speed: %ld\r\n", base_speed);
}

