#include "motion.h"
#include <math.h> // For fabsf
#include "sensors.h"
#include <stdio.h> // For stall detection print

volatile Micromouse_t pascal = {0};

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

// Internal software counters for the encoders
static uint16_t prev_left = 0;
static uint16_t prev_right = 0;
static int32_t left_total_ticks = 0;
static int32_t right_total_ticks = 0;

// --- GLOBAL PID TUNING VARIABLES ---
float Kp_IR = 0.10f;
float Kd_IR = 0.50f;

/* ================================================================
   1. HARDWARE INIT
   ================================================================ */
void Motion_Init(void) {
    // 1. Reset Hardware Timers to 0
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    // 2. Reset Software Accumulators
    prev_left = 0;
    prev_right = 0;
    left_total_ticks = 0;
    right_total_ticks = 0;

    // 3. Reset Pascal's physical state
    pascal.current_distance_mm = 0.0f;
    pascal.target_distance_mm = 0.0f;
    pascal.current_heading = 0.0f;
    pascal.target_heading = 0.0f;
    pascal.state = STATE_IDLE;

    // 4. Restart PWM and Encoders
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_SET);
}

/* ================================================================
   2. MOTOR DRIVERS
   ================================================================ */
static void Set_Left_Motor(int speed) {
    if (speed > 1000) speed = 1000;
    if (speed < -1000) speed = -1000;

    if (speed >= 0) {
        HAL_GPIO_WritePin(MOTOR_L_AIN1_PORT, MOTOR_L_AIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_L_AIN2_PORT, MOTOR_L_AIN2_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
    } else {
        HAL_GPIO_WritePin(MOTOR_L_AIN1_PORT, MOTOR_L_AIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_L_AIN2_PORT, MOTOR_L_AIN2_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -speed);
    }
}

static void Set_Right_Motor(int speed) {
    if (speed > 1000) speed = 1000;
    if (speed < -1000) speed = -1000;

    if (speed >= 0) {
        HAL_GPIO_WritePin(MOTOR_R_BIN1_PORT, MOTOR_R_BIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_R_BIN2_PORT, MOTOR_R_BIN2_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
    } else {
        HAL_GPIO_WritePin(MOTOR_R_BIN1_PORT, MOTOR_R_BIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_R_BIN2_PORT, MOTOR_R_BIN2_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -speed);
    }
}

/* ================================================================
   3. SENSOR UPDATES (Called every 1ms by TIM6)
   ================================================================ */
void Update_Encoders(void) {
    // Harvested from your smart rollover logic!
    uint16_t curr_left = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    uint16_t curr_right = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

    int16_t diff_left = (int16_t)(curr_left - prev_left);
    int16_t diff_right = (int16_t)(curr_right - prev_right);

    left_total_ticks += diff_left;
    right_total_ticks += diff_right;

    prev_left = curr_left;
    prev_right = curr_right;

    // Convert raw ticks to real-world millimeters
    float left_mm = (float)left_total_ticks * MM_PER_TICK;
    float right_mm = (float)right_total_ticks * MM_PER_TICK;

    pascal.current_distance_mm = (left_mm + right_mm) / 2.0f;
}

/* ================================================================
   4. THE AUTOPILOT (Called every 1ms by TIM6)
   ================================================================ */

void Run_Motion_Control_Loop(void) {
    // --- STALL PROTECTION LOGIC ---
    static uint32_t stall_timer = 0;
    static float last_distance = 0.0f;

    if (pascal.state != STATE_IDLE) {
        // Are we actually moving? (Less than 0.5mm change in 1ms)
        if (fabsf(pascal.current_distance_mm - last_distance) < 0.5f) {
            stall_timer++;
            if (stall_timer > 500) { // Stuck for 500ms!
                pascal.state = STATE_IDLE; // EMERGENCY KILL
                Set_Left_Motor(0);
                Set_Right_Motor(0);
                printf("STALL DETECTED! Motors killed.\r\n");
                return;
            }
        } else {
            stall_timer = 0; // We are moving, reset the timer
            last_distance = pascal.current_distance_mm;
        }
    } else {
        stall_timer = 0; // Reset when idle
    }
    // --- END STALL PROTECTION ---

    if (pascal.state == STATE_IDLE) {
        Set_Left_Motor(0);
        Set_Right_Motor(0);
        return;
    }

    // --- STEERING PID (Gyro Tracking) ---
    // The Gyro is your "Global" heading. It always runs.
    float heading_error = pascal.target_heading - pascal.current_heading;

    // Normalize the error to strictly -180 to +180 degrees to prevent float drift
    while (heading_error > 180.0f)  heading_error -= 360.0f;
    while (heading_error < -180.0f) heading_error += 360.0f;

    float Kp_heading = 15.0f; // Tweak this for straightness
    int steering_correction = (int)(heading_error * Kp_heading);

    // --- DISTANCE CHECK ---
    float distance_remaining = pascal.target_distance_mm - pascal.current_distance_mm;

    if (pascal.state == STATE_STRAIGHT) {
        // If we reached the target, stop moving
        if (distance_remaining <= 0.5f) {
            pascal.state = STATE_IDLE;
            return;
        }

        // --- PROPORTIONAL BRAKING ---
        int drive_speed = pascal.base_speed;

        // Start braking when 40mm away from the target
        if (distance_remaining < 40.0f) {
            float brake_factor = distance_remaining / 40.0f;
            drive_speed = (int)(pascal.base_speed * brake_factor);

            // Minimum speed to prevent stalling before the line
            if (drive_speed < 150) drive_speed = 150;
        }

        // ==========================================================
        // --- IR STEERING INJECTION (PD Controller) ---
        // ==========================================================
        // Use 'static' so this variable remembers its value between 1ms ticks!
        static float previous_ir_error = 0.0f;

        // 1. Get the current error
        float current_ir_error = Sensors_Get_Steering_Error();

        // 2. Calculate Derivative (Current Error - Previous Error)
        // If error is shrinking rapidly, this becomes a negative number,
        // acting as a brake against the P-term!
        float derivative_ir = current_ir_error - previous_ir_error;

        // 3. Calculate P and D terms using global variables
        float p_term = current_ir_error * Kp_IR;
        float d_term = derivative_ir * Kd_IR;

        // 4. Combine them
        int ir_correction = (int)(p_term + d_term);

        // 5. CRITICAL: Save the current error for the NEXT millisecond
        previous_ir_error = current_ir_error;

        // 6. Combine the Gyro correction and the new PD IR correction
        // (Keeping your fix with the minus sign!)
        int total_correction = steering_correction - ir_correction;

        // 7. Apply to the motors
        Set_Left_Motor(drive_speed - total_correction);
        Set_Right_Motor(drive_speed + total_correction);
        // ==========================================================
    }

    else if (pascal.state == STATE_TURN_LEFT || pascal.state == STATE_TURN_RIGHT) {

        // 1. Widen the stopping window slightly
        if (fabsf(heading_error) < 1.5f) {
            pascal.state = STATE_IDLE;
            return;
        }

        // 2. Clamp the maximum turn speed
        if (steering_correction > pascal.base_speed) steering_correction = pascal.base_speed;
        if (steering_correction < -pascal.base_speed) steering_correction = -pascal.base_speed;

        // 3. Minimum speed limit (Anti-Stall)
        int min_turn_speed = 120;
        if (steering_correction > 0 && steering_correction < min_turn_speed) steering_correction = min_turn_speed;
        if (steering_correction < 0 && steering_correction > -min_turn_speed) steering_correction = -min_turn_speed;

        // 4. Pivot (Notice we ONLY use steering_correction here, IR is ignored during turns!)
        Set_Left_Motor(-steering_correction);
        Set_Right_Motor(steering_correction);
    }
}
