/*
 * pid.c
 *
 *  Created on: Apr 4, 2026
 *      Author: Vedant Pathak
 */

#include "pid.h"

// Initialize the constants and reset the error history
void PID_Init(PID_Controller *pid, float kp, float kd) {
    pid->Kp = kp;
    pid->Kd = kd;
    pid->prev_error = 0;
}

// Calculate the correction value
float PID_Calculate(PID_Controller *pid, int current_error) {
    int derivative = current_error - pid->prev_error;

    float correction = (pid->Kp * current_error) + (pid->Kd * derivative);

    pid->prev_error = current_error;

    return correction;
}
