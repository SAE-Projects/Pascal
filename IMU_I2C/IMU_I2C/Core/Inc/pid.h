/*
 * pid.h
 *
 *  Created on: Apr 4, 2026
 *      Author: Vedant Pathak
 */

#ifndef PID_H
#define PID_H

#include <stdint.h>

/* * A structure to hold the state of a PID controller.
 * Doing it this way means you can have multiple PIDs running at the same time
 * (e.g., one for wall following, one for forward speed).
 */
typedef struct {
    float Kp;
    float Kd;
    int prev_error;
} PID_Controller;

// Function Prototypes
void PID_Init(PID_Controller *pid, float kp, float kd);
float PID_Calculate(PID_Controller *pid, int current_error);

#endif /* PID_H */
