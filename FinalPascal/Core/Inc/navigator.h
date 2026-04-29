#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include "stm32g4xx_hal.h"
#include "motion.h"
#include "sensors.h"
#include "maze.h"

// Threshold for the highly sensitive front sensors
#define FRONT_WALL_THRESHOLD 150

// Threshold for the weaker side sensors
#define SIDE_WALL_THRESHOLD 150

// Standard Micromouse cell is 180mm. Change this to tune overshoot!
#define CELL_SIZE_MM 200.0f

// Threshold to hit the brakes before crashing (Stop Distance)
// 1000 = touching wall. Tune this to stop ~2cm away.
#define FRONT_CRASH_THRESHOLD 850

// The core function to run in your while(1) loop
void Navigator_Run_LeftWallFollow(void);
void Navigator_Run_Floodfill(void);
void Navigator_Solve_Step(void);
void Navigator_Auto_Calibrate(void);
void Navigator_Manual_Calibrate(void);
// The new Speed Run function!
void Navigator_Final_Run(void);

#endif
