#include "navigator.h"
#include "floodfill.h"
#include <stdio.h>

// --- ADD THESE INCLUDES ---
#include "ism330dhcx_i2c.h"  // Fixes ISM330 function warnings/errors
#include "main.h"            // Fixes HAL and handle errors

// --- ADD THESE EXTERN DECLARATIONS ---
// This tells navigator.c that these exist in main.c
extern I2C_HandleTypeDef hi2c1;
extern IWDG_HandleTypeDef hiwdg;
extern float gyro_z_bias;
extern ISM330_Raw_t imu_raw;
extern ISM330_Phys_t imu_phys;
extern void Safe_Delay(uint32_t ms);

void Navigator_Run_LeftWallFollow(void) {
    // 1. Prepare for decision
    IR_Apply_Calibration();

    // SMOOTH ACCELERATION: Ramp up to 350 over ~70 milliseconds
    if (pascal.base_speed == 0) {
        for (int speed = 0; speed <= 350; speed += 25) {
            pascal.base_speed = speed;
            HAL_IWDG_Refresh(&hiwdg); // Keep watchdog happy
            HAL_Delay(5);             // 5ms delay per step
        }
    } else {
        pascal.base_speed = 350; // Already moving, keep speed
    }

    // --- PASTE THIS DIAGNOSTIC PRINT ---
    printf("VISION -> L:%d | F_L:%d | F_R:%d | R:%d\r\n",
           my_mouse_sensors.left_cal,
           my_mouse_sensors.left_fwd_cal,
           my_mouse_sensors.right_fwd_cal,
           my_mouse_sensors.right_cal);

    // Read current wall states with split thresholds
    uint8_t wall_left  = (my_mouse_sensors.left_cal > SIDE_WALL_THRESHOLD);
    uint8_t wall_front = (my_mouse_sensors.left_fwd_cal > FRONT_WALL_THRESHOLD || my_mouse_sensors.right_fwd_cal > FRONT_WALL_THRESHOLD);
    uint8_t wall_right = (my_mouse_sensors.right_cal > SIDE_WALL_THRESHOLD);

    // Update our internal map memory as we go
    Map_Current_Cell(my_mouse_sensors.left_fwd_cal, my_mouse_sensors.left_diag_cal, my_mouse_sensors.right_diag_cal);

    /* --- THE LEFT-HAND RULE DECISION TREE --- */

    if (!wall_left) {
        // Priority 1: If Left is open, always take it.
        __disable_irq(); // Pause TIM6
        pascal.target_heading += 90.0f;
        pascal.state = STATE_TURN_LEFT;
        __enable_irq();  // Resume TIM6

        // WATCHDOG FIX: Added refresh inside the wait loop
        while(pascal.state != STATE_IDLE) {
            HAL_IWDG_Refresh(&hiwdg);
            HAL_Delay(1);
        }
        Update_Compass_TurnLeft();
    }
    else if (wall_front) {
        // Priority 2: Left is blocked. If Front is also blocked, we must turn.
        if (!wall_right) {
            __disable_irq();
            pascal.target_heading -= 90.0f;
            pascal.state = STATE_TURN_RIGHT;
            __enable_irq();

            // WATCHDOG FIX
            while(pascal.state != STATE_IDLE) {
                HAL_IWDG_Refresh(&hiwdg);
                HAL_Delay(1);
            }
            Update_Compass_TurnRight();
        } else {
            // Priority 3: Dead end.
            __disable_irq();
            pascal.target_heading -= 180.0f; // Note: This is an angle (degrees), not distance! Keep as 180.0f.
            pascal.state = STATE_TURN_RIGHT;
            __enable_irq();

            // WATCHDOG FIX
            while(pascal.state != STATE_IDLE) {
                HAL_IWDG_Refresh(&hiwdg);
                HAL_Delay(1);
            }
            Update_Compass_TurnAround();
        }
    }
    // If Left is blocked but Front is clear, we do nothing and just stay facing forward.

    // 2. Drive Forward 1 Cell
    __disable_irq();
    pascal.target_distance_mm = pascal.current_distance_mm + CELL_SIZE_MM;
    pascal.state = STATE_STRAIGHT;
    __enable_irq();

    // Wait for movement to finish
    while(pascal.state != STATE_IDLE) {
        HAL_IWDG_Refresh(&hiwdg); // WATCHDOG FIX: Crucial for long straight drives
        IR_Apply_Calibration();   // Keep sensors fresh for safety/logging

        // --- NEW SAFETY OVERRIDE ---
        // If either front sensor sees a wall getting dangerously close
        if (my_mouse_sensors.left_fwd_cal > FRONT_CRASH_THRESHOLD ||
            my_mouse_sensors.right_fwd_cal > FRONT_CRASH_THRESHOLD) {

            __disable_irq();
            pascal.state = STATE_IDLE;  // Hit the brakes immediately
            __enable_irq();
            break;                      // Exit the movement loop early
        }

        HAL_Delay(1);
    }

    // 3. Finalize movement
    Update_Position_Forward();
    printf("Pos: (%d, %d) | Dir: %d\r\n", pascal_x, pascal_y, pascal_dir);

    printf("Settling & Re-Zeroing...\r\n");
    float drift_sum = 0.0f;
    int samples = 30;

    for (int i = 0; i < samples; i++) {
        if (ISM330_ReadRaw(&hi2c1, &imu_raw) == ISM330_OK) {
            ISM330_ToPhysical(&imu_raw, &imu_phys);
            drift_sum += imu_phys.gz;
        }
        HAL_IWDG_Refresh(&hiwdg); // Keep the watchdog happy!
        HAL_Delay(5);
    }
    // Update global bias with a moving average (gentle correction)
    gyro_z_bias = (0.8f * gyro_z_bias) + (0.2f * (drift_sum / (float)samples));
}

void Navigator_Solve_Step(void) {
    // 1. SCAN: Map the cell I am physically standing in RIGHT NOW
    IR_Apply_Calibration();
    Map_Current_Cell(my_mouse_sensors.left_fwd_cal, my_mouse_sensors.left_cal, my_mouse_sensors.right_cal);

    // Print the map to serial so we see it update live
    Maze_Print_Map();

    // Smoothly ramp up speed if we are starting from a dead stop
    if (pascal.base_speed == 0) {
        for (int speed = 0; speed <= 350; speed += 25) {
            pascal.base_speed = speed;
            HAL_IWDG_Refresh(&hiwdg);
            HAL_Delay(5);
        }
    } else {
        pascal.base_speed = 350; // Keep cruising speed
    }

    // 2. THINK: Calculate Floodfill and decide where to go
    Floodfill_Recalculate();
    Direction_t next_dir = Floodfill_Get_Best_Dir(pascal_x, pascal_y, pascal_dir);

    // 3. ACT: Turn OR Move
    if (next_dir != pascal_dir) {
        // --- SITUATION A: I need to turn ---
        int turn_diff = (next_dir - pascal_dir + 4) % 4;

        if (turn_diff == 1) { // RIGHT
            __disable_irq();
            pascal.target_heading -= 90.0f;
            pascal.state = STATE_TURN_RIGHT;
            __enable_irq();
            while(pascal.state != STATE_IDLE) {
                HAL_IWDG_Refresh(&hiwdg);
                HAL_Delay(1);
            }
            Update_Compass_TurnRight();
        }
        else if (turn_diff == 2) { // U-TURN
            __disable_irq();
            pascal.target_heading -= 180.0f;
            pascal.state = STATE_TURN_RIGHT;
            __enable_irq();
            while(pascal.state != STATE_IDLE) {
                HAL_IWDG_Refresh(&hiwdg);
                HAL_Delay(1);
            }
            Update_Compass_TurnAround();
        }
        else if (turn_diff == 3) { // LEFT
            __disable_irq();
            pascal.target_heading += 90.0f;
            pascal.state = STATE_TURN_LEFT;
            __enable_irq();
            while(pascal.state != STATE_IDLE) {
                HAL_IWDG_Refresh(&hiwdg);
                HAL_Delay(1);
            }
            Update_Compass_TurnLeft();
        }

        Safe_Delay(100);
        return;
    }
    else {
        // --- SITUATION B: I'm facing the right way, MOVE FORWARD ---
        __disable_irq();
        pascal.target_distance_mm = pascal.current_distance_mm + CELL_SIZE_MM;
        pascal.state = STATE_STRAIGHT;
        __enable_irq();

        while(pascal.state != STATE_IDLE) {
            HAL_IWDG_Refresh(&hiwdg);
            IR_Apply_Calibration();

            // Emergency Brake
            if (my_mouse_sensors.left_fwd_cal > FRONT_CRASH_THRESHOLD ||
                my_mouse_sensors.right_fwd_cal > FRONT_CRASH_THRESHOLD) {
                __disable_irq();
                pascal.state = STATE_IDLE;
                __enable_irq();
                printf("FRONT WALL DETECTED! Braking for safety.\n");
                break;
            }
            HAL_Delay(1);
        }

        // CRITICAL FIX: ALWAYS update position! If we braked because of a wall,
        // we still successfully entered the cell. The brain must follow the body!
        Update_Position_Forward();
    }
}

/* ================================================================
   THE CALIBRATION DANCE
   ================================================================ */
void Navigator_Auto_Calibrate(void) {
    // 1. Start mapping the min/max values in the background
    IR_Start_Dynamic_Calibration();

    int original_speed = pascal.base_speed;
    pascal.base_speed = 400;

    int print_throttle = 0; // Add a counter

    // 2. Loop four times to achieve a full 360-degree spin safely!
    for (int turn = 0; turn < 4; turn++) {
        __disable_irq();
        pascal.target_heading += 90.0f; // 90 degrees per step
        pascal.state = STATE_TURN_RIGHT;
        __enable_irq();

        // 3. Wait for this 90-degree chunk to complete
        while (pascal.state != STATE_IDLE) {
            HAL_IWDG_Refresh(&hiwdg); // WATCHDOG FIX: Keep alive during spin

            // Print the LIVE RAW vision every ~100ms (20 loops * 5ms)
            if (print_throttle % 20 == 0) {
                printf("SPINNING -> L: %4d | F_L: %4d | F_R: %4d | R: %4d\r\n",
                       my_mouse_sensors.left,
                       my_mouse_sensors.left_fwd,
                       my_mouse_sensors.right_fwd,
                       my_mouse_sensors.right);
            }
            print_throttle++;

            HAL_Delay(5);
        }

        // Small 50ms pause between 90-degree chunks to keep gyro happy
        Safe_Delay(50);
    }

    // 4. Lock in the calibration
    IR_Stop_Dynamic_Calibration();

    // 5. --- THE DYNAMIC SNAPSHOT ---
    printf("Spin complete. Settling...\r\n");

    // Give Pascal 250ms to stop shaking/wobbling after the spin
    Safe_Delay(250);

    // Take the snapshot! The bot is perfectly centered right now.
    Sensors_Capture_Center_Targets();

    // Restore normal driving speed
    pascal.base_speed = original_speed;
}

void Navigator_Manual_Calibrate(void) {
    printf("\r\n--- MANUAL CALIBRATION STARTING (5 SECONDS) ---\r\n");
    printf("Action: Move a wall piece close and far from EVERY sensor now!\r\n");

    // 1. Reset and start tracking min/max values
    IR_Start_Dynamic_Calibration();

    uint32_t startTime = HAL_GetTick();
    uint32_t lastPrint = 0;

    // 2. Loop for 5000ms (5 seconds)
    while ((HAL_GetTick() - startTime) < 5000) {
        HAL_IWDG_Refresh(&hiwdg); // WATCHDOG FIX: Essential for a 5-second loop!

        // Print raw values every 100ms so you can see the peaks happening
        if (HAL_GetTick() - lastPrint > 100) {
            printf("CALIBRATING... L:%4d | FL:%4d | FR:%4d | R:%4d | Time Left: %ldms\r\n",
                   my_mouse_sensors.left,
                   my_mouse_sensors.left_fwd,
                   my_mouse_sensors.right_fwd,
                   my_mouse_sensors.right,
                   5000 - (HAL_GetTick() - startTime));
            lastPrint = HAL_GetTick();
        }

        // Brief delay to prevent UART flooding
        HAL_Delay(10);
    }

    // 3. Lock in the values
    IR_Stop_Dynamic_Calibration();

    printf("--- MANUAL CALIBRATION COMPLETE ---\r\n");
}

// navigator.c

void Navigator_Final_Run(void) {
    // ====================================================================
    // 1. THE "KIDNAPPED ROBOT" RESET
    // You physically picked him up, so we MUST wipe his physical memory!
    // ====================================================================
    __disable_irq();
    // Reset Coordinates to Start
    pascal_x = 0;
    pascal_y = 0;
    pascal_dir = NORTH;

    // WIPE the Gyro and Encoders back to absolute zero!
    pascal.current_heading = 0.0f;
    pascal.target_heading = 0.0f;
    pascal.current_distance_mm = 0.0f;
    pascal.target_distance_mm = 0.0f;
    pascal.state = STATE_IDLE;
    __enable_irq();

    // Give the gyro a moment to realize it's stationary again
    Safe_Delay(250);

    // Run the Floodfill math one last time to perfectly calculate the route
    Floodfill_Recalculate();

    // ====================================================================
    // 2. THE SPEED RUN LAUNCH
    // ====================================================================
    pascal.base_speed = 950; // BLAST OFF SPEED! (Tune this up or down)

    // Loop until we reach the target cell (distance == 0)
    while (dist[pascal_x][pascal_y] != 0) {
        HAL_IWDG_Refresh(&hiwdg);

        // Ask the map where the optimal path goes
        Direction_t next_dir = Floodfill_Get_Best_Dir(pascal_x, pascal_y, pascal_dir);

        // 3. TURN IF NECESSARY
        if (next_dir != pascal_dir) {
            int turn_diff = (next_dir - pascal_dir + 4) % 4;

            if (turn_diff == 1) { // RIGHT
                __disable_irq(); pascal.target_heading -= 90.0f; pascal.state = STATE_TURN_RIGHT; __enable_irq();
                while(pascal.state != STATE_IDLE) { HAL_IWDG_Refresh(&hiwdg); HAL_Delay(1); }
                Update_Compass_TurnRight();
            }
            else if (turn_diff == 2) { // U-TURN
                __disable_irq(); pascal.target_heading -= 180.0f; pascal.state = STATE_TURN_RIGHT; __enable_irq();
                while(pascal.state != STATE_IDLE) { HAL_IWDG_Refresh(&hiwdg); HAL_Delay(1); }
                Update_Compass_TurnAround();
            }
            else if (turn_diff == 3) { // LEFT
                __disable_irq(); pascal.target_heading += 90.0f; pascal.state = STATE_TURN_LEFT; __enable_irq();
                while(pascal.state != STATE_IDLE) { HAL_IWDG_Refresh(&hiwdg); HAL_Delay(1); }
                Update_Compass_TurnLeft();
            }

            Safe_Delay(50); // Let the bot settle after a fast turn
            continue; // Go back to the top of the loop to look ahead again
        }

        // 4. THE LOOK-AHEAD (How many cells can we go straight?)
        int straight_cells = 0;
        int8_t sim_x = pascal_x;
        int8_t sim_y = pascal_y;

        // Simulate walking forward in memory until we hit the goal or have to turn
        while (dist[sim_x][sim_y] != 0) {
            Direction_t sim_next_dir = Floodfill_Get_Best_Dir(sim_x, sim_y, pascal_dir);
            if (sim_next_dir == pascal_dir) {
                straight_cells++;
                // Move our "simulation" coordinates forward
                if (pascal_dir == NORTH) sim_y++;
                else if (pascal_dir == EAST) sim_x++;
                else if (pascal_dir == SOUTH) sim_y--;
                else if (pascal_dir == WEST) sim_x--;
            } else {
                break; // We found a turn in the future, stop looking ahead
            }
        }

        // 5. BLAST FORWARD CONTINUOUSLY
        if (straight_cells > 0) {
            __disable_irq();

            // EXACTLY 200mm per cell as requested!
            // If it sees 3 straight cells, it adds 600mm to the target!
            pascal.target_distance_mm = pascal.current_distance_mm + ((float)straight_cells * 200.0f);
            pascal.state = STATE_STRAIGHT;
            __enable_irq();

            while(pascal.state != STATE_IDLE) {
                HAL_IWDG_Refresh(&hiwdg);
                IR_Apply_Calibration(); // Keep PID steering active

                // Emergency crash protection (Just in case he drifts at high speed)
                if (my_mouse_sensors.left_fwd_cal > FRONT_CRASH_THRESHOLD ||
                    my_mouse_sensors.right_fwd_cal > FRONT_CRASH_THRESHOLD) {

                    __disable_irq(); pascal.state = STATE_IDLE; __enable_irq();
                    printf("SPEED RUN EMERGENCY BRAKE!\n");
                    break;
                }
                HAL_Delay(1);
            }

            // Update physical coordinates by the number of cells we just drove
            for (int i = 0; i < straight_cells; i++) {
                Update_Position_Forward();
            }
        }
    }
}
