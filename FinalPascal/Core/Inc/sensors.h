#ifndef SENSORS_H
#define SENSORS_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

/* Structure to hold all 6 sensor readings at once */
typedef struct {
    volatile uint16_t left, left_diag, left_fwd;
    volatile uint16_t right_fwd, right_diag, right;

    uint16_t left_cal, left_diag_cal, left_fwd_cal;
    uint16_t right_fwd_cal, right_diag_cal, right_cal;
} IR_SensorData_t;

typedef struct {
    uint16_t min[6];
    uint16_t max[6];
} IR_Calib_t;

extern volatile IR_SensorData_t my_mouse_sensors;
extern volatile uint8_t is_dynamic_calibrating;

void Sensors_Init(void);
void Sensors_Update_ISR(void);            // Call this in TIM6
void IR_Apply_Calibration(void);          // Call this in main while(1)
void IR_Print_Calibration_Data(void);

void IR_Start_Dynamic_Calibration(void);
void IR_Stop_Dynamic_Calibration(void);

void Sensors_Capture_Center_Targets(void);
float Sensors_Get_Steering_Error(void);

#endif /* SENSORS_H */
