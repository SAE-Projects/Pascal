#ifndef IR_SENSORS_H
#define IR_SENSORS_H

#include <stdint.h>
#include "main.h"

/* Structure to hold all 6 sensor readings at once */
typedef struct {
    // Raw ADC values (0-4095)
    volatile uint16_t left, left_diag, left_fwd;
    volatile uint16_t right_fwd, right_diag, right;

    // Calibrated values (0-1000)
    uint16_t left_cal, left_diag_cal, left_fwd_cal;
    uint16_t right_fwd_cal, right_diag_cal, right_cal;
} IR_SensorData_t;

/* Struct to hold calibration constants */
typedef struct {
    uint16_t min[6];
    uint16_t max[6];
} IR_Calib_t;

/* Global variables accessible in main.c */
extern IR_SensorData_t my_mouse_sensors;
extern IR_Calib_t calibration_data;
extern volatile uint8_t is_dynamic_calibrating;

/* Function Prototypes */
void IR_Sequence_ISR(void);             // Call in TIM6 Callback
void IR_Apply_Calibration(void);        // Call in While(1)
void IR_Calibrate_Starting_Cell(void);  // Static calibration
void IR_Reset_Calibration_Peaks(void);  // Prepare for dance
void IR_Set_Min_Baselines(uint16_t ambient_val);

#endif /* IR_SENSORS_H */
