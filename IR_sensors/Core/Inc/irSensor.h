#ifndef IRSENSOR_H
#define IRSENSOR_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

// Sensor indices
#define IR_LEFT_FWD   0    // PA3  ADC1_IN4
#define IR_LEFT       1    // PB0  ADC1_IN15
#define IR_LEFT_DIAG  2    // PB1  ADC1_IN12
#define IR_RIGHT_FWD  3    // PB11 ADC1_IN14
#define IR_RIGHT_DIAG 4    // PB12 ADC1_IN11
#define IR_RIGHT      5    // PB14 ADC1_IN5

// Readable struct to access all readings at once
typedef struct {
    uint16_t left_fwd;
    uint16_t left;
    uint16_t left_diag;
    uint16_t right_fwd;
    uint16_t right_diag;
    uint16_t right;
} IR_Readings;

// Call once in main() before starting
void IR_Init(ADC_HandleTypeDef *hadc);

// Call once in main() to start scanning
void IR_Start(TIM_HandleTypeDef *htim);

// Call this inside HAL_TIM_PeriodElapsedCallback
void IR_TimerCallback(TIM_HandleTypeDef *htim);

// Get a snapshot of all current readings
IR_Readings IR_GetReadings(void);

// Get a single sensor reading by index
uint16_t IR_GetSingle(uint8_t sensor_index);

#endif // IR_SENSORS_H
