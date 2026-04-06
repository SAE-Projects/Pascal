#ifndef UTILS_H
#define UTILS_H

#include "stm32g4xx_hal.h"

/* ADC Utility Functions */
uint16_t read_adc(ADC_HandleTypeDef *hadc, uint32_t channel);
uint16_t read_adc1_channel(uint32_t channel);
uint16_t read_adc2_channel(uint32_t channel);

#endif
