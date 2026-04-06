#include "utils.h"

/* These handles are created in main.c by CubeMX */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;


/* Generic ADC function */
uint16_t read_adc(ADC_HandleTypeDef *hadc, uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    HAL_ADC_ConfigChannel(hadc, &sConfig);

    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);

    uint16_t value = HAL_ADC_GetValue(hadc);

    HAL_ADC_Stop(hadc);

    return value;
}


/* ADC1 wrapper */
uint16_t read_adc1_channel(uint32_t channel)
{
    return read_adc(&hadc1, channel);
}


/* ADC2 wrapper */
uint16_t read_adc2_channel(uint32_t channel)
{
    return read_adc(&hadc2, channel);
}
