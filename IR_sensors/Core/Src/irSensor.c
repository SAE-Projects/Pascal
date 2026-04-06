#include "irSensor.h"

// ── Private types ────────────────────────────────────────────────────────────

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} IR_LED;

// ── Private variables ────────────────────────────────────────────────────────

extern static ADC_HandleTypeDef *_hadc;

static IR_LED ir_leds[6] = {
    {GPIOB, GPIO_PIN_5},   // leftForward  → PB5
    {GPIOB, GPIO_PIN_2},   // left         → PB2
    {GPIOB, GPIO_PIN_6},   // leftDiagonal → PB6
    {GPIOB, GPIO_PIN_13},  // rightForward → PB13
    {GPIOB, GPIO_PIN_10},  // rightDiag    → PB10
    {GPIOB, GPIO_PIN_15},  // right        → PB15
};

static ADC_ChannelConfTypeDef ir_adc_channels[6] = {
    {ADC_CHANNEL_4,  ADC_REGULAR_RANK_1, ADC_SAMPLETIME_47CYCLES_5, 0, 0}, // PA3
    {ADC_CHANNEL_15, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_47CYCLES_5, 0, 0}, // PB0
    {ADC_CHANNEL_12, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_47CYCLES_5, 0, 0}, // PB1
    {ADC_CHANNEL_14, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_47CYCLES_5, 0, 0}, // PB11
    {ADC_CHANNEL_11, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_47CYCLES_5, 0, 0}, // PB12
    {ADC_CHANNEL_5,  ADC_REGULAR_RANK_1, ADC_SAMPLETIME_47CYCLES_5, 0, 0}, // PB14
};

static volatile uint16_t ir_readings[6];
static volatile uint8_t  ir_step = 0;

// ── Private functions ────────────────────────────────────────────────────────

static void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static void DWT_Delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}

static void LED_Off(uint8_t idx) {
    HAL_GPIO_WritePin(ir_leds[idx].port, ir_leds[idx].pin, GPIO_PIN_RESET);
}

static void LED_On(uint8_t idx) {
    HAL_GPIO_WritePin(ir_leds[idx].port, ir_leds[idx].pin, GPIO_PIN_SET);
}

static uint16_t Read_ADC(uint8_t idx) {
    HAL_ADC_ConfigChannel(_hadc, &ir_adc_channels[idx]);
    HAL_ADC_Start(_hadc);
    HAL_ADC_PollForConversion(_hadc, 1);
    uint16_t val = HAL_ADC_GetValue(_hadc);
    HAL_ADC_Stop(_hadc);
    return 4095 - val;  // invert: higher = closer
}

// ── Public functions ─────────────────────────────────────────────────────────

void IR_Init(ADC_HandleTypeDef *hadc) {
    _hadc = hadc;
    DWT_Init();

    // Make sure all LEDs start OFF
    for (int i = 0; i < 6; i++) {
        LED_Off(i);
    }
}

void IR_Start(TIM_HandleTypeDef *htim) {
    HAL_TIM_Base_Start_IT(htim);
}

void IR_TimerCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance != TIM6) return;

    // Turn off previous LED
    if (ir_step > 0) {
        LED_Off(ir_step - 1);
    }

    if (ir_step < 6) {
        LED_On(ir_step);
        DWT_Delay_us(30);
        ir_readings[ir_step] = Read_ADC(ir_step);
        ir_step++;
    } else {
        LED_Off(5);
        ir_step = 0;
    }
}

IR_Readings IR_GetReadings(void) {
    IR_Readings r;
    r.left_fwd   = ir_readings[IR_LEFT_FWD];
    r.left       = ir_readings[IR_LEFT];
    r.left_diag  = ir_readings[IR_LEFT_DIAG];
    r.right_fwd  = ir_readings[IR_RIGHT_FWD];
    r.right_diag = ir_readings[IR_RIGHT_DIAG];
    r.right      = ir_readings[IR_RIGHT];
    return r;
}

uint16_t IR_GetSingle(uint8_t sensor_index) {
    return ir_readings[sensor_index];
}
