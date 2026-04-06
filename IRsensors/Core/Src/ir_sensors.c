#include "ir_sensors.h"
#include "utils.h"

/* Global data structures */
IR_SensorData_t my_mouse_sensors = {0};
volatile uint8_t is_dynamic_calibrating = 0; // 1 = Record peaks, 0 = Normal mode

IR_Calib_t calibration_data = {
    .min = {100, 100, 100, 100, 100, 100},
    .max = {2500, 2500, 2500, 2500, 2500, 2500}
};

/* --- Multiplexing Engine Setup --- */

typedef struct {
    GPIO_TypeDef* emitter_port;
    uint16_t      emitter_pin;
    uint32_t      adc_channel;
    volatile uint16_t* dest_value;
    uint8_t       index;
} IR_Config_t;

// Mapping: PB2, PB6, PB5, PB13, PB10, PB15
static const IR_Config_t ir_map[6] = {
    {GPIOB, GPIO_PIN_2,  ADC_CHANNEL_15, &my_mouse_sensors.left,       0},
    {GPIOB, GPIO_PIN_6,  ADC_CHANNEL_12, &my_mouse_sensors.left_diag,  1},
    {GPIOB, GPIO_PIN_5,  ADC_CHANNEL_4,  &my_mouse_sensors.left_fwd,   2},
    {GPIOB, GPIO_PIN_13, ADC_CHANNEL_14, &my_mouse_sensors.right_fwd,  3},
    {GPIOB, GPIO_PIN_10, ADC_CHANNEL_11, &my_mouse_sensors.right_diag, 4},
    {GPIOB, GPIO_PIN_15, ADC_CHANNEL_5,  &my_mouse_sensors.right,      5}
};

static uint8_t current_sensor = 0;

static void delay_us(uint32_t us) {
    uint32_t count = us * (SystemCoreClock / 10000000);
    for(volatile uint32_t i = 0; i < count; i++) {
        __NOP();
    }
}

/**
 * @brief background ISR triggered by TIM6 every 1ms
 */
void IR_Sequence_ISR(void)
{
    uint8_t prev_sensor = (current_sensor == 0) ? 5 : (current_sensor - 1);

    // 1. Turn OFF previous Emitter
    HAL_GPIO_WritePin(ir_map[prev_sensor].emitter_port, ir_map[prev_sensor].emitter_pin, GPIO_PIN_RESET);

    // 2. Turn ON current Emitter
    HAL_GPIO_WritePin(ir_map[current_sensor].emitter_port, ir_map[current_sensor].emitter_pin, GPIO_PIN_SET);

    // 3. Physical stabilization delay
    delay_us(30);

    // 4. Read raw ADC value
    uint16_t raw_val = read_adc1_channel(ir_map[current_sensor].adc_channel);
    *(ir_map[current_sensor].dest_value) = raw_val;

    // 5. DYNAMIC CALIBRATION: Record peak if mode is active
    if (is_dynamic_calibrating) {
        if (raw_val > calibration_data.max[current_sensor]) {
            calibration_data.max[current_sensor] = raw_val;
        }
    }

    // 6. Iterate index
    current_sensor++;
    if (current_sensor >= 6) current_sensor = 0;
}

/* --- Calibration Logic --- */

void IR_Reset_Calibration_Peaks(void) {
    for(int i = 0; i < 6; i++) {
        calibration_data.max[i] = 0;
    }
}

void IR_Set_Min_Baselines(uint16_t ambient_val) {
    for(int i = 0; i < 6; i++) {
        calibration_data.min[i] = ambient_val;
    }
}

void IR_Calibrate_Starting_Cell(void) {
    uint32_t sums[6] = {0};
    const int num_samples = 100;

    for(int i = 0; i < num_samples; i++) {
        sums[0] += my_mouse_sensors.left;
        sums[1] += my_mouse_sensors.left_diag;
        sums[2] += my_mouse_sensors.left_fwd;
        sums[3] += my_mouse_sensors.right_fwd;
        sums[4] += my_mouse_sensors.right_diag;
        sums[5] += my_mouse_sensors.right;
        HAL_Delay(2);
    }

    for(int i = 0; i < 6; i++) {
        calibration_data.max[i] = sums[i] / num_samples;
        calibration_data.min[i] = 100; // Default ambient
    }
}

static uint16_t map_value(uint16_t val, uint16_t min, uint16_t max) {
    if (val <= min) return 0;
    if (val >= max) return 1000;
    return (uint32_t)(val - min) * 1000 / (max - min);
}

void IR_Apply_Calibration(void) {
    my_mouse_sensors.left_cal       = map_value(my_mouse_sensors.left,       calibration_data.min[0], calibration_data.max[0]);
    my_mouse_sensors.left_diag_cal  = map_value(my_mouse_sensors.left_diag,  calibration_data.min[1], calibration_data.max[1]);
    my_mouse_sensors.left_fwd_cal   = map_value(my_mouse_sensors.left_fwd,   calibration_data.min[2], calibration_data.max[2]);
    my_mouse_sensors.right_fwd_cal  = map_value(my_mouse_sensors.right_fwd,  calibration_data.min[3], calibration_data.max[3]);
    my_mouse_sensors.right_diag_cal = map_value(my_mouse_sensors.right_diag, calibration_data.min[4], calibration_data.max[4]);
    my_mouse_sensors.right_cal      = map_value(my_mouse_sensors.right,      calibration_data.min[5], calibration_data.max[5]);
}
