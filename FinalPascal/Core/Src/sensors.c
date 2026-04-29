#include "sensors.h"
#include <stdio.h> // Added for the calibration print report

extern ADC_HandleTypeDef hadc1;

volatile IR_SensorData_t my_mouse_sensors = {0};
volatile uint8_t is_dynamic_calibrating = 0;

IR_Calib_t calibration_data = {
    .min = {305,  590,  430,  585,  975,  700},
    .max = {3855, 3830, 3820, 3780, 3800, 3850}
};

typedef struct {
    GPIO_TypeDef* emitter_port;
    uint16_t      emitter_pin;
    uint32_t      adc_channel;
    volatile uint16_t* dest_value;
} IR_Config_t;

// Your excellent hardware map
static const IR_Config_t ir_map[6] = {
    {GPIOB, GPIO_PIN_2,  ADC_CHANNEL_15, &my_mouse_sensors.left},
    {GPIOB, GPIO_PIN_6,  ADC_CHANNEL_12, &my_mouse_sensors.left_diag},
    {GPIOB, GPIO_PIN_5,  ADC_CHANNEL_4,  &my_mouse_sensors.left_fwd},
    {GPIOB, GPIO_PIN_13, ADC_CHANNEL_14, &my_mouse_sensors.right_fwd},
    {GPIOB, GPIO_PIN_10, ADC_CHANNEL_11, &my_mouse_sensors.right_diag},
    {GPIOB, GPIO_PIN_15, ADC_CHANNEL_5,  &my_mouse_sensors.right}
};

static uint8_t current_sensor = 0;

/* Helper to safely switch ADC channels */
static void ADC_Select_Channel(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

void Sensors_Init(void) {
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

    // FIX: Removed the "Pump Priming" code!
    // We no longer turn on the first LED here. Pascal boots up completely dark.
    // The state machine in Sensors_Update_ISR is smart enough to handle
    // the very first timer tick without needing a head start.
    current_sensor = 0;
}

/* ================================================================
   THE NON-BLOCKING SENSOR STATE MACHINE (Runs in TIM6)
   ================================================================ */
void Sensors_Update_ISR(void) {

    // 1. READ PREVIOUS CONVERSION
    // The ADC was started 1ms ago during the last interrupt. It is definitely finished.
    // Timeout of 0 means "grab it if it's there, but don't ever wait".
    if (HAL_ADC_PollForConversion(&hadc1, 0) == HAL_OK) {

        uint16_t raw_hardware = HAL_ADC_GetValue(&hadc1);
        uint16_t inverted_raw = 4095 - raw_hardware; // Active-Low Math!

        // LOW PASS FILTER (Alpha = 0.3) to destroy noise!
        uint16_t old_val = *(ir_map[current_sensor].dest_value);
        uint16_t filtered_val = (uint16_t)((0.6f * inverted_raw) + (0.4f * old_val));

        *(ir_map[current_sensor].dest_value) = filtered_val;

        // Dynamic Calibration
        if (is_dynamic_calibrating) {
            if (filtered_val > calibration_data.max[current_sensor])
                calibration_data.max[current_sensor] = filtered_val;
            if (filtered_val < calibration_data.min[current_sensor])
                calibration_data.min[current_sensor] = filtered_val;
        }
    }

    // 2. TURN OFF CURRENT EMITTER
    HAL_GPIO_WritePin(ir_map[current_sensor].emitter_port, ir_map[current_sensor].emitter_pin, GPIO_PIN_RESET);

    // 3. MOVE TO NEXT SENSOR
    current_sensor++;
    if (current_sensor >= 6) current_sensor = 0;

    // 4. PREPARE NEXT CONVERSION
    ADC_Select_Channel(ir_map[current_sensor].adc_channel);
    HAL_GPIO_WritePin(ir_map[current_sensor].emitter_port, ir_map[current_sensor].emitter_pin, GPIO_PIN_SET);

    // 5. START ADC (We will read this data on the next 1ms tick!)
    HAL_ADC_Start(&hadc1);
}

/* ================================================================
   CALIBRATION MAPPING
   ================================================================ */
void IR_Start_Dynamic_Calibration(void) {
    for(int i = 0; i < 6; i++) {
        calibration_data.min[i] = 4095;
        calibration_data.max[i] = 0;
    }
    is_dynamic_calibrating = 1;
}

void IR_Stop_Dynamic_Calibration(void) {
    is_dynamic_calibrating = 0;
}

static uint16_t map_value(uint16_t val, uint16_t min, uint16_t max) {
    // FIX: The Safety Net!
    // If the difference is tiny, it's just electrical noise or open air. Default to 0.
    if ((max - min) < 50) return 0;

    if (max <= min) return 0;
    if (val <= min) return 0;
    if (val >= max) return 1000;
    return (uint32_t)(val - min) * 1000 / (max - min);
}

// Call this in your slow while(1) loop before making navigation decisions
void IR_Apply_Calibration(void) {
    my_mouse_sensors.left_cal       = map_value(my_mouse_sensors.left,       calibration_data.min[0], calibration_data.max[0]);
    my_mouse_sensors.left_diag_cal  = map_value(my_mouse_sensors.left_diag,  calibration_data.min[1], calibration_data.max[1]);
    my_mouse_sensors.left_fwd_cal   = map_value(my_mouse_sensors.left_fwd,   calibration_data.min[2], calibration_data.max[2]);
    my_mouse_sensors.right_fwd_cal  = map_value(my_mouse_sensors.right_fwd,  calibration_data.min[3], calibration_data.max[3]);
    my_mouse_sensors.right_diag_cal = map_value(my_mouse_sensors.right_diag, calibration_data.min[4], calibration_data.max[4]);
    my_mouse_sensors.right_cal      = map_value(my_mouse_sensors.right,      calibration_data.min[5], calibration_data.max[5]);
}

/* ================================================================
   DIAGNOSTICS & DEBUGGING
   ================================================================ */
void IR_Print_Calibration_Data(void) {
    printf("\n--- CALIBRATION RESULTS ---\r\n");
    printf("SENSOR     |  MIN  |  MAX  | DIFFERENCE\r\n");
    printf("---------------------------------------\r\n");

    // Names aligned with the ir_map array order
    const char* names[] = {"LEFT", "LEFT_DIAG", "LEFT_FWD", "RIGHT_FWD", "RIGHT_DIAG", "RIGHT"};

    for(int i=0; i<6; i++) {
        uint16_t diff = calibration_data.max[i] - calibration_data.min[i];
        printf("%-10s | %4d  | %4d  | %4d", names[i], calibration_data.min[i], calibration_data.max[i], diff);

        if (diff < 50) {
            printf(" <-- FAILED SAFETY NET!\r\n");
        } else {
            printf(" <-- GOOD\r\n");
        }
    }
    printf("---------------------------------------\r\n\n");
}

/* ================================================================
   DYNAMIC IR PID STEERING LOGIC
   ================================================================ */

// Global variables to hold the captured center targets
float dynamic_target_left = 400.0f;  // Safe default
float dynamic_target_right = 400.0f; // Safe default

#define SIDE_WALL_THRESH 300

// The Snapshot Function
void Sensors_Capture_Center_Targets(void) {
    // 1. Force an update of the 0-1000 mapped values
    IR_Apply_Calibration();

    // 2. Overwrite the targets with the current physical reality
    dynamic_target_left = (float)my_mouse_sensors.left_cal;
    dynamic_target_right = (float)my_mouse_sensors.right_cal;

    printf("CENTER LOCKED! Left Target: %.1f | Right Target: %.1f\r\n",
           dynamic_target_left, dynamic_target_right);
}

// Your updated Error Function using the dynamic variables
float Sensors_Get_Steering_Error(void) {
    float error = 0.0f;
    IR_Apply_Calibration();

    uint8_t wall_left = (my_mouse_sensors.left_cal > SIDE_WALL_THRESH);
    uint8_t wall_right = (my_mouse_sensors.right_cal > SIDE_WALL_THRESH);

    if (wall_left && wall_right) {
        // Use the dynamically captured targets!
        float left_error = my_mouse_sensors.left_cal - dynamic_target_left;
        float right_error = my_mouse_sensors.right_cal - dynamic_target_right;

        error = left_error - right_error;
    }
    else if (wall_left) {
        error = 2.0f * (my_mouse_sensors.left_cal - dynamic_target_left);
    }
    else if (wall_right) {
        error = -2.0f * (my_mouse_sensors.right_cal - dynamic_target_right);
    }
    else {
        error = 0.0f; // No walls, let the gyro drive straight
    }

    return error;
}
