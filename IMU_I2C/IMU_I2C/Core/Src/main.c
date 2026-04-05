/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ISM330DHCX IMU on STM32G431CBU6
  *                   I2C1: SCL=PA15  SDA=PB7
  *                   UART2: TX=PA2   RX=PA3   115200 baud
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ism330dhcx_i2c.h"
#include <stdio.h>
#include <math.h>
#include "motor.h"
#include <stdlib.h>
#include "encoder.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LPF_ALPHA           0.15f   /* low-pass filter strength 0.0–1.0   */
#define GYRO_DEADBAND_DPS   0.3f    /* gz below this treated as zero      */
#define DEBUG_PRINT_MS      20u    /* UART print interval in ms          */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
ISM330_Raw_t      imu_raw;
ISM330_Phys_t     imu_phys;
ISM330_Filtered_t imu_filtered;

float    heading_deg = 0.0f;
uint32_t last_tick   = 0u;
uint32_t last_print  = 0u;

/* Gyro bias — measured at startup, subtracted every read */
float gyro_bias_x = 0.0f;
float gyro_bias_y = 0.0f;
float gyro_bias_z = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

void IMU_Init(void);
void IMU_CalibrateGyro(void);
void IMU_Read(void);
void IMU_UpdateHeading(void);
void IMU_PrintDebug(void);
int  _write(int fd, char *ptr, int len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE BEGIN 0 */

/* ARM Cortex-M4 DWT (Data Watchpoint and Trace) Registers */

volatile uint32_t *DWT_CONTROL = (volatile uint32_t *)0xE0001000;
volatile uint32_t *DWT_CYCCNT  = (volatile uint32_t *)0xE0001004;
volatile uint32_t *SCB_DEMCR   = (volatile uint32_t *)0xE000EDFC;

/* Initializes the CPU cycle counter */
void DWT_Init(void) {
    *SCB_DEMCR   |= 0x01000000;  // Enable Trace
    *DWT_CYCCNT   = 0;           // Reset counter
    *DWT_CONTROL |= 1;           // Enable counter
}

/* Returns exact microseconds passed since boot */
uint32_t DWT_GetMicros(void) {
    // SystemCoreClock is automatically updated by HAL (170MHz on your G4)
    return *DWT_CYCCNT / (SystemCoreClock / 1000000U);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  // 0. Start the microsecond timer
  DWT_Init();

  /* 1. STBY HIGH */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
  printf("STBY PB4 = HIGH\r\n");

  /* 2. Init IMU */
  IMU_Init();

  /* 3. Init motors */
  Motor_Init(&htim1);
  Encoder_Init();
  /* 4. Calibrate gyro — keep bot STILL */
  IMU_CalibrateGyro();

  // Initialize your PID
  PID_Init(&wall_pid, 0.5f, 0.1f);

  // START THE TIMER INTERRUPT
  HAL_TIM_Base_Start_IT(&htim6);

  last_tick  = DWT_GetMicros();   /* seed with microseconds — matches IMU_UpdateHeading */

  /* 5. Seed timers */
  last_print = HAL_GetTick();

  printf("Ready — starting in 1 second...\r\n");
  HAL_Delay(1000);
  printf("--- RAW ELECTRICAL PIN TEST ---\r\n");
  printf("Spin the wheels EXTREMELY SLOWLY by hand.\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
//	  /* ========================================================
//	           PASCAL: CONTINUOUS CURVE & DIAGONAL TEST SEQUENCE
//	           ======================================================== */
//
//	        int32_t run_speed = 400; /* Set your global run speed here */
//
//	        printf("\r\n[MAIN] Starting Continuous Curve Sequence in 3s...\r\n");
//	        HAL_Delay(3000);
//
//	        // 1. Drive straight (Entering the sequence)
//	        // Driving forward for 600ms at run_speed
//	        Motor_DriveStraight_HDG(600, run_speed, &heading_deg, IMU_Read, IMU_UpdateHeading);
//
//	        // 2. Smooth 90-Degree Right Turn
//	        // Ratio 0.4: Inner wheel runs at 40% speed for a nice, tight corner.
//	        Motor_Curve90R_Continuous(run_speed, 0.4f, &heading_deg, IMU_Read, IMU_UpdateHeading);
//
//	        // 3. Drive straight for a short orthogonal segment
//	        Motor_DriveStraight_HDG(400, run_speed, &heading_deg, IMU_Read, IMU_UpdateHeading);
//
//	        // 4. Smooth 45-Degree Left Turn (Entering a diagonal path)
//	        // Ratio 0.6: 45-degree turns need a shallower, wider arc so you don't lose speed.
//	        Motor_Curve45L_Continuous(run_speed, 0.6f, &heading_deg, IMU_Read, IMU_UpdateHeading);
//
//	        // 5. Drive straight along the diagonal
//	        Motor_DriveStraight_HDG(800, run_speed, &heading_deg, IMU_Read, IMU_UpdateHeading);
//
//	        // 6. Smooth 45-Degree Right Turn (Exiting the diagonal back to orthogonal)
//	        Motor_Curve45R_Continuous(run_speed, 0.6f, &heading_deg, IMU_Read, IMU_UpdateHeading);
//
//	        // 7. Final straight away
//	        Motor_DriveStraight_HDG(600, run_speed, &heading_deg, IMU_Read, IMU_UpdateHeading);
//
//	        // 8. Hard Stop
//	        Motor_Stop();
//	        printf("[MAIN] Sequence Complete. Holding position.\r\n");
//
//	        // Trap the robot here in an infinite loop so it doesn't repeat
//	        // the sequence and fly off your desk! Press reset to run again.
//	        while (1)
//	        {
//	            HAL_Delay(100);
//	        }
//
//
//
//





//	  // 1. Drive straight towards the corner
//	  Motor_DriveStraight_HDG(500, 300, &heading_deg, IMU_Read, IMU_UpdateHeading);
//
//	  // 2. We hit the corner! Seamlessly carve a 90-degree right turn without stopping.
//	  // Speed remains 300, inner wheel drops to 150 (ratio 0.5) to swing around.
//	  Motor_Curve90R_Continuous(500, 0.5f, &heading_deg, IMU_Read, IMU_UpdateHeading);
//
//	  // 3. The curve function automatically sets motors back to 300 when finished.
//	  // Now you can immediately drop back into a DriveStraight or maze following function.
//	  Motor_DriveStraight_HDG(500, 300, &heading_deg, IMU_Read, IMU_UpdateHeading);






//	  /* Test Sequence */
//	    printf("Starting Distance Test in 2 seconds...\r\n");
//	    HAL_Delay(2000);
//
//	    // Drive exactly 180mm forward (Standard Micromouse cell size)
//	    Motor_DriveDistance_HDG(1000.0f, 500, &heading_deg, IMU_Read, IMU_UpdateHeading);
//
//	    HAL_Delay(1000);
//
//	    // Drive exactly 180mm backward to the starting point!
//	    Motor_DriveDistance_HDG(-1000.0f, 500, &heading_deg, IMU_Read, IMU_UpdateHeading);
//

//	  printf("--- PURE HARDWARE TEST ---\r\n");
//	  Encoder_Update();
////
//	  printf("%f %f \r\n",Encoder_GetDistanceLeft(),Encoder_GetDistanceRight());
////
//	  HAL_Delay(50);

//	          // 1. Drive Forward for 1 second at half speed (PWM 500)
//	          Motor_Forward(1000, 500);

//	           2. Turn 45 Degrees Right
//	          Motor_Turn45R_HDG(&heading_deg, IMU_Read, IMU_UpdateHeading);
//	          HAL_Delay(100);
//
////	           2. Turn 45 Degrees Left
//	          Motor_Turn45L_HDG(&heading_deg, IMU_Read, IMU_UpdateHeading);
//	          HAL_Delay(100);
//
//
//	          // 2. Turn 90 Degrees Right
//	          Motor_Turn90R_HDG(&heading_deg, IMU_Read, IMU_UpdateHeading);
//	          HAL_Delay(300);
//
//	          // 4. Turn 90 Degrees Left
//	          Motor_Turn90L_HDG(&heading_deg, IMU_Read, IMU_UpdateHeading);
//	          HAL_Delay(300);

	          // 5. Back up for half a second at slower speed
//	          Motor_Backward(500, 300);
//	          HAL_Delay(300);

//	          Motor_Turn180R_HDG(&heading_deg, IMU_Read, IMU_UpdateHeading);
//	          HAL_Delay(300); // Wait 3 seconds before repeating the whole loop
//
//              Motor_Turn180L_HDG(&heading_deg, IMU_Read, IMU_UpdateHeading);
//	          HAL_Delay(300); // Wait 3 seconds before repeating the whole loop
//
//	          Motor_Turn360R_HDG(&heading_deg, IMU_Read, IMU_UpdateHeading);
//              HAL_Delay(300);
//
//	  	      Motor_Turn360L_HDG(&heading_deg, IMU_Read, IMU_UpdateHeading);
//	  	      HAL_Delay(3000);

//	          Motor_DriveStraight_HDG(500, 300, &heading_deg, IMU_Read, IMU_UpdateHeading);

//	          Motor_DriveDistance_HDG(500.0f, 1000, &heading_deg, IMU_Read, IMU_UpdateHeading);
//	          HAL_Delay(500);

//
//	          Motor_Turn90R_HDG(&heading_deg, IMU_Read, IMU_UpdateHeading);
//	          HAL_Delay(300);
//	          Motor_Turn90R_HDG(&heading_deg, IMU_Read, IMU_UpdateHeading);
//	          HAL_Delay(300);



//
//	          Motor_DriveDistance_HDG(-500.0f, 1000, &heading_deg, IMU_Read, IMU_UpdateHeading);
//	          HAL_Delay(500);



//	  /* Test 90° right turn — runs once then holds */
//	     Motor_Turn90R(&heading_deg, IMU_Read, IMU_UpdateHeading);
//
//	     /* Hold here after turn — keep reading IMU so HDG stays fresh */
//	     printf("Turn complete. Holding...\r\n");
//	     while (1)
//	     {
//	         IMU_Read();
//	         IMU_UpdateHeading();
//	         IMU_PrintDebug();
//	     }
//


//    IMU_Read();          /* wait for data-ready, burst-read, filter */
//    IMU_UpdateHeading(); /* integrate gz → heading angle            */
//    IMU_PrintDebug();    /* print over UART every 200 ms            */

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_17;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x40621236;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 169;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 170-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|BIN2_Pin|STBY_Pin|GPIO_PIN_5
                          |GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN1_Pin|AIN2_Pin|BIN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB2 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN1_Pin AIN2_Pin BIN1_Pin */
  GPIO_InitStruct.Pin = AIN1_Pin|AIN2_Pin|BIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BIN2_Pin STBY_Pin */
  GPIO_InitStruct.Pin = BIN2_Pin|STBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        /* --- 1. READ SENSORS --- */
        HAL_ADC_Start(&hadc1);

        if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) {
            left_ir_val = HAL_ADC_GetValue(&hadc1);
        }
        if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) {
            right_ir_val = HAL_ADC_GetValue(&hadc1);
        }

        HAL_ADC_Stop(&hadc1);

        /* --- 2. CALCULATE PID --- */
        int error = right_ir_val - left_ir_val;

        // Let the clean module do the math!
        float correction = PID_Calculate(&wall_pid, error);

        /* --- 3. APPLY TO MOTORS --- */
        int left_pwm  = base_speed + (int)correction;
        int right_pwm = base_speed - (int)correction;

        if (left_pwm > 1000) left_pwm = 1000;
        if (left_pwm < 0) left_pwm = 0;
        if (right_pwm > 1000) right_pwm = 1000;
        if (right_pwm < 0) right_pwm = 0;

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, left_pwm);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, right_pwm);
    }
}


/* IMU_Init — verify sensor presence, configure ODR and FS.
   Hangs in Error_Handler if sensor not found.
   Address is 0x6B (confirmed by I2C scanner). */
void IMU_Init(void)
{
    ISM330_Status_t status = ISM330_Init(&hi2c1);

    if (status == ISM330_ERR_I2C)
    {
        printf("[IMU] ERROR: I2C no response. Check wiring.\r\n");
        Error_Handler();
    }
    if (status == ISM330_ERR_ID)
    {
        printf("[IMU] ERROR: wrong WHO_AM_I. Check address in .h\r\n");
        Error_Handler();
    }

    ISM330_LPF_Init(&imu_filtered, LPF_ALPHA);
    printf("[IMU] ISM330DHCX ready. ODR=416Hz FS=2g/250dps\r\n");
}

/* IMU_Read — wait for fresh data, read 14 bytes, convert, filter. */
/* IMU_Read — wait for fresh data, read 14 bytes, convert, apply bias. */
void IMU_Read(void)
{
    ISM330_Status_t status = ISM330_WaitAndRead(&hi2c1, &imu_raw);
    if (status != ISM330_OK) { return; }

    ISM330_ToPhysical(&imu_raw, &imu_phys);

    /* Subtract calibration bias (calculated at startup) */
    imu_phys.gx -= gyro_bias_x;
    imu_phys.gy -= gyro_bias_y;
    imu_phys.gz -= gyro_bias_z;

    /* Negate gz — sensor Z axis is upside down on this bot mounting */
    imu_phys.gz = -imu_phys.gz;
}

/* IMU_UpdateHeading — integrate filtered gz over time → heading angle.
   Deadband removes slow drift when mouse is stationary. */
/* IMU_UpdateHeading — integrate hardware-filtered gz over time → heading angle. */
void IMU_UpdateHeading(void)
{
    static float prev_gz = 0.0f;

    uint32_t now = DWT_GetMicros();

    /* Optimized: Multiply by 0.000001 instead of dividing by 1,000,000 */
    float dt_sec = (float)(now - last_tick) * 0.000001f;
    last_tick = now;

    /* Failsafe: If MCU hangs, don't integrate a massive time gap */
    if (dt_sec > 0.1f) { return; }

    /* Grab the beautifully hardware-filtered raw value */
    float gz = imu_phys.gz;

    /* Deadband: Ignore tiny sensor noise to stop stationary drift.
       Keep this small (e.g., 0.3f) so it doesn't blind the PID! */
    if (gz > -0.3f && gz < 0.3f) {
        gz = 0.0f;
    }

    /* --- GYRO SCALE FACTOR FIX --- */
    /* Adjust these multipliers if your 90-degree turns are slightly off */
    float gyro_scale_right = 1.000f;  //0.9875f
    float gyro_scale_left  = 1.000f;  //1.003f

    if (gz > 0.0f) {
        gz *= gyro_scale_right;
    } else {
        gz *= gyro_scale_left;
    }

    /* Trapezoidal Integration (Optimized: * 0.5f instead of / 2.0f) */
    heading_deg += (gz + prev_gz) * 0.5f * dt_sec;

    prev_gz = gz;
}
/* IMU_PrintDebug — send one UART line every DEBUG_PRINT_MS ms.
   Expected when sitting flat: AZ≈1.000g  GZ≈0.00  HDG≈0.0
   Remove before competition — printf slows the loop. */
void IMU_PrintDebug(void)
{
    if ((HAL_GetTick() - last_print) < DEBUG_PRINT_MS) { return; }
    last_print = HAL_GetTick();

    printf(
        "AX:%6.3f AY:%6.3f AZ:%6.3f g | "
        "GX:%7.2f GY:%7.2f GZ:%7.2f dps | "
        "HDG:%6.1f deg\r\n",
        (double)imu_phys.ax, (double)imu_phys.ay, (double)imu_phys.az,
        (double)imu_phys.gx, (double)imu_phys.gy, (double)imu_phys.gz,
        (double)heading_deg
    );
}

/* _write — redirects printf to USART2 (PA2 TX pin). */
int _write(int fd, char *ptr, int len)
{
    (void)fd;
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, (uint16_t)len, 100u);
    return len;
}

/* Gyro calibration — call once at startup, mouse must be STILL.
   Reads 200 samples and averages them → that average IS the bias.
   All subsequent readings subtract this offset. */
void IMU_CalibrateGyro(void)
{
    printf("[IMU] Calibrating gyro — keep mouse still...\r\n");

    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    int   samples = 200;

    for (int i = 0; i < samples; i++)
    {
        ISM330_WaitAndRead(&hi2c1, &imu_raw);
        ISM330_ToPhysical(&imu_raw, &imu_phys);
        sum_x += imu_phys.gx;
        sum_y += imu_phys.gy;
        sum_z += imu_phys.gz;
        HAL_Delay(5);
    }

    gyro_bias_x = sum_x / (float)samples;
    gyro_bias_y = sum_y / (float)samples;
    gyro_bias_z = sum_z / (float)samples;

    printf("[IMU] Bias: GX=%.4f GY=%.4f GZ=%.4f dps\r\n",
           (double)gyro_bias_x,
           (double)gyro_bias_y,
           (double)gyro_bias_z);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
