/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/*
 * Hardware Configuration:
 * - STM32F411CEU6 @ 84MHz
 * - ADC1: Battery + 4 IR sensors (PA0,PA2,PA3,PA4,PA5)
 * - GPIO: IR emitters (PA8,PA9,PB8,PB9), LEDs (PB4,PB5), Motors (PA6,PA7,PB0,PB1)
 * - TIM3/TIM4: Encoders for precise movement
 * - USART6: Bluetooth communication (PA11,PA12)
 * - SPI2: MPU9250 gyroscope
 * - TIM1: Speaker PWM (PA10)
 * - External Interrupts: Buttons PA1, PB10
 */
#include "micromouse.h"
//#include <stdio.h>
//#include <string.h>
//#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Micromouse global variables */
MazeCell maze[MAZE_SIZE][MAZE_SIZE];
RobotState robot;
SensorData sensors;
GyroData gyro;
EncoderData encoders;
volatile uint8_t button_pressed = 0;
volatile uint8_t start_flag = 0;


/* Direction vectors for movement */
const int dx[4] = {0, 1, 0, -1}; // N, E, S, W
const int dy[4] = {1, 0, -1, 0};

/* Goal positions (center of maze)  */

const int goal_x1 = (MAZE_SIZE/2 - 1), goal_y1 = (MAZE_SIZE/2 - 1);
const int goal_x2 = (MAZE_SIZE/2), goal_y2 = (MAZE_SIZE/2);

/* Exploration state variables */
static bool system_ready = false;
static bool exploration_started = false;
static uint32_t last_status_time = 0;
static uint32_t last_blink_time = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

static void initialize_hardware_systems(void);
static void run_system_diagnostics(void);
static void send_periodic_status(void);
extern void dwt_delay_init(uint32_t cpu_hz);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/**
 * @brief Initialize all hardware systems and perform diagnostics
 */
static void initialize_hardware_systems(void) {
    //send_bluetooth_message("\r\n" "="*60 "\r\n");
    send_bluetooth_message("🤖 CHAMPIONSHIP MICROMOUSE INITIALIZATION 🤖\r\n");
    //send_bluetooth_message("="*60 "\r\n");

    // Initialize PWM for motors
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // PA6 (MOTOR_IN1)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // PA7 (MOTOR_IN2)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // PB0 (MOTOR_IN3)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // PB1 (MOTOR_IN4)
    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_SET); // Enable DRV8833

    // Verify ADC GPIO configuration
    verify_adc_gpio_configuration();
    adc_system_diagnostics();

    // Initialize and test MPU9250 gyroscope
    if (mpu9250_init()) {
        send_bluetooth_message("✅ MPU9250 gyroscope initialized successfully\r\n");
        send_bluetooth_message("⚠️ KEEP ROBOT STATIONARY during gyro calibration!\r\n");
        HAL_Delay(2000);
        mpu9250_calibrate_bias();
        send_bluetooth_message("✅ Gyro calibration complete\r\n");
    } else {
        send_bluetooth_message("⚠️ Gyro initialization failed - using basic movement\r\n");
    }

    // Initialize encoders
    start_encoders();
    HAL_Delay(100);

    // Test encoder functionality
    if (get_left_encoder_total() == 0 && get_right_encoder_total() == 0) {
        send_bluetooth_message("⚠️ WARNING: Encoders may not be working properly\r\n");
    } else {
        send_bluetooth_message("✅ Encoders initialized and responding\r\n");
    }

    // Initialize maze exploration system
    initialize_maze_exploration();

    send_bluetooth_message("✅ All systems initialized successfully!\r\n");
}

/**
 * @brief Run comprehensive system diagnostics
 */
static void run_system_diagnostics(void) {
    send_bluetooth_message("\r\n🔧 SYSTEM DIAGNOSTICS 🔧\r\n");

    // Test sensors
    update_sensors();
    if (sensors.battery == 0 && sensors.front_left == 0 &&
        sensors.front_right == 0 && sensors.side_left == 0 && sensors.side_right == 0) {
        send_bluetooth_message("❌ CRITICAL: All sensors reading zero - check connections!\r\n");
    } else {
        send_bluetooth_message("✅ Sensors responding normally\r\n");
        send_sensor_data();
    }

    // Test battery
    send_battery_status();

    // Test gyro if available
    if (mpu9250_is_initialized()) {
        mpu9250_send_status();
    }

    // Test encoders
    send_encoder_status();

    // System health check
    if (system_health_check()) {
        send_bluetooth_message("✅ System health check PASSED\r\n");
        system_ready = true;
    } else {
        send_bluetooth_message("⚠️ System health check FAILED - check warnings above\r\n");
        system_ready = false;
    }

    send_bluetooth_message("🔧 Diagnostics complete!\r\n");
}


/**
 * @brief Send periodic status updates
 */
static void send_periodic_status(void) {
    uint32_t current_time = HAL_GetTick();

    // Send status every 10 seconds when not exploring
    if (current_time - last_status_time > 10000 && !exploration_started) {
        send_battery_status();

        if (system_ready) {
            send_bluetooth_message("💚 System ready - Press LEFT button to start exploration\r\n");
        } else {
            send_bluetooth_message("🔴 System not ready - Check diagnostics\r\n");
        }

        last_status_time = current_time;
    }

    // Blink LED to show system is alive
    if (current_time - last_blink_time > 2000) {
        if (system_ready) {
            HAL_GPIO_TogglePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin);
        } else {
            // Fast blink if system not ready
            HAL_GPIO_TogglePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin);
        }
        last_blink_time = current_time;
    }
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
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();


  // main.c — after SystemClock_Config();


  dwt_delay_init(HAL_RCC_GetHCLKFreq());


  /* USER CODE BEGIN 2 */
  // Initialize all hardware systems
  initialize_hardware_systems();

  // Run system diagnostics
  //run_system_diagnostics();

  // Play startup sequence
  play_startup_tone();
  led_sequence_startup();

  // Send startup message
  send_bluetooth_message("\r\n🎮 MICROMOUSE CONTROL READY 🎮\r\n");
  send_bluetooth_message("LEFT button: Start exploration\r\n");
  send_bluetooth_message("RIGHT button: Status/Emergency stop\r\n");
  send_bluetooth_message("Waiting for user input...\r\n");

    // Initial status
  last_status_time = HAL_GetTick();
  last_blink_time = HAL_GetTick();

//  while (!start_flag) {
//      HAL_Delay(10);
//      HAL_GPIO_TogglePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin); // Blink indicator
//  }


    // run a left-turn step test
  //r// base, delta, delay_before_step(ms), step_duration(ms), sample_ms, total_ms
//  run_gyro_step_test(700, 200, 1000, 2000, 5, 7000);   // +delta (turn one way)
//  HAL_Delay(1500);
//  run_gyro_step_test(700,-200, 1000, 2000, 5, 7000);   // -delta (turn the other way)
//  HAL_Delay(1500);
//  run_gyro_step_test(600, 200, 1000, 2000, 5, 7000);   // check base dependence
//  HAL_Delay(1500);
//  run_gyro_step_test(800, 200, 1000, 2000, 5, 7000);


   //use gyro PID/////////////////////////////////////////////////////////////
  moveStraightGyroPID_Reset();
  //moveStraightPID_Reset();
  int left=0;
  int right=0;
  while(left*(-1)<=2555 || right*(-1)<=2539){
	  mpu9250_read_gyro();
	  left=get_left_encoder_total();
	  right=get_right_encoder_total();
	  moveStraightGyroPID();
	  //send_bluetooth_printf("L:%ld R:%ld\r\n",left,right);
  }
  break_motors();


  // Sensor Fusion///////////////////////////////////////////////////////////////////////
  // use single fusion straight controller
//  fusion_reset();
//  fusion_set_heading_ref_to_current();  // lock heading for this straight
//
//  while (1) {
//      fusion_step(570);                 // pick your cruise PWM (or 0 to use WF_BASE_PWM)
//      // optional telemetry
//      // send_bluetooth_printf("L:%ld R:%ld\r\n", get_left_encoder_total(), get_right_encoder_total());
//      HAL_Delay(2);                     // ~500 Hz outer loop
//  }


  // 0 = auto (both → center; else follow visible side), 1 = left, 2 = right
//  int mode = 0;               // WF_AUTO
//  int base_pwm = 570;         // use the speed you tuned at
//
//  // bootstrap targets & reset integrators
//  wall_follow_reset_int(mode, base_pwm);
//
//  while (1) {
//      wall_follow_step();     // computes e, PID, sets motor PWMs
//      //HAL_Delay(2);           // keep a steady loop
//      dwt_delay_us(50);
//  }



//  debug_encoder_setup();
//  test_encoder_manual();
//  test_encoder_rotation();


  /* Initialize movement system */
//  while(1){
//	  send_encoder_status();
//	  HAL_Delay(500);
//  }

  // get ADC Values//////////////////////////////////////////////////////
//  calibrate_sensors();
//  while(1){
//	  update_sensors();
//
//	  HAL_Delay(500);
//  }

//  run_wall_single_left_step_test(   // correct one to use with one wall
//      /*base_pwm=*/650,
//      /*delta_pwm=*/150,
//      /*step_delay_ms=*/1000,
//      /*step_duration_ms=*/2000,
//      /*sample_ms=*/10,      // 100 Hz; use 20 if BLE is slow
//      /*total_ms=*/6000
//  );


  //  run_wall_lateral_step_test(
  //      /*base_pwm=*/500,   /*delta_pwm=*/150,
  //      /*step_delay_ms=*/1000, /*step_duration_ms=*/2000,
  //      /*sample_ms=*/10,   /*total_ms=*/6000);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	// Handle button events
	if (button_pressed == 1) {
		button_pressed = 0;

		if (!exploration_started && system_ready) {
			send_bluetooth_message("\r\n🚀 STARTING MAZE EXPLORATION! 🚀\r\n");
			play_confirmation_tone();
			HAL_Delay(1000);

			exploration_started = true;
			run_maze_exploration_sequence();

		} else if (is_exploration_complete()) {
			send_bluetooth_message("\r\n🏁 EXPLORATION COMPLETE - Ready for speed run! 🏁\r\n");
			send_performance_metrics();

		} else if (!system_ready) {
			send_bluetooth_message("⚠️ System not ready - check diagnostics!\r\n");
			play_error_tone();
		}
	}

	if (button_pressed == 2) {
		button_pressed = 0;

		// Right button - emergency stop or reset
		if (exploration_started && !is_exploration_complete()) {
			send_bluetooth_message("🛑 EMERGENCY STOP!\r\n");
			stop_motors();
			play_error_tone();
			exploration_started = false;
		} else {
			// Send detailed status
			send_bluetooth_message("\r\n📊 DETAILED STATUS REPORT 📊\r\n");
			send_maze_state();
			send_sensor_data();
			send_position_data();
			if (exploration_started) {
				send_performance_metrics();
			}
		}
	}

	// Send periodic status updates
	send_periodic_status();

	// If exploration is running, let it continue
	if (exploration_started && !is_exploration_complete()) {
		// The exploration runs in run_maze_exploration_sequence()
		// and handles its own loop until complete
	}

	// Small delay to prevent overwhelming the system
	HAL_Delay(50);

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 20;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 200;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
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
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 4;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 838;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Chip_Select_Pin|LED_LEFT_Pin|LED_RIGHT_Pin|EMIT_SIDE_RIGHT_Pin
                          |EMIT_FRONT_LEFT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EMIT_FRONT_RIGHT_Pin|EMIT_SIDE_LEFT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : MOTOR_STBY_Pin */
  GPIO_InitStruct.Pin = MOTOR_STBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_STBY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_LEFT_Pin */
  GPIO_InitStruct.Pin = BTN_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_LEFT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_RIGHT_Pin */
  GPIO_InitStruct.Pin = BTN_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_RIGHT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Chip_Select_Pin LED_LEFT_Pin LED_RIGHT_Pin EMIT_SIDE_RIGHT_Pin
                           EMIT_FRONT_LEFT_Pin */
  GPIO_InitStruct.Pin = Chip_Select_Pin|LED_LEFT_Pin|LED_RIGHT_Pin|EMIT_SIDE_RIGHT_Pin
                          |EMIT_FRONT_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : EMIT_FRONT_RIGHT_Pin EMIT_SIDE_LEFT_Pin */
  GPIO_InitStruct.Pin = EMIT_FRONT_RIGHT_Pin|EMIT_SIDE_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t last_press = 0;
    uint32_t current_time = HAL_GetTick();

    // Debounce - ignore presses within 200ms
    if ((current_time - last_press) > 200) {
        if (GPIO_Pin == BTN_LEFT_Pin) {
            button_pressed = 1;
            start_flag = 1;  // Allow system to start
            send_bluetooth_message("Left button pressed\r\n");
        } else if (GPIO_Pin == BTN_RIGHT_Pin) {
            button_pressed = 2;
            send_bluetooth_message("Right button pressed\r\n");
        }
        last_press = current_time;
    }
}

// Add this function to main.c after MX_GPIO_Init()
void verify_adc_gpio_configuration(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Ensure all ADC pins are in analog mode
    // PA0 (ADC_CHANNEL_0) - Battery
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PA2 (ADC_CHANNEL_2) - Front Right
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PA3 (ADC_CHANNEL_3) - Side Right
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PA4 (ADC_CHANNEL_4) - Side Left
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PA5 (ADC_CHANNEL_5) - Front Left
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    send_bluetooth_message("✅ ADC GPIO configuration verified\r\n");
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
      HAL_GPIO_TogglePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin);
      HAL_GPIO_TogglePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin);
      HAL_Delay(100);
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
