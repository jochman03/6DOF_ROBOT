/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "oled.h"
#include "nm_simplex.h"
#include "servo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*   SERVOS   */
#define SERVO_COUNT 	6

/*   DELAYS   */
#define INPUT_DELAY 	100
#define BTN_DELAY 		1000

/*   INPUTS   */
#define DEADZONE		0.3f
#define AXIS_SPEED		30
#define AXIS_INVERSION	-1
#define DT				0.1f
#define ADC_RES			2048.0f

/*   IK   */
#define IK_ITERATIONS	20

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint8_t uart2_rx;
uint8_t uart2_buf[64];
uint8_t uart2_idx = 0;


/*   SERVOS    */

servo_t servos[SERVO_COUNT];

typedef enum {
	R_START = 0,
	R_BASE_ZERO_PRE,
	R_BASE_ZERO,
	R_BASE_PRE,
	R_BASE,
	R_IK,
	R_CMD_POS,
	R_ZERO_PRE,
	R_ZERO
} robot_state_t;

robot_state_t robot_state = R_START;


/*    IK    */
uint8_t update_ik = 0;

int32_t workspaceX_lim[2] = {-500, -50};
int32_t workspaceY_lim[2] = {-500, 500};
int32_t workspaceZ_lim[2] = {30, 500};

float ik_phi0[4] = {0};
float ik_solution[4] = {0};
float ik_score = 0;
float ik_target_pos[3] = {0};

float phi_min[4] = {-80, -50, -15, -15};
float phi_max[4] = {80, 50, 100, 90};

uint16_t phi_min_pulse[4];
uint16_t phi_max_pulse[4];


float ik_debug_xdeg[4];
float ik_pos[3];
uint16_t ik_debug_xpulse[4];

/*   BASE   */
float base_deg[4] = {50.0f, 40.0f, 60.0f, 15.0f};
uint16_t base_pos[4];


/*   CONTROLS   */

uint32_t last_button_click = 0;
uint8_t last_button_state = 1;
uint32_t last_inputs_check = 0;

uint16_t adc_buf[3];

float controls_pos[3];


/*   Display  */
uint8_t update_display = 1;
char disp_buff[64];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

void parse_servo_command(char *buf);

void robotHandle();
void ikHandle();
void inputsHandle();
void buttonsHandle();
void displayHandle();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */


  /*  CALCULATE ANGLES  */
  deg_2_rad(ik_phi0, 4);
  deg_2_rad(phi_min, 4);
  deg_2_rad(phi_max, 4);
  deg_2_pulse(base_deg, base_pos, 4);
  rad_2_pulse(phi_min, phi_min_pulse, 4);
  rad_2_pulse(phi_max, phi_max_pulse, 4);

  /*  ADC INIT  */
  HAL_TIM_Base_Start(&htim3);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 3);

  /*  DISPLAY INIT  */
  OLED_Init(&hi2c1);
  OLED_Clear();
  update_display = 1;

  /*  SERVOS INIT  */
  servoInit(&servos[0], &htim1, TIM_CHANNEL_1, 550, 2450, SERVO_POS_MID, 0);
  servoInit(&servos[1], &htim1, TIM_CHANNEL_2, 550, 2400, SERVO_POS_MID, 0);
  servoInit(&servos[2], &htim1, TIM_CHANNEL_3, 530, 2350, SERVO_POS_MID, 0);
  servoInit(&servos[3], &htim1, TIM_CHANNEL_4, 525, 2425, SERVO_POS_MID, 1);
  servoInit(&servos[4], &htim2, TIM_CHANNEL_1, 700, 2600, SERVO_POS_MID, 0);
  servoInit(&servos[5], &htim2, TIM_CHANNEL_2, 700, 2600, SERVO_POS_MID, 0);

  /*  UART RECEIVE  */
  HAL_UART_Receive_IT(&huart2, &uart2_rx, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  robotHandle();
	  ikHandle();
	  inputsHandle();
	  buttonsHandle();
	  displayHandle();

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
  RCC_OscInitStruct.PLL.PLLN = 192;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 95;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 1000;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 95;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 95;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


static void clampInt32(int* val, int32_t min, int32_t max){
	if(*val < min){
		*val = min;
	}
	else if(*val > max){
		*val = max;
	}
}

static void clampUint16(uint16_t* vector, uint16_t* min_vector, uint16_t* max_vector, uint8_t size){
	for(uint8_t i = 0; i < size; i++){
		if(vector[i] < min_vector[i]){
			vector[i] = min_vector[i];
		}
		else if(vector[i] > max_vector[i]){
			vector[i] = max_vector[i];
		}
	}
}

static void clampFloat(float* val, float min, float max){
    if(*val < min){
    	*val = min;
    }
    else if(*val > max){
    	*val = max;
    }
}

void parse_servo_command(char *buf){
	if (strncmp(buf, "SERVO_POS", 9) == 0) {
        if (robot_state != R_CMD_POS) {
            return;
        }

        int id;
        int pos;

        if (sscanf(buf, "SERVO_POS,%d,%d", &id, &pos) != 2) {
            return;
        }

        if (id < 0 || id >= SERVO_COUNT) {
            return;
        }

        if (pos < SERVO_POS_MIN || pos > SERVO_POS_MAX) {
            return;
        }

        servoSetTarget(&servos[id], (uint16_t)pos);
        update_display = 1;
        return;
    }
    else if (strncmp(buf, "SET_IK", 6) == 0) {
        if (robot_state != R_IK) {
            return;
        }

        int tempx;
        int tempy;
        int tempz;

        if (sscanf(buf, "SET_IK,%d,%d,%d", &tempx, &tempy, &tempz) != 3) {
            return;
        }
        clampInt32(&tempx, workspaceX_lim[0], workspaceX_lim[1]);
        clampInt32(&tempy, workspaceY_lim[0], workspaceY_lim[1]);
        clampInt32(&tempz, workspaceZ_lim[0], workspaceZ_lim[1]);

        ik_target_pos[0] = (float) tempx;
        ik_target_pos[1] = (float) tempy;
        ik_target_pos[2] = (float) tempz;

        controls_pos[0] = (float) tempx;
        controls_pos[1] = (float) tempy;
        controls_pos[2] = (float) tempz;

        update_ik = 1;

        return;
    }
    else if (strncmp(buf, "ZERO", 4) == 0) {
        robot_state = R_ZERO_PRE;
        update_display = 1;
        return;
    }
    else if (strncmp(buf, "START", 5) == 0) {
        robot_state = R_BASE_ZERO_PRE;
        update_display = 1;
        return;
    }
    else if (strncmp(buf, "CMD_POS", 7) == 0) {
        robot_state = R_CMD_POS;
        update_display = 1;
        return;
    }
    else if (strncmp(buf, "IK", 2) == 0) {
        robot_state = R_IK;
        update_display = 1;
        return;
    }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2){
        if (uart2_rx == '\n'){
            uart2_buf[uart2_idx] = 0;
            parse_servo_command((char*)uart2_buf);
            uart2_idx = 0;
        }
        else{
            uart2_buf[uart2_idx++] = uart2_rx;

            if (uart2_idx >= sizeof(uart2_buf)){
                uart2_idx = 0;
            }
        }

        HAL_UART_Receive_IT(&huart2, &uart2_rx, 1);
    }
}

static inline uint8_t reached(uint16_t a, uint16_t b){
    return (a > b) ? (a - b < 5) : (b - a < 5);
}

static void updatePos(){
	uint16_t temp_pulse[4];
	for(uint8_t i = 0; i < 4; i++){
		temp_pulse[i] = servos[i].current_pos;
	}
	pulse_2_rad(ik_phi0, temp_pulse, 4);
	for(uint8_t i = 0; i < 4; i++){
		ik_debug_xdeg[i] = ik_phi0[i];
	}
	rad_2_deg(ik_debug_xdeg, 4);
	pos_c(ik_phi0, ik_target_pos);
	for(uint8_t i = 0; i < 3; i++){
		controls_pos[i] = ik_target_pos[i];
	}
}

void robotHandle(void){
    uint8_t all_in_pos = 1;

    switch (robot_state){
		case R_BASE_ZERO_PRE:
			for (uint8_t i = 0; i < 5; i++) {
				servos[i].target_pos = SERVO_POS_MID;
			}
			robot_state = R_BASE_ZERO;
			break;
		case R_BASE_ZERO:
			for (uint8_t i = 0; i < SERVO_COUNT; i++) {
				if (!reached(servos[i].current_pos, servos[i].target_pos)) {
					all_in_pos = 0;
					break;
				}
			}
			if (all_in_pos) {
				robot_state = R_BASE_PRE;
			}
			break;
		case R_BASE_PRE:
			for(uint8_t i = 0; i < 4; i++){
				servos[i].target_pos = base_pos[i];
			}
			robot_state = R_BASE;
			update_display = 1;
			break;

		case R_BASE:
			for (uint8_t i = 0; i < SERVO_COUNT; i++) {
				if (!reached(servos[i].current_pos, servos[i].target_pos)) {
					all_in_pos = 0;
					break;
				}
			}
			if (all_in_pos) {
				updatePos();
				robot_state = R_IK;
				update_display = 1;
			}

			break;
		case R_ZERO_PRE:
			for (uint8_t i = 0; i < 5; i++) {
				servos[i].target_pos = SERVO_POS_MID;
			}
			robot_state = R_ZERO;
			update_display = 1;
			break;
		case R_ZERO:
			break;

		default:
			break;
	}

    uint32_t now = HAL_GetTick();
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        servoUpdate(&servos[i], now);
    }
}



void ikHandle(){
	if(robot_state != R_IK){
		return;
	}
	if(!update_ik){
		return;
	}
	update_ik = 0;
    simplex(pos_f, ik_phi0, IK_ITERATIONS, &ik_score, ik_solution, ik_target_pos, phi_min, phi_max);

    for(uint8_t i = 0; i < 4; i++){
    	ik_phi0[i] = ik_solution[i];
    	ik_debug_xdeg[i] = ik_solution[i];
    }
	rad_2_deg(ik_debug_xdeg, 4);
	rad_2_pulse(ik_solution, ik_debug_xpulse, 4);

	clampUint16(ik_debug_xpulse, phi_min_pulse, phi_max_pulse, 4);

	for(uint8_t i = 0; i < 4; i++){
		servoSetTarget(&servos[i], ik_debug_xpulse[i]);
	}
    pos_c(ik_solution, ik_pos);
    update_display = 1;
}

void inputsHandle(){
	if(robot_state != R_IK){
		return;
	}

	uint32_t now = HAL_GetTick();
	if(now - last_inputs_check < INPUT_DELAY){
		return;
	}
	last_inputs_check = now;

	float axis_speed = AXIS_SPEED * AXIS_INVERSION;

	float norm_X = ((float)adc_buf[0] - ADC_RES)/ADC_RES;
	float norm_Y = ((float)adc_buf[1] - ADC_RES)/ADC_RES;
	float norm_Z = ((float)adc_buf[2] - ADC_RES)/ADC_RES;

	float abs_norm_X = fabsf(norm_X);
	float abs_norm_Y = fabsf(norm_Y);
	float abs_norm_Z = fabsf(norm_Z);


	if(abs_norm_X < DEADZONE){
		abs_norm_X = 0;
	}
	if(abs_norm_Y < DEADZONE){
		abs_norm_Y = 0;
	}
	if(abs_norm_Z < DEADZONE){
		abs_norm_Z = 0;
	}

	if(abs_norm_X > 0 || abs_norm_Y > 0 || abs_norm_Z > 0){
		controls_pos[0] += norm_X * axis_speed * DT;
		controls_pos[1] += norm_Y * axis_speed * DT;
		controls_pos[2] += norm_Z * axis_speed * DT;
		clampFloat(&controls_pos[0], (float)workspaceX_lim[0], (float)workspaceX_lim[1]);
		clampFloat(&controls_pos[1], (float)workspaceY_lim[0], (float)workspaceY_lim[1]);
		clampFloat(&controls_pos[2], (float)workspaceZ_lim[0], (float)workspaceZ_lim[1]);

		for(uint8_t i = 0; i < 3; i++){
			ik_target_pos[i] = controls_pos[i];
		}
		update_ik = 1;
	}
}


void buttonsHandle(){
	uint32_t now = HAL_GetTick();

	if(now - last_button_click < BTN_DELAY){
		return;
	}

	uint8_t state = HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin);

	if(state == 0 && last_button_state == 1){
		if (robot_state == R_IK){
			robot_state = R_ZERO_PRE;
			update_display = 1;
		}
		else if (robot_state == R_ZERO || robot_state == R_START){
			robot_state = R_BASE_ZERO_PRE;
			update_display = 1;
		}
		last_button_click = now;
	}
	last_button_state = state;

}



void displayHandle(){
	if(!update_display){
		return;
	}
	update_display = 0;
	OLED_Clear();
	switch(robot_state){
		case R_ZERO:
			sprintf(disp_buff, "Zero");
			OLED_WriteText(disp_buff, 0, 0, 6);
			sprintf(disp_buff, "Press BASE btn");
			OLED_WriteText(disp_buff, 0, 12, 6);
			break;
		case R_START:
			sprintf(disp_buff, "Start");
			OLED_WriteText(disp_buff, 0, 0, 6);
			sprintf(disp_buff, "Press BASE btn");
			OLED_WriteText(disp_buff, 0, 12, 6);
			break;
		case R_CMD_POS:
			sprintf(disp_buff, "CMD POS");
			OLED_WriteText(disp_buff, 0, 0, 6);
			sprintf(disp_buff, "%d,%d,%d,%d,%d,%d", servos[0].current_pos, servos[1].current_pos, servos[2].current_pos, servos[3].current_pos, servos[4].current_pos, servos[5].current_pos);
			OLED_WriteText(disp_buff, 0, 12, 6);
			break;
		case R_IK:
			sprintf(disp_buff, "Inverse Kinematics:");
			OLED_WriteText(disp_buff, 0, 0, 6);
			sprintf(disp_buff, "P: %0.0lf,%0.0lf,%0.0lf", ik_target_pos[0], ik_target_pos[1], ik_target_pos[2]);
			OLED_WriteText(disp_buff, 0, 12, 6);
			sprintf(disp_buff, "K: %0.0lf,%0.0lf,%0.0lf", ik_pos[0], ik_pos[1], ik_pos[2]);
			OLED_WriteText(disp_buff, 0, 24, 6);
			break;
		default:
			sprintf(disp_buff, "Please wait.");
			OLED_WriteText(disp_buff, 0, 0, 6);
			break;
	}
	OLED_Update();
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
