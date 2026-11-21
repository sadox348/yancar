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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_A_PWM_MAX 999U
#define ENCODER_DIR_SIGN (-1)
#define RIGHT_ENCODER_SIGN 1
#define LEFT_ENCODER_FB_SIGN (1)
#define RIGHT_ENCODER_FB_SIGN (-1)
#define CMD_TO_DELTA_DIV 4
#define LEFT_PWM_PER_DELTA 5.0f
#define RIGHT_PWM_PER_DELTA 2.8f
#define SPEED_KP 1.0f
#define SPEED_KI 0.0f
#define SPEED_KD 0.0f
#define SPEED_I_LIMIT 300.0f
#define PWM_LIMIT 999
#define PWM_MIN_START 120
#define LEFT_MOTOR_SIGN -1
#define RIGHT_MOTOR_SIGN 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint8_t rx_byte;
uint8_t rx_buf[128];
volatile uint16_t rx_len = 0;
volatile uint8_t line_ready = 0;
float cmd_vx = 0.0f; 
float cmd_wz = 0.0f;
volatile uint32_t last_cmd_tick = 0;
volatile int16_t current_motor_pwm = 0;
volatile uint8_t motor_timeout = 0;
volatile int16_t current_b_pwm = 0;
volatile uint32_t last_b_cmd_tick = 0;
volatile int16_t current_left_pwm = 0;
volatile int16_t current_right_pwm = 0;
volatile uint8_t base_timeout = 0;
volatile int16_t right_encoder_count = 0;
volatile int16_t right_encoder_last = 0;
volatile int16_t right_encoder_delta = 0;
volatile int16_t target_left_delta = 0;
volatile int16_t target_right_delta = 0;
volatile int16_t actual_left_delta = 0;
volatile int16_t actual_right_delta = 0;
volatile int16_t control_left_pwm = 0;
volatile int16_t control_right_pwm = 0;
uint8_t robot_error = 0;
int cmd_vx_i = 0;
int cmd_wz_i = 0;

/* encoder variables */
volatile int32_t encoder_cnt = 0;
volatile int32_t encoder_last_cnt = 0;
volatile int32_t encoder_delta = 0;
volatile uint32_t encoder_last_tick = 0;

volatile int16_t target_speed_delta = 0;
volatile int16_t actual_speed_delta = 0;
volatile int16_t control_pwm = 0;
volatile uint8_t speed_loop_enabled = 0;

int32_t encoder_count = 0;
int32_t encoder_last = 0;
uint32_t last_speed_tick = 0;

float kp = 3.0f;
float ki = 0.0f;
float kd = 0.0f;

float pid_integral = 0.0f;
float pid_last_error = 0.0f;
static float left_err_i = 0.0f;
static float right_err_i = 0.0f;
static float left_last_err = 0.0f;
static float right_last_err = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void Motor_A_Stop(void);
void Motor_A_Forward(uint16_t pwm);
void Motor_A_Backward(uint16_t pwm);
void Motor_A_Set(int16_t pwm);
void Motor_B_Stop(void);
void Motor_B_Forward(uint16_t pwm);
void Motor_B_Backward(uint16_t pwm);
void Motor_B_Set(int16_t pwm);
void Base_Stop(void);
void Base_OpenLoop_Set(int16_t left_pwm, int16_t right_pwm);
void Speed_Loop_Update(void);
void Encoder_Right_Update(void);
void Base_SpeedLoop_Update(void);

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
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  // char *hello = "Hello, UART!\r\n";
  // HAL_UART_Transmit(&huart1, (uint8_t *)hello, strlen(hello), 100);
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  /* start encoder */
  if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK)
  {
    Error_Handler();
  }

  Base_Stop();

  encoder_last = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
  encoder_count = encoder_last;
  encoder_cnt = encoder_count;
  encoder_last_cnt = encoder_count;
  encoder_delta = 0;
  actual_speed_delta = 0;
  encoder_last_tick = HAL_GetTick();
  last_speed_tick = HAL_GetTick();
  last_cmd_tick = HAL_GetTick();
  target_speed_delta = 0;
  control_pwm = 0;
  current_motor_pwm = 0;
  current_b_pwm = 0;
  current_left_pwm = 0;
  current_right_pwm = 0;
  right_encoder_last = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
  right_encoder_count = 0;
  right_encoder_delta = 0;
  target_left_delta = 0;
  target_right_delta = 0;
  actual_left_delta = 0;
  actual_right_delta = 0;
  control_left_pwm = 0;
  control_right_pwm = 0;
  speed_loop_enabled = 0;
  pid_integral = 0.0f;
  pid_last_error = 0.0f;
  left_err_i = 0.0f;
  right_err_i = 0.0f;
  left_last_err = 0.0f;
  right_last_err = 0.0f;
  motor_timeout = 0;
  base_timeout = 0;
  last_b_cmd_tick = HAL_GetTick();

  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  const uint8_t boot_msg[] = "STM32 v0.9 STATUS PACKET BOOT\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t *)boot_msg, sizeof(boot_msg) - 1U, 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (line_ready)
    {
      uint8_t line_buf[sizeof(rx_buf)];
      uint16_t line_len;

      __disable_irq();
      line_len = rx_len;
      if (line_len >= sizeof(line_buf))
      {
        line_len = sizeof(line_buf) - 1U;
      }
      memcpy(line_buf, rx_buf, line_len);
      line_buf[line_len] = '\0';
      rx_len = 0;
      rx_buf[0] = '\0';
      line_ready = 0;
      __enable_irq();

      if (sscanf((char *)line_buf, "V,%d,%d", &cmd_vx_i, &cmd_wz_i) == 2)
      {
        char msg[64];
        int32_t left_pwm_calc = (int32_t)cmd_vx_i - (int32_t)cmd_wz_i;
        int32_t right_pwm_calc = (int32_t)cmd_vx_i + (int32_t)cmd_wz_i;

        last_cmd_tick = HAL_GetTick();
        last_b_cmd_tick = HAL_GetTick();
        base_timeout = 0;
        motor_timeout = 0;

        if (left_pwm_calc > (int32_t)MOTOR_A_PWM_MAX)
        {
          left_pwm_calc = (int32_t)MOTOR_A_PWM_MAX;
        }
        else if (left_pwm_calc < -(int32_t)MOTOR_A_PWM_MAX)
        {
          left_pwm_calc = -(int32_t)MOTOR_A_PWM_MAX;
        }

        if (right_pwm_calc > (int32_t)MOTOR_A_PWM_MAX)
        {
          right_pwm_calc = (int32_t)MOTOR_A_PWM_MAX;
        }
        else if (right_pwm_calc < -(int32_t)MOTOR_A_PWM_MAX)
        {
          right_pwm_calc = -(int32_t)MOTOR_A_PWM_MAX;
        }

        target_left_delta = (int16_t)(left_pwm_calc / CMD_TO_DELTA_DIV);
        target_right_delta = (int16_t)(right_pwm_calc / CMD_TO_DELTA_DIV);

        if ((left_pwm_calc == 0) && (right_pwm_calc == 0))
        {
          speed_loop_enabled = 0;
          target_speed_delta = 0;
          control_pwm = 0;
          target_left_delta = 0;
          target_right_delta = 0;
          control_left_pwm = 0;
          control_right_pwm = 0;
          left_err_i = 0.0f;
          right_err_i = 0.0f;
          left_last_err = 0.0f;
          right_last_err = 0.0f;
          pid_integral = 0.0f;
          pid_last_error = 0.0f;
          Base_Stop();
        }
        else
        {
          speed_loop_enabled = 1;
        }

        int len = snprintf(msg, sizeof(msg),
                           "ACK,V,%d,%d,%d,%d,%d\r\n",
                           cmd_vx_i, cmd_wz_i, target_left_delta,
                           target_right_delta, speed_loop_enabled);
        if (len >= (int)sizeof(msg))
        {
          len = (int)sizeof(msg) - 1;
        }
        if (len > 0)
        {
          HAL_UART_Transmit(&huart1, (uint8_t *)msg, (uint16_t)len, 100);
        }
      }
      else if (strcmp((char *)line_buf, "Q") == 0)
      {
        const uint8_t ver_msg[] = "VER,0.9,DUAL_SPEED_LOOP_STATUS\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t *)ver_msg, sizeof(ver_msg) - 1U, 100);
      }
      else
{
  int a_pwm = 0;
  int b_pwm = 0;

  if (sscanf((char *)line_buf, "A,%d", &a_pwm) == 1)
  {
    char msg[32];

    if (a_pwm > (int)MOTOR_A_PWM_MAX)
    {
      a_pwm = (int)MOTOR_A_PWM_MAX;
    }
    else if (a_pwm < -(int)MOTOR_A_PWM_MAX)
    {
      a_pwm = -(int)MOTOR_A_PWM_MAX;
    }

    // 关闭闭环，防止 PID 抢控制权
    speed_loop_enabled = 0;
    target_speed_delta = 0;
    control_pwm = 0;
    target_left_delta = 0;
    target_right_delta = 0;
    control_left_pwm = 0;
    control_right_pwm = 0;

    left_err_i = 0.0f;
    right_err_i = 0.0f;
    left_last_err = 0.0f;
    right_last_err = 0.0f;
    pid_integral = 0.0f;
    pid_last_error = 0.0f;

    // 只转 A 路，B 路停止
    Motor_B_Stop();
    current_b_pwm = 0;
    current_left_pwm = a_pwm;
    current_right_pwm = 0;

    // 这里故意不乘 LEFT_MOTOR_SIGN，用来检查 A 路物理对应哪个轮子
    Motor_A_Set((int16_t)a_pwm);

    last_b_cmd_tick = HAL_GetTick();

    int len = snprintf(msg, sizeof(msg), "ACK,A,%d\r\n", a_pwm);
    if (len >= (int)sizeof(msg))
    {
      len = (int)sizeof(msg) - 1;
    }
    if (len > 0)
    {
      HAL_UART_Transmit(&huart1, (uint8_t *)msg, (uint16_t)len, 100);
    }
  }
  else if (sscanf((char *)line_buf, "B,%d", &b_pwm) == 1)
  {
    char msg[32];

    if (b_pwm > (int)MOTOR_A_PWM_MAX)
    {
      b_pwm = (int)MOTOR_A_PWM_MAX;
    }
    else if (b_pwm < -(int)MOTOR_A_PWM_MAX)
    {
      b_pwm = -(int)MOTOR_A_PWM_MAX;
    }

    // 关闭闭环，防止 PID 抢控制权
    speed_loop_enabled = 0;
    target_speed_delta = 0;
    control_pwm = 0;
    target_left_delta = 0;
    target_right_delta = 0;
    control_left_pwm = 0;
    control_right_pwm = 0;

    left_err_i = 0.0f;
    right_err_i = 0.0f;
    left_last_err = 0.0f;
    right_last_err = 0.0f;
    pid_integral = 0.0f;
    pid_last_error = 0.0f;

    // 只转 B 路，A 路停止
    Motor_A_Stop();
    current_left_pwm = 0;
    current_right_pwm = b_pwm;

    // 这里故意不乘 RIGHT_MOTOR_SIGN，用来检查 B 路物理对应哪个轮子
    Motor_B_Set((int16_t)b_pwm);

    last_b_cmd_tick = HAL_GetTick();

    int len = snprintf(msg, sizeof(msg), "ACK,B,%d\r\n", b_pwm);
    if (len >= (int)sizeof(msg))
    {
      len = (int)sizeof(msg) - 1;
    }
    if (len > 0)
    {
      HAL_UART_Transmit(&huart1, (uint8_t *)msg, (uint16_t)len, 100);
    }
  }
  else
  {
    if ((line_buf[0] == 'V') || (line_buf[0] == 'A') || (line_buf[0] == 'B'))
    {
      const uint8_t parse_err_msg[] = "ERR,PARSE\r\n";
      HAL_UART_Transmit(&huart1, (uint8_t *)parse_err_msg, sizeof(parse_err_msg) - 1U, 100);
    }
    else
    {
      const uint8_t unknown_msg[] = "ERR,UNKNOWN_CMD\r\n";
      HAL_UART_Transmit(&huart1, (uint8_t *)unknown_msg, sizeof(unknown_msg) - 1U, 100);
    }
  }
}
    }

    if ((HAL_GetTick() - last_cmd_tick > 500U) && (speed_loop_enabled == 1U))
    {
      const uint8_t timeout_msg[] = "EVT,BASE_TIMEOUT_STOP\r\n";

      speed_loop_enabled = 0;
      target_speed_delta = 0;
      control_pwm = 0;
      target_left_delta = 0;
      target_right_delta = 0;
      control_left_pwm = 0;
      control_right_pwm = 0;
      pid_integral = 0.0f;
      pid_last_error = 0.0f;
      left_err_i = 0.0f;
      right_err_i = 0.0f;
      left_last_err = 0.0f;
      right_last_err = 0.0f;
      base_timeout = 1;
      Base_Stop();
      HAL_UART_Transmit(&huart1, (uint8_t *)timeout_msg, sizeof(timeout_msg) - 1U, 100);
    }

    if ((speed_loop_enabled == 0U) &&
        (HAL_GetTick() - last_b_cmd_tick > 1000U) && (current_b_pwm != 0))
    {
      const uint8_t timeout_b_msg[] = "MOTOR B TIMEOUT STOP\r\n";

      Motor_B_Stop();
      HAL_UART_Transmit(&huart1, (uint8_t *)timeout_b_msg, sizeof(timeout_b_msg) - 1U, 100);
    }

    if (HAL_GetTick() - last_speed_tick >= 100U)
    {
      last_speed_tick += 100U;
      Base_SpeedLoop_Update();
    }

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Period = 4294967295;
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
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB10 PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Motor_A_Stop(void)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  current_motor_pwm = 0;
}

void Motor_A_Forward(uint16_t pwm)
{
  if (pwm > MOTOR_A_PWM_MAX)
  {
    pwm = MOTOR_A_PWM_MAX;
  }

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);
}

void Motor_A_Backward(uint16_t pwm)
{
  if (pwm > MOTOR_A_PWM_MAX)
  {
    pwm = MOTOR_A_PWM_MAX;
  }

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);
}

void Motor_A_Set(int16_t pwm)
{
  if (pwm > (int16_t)MOTOR_A_PWM_MAX)
  {
    pwm = (int16_t)MOTOR_A_PWM_MAX;
  }
  else if (pwm < -(int16_t)MOTOR_A_PWM_MAX)
  {
    pwm = -(int16_t)MOTOR_A_PWM_MAX;
  }

  if (pwm > 0)
  {
    Motor_A_Forward((uint16_t)pwm);
    current_motor_pwm = pwm;
  }
  else if (pwm < 0)
  {
    Motor_A_Backward((uint16_t)(-pwm));
    current_motor_pwm = pwm;
  }
  else
  {
    Motor_A_Stop();
  }
}

void Motor_B_Stop(void)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
  current_b_pwm = 0;
}

void Motor_B_Forward(uint16_t pwm)
{
  if (pwm > MOTOR_A_PWM_MAX)
  {
    pwm = MOTOR_A_PWM_MAX;
  }

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm);
}

void Motor_B_Backward(uint16_t pwm)
{
  if (pwm > MOTOR_A_PWM_MAX)
  {
    pwm = MOTOR_A_PWM_MAX;
  }

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm);
}

void Motor_B_Set(int16_t pwm)
{
  if (pwm > (int16_t)MOTOR_A_PWM_MAX)
  {
    pwm = (int16_t)MOTOR_A_PWM_MAX;
  }
  else if (pwm < -(int16_t)MOTOR_A_PWM_MAX)
  {
    pwm = -(int16_t)MOTOR_A_PWM_MAX;
  }

  if (pwm > 0)
  {
    Motor_B_Forward((uint16_t)pwm);
    current_b_pwm = pwm;
  }
  else if (pwm < 0)
  {
    Motor_B_Backward((uint16_t)(-pwm));
    current_b_pwm = pwm;
  }
  else
  {
    Motor_B_Stop();
  }
}

void Base_Stop(void)
{
  Motor_A_Stop();
  Motor_B_Stop();
  current_left_pwm = 0;
  current_right_pwm = 0;
  control_left_pwm = 0;
  control_right_pwm = 0;
}

void Base_OpenLoop_Set(int16_t left_pwm, int16_t right_pwm)
{
  int32_t left_output;
  int32_t right_output;

  if (left_pwm > (int16_t)MOTOR_A_PWM_MAX)
  {
    left_pwm = (int16_t)MOTOR_A_PWM_MAX;
  }
  else if (left_pwm < -(int16_t)MOTOR_A_PWM_MAX)
  {
    left_pwm = -(int16_t)MOTOR_A_PWM_MAX;
  }

  if (right_pwm > (int16_t)MOTOR_A_PWM_MAX)
  {
    right_pwm = (int16_t)MOTOR_A_PWM_MAX;
  }
  else if (right_pwm < -(int16_t)MOTOR_A_PWM_MAX)
  {
    right_pwm = -(int16_t)MOTOR_A_PWM_MAX;
  }

  left_output = (int32_t)LEFT_MOTOR_SIGN * (int32_t)left_pwm;
  right_output = (int32_t)RIGHT_MOTOR_SIGN * (int32_t)right_pwm;

  if (left_output > (int32_t)MOTOR_A_PWM_MAX)
  {
    left_output = (int32_t)MOTOR_A_PWM_MAX;
  }
  else if (left_output < -(int32_t)MOTOR_A_PWM_MAX)
  {
    left_output = -(int32_t)MOTOR_A_PWM_MAX;
  }

  if (right_output > (int32_t)MOTOR_A_PWM_MAX)
  {
    right_output = (int32_t)MOTOR_A_PWM_MAX;
  }
  else if (right_output < -(int32_t)MOTOR_A_PWM_MAX)
  {
    right_output = -(int32_t)MOTOR_A_PWM_MAX;
  }

  Motor_A_Set((int16_t)left_output);
  Motor_B_Set((int16_t)right_output);
  current_left_pwm = left_pwm;
  current_right_pwm = right_pwm;
}

void Speed_Loop_Update(void)
{
  int32_t raw_delta;

  encoder_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
  raw_delta = encoder_count - encoder_last;
  encoder_last = encoder_count;

  actual_speed_delta = (int16_t)raw_delta;
  encoder_cnt = encoder_count;
  encoder_delta = raw_delta;
  encoder_last_cnt = encoder_count;
  encoder_last_tick = HAL_GetTick();
}

void Encoder_Right_Update(void)
{
  int16_t now = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
  int16_t raw_delta = now - right_encoder_last;

  right_encoder_last = now;
  right_encoder_count += raw_delta;
  right_encoder_delta = raw_delta;
}

void Base_SpeedLoop_Update(void)
{
  char msg[96];
  int len;

  Speed_Loop_Update();
  Encoder_Right_Update();

  actual_left_delta = (int16_t)(LEFT_ENCODER_FB_SIGN * encoder_delta);
  actual_right_delta = (int16_t)(RIGHT_ENCODER_FB_SIGN * right_encoder_delta);

  float left_ff = (float)target_left_delta * LEFT_PWM_PER_DELTA;
  float right_ff = (float)target_right_delta * RIGHT_PWM_PER_DELTA;

  if (speed_loop_enabled == 0U)
  {
    control_left_pwm = 0;
    control_right_pwm = 0;
  }
  else
  {
    float left_err = (float)target_left_delta - (float)actual_left_delta;
    float right_err = (float)target_right_delta - (float)actual_right_delta;

    left_err_i += left_err;
    right_err_i += right_err;

    if (left_err_i > SPEED_I_LIMIT)
    {
      left_err_i = SPEED_I_LIMIT;
    }
    else if (left_err_i < -SPEED_I_LIMIT)
    {
      left_err_i = -SPEED_I_LIMIT;
    }

    if (right_err_i > SPEED_I_LIMIT)
    {
      right_err_i = SPEED_I_LIMIT;
    }
    else if (right_err_i < -SPEED_I_LIMIT)
    {
      right_err_i = -SPEED_I_LIMIT;
    }

    float left_err_d = left_err - left_last_err;
    float right_err_d = right_err - right_last_err;
    left_last_err = left_err;
    right_last_err = right_err;

    float left_out = left_ff + (SPEED_KP * left_err) +
                     (SPEED_KI * left_err_i) + (SPEED_KD * left_err_d);
    float right_out = right_ff + (SPEED_KP * right_err) +
                      (SPEED_KI * right_err_i) + (SPEED_KD * right_err_d);

    if ((target_left_delta != 0) &&
        abs(actual_left_delta) < 5 &&
        abs((int)left_out) < PWM_MIN_START)
    {
      left_out = (target_left_delta > 0) ? PWM_MIN_START : -PWM_MIN_START;
    }

    if ((target_right_delta != 0) &&
        abs(actual_right_delta) < 5 &&
        abs((int)right_out) < PWM_MIN_START)
    {
      right_out = (target_right_delta > 0) ? PWM_MIN_START : -PWM_MIN_START;
    }

    if (left_out > (float)PWM_LIMIT)
    {
      left_out = (float)PWM_LIMIT;
    }
    else if (left_out < -(float)PWM_LIMIT)
    {
      left_out = -(float)PWM_LIMIT;
    }

    if (right_out > (float)PWM_LIMIT)
    {
      right_out = (float)PWM_LIMIT;
    }
    else if (right_out < -(float)PWM_LIMIT)
    {
      right_out = -(float)PWM_LIMIT;
    }

    control_left_pwm = (int16_t)left_out;
    control_right_pwm = (int16_t)right_out;

    Motor_A_Set((int16_t)(LEFT_MOTOR_SIGN * control_left_pwm));
    Motor_B_Set((int16_t)(RIGHT_MOTOR_SIGN * control_right_pwm));
    current_left_pwm = control_left_pwm;
    current_right_pwm = control_right_pwm;
  }

  len = snprintf(msg, sizeof(msg),
                 "STAT,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                 speed_loop_enabled, target_left_delta, actual_left_delta, control_left_pwm,
                 target_right_delta, actual_right_delta, control_right_pwm,
                 (int)left_ff, (int)right_ff);
  if (len >= (int)sizeof(msg))
  {
    len = (int)sizeof(msg) - 1;
  }
  if (len > 0)
  {
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, (uint16_t)len, 100);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    if (rx_byte == '\n' || rx_byte == '\r')
    {
      if (rx_len > 0)
      {
        rx_buf[rx_len] = '\0';
        line_ready = 1;
      }
    }
    else if (!line_ready)
    {
      if (rx_len < sizeof(rx_buf) - 1)
      {
        rx_buf[rx_len++] = rx_byte;
      }
    }

    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  }
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
