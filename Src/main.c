/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (RTOS + Relays + Buttons + ADC + UART)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "rtos.h"
#include <string.h>
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Sensor data structure
typedef struct {
    float current_A;
    float voltage_V;
    float power_W;
    uint32_t timestamp;
} SensorData_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* === Fix CubeMX names === */
#define BTN_ALL_Pin        GPIO_PIN_0
#define BTN_ALL_GPIO_Port  GPIOC
#define BTN_L1_Pin         GPIO_PIN_1
#define BTN_L1_GPIO_Port   GPIOC
#define BTN_L2_Pin         GPIO_PIN_2
#define BTN_L2_GPIO_Port   GPIOC
#define Relay2_Pin         GPIO_PIN_1
#define Relay2_GPIO_Port   GPIOB

/* === ADC / Sensor Data === */
volatile uint16_t g_adcLatest = 0;
float g_current_A = 0.0f;
float g_voltage_V = 0.0f;

/* === Relay Control === */
uint8_t g_relayPattern = 0;
uint8_t g_cmdL1_on  = 0;
uint8_t g_cmdL1_off = 0;
uint8_t g_cmdL2_on  = 0;
uint8_t g_cmdL2_off = 0;

/* === Button Debouncing === */
uint8_t  btnAll_prev = 0, btnL1_prev = 0, btnL2_prev = 0;
uint32_t btnAll_last = 0, btnL1_last = 0, btnL2_last = 0;

/* === Inter-Task Communication === */
static int8_t g_sensorQueue = -1;
static int8_t g_uartSemaphore = -1;
static volatile SensorData_t g_latestSensorData = {0};

/* === UART RX === */
static uint8_t uartRxBuffer[1];
static char cmdBuffer[32];
static uint8_t cmdIndex = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART6_UART_Init(void);

/* USER CODE BEGIN PFP */

/* RTOS task prototypes */
void SensorTask(void *param);
void ControlTask(void *param);
void UartTxTask(void *param);
void HeartbeatTask(void *param);

/* Helpers */
uint8_t ButtonPressedDebounced(GPIO_TypeDef *port, uint16_t pin,
                               uint8_t *prevState, uint32_t *lastTime);
void Relay1_On(void);
void Relay1_Off(void);
void Relay2_On(void);
void Relay2_Off(void);
void LV8153_WritePattern(uint8_t pattern);
void RTOS_InitResources(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/* ADC conversion complete callback */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        g_adcLatest = HAL_ADC_GetValue(&hadc1);
    }
}

/* UART RX callback */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        uint8_t rxByte = uartRxBuffer[0];

        if (rxByte == '\n' || rxByte == '\r') {
            cmdBuffer[cmdIndex] = '\0';

            // Parse command
            if (strcmp(cmdBuffer, "L1_ON") == 0) {
                g_cmdL1_on = 1;
            }
            else if (strcmp(cmdBuffer, "L1_OFF") == 0) {
                g_cmdL1_off = 1;
            }
            else if (strcmp(cmdBuffer, "L2_ON") == 0) {
                g_cmdL2_on = 1;
            }
            else if (strcmp(cmdBuffer, "L2_OFF") == 0) {
                g_cmdL2_off = 1;
            }

            cmdIndex = 0;
        }
        else if (cmdIndex < sizeof(cmdBuffer) - 1) {
            cmdBuffer[cmdIndex++] = rxByte;
        }

        HAL_UART_Receive_IT(&huart2, uartRxBuffer, 1);
    }
}



uint8_t ButtonPressedDebounced(GPIO_TypeDef *port, uint16_t pin,
                               uint8_t *prevState, uint32_t *lastTime)
{
    uint8_t raw = (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 1 : 0;
    uint32_t now = HAL_GetTick();
    uint8_t event = 0;

    if (raw != *prevState) {
        if ((now - *lastTime) > 50) {
            *prevState = raw;
            *lastTime = now;
            if (raw == 1) {
                event = 1;
            }
        }
    }
    return event;
}

void Relay1_On(void)  { HAL_GPIO_WritePin(Relay1_GPIO_Port, Relay1_Pin, GPIO_PIN_SET); }
void Relay1_Off(void) { HAL_GPIO_WritePin(Relay1_GPIO_Port, Relay1_Pin, GPIO_PIN_RESET); }
void Relay2_On(void)  { HAL_GPIO_WritePin(Relay2_GPIO_Port, Relay2_Pin, GPIO_PIN_SET); }
void Relay2_Off(void) { HAL_GPIO_WritePin(Relay2_GPIO_Port, Relay2_Pin, GPIO_PIN_RESET); }

void LV8153_WritePattern(uint8_t pattern)
{
    HAL_UART_Transmit(&huart6, &pattern, 1, 10);
}

void RTOS_InitResources(void)
{
    g_sensorQueue = RTOS_CreateQueue();
    g_uartSemaphore = RTOS_CreateSemaphore(1, 1);
    HAL_UART_Receive_IT(&huart2, uartRxBuffer, 1);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_USART6_UART_Init();

  /* USER CODE BEGIN 2 */

  // Start ADC trigger timer (TIM2 @ 1kHz) and ADC with interrupt
  HAL_TIM_Base_Start(&htim2);
  HAL_ADC_Start_IT(&hadc1);

  // Initialize custom RTOS with 1 ms tick
  RTOS_Init(1);

  // Initialize RTOS resources (queues, semaphores, UART RX)
  RTOS_InitResources();

  // Add tasks with priority (0 = highest)
  RTOS_AddTask(SensorTask,    NULL, 10,   0, 0);   // 10 ms, priority 0
  RTOS_AddTask(ControlTask,   NULL, 20,   0, 1);   // 20 ms, priority 1
  RTOS_AddTask(UartTxTask,    NULL, 1000, 0, 2);   // 1 s, priority 2
  RTOS_AddTask(HeartbeatTask, NULL, 500,  0, 3);   // 0.5 s, priority 3

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      RTOS_RunScheduler();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
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
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
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
}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{
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
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Relay1_Pin|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = BTN_ALL_Pin|BTN_L1_Pin|BTN_L2_Pin;  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Relay1_Pin PB1 */
  GPIO_InitStruct.Pin = Relay1_Pin|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */


void SensorTask(void *param)
{
    (void)param;

    uint16_t adcRaw = g_adcLatest;
    float adcVoltage = (adcRaw * 3.3f) / 4095.0f;

    const float ACS712_ZERO_CURRENT_V = 2.5f;
    const float ACS712_SENSITIVITY = 0.1f;  // V/A for ACS712-20A

    float currentOffset = adcVoltage - ACS712_ZERO_CURRENT_V;
    g_current_A = currentOffset / ACS712_SENSITIVITY;

    if (g_current_A < 0.05f && g_current_A > -0.05f) {
        g_current_A = 0.0f;
    }

    g_voltage_V = 120.0f;
    float power_W = g_current_A * g_voltage_V;

    RTOS_EnterCritical();
    g_latestSensorData.current_A = g_current_A;
    g_latestSensorData.voltage_V = g_voltage_V;
    g_latestSensorData.power_W = power_W;
    g_latestSensorData.timestamp = RTOS_GetTickCount();
    RTOS_ExitCritical();

    if (g_sensorQueue >= 0) {
        uint32_t packedData = (uint32_t)(g_current_A * 1000.0f);
        RTOS_QueueSend(g_sensorQueue, packedData);
    }
}

void ControlTask(void *param)
{
    (void)param;

    if (ButtonPressedDebounced(BTN_ALL_GPIO_Port, BTN_ALL_Pin,
                               &btnAll_prev, &btnAll_last))
    {
        Relay1_On();
        Relay2_On();
        g_relayPattern = 0x03;
        LV8153_WritePattern(g_relayPattern);
    }

    if (ButtonPressedDebounced(BTN_L1_GPIO_Port, BTN_L1_Pin,
                               &btnL1_prev, &btnL1_last))
    {
        if (HAL_GPIO_ReadPin(Relay1_GPIO_Port, Relay1_Pin) == GPIO_PIN_SET) {
            Relay1_Off();
            g_relayPattern &= ~0x01;
        } else {
            Relay1_On();
            g_relayPattern |= 0x01;
        }
        LV8153_WritePattern(g_relayPattern);
    }

    if (ButtonPressedDebounced(BTN_L2_GPIO_Port, BTN_L2_Pin,
                               &btnL2_prev, &btnL2_last))
    {
        if (HAL_GPIO_ReadPin(Relay2_GPIO_Port, Relay2_Pin) == GPIO_PIN_SET) {
            Relay2_Off();
            g_relayPattern &= ~0x02;
        } else {
            Relay2_On();
            g_relayPattern |= 0x02;
        }
        LV8153_WritePattern(g_relayPattern);
    }

    if (g_cmdL1_on) {
        g_cmdL1_on = 0;
        Relay1_On();
        g_relayPattern |= 0x01;
        LV8153_WritePattern(g_relayPattern);
    }

    if (g_cmdL1_off) {
        g_cmdL1_off = 0;
        Relay1_Off();
        g_relayPattern &= ~0x01;
        LV8153_WritePattern(g_relayPattern);
    }

    if (g_cmdL2_on) {
        g_cmdL2_on = 0;
        Relay2_On();
        g_relayPattern |= 0x02;
        LV8153_WritePattern(g_relayPattern);
    }

    if (g_cmdL2_off) {
        g_cmdL2_off = 0;
        Relay2_Off();
        g_relayPattern &= ~0x02;
        LV8153_WritePattern(g_relayPattern);
    }

    if (g_latestSensorData.power_W > 1000.0f) {
        Relay2_Off();
        g_relayPattern &= ~0x02;
        LV8153_WritePattern(g_relayPattern);
    }
}

void UartTxTask(void *param)
{
    (void)param;

    char jsonBuffer[256];
    int len;

    if (g_uartSemaphore >= 0) {
        if (!RTOS_SemaphoreTake(g_uartSemaphore)) {
            return;
        }
    }

    RTOS_EnterCritical();
    float current = g_latestSensorData.current_A;
    float voltage = g_latestSensorData.voltage_V;
    float power = g_latestSensorData.power_W;
    uint32_t timestamp = g_latestSensorData.timestamp;
    RTOS_ExitCritical();

    uint8_t relay1 = (HAL_GPIO_ReadPin(Relay1_GPIO_Port, Relay1_Pin) == GPIO_PIN_SET) ? 1 : 0;
    uint8_t relay2 = (HAL_GPIO_ReadPin(Relay2_GPIO_Port, Relay2_Pin) == GPIO_PIN_SET) ? 1 : 0;

    len = snprintf(jsonBuffer, sizeof(jsonBuffer),
                   "{\"current\":%.3f,\"voltage\":%.1f,\"power\":%.2f,"
                   "\"relay1\":%d,\"relay2\":%d,\"time\":%lu}\n",
                   current, voltage, power, relay1, relay2, timestamp);

    HAL_UART_Transmit(&huart2, (uint8_t*)jsonBuffer, len, 100);

    if (g_uartSemaphore >= 0) {
        RTOS_SemaphoreGive(g_uartSemaphore);
    }
}

void HeartbeatTask(void *param)
{
    (void)param;
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}

/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif
