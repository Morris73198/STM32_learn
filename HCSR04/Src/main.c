/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : HC-SR04 Ultrasonic Distance Sensor (DWT Version)
  * @description    : 使用 DWT 計數器提供微秒級計時精度
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CPU_FREQ_MHZ    84  // STM32F446 系統時鐘頻率 (MHz)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// HC-SR04 相關變數
float distance_cm = 0.0;
char output_buffer[100];

// 測量參數
uint32_t measurement_counter = 0;
uint32_t last_measurement_time = 0;

// 濾波和穩定性變數
#define FILTER_SIZE 3
float distance_history[FILTER_SIZE] = {0};
uint8_t history_index = 0;
uint8_t history_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
void DWT_Init(void);
uint32_t DWT_Get_Microseconds(void);
void DWT_Delay_us(uint32_t us);
float HC_SR04_Measure_Distance_DWT(void);
float Apply_Distance_Filter(float new_distance);
float HC_SR04_Measure_With_Retry(void);
void Display_Distance(float distance);
void System_Status_LED(void);

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
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  // 初始化 DWT 微秒計時器
  DWT_Init();

  // 系統初始化訊息
  HAL_UART_Transmit(&huart2, (uint8_t*)"HC-SR04 Ultrasonic Distance Sensor\r\n", 36, 1000);
  HAL_UART_Transmit(&huart2, (uint8_t*)"DWT Version - Microsecond Precision\r\n", 37, 1000);
  HAL_UART_Transmit(&huart2, (uint8_t*)"System Initialized\r\n", 20, 1000);
  HAL_UART_Transmit(&huart2, (uint8_t*)"Starting measurements...\r\n", 26, 1000);

  HAL_Delay(1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t current_time = HAL_GetTick();

    // 每 1000ms 進行一次測量
    if(current_time - last_measurement_time >= 1000)
    {
      last_measurement_time = current_time;
      measurement_counter++;

      // 執行 HC-SR04 測量 (DWT 版本 - 帶重試和濾波)
      distance_cm = HC_SR04_Measure_With_Retry();

      // 顯示結果
      Display_Distance(distance_cm);
    }

    // 系統狀態 LED
    System_Status_LED();

    // 短暫延遲
    HAL_Delay(100);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_8;  // PA5=LED, PA8=Trig
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;  // PA9=Echo
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief 初始化 DWT (Data Watchpoint and Trace) 計數器
  * @param None
  * @retval None
  * @description DWT 是 Cortex-M4 內建的調試單元，可提供高精度計時
  */
void DWT_Init(void)
{
  // 啟用 DWT 和 ITM 單元
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  // 重置計數器
  DWT->CYCCNT = 0;

  // 啟用計數器
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
  * @brief 獲取當前微秒時間戳
  * @param None
  * @retval 微秒時間戳
  * @description 將 CPU 週期轉換為微秒
  */
uint32_t DWT_Get_Microseconds(void)
{
  return DWT->CYCCNT / CPU_FREQ_MHZ;
}

/**
  * @brief DWT 微秒精確延遲
  * @param us: 延遲微秒數
  * @retval None
  */
void DWT_Delay_us(uint32_t us)
{
  uint32_t start_cycles = DWT->CYCCNT;
  uint32_t target_cycles = start_cycles + (us * CPU_FREQ_MHZ);

  // 等待到達目標週期數
  while (DWT->CYCCNT < target_cycles);
}

/**
  * @brief HC-SR04 測距函數 (DWT 版本 - 改進穩定性)
  * @param None
  * @retval 距離 (公分)
  */
float HC_SR04_Measure_Distance_DWT(void)
{
  uint32_t start_time, end_time, pulse_duration;
  uint32_t timeout_start;

  // 確保 Trig 腳初始為低電平
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  DWT_Delay_us(2);  // 等待穩定

  // 1. 發送觸發脈衝
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);   // Trig 高電平
  DWT_Delay_us(10);                                     // 精確等待 10μs
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); // Trig 低電平

  // 2. 等待 Echo 上升沿 (超時 50ms = 50000μs，增加超時時間)
  timeout_start = DWT_Get_Microseconds();
  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_RESET)
  {
    uint32_t elapsed = DWT_Get_Microseconds() - timeout_start;
    if(elapsed > 50000)  // 增加到 50ms
    {
      return -1;  // 超時錯誤
    }
  }
  start_time = DWT_Get_Microseconds();

  // 3. 等待 Echo 下降沿 (超時 50ms = 50000μs)
  timeout_start = DWT_Get_Microseconds();
  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET)
  {
    uint32_t elapsed = DWT_Get_Microseconds() - timeout_start;
    if(elapsed > 50000)  // 增加到 50ms
    {
      return -2;  // 超時錯誤
    }
  }
  end_time = DWT_Get_Microseconds();

  // 4. 計算脈衝寬度 (微秒)
  if (end_time >= start_time) {
    pulse_duration = end_time - start_time;
  } else {
    // 處理計數器溢出
    pulse_duration = (0xFFFFFFFF / CPU_FREQ_MHZ - start_time) + end_time + 1;
  }

  // 5. 檢查脈衝寬度合理性 (150μs ~ 25000μs 對應 2.5cm ~ 430cm)
  if(pulse_duration < 150 || pulse_duration > 25000)
  {
    return -3;  // 脈衝寬度異常
  }

  // 6. 轉換為距離 (cm)
  // 聲速 = 343 m/s = 0.0343 cm/μs
  // 距離 = (脈衝寬度 * 聲速) / 2
  float distance = (float)pulse_duration * 0.0343 / 2.0;

  return distance;
}

/**
  * @brief 移動平均濾波器
  * @param new_distance: 新的距離值
  * @retval 濾波後的距離值
  */
float Apply_Distance_Filter(float new_distance)
{
  // 檢查距離是否有效
  if(new_distance < 0) {
    return new_distance;  // 錯誤值直接返回
  }

  // 加入歷史陣列
  distance_history[history_index] = new_distance;
  history_index = (history_index + 1) % FILTER_SIZE;

  if(history_count < FILTER_SIZE) {
    history_count++;
  }

  // 計算平均值
  float sum = 0;
  for(int i = 0; i < history_count; i++) {
    sum += distance_history[i];
  }

  return sum / history_count;
}

/**
  * @brief HC-SR04 測量with重試機制
  * @param None
  * @retval 距離 (公分)
  */
float HC_SR04_Measure_With_Retry(void)
{
  float distance;
  uint8_t retry_count = 0;
  const uint8_t max_retries = 3;

  do {
    distance = HC_SR04_Measure_Distance_DWT();

    // 如果測量成功，套用濾波器
    if(distance > 0) {
      return Apply_Distance_Filter(distance);
    }

    retry_count++;
    if(retry_count < max_retries) {
      HAL_Delay(10);  // 重試前短暫等待
    }

  } while(retry_count < max_retries);

  return distance;  // 返回最後的錯誤碼
}

/**
  * @brief 顯示距離結果 (改進版 - 處理所有錯誤類型)
  * @param distance: 距離值
  * @retval None
  */
void Display_Distance(float distance)
{
  if(distance == -1)
  {
    snprintf(output_buffer, sizeof(output_buffer),
            "Measurement %lu: TIMEOUT - No echo start (檢查接線)\r\n", measurement_counter);
  }
  else if(distance == -2)
  {
    snprintf(output_buffer, sizeof(output_buffer),
            "Measurement %lu: TIMEOUT - No echo end (距離太遠?)\r\n", measurement_counter);
  }
  else if(distance == -3)
  {
    snprintf(output_buffer, sizeof(output_buffer),
            "Measurement %lu: INVALID - Pulse width error (干擾?)\r\n", measurement_counter);
  }
  else if(distance > 0 && distance < 400)
  {
    // 顯示到小數點後 2 位 (足夠精確)
    int integer_part = (int)distance;
    int decimal_part = (int)((distance - integer_part) * 100);

    snprintf(output_buffer, sizeof(output_buffer),
            "Measurement %lu: Distance = %d.%02d cm (filtered)\r\n",
            measurement_counter, integer_part, decimal_part);
  }
  else
  {
    snprintf(output_buffer, sizeof(output_buffer),
            "Measurement %lu: Out of range (%.2f cm)\r\n",
            measurement_counter, distance);
  }

  HAL_UART_Transmit(&huart2, (uint8_t*)output_buffer, strlen(output_buffer), 1000);
}

/**
  * @brief 系統狀態 LED 控制
  * @param None
  * @retval None
  */
void System_Status_LED(void)
{
  static uint32_t led_timer = 0;
  static uint8_t led_state = 0;

  led_timer++;

  // 每 10 次循環切換 LED (約 1000ms)
  if(led_timer >= 10)
  {
    led_timer = 0;
    led_state = !led_state;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
