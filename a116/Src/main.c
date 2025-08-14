/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : A1-16 Servo Motor Control with STM32F446RE
  ******************************************************************************
  * @attention
  *
  * A1-16智能舵機控制程序
  * 功能：簡單單方向旋轉控制
  * 硬體：STM32 Nucleo-F446RE + A1-16舵機
  * 連接：PB6(TX), PB7(RX) - 5V tolerant引腳
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// A1-16 命令定義
#define CMD_I_JOG       0x05    // 位置控制命令
#define CMD_STAT        0x07    // 狀態讀取命令

// 控制模式
#define MODE_POSITION   0       // 位置控制模式
#define MODE_SPEED      1       // 速度控制模式

// 舵機參數
#define SERVO_ID        2       // 舵機ID (根據您的舵機標籤)
#define MIN_POSITION    100     // 最小位置
#define MAX_POSITION    900     // 最大位置
#define CENTER_POSITION 512     // 中心位置
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t servo_id = SERVO_ID;
uint16_t current_target = CENTER_POSITION;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
// A1-16 控制函數
uint8_t calculate_checksum1(uint8_t* data, uint8_t length);
uint8_t calculate_checksum2(uint8_t checksum1);
HAL_StatusTypeDef send_a116_command(uint8_t id, uint8_t cmd, uint8_t* data, uint8_t data_length);
HAL_StatusTypeDef a116_set_position(uint8_t id, uint16_t position, uint8_t playtime);
HAL_StatusTypeDef a116_read_status(uint8_t id);
void print_message(char* message);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * 計算校驗和1
 */
uint8_t calculate_checksum1(uint8_t* data, uint8_t length)
{
    uint8_t checksum = 0;
    for (int i = 0; i < length; i++)
    {
        checksum ^= data[i];
    }
    return checksum & 0xFE;
}

/**
 * 計算校驗和2
 */
uint8_t calculate_checksum2(uint8_t checksum1)
{
    return (~checksum1) & 0xFE;
}

/**
 * 發送A1-16命令
 */
HAL_StatusTypeDef send_a116_command(uint8_t id, uint8_t cmd, uint8_t* data, uint8_t data_length)
{
    uint8_t packet[108];
    uint8_t packet_size = 7 + data_length;

    // 構建數據包
    packet[0] = 0xFF;  // Header 1
    packet[1] = 0xFF;  // Header 2
    packet[2] = packet_size;  // Packet size
    packet[3] = id;    // Servo ID
    packet[4] = cmd;   // Command

    // 複製數據
    if (data != NULL && data_length > 0)
    {
        memcpy(&packet[7], data, data_length);
    }

    // 計算校驗和
    uint8_t checksum_data[100];
    checksum_data[0] = packet_size;
    checksum_data[1] = id;
    checksum_data[2] = cmd;

    for (int i = 0; i < data_length; i++)
    {
        checksum_data[3 + i] = data[i];
    }

    packet[5] = calculate_checksum1(checksum_data, 3 + data_length);
    packet[6] = calculate_checksum2(packet[5]);

    // 發送數據包
    return HAL_UART_Transmit(&huart1, packet, packet_size, 1000);
}

/**
 * 設置舵機位置
 */
HAL_StatusTypeDef a116_set_position(uint8_t id, uint16_t position, uint8_t playtime)
{
    uint8_t data[5];

    // I-JOG 數據格式
    data[0] = position & 0xFF;        // 目標位置 LSB
    data[1] = (position >> 8) & 0xFF; // 目標位置 MSB
    data[2] = MODE_POSITION;          // 控制模式
    data[3] = id;                     // 舵機ID
    data[4] = playtime;               // 運動時間 (單位: 10ms)

    return send_a116_command(id, CMD_I_JOG, data, 5);
}

/**
 * 讀取舵機狀態
 */
HAL_StatusTypeDef a116_read_status(uint8_t id)
{
    HAL_StatusTypeDef status;
    uint8_t response[20];

    // 發送狀態讀取命令
    status = send_a116_command(id, CMD_STAT, NULL, 0);
    if (status != HAL_OK) return status;

    // 接收響應
    status = HAL_UART_Receive(&huart1, response, 17, 1000);
    if (status == HAL_OK)
    {
        // 解析狀態信息
        uint8_t status_error = response[7];
        uint8_t status_detail = response[8];
        uint16_t position = response[12] | (response[13] << 8);

        char msg[100];
        sprintf(msg, "狀態: 錯誤=0x%02X, 詳情=0x%02X, 位置=%d\r\n",
                status_error, status_detail, position);
        print_message(msg);
    }

    return status;
}

/**
 * 發送調試信息到PC
 */
void print_message(char* message)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 1000);
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  // 發送啟動信息
  print_message("\r\n=== A1-16舵機控制系統啟動 ===\r\n");
  print_message("硬體: STM32F446RE + A1-16智能舵機\r\n");
  print_message("功能: 單方向旋轉控制\r\n");
  print_message("按藍色按鈕開始運動...\r\n\r\n");

  // 等待舵機初始化完成 (LED閃爍序列)
  print_message("等待舵機初始化...\r\n");
  HAL_Delay(3000);

  // 測試舵機通信
  print_message("測試舵機通信...\r\n");
  if (a116_read_status(servo_id) == HAL_OK)
  {
      print_message("✅ 舵機通信成功！\r\n");
  }
  else
  {
      print_message("❌ 舵機通信失敗，請檢查連接\r\n");
  }

  // 移動到中心位置
  print_message("移動到中心位置...\r\n");
  a116_set_position(servo_id, CENTER_POSITION, 100);
  HAL_Delay(2000);

  print_message("初始化完成！準備開始旋轉\r\n\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // 檢查按鈕是否按下
    if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
    {
        // 按鈕被按下，開始單方向旋轉
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); // 點亮LED

        print_message("🔄 開始單方向旋轉...\r\n");

        // 單方向旋轉序列：中心 → 右端 → 中心 → 左端 → 中心
        uint16_t positions[] = {CENTER_POSITION, MAX_POSITION, CENTER_POSITION, MIN_POSITION, CENTER_POSITION};
        char* position_names[] = {"中心", "右端", "中心", "左端", "中心"};

        for (int i = 0; i < 5; i++)
        {
            char msg[50];
            sprintf(msg, "移動到: %s (位置=%d)\r\n", position_names[i], positions[i]);
            print_message(msg);

            // 發送位置命令
            a116_set_position(servo_id, positions[i], 100); // 1秒運動時間

            // 等待運動完成
            HAL_Delay(1500);

            // 讀取當前狀態
            a116_read_status(servo_id);

            // 暫停一下
            HAL_Delay(500);
        }

        print_message("單方向旋轉完成！\r\n\r\n");
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // 關閉LED

        // 等待按鈕釋放
        while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
        {
            HAL_Delay(10);
        }
        HAL_Delay(200); // 防抖動
    }

    // 連續旋轉模式 (可選 - 註解掉按鈕控制部分，啟用此部分)
    /*
    // LED閃爍指示運行狀態
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

    // 緩慢單方向旋轉
    current_target += 10;
    if (current_target > MAX_POSITION)
    {
        current_target = MIN_POSITION;
        print_message("循環旋轉：重新開始\r\n");
    }

    // 發送位置命令
    a116_set_position(servo_id, current_target, 20); // 200ms運動時間

    // 短暫延遲
    HAL_Delay(300);
    */

    HAL_Delay(100); // 主循環延遲
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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
