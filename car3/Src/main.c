/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 麥克納姆輪車控制程序 - 完整版含USART3測試，已修復ID衝突
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
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// A1-16舵機控制結構體
typedef struct {
    uint8_t id;
    UART_HandleTypeDef *huart;
    uint8_t rx_buffer[100];
    uint8_t tx_buffer[100];
    bool response_received;
    char name[20];  // 輪子名稱
    int16_t current_speed;  // 當前速度
} A1_16_Handle;

typedef struct {
    uint16_t position;
    uint16_t speed;
    uint8_t temperature;
    uint8_t voltage;
    uint16_t current;
    uint8_t status_error;
    uint8_t status_detail;
} A1_16_Status;

// 麥克納姆輪車結構體
typedef struct {
    A1_16_Handle front_right;  // 右前輪 USART1, ID=7
    A1_16_Handle front_left;   // 左前輪 USART1, ID=8
    A1_16_Handle rear_left;    // 左後輪 USART3, ID=9
    A1_16_Handle rear_right;   // 右後輪 USART3, ID=10
    float wheel_radius;        // 輪子半徑 (mm)
    float wheel_base;          // 軸距 (mm)
    float track_width;         // 輪距 (mm)
    bool initialized;          // 初始化狀態
} MecanumCar;

// 運動狀態枚舉
typedef enum {
    CAR_STOP = 0,
    CAR_FORWARD,
    CAR_BACKWARD,
    CAR_STRAFE_LEFT,
    CAR_STRAFE_RIGHT,
    CAR_ROTATE_LEFT,
    CAR_ROTATE_RIGHT,
    CAR_DIAGONAL_FL,
    CAR_DIAGONAL_FR,
    CAR_DIAGONAL_BL,
    CAR_DIAGONAL_BR,
    CAR_CUSTOM
} CarMovementState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// A1-16 命令定義
#define A1_16_CMD_EEP_WRITE    0x01
#define A1_16_CMD_EEP_READ     0x02
#define A1_16_CMD_RAM_WRITE    0x03
#define A1_16_CMD_RAM_READ     0x04
#define A1_16_CMD_I_JOG        0x05
#define A1_16_CMD_S_JOG        0x06
#define A1_16_CMD_STAT         0x07
#define A1_16_CMD_ROLLBACK     0x08
#define A1_16_CMD_REBOOT       0x09

// 控制模式
#define A1_16_MODE_POSITION    0
#define A1_16_MODE_SPEED       1
#define A1_16_MODE_TORQUE_OFF  2
#define A1_16_MODE_SERVO_ON    3

// 麥克納姆輪參數
#define WHEEL_RADIUS    50.0f   // mm
#define WHEEL_BASE      200.0f  // mm
#define TRACK_WIDTH     180.0f  // mm
#define MAX_SPEED       100     // 最大速度值
#define MIN_SPEED       10      // 最小有效速度

// 調試開關
#define DEBUG_ENABLED   1
#define DEBUG_SERVO_STATUS  1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#if DEBUG_ENABLED
#define DEBUG_PRINTF(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define DEBUG_PRINTF(fmt, ...)
#endif
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
MecanumCar car;
CarMovementState current_state = CAR_STOP;
uint32_t last_command_time = 0;
uint8_t test_mode = 0;  // 0=自動測試, 1=單輪測試, 2=基本運動, 3=高級運動
bool button_pressed = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

// A1-16舵機控制函數
HAL_StatusTypeDef A1_16_Init(A1_16_Handle *handle, UART_HandleTypeDef *huart, uint8_t id, const char* name);
HAL_StatusTypeDef A1_16_SetSpeed(A1_16_Handle *handle, int16_t speed);
HAL_StatusTypeDef A1_16_GetStatus(A1_16_Handle *handle, A1_16_Status *status);
HAL_StatusTypeDef A1_16_TorqueOff(A1_16_Handle *handle);
HAL_StatusTypeDef A1_16_TorqueOn(A1_16_Handle *handle);
HAL_StatusTypeDef A1_16_SetID(A1_16_Handle *handle, uint8_t old_id, uint8_t new_id);
HAL_StatusTypeDef A1_16_Reboot(A1_16_Handle *handle);

// 內部函數
static HAL_StatusTypeDef A1_16_SendPacket(A1_16_Handle *handle, uint8_t cmd, uint8_t *data, uint8_t data_len);
static HAL_StatusTypeDef A1_16_ReceiveResponse(A1_16_Handle *handle, uint8_t expected_cmd, uint32_t timeout);
static uint8_t A1_16_CalculateChecksum1(uint8_t *data, uint8_t length);
static uint8_t A1_16_CalculateChecksum2(uint8_t checksum1);

// 舵機ID設置函數
HAL_StatusTypeDef Setup_ServoIDs(void);

// 麥克納姆輪運動控制函數
void Mecanum_Init(MecanumCar *car);
void Mecanum_SetVelocity(MecanumCar *car, float vx, float vy, float omega);
void Mecanum_Forward(MecanumCar *car, int16_t speed);
void Mecanum_Backward(MecanumCar *car, int16_t speed);
void Mecanum_StrafeLeft(MecanumCar *car, int16_t speed);
void Mecanum_StrafeRight(MecanumCar *car, int16_t speed);
void Mecanum_RotateLeft(MecanumCar *car, int16_t speed);
void Mecanum_RotateRight(MecanumCar *car, int16_t speed);
void Mecanum_Stop(MecanumCar *car);
void Mecanum_DiagonalFrontLeft(MecanumCar *car, int16_t speed);
void Mecanum_DiagonalFrontRight(MecanumCar *car, int16_t speed);
void Mecanum_DiagonalBackLeft(MecanumCar *car, int16_t speed);
void Mecanum_DiagonalBackRight(MecanumCar *car, int16_t speed);

// 測試和調試函數
void Test_IndividualWheel(A1_16_Handle *wheel, int16_t speed, uint32_t duration);
void Test_AllWheels(MecanumCar *car);
void Test_BasicMovements(MecanumCar *car);
void Test_AdvancedMovements(MecanumCar *car);
void Check_ServoStatus(A1_16_Handle *handle);
void Check_AllServos(MecanumCar *car);
void Scan_ServoIDs(UART_HandleTypeDef *huart, const char* uart_name);

// USART3 測試函數
void Test_USART3_Loopback(void);
void Test_USART3_Raw(void);

// 系統函數
void System_Info(void);
void Change_TestMode(void);
void Handle_ButtonPress(void);

// 中斷回調函數
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 重定向printf到UART2
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

// USART3 測試函數
void Test_USART3_Loopback(void) {
    DEBUG_PRINTF("測試USART3迴路...\r\n");

    // 暫時短接PB10和PC5進行迴路測試
    uint8_t test_data[] = "TEST";
    uint8_t rx_data[10] = {0};

    DEBUG_PRINTF("發送數據: %s\r\n", test_data);
    HAL_StatusTypeDef tx_result = HAL_UART_Transmit(&huart3, test_data, 4, 100);
    DEBUG_PRINTF("發送結果: %s\r\n", (tx_result == HAL_OK) ? "成功" : "失敗");

    HAL_StatusTypeDef rx_result = HAL_UART_Receive(&huart3, rx_data, 4, 1000);
    DEBUG_PRINTF("接收結果: %s\r\n", (rx_result == HAL_OK) ? "成功" : "超時");

    if (rx_result == HAL_OK) {
        DEBUG_PRINTF("接收到的數據: %s\r\n", rx_data);
        if (memcmp(test_data, rx_data, 4) == 0) {
            DEBUG_PRINTF("✓ USART3迴路測試: 成功\r\n");
        } else {
            DEBUG_PRINTF("✗ USART3迴路測試: 數據不匹配\r\n");
        }
    } else {
        DEBUG_PRINTF("✗ USART3迴路測試: 失敗\r\n");
    }
}

void Test_USART3_Raw(void) {
    DEBUG_PRINTF("測試USART3原始通訊...\r\n");

    // A1-16基本狀態查詢命令 (ID=9)
    uint8_t test_cmd[] = {0xFF, 0xFF, 0x07, 0x09, 0x07, 0x00, 0x00};
    HAL_StatusTypeDef tx_result = HAL_UART_Transmit(&huart3, test_cmd, 7, 100);
    DEBUG_PRINTF("USART3 發送A1-16命令結果: %s\r\n", (tx_result == HAL_OK) ? "成功" : "失敗");

    uint8_t rx_buffer[50];
    HAL_StatusTypeDef rx_result = HAL_UART_Receive(&huart3, rx_buffer, 50, 500);
    DEBUG_PRINTF("USART3 接收A1-16響應: %s\r\n", (rx_result == HAL_OK) ? "有響應" : "無響應");

    if (rx_result == HAL_OK) {
        DEBUG_PRINTF("響應數據: ");
        for (int i = 0; i < 10; i++) {
            DEBUG_PRINTF("0x%02X ", rx_buffer[i]);
        }
        DEBUG_PRINTF("\r\n");
    }
}

// 設置舵機ID的專用函數
HAL_StatusTypeDef Setup_ServoIDs(void) {
    DEBUG_PRINTF("\r\n=== 設置舵機ID ===\r\n");

    A1_16_Handle temp_handle;
    HAL_StatusTypeDef result;

    // 初始化臨時處理器用於USART3上的舵機
    // 假設後輪舵機當前ID是7和8，需要改為9和10

    // 將USART3上的ID=7改為ID=9 (左後輪)
    DEBUG_PRINTF("正在將USART3上的ID=7改為ID=9...\r\n");
    A1_16_Init(&temp_handle, &huart3, 7, "TempHandle");
    result = A1_16_SetID(&temp_handle, 7, 9);
    if (result == HAL_OK) {
        DEBUG_PRINTF("✓ 成功將舵機ID從7改為9\r\n");
        HAL_Delay(1000);  // 等待舵機重新啟動
    } else {
        DEBUG_PRINTF("✗ 設置舵機ID=9失敗\r\n");
        return result;
    }

    // 將USART3上的ID=8改為ID=10 (右後輪)
    DEBUG_PRINTF("正在將USART3上的ID=8改為ID=10...\r\n");
    A1_16_Init(&temp_handle, &huart3, 8, "TempHandle");
    result = A1_16_SetID(&temp_handle, 8, 10);
    if (result == HAL_OK) {
        DEBUG_PRINTF("✓ 成功將舵機ID從8改為10\r\n");
        HAL_Delay(1000);  // 等待舵機重新啟動
    } else {
        DEBUG_PRINTF("✗ 設置舵機ID=10失敗\r\n");
        return result;
    }

    DEBUG_PRINTF("=== 舵機ID設置完成 ===\r\n");
    return HAL_OK;
}

// A1-16舵機初始化
HAL_StatusTypeDef A1_16_Init(A1_16_Handle *handle, UART_HandleTypeDef *huart, uint8_t id, const char* name) {
    handle->huart = huart;
    handle->id = id;
    handle->response_received = false;
    handle->current_speed = 0;
    strncpy(handle->name, name, sizeof(handle->name) - 1);
    handle->name[sizeof(handle->name) - 1] = '\0';

    memset(handle->rx_buffer, 0, sizeof(handle->rx_buffer));
    memset(handle->tx_buffer, 0, sizeof(handle->tx_buffer));

    return HAL_OK;
}

// 設置舵機速度
HAL_StatusTypeDef A1_16_SetSpeed(A1_16_Handle *handle, int16_t speed) {
    uint8_t data[5];
    uint16_t speed_val;

    // 記錄當前速度
    handle->current_speed = speed;

    // 轉換速度值 (負數表示反向)
    if (speed < 0) {
        speed_val = (uint16_t)(-speed);
        speed_val |= 0x8000; // 設置方向位
    } else {
        speed_val = (uint16_t)speed;
    }

    data[0] = speed_val & 0xFF;
    data[1] = (speed_val >> 8) & 0xFF;
    data[2] = A1_16_MODE_SPEED;
    data[3] = handle->id;
    data[4] = 0;

    return A1_16_SendPacket(handle, A1_16_CMD_I_JOG, data, 5);
}

// 獲取舵機狀態
HAL_StatusTypeDef A1_16_GetStatus(A1_16_Handle *handle, A1_16_Status *status) {
    HAL_StatusTypeDef ret;

    ret = A1_16_SendPacket(handle, A1_16_CMD_STAT, NULL, 0);
    if (ret != HAL_OK) return ret;

    ret = A1_16_ReceiveResponse(handle, A1_16_CMD_STAT + 0x40, 100);
    if (ret != HAL_OK) return ret;

    if (handle->rx_buffer[2] >= 17) {
        status->status_error = handle->rx_buffer[7];
        status->status_detail = handle->rx_buffer[8];
        status->position = handle->rx_buffer[13] | (handle->rx_buffer[14] << 8);
        status->current = handle->rx_buffer[15] | (handle->rx_buffer[16] << 8);
        status->temperature = handle->rx_buffer[17];
        status->voltage = handle->rx_buffer[18];
    }

    return HAL_OK;
}

// 關閉扭矩
HAL_StatusTypeDef A1_16_TorqueOff(A1_16_Handle *handle) {
    uint8_t data[5] = {0, 0, A1_16_MODE_TORQUE_OFF, handle->id, 0};
    handle->current_speed = 0;
    return A1_16_SendPacket(handle, A1_16_CMD_I_JOG, data, 5);
}

// 開啟扭矩
HAL_StatusTypeDef A1_16_TorqueOn(A1_16_Handle *handle) {
    uint8_t data[5] = {0, 0, A1_16_MODE_SERVO_ON, handle->id, 0};
    return A1_16_SendPacket(handle, A1_16_CMD_I_JOG, data, 5);
}

// 設置舵機ID
HAL_StatusTypeDef A1_16_SetID(A1_16_Handle *handle, uint8_t old_id, uint8_t new_id) {
    uint8_t data[3];
    handle->id = old_id;

    data[0] = 6;        // EEPROM地址6是sID參數
    data[1] = 1;        // length
    data[2] = new_id;   // 新的ID值

    HAL_StatusTypeDef ret = A1_16_SendPacket(handle, A1_16_CMD_EEP_WRITE, data, 3);
    if (ret != HAL_OK) return ret;

    ret = A1_16_ReceiveResponse(handle, A1_16_CMD_EEP_WRITE + 0x40, 500);
    if (ret != HAL_OK) return ret;

    handle->id = new_id;
    return HAL_OK;
}

// 重啟舵機
HAL_StatusTypeDef A1_16_Reboot(A1_16_Handle *handle) {
    HAL_StatusTypeDef ret = A1_16_SendPacket(handle, A1_16_CMD_REBOOT, NULL, 0);
    if (ret != HAL_OK) return ret;
    return A1_16_ReceiveResponse(handle, A1_16_CMD_REBOOT + 0x40, 500);
}

// 計算校驗和函數
static uint8_t A1_16_CalculateChecksum1(uint8_t *data, uint8_t length) {
    uint8_t checksum = 0;
    for (int i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum & 0xFE;
}

static uint8_t A1_16_CalculateChecksum2(uint8_t checksum1) {
    return (~checksum1) & 0xFE;
}

// 發送數據包
static HAL_StatusTypeDef A1_16_SendPacket(A1_16_Handle *handle, uint8_t cmd, uint8_t *data, uint8_t data_len) {
    uint8_t packet_size = 7 + data_len;
    uint8_t checksum_data[100];
    uint8_t checksum1, checksum2;

    handle->tx_buffer[0] = 0xFF;
    handle->tx_buffer[1] = 0xFF;
    handle->tx_buffer[2] = packet_size;
    handle->tx_buffer[3] = handle->id;
    handle->tx_buffer[4] = cmd;

    if (data && data_len > 0) {
        memcpy(&handle->tx_buffer[7], data, data_len);
    }

    checksum_data[0] = packet_size;
    checksum_data[1] = handle->id;
    checksum_data[2] = cmd;
    if (data && data_len > 0) {
        memcpy(&checksum_data[3], data, data_len);
    }

    checksum1 = A1_16_CalculateChecksum1(checksum_data, 3 + data_len);
    checksum2 = A1_16_CalculateChecksum2(checksum1);

    handle->tx_buffer[5] = checksum1;
    handle->tx_buffer[6] = checksum2;

    return HAL_UART_Transmit(handle->huart, handle->tx_buffer, packet_size, 100);
}

// 接收響應
static HAL_StatusTypeDef A1_16_ReceiveResponse(A1_16_Handle *handle, uint8_t expected_cmd, uint32_t timeout) {
    HAL_StatusTypeDef ret;
    uint32_t start_time = HAL_GetTick();

    while ((HAL_GetTick() - start_time) < timeout) {
        ret = HAL_UART_Receive(handle->huart, handle->rx_buffer, 100, 10);
        if (ret == HAL_OK) {
            if (handle->rx_buffer[0] == 0xFF &&
                handle->rx_buffer[1] == 0xFF &&
                handle->rx_buffer[3] == handle->id &&
                handle->rx_buffer[4] == expected_cmd) {
                return HAL_OK;
            }
        }
    }
    return HAL_TIMEOUT;
}

// 麥克納姆輪初始化 - 修改版本，使用新的舵機ID
void Mecanum_Init(MecanumCar *car) {
    DEBUG_PRINTF("\r\n=== 麥克納姆輪車初始化 ===\r\n");

    // 初始化各個輪子 - 注意後輪使用新的ID
    A1_16_Init(&car->front_right, &huart1, 7, "FrontRight");  // USART1, ID=7
    A1_16_Init(&car->front_left, &huart1, 8, "FrontLeft");    // USART1, ID=8
    A1_16_Init(&car->rear_left, &huart3, 9, "RearLeft");      // USART3, ID=9 (修改)
    A1_16_Init(&car->rear_right, &huart3, 10, "RearRight");   // USART3, ID=10 (修改)

    // 設置車輛參數
    car->wheel_radius = WHEEL_RADIUS;
    car->wheel_base = WHEEL_BASE;
    car->track_width = TRACK_WIDTH;
    car->initialized = false;

    HAL_Delay(500);

    // 檢查舵機連接
    DEBUG_PRINTF("檢查舵機連接...\r\n");
    Scan_ServoIDs(&huart1, "USART1(前輪)");
    Scan_ServoIDs(&huart3, "USART3(後輪)");

    // 啟用所有輪子的扭矩
    DEBUG_PRINTF("\r\n啟用舵機扭矩...\r\n");

    if (A1_16_TorqueOn(&car->front_right) == HAL_OK) {
        DEBUG_PRINTF("✓ 右前輪(USART1, ID=7)啟用成功\r\n");
    } else {
        DEBUG_PRINTF("✗ 右前輪(USART1, ID=7)啟用失敗！\r\n");
    }
    HAL_Delay(50);

    if (A1_16_TorqueOn(&car->front_left) == HAL_OK) {
        DEBUG_PRINTF("✓ 左前輪(USART1, ID=8)啟用成功\r\n");
    } else {
        DEBUG_PRINTF("✗ 左前輪(USART1, ID=8)啟用失敗！\r\n");
    }
    HAL_Delay(50);

    if (A1_16_TorqueOn(&car->rear_left) == HAL_OK) {
        DEBUG_PRINTF("✓ 左後輪(USART3, ID=9)啟用成功\r\n");
    } else {
        DEBUG_PRINTF("✗ 左後輪(USART3, ID=9)啟用失敗！\r\n");
    }
    HAL_Delay(50);

    if (A1_16_TorqueOn(&car->rear_right) == HAL_OK) {
        DEBUG_PRINTF("✓ 右後輪(USART3, ID=10)啟用成功\r\n");
    } else {
        DEBUG_PRINTF("✗ 右後輪(USART3, ID=10)啟用失敗！\r\n");
    }

    car->initialized = true;
    DEBUG_PRINTF("✓ 麥克納姆輪車初始化完成\r\n");
}

// 麥克納姆輪運動學控制（精確版本）
void Mecanum_SetVelocity(MecanumCar *car, float vx, float vy, float omega) {
    if (!car->initialized) return;

    float L = car->wheel_base / 2.0f;
    float W = car->track_width / 2.0f;

    // 麥克納姆輪運動學逆解
    float v_fr = vx - vy - (L + W) * omega;  // 右前
    float v_fl = vx + vy + (L + W) * omega;  // 左前
    float v_rl = vx - vy + (L + W) * omega;  // 左後
    float v_rr = vx + vy - (L + W) * omega;  // 右後

    // 轉換為舵機速度值並限制範圍
    int16_t speed_fr = (int16_t)fmax(-MAX_SPEED, fmin(MAX_SPEED, v_fr * MAX_SPEED / 100.0f));
    int16_t speed_fl = (int16_t)fmax(-MAX_SPEED, fmin(MAX_SPEED, v_fl * MAX_SPEED / 100.0f));
    int16_t speed_rl = (int16_t)fmax(-MAX_SPEED, fmin(MAX_SPEED, v_rl * MAX_SPEED / 100.0f));
    int16_t speed_rr = (int16_t)fmax(-MAX_SPEED, fmin(MAX_SPEED, v_rr * MAX_SPEED / 100.0f));

    // 發送速度命令
    A1_16_SetSpeed(&car->front_right, speed_fr);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->front_left, speed_fl);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->rear_left, speed_rl);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->rear_right, speed_rr);

    current_state = CAR_CUSTOM;
    last_command_time = HAL_GetTick();

    DEBUG_PRINTF("自定義運動: FR=%d FL=%d RL=%d RR=%d\r\n", speed_fr, speed_fl, speed_rl, speed_rr);
}

// 基本運動函數
void Mecanum_Forward(MecanumCar *car, int16_t speed) {
    if (!car->initialized) return;

    DEBUG_PRINTF("前進，速度: %d\r\n", speed);
    A1_16_SetSpeed(&car->front_right, speed);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->front_left, speed);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->rear_left, speed);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->rear_right, speed);

    current_state = CAR_FORWARD;
    last_command_time = HAL_GetTick();
}

void Mecanum_Backward(MecanumCar *car, int16_t speed) {
    if (!car->initialized) return;

    DEBUG_PRINTF("後退，速度: %d\r\n", speed);
    A1_16_SetSpeed(&car->front_right, -speed);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->front_left, -speed);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->rear_left, -speed);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->rear_right, -speed);

    current_state = CAR_BACKWARD;
    last_command_time = HAL_GetTick();
}

void Mecanum_StrafeLeft(MecanumCar *car, int16_t speed) {
    if (!car->initialized) return;

    DEBUG_PRINTF("左平移，速度: %d\r\n", speed);
    A1_16_SetSpeed(&car->front_right, -speed);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->front_left, speed);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->rear_left, -speed);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->rear_right, speed);

    current_state = CAR_STRAFE_LEFT;
    last_command_time = HAL_GetTick();
}

void Mecanum_StrafeRight(MecanumCar *car, int16_t speed) {
    if (!car->initialized) return;

    DEBUG_PRINTF("右平移，速度: %d\r\n", speed);
    A1_16_SetSpeed(&car->front_right, speed);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->front_left, -speed);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->rear_left, speed);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->rear_right, -speed);

    current_state = CAR_STRAFE_RIGHT;
    last_command_time = HAL_GetTick();
}

void Mecanum_RotateLeft(MecanumCar *car, int16_t speed) {
    if (!car->initialized) return;

    DEBUG_PRINTF("左旋轉，速度: %d\r\n", speed);
    A1_16_SetSpeed(&car->front_right, speed);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->front_left, -speed);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->rear_left, -speed);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->rear_right, speed);

    current_state = CAR_ROTATE_LEFT;
    last_command_time = HAL_GetTick();
}

void Mecanum_RotateRight(MecanumCar *car, int16_t speed) {
    if (!car->initialized) return;

    DEBUG_PRINTF("右旋轉，速度: %d\r\n", speed);
    A1_16_SetSpeed(&car->front_right, -speed);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->front_left, speed);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->rear_left, speed);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->rear_right, -speed);

    current_state = CAR_ROTATE_RIGHT;
    last_command_time = HAL_GetTick();
}

void Mecanum_DiagonalFrontLeft(MecanumCar *car, int16_t speed) {
    if (!car->initialized) return;

    DEBUG_PRINTF("左前斜移，速度: %d\r\n", speed);
    A1_16_SetSpeed(&car->front_right, 0);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->front_left, speed);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->rear_left, 0);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->rear_right, speed);

    current_state = CAR_DIAGONAL_FL;
    last_command_time = HAL_GetTick();
}

void Mecanum_DiagonalFrontRight(MecanumCar *car, int16_t speed) {
    if (!car->initialized) return;

    DEBUG_PRINTF("右前斜移，速度: %d\r\n", speed);
    A1_16_SetSpeed(&car->front_right, speed);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->front_left, 0);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->rear_left, speed);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->rear_right, 0);

    current_state = CAR_DIAGONAL_FR;
    last_command_time = HAL_GetTick();
}

void Mecanum_DiagonalBackLeft(MecanumCar *car, int16_t speed) {
    if (!car->initialized) return;

    DEBUG_PRINTF("左後斜移，速度: %d\r\n", speed);
    A1_16_SetSpeed(&car->front_right, -speed);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->front_left, 0);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->rear_left, -speed);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->rear_right, 0);

    current_state = CAR_DIAGONAL_BL;
    last_command_time = HAL_GetTick();
}

void Mecanum_DiagonalBackRight(MecanumCar *car, int16_t speed) {
    if (!car->initialized) return;

    DEBUG_PRINTF("右後斜移，速度: %d\r\n", speed);
    A1_16_SetSpeed(&car->front_right, 0);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->front_left, -speed);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->rear_left, 0);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->rear_right, -speed);

    current_state = CAR_DIAGONAL_BR;
    last_command_time = HAL_GetTick();
}

void Mecanum_Stop(MecanumCar *car) {
    if (!car->initialized) return;

    DEBUG_PRINTF("停止\r\n");
    A1_16_SetSpeed(&car->front_right, 0);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->front_left, 0);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->rear_left, 0);
    HAL_Delay(5);
    A1_16_SetSpeed(&car->rear_right, 0);

    current_state = CAR_STOP;
    last_command_time = HAL_GetTick();
}

// 測試函數
void Test_IndividualWheel(A1_16_Handle *wheel, int16_t speed, uint32_t duration) {
    DEBUG_PRINTF("測試 %s 輪子 (ID=%d)，速度=%d，持續%ldms\r\n",
           wheel->name, wheel->id, speed, duration);

    A1_16_SetSpeed(wheel, speed);
    HAL_Delay(duration);
    A1_16_SetSpeed(wheel, 0);
    HAL_Delay(500);
}

void Test_AllWheels(MecanumCar *car) {
    DEBUG_PRINTF("\r\n=== 單輪測試開始 ===\r\n");

    Test_IndividualWheel(&car->front_right, 30, 2000);
    Test_IndividualWheel(&car->front_left, 30, 2000);
    Test_IndividualWheel(&car->rear_left, 30, 2000);
    Test_IndividualWheel(&car->rear_right, 30, 2000);

    DEBUG_PRINTF("=== 單輪測試完成 ===\r\n\r\n");
}

void Test_BasicMovements(MecanumCar *car) {
    DEBUG_PRINTF("\r\n=== 基本運動測試 ===\r\n");

    const int16_t test_speed = 25;
    const uint32_t test_duration = 3000;
    const uint32_t stop_duration = 1000;

    // 前進
    Mecanum_Forward(car, test_speed);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(test_duration);
    Mecanum_Stop(car);
    HAL_Delay(stop_duration);

    // 後退
    Mecanum_Backward(car, test_speed);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(test_duration);
    Mecanum_Stop(car);
    HAL_Delay(stop_duration);

    // 左平移
    Mecanum_StrafeLeft(car, test_speed);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(test_duration);
    Mecanum_Stop(car);
    HAL_Delay(stop_duration);

    // 右平移
    Mecanum_StrafeRight(car, test_speed);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(test_duration);
    Mecanum_Stop(car);
    HAL_Delay(stop_duration);

    // 左旋轉
    Mecanum_RotateLeft(car, 20);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(2000);
    Mecanum_Stop(car);
    HAL_Delay(stop_duration);

    // 右旋轉
    Mecanum_RotateRight(car, 20);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(2000);
    Mecanum_Stop(car);
    HAL_Delay(stop_duration);
}

void Test_AdvancedMovements(MecanumCar *car) {
    DEBUG_PRINTF("\r\n=== 高級運動測試 ===\r\n");

    // 斜向運動
    Mecanum_DiagonalFrontRight(car, 25);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(2000);
    Mecanum_Stop(car);
    HAL_Delay(1000);

    Mecanum_DiagonalFrontLeft(car, 25);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(2000);
    Mecanum_Stop(car);
    HAL_Delay(1000);

    Mecanum_DiagonalBackRight(car, 25);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(2000);
    Mecanum_Stop(car);
    HAL_Delay(1000);

    Mecanum_DiagonalBackLeft(car, 25);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(2000);
    Mecanum_Stop(car);
    HAL_Delay(1000);

    // 組合運動：前進+左平移
    DEBUG_PRINTF("組合運動：前進+左平移\r\n");
    Mecanum_SetVelocity(car, 50.0f, -30.0f, 0.0f);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(3000);
    Mecanum_Stop(car);
    HAL_Delay(1000);

    // 組合運動：前進+旋轉
    DEBUG_PRINTF("組合運動：前進+旋轉\r\n");
    Mecanum_SetVelocity(car, 40.0f, 0.0f, 0.5f);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(3000);
    Mecanum_Stop(car);
    HAL_Delay(1000);
}

// 檢查舵機狀態
void Check_ServoStatus(A1_16_Handle *handle) {
    A1_16_Status status;
    if (A1_16_GetStatus(handle, &status) == HAL_OK) {
        DEBUG_PRINTF("%s (ID=%d): 位置=%d, 電流=%d, 溫度=%d℃, 電壓=%dV, 錯誤=0x%02X\r\n",
               handle->name, handle->id, status.position, status.current,
               status.temperature, status.voltage/10, status.status_error);
    } else {
        DEBUG_PRINTF("%s (ID=%d): 狀態讀取失敗\r\n", handle->name, handle->id);
    }
}

void Check_AllServos(MecanumCar *car) {
    DEBUG_PRINTF("\r\n=== 舵機狀態檢查 ===\r\n");
    Check_ServoStatus(&car->front_right);
    Check_ServoStatus(&car->front_left);
    Check_ServoStatus(&car->rear_left);
    Check_ServoStatus(&car->rear_right);
    DEBUG_PRINTF("\r\n");
}

void Scan_ServoIDs(UART_HandleTypeDef *huart, const char* uart_name) {
    A1_16_Handle temp_handle;
    A1_16_Status status;
    bool found = false;

    DEBUG_PRINTF("%s 掃描舵機ID: ", uart_name);

    for (uint8_t id = 1; id <= 15; id++) {  // 擴展掃描範圍到15
        A1_16_Init(&temp_handle, huart, id, "Test");
        if (A1_16_GetStatus(&temp_handle, &status) == HAL_OK) {
            DEBUG_PRINTF("%d ", id);
            found = true;
        }
        HAL_Delay(50);
    }

    if (!found) {
        DEBUG_PRINTF("無響應");
    }
    DEBUG_PRINTF("\r\n");
}

// 系統信息
void System_Info(void) {
    DEBUG_PRINTF("\r\n=== STM32 麥克納姆輪車控制系統 ===\r\n");
    DEBUG_PRINTF("固件版本: v1.2 (修復ID衝突)\r\n");
    DEBUG_PRINTF("系統時鐘: %ld MHz\r\n", HAL_RCC_GetHCLKFreq() / 1000000);
    DEBUG_PRINTF("USART1: 前輪控制 (115200 bps) - ID: 7, 8\r\n");
    DEBUG_PRINTF("USART2: 調試輸出 (115200 bps)\r\n");
    DEBUG_PRINTF("USART3: 後輪控制 (115200 bps) - ID: 9, 10\r\n");
    DEBUG_PRINTF("車輛參數: 輪半徑=%.1fmm, 軸距=%.1fmm, 輪距=%.1fmm\r\n",
            WHEEL_RADIUS, WHEEL_BASE, TRACK_WIDTH);
    DEBUG_PRINTF("按USER按鈕切換測試模式\r\n");
    DEBUG_PRINTF("=====================================\r\n");
}

void Change_TestMode(void) {
    test_mode = (test_mode + 1) % 4;
    const char* mode_names[] = {"自動循環", "單輪測試", "基本運動", "高級運動"};
    DEBUG_PRINTF("\r\n>>> 切換到模式 %d: %s <<<\r\n", test_mode, mode_names[test_mode]);

    // 停止當前運動
    Mecanum_Stop(&car);
    HAL_Delay(500);
}

void Handle_ButtonPress(void) {
    static uint32_t last_press_time = 0;
    uint32_t current_time = HAL_GetTick();

    // 防抖處理
    if (current_time - last_press_time > 200) {
        button_pressed = true;
        last_press_time = current_time;
    }
}

// GPIO中斷回調函數
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == B1_Pin) {
        Handle_ButtonPress();
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  // 系統啟動
  HAL_Delay(1000);  // 等待系統穩定
  System_Info();

  // USART3硬體測試
  DEBUG_PRINTF("\r\n=== USART3硬體測試 ===\r\n");
  DEBUG_PRINTF("請注意：如需迴路測試，請短接PB10和PC5\r\n");
  DEBUG_PRINTF("否則跳過迴路測試，進行舵機通訊測試\r\n");
  HAL_Delay(2000);

  // 迴路測試（需要短接PB10和PC5）
  DEBUG_PRINTF("\r\n1. USART3迴路測試:\r\n");
  Test_USART3_Loopback();

  HAL_Delay(1000);

  // 原始舵機通訊測試
  DEBUG_PRINTF("\r\n2. USART3舵機通訊測試:\r\n");
  Test_USART3_Raw();

  // *** 重要：首次運行時需要設置舵機ID ***
  // 如果這是首次配置，請取消註釋下面這行
  // Setup_ServoIDs();

  // 初始化麥克納姆輪車
  Mecanum_Init(&car);

  // 啟動後狀態檢查
  if (DEBUG_SERVO_STATUS) {
      Check_AllServos(&car);
  }

  DEBUG_PRINTF("\r\n=== 系統準備就緒，開始測試 ===\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // 處理按鈕按下事件
    if (button_pressed) {
        button_pressed = false;
        Change_TestMode();
    }

    // 根據測試模式執行不同的測試
    switch(test_mode) {
        case 0:  // 自動循環測試
            Test_BasicMovements(&car);
            break;

        case 1:  // 單輪測試
            Test_AllWheels(&car);
            HAL_Delay(2000);
            break;

        case 2:  // 基本運動測試
            Test_BasicMovements(&car);
            break;

        case 3:  // 高級運動測試
            Test_AdvancedMovements(&car);
            break;
    }

    // 測試間隔
    DEBUG_PRINTF("\r\n=== 測試循環完成，5秒後重新開始 ===\r\n");
    for(int i = 5; i > 0; i--) {
        DEBUG_PRINTF("倒數: %d (按按鈕切換模式)\r\n", i);
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay(1000);

        // 檢查按鈕
        if (button_pressed) {
            break;
        }
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
