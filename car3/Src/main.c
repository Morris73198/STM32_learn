/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 麦克纳姆轮车控制程序 - 全USART1版本，ID=1,2,3,4
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
// A1-16舵机控制结构体
typedef struct {
    uint8_t id;
    UART_HandleTypeDef *huart;
    uint8_t rx_buffer[100];
    uint8_t tx_buffer[100];
    bool response_received;
    char name[20];  // 轮子名称
    int16_t current_speed;  // 当前速度
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

// 麦克纳姆轮车结构体
typedef struct {
    A1_16_Handle front_right;  // 右前轮 USART1, ID=1
    A1_16_Handle front_left;   // 左前轮 USART1, ID=2
    A1_16_Handle rear_left;    // 左后轮 USART1, ID=3
    A1_16_Handle rear_right;   // 右后轮 USART1, ID=4
    float wheel_radius;        // 轮子半径 (mm)
    float wheel_base;          // 轴距 (mm)
    float track_width;         // 轮距 (mm)
    bool initialized;          // 初始化状态
} MecanumCar;

// 运动状态枚举
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
// A1-16 命令定义
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

// 麦克纳姆轮参数
#define WHEEL_RADIUS    50.0f   // mm
#define WHEEL_BASE      200.0f  // mm
#define TRACK_WIDTH     180.0f  // mm
#define MAX_SPEED       100     // 最大速度值
#define MIN_SPEED       10      // 最小有效速度

// 调试开关
#define DEBUG_ENABLED   1
#define DEBUG_SERVO_STATUS  1

// 舵机ID设置开关 - 首次运行时设为1，设置完成后改为0
#define SETUP_SERVO_IDS 0
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

/* USER CODE BEGIN PV */
MecanumCar car;
CarMovementState current_state = CAR_STOP;
uint32_t last_command_time = 0;
uint8_t test_mode = 0;  // 0=自动测试, 1=单轮测试, 2=基本运动, 3=高级运动
bool button_pressed = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

// A1-16舵机控制函数
HAL_StatusTypeDef A1_16_Init(A1_16_Handle *handle, UART_HandleTypeDef *huart, uint8_t id, const char* name);
HAL_StatusTypeDef A1_16_SetSpeed(A1_16_Handle *handle, int16_t speed);
HAL_StatusTypeDef A1_16_GetStatus(A1_16_Handle *handle, A1_16_Status *status);
HAL_StatusTypeDef A1_16_TorqueOff(A1_16_Handle *handle);
HAL_StatusTypeDef A1_16_TorqueOn(A1_16_Handle *handle);
HAL_StatusTypeDef A1_16_SetID(A1_16_Handle *handle, uint8_t old_id, uint8_t new_id);
HAL_StatusTypeDef A1_16_Reboot(A1_16_Handle *handle);

// 内部函数
static HAL_StatusTypeDef A1_16_SendPacket(A1_16_Handle *handle, uint8_t cmd, uint8_t *data, uint8_t data_len);
static HAL_StatusTypeDef A1_16_ReceiveResponse(A1_16_Handle *handle, uint8_t expected_cmd, uint32_t timeout);
static uint8_t A1_16_CalculateChecksum1(uint8_t *data, uint8_t length);
static uint8_t A1_16_CalculateChecksum2(uint8_t checksum1);

// 舵机ID设置函数
HAL_StatusTypeDef Setup_NewServoIDs(void);

// 麦克纳姆轮运动控制函数
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

// 测试和调试函数
void Test_IndividualWheel(A1_16_Handle *wheel, int16_t speed, uint32_t duration);
void Test_AllWheels(MecanumCar *car);
void Test_BasicMovements(MecanumCar *car);
void Test_AdvancedMovements(MecanumCar *car);
void Check_ServoStatus(A1_16_Handle *handle);
void Check_AllServos(MecanumCar *car);
void Scan_ServoIDs(UART_HandleTypeDef *huart, const char* uart_name);

// 系统函数
void System_Info(void);
void Change_TestMode(void);
void Handle_ButtonPress(void);

// 中断回调函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 重定向printf到UART2
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

// 设置新的舵机ID：1,2,3,4 (从原来的7,8改过来)
HAL_StatusTypeDef Setup_NewServoIDs(void) {
    DEBUG_PRINTF("\r\n=== 设置新的舵机ID (7,8 → 1,2,3,4) ===\r\n");
    DEBUG_PRINTF("警告：此操作将永久修改舵机ID！\r\n");
    DEBUG_PRINTF("请确保已将所有舵机串联连接到USART1\r\n");
    HAL_Delay(3000);

    A1_16_Handle temp_handle;
    bool success = false;

    // 扫描当前可用的舵机
    DEBUG_PRINTF("扫描当前舵机ID...\r\n");
    for (uint8_t id = 1; id <= 15; id++) {
        A1_16_Init(&temp_handle, &huart1, id, "Scan");
        A1_16_Status status;
        if (A1_16_GetStatus(&temp_handle, &status) == HAL_OK) {
            DEBUG_PRINTF("发现舵机ID: %d\r\n", id);
        }
        HAL_Delay(100);
    }

    DEBUG_PRINTF("\r\n开始设置新ID...\r\n");

    // 第一步：将第一个ID=7改为ID=1
    DEBUG_PRINTF("1. 设置第一个舵机: ID=7 → ID=1\r\n");
    A1_16_Init(&temp_handle, &huart1, 7, "Temp1");
    if (A1_16_SetID(&temp_handle, 7, 1) == HAL_OK) {
        DEBUG_PRINTF("✓ 第一个舵机ID修改成功: 7 → 1\r\n");
        A1_16_Reboot(&temp_handle);
        HAL_Delay(2000);
        success = true;
    } else {
        DEBUG_PRINTF("✗ 第一个舵机ID修改失败\r\n");
    }

    // 第二步：将第一个ID=8改为ID=2
    DEBUG_PRINTF("2. 设置第二个舵机: ID=8 → ID=2\r\n");
    A1_16_Init(&temp_handle, &huart1, 8, "Temp2");
    if (A1_16_SetID(&temp_handle, 8, 2) == HAL_OK) {
        DEBUG_PRINTF("✓ 第二个舵机ID修改成功: 8 → 2\r\n");
        A1_16_Reboot(&temp_handle);
        HAL_Delay(2000);
    } else {
        DEBUG_PRINTF("✗ 第二个舵机ID修改失败\r\n");
    }

    // 第三步：将第二个ID=7改为ID=3
    DEBUG_PRINTF("3. 设置第三个舵机: ID=7 → ID=3\r\n");
    A1_16_Init(&temp_handle, &huart1, 7, "Temp3");
    if (A1_16_SetID(&temp_handle, 7, 3) == HAL_OK) {
        DEBUG_PRINTF("✓ 第三个舵机ID修改成功: 7 → 3\r\n");
        A1_16_Reboot(&temp_handle);
        HAL_Delay(2000);
    } else {
        DEBUG_PRINTF("✗ 第三个舵机ID修改失败\r\n");
    }

    // 第四步：将第二个ID=8改为ID=4
    DEBUG_PRINTF("4. 设置第四个舵机: ID=8 → ID=4\r\n");
    A1_16_Init(&temp_handle, &huart1, 8, "Temp4");
    if (A1_16_SetID(&temp_handle, 8, 4) == HAL_OK) {
        DEBUG_PRINTF("✓ 第四个舵机ID修改成功: 8 → 4\r\n");
        A1_16_Reboot(&temp_handle);
        HAL_Delay(2000);
    } else {
        DEBUG_PRINTF("✗ 第四个舵机ID修改失败\r\n");
    }

    // 验证新ID
    DEBUG_PRINTF("\r\n验证新的舵机ID...\r\n");
    Scan_ServoIDs(&huart1, "USART1");

    DEBUG_PRINTF("=== 舵机ID设置完成 ===\r\n");
    DEBUG_PRINTF("请将代码中 SETUP_SERVO_IDS 改为 0 以避免重复设置\r\n");

    return success ? HAL_OK : HAL_ERROR;
}

// A1-16舵机初始化
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

// 设置舵机速度
HAL_StatusTypeDef A1_16_SetSpeed(A1_16_Handle *handle, int16_t speed) {
    uint8_t data[5];
    uint16_t speed_val;

    // 记录当前速度
    handle->current_speed = speed;

    // 转换速度值 (负数表示反向)
    if (speed < 0) {
        speed_val = (uint16_t)(-speed);
        speed_val |= 0x8000; // 设置方向位
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

// 获取舵机状态
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

// 关闭扭矩
HAL_StatusTypeDef A1_16_TorqueOff(A1_16_Handle *handle) {
    uint8_t data[5] = {0, 0, A1_16_MODE_TORQUE_OFF, handle->id, 0};
    handle->current_speed = 0;
    return A1_16_SendPacket(handle, A1_16_CMD_I_JOG, data, 5);
}

// 开启扭矩
HAL_StatusTypeDef A1_16_TorqueOn(A1_16_Handle *handle) {
    uint8_t data[5] = {0, 0, A1_16_MODE_SERVO_ON, handle->id, 0};
    return A1_16_SendPacket(handle, A1_16_CMD_I_JOG, data, 5);
}

// 设置舵机ID
HAL_StatusTypeDef A1_16_SetID(A1_16_Handle *handle, uint8_t old_id, uint8_t new_id) {
    uint8_t data[3];
    handle->id = old_id;

    data[0] = 6;        // EEPROM地址6是sID参数
    data[1] = 1;        // length
    data[2] = new_id;   // 新的ID值

    HAL_StatusTypeDef ret = A1_16_SendPacket(handle, A1_16_CMD_EEP_WRITE, data, 3);
    if (ret != HAL_OK) return ret;

    ret = A1_16_ReceiveResponse(handle, A1_16_CMD_EEP_WRITE + 0x40, 500);
    if (ret != HAL_OK) return ret;

    handle->id = new_id;
    return HAL_OK;
}

// 重启舵机
HAL_StatusTypeDef A1_16_Reboot(A1_16_Handle *handle) {
    HAL_StatusTypeDef ret = A1_16_SendPacket(handle, A1_16_CMD_REBOOT, NULL, 0);
    if (ret != HAL_OK) return ret;
    return A1_16_ReceiveResponse(handle, A1_16_CMD_REBOOT + 0x40, 500);
}

// 计算校验和函数
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

// 发送数据包
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

// 接收响应
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

// 麦克纳姆轮初始化 - 全USART1版本，ID=1,2,3,4
void Mecanum_Init(MecanumCar *car) {
    DEBUG_PRINTF("\r\n=== 麦克纳姆轮车初始化 (全USART1版本) ===\r\n");

    // 初始化各个轮子 - 全部使用USART1，但使用不同ID
    A1_16_Init(&car->front_right, &huart1, 1, "FrontRight");  // USART1, ID=1
    A1_16_Init(&car->front_left, &huart1, 2, "FrontLeft");    // USART1, ID=2
    A1_16_Init(&car->rear_left, &huart1, 3, "RearLeft");      // USART1, ID=3
    A1_16_Init(&car->rear_right, &huart1, 4, "RearRight");    // USART1, ID=4

    // 设置车辆参数
    car->wheel_radius = WHEEL_RADIUS;
    car->wheel_base = WHEEL_BASE;
    car->track_width = TRACK_WIDTH;
    car->initialized = false;

    HAL_Delay(500);

    // 检查舵机连接
    DEBUG_PRINTF("检查舵机连接...\r\n");
    Scan_ServoIDs(&huart1, "USART1(所有轮子)");

    // 启用所有轮子的扭矩
    DEBUG_PRINTF("\r\n启用舵机扭矩...\r\n");

    if (A1_16_TorqueOn(&car->front_right) == HAL_OK) {
        DEBUG_PRINTF("✓ 右前轮(USART1, ID=1)启用成功\r\n");
    } else {
        DEBUG_PRINTF("✗ 右前轮(USART1, ID=1)启用失败！\r\n");
    }
    HAL_Delay(50);

    if (A1_16_TorqueOn(&car->front_left) == HAL_OK) {
        DEBUG_PRINTF("✓ 左前轮(USART1, ID=2)启用成功\r\n");
    } else {
        DEBUG_PRINTF("✗ 左前轮(USART1, ID=2)启用失败！\r\n");
    }
    HAL_Delay(50);

    if (A1_16_TorqueOn(&car->rear_left) == HAL_OK) {
        DEBUG_PRINTF("✓ 左后轮(USART1, ID=3)启用成功\r\n");
    } else {
        DEBUG_PRINTF("✗ 左后轮(USART1, ID=3)启用失败！\r\n");
    }
    HAL_Delay(50);

    if (A1_16_TorqueOn(&car->rear_right) == HAL_OK) {
        DEBUG_PRINTF("✓ 右后轮(USART1, ID=4)启用成功\r\n");
    } else {
        DEBUG_PRINTF("✗ 右后轮(USART1, ID=4)启用失败！\r\n");
    }

    car->initialized = true;
    DEBUG_PRINTF("✓ 麦克纳姆轮车初始化完成\r\n");
}

// 麦克纳姆轮运动学控制（精确版本）
void Mecanum_SetVelocity(MecanumCar *car, float vx, float vy, float omega) {
    if (!car->initialized) return;

    float L = car->wheel_base / 2.0f;
    float W = car->track_width / 2.0f;

    // 麦克纳姆轮运动学逆解
    float v_fr = vx - vy - (L + W) * omega;  // 右前
    float v_fl = vx + vy + (L + W) * omega;  // 左前
    float v_rl = vx - vy + (L + W) * omega;  // 左后
    float v_rr = vx + vy - (L + W) * omega;  // 右后

    // 转换为舵机速度值并限制范围
    int16_t speed_fr = (int16_t)fmax(-MAX_SPEED, fmin(MAX_SPEED, v_fr * MAX_SPEED / 100.0f));
    int16_t speed_fl = (int16_t)fmax(-MAX_SPEED, fmin(MAX_SPEED, v_fl * MAX_SPEED / 100.0f));
    int16_t speed_rl = (int16_t)fmax(-MAX_SPEED, fmin(MAX_SPEED, v_rl * MAX_SPEED / 100.0f));
    int16_t speed_rr = (int16_t)fmax(-MAX_SPEED, fmin(MAX_SPEED, v_rr * MAX_SPEED / 100.0f));

    // 发送速度命令（增加延迟确保串行通讯稳定）
    A1_16_SetSpeed(&car->front_right, speed_fr);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->front_left, speed_fl);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->rear_left, speed_rl);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->rear_right, speed_rr);

    current_state = CAR_CUSTOM;
    last_command_time = HAL_GetTick();

    DEBUG_PRINTF("自定义运动: FR=%d FL=%d RL=%d RR=%d\r\n", speed_fr, speed_fl, speed_rl, speed_rr);
}

// 基本运动函数
void Mecanum_Forward(MecanumCar *car, int16_t speed) {
    if (!car->initialized) return;

    DEBUG_PRINTF("前进，速度: %d\r\n", speed);
    A1_16_SetSpeed(&car->front_right, speed);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->front_left, speed);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->rear_left, speed);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->rear_right, speed);

    current_state = CAR_FORWARD;
    last_command_time = HAL_GetTick();
}

void Mecanum_Backward(MecanumCar *car, int16_t speed) {
    if (!car->initialized) return;

    DEBUG_PRINTF("后退，速度: %d\r\n", speed);
    A1_16_SetSpeed(&car->front_right, -speed);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->front_left, -speed);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->rear_left, -speed);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->rear_right, -speed);

    current_state = CAR_BACKWARD;
    last_command_time = HAL_GetTick();
}

void Mecanum_StrafeLeft(MecanumCar *car, int16_t speed) {
    if (!car->initialized) return;

    DEBUG_PRINTF("左平移，速度: %d\r\n", speed);
    A1_16_SetSpeed(&car->front_right, -speed);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->front_left, speed);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->rear_left, -speed);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->rear_right, speed);

    current_state = CAR_STRAFE_LEFT;
    last_command_time = HAL_GetTick();
}

void Mecanum_StrafeRight(MecanumCar *car, int16_t speed) {
    if (!car->initialized) return;

    DEBUG_PRINTF("右平移，速度: %d\r\n", speed);
    A1_16_SetSpeed(&car->front_right, speed);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->front_left, -speed);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->rear_left, speed);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->rear_right, -speed);

    current_state = CAR_STRAFE_RIGHT;
    last_command_time = HAL_GetTick();
}

void Mecanum_RotateLeft(MecanumCar *car, int16_t speed) {
    if (!car->initialized) return;

    DEBUG_PRINTF("左旋转，速度: %d\r\n", speed);
    A1_16_SetSpeed(&car->front_right, speed);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->front_left, -speed);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->rear_left, -speed);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->rear_right, speed);

    current_state = CAR_ROTATE_LEFT;
    last_command_time = HAL_GetTick();
}

void Mecanum_RotateRight(MecanumCar *car, int16_t speed) {
    if (!car->initialized) return;

    DEBUG_PRINTF("右旋转，速度: %d\r\n", speed);
    A1_16_SetSpeed(&car->front_right, -speed);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->front_left, speed);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->rear_left, speed);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->rear_right, -speed);

    current_state = CAR_ROTATE_RIGHT;
    last_command_time = HAL_GetTick();
}

void Mecanum_DiagonalFrontLeft(MecanumCar *car, int16_t speed) {
    if (!car->initialized) return;

    DEBUG_PRINTF("左前斜移，速度: %d\r\n", speed);
    A1_16_SetSpeed(&car->front_right, 0);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->front_left, speed);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->rear_left, 0);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->rear_right, speed);

    current_state = CAR_DIAGONAL_FL;
    last_command_time = HAL_GetTick();
}

void Mecanum_DiagonalFrontRight(MecanumCar *car, int16_t speed) {
    if (!car->initialized) return;

    DEBUG_PRINTF("右前斜移，速度: %d\r\n", speed);
    A1_16_SetSpeed(&car->front_right, speed);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->front_left, 0);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->rear_left, speed);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->rear_right, 0);

    current_state = CAR_DIAGONAL_FR;
    last_command_time = HAL_GetTick();
}

void Mecanum_DiagonalBackLeft(MecanumCar *car, int16_t speed) {
    if (!car->initialized) return;

    DEBUG_PRINTF("左后斜移，速度: %d\r\n", speed);
    A1_16_SetSpeed(&car->front_right, -speed);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->front_left, 0);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->rear_left, -speed);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->rear_right, 0);

    current_state = CAR_DIAGONAL_BL;
    last_command_time = HAL_GetTick();
}

void Mecanum_DiagonalBackRight(MecanumCar *car, int16_t speed) {
    if (!car->initialized) return;

    DEBUG_PRINTF("右后斜移，速度: %d\r\n", speed);
    A1_16_SetSpeed(&car->front_right, 0);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->front_left, -speed);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->rear_left, 0);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->rear_right, -speed);

    current_state = CAR_DIAGONAL_BR;
    last_command_time = HAL_GetTick();
}

void Mecanum_Stop(MecanumCar *car) {
    if (!car->initialized) return;

    DEBUG_PRINTF("停止\r\n");
    A1_16_SetSpeed(&car->front_right, 0);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->front_left, 0);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->rear_left, 0);
    HAL_Delay(10);
    A1_16_SetSpeed(&car->rear_right, 0);

    current_state = CAR_STOP;
    last_command_time = HAL_GetTick();
}

// 测试函数
void Test_IndividualWheel(A1_16_Handle *wheel, int16_t speed, uint32_t duration) {
    DEBUG_PRINTF("测试 %s 轮子 (ID=%d)，速度=%d，持续%ldms\r\n",
           wheel->name, wheel->id, speed, duration);

    A1_16_SetSpeed(wheel, speed);
    HAL_Delay(duration);
    A1_16_SetSpeed(wheel, 0);
    HAL_Delay(500);
}

void Test_AllWheels(MecanumCar *car) {
    DEBUG_PRINTF("\r\n=== 单轮测试开始 ===\r\n");

    Test_IndividualWheel(&car->front_right, 30, 2000);
    Test_IndividualWheel(&car->front_left, 30, 2000);
    Test_IndividualWheel(&car->rear_left, 30, 2000);
    Test_IndividualWheel(&car->rear_right, 30, 2000);

    DEBUG_PRINTF("=== 单轮测试完成 ===\r\n\r\n");
}

void Test_BasicMovements(MecanumCar *car) {
    DEBUG_PRINTF("\r\n=== 基本运动测试 ===\r\n");

    const int16_t test_speed = 25;
    const uint32_t test_duration = 3000;
    const uint32_t stop_duration = 1000;

    // 前进
    Mecanum_Forward(car, test_speed);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(test_duration);
    Mecanum_Stop(car);
    HAL_Delay(stop_duration);

    // 后退
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

    // 左旋转
    Mecanum_RotateLeft(car, 20);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(2000);
    Mecanum_Stop(car);
    HAL_Delay(stop_duration);

    // 右旋转
    Mecanum_RotateRight(car, 20);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(2000);
    Mecanum_Stop(car);
    HAL_Delay(stop_duration);
}

void Test_AdvancedMovements(MecanumCar *car) {
    DEBUG_PRINTF("\r\n=== 高级运动测试 ===\r\n");

    // 斜向运动
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

    // 组合运动：前进+左平移
    DEBUG_PRINTF("组合运动：前进+左平移\r\n");
    Mecanum_SetVelocity(car, 50.0f, -30.0f, 0.0f);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(3000);
    Mecanum_Stop(car);
    HAL_Delay(1000);

    // 组合运动：前进+旋转
    DEBUG_PRINTF("组合运动：前进+旋转\r\n");
    Mecanum_SetVelocity(car, 40.0f, 0.0f, 0.5f);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(3000);
    Mecanum_Stop(car);
    HAL_Delay(1000);
}

// 检查舵机状态
void Check_ServoStatus(A1_16_Handle *handle) {
    A1_16_Status status;
    if (A1_16_GetStatus(handle, &status) == HAL_OK) {
        DEBUG_PRINTF("%s (ID=%d): 位置=%d, 电流=%d, 温度=%d℃, 电压=%dV, 错误=0x%02X\r\n",
               handle->name, handle->id, status.position, status.current,
               status.temperature, status.voltage/10, status.status_error);
    } else {
        DEBUG_PRINTF("%s (ID=%d): 状态读取失败\r\n", handle->name, handle->id);
    }
}

void Check_AllServos(MecanumCar *car) {
    DEBUG_PRINTF("\r\n=== 舵机状态检查 ===\r\n");
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

    DEBUG_PRINTF("%s 扫描舵机ID: ", uart_name);

    for (uint8_t id = 1; id <= 15; id++) {  // 扫描ID 1-15
        A1_16_Init(&temp_handle, huart, id, "Test");
        if (A1_16_GetStatus(&temp_handle, &status) == HAL_OK) {
            DEBUG_PRINTF("%d ", id);
            found = true;
        }
        HAL_Delay(50);
    }

    if (!found) {
        DEBUG_PRINTF("无响应");
    }
    DEBUG_PRINTF("\r\n");
}

// 系统信息
void System_Info(void) {
    DEBUG_PRINTF("\r\n=== STM32 麦克纳姆轮车控制系统 (全USART1版本) ===\r\n");
    DEBUG_PRINTF("固件版本: v2.0 (全USART1版本)\r\n");
    DEBUG_PRINTF("系统时钟: %ld MHz\r\n", HAL_RCC_GetHCLKFreq() / 1000000);
    DEBUG_PRINTF("USART1: 所有舵机控制 (115200 bps) - ID: 1,2,3,4\r\n");
    DEBUG_PRINTF("USART2: 调试输出 (115200 bps)\r\n");
    DEBUG_PRINTF("舵机分配: 右前轮=ID1, 左前轮=ID2, 左后轮=ID3, 右后轮=ID4\r\n");
    DEBUG_PRINTF("车辆参数: 轮半径=%.1fmm, 轴距=%.1fmm, 轮距=%.1fmm\r\n",
            WHEEL_RADIUS, WHEEL_BASE, TRACK_WIDTH);
    DEBUG_PRINTF("按USER按钮切换测试模式\r\n");
    DEBUG_PRINTF("=====================================\r\n");
}

void Change_TestMode(void) {
    test_mode = (test_mode + 1) % 4;
    const char* mode_names[] = {"自动循环", "单轮测试", "基本运动", "高级运动"};
    DEBUG_PRINTF("\r\n>>> 切换到模式 %d: %s <<<\r\n", test_mode, mode_names[test_mode]);

    // 停止当前运动
    Mecanum_Stop(&car);
    HAL_Delay(500);
}

void Handle_ButtonPress(void) {
    static uint32_t last_press_time = 0;
    uint32_t current_time = HAL_GetTick();

    // 防抖处理
    if (current_time - last_press_time > 200) {
        button_pressed = true;
        last_press_time = current_time;
    }
}

// GPIO中断回调函数
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
  /* USER CODE BEGIN 2 */

  // 系统启动
  HAL_Delay(1000);  // 等待系统稳定
  System_Info();

#if SETUP_SERVO_IDS
  // *** 首次运行时设置舵机ID ***
  DEBUG_PRINTF("\r\n!!! 警告：将执行舵机ID设置 !!!\r\n");
  DEBUG_PRINTF("请确认：\r\n");
  DEBUG_PRINTF("1. 所有4个舵机已串联连接到USART1\r\n");
  DEBUG_PRINTF("2. 当前舵机ID为7,8 (各2个)\r\n");
  DEBUG_PRINTF("3. 将被重新分配为ID=1,2,3,4\r\n");
  HAL_Delay(5000);

  Setup_NewServoIDs();

  DEBUG_PRINTF("\r\n设置完成！请将代码中 SETUP_SERVO_IDS 改为 0\r\n");
  DEBUG_PRINTF("然后重新编译上传程序。\r\n");
  while(1) {
    HAL_Delay(1000);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  }
#endif

  // 检查舵机连接
  DEBUG_PRINTF("检查舵机连接...\r\n");
  Scan_ServoIDs(&huart1, "USART1(所有轮子)");

  // 初始化麦克纳姆轮车
  Mecanum_Init(&car);

  // 启动后状态检查
  if (DEBUG_SERVO_STATUS) {
      Check_AllServos(&car);
  }

  DEBUG_PRINTF("\r\n=== 系统准备就绪，开始测试 ===\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // 处理按钮按下事件
    if (button_pressed) {
        button_pressed = false;
        Change_TestMode();
    }

    // 根据测试模式执行不同的测试
    switch(test_mode) {
        case 0:  // 自动循环测试
            Test_BasicMovements(&car);
            break;

        case 1:  // 单轮测试
            Test_AllWheels(&car);
            HAL_Delay(2000);
            break;

        case 2:  // 基本运动测试
            Test_BasicMovements(&car);
            break;

        case 3:  // 高级运动测试
            Test_AdvancedMovements(&car);
            break;
    }

    // 测试间隔
    DEBUG_PRINTF("\r\n=== 测试循环完成，5秒后重新开始 ===\r\n");
    for(int i = 5; i > 0; i--) {
        DEBUG_PRINTF("倒数: %d (按按钮切换模式)\r\n", i);
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay(1000);

        // 检查按钮
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
