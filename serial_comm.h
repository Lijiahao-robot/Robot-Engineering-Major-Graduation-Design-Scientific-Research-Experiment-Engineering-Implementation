#ifndef __SERIAL_COMM_H
#define __SERIAL_COMM_H

#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "motor.h"

// -------------------------- 通信协议配置 --------------------------
#define FRAME_LEN         18          // 单帧数据长度（固定）
#define FRAME_HEADER1     0xAA        // ROS2 → STM32 帧头1
#define FRAME_HEADER2     0x55        // ROS2 → STM32 帧头2
#define FRAME_RESP_H1     0x55        // STM32 → ROS2 帧头1
#define FRAME_RESP_H2     0xAA        // STM32 → ROS2 帧头2
#define CRC8_POLY         0x31        // CRC8 多项式

// ✅ 新增：调试串口配置（USART1，用于打印调试信息）
#define DEBUG_USART       &huart1
#define DEBUG_TX_LEN      128         // 调试打印缓冲区长度

// -------------------------- 数据结构体 --------------------------
typedef struct {
    float motor_speed[4];      // 接收：ROS2下发的电机速度指令（rpm）
    int32_t encoder_data[4];   // 发送：4个电机编码器数据
    uint8_t rx_complete_flag;  // 接收完成标志（1:完成，0:未完成）
    uint8_t tx_complete_flag;  // 发送完成标志（1:完成，0:未完成）
    uint8_t err_flag;          // 通信错误标志（1:错误，0:正常）
} Serial_CommDef;

// -------------------------- 外部接口 --------------------------
extern Serial_CommDef serial_comm;

void Serial_Comm_Init(UART_HandleTypeDef *huart);  // 初始化串口通信（DMA中断）
void Serial_Parse_Msg(void);                       // 解析ROS2下发的数据帧
void Serial_Send_Sensor_Data(void);                // 发送传感器数据给ROS2
uint8_t CRC8_Calc(uint8_t *data, uint16_t len);    // CRC8 校验计算
void Debug_Printf(const char *format, ...);         // ✅ 新增：调试打印函数

#endif
