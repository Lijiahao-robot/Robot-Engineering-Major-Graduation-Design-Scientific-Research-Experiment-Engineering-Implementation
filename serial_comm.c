#include "serial_comm.h"
#include "string.h"
#include <stdarg.h>  // 用于可变参数打印

// 全局通信结构体（初始化清零）
Serial_CommDef serial_comm = {0};
// 串口句柄（全局，供中断回调使用）
UART_HandleTypeDef *huart_serial = NULL;
// 接收/发送缓冲区（固定18字节，避免内存溢出）
uint8_t rx_buf[FRAME_LEN] = {0};
uint8_t tx_buf[FRAME_LEN] = {0};
// ✅ 新增：调试打印缓冲区
uint8_t debug_buf[DEBUG_TX_LEN] = {0};

/**
  * @brief  CRC8 校验计算（与ROS2端完全一致）
  * @param  data: 待校验数据指针
  * @param  len:  待校验数据长度
  * @retval crc8: 校验结果
  */
uint8_t CRC8_Calc(uint8_t *data, uint16_t len) {
    uint8_t crc = 0x00;
    for(uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for(uint8_t j = 0; j < 8; j++) {
            if(crc & 0x80) {
                crc = (crc << 1) ^ CRC8_POLY;
            } else {
                crc <<= 1;
            }
            crc &= 0xFF;
        }
    }
    return crc;
}

/**
  * @brief  串口通信初始化（开启DMA中断接收，避免CPU占用）
  * @param  huart: 串口句柄（如&huart3）
  * @retval None
  */
void Serial_Comm_Init(UART_HandleTypeDef *huart) {
    if(huart == NULL) {
        serial_comm.err_flag = 1;
        Debug_Printf("Serial Init Error: huart is NULL!\r\n");
        return;
    }
    huart_serial = huart;
    // 开启DMA接收（固定接收FRAME_LEN字节）
    HAL_UART_Receive_DMA(huart_serial, rx_buf, FRAME_LEN);
    serial_comm.err_flag = 0;
    Debug_Printf("Serial Comm Init Success! USART%d\r\n", huart_serial->Instance - USART1 + 1);
}

/**
  * @brief  解析ROS2下发的电机速度指令
  * @note   帧结构：0xAA + 0x55 + 4*float(16字节) + CRC8(1字节)
  * @retval None
  */
void Serial_Parse_Msg(void) {
    // 1. 帧头校验
    if(rx_buf[0] != FRAME_HEADER1 || rx_buf[1] != FRAME_HEADER2) {
        memset(serial_comm.motor_speed, 0, sizeof(serial_comm.motor_speed));
        serial_comm.err_flag = 1;
        Debug_Printf("Frame Header Error: 0x%02x 0x%02x\r\n", rx_buf[0], rx_buf[1]);
        return;
    }
    
    // 2. CRC8 校验
    uint8_t recv_crc = rx_buf[FRAME_LEN - 1];
    uint8_t calc_crc = CRC8_Calc(rx_buf, FRAME_LEN - 1);
    if(recv_crc != calc_crc) {
        memset(serial_comm.motor_speed, 0, sizeof(serial_comm.motor_speed));
        serial_comm.err_flag = 1;
        Debug_Printf("CRC Error: Recv=0x%02x | Calc=0x%02x\r\n", recv_crc, calc_crc);
        return;
    }
    
    // 3. 解析电机速度数据（float类型，小端模式）
    memcpy(&serial_comm.motor_speed[0], &rx_buf[2], 4);
    memcpy(&serial_comm.motor_speed[1], &rx_buf[6], 4);
    memcpy(&serial_comm.motor_speed[2], &rx_buf[10], 4);
    memcpy(&serial_comm.motor_speed[3], &rx_buf[14], 4);
    
    // ✅ 新增：调试打印（接收的电机速度）
    Debug_Printf("Recv Speed: LF=%.1f | RF=%.1f | LB=%.1f | RB=%.1f | CRC=0x%02x\r\n",
                 serial_comm.motor_speed[0], serial_comm.motor_speed[1],
                 serial_comm.motor_speed[2], serial_comm.motor_speed[3], recv_crc);
    
    // 4. 标记接收完成，清除错误标志
    serial_comm.rx_complete_flag = 1;
    serial_comm.err_flag = 0;
}

/**
  * @brief  发送编码器数据给ROS2（打包+CRC校验）
  * @note   帧结构：0x55 + 0xAA + 4*int32_t(16字节) + CRC8(1字节)
  * @retval None
  */
void Serial_Send_Sensor_Data(void) {
    if(huart_serial == NULL || serial_comm.err_flag) {
        return;
    }
    
    // 1. 填充帧头
    tx_buf[0] = FRAME_RESP_H1;
    tx_buf[1] = FRAME_RESP_H2;
    
    // 2. 填充编码器数据（4个int32_t，16字节）
    memcpy(&tx_buf[2], serial_comm.encoder_data, 16);
    
    // 3. 计算CRC8校验和（填充最后1字节）
    tx_buf[FRAME_LEN - 1] = CRC8_Calc(tx_buf, FRAME_LEN - 1);
    
    // ✅ 新增：调试打印（发送的编码器数据）
    Debug_Printf("Send Encoder: Enc1=%d | Enc2=%d | Enc3=%d | Enc4=%d | CRC=0x%02x\r\n",
                 serial_comm.encoder_data[0], serial_comm.encoder_data[1],
                 serial_comm.encoder_data[2], serial_comm.encoder_data[3], tx_buf[FRAME_LEN - 1]);
    
    // 4. DMA发送（非阻塞，不占用CPU）
    HAL_UART_Transmit_DMA(huart_serial, tx_buf, FRAME_LEN);
}

/**
  * @brief  串口DMA接收完成回调函数
  * @note   接收完一帧数据后，自动解析，重新开启接收
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart == huart_serial) {
        Serial_Parse_Msg();        // 解析数据
        memset(rx_buf, 0, FRAME_LEN); // 清空接收缓冲区
        HAL_UART_Receive_DMA(huart_serial, rx_buf, FRAME_LEN); // 重新开启接收
    }
}

/**
  * @brief  串口DMA发送完成回调函数
  * @note   标记发送完成，便于后续状态判断
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart == huart_serial) {
        serial_comm.tx_complete_flag = 1;
        memset(tx_buf, 0, FRAME_LEN); // 清空发送缓冲区
    }
}

/**
  * @brief  ✅ 调试打印函数（可变参数，类似printf，用USART1发送）
  * @param  format: 打印格式
  * @retval None
  */
void Debug_Printf(const char *format, ...) {
    va_list arg;
    va_start(arg, format);
    // 格式化字符串到调试缓冲区
    uint16_t len = vsnprintf((char*)debug_buf, DEBUG_TX_LEN, format, arg);
    va_end(arg);
    // 串口发送调试信息（阻塞发送，仅用于调试，不影响控制）
    if(len <= DEBUG_TX_LEN) {
        HAL_UART_Transmit(DEBUG_USART, debug_buf, len, 10);
    }
    memset(debug_buf, 0, DEBUG_TX_LEN); // 清空缓冲区
}
