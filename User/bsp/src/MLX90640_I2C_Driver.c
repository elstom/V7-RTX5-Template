/* MLX90640_I2C_Driver.c */

#include "bsp.h"
#include "MLX90640_I2C_Driver.h"
#include "stm32h7xx_hal.h"

extern I2C_HandleTypeDef I2cHandle; // 使用您提供的 I2C 句柄

/* 初始化 MLX90640 I2C 接口 */
void MLX90640_I2CInit(void)
{
    /* 调用本地驱动的初始化函数 */
    bsp_InitI2CBus();

    /* 修改 I2C 配置为主机模式 */
    I2cHandle.Init.OwnAddress1 = 0x00; // 主机模式下自地址可设为 0x00

    /* 重新初始化 I2C 外设 */
    if (HAL_I2C_Init(&I2cHandle) != HAL_OK)
    {
        /* 初始化错误处理 */
        Error_Handler(__FILE__, __LINE__);
    }
}

// /* 发送 I2C 通用复位命令 */
// int MLX90640_I2CGeneralReset(void)
// {
//     uint8_t cmd = 0x06; // 通用复位命令
//     HAL_StatusTypeDef result;

//     /* 发送通用调用复位命令 */
//     result = HAL_I2C_Master_Transmit(&I2cHandle, 0x00, &cmd, 1, HAL_MAX_DELAY);
//     if (result != HAL_OK)
//     {
//         return -1; // 发送失败
//     }

//     return 0; // 发送成功
// }

/* 从 MLX90640 读取数据 */
int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{
    uint8_t regAddr[2];
    regAddr[0] = (startAddress >> 8) & 0xFF; // 高字节
    regAddr[1] = startAddress & 0xFF;        // 低字节

    // 写入寄存器地址
    i2c_Start();
    i2c_SendByte(slaveAddr << 1 | I2C_WR); // 写入操作
    if (i2c_WaitAck())
    {
        i2c_Stop();
        return -1; // 无应答，返回错误
    }
    i2c_SendByte(regAddr[0]); // 发送高字节
    i2c_WaitAck();
    i2c_SendByte(regAddr[1]); // 发送低字节
    i2c_WaitAck();

    // 读取数据
    i2c_Start();
    i2c_SendByte(slaveAddr << 1 | I2C_RD); // 读操作
    if (i2c_WaitAck())
    {
        i2c_Stop();
        return -1; // 无应答，返回错误
    }
    for (int i = 0; i < nMemAddressRead; i++)
    {
        data[i] = i2c_ReadByte(); // 读取数据
        if (i < (nMemAddressRead - 1))
        {
            i2c_Ack(); // 继续读取，发送 ACK
        }
        else
        {
            i2c_NAck(); // 最后一个字节，发送 NACK
        }
    }
    i2c_Stop(); // 停止 I2C 操作

    return 0; // 成功
}

/* 向 MLX90640 写入数据 */
int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
{
    uint8_t buffer[4];
    buffer[0] = (writeAddress >> 8) & 0xFF; // 地址高字节
    buffer[1] = writeAddress & 0xFF;        // 地址低字节
    buffer[2] = (data >> 8) & 0xFF;         // 数据高字节
    buffer[3] = data & 0xFF;                // 数据低字节

    // 写入数据
    i2c_Start();
    i2c_SendByte(slaveAddr << 1 | I2C_WR); // 写操作
    if (i2c_WaitAck())
    {
        i2c_Stop();
        return -1; // 无应答，返回错误
    }
    i2c_SendByte(buffer[0]); // 发送寄存器地址高字节
    i2c_WaitAck();
    i2c_SendByte(buffer[1]); // 发送寄存器地址低字节
    i2c_WaitAck();
    i2c_SendByte(buffer[2]); // 发送数据高字节
    i2c_WaitAck();
    i2c_SendByte(buffer[3]); // 发送数据低字节
    i2c_WaitAck();
    i2c_Stop(); // 停止 I2C 操作

    return 0; // 成功
}

// /* 设置 I2C 通信频率 */
// void MLX90640_I2CFreqSet(int freq)
// {
//     uint32_t timing;

//     /* 根据频率选择合适的定时配置 */
//     if (freq == 100000) // 100kHz
//     {
//         timing = 0x10D07DB5; // 示例值，请根据实际情况调整
//     }
//     else if (freq == 400000) // 400kHz
//     {
//         timing = 0x00C0216C; // 示例值，请根据实际情况调整
//     }
//     else
//     {
//         /* 不支持的频率 */
//         return;
//     }

//     /* 更新 I2C 定时寄存器 */
//     I2cHandle.Init.Timing = timing;

//     /* 重新初始化 I2C 外设 */
//     if (HAL_I2C_Init(&I2cHandle) != HAL_OK)
//     {
//         /* 初始化错误处理 */
//         Error_Handler(__FILE__, __LINE__);
//     }
// }
