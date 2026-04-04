#include "W25Q64.h"


extern SPI_HandleTypeDef hspi1;

/* 读取W25Q64ID */
uint16_t W25Q64_ReadId(void)
{
    uint8_t tx_data[4] = {0x90, 0x00, 0x00, 0x00}; // 0x90指令 + 3个字节地址
    uint8_t rx_data[2] = {0, 0};

    // 1. 拉低片选
    HAL_GPIO_WritePin(W25Q64_CS_GPIO_Port, W25Q64_CS_Pin, GPIO_PIN_RESET);

    // 2. 发送指令
    HAL_SPI_Transmit(&hspi1, tx_data, 4, 100);

    // 3. 接收 2 个字节 (厂商ID 0xEF, 设备ID 0x16)
    HAL_SPI_Receive(&hspi1, rx_data, 2, 100);

    // 4. 拉高片选
    HAL_GPIO_WritePin(W25Q64_CS_GPIO_Port, W25Q64_CS_Pin, GPIO_PIN_SET);

    // 5. 拼接
    return (rx_data[0] << 8) | rx_data[1];
}

/* 写入W25Q64数据 */
void ReadStatus()
{
    uint8_t cmd = 0x05;
    uint8_t status;
    do
    {
        HAL_GPIO_WritePin(W25Q64_CS_GPIO_Port, W25Q64_CS_Pin, GPIO_PIN_RESET);

        HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
       
        HAL_SPI_Receive(&hspi1, &status, 1, 100);

        HAL_GPIO_WritePin(W25Q64_CS_GPIO_Port, W25Q64_CS_Pin, GPIO_PIN_SET);
        /* code */
    } while ((status & 0x01) == 0x01);
    
}
void WriteEnable(void)
{
    uint8_t cmd = 0x06;
    // 1. 拉低片选
    HAL_GPIO_WritePin(W25Q64_CS_GPIO_Port, W25Q64_CS_Pin, GPIO_PIN_RESET);
    // 2. 发送指令
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
    //拉高 写使能
    HAL_GPIO_WritePin(W25Q64_CS_GPIO_Port, W25Q64_CS_Pin, GPIO_PIN_SET);

}
void WriteByte(uint32_t WriteAddr , uint8_t *pData, uint16_t DataSize)
{
    uint8_t Write_Data[4];
    
    Write_Data[0] = 0x02; // 指令
    Write_Data[1] = (WriteAddr >> 16) & 0xFF; 
    Write_Data[2] = (WriteAddr >> 8) & 0xFF;
    Write_Data[3] = WriteAddr & 0xFF;
    //开启写使能
    WriteEnable();
    //拉低CS 发送信号
    HAL_GPIO_WritePin(W25Q64_CS_GPIO_Port, W25Q64_CS_Pin, GPIO_PIN_RESET);
    //发送写入指令 页编程
    HAL_SPI_Transmit(&hspi1, Write_Data, 4, 100);
    //写入数据
    HAL_SPI_Transmit(&hspi1, pData, DataSize, 100);
    //拉高cs 发送结束
    HAL_GPIO_WritePin(W25Q64_CS_GPIO_Port, W25Q64_CS_Pin, GPIO_PIN_SET);

    ReadStatus();
}

/* 读取W25Q64数据 */
void ReadByte(uint32_t address, uint8_t *pData, uint16_t DataSize)
{
    uint8_t cmd_add[4];

    cmd_add[0] = 0x03 & 0xFF;
    cmd_add[1] = (address >> 16) & 0xFF;
    cmd_add[2] = (address >> 8) & 0xFF;
    cmd_add[3] = address & 0xFF;
    //拉低开始
    HAL_GPIO_WritePin(W25Q64_CS_GPIO_Port, W25Q64_CS_Pin, GPIO_PIN_RESET);
    //发送读指令
    HAL_SPI_Transmit(&hspi1, cmd_add, 4, 100);
    //接收cmd数据
    HAL_SPI_Receive(&hspi1, pData, DataSize, 100);
    //拉高结束
    HAL_GPIO_WritePin(W25Q64_CS_GPIO_Port, W25Q64_CS_Pin, GPIO_PIN_SET);

}

/* 擦除W25Q64数据 */
void W25Q64_SectorErase(uint32_t Address)
{
    uint8_t cmd_addr[4];
    // 0x20 是扇区擦除指令
    cmd_addr[0] = 0x20; 
    cmd_addr[1] = (Address >> 16) & 0xFF; // 地址高位
    cmd_addr[2] = (Address >> 8) & 0xFF;  // 地址中位
    cmd_addr[3] = Address & 0xFF;         // 地址低位
    // 1. 擦除是“修改”操作，必须先开启写使能
    WriteEnable();
    // 2. 拉低片选 (开始)
    HAL_GPIO_WritePin(W25Q64_CS_GPIO_Port, W25Q64_CS_Pin, GPIO_PIN_RESET);
    // 3. 发送指令和地址 (STM32 -> W25Q64)
    HAL_SPI_Transmit(&hspi1, cmd_addr, 4, 100);
    // 4. 拉高片选 (结束)
    HAL_GPIO_WritePin(W25Q64_CS_GPIO_Port, W25Q64_CS_Pin, GPIO_PIN_SET);
    // 5. 死等擦除完成
    ReadStatus();
}
