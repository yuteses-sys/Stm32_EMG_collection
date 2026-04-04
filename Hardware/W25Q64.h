#ifndef __W25Q64_H
#define __W25Q64_H

#include "main.h"
#include "spi.h"
#include "gpio.h"


void ReadStatus();
void WriteEnable(void);
void WriteByte(uint32_t WriteAddr , uint8_t *pData, uint16_t DataSize);
void ReadByte(uint32_t address, uint8_t *pData, uint16_t DataSize);
void W25Q64_SectorErase(uint32_t Address);

#endif
