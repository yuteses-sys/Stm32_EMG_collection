#ifndef __FSR_SENSOR_H
#define __FSR_SENSOR_H

#include "main.h"

// --- 宏定义 ---
#define FSR_DEAD_ZONE 30.0f      // 死区门限，消除轻微误触
#define FSR_MAX_FORCE 10000.0f   // 假设最大受力量程 (比如 10000克)

// --- 接口函数 ---
// 1. 初始化
void FSR_Init(void);

// 2. 数据处理函数 (每5ms调用，传入ADC原始值，内部进行滤波和换算)
void FSR_Process(uint16_t raw_adc);

// 3. 获取最新的力值
float FSR_Get_Force(void);

#endif