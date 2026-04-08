#include "FSR_Sensor.h"

// 私有全局变量
static float fsr_filtered = 0.0f;
static float current_force = 0.0f;

// 初始化函数
void FSR_Init(void) {
    fsr_filtered = 0.0f;
    current_force = 0.0f;
}

// 核心处理函数 (包含滤波、硬件反转和映射)
void FSR_Process(uint16_t raw_adc) {
    

    float inverted_adc = 4095.0f - (float)raw_adc;

    fsr_filtered = fsr_filtered * 0.85f + inverted_adc * 0.15f; 

    if (fsr_filtered > FSR_DEAD_ZONE) {
        current_force = (fsr_filtered / 4095.0f) * FSR_MAX_FORCE; 
    } else {
        current_force = 0.0f; 
    }
}

float FSR_Get_Force(void) {
    return current_force;
}