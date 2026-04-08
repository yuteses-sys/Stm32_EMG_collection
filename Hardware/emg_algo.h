#ifndef __EMG_ALGO_H
#define __EMG_ALGO_H

#include "main.h"
#include "arm_math.h"

// --- 算法核心参数配置 ---
#define FFT_LENGTH 256               // 采样点数
#define SAMPLE_RATE 200.0f           // 采样频率 200Hz
#define STABLE_THRESHOLD_RATIO 0.25f // 力量波动 <25% 认为稳定
#define MIN_FORCE_FATIGUE 60         // 最小发力门限（低于此值认为放松）
#define RELAX_CLEAR_DELAY_MS 500     // 放松持续500ms后才清零疲劳度
#define NOISE_GATE_THRESHOLD 10.0f   // 底噪静音门限

// --- 对外提供的函数接口 ---
// 1. 算法初始化 (传入开机粗略采集的基线)
void EMG_Algo_Init(uint32_t initial_baseline);

// 2. 核心处理函数 (每5ms调用一次，传入最新的ADC值。返回1代表FFT刚算完一轮，返回0代表还在采集中)
uint8_t EMG_Algo_Process(uint16_t raw_adc);

// 3. 数据获取接口 (Getters)
uint32_t EMG_Get_Envelope(void);
float32_t EMG_Get_MedianFreq(void);
float32_t EMG_Get_Fatigue(void);

#endif