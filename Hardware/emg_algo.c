#include "emg_algo.h"
#include <math.h>

// ================== 私有全局变量 ==================
static arm_rfft_fast_instance_f32 S;
static float32_t fft_input[FFT_LENGTH];
static float32_t fft_output[FFT_LENGTH];
static float32_t fft_mag[FFT_LENGTH / 2];
static uint16_t sample_count = 0;

static float dynamic_baseline_f = 0.0f;
static float emg_envelope_f = 0.0f;
static uint32_t emg_envelope = 0;

static float32_t median_freq = 0.0f;
static float32_t smoothed_mf = 0.0f;
static float32_t fatigue_percent = 0.0f;
static uint32_t prev_envelope = 0;

static uint32_t relax_start_time = 0;
static uint8_t was_relaxed = 0;

// ================== 对外接口实现 ==================

void EMG_Algo_Init(uint32_t initial_baseline) {
    arm_rfft_fast_init_f32(&S, FFT_LENGTH);
    dynamic_baseline_f = (float)initial_baseline;
    sample_count = 0;
}

uint8_t EMG_Algo_Process(uint16_t raw_adc) {
    uint8_t fft_completed = 0;

    // 1. 动态基线追踪
    dynamic_baseline_f = dynamic_baseline_f * 0.998f + (float)raw_adc * 0.002f;
    // 2. 提取交流与静噪
    float emg_ac_f = (float)raw_adc - dynamic_baseline_f;
    float emg_rectified_f = fabsf(emg_ac_f); 
    if (emg_rectified_f < NOISE_GATE_THRESHOLD) {
        emg_rectified_f = 0.0f; 
    }
    // 3. 包络提取
    emg_envelope_f = emg_envelope_f * 0.90f + emg_rectified_f * 0.10f;
    emg_envelope = (uint32_t)emg_envelope_f;
    // 4. 塞入FFT水桶
    fft_input[sample_count] = emg_ac_f;
    sample_count++;
    
    // 5. 满足 256 个点，执行核心解算
    if (sample_count >= FFT_LENGTH) {
        // 执行 FFT
        arm_rfft_fast_f32(&S, fft_input, fft_output, 0);
        arm_cmplx_mag_f32(fft_output, fft_mag, FFT_LENGTH / 2);
        
        // 强力陷波 50Hz (索引 62~66)
        for (int i = 62; i <= 66; i++) {
            if (i < FFT_LENGTH / 2) fft_mag[i] = 0.0f;
        } 
        // 计算中位频率
        float32_t total_power = 0.0f;
        for (int i = 1; i < FFT_LENGTH / 2; i++) total_power += fft_mag[i];
        float32_t current_power = 0.0f;
        uint16_t mf_index = 0;
        if (total_power > 0) {
            for (int i = 1; i < FFT_LENGTH / 2; i++) {
                current_power += fft_mag[i];
                if (current_power >= total_power * 0.5f) {
                    mf_index = i;
                    break;
                }
            }
        }
        float32_t raw_median_freq = (float32_t)mf_index * (SAMPLE_RATE / (float32_t)FFT_LENGTH);
        // 频率平滑
        if (smoothed_mf == 0.0f) smoothed_mf = raw_median_freq;
        smoothed_mf = (smoothed_mf * 95.0f + raw_median_freq * 5.0f) / 100.0f;
        median_freq = smoothed_mf;
        // 疲劳度解算与延时归零状态机
        uint32_t envelope_diff = (emg_envelope > prev_envelope) ? (emg_envelope - prev_envelope) : (prev_envelope - emg_envelope);
        uint32_t max_env = (emg_envelope > prev_envelope) ? emg_envelope : prev_envelope;
        uint32_t stable_condition = (envelope_diff < (max_env * STABLE_THRESHOLD_RATIO));
        if (emg_envelope > MIN_FORCE_FATIGUE && stable_condition) {
            if (median_freq >= 55.0f) fatigue_percent = 0.0f;
            else if (median_freq <= 28.0f) fatigue_percent = 100.0f;
            else fatigue_percent = (55.0f - median_freq) / (55.0f - 28.0f) * 100.0f;
            
            relax_start_time = 0;
            was_relaxed = 0;
        } 
        else if (emg_envelope <= MIN_FORCE_FATIGUE) {
            if (!was_relaxed) {
                relax_start_time = HAL_GetTick();
                was_relaxed = 1;
            }
            if (HAL_GetTick() - relax_start_time > RELAX_CLEAR_DELAY_MS) {
                fatigue_percent = 0.0f;
            }
        }
        else {
            relax_start_time = 0;
            was_relaxed = 0;
        }
        prev_envelope = emg_envelope;
        sample_count = 0;
        fft_completed = 1; // 标记：算完了一轮！
    }
    
    return fft_completed;
}

// --- 数据获取函数 ---
uint32_t EMG_Get_Envelope(void) { return emg_envelope; }
float32_t EMG_Get_MedianFreq(void) { return median_freq; }
float32_t EMG_Get_Fatigue(void) { return fatigue_percent; }