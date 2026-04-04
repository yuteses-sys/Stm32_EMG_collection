#include "FSR_Sensor.h"
#include "adc.h"

extern volatile uint16_t ADC_Values[2]; 

void FSR_Init(void)
{
    // 因为 ADC 和 DMA 已经在 main() 函数开头通过 HAL_ADC_Start_DMA 启动了
    // 所以这里其实不需要写任何硬件启动代码
    // 留个空函数是为了保持代码结构统一，方便以后扩展
}
uint16_t FSR_ReadRaw(void)
{
    // 静态变量：保留上一次的滤波结果
    static uint32_t filtered_val = 0; 
    
    // 获取当前原始值（逻辑翻转后）
    uint16_t current_raw = 4095 - ADC_Values[1]; 
    
    // 一阶互补滤波：平滑度取决于系数（如 0.2 和 0.8）
    // 数值越小越丝滑，但响应会变慢
    filtered_val = (filtered_val * 8 + current_raw * 2) / 10;
    
    return (uint16_t)filtered_val;
}
Fsr_Sensor FSR_GetState(void)
{
    uint16_t val = FSR_ReadRaw();

    // 逻辑：数值越大 = 力越大
    if (val > FSR_TH_GRIP) 
    {
        return Fsr_Grip ; // 抓紧了
    }
    else if (val > FSR_TH_TOUCH) 
    {
        return Fsr_Touch; // 摸到了
    }
    else 
    {
        return Fsr_None; // 没碰到
    }
}