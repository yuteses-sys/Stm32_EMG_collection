#include "EMG.h" // 引用自己的头文件

extern volatile uint16_t ADC_Values[2]; 
uint16_t EMG_Get(void)
{
    //直接返回DMA中ADC的数值
    return  ADC_Values[0];
}