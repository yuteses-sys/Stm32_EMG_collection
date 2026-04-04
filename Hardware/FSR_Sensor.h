#ifndef __FSR_SENSOR_H__
#define __FSR_SENSOR_H__

#include "main.h"

#define FSR_TH_TOUCH    500   // 灵敏度：轻轻碰到 (大于这个值算 Touch)
#define FSR_TH_GRIP     3500  // 灵敏度：用力握紧 (大于这个值算 Grip)

typedef enum{
    Fsr_None = 0, //没碰到
    Fsr_Touch,    //轻触
    Fsr_Grip      //重握
} Fsr_Sensor;

// ================= 函数声明 =================
void FSR_Init(void);           // 初始化(其实是空的，为了格式统一)
uint16_t FSR_ReadRaw(void);    // 读原始值 (调试用)
Fsr_Sensor FSR_GetState(void);// 读状态 (业务用)

#endif
