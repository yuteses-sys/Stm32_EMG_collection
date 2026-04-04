#ifndef __EMG_H
#define __EMG_H

// 必须包含 HAL 库头文件，否则编译器不认识 uint8_t 这种类型
#include "main.h"  
#include "string.h"  

uint16_t EMG_Get(void);


#endif
