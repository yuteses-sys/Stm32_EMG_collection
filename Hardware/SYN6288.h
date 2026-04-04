#ifndef __SYN6288_H
#define __SYN6288_H

// 必须包含 HAL 库头文件，否则编译器不认识 uint8_t 这种类型
#include "main.h"  
#include "usart.h"   
#include "string.h"  

void SYN6288_Say(char *text);

#endif
