#ifndef __OLEDMENU_H
#define __OLEDMENU_H

#include "main.h"          
#include "i2c.h"           
#include "OLED.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>



typedef struct{
    uint8_t current_State;

    uint8_t up_State;
    uint8_t down_State;
    uint8_t enter_State;
    uint8_t back_State;

    void (*task_func)(void);
}Menu_Item_t;

extern uint8_t Current_Menu_Index;


void Menu_Key_Handler(void);
void Menu_Task(void);
void  Menu_Init();
#endif
