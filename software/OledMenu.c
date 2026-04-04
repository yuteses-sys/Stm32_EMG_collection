#include "OledMenu.h"

uint8_t Current_Menu_Index = 0;
static uint8_t Last_Menu_Index = 0xFF; // 用于记录上一次的页面索引，0xFF确保开机首次必定刷新

// 页面函数声明
void Page_Standby(void);
void Page_Menu_Borrow(void);
void Page_Menu_AddCard(void);
void Page_Menu_Setting(void);

// 修复越界Bug：原表中 10, 20, 30 超出了数组界限(最大下标为3)。
// 现将确认键(Enter)暂时映射回 0(待机) 或停留在当前页，避免数组越界引发硬件错误(HardFault)。
Menu_Item_t Menu_Table[] = {
// 当前 | 上键 | 下键 | 确认 | 返回 | 这个房间专属的动作函数
{    0,      0,    0,     1 ,     0,      Page_Standby },
{    1,      3,    2,     1,      0,      Page_Menu_Borrow }, // 确认键暂时映射为1，或者你需要增加子菜单数组元素
{    2,      1,    3,     2,      0,      Page_Menu_AddCard },
{    3,      2,    1,     3,      0,      Page_Menu_Setting }
};

// ==========================================
// 页面绘制函数：仅操作显存，最后统一Update
// ==========================================
void Page_Standby(void) {
    OLED_Clear();   
    // 待机界面
    OLED_ShowString(32, 24, "Standby...", OLED_8X16);
    OLED_Update(); // 将显存写入 OLED RAM
}

void Page_Menu_Borrow(void) {
    OLED_Clear();   
    OLED_ShowString(16, 0,  "--- MENU ---", OLED_8X16);
    OLED_ShowString(16, 16, "1. Borrow", OLED_8X16);
    OLED_ShowString(16, 32, "2. Add Card", OLED_8X16);
    OLED_ShowString(16, 48, "3. Setting", OLED_8X16);
    
    // 绘制光标指示当前行 (Y轴坐标对应)
    OLED_ShowString(0, 16, ">", OLED_8X16); 
    OLED_Update();   
}

void Page_Menu_AddCard(void) {
    OLED_Clear();   
    OLED_ShowString(16, 0,  "--- MENU ---", OLED_8X16);
    OLED_ShowString(16, 16, "1. Borrow", OLED_8X16);
    OLED_ShowString(16, 32, "2. Add Card", OLED_8X16);
    OLED_ShowString(16, 48, "3. Setting", OLED_8X16);
    
    OLED_ShowString(0, 32, ">", OLED_8X16); 
    OLED_Update();   
}

void Page_Menu_Setting(void) {
    OLED_Clear();   
    OLED_ShowString(16, 0,  "--- MENU ---", OLED_8X16);
    OLED_ShowString(16, 16, "1. Borrow", OLED_8X16);
    OLED_ShowString(16, 32, "2. Add Card", OLED_8X16);
    OLED_ShowString(16, 48, "3. Setting", OLED_8X16);
    
    OLED_ShowString(0, 48, ">", OLED_8X16); 
    OLED_Update();   
}

// ==========================================
// 按键检测引擎：此处采用轮询防抖。如需极致性能，建议改用外部中断(EXTI)
// 结合定时器或OS任务进行消抖。
// ==========================================
void Menu_Key_Handler(void) {
    // --- 检查【上】键 (PB4) --- (对应寄存器：GPIOB->IDR & GPIO_IDR_IDR4)
    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET){
        HAL_Delay(20); // 简单延时消抖
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET){
            Current_Menu_Index = Menu_Table[Current_Menu_Index].up_State;
            while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET);
        }    
    }
    // --- 检查【下】键 (PB5) ---
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET) {
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET) {
            Current_Menu_Index = Menu_Table[Current_Menu_Index].down_State;
            while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET);
        }
    }
    // --- 检查【确认】键 (PB6) ---
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_RESET) {
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_RESET) {
            Current_Menu_Index = Menu_Table[Current_Menu_Index].enter_State;
            while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_RESET);
        }
    }
    // --- 检查【返回】键 (PB7) ---
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET) {
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET) {
            Current_Menu_Index = Menu_Table[Current_Menu_Index].back_State;
            while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET);
        }
    }
}

// ==========================================
// 屏幕刷新引擎：加入状态拦截，防止重复刷新打爆I2C总线
// ==========================================
void Menu_Task(void) {
    // 仅在菜单状态发生变化时，才调用对应的页面函数进行I2C数据下发
    if (Current_Menu_Index != Last_Menu_Index) {
        // 安全校验：防止指针越界引发 HardFault (PC指针飞跑)
        if (Current_Menu_Index < (sizeof(Menu_Table)/sizeof(Menu_Table[0]))) {
            Menu_Table[Current_Menu_Index].task_func();
        } else {
            // 越界保护，强制复位到主界面
            Current_Menu_Index = 0; 
            Menu_Table[0].task_func();
        }
        Last_Menu_Index = Current_Menu_Index; // 同步状态
    }
}
void Menu_Init(void) {
    Current_Menu_Index = 0;
    Last_Menu_Index = 0xFF;     // 确保状态机认为发生了切换
    Menu_Table[0].task_func();  // 强制刷出待机界面首帧
}