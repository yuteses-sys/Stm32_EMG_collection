/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 基于STM32的肌肉发力检测系统 (终极商业版 - 双屏交互+持久化存储)
  * @version        : 7.0 (全量无删减版)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "W25Q64.h"
#include "OLED.h"
#include "FSR_Sensor.h"
#include <stdio.h>  
#include "string.h"
#include "syn6288.h" 
#include "emg_algo.h" 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

// ==============================================================================
// 硬件底层变量
// ==============================================================================
volatile uint16_t ADC_Values[2] = {0}; // 存ADC数据的数组：通道0是肌电，通道1是压力
extern UART_HandleTypeDef huart3;      // 串口3句柄，用来发送数据

// ==============================================================================
// 屏幕显示相关变量
// ==============================================================================
volatile uint8_t current_page = 0;       // 当前页面：0代表“实时检测页”，1代表“历史记录页”
volatile uint8_t action_key_pressed = 0; // 按键标志位（用来触发保存或翻页）
volatile uint8_t page_changed = 1;       // 刷新屏幕标志位（为1时重新画屏）

// ==============================================================================
// Flash存储相关变量
// ==============================================================================
uint32_t flash_save_address = 0x000000;  // 记录当前数据存到了Flash的哪个地址
uint32_t view_record_index = 0;          // 查看历史记录的索引，记录当前看到了第几条

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  /* 初始化单片机 HAL 库 */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* 配置系统时钟 (72MHz) */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* 初始化各个外设 */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  
  // 1. 初始化OLED屏幕，清屏并显示开机提示
  OLED_Init();
  OLED_Clear();
  OLED_ShowString(0, 0, "System Booting..", OLED_8X16);
  OLED_Update();
  
  // 2. 开启ADC校准，并启动DMA搬运ADC数据
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Values, 2);
  HAL_TIM_Base_Start(&htim3);
  HAL_Delay(1000); // 等待1秒，让传感器和皮肤接触稳定一点
  
  // 3. 采集500个点，计算一下当前的基准底噪
  uint32_t dc_sum = 0;
  for(int i = 0; i < 500; i++) {
      dc_sum += ADC_Values[0];
      HAL_Delay(2);
  }
  
  // 4. 初始化传感器和算法，传入算好的底噪
  FSR_Init();  
  EMG_Algo_Init(dc_sum / 500);
  
  // 5. 记录当前时间，准备每5ms跑一次处理逻辑
  uint32_t last_sample_tick = HAL_GetTick();
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t current_tick = HAL_GetTick();

    // =========================================================================
    // 后台数据处理：不管屏幕在显示什么，每5ms固定处理一次数据
    // =========================================================================
    if (current_tick - last_sample_tick >= 5) {
        last_sample_tick += 5; // 更新时间，保证严格的5ms间隔
        
        // --- 1. 处理压力传感器数据 ---
        FSR_Process(ADC_Values[1]);
        float force_value = FSR_Get_Force();
         
        // --- 2. 处理肌电算法算 FFT ---
        // (如果返回1，说明凑够了256个点，算完了一次频谱)
        uint8_t emg_fft_ready = EMG_Algo_Process(ADC_Values[0]);
        
        // --- 3. 计算“发力效率” (出力 / 肌电信号) ---
        float force_efficiency = 0.0f;
        // 只有确实出力了(大于50克)，才计算效率，防止除以0或数值乱跳
        if (EMG_Get_Envelope() > MIN_FORCE_FATIGUE && force_value > 50.0f) {
            force_efficiency = (force_value / (float)EMG_Get_Envelope()) * 10.0f;
        }


        // =====================================================================
        // 第 0 页 —— 实时监控界面
        // =====================================================================
        if (current_page == 0) {
            
            // --- 只有FFT刚算完，或者刚切到这个页面时，才刷新屏幕 ---
            if (emg_fft_ready == 1 || page_changed == 1) {
                page_changed = 0; // 清除换页标志
                
                char line1[24], line2[24], line3[24], line4[24];
                sprintf(line1, "Frc: %-4.0f g   ", force_value);        // 压力值
                sprintf(line2, "Eff: %.1f      ", force_efficiency);    // 发力效率
                sprintf(line3, "MF : %5.1f Hz  ", EMG_Get_MedianFreq()); // 肌电中值频率
                sprintf(line4, "Fat: %3.0f %%   ", EMG_Get_Fatigue());    // 疲劳度百分比
                
                OLED_Clear(); 
                OLED_ShowString(0, 0,  (uint8_t*)line1, OLED_8X16); 
                OLED_ShowString(0, 16, (uint8_t*)line2, OLED_8X16); 
                OLED_ShowString(0, 32, (uint8_t*)line3, OLED_8X16); 
                OLED_ShowString(0, 48, (uint8_t*)line4, OLED_8X16);
                OLED_Update();
            }

            // --- 通过串口发给电脑端查看 ---
            printf("%lu,%.2f,%.1f\n", EMG_Get_Envelope(), EMG_Get_MedianFreq(), EMG_Get_Fatigue());

            // --- 语音报警逻辑 ---
            static uint32_t last_voice_tick = 0;
            if (current_tick - last_voice_tick > 5000) { // 限制5秒报一次，防止语音重叠
                if (EMG_Get_Fatigue() > 80.0f) {
                    SYN6288_Say("警告，肌肉已严重疲劳，请立即放松");
                    last_voice_tick = current_tick;
                } 
                else if (force_value > 8000.0f) { 
                    SYN6288_Say("警告，负荷过高，注意安全");
                    last_voice_tick = current_tick;
                } 
                else if (EMG_Get_Envelope() > 100 && force_value > 1000.0f && force_efficiency < 1.5f) {
                    SYN6288_Say("发力效率偏低，请检查训练姿势");
                    last_voice_tick = current_tick;
                }
            }

            // --- 按下保存键的处理逻辑 ---
            if (action_key_pressed == 1) {
                action_key_pressed = 0; // 清除按键标志
                
                // 1. 把当前的数据格式化成一串字符
                char save_buf[128];
                sprintf(save_buf, "E:%lu F:%.0f MF:%.1f Fat:%.0f Eff:%.1f",
                        EMG_Get_Envelope(), force_value, EMG_Get_MedianFreq(), 
                        EMG_Get_Fatigue(), force_efficiency);

                // 2. 擦除对应的Flash扇区，把字符串存进去
                W25Q64_SectorErase(flash_save_address); 
                WriteByte(flash_save_address, (uint8_t*)save_buf, strlen(save_buf));
                
                // 3. 地址往后挪一个扇区(4096字节)，超限了就从头开始写
                flash_save_address += 4096; 
                if(flash_save_address >= 0x800000) flash_save_address = 0; 
                
                // 4. 屏幕和语音提示“保存成功”
                OLED_Clear();
                OLED_ShowString(16, 24, "Data Saved!", OLED_8X16);
                OLED_Update();
                SYN6288_Say("数据已保存");
                HAL_Delay(800);   // 画面停顿0.8秒让人看清
                page_changed = 1; // 标记一下，等会把实时画面刷新出来
            }
        }
        
        // =====================================================================
        // 第 1 页 —— 历史记录回看界面
        // =====================================================================
        else if (current_page == 1) {
            
            uint32_t total_records = flash_save_address / 4096; // 计算一下存了几条数据
            
            // --- 按下翻页键的处理逻辑 ---
            if (action_key_pressed == 1) {
                action_key_pressed = 0; // 清除按键标志
                view_record_index++;    // 看下一条记录
                if (view_record_index >= total_records) {
                    view_record_index = 0; // 翻到底了就回到第一条
                }
                page_changed = 1; // 标记需要刷新屏幕
            }
            // --- 读取Flash数据并显示 ---
            if (page_changed == 1) {
                page_changed = 0;
                OLED_Clear();
                
                if (total_records == 0) {
                    OLED_ShowString(0, 24, "No History Data", OLED_8X16);
                } else {
                    // 1. 显示当前是第几条记录
                    char title[24];
                    sprintf(title, "[Record %02lu/%02lu]", view_record_index + 1, total_records);
                    OLED_ShowString(0, 0, (uint8_t*)title, OLED_8X16);
                    // 2. 从Flash里读取存好的字符串
                    uint8_t read_buf[100] = {0};
                    ReadByte(view_record_index * 4096, read_buf, 60);
                    
                    // 3. 把长字符串拆开显示
                    char display_l2[24] = {0};
                    char display_l3[24] = {0};
                    char display_l4[24] = {0};
                    
                    // 用 sscanf 把字符串里的数值提取出来
                    uint32_t r_e; float r_f, r_mf, r_fat, r_eff;
                    sscanf((char*)read_buf, "E:%lu F:%f MF:%f Fat:%f Eff:%f", &r_e, &r_f, &r_mf, &r_fat, &r_eff);
                    
                    // 重新排版以便在OLED上显示
                    sprintf(display_l2, "EMG:%lu F:%.0f", r_e, r_f);
                    sprintf(display_l3, "MF :%.1f Hz", r_mf);
                    sprintf(display_l4, "Fat:%.0f%% Ef:%.1f", r_fat, r_eff);
                    
                    OLED_ShowString(0, 16, (uint8_t*)display_l2, OLED_8X16); 
                    OLED_ShowString(0, 32, (uint8_t*)display_l3, OLED_8X16); 
                    OLED_ShowString(0, 48, (uint8_t*)display_l4, OLED_8X16); 
                }
                OLED_Update();
            }
        }
    }
    /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// ===========================================================================
// 串口重定向与外部按键中断
// ===========================================================================

// 串口重定向，让 printf 能直接发给串口3
#pragma import(__use_no_semihosting)
struct __FILE { int handle; };
FILE __stdout;
void _sys_exit(int x) { x = x; }
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

// 外部中断回调函数，处理按键按下事件
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // --- 按键1：第0页保存，第1页翻页 ---
    if (GPIO_Pin == SW1_Pin) { 
        action_key_pressed = 1; 
    }
    
    // --- 按键2：切换页面（实时监测/历史记录） ---
    else if (GPIO_Pin == SW2_Pin) {  
        current_page = !current_page;   // 切换 0 和 1
        page_changed = 1;               // 标记需要刷新屏幕
        if (current_page == 1) {
            view_record_index = 0;     
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */