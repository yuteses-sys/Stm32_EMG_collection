/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (裸机 EMG 动态校准与包络测试版)
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
#include "OledMenu.h"
#include "EMG.h"
#include <math.h>
#include "arm_math.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
volatile uint16_t ADC_Values[2] = {0};
extern UART_HandleTypeDef huart3;

/* USER CODE END PD */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  
  /* USER CODE BEGIN 2 */
  OLED_Init();
  OLED_Clear();
  OLED_ShowString(0, 0, "EMG Testing...", OLED_8X16);
  OLED_Update();

  // 1. 启动底层 ADC 和 定时器触发的 DMA 搬运
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Values, 2);
  HAL_TIM_Base_Start(&htim3);
  
  printf("System Start! Calibrating baseline...\r\n");
  HAL_Delay(1000); // 延时1秒，等待运放电路和皮肤接触完全稳定

  // ===== 核心：开机自动校准真实基准线 =====
  uint32_t dc_sum = 0;
  // 连续采样 500 次求平均，得出当下最精准的放松基线
  for(int i = 0; i < 500; i++) {
      dc_sum += ADC_Values[0];
      HAL_Delay(2); 
  }
  int32_t dynamic_baseline = dc_sum / 500;
  printf("Calibration OK! Baseline: %ld\r\n", dynamic_baseline);
  
  // 定义包络线存储变量
  static uint32_t emg_envelope = 0; 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // 1. 抓取当前原始数据
      uint16_t emg_raw = ADC_Values[0];
      
      // 2. 减去开机时自动校准出来的真实基线 (消除直流偏置)
      int32_t emg_ac = (int32_t)emg_raw - dynamic_baseline;
      
      // 3. 全波整流 (将负半轴信号翻折成正数)
      if (emg_ac < 0) {
          emg_ac = -emg_ac;
      }
      
      // 4. 提取发力包络 (指数滑动平均滤波 EMA)
      // 比例 80:20，兼顾平滑度与肌肉响应速度
      emg_envelope = (emg_envelope * 80 + emg_ac * 20) / 100;
      
      // 5. 打印最终结果给 Vofa+
      // 注意：这里只发一个数，就是干净的包络力量值！
      printf("%lu\n", emg_envelope);
      
      // 延时 5ms 控制串口发送速率
      HAL_Delay(5);
      
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
#pragma import(__use_no_semihosting)             
struct __FILE 
{ 
    int handle; 
}; 
FILE __stdout;       

void _sys_exit(int x) 
{ 
    x = x; 
} 

int fputc(int ch, FILE *f)
{      
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
/* USER CODE END 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}