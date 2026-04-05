/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 裸机 EMG 动态浮点基线追踪、包络提取、50Hz陷波与平滑 FFT 疲劳分析
  * @version        : 4.0 (彻底解决放松不归零与基线漂移问题)
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

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FFT_LENGTH 256               // 采样点数
#define SAMPLE_RATE 200.0f           // 采样频率 200Hz
#define SAMPLE_INTERVAL_MS 5         // 采样间隔 5ms
#define STABLE_THRESHOLD_RATIO 0.25f // 力量波动 <25% 认为稳定（放宽，适应更多波动）
#define MIN_FORCE_FATIGUE 60         // 最小发力门限（低于此值认为放松）
#define RELAX_CLEAR_DELAY_MS 500     // 放松持续500ms后才清零疲劳度
#define NOISE_GATE_THRESHOLD 10.0f   // 底噪静音门限（去除ADC微小抖动，确保放松时归零）
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint16_t ADC_Values[2] = {0};
extern UART_HandleTypeDef huart3;

// FFT 相关
float32_t fft_input[FFT_LENGTH];
float32_t fft_output[FFT_LENGTH];
float32_t fft_mag[FFT_LENGTH / 2];
uint16_t sample_count = 0;
float32_t median_freq = 0.0f;
arm_rfft_fast_instance_f32 S;

// 疲劳度相关
static float32_t fatigue_percent = 0.0f;
static float32_t smoothed_mf = 0.0f;
static uint32_t prev_envelope = 0;

// 放松计时器（用于延时清零）
static uint32_t relax_start_time = 0;
static uint8_t was_relaxed = 0;
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
  HAL_Init();
  SystemClock_Config();

  // 初始化外设
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
  OLED_ShowString(0, 0, "EMG System Init", OLED_8X16);
  OLED_Update();

  // 初始化 FFT
  arm_rfft_fast_init_f32(&S, FFT_LENGTH);

  // 启动 ADC + DMA
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Values, 2);
  HAL_TIM_Base_Start(&htim3);

  HAL_Delay(1000); // 等待电路稳定

  // 开机粗略校准初始基线
  uint32_t dc_sum = 0;
  for(int i = 0; i < 500; i++) {
      dc_sum += ADC_Values[0];
      HAL_Delay(2);
  }
  // 【修改点1】将基线改为浮点型，作为动态追踪的起点
  float dynamic_baseline_f = (float)dc_sum / 500.0f;

  // 包络变量改为浮点型，防止低幅值被整数截断
  static float emg_envelope_f = 0.0f;
  static uint32_t emg_envelope = 0;

  // 精准定时变量
  uint32_t last_sample_tick = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    uint32_t current_tick = HAL_GetTick();

    // ========== 严格 5ms 采样定时 ==========
    if (current_tick - last_sample_tick >= SAMPLE_INTERVAL_MS) {
        last_sample_tick += SAMPLE_INTERVAL_MS; // 步进累加，消除漂移

        // 1. 获取原始 ADC 值
        uint16_t emg_raw = ADC_Values[0];

        // 削顶警告
        if (emg_raw >= 4000) {
            printf("WARNING: ADC Clipping!\n");
        }

        // 2. 实时动态基线追踪 (极慢速的低通滤波器，提取DC偏置)
        // 0.998 和 0.002 允许基线跟随皮肤出汗导致的漂移，但不会抹除肌肉发力的快变化
        dynamic_baseline_f = dynamic_baseline_f * 0.998f + (float)emg_raw * 0.002f;

        // 3. 提取交流分量
        float emg_ac_f = (float)emg_raw - dynamic_baseline_f;
        
        // 4. 整流与静噪门 (Noise Gate)
        // 【修改点2】强制消除微弱底噪，让放松时信号彻底为0
        float emg_rectified_f = fabsf(emg_ac_f); 
        if (emg_rectified_f < NOISE_GATE_THRESHOLD) {
            emg_rectified_f = 0.0f; 
        }

        // 5. 包络提取 (平滑滤波，使用浮点运算)
        // 0.90 和 0.10 使得曲线平滑，不会出现剧烈锯齿
        emg_envelope_f = emg_envelope_f * 0.90f + emg_rectified_f * 0.10f;

        // 转回整数供后续逻辑判断使用
        emg_envelope = (uint32_t)emg_envelope_f;

        // 6. 将交流信号存入 FFT 输入缓存
        fft_input[sample_count] = emg_ac_f;
        sample_count++;

        // 7. 当采集够 FFT_LENGTH 个点时，进行一次频谱分析
        if (sample_count >= FFT_LENGTH) {
            // 执行 FFT
            arm_rfft_fast_f32(&S, fft_input, fft_output, 0);
            arm_cmplx_mag_f32(fft_output, fft_mag, FFT_LENGTH / 2);

            // 强力陷波 50Hz 及其邻近频点（索引 62~66）
            for (int i = 62; i <= 66; i++) {
                if (i < FFT_LENGTH / 2) fft_mag[i] = 0.0f;
            }

            // 计算中位频率 (Median Frequency)
            float32_t total_power = 0.0f;
            for (int i = 1; i < FFT_LENGTH / 2; i++) {
                total_power += fft_mag[i];
            }

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

            // 中位频率平滑 (一阶低通)
            if (smoothed_mf == 0.0f) smoothed_mf = raw_median_freq;
            smoothed_mf = (smoothed_mf * 95.0f + raw_median_freq * 5.0f) / 100.0f;
            median_freq = smoothed_mf;

            // ========== 智能疲劳度更新（带有延时清零逻辑） ==========
            uint32_t envelope_diff = (emg_envelope > prev_envelope) ?
                                     (emg_envelope - prev_envelope) :
                                     (prev_envelope - emg_envelope);

            uint32_t max_env = (emg_envelope > prev_envelope) ? emg_envelope : prev_envelope;
            uint32_t stable_condition = (envelope_diff < (max_env * STABLE_THRESHOLD_RATIO));

            if (emg_envelope > MIN_FORCE_FATIGUE && stable_condition) {
                // 发力且力量稳定，更新疲劳度
                if (median_freq >= 55.0f) {
                    fatigue_percent = 0.0f;
                } else if (median_freq <= 28.0f) {
                    fatigue_percent = 100.0f;
                } else {
                    fatigue_percent = (55.0f - median_freq) / (55.0f - 28.0f) * 100.0f;
                }
                // 重置放松计时器
                relax_start_time = 0;
                was_relaxed = 0;
            } 
            else if (emg_envelope <= MIN_FORCE_FATIGUE) {
                // 【已修复】包络归零后，此处逻辑将被成功触发
                if (!was_relaxed) {
                    relax_start_time = HAL_GetTick();
                    was_relaxed = 1;
                }
                // 放松持续时间大于设定值，执行清零
                if (HAL_GetTick() - relax_start_time > RELAX_CLEAR_DELAY_MS) {
                    fatigue_percent = 0.0f;
                }
            }
            else {
                // 力量不稳定，保持上一次状态不突变
                relax_start_time = 0;
                was_relaxed = 0;
            }

            prev_envelope = emg_envelope;

            // 显示到 OLED
            char mf_str[16];
            char fat_str[16];
            sprintf(mf_str, "MF: %5.1f Hz  ", median_freq);
            sprintf(fat_str, "Fatigue:%3.0f %% ", fatigue_percent);
            OLED_ShowString(0, 32, (uint8_t*)mf_str, OLED_8X16);
            OLED_ShowString(0, 48, (uint8_t*)fat_str, OLED_8X16);
            OLED_Update();

            sample_count = 0;
        }

        // 8. 串口输出
        printf("%lu,%.2f,%.1f\n", emg_envelope, median_freq, fatigue_percent);
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();
}

/* USER CODE BEGIN 4 */
// 串口重定向
#pragma import(__use_no_semihosting)
struct __FILE { int handle; };
FILE __stdout;
void _sys_exit(int x) { x = x; }
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
/* USER CODE END 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4) HAL_IncTick();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif