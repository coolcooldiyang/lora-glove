/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_CH_NUM        5U      /* CH1~CH5 = PA2~PA6 = ADC1_IN2~IN6 */
#define ADC_VREF_MV       3300U   /* VDDA = 3.3V */
#define ADC_FULL_SCALE    4095U   /* 12bit */

/* ---------------- 往从机发帧的节奏 ----------------
   以前是固定 500ms 发一帧，所以手上动作要等下一帧才反映到灯上，看起来"一级一级慢慢变"。
   现在改成：状态一变就立刻发（最小间隔 20ms 防抖），状态不变也至少 200ms 发一次保底。 */
#define SEND_MIN_INTERVAL_MS 20U    /* 两次上报的最小间隔（防止读数抖动时猛发） */
#define SEND_KEEPALIVE_MS    200U   /* 状态没变也至少这么久发一次 */
#define LED_BLINK_MS         500U   /* 主机板载蓝灯闪烁周期（和上报解耦） */

/* 迟滞（mV）：读数在门限附近抖动时，灯的档位不要来回跳。
   规则：>门限 -> 算 1；<门限-迟滞 -> 算 0；中间这段保持上一次状态不变 */
#define GLOVE_HYST_MV        40U

/* ---------------- PA0 按键标定相关 ---------------- */
#define KEY_GPIO_PORT     GPIOA
#define KEY_GPIO_PIN      GPIO_PIN_0
#define KEY_PRESSED_LEVEL 0U      /* 0 = 低电平按下（按键另一端接 GND，本板就是这样）
                                     1 = 高电平按下（按键另一端接 3.3V） */
#define KEY_DEBOUNCE_MS   20U     /* 按键去抖时间 */
#define KEY_SCAN_MS       10U     /* 主循环扫描按键的间隔 */
#define ADC_AVG_TIMES     10U     /* 每次按下采集 10 次 */
#define ADC_AVG_INTERVAL_MS 20U   /* 每次采集间隔 20ms */

/* 标定有效性：两次采样（伸直 / 弯曲）的差至少要有这么大，否则认为这一通道没真正变过姿势，
   标定无效 -> 保留原来的门限。100mV 是经验值，嫌松/紧可以调 */
#define GLOVE_CAL_MIN_DIFF_MV 100U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint16_t adc_raw[ADC_CH_NUM];   /* 本帧 ADC 原始值 0~4095 */
static uint32_t adc_mv[ADC_CH_NUM];    /* 换算后的电压 mV  */

/* DMA 的目标缓冲区：ADC 连续转换 + DMA 循环搬运，这里始终是"最新一帧"的 5 个值 */
static uint16_t adc_dma_buf[ADC_CH_NUM];

/* CH1~CH5 对应的 ADC 通道：PA2=IN2 ... PA6=IN6 */
static const uint32_t ADC_ChannelList[ADC_CH_NUM] =
{
  ADC_CHANNEL_2, ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6
};
static const uint32_t ADC_RankList[ADC_CH_NUM] =
{
  ADC_REGULAR_RANK_1, ADC_REGULAR_RANK_2, ADC_REGULAR_RANK_3,
  ADC_REGULAR_RANK_4, ADC_REGULAR_RANK_5
};

/* ---------------- 按键标定相关 ----------------
   第 1 次按下：采 10 次求平均 -> adc_avg1[]
   第 2 次按下：采 10 次求平均 -> adc_avg2[]
   第 3 次按下：glove_off[i] = (adc_avg1[i] + adc_avg2[i]) / 2，并回到第 1 次重新开始
   例：CH1 第一次 1500、第二次 1600 -> glove_off[0] = 1550                            */
static uint16_t adc_avg1[ADC_CH_NUM];      /* 第一次按下的平均值（mV） */
static uint16_t adc_avg2[ADC_CH_NUM];      /* 第二次按下的平均值（mV） */
static uint16_t glove_off[ADC_CH_NUM] =    /* 判定门限（mV），标定前先用原来的固定值 */
{
  8000, 8000, 8000, 1550, 1600
};
static uint8_t  key_stage = 0U;            /* 0=等第1次  1=已采第1次  2=已采第2次 */
static uint32_t last_send_tick = 0U;       /* 上一次往从机发帧的时刻 */
static uint8_t  last_sent_bits = 0xFFU;    /* 上一次发出去的 5bit 状态（0xFF = 还没发过，保证上电先发一帧） */
static uint32_t led_tick = 0U;             /* 板载蓝灯闪烁计时 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  把 ADC1 配成 5 个规则通道 + DMA 循环搬运（CH1~CH5 = PA2~PA6 = IN2~IN6）
  * @note   1) CubeMX 里 ADC1 目前只勾了 1 个转换（IN2），所以这里重新配一遍；
  *            以后在 CubeMX 里把 5 个通道都勾上，这段再跑一次也不冲突（幂等）。
  *         2) 全程不用中断：ADC 连续转换，DMA 循环往 adc_dma_buf[] 搬，
  *            CPU 只在要上报时读一次缓冲区，完全不参与搬运。
  *         3) 放在 USER CODE 区，CubeMX 重新生成代码不会覆盖。
  */
static void ADC1_Init_5CH_DMA(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /* ------------------ 1. ADC1：5 通道 + 连续转换 + 允许 DMA 循环请求 ------------------ */
  HAL_ADC_DeInit(&hadc1);          /* 复位 ADC，才允许改 NbrOfConversion/ScanConvMode/ContinuousConvMode */

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV2;  /* PCLK 64M / 2 = 32MHz */
  hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode          = ADC_SCAN_ENABLE;           /* 序列扫描 */
  hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait      = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff  = DISABLE;
  hadc1.Init.ContinuousConvMode    = ENABLE;                    /* DMA 模式：连续转换不停机 */
  hadc1.Init.NbrOfConversion       = ADC_CH_NUM;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;                    /* ADC 侧持续发 DMA 请求（配合循环模式） */
  hadc1.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;  /* 连续采集用覆盖，避免溢出卡死 */
  hadc1.Init.SamplingTimeCommon1   = ADC_SAMPLETIME_39CYCLES_5; /* 采样时间给足，外部传感器也能测 */
  hadc1.Init.SamplingTimeCommon2   = ADC_SAMPLETIME_39CYCLES_5;
  hadc1.Init.OversamplingMode      = DISABLE;
  hadc1.Init.TriggerFrequencyMode  = ADC_TRIGGER_FREQ_HIGH;

  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /* rank1~rank5 依次对应 CH1~CH5，DMA 也就按这个顺序搬进 adc_dma_buf[] */
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  for (uint8_t i = 0; i < ADC_CH_NUM; i++)
  {
    sConfig.Channel = ADC_ChannelList[i];
    sConfig.Rank    = ADC_RankList[i];
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
      Error_Handler();
    }
  }

  /* 上电校准一次，提高精度 */
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /* ------------------ 2. DMA1 Channel1：外设(ADC->DR) → 内存(adc_dma_buf) ------------------
     CubeMX 已经在 adc.c 的 HAL_ADC_MspInit() 里把 DMA1_Channel1 配好并挂到 hadc1 上了
     （DMA 请求 = ADC1，外设→内存，半字，内存递增），这里只改一处：模式 Normal -> Circular，
     让它搬满 5 个自动回到开头，永远循环搬运。                                          */
  __HAL_RCC_DMA1_CLK_ENABLE();

  if (hadc1.DMA_Handle == NULL)      /* CubeMX 里 DMA 没勾上时会走到这里 */
  {
    Error_Handler();
  }
  hadc1.DMA_Handle->Init.Mode = DMA_CIRCULAR;
  if (HAL_DMA_Init(hadc1.DMA_Handle) != HAL_OK)
  {
    Error_Handler();
  }

  /* ------------------ 3. 启动：只启动这一次，之后 DMA 一直在后台搬运 ------------------ */
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buf, ADC_CH_NUM) != HAL_OK)
  {
    Error_Handler();
  }

  /* 本方案刻意不用中断：一帧只要 ~8.5us，若开着 DMA 传输完成中断，中断频率会到 100kHz 以上，
     白白吃 CPU。这里把中断源关掉，主循环想读的时候直接读缓冲区就行。
     （CubeMX 在 NVIC 里打开的 DMA1_Channel1_IRQn 因此永远不会被触发，留着不影响） */
  __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);
}

/**
  * @brief  取一帧数据（直接从 DMA 缓冲区读，不触发转换、不等待）
  * @note   缓冲区由 DMA 循环覆盖，取到的值最多比"此刻"旧一帧（约 8.5us），做电压上报完全够用。
  *         如果一定要"整帧快照 + 中断通知"，别用 5 个 half-word 的循环缓冲开 TC 中断
  *         （中断频率 100kHz+），要么把缓冲开大几帧，要么改成定时器触发采样。
  */
static void ADC_GetFrame(uint16_t *dst)
{
  for (uint8_t i = 0; i < ADC_CH_NUM; i++)
  {
    dst[i] = adc_dma_buf[i];
  }
}

static void UART_SendStr(const char *s)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), 100U);
}

/*==============================================================================
 *                          PA0 按键 + 标定
 *============================================================================*/

/* 读按键当前是否按下（按 KEY_PRESSED_LEVEL 判断有效电平） */
static uint8_t key_is_pressed(void)
{
  uint8_t level = (HAL_GPIO_ReadPin(KEY_GPIO_PORT, KEY_GPIO_PIN) == GPIO_PIN_SET) ? 1U : 0U;

#if (KEY_PRESSED_LEVEL == 0U)
  return (level == 0U) ? 1U : 0U;     /* 低电平按下 */
#else
  return (level == 1U) ? 1U : 0U;     /* 高电平按下 */
#endif
}

/* 按键扫描（要经常调用，比如每 10ms 一次）；返回 1 = 检测到一次“按下” */
static uint8_t key_scan_press(void)
{
  static uint8_t  stable = 0U;              /* 去抖后的稳定状态：1=按下 */
  static uint32_t stable_tick = 0U;

  uint8_t now = key_is_pressed();

  if (now == stable)
  {
    stable_tick = HAL_GetTick();            /* 状态没变，刷新计时 */
  }
  else if ((HAL_GetTick() - stable_tick) >= KEY_DEBOUNCE_MS)
  {
    stable = now;                           /* 状态稳定保持够了，认可这次变化 */
    if (stable != 0U)
    {
      return 1U;                            /* 刚刚按下 */
    }
  }
  return 0U;
}

/* 连续采 10 次（每次间隔 20ms）求平均，结果换算成 mV 存到 dst[] */
static void adc_sample_avg10(uint16_t *dst)
{
  uint32_t sum[ADC_CH_NUM] = {0};
  uint32_t avg_raw;

  for (uint8_t n = 0U; n < ADC_AVG_TIMES; n++)
  {
    ADC_GetFrame(adc_raw);                  /* 从 DMA 缓冲取一帧（5 个通道） */
    for (uint8_t ch = 0U; ch < ADC_CH_NUM; ch++)
    {
      sum[ch] += adc_raw[ch];
    }
    HAL_Delay(ADC_AVG_INTERVAL_MS);         /* 间隔 20ms 再采下一次 */
  }

  for (uint8_t ch = 0U; ch < ADC_CH_NUM; ch++)
  {
    avg_raw = (sum[ch] + (ADC_AVG_TIMES / 2U)) / ADC_AVG_TIMES;                        /* 平均值(原始值) */
    dst[ch] = (uint16_t)((avg_raw * ADC_VREF_MV + (ADC_FULL_SCALE / 2U)) / ADC_FULL_SCALE); /* 换算成 mV，四舍五入 */
  }
}

/* 按下一次按键要干的事：第1次/第2次采样，第3次算门限 */
static void key_on_press(void)
{
  char line[110];
  int  len;

  switch (key_stage)
  {
    case 0U:                                /* 第 1 次按下 */
      adc_sample_avg10(adc_avg1);
      key_stage = 1U;
      len = snprintf(line, sizeof(line), "KEY1 avg: %u %u %u %u %u (mV)\r\n",
                     (unsigned)adc_avg1[0], (unsigned)adc_avg1[1], (unsigned)adc_avg1[2],
                     (unsigned)adc_avg1[3], (unsigned)adc_avg1[4]);
      break;

    case 1U:                                /* 第 2 次按下 */
      adc_sample_avg10(adc_avg2);
      key_stage = 2U;
      len = snprintf(line, sizeof(line), "KEY2 avg: %u %u %u %u %u (mV)\r\n",
                     (unsigned)adc_avg2[0], (unsigned)adc_avg2[1], (unsigned)adc_avg2[2],
                     (unsigned)adc_avg2[3], (unsigned)adc_avg2[4]);
      break;

    default:                                /* 第 3 次按下：取两次平均值的平均 */
    {
      uint8_t  ok_cnt = 0U;                 /* 本次真正标定成功的通道数 */
      uint16_t diff;

      for (uint8_t ch = 0U; ch < ADC_CH_NUM; ch++)
      {
        /* 两次采样的差（绝对值）——差太小说明两次采的是同一个姿势，标定无效 */
        diff = (adc_avg1[ch] > adc_avg2[ch]) ? (uint16_t)(adc_avg1[ch] - adc_avg2[ch])
                                             : (uint16_t)(adc_avg2[ch] - adc_avg1[ch]);

        if (diff >= GLOVE_CAL_MIN_DIFF_MV)
        {
          glove_off[ch] = (uint16_t)(((uint32_t)adc_avg1[ch] + (uint32_t)adc_avg2[ch] + 1U) / 2U);
          ok_cnt++;
        }
        /* else：这一通道两次姿势几乎一样，门限取中点会落在读数正中间，
           之后读数稍微一抖就全变 1（灯卡在 100% 不动）——所以保留原门限不改 */
      }

      key_stage = 0U;                       /* 回到第 1 次，下次按下重新标定 */
      len = snprintf(line, sizeof(line), "KEY3 off:%u %u %u %u %u ok=%u\r\n",
                     (unsigned)glove_off[0], (unsigned)glove_off[1], (unsigned)glove_off[2],
                     (unsigned)glove_off[3], (unsigned)glove_off[4], (unsigned)ok_cnt);
      break;
    }
  }

  if (len > 0)
  {
    HAL_UART_Transmit(&huart1, (uint8_t *)line, (uint16_t)len, 100U);
  }
}

uint8_t glove[5],glove_send = 0,send_data[3] = {0xCC,0x00,0xEE};
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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  ADC1_Init_5CH_DMA();                   /* ADC1 五通道 + DMA 循环搬运，启动一次即可 */

  /* PA0 按键：CubeMX 里配的是 Pull-down，但板上按键另一端接的是 GND（按下=低电平），
     松开时如果没有上拉引脚会悬空，所以这里把 PA0 重新配成内部上拉。
     ★ 如果你的按键是接 3.3V 的（按下=高电平），把这段删掉，并把 KEY_PRESSED_LEVEL 改成 1 */
  {
    GPIO_InitTypeDef key_gpio = {0};

    key_gpio.Pin  = KEY_GPIO_PIN;
    key_gpio.Mode = GPIO_MODE_INPUT;
    key_gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(KEY_GPIO_PORT, &key_gpio);
  }

  UART_SendStr("glove ready\r\n");        /* 上电提示 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint8_t  bits;
    uint32_t now;

    /* ---------------- 1. 每 10ms 扫一次 PA0 按键（去抖在 key_scan_press 里做） ---------------- */
    if (key_scan_press() != 0U)
    {
      /* 第1次按下采 avg1，第2次按下采 avg2，第3次按下算出 glove_off[] */
      key_on_press();
    }

    /* ---------------- 2. 每轮都算一次当前手套状态（读 DMA 缓冲 + 比门限，很快） ---------------- */
    ADC_GetFrame(adc_raw);                   /* DMA 在后台循环搬运，这里不触发转换、不等待 */

    for (uint8_t i = 0U; i < ADC_CH_NUM; i++)
    {
      uint16_t low_th;

      adc_mv[i] = ((uint32_t)adc_raw[i] * ADC_VREF_MV + (ADC_FULL_SCALE / 2U)) / ADC_FULL_SCALE;

      /* 带迟滞的判断：避免读数卡在门限上导致灯的档位来回跳 */
      low_th = (glove_off[i] > GLOVE_HYST_MV) ? (uint16_t)(glove_off[i] - GLOVE_HYST_MV) : 0U;

      if (adc_mv[i] > glove_off[i])
      {
        glove[i] = 1U;
      }
      else if (adc_mv[i] < low_th)
      {
        glove[i] = 0U;
      }
      /* else：落在门限附近的死区里，保持上一次状态不变 */
    }

    /* 只发 CH4/CH5 的数据：bit3 = CH4，bit4 = CH5；CH1~CH3 不发，对应位恒为 0。
       （★ 想改成紧凑格式 bit0=CH4、bit1=CH5，就把下面这行换成：
           bits = (uint8_t)(glove[3] + (glove[4] << 1)); ） */
    bits = (uint8_t)((glove[3] << 3) + (glove[4] << 4));

    if (key_stage != 0U)
    {
      /* 正在标定（按了第 1 次、还没按第 3 次）：不要用旧门限去控制灯，直接发 0（灯灭） */
      bits = 0U;
    }

    /* ---------------- 3. 该发就发：状态一变立刻发；不变则每 SEND_KEEPALIVE_MS 保底发一次 ----------------
       两次发送之间至少隔 SEND_MIN_INTERVAL_MS，避免读数抖动时猛发 */
    now = HAL_GetTick();
    if (((bits != last_sent_bits) || ((now - last_send_tick) >= SEND_KEEPALIVE_MS)) &&
        ((now - last_send_tick) >= SEND_MIN_INTERVAL_MS))
    {
      send_data[1]   = bits;
      last_sent_bits = bits;
      last_send_tick = now;

      HAL_UART_Transmit(&huart1, send_data, 3U, 100U);
    }

    /* ---------------- 4. 板载蓝灯：固定周期翻转，表示程序在跑 ---------------- */
    if ((now - led_tick) >= LED_BLINK_MS)
    {
      led_tick = now;
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
    }

    HAL_Delay(KEY_SCAN_MS);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
