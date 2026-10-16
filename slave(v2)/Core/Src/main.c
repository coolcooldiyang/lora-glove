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
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ===================== 从机：串口 DMA 接收 + 帧解析 + PWM 输出 =====================
   主机每 600ms 发一帧：  CC      data      EE
                         帧头     数据      帧尾      （固定 3 字节）
   规则：rx_data 里每有 1 个二进制 1，PA6 的 PWM 占空比就 +10%（最高 100%）
         例：0x00 -> 0%，0x01 -> 10%，0x07 -> 30%，0x1F -> 50%，0xFF -> 80%

   接收方式：环形缓冲 + 字节流状态机（不是"一次收固定 3 字节"）
     收到 0xCC 才当帧头、收到 0xEE 才认帧尾，中间夹进来的任何杂字节都会被丢掉，
     所以不管主机比从机晚开机多久、中间有没有打印调试信息，都能自动重新对齐。
   ===================================================================== */
#define RX_RING_SIZE    32u     /* 环形接收缓冲大小（字节） */
#define FRAME_HEAD      0xCCu   /* 帧头 */
#define FRAME_TAIL      0xEEu   /* 帧尾 */

/* ---------- PA6 = TIM3_CH1 PWM 输出（直接用寄存器配，不依赖 HAL 的 TIM 模块）----------
   PA6 的复用：AF1 = TIM3_CH1（STM32G030 数据手册 Table 13 Port A）
   本工程 CubeMX 里没勾 TIM，hal_conf.h 没开 HAL_TIM 模块，所以这里用寄存器配置，
   这样既不依赖 HAL 的 TIM 驱动，CubeMX 重新生成代码也不会把它冲掉。
   TIM3 时钟 = APB1 定时器时钟 = 64MHz
     PSC = 64-1   -> 计数器 1MHz
     ARR = 1000-1 -> PWM 频率 1kHz，占空比分辨率 0.1%
   规则：rx_data 里每有 1 个二进制 1，占空比 +10%（最高 100%） */
#define PWM_PRESCALER   (64u - 1u)
#define PWM_PERIOD      (1000u - 1u)
#define DUTY_PER_ONE    10u     /* rx_data 里每多 1 个 1，占空比增加 10% */

/* 输出脚：PA6 —— 板子上丝印的 CH5 网络就是 PA6（ADC1_IN6） */
#define OUT_GPIO_PORT   GPIOA
#define OUT_GPIO_PIN    GPIO_PIN_6

/* PB8 接收指示：串口在收数据时每 200ms 翻转一次；一段时间收不到就停住、保持当前电平 */
#define LED_BLINK_MS      200u    /* 有数据时 PB8 的翻转间隔 */
#define LED_RX_TIMEOUT_MS 1000u   /* 超过这么久没收到数据 -> 认为“没有接收”，PB8 保持不变 */

/* 串口接收 DMA：G030 只有 DMA1_Channel1~3，Channel1 被 ADC 占了，这里用 Channel2 */
#define RX_DMA_CHANNEL     DMA1_Channel2
#define RX_DMA_IRQn        DMA1_Channel2_3_IRQn
#define RX_DMA_IRQHandler  DMA1_Channel2_3_IRQHandler
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t  rx_ring[RX_RING_SIZE];          /* DMA 循环往里写的环形缓冲 */
static uint16_t rx_read_pos = 0U;               /* 已经解析到环形缓冲的哪个位置 */
static DMA_HandleTypeDef hdma_usart1_rx_sw;     /* 串口接收 DMA 句柄 */

/* 帧解析状态机：找帧头 -> 收 data -> 等帧尾 */
#define RX_ST_HEAD  0U
#define RX_ST_DATA  1U
#define RX_ST_TAIL  2U
static uint8_t rx_st   = RX_ST_HEAD;
static uint8_t rx_data = 0U;

/* PB8 接收指示用 */
static uint8_t  rx_active  = 0U;                /* 1 = 最近收到过数据 */
static uint32_t rx_tick    = 0U;                /* 最近一次收到数据的时刻 */
static uint32_t blink_tick = 0U;                /* 最近一次翻转 PB8 的时刻 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*==============================================================================
 *                          从机功能代码
 *============================================================================*/

/* 数一个字节里有几个二进制 1 */
static uint8_t bit_count(uint8_t v)
{
  uint8_t cnt = 0;

  while (v != 0u)
  {
    cnt += (uint8_t)(v & 0x01u);
    v >>= 1;
  }
  return cnt;
}

/* PA6 配成 TIM3_CH1 PWM 输出并启动 */
static void out_pwm_init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();

  /* PA6 -> AF1 (TIM3_CH1) */
  gpio.Pin       = OUT_GPIO_PIN;
  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = GPIO_NOPULL;
  gpio.Speed     = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = GPIO_AF1_TIM3;
  HAL_GPIO_Init(OUT_GPIO_PORT, &gpio);

  /* TIM3 寄存器配置：64MHz/(63+1)=1MHz，1MHz/(999+1)=1kHz */
  TIM3->CR1   = 0U;                              /* 先关掉定时器再配置 */
  TIM3->PSC   = PWM_PRESCALER;
  TIM3->ARR   = PWM_PERIOD;
  TIM3->CCR1  = 0U;                              /* 初始占空比 0% */
  TIM3->CCMR1 = (0x6U << TIM_CCMR1_OC1M_Pos);    /* OC1M = 110：PWM 模式 1（不使能 CCR 预装载） */
  TIM3->CCER  = TIM_CCER_CC1E;                   /* CC1P = 0 高电平有效；CC1E = 1 使能输出 */
  TIM3->EGR   = TIM_EGR_UG;                      /* 产生一次更新事件，让 PSC/ARR 立即生效 */
  TIM3->SR    = 0U;                              /* 清掉更新标志 */
  TIM3->CR1   = TIM_CR1_CEN;                     /* 启动计数 */
}

/* 按 rx_data 里 1 的个数设置占空比：每多 1 个 1 加 10%，最高 100%
   0 个 -> 0%   1 个 -> 10%   2 个 -> 20%  ...  5 个 -> 50%  ...  10 个及以上 -> 100% */
static void out_set_duty_by_ones(uint8_t ones)
{
  uint32_t full = (uint32_t)TIM3->ARR + 1U;                       /* ARR+1 = 1000，对应 100% */
  uint32_t ccr  = (full * (uint32_t)ones * DUTY_PER_ONE) / 100U;

  if (ccr > full)                        /* 超过 10 个 1 就封顶 100% */
  {
    ccr = full;
  }
  TIM3->CCR1 = ccr;                      /* 没使能预装载，写入立即生效 */
}

/* 串口接收 DMA：外设(USART1->RDR) -> 内存(rx_ring)，循环模式，一直收不停 */
static void uart1_rx_dma_init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  hdma_usart1_rx_sw.Instance                 = RX_DMA_CHANNEL;
  hdma_usart1_rx_sw.Init.Request             = DMA_REQUEST_USART1_RX;
  hdma_usart1_rx_sw.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_usart1_rx_sw.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_usart1_rx_sw.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_usart1_rx_sw.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart1_rx_sw.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_usart1_rx_sw.Init.Mode                = DMA_CIRCULAR;     /* 循环搬运：满了自动回到开头 */
  hdma_usart1_rx_sw.Init.Priority            = DMA_PRIORITY_HIGH;
  if (HAL_DMA_Init(&hdma_usart1_rx_sw) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx_sw);

  /* DMA1_Channel2 和 Channel3 共用一个中断向量 */
  HAL_NVIC_SetPriority(RX_DMA_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(RX_DMA_IRQn);
}

/* DMA 中断（本方案靠主循环查计数取数据，其实用不到，留着以防以后改用中断） */
void RX_DMA_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart1_rx_sw);
}

/* 挂上接收：只挂这一次，之后 DMA 一直在环形缓冲里收 */
static void rx_start(void)
{
  if (HAL_UART_Receive_DMA(&huart1, rx_ring, RX_RING_SIZE) != HAL_OK)
  {
    Error_Handler();
  }

  /* 不需要 DMA 中断：把中断源关掉，主循环自己查 DMA 剩余计数取数据 */
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx_sw, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);
}

/* 把 DMA 新收进来的字节喂给状态机（在主循环里频繁调用） */
static void rx_poll(void)
{
  /* DMA 剩余计数 = 还没搬的字节数，所以“环形大小 - 剩余”= 已经收到的字节位置 */
  uint16_t pos = (uint16_t)(RX_RING_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx));

  while (rx_read_pos != pos)
  {
    uint8_t b = rx_ring[rx_read_pos];

    rx_read_pos++;
    if (rx_read_pos >= RX_RING_SIZE)
    {
      rx_read_pos = 0U;
    }

    /* 串口只要收到字节就算“有接收到信息”（PB8 指示用；想只在合法帧时闪，把这两行挪到帧尾判断里） */
    rx_tick   = HAL_GetTick();
    rx_active = 1U;

    switch (rx_st)
    {
      case RX_ST_HEAD:                        /* 找帧头：不是 0xCC 就丢掉，自动重新对齐 */
        if (b == FRAME_HEAD)
        {
          rx_st = RX_ST_DATA;
        }
        break;

      case RX_ST_DATA:                        /* 收 data */
        rx_data = b;
        rx_st   = RX_ST_TAIL;
        break;

      default:                                /* 等帧尾 */
        if (b == FRAME_TAIL)                  /* 完整有效帧 CC data EE */
        {
          /* rx_data 里每有 1 个二进制 1，占空比 +10% */
          out_set_duty_by_ones(bit_count(rx_data));
          rx_st = RX_ST_HEAD;
        }
        else if (b == FRAME_HEAD)             /* 帧尾不对，但这个字节可能就是下一帧的帧头 */
        {
          rx_st = RX_ST_DATA;
        }
        else
        {
          rx_st = RX_ST_HEAD;                 /* 彻底重新找帧头 */
        }
        break;
    }
  }
}
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
  out_pwm_init();           /* PA6 配成 TIM3_CH1 PWM 输出（1kHz，初始 0%） */
  uart1_rx_dma_init();      /* 自己配一套串口接收 DMA（DMA1_Channel2，循环模式） */
  rx_start();               /* 只挂这一次，之后一直收 */

  /* 上电自检：PB8 快闪 3 下（板载蓝灯低电平点亮），看到这 3 下就说明新固件在跑 */
  for (uint8_t i = 0U; i < 3U; i++)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);   /* 亮 */
    HAL_Delay(100U);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);     /* 灭 */
    HAL_Delay(100U);
  }
  blink_tick = HAL_GetTick();   /* 从当前时刻开始算 200ms */
/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 把 DMA 收到的字节喂给状态机：自动找 0xCC 帧头、0xEE 帧尾 */
    rx_poll();

    /* PB8 指示：还在收数据 -> 每 200ms 翻转一次；收不到了 -> 停住，保持当前电平不变 */
    if ((rx_active != 0U) && ((HAL_GetTick() - rx_tick) < LED_RX_TIMEOUT_MS))
    {
      if ((HAL_GetTick() - blink_tick) >= LED_BLINK_MS)
      {
        blink_tick = HAL_GetTick();
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
      }
    }
    else
    {
      rx_active = 0U;       /* 超时没数据：不再翻转，PB8 就停在当前电平 */
    }
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
