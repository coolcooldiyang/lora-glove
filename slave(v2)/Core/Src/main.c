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
/* ===================== 从机：串口 DMA 接收 + 帧解析 + GPIO 输出 =====================
   主机每 600ms 发一帧：  CC      data      EE
                         帧头     数据      帧尾      （固定 3 字节）
   主机只发 CH4/CH5：bit3 = CH4，bit4 = CH5

   规则：CH4 和 CH5 都是 1  ->  PA6 输出"亮" + 板载蓝灯 PB8 亮
         否则               ->  PA6 输出"灭" + PB8 灭

   接收方式：环形缓冲 + 字节流状态机（不是"一次收固定 3 字节"）
     收到 0xCC 才当帧头、收到 0xEE 才认帧尾，中间夹进来的任何杂字节都会被丢掉，
     所以不管主机比从机晚开机多久、中间有没有打印调试信息，都能自动重新对齐。
   ===================================================================== */
#define RX_RING_SIZE    32u     /* 环形接收缓冲大小（字节） */
#define FRAME_HEAD      0xCCu   /* 帧头 */
#define FRAME_TAIL      0xEEu   /* 帧尾 */

/* ---------- 输出脚 PA6（板子丝印 CH5 网络）：普通 GPIO 输出 ----------
   规则：主机发来的 CH4 和 CH5 都是 1 -> 输出"亮"电平；否则 -> 输出"灭"电平
   默认 高电平 = 亮（接继电器模块/LED 串电阻到 GND 这类负载）；
   如果你的灯是低电平点亮，把 OUT_ON_LEVEL / OUT_OFF_LEVEL 两个宏对调即可 */
#define OUT_GPIO_PORT   GPIOA
#define OUT_GPIO_PIN    GPIO_PIN_6
#define OUT_ON_LEVEL    GPIO_PIN_SET
#define OUT_OFF_LEVEL   GPIO_PIN_RESET

/* 板载蓝灯 PB8：和 PA6 同一个条件（CH4 和 CH5 都为 1 -> 亮）
   板载蓝灯是低电平点亮（VCC3V3→LED→1K→PB8） */
#define LED_GPIO_PORT   GPIOB
#define LED_GPIO_PIN    GPIO_PIN_8
#define LED_ON_LEVEL    GPIO_PIN_RESET
#define LED_OFF_LEVEL   GPIO_PIN_SET

/* 主机只发 CH4/CH5：bit3 = CH4，bit4 = CH5（见主机 main.c 里 bits 的拼法） */
#define CH4_MASK        0x08u   /* bit3 */
#define CH5_MASK        0x10u   /* bit4 */
#define CH45_MASK       (CH4_MASK | CH5_MASK)

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
/* 板载蓝灯：on != 0 点亮，否则熄灭 */
static void out_led(uint8_t on)
{
  HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, (on != 0u) ? LED_ON_LEVEL : LED_OFF_LEVEL);
}

/* PA6 配成普通推挽输出（不用 PWM 了），上电先输出"灭" */
static void out_gpio_init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  gpio.Pin   = OUT_GPIO_PIN;              /* PA6 */
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;       /* 推挽输出 */
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OUT_GPIO_PORT, &gpio);

  HAL_GPIO_WritePin(OUT_GPIO_PORT, OUT_GPIO_PIN, OUT_OFF_LEVEL);   /* 上电默认"灭" */
}

/* PA6 输出开关：on != 0 -> 亮电平，否则 -> 灭电平 */
static void out_write(uint8_t on)
{
  HAL_GPIO_WritePin(OUT_GPIO_PORT, OUT_GPIO_PIN, (on != 0u) ? OUT_ON_LEVEL : OUT_OFF_LEVEL);
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

    /* 串口收到字节就不再需要额外处理（灯的亮灭由下面的合法帧控制） */

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
          /* CH4 和 CH5 都是 1 -> 输出"亮"；否则 -> 输出"灭"（bit3=CH4，bit4=CH5）
             PA6（板子丝印 CH5）和板载蓝灯 PB8 是同一个条件 */
          if ((rx_data & CH45_MASK) == CH45_MASK)
          {
            out_write(1U);
            out_led(1U);
          }
          else
          {
            out_write(0U);
            out_led(0U);
          }

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
  out_gpio_init();          /* PA6 配成普通推挽输出（上电"灭"） */
  uart1_rx_dma_init();      /* 自己配一套串口接收 DMA（DMA1_Channel2，循环模式） */
  rx_start();               /* 只挂这一次，之后一直收 */

  /* 上电自检：PB8 快闪 3 下，看到这 3 下就说明新固件在跑；闪完停在"灭" */
  for (uint8_t i = 0U; i < 3U; i++)
  {
    out_led(1U);
    HAL_Delay(100U);
    out_led(0U);
    HAL_Delay(100U);
  }
  out_led(0U);              /* 上电默认灭，之后由主机的 CH4/CH5 决定亮灭 */
/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 把 DMA 收到的字节喂给状态机：自动找 0xCC 帧头、0xEE 帧尾；
       收到合法帧就更新 PA6 的占空比和 PB8 的亮灭（CH4 和 CH5 都为 1 才亮） */
    rx_poll();
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
