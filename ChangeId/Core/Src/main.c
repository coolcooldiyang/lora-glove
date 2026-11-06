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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*==============================================================================
 *                     HC-12 无线模块参数配置工具
 *
 *  作用：通过串口给 HC-12 发 AT 指令，把模块的
 *          · 串口波特率 改成 4800   （AT+B4800）
 *          · 通信频道   改成 C002   （AT+C002）
 *
 *  接线（板子上 USART1 已经连好了：MCU PA9(TX) -> 模块 RXD，MCU PA10(RX) <- 模块 TXD）
 *      ★ 模块的第 5 脚 SET 必须拉低才能进 AT 指令模式，二选一：
 *        ①（推荐）把 SET 接到板子的 PB4 —— 程序会自动拉低进 AT 模式、配完再拉高退出透传；
 *        ② 把 SET 直接接 GND —— 程序按 9600 通信（上电时 SET 为低，AT 模式固定 9600），
 *           但配完必须自己把这根线拆掉，模块才会回到透传模式。
 *
 *  指示灯 PB8（板载蓝灯，低电平点亮）：
 *        快闪 3 下        = 上电，准备开始配置
 *        常亮             = 配置成功
 *        熄灭             = 配置失败（没收到模块回应），按复位键可重试
 *
 *  调试：不确定成功与否时，可以在 Keil 里看这几个变量：
 *        hc_at_baud  —— 自动识别到的模块 AT 波特率
 *        hc_rx       —— 模块最后一条回包（比如 "OK+C002"）
 *
 *  注意：AT+Bxxxx 改的波特率是"下次上电生效"（掉电保存），所以配完要给模块
 *        重新上电一次；频道则是退出 AT 模式后立即生效。
 *============================================================================*/
#define HC_UART              huart1
#define HC_USE_SET_GPIO      1              /* 1 = 用 PB4 控制模块 SET；0 = SET 自己接 GND */
#define HC_SET_PORT          GPIOB
#define HC_SET_PIN           GPIO_PIN_4

#define HC_BAUD_TARGET       4800u          /* 目标波特率 */
#define HC_CHANNEL_TARGET    "002"          /* 目标频道 -> C002 */

#define LED_PORT             GPIOB
#define LED_PIN              GPIO_PIN_8
#define LED_ON_LEVEL         GPIO_PIN_RESET /* 板载蓝灯低电平点亮 */
#define LED_OFF_LEVEL        GPIO_PIN_SET
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static char     hc_rx[64];          /* 模块回包缓冲 */
static uint16_t hc_rx_len = 0u;     /* 回包长度 */
static uint8_t  hc_ok = 0u;         /* 1 = 配置成功 */
static volatile uint32_t hc_at_baud = 0u;  /* 自动识别到的 AT 波特率（调试时可在 Keil 里看） */

/* AT 模式的波特率候选：上电时 SET 为低 -> 固定 9600；运行中拉低 SET -> 模块当前波特率，
   所以这里把可能的值都试一遍 */
static const uint32_t hc_baud_list[] =
{
  9600u, 4800u, 115200u, 19200u, 57600u, 38400u, 2400u, 1200u
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*==============================================================================
 *                              HC-12 配置工具
 *============================================================================*/

/* ---------------- 指示灯 ---------------- */
static void led_on(void)  { HAL_GPIO_WritePin(LED_PORT, LED_PIN, LED_ON_LEVEL); }
static void led_off(void) { HAL_GPIO_WritePin(LED_PORT, LED_PIN, LED_OFF_LEVEL); }

/* 快闪 n 下，表示"正在干活" */
static void led_blink_fast(uint8_t times)
{
  for (uint8_t i = 0u; i < times; i++)
  {
    led_on();
    HAL_Delay(100u);
    led_off();
    HAL_Delay(100u);
  }
}

/* ---------------- 串口 ---------------- */
static void hc_uart_set_baud(uint32_t baud)
{
  HC_UART.Init.BaudRate = baud;
  if (HAL_UART_Init(&HC_UART) != HAL_OK)
  {
    Error_Handler();
  }
}

static void hc_rx_clear(void)
{
  hc_rx_len = 0u;
  memset(hc_rx, 0, sizeof(hc_rx));
}

/* 收模块回包：一直收到超时（每收到一个字节就把超时重新计时） */
static void hc_read_reply(uint32_t timeout_ms)
{
  uint8_t  b;
  uint32_t t0 = HAL_GetTick();

  while ((HAL_GetTick() - t0) < timeout_ms)
  {
    if (HAL_UART_Receive(&HC_UART, &b, 1u, 2u) == HAL_OK)
    {
      if (hc_rx_len < (uint16_t)(sizeof(hc_rx) - 1u))
      {
        hc_rx[hc_rx_len++] = (char)b;
      }
      t0 = HAL_GetTick();          /* 收到字节，续期 */
    }
  }
}

/* 发一条 AT 指令并检查回包：回包里要有 "OK"，expect 不为 NULL 时还要包含它 */
static uint8_t hc_send_cmd(const char *cmd, const char *expect, uint32_t timeout_ms)
{
  hc_rx_clear();
  (void)HAL_UART_Transmit(&HC_UART, (uint8_t *)cmd, (uint16_t)strlen(cmd), 200u);
  hc_read_reply(timeout_ms);

  if (strstr(hc_rx, "OK") == NULL)
  {
    return 0u;                     /* 模块没理我们 */
  }
  if ((expect != NULL) && (strstr(hc_rx, expect) == NULL))
  {
    return 0u;                     /* 回的 OK 不对 */
  }
  return 1u;
}

/* 自动识别模块当前的 AT 波特率：逐个波特率发 "AT"，谁能回 "OK" 就用谁 */
static uint8_t hc_find_baud(void)
{
  for (uint8_t i = 0u; i < (uint8_t)(sizeof(hc_baud_list) / sizeof(hc_baud_list[0])); i++)
  {
    hc_uart_set_baud(hc_baud_list[i]);
    HAL_Delay(20u);

    /* 有的模块上电后第一条指令会被吞掉，所以给两次机会 */
    if (hc_send_cmd("AT", "OK", 150u) != 0u)
    {
      hc_at_baud = hc_baud_list[i];
      return 1u;
    }
    if (hc_send_cmd("AT", "OK", 150u) != 0u)
    {
      hc_at_baud = hc_baud_list[i];
      return 1u;
    }
  }
  return 0u;
}

/* 配置流程：进 AT 模式 -> 找波特率 -> 改波特率/频道 -> 退出 AT 模式 */
static void hc_configure(void)
{
  char    cmd[24];
  uint8_t ok = 1u;

  /* 1. 把模块 SET 拉低，进入 AT 指令模式（手册要求至少等 40ms） */
#if (HC_USE_SET_GPIO != 0)
  {
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin   = HC_SET_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(HC_SET_PORT, &gpio);

    HAL_GPIO_WritePin(HC_SET_PORT, HC_SET_PIN, GPIO_PIN_RESET);   /* SET = 低 -> AT 模式 */
  }
#endif
  HAL_Delay(100u);

  /* 2. 找模块当前的 AT 波特率 */
  if (hc_find_baud() == 0u)
  {
    ok = 0u;                       /* 一个波特率都不通：多半是 SET 没拉低 / 接线不对 */
  }

  /* 3. 改串口波特率：AT+B4800 -> 回 OK+B4800 */
  if ((ok != 0u) && (hc_send_cmd("AT+B" "4800", "OK+B", 300u) == 0u))
  {
    ok = 0u;
  }

  /* 4. 改通信频道：AT+C002 -> 回 OK+C002 */
  if (ok != 0u)
  {
    (void)snprintf(cmd, sizeof(cmd), "AT+C%s", HC_CHANNEL_TARGET);
    if (hc_send_cmd(cmd, "OK+C", 300u) == 0u)
    {
      ok = 0u;
    }
  }

  /* 5. 读回参数确认：AT+RX 会回 "OK+FU3 OK+B4800 OK+C002 OK+RP:+20dBm" 这样的串 */
  if (ok != 0u)
  {
    (void)hc_send_cmd("AT+RX", "OK+", 300u);   /* hc_rx 里会留下完整回包，调试时可以看 */
  }

  /* 6. 退出 AT 指令模式：SET 拉高，模块 80ms 后回到透传模式 */
#if (HC_USE_SET_GPIO != 0)
  HAL_GPIO_WritePin(HC_SET_PORT, HC_SET_PIN, GPIO_PIN_SET);
#endif
  HAL_Delay(100u);

  hc_ok = ok;
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
  /* -------- HC-12 参数配置：波特率 4800、频道 C002 -------- */
  led_blink_fast(3u);        /* 快闪 3 下：准备开始 */
  HAL_Delay(300u);           /* 等模块上电稳定 */
  hc_configure();            /* 进 AT 模式 -> 找波特率 -> 改参数 -> 退出 */
/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 配置结果：成功常亮；失败熄灭（按复位键重试） */
    if (hc_ok != 0u)
    {
      led_on();
    }
    else
    {
      led_off();
    }
    HAL_Delay(200u);
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
