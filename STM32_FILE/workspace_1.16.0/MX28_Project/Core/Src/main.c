/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 256
#define TX_BUFFER_SIZE 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t PCrxBuffer[RX_BUFFER_SIZE];
uint8_t PCtxBuffer[TX_BUFFER_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void setMX28GoalPosition(uint8_t id, uint32_t position);
uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);
void delay_ms(uint32_t ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// CRC Í≥ÑÏÇ∞ ?ï®?àò (Dynamixel Protocol 2.0?óê ?î∞Î•? CRC-16 Í≥ÑÏÇ∞)
uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size) {
    uint16_t i, j;
    uint16_t crc_table[256] = { /* CRC ?Öå?ù¥Î∏? ?Éù?ûµ (?†ÑÏ≤? 256Í∞? ?ï≠Î™?) */
        0x0000, 0x8005, 0x800F, 0x000A, /* ?ÇòÎ®∏Ï? CRC ?Öå?ù¥Î∏? ?ï≠Î™? Ï∂îÍ? */
    };

    for (j = 0; j < data_blk_size; j++) {
        i = ((uint16_t)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }
    return crc_accum;
}

void setMX28GoalPosition(uint8_t id, uint32_t position) {
    uint8_t txPacket[16];
    uint16_t crc;

    // Header
    txPacket[0] = 0xFF;
    txPacket[1] = 0xFF;
    txPacket[2] = 0xFD;
    txPacket[3] = 0x00;

    // ID
    txPacket[4] = id;

    // Length (4 for goal position + 3 for instruction + 2 for CRC)
    txPacket[5] = 0x09;
    txPacket[6] = 0x00;

    // Instruction (Write)
    txPacket[7] = 0x03;

    // Parameter 1 (Goal Position Address: 116)
    txPacket[8] = 0x74;  // Address LSB (116)
    txPacket[9] = 0x00;  // Address MSB

    // Parameter 2-5 (Goal Position Value)
    txPacket[10] = position & 0xFF;
    txPacket[11] = (position >> 8) & 0xFF;
    txPacket[12] = (position >> 16) & 0xFF;
    txPacket[13] = (position >> 24) & 0xFF;

    // Calculate CRC
    crc = update_crc(0, txPacket, 14);
    txPacket[14] = crc & 0xFF;
    txPacket[15] = (crc >> 8) & 0xFF;

    // Send packet via UART
    HAL_UART_Transmit(&huart3, txPacket, 16, HAL_MAX_DELAY);
}

void delay_ms(uint32_t ms) {
    HAL_Delay(ms);  // HAL ?ùº?ù¥Î∏åÎü¨Î¶¨Ïùò Delay ?ï®?àò
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1, (uint32_t)PCrxBuffer);
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_1, (uint32_t)&USART3->DR);
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, RX_BUFFER_SIZE);
  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
  LL_USART_EnableDMAReq_RX(USART3);
  LL_USART_EnableIT_IDLE(USART3);
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
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  // MX28 Î™®ÌÑ∞?ùò Î™©Ìëú ?úÑÏπ? ?Ñ§?†ï (0~4095 Î≤îÏúÑ, 360?èÑ?óê ?ï¥?ãπ)
  uint32_t currentPosition = 2048;  // Ï§ëÍ∞Ñ ?úÑÏπ? (2048 = 180?èÑ)
  uint32_t positionOffset = 227;    // 20?èÑ?óê ?ï¥?ãπ?ïò?äî ?úÑÏπ? Î≥??ôî?üâ

  // Î™®ÌÑ∞ Ï¥àÍ∏∞ ?úÑÏπ? ?Ñ§?†ï (Ï§ëÍ∞Ñ Í∞?)
  setMX28GoalPosition(1, currentPosition);
  delay_ms(1000);  // 1Ï¥? ??Í∏?

  // ?ãúÍ≥? Î∞©Ìñ•?úºÎ°? 20?èÑ ?öå?†Ñ
  setMX28GoalPosition(1, currentPosition + positionOffset);
  delay_ms(1000);  // 1Ï¥? ??Í∏?

  // Î∞òÏãúÍ≥? Î∞©Ìñ•?úºÎ°? 20?èÑ ?öå?†Ñ (?õê?ûò ?úÑÏπòÎ°ú Î≥µÍ?)
  setMX28GoalPosition(1, currentPosition - positionOffset);
  delay_ms(1000);  // 1Ï¥? ??Í∏?

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

#ifdef  USE_FULL_ASSERT
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
