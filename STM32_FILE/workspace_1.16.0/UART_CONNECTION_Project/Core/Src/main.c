/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private variables ---------------------------------------------------------*/
uint32_t adc1_buffer[5] = { 0, };
uint32_t sum = 0;
uint8_t switch_state[3];
uint8_t start_packet = 0xAA;        //시작 패킷
uint8_t check_sum;
float result = 0;
const float v_ref = 5.0;
const float K = 50.0;
float distance;
uint32_t buffer_index = 0;

/* Function prototypes -------------------------------------------------------*/
void SystemClock_Config(void);

/* Main code -----------------------------------------------------------------*/
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/
    HAL_Init();
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM8_Init();
    MX_USART1_UART_Init();

    /* ADC와 UART DMA 시작 */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buffer, 5);
    HAL_UART_Receive_DMA(&huart1, switch_state, sizeof(switch_state));

    /* Infinite loop */
    while (1)
    {
        // 통신 시작 패킷 송신
        HAL_UART_Transmit(&huart1, &start_packet, 1, HAL_MAX_DELAY);


        sum = 0;
        for (int i = 0; i < 5; i++)
        {
            sum += adc1_buffer[i];
        }
        result = (float)sum / 5.0;


        float tmp = (float)result * v_ref / 4095.0;  // ADC 값을 전압으로 변환
        if (tmp != 0)
        {
            distance = K * (1.0 / tmp);
        }
        else
        {
            distance = 0;
        }


        HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&distance, sizeof(distance));


        if (switch_state[0] == 1)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
        }

        if (switch_state[1] == 1)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
        }

        if (switch_state[2] == 1)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
        }

        HAL_Delay(1000);  //qksqhr 1second
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    // 클럭 설정 코드
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage */
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

    /** Activate the Over-Drive mode */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* Error Handler */
void Error_Handler(void)
{
    while (1)
    {
    }
}
