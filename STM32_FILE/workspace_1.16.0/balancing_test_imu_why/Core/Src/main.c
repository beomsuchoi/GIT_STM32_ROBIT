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

#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
int desired_velocity = 0;
int current_velocity = 0;
int actual_velocity;
int rotary_sensor_value;
int iteration = 0;
int rotation_data[2];
int desired_rpm;
int actual_rpm;
float tilt_angle_filtered = 0;
const float beta = 0.98;
float tilt_angle_estimated = 0;
float Process_Noise_Pos = 0.001;
float Process_Noise_Vel = 0.003;
float Measurement_Noise = 0.5;

float rate_bias = 0;
float Error_Cov[2][2] = {{1, 0}, {0, 1}};
float Accel_X, Accel_Y, Accel_Z, Gyro_X, Gyro_Y, Gyro_Z;
float tilt_angle;
float processed_angle;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMU_ADDR 0xD0
#define IMU_SAMPLE_RATE_REG 0x19
#define IMU_CONFIG_REG 0x1A
#define IMU_GYRO_CONFIG_REG 0x1B
#define IMU_ACCEL_CONFIG_REG 0x1C
#define IMU_WHO_AM_I_REG 0x75
#define IMU_PWR_MGMT_1_REG 0x6B
#define IMU_ACCEL_XOUT_H_REG 0x3B
#define IMU_GYRO_XOUT_H_REG 0x43
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t imu_check;
int16_t Raw_AccelX = 0;
int16_t Raw_AccelY = 0;
int16_t Raw_AccelZ = 0;
int16_t Raw_GyroX = 0;
int16_t Raw_GyroY = 0;
int16_t Raw_GyroZ = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void IMU_Initialize(void);
void IMU_ReadAccelerometer(void);
void IMU_ReadGyroscope(void);
float ComputeTiltAngle(float Gx, float Gz);

void SystemClock_Config(void);
void Error_Handler(void);
void UpdateTiltAngle(void);
void TiltEstimator(float new_angle, float new_rate);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void TiltEstimator(float new_angle, float new_rate)
{
    float dt = 0.01;
    tilt_angle_estimated += (new_rate - rate_bias) * dt;

    Error_Cov[0][0] += Process_Noise_Pos;
    Error_Cov[0][1] -= Error_Cov[0][1] * dt;
    Error_Cov[1][0] -= Error_Cov[1][0] * dt;
    Error_Cov[1][1] += Process_Noise_Vel * dt;

    float y = new_angle - tilt_angle_estimated;
    float S = Error_Cov[0][0] + Measurement_Noise;
    float K[2];
    K[0] = Error_Cov[0][0] / S;
    K[1] = Error_Cov[1][0] / S;

    tilt_angle_estimated += K[0] * y;
    rate_bias += K[1] * y;

    float P00_temp = Error_Cov[0][0];
    Error_Cov[0][0] -= K[0] * P00_temp;
    Error_Cov[0][1] -= K[0] * Error_Cov[0][1];
    Error_Cov[1][0] -= K[1] * P00_temp;
    Error_Cov[1][1] -= K[1] * Error_Cov[0][1];
}

void UpdateTiltAngle(void)
{
    tilt_angle += Gyro_X * (1.0 / 1000.0);
    float tilt_angle_accel = ComputeTiltAngle(Accel_X, Accel_Z);
    tilt_angle_filtered = beta * tilt_angle_filtered + (1 - beta) * tilt_angle_accel;
}

void IMU_Initialize(void)
{
  uint8_t imu_check;
  uint8_t ConfigData;

  HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, IMU_WHO_AM_I_REG, 1, &imu_check, 1, 1000);

  if (imu_check == 104)
  {
    ConfigData = 0;
    HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, IMU_PWR_MGMT_1_REG, 1, &ConfigData, 1, 1000);

    ConfigData = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, IMU_SAMPLE_RATE_REG, 1, &ConfigData, 1, 1000);

    ConfigData = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, IMU_ACCEL_CONFIG_REG, 1, &ConfigData, 1, 1000);

    ConfigData = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, IMU_GYRO_CONFIG_REG, 1, &ConfigData, 1, 1000);
  }
}

void IMU_ReadAccelerometer(void)
{
  uint8_t ReceivedData[6];

  HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, IMU_ACCEL_XOUT_H_REG, 1, ReceivedData, 6, 1000);

  Raw_AccelX = (int16_t)(ReceivedData[0] << 8 | ReceivedData[1]);
  Raw_AccelY = (int16_t)(ReceivedData[2] << 8 | ReceivedData[3]);
  Raw_AccelZ = (int16_t)(ReceivedData[4] << 8 | ReceivedData[5]);

  Accel_X = Raw_AccelX / 16384.0;
  Accel_Y = Raw_AccelY / 16384.0;
  Accel_Z = Raw_AccelZ / 16384.0;
}

void IMU_ReadGyroscope(void)
{
  uint8_t ReceivedData[6];

  HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, IMU_GYRO_XOUT_H_REG, 1, ReceivedData, 6, 1000);

  Raw_GyroX = (int16_t)(ReceivedData[0] << 8 | ReceivedData[1]);
  Raw_GyroY = (int16_t)(ReceivedData[2] << 8 | ReceivedData[3]);
  Raw_GyroZ = (int16_t)(ReceivedData[4] << 8 | ReceivedData[5]);

  Gyro_X = Raw_GyroX / 131.0;
  Gyro_Y = Raw_GyroY / 131.0;
  Gyro_Z = Raw_GyroZ / 131.0;
}

float ComputeTiltAngle(float Gx, float Gz)
{
  return atan2(Gx, Gz) * 180 / 3.14159265;
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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim6);

  IMU_Initialize();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    IMU_ReadAccelerometer();
    IMU_ReadGyroscope();

    tilt_angle = ComputeTiltAngle(Gyro_X, Gyro_Z);
    UpdateTiltAngle();
    TiltEstimator(tilt_angle, Gyro_X);

    processed_angle = tilt_angle_filtered;
    rotary_sensor_value = TIM3->CNT;

   if(processed_angle<=0)
        {
           HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, RESET);
           HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, RESET);
           if(processed_angle*10>-800)
           {
              TIM1->CCR1 = -processed_angle*7;
              TIM1->CCR2 = -processed_angle*7;
           }
        }
        else if(processed_angle>0)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, SET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, SET);
            if(processed_angle*10<800)
            {
               TIM1->CCR1 = processed_angle*7;
               TIM1->CCR2 = processed_angle*7;
            }
        }

    if((rotation_data[0] - rotation_data[1])>=0)
    {
       actual_velocity = ((rotation_data[0] - rotation_data[1]) + 3) * 10;
    }
    else if((rotation_data[0] - rotation_data[1])<0)
    {
       actual_velocity = -((rotation_data[0] - rotation_data[1]) + 3) * 10;
    }
  }
  /* USER CODE END WHILE */
}
/* USER CODE END 3 */

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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
