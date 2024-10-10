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
int target_speed = 0;
int speed = 0;
int current_speed;
int encoder_value;
int count = 0;
int encoder_arr[2];
int target_rpm;
int current_rpm;
float angle_xz_filtered = 0;
const float alpha = 0.98;
float angle_xz_kalman = 0;
float Q_angle = 0.001;
float Q_gyro = 0.003;
float R_angle = 0.5;

float bias = 0;
float P[2][2] = {{1, 0}, {0, 1}};
float Ax, Ay, Az, Gx, Gy, Gz;
float angle_xz;
float filtered_angle;

float Kp = 7;
float Ki = 0.6;
float Kd = 0.6;
float integral = 0;
float prev_error = 0;
float prev_derivative = 0;
float dt = 0.01;
float motor_value;
float a;
float pid_output;
float chg = -3.5;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR 0xD0
#define MPU6050_SMPLRT_DIV_REG 0x19
#define MPU6050_CONFIG_REG 0x1A
#define MPU6050_GYRO_CONFIG_REG 0x1B
#define MPU6050_ACCEL_CONFIG_REG 0x1C
#define MPU6050_WHO_AM_I_REG 0x75
#define MPU6050_PWR_MGMT_1_REG 0x6B
#define MPU6050_ACCEL_XOUT_H_REG 0x3B
#define MPU6050_GYRO_XOUT_H_REG 0x43

#define INTEGRAL_MAX 200
#define INTEGRAL_MIN -200

#define MIN_MOTOR_OUTPUT 80
#define MAX_MOTOR_OUTPUT 450

#define ANGLE_THRESHOLD 0
#define MAX_RESPONSE_ANGLE 45.0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t check;
int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;
int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void MPU6050_Init(void);
void MPU6050_Read_Accel(void);
void MPU6050_Read_Gyro(void);
float Calculate_XZ_Angle(float Gx, float Gz);

void SystemClock_Config(void);
void Error_Handler(void);
void Update_Angle(void);
void Kalman_Filter(float new_angle, float new_rate);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float non_linear_response(float angle)
{
    float abs_angle = fabs(angle);
    float sign = (angle >= 0) ? 1.0f : -1.0f;

    if (abs_angle <= ANGLE_THRESHOLD)
    {
        return sign * (abs_angle * 0.8f);
    }
    else
    {
        float normalized = (abs_angle - ANGLE_THRESHOLD) / (MAX_RESPONSE_ANGLE - ANGLE_THRESHOLD);
        float response = ANGLE_THRESHOLD + (MAX_RESPONSE_ANGLE - ANGLE_THRESHOLD) *
                         (1 - expf(-1.5 * normalized));
        return sign * fminf(response, MAX_RESPONSE_ANGLE);
    }
}

float calculate_pid(float setpoint, float measured_value)
{
    float error = setpoint - measured_value;
    float non_linear_error = non_linear_response(error);

    integral += non_linear_error * dt;
    integral = fmaxf(fminf(integral, INTEGRAL_MAX), INTEGRAL_MIN);

    float derivative = (non_linear_error - prev_error) / dt;
    float filtered_derivative = 0.7f * derivative + 0.3f * prev_derivative;

    float output = Kp * non_linear_error + Ki * integral + Kd * filtered_derivative;

    if (fabs(error) > 0.2f)
    {
        float min_output = MIN_MOTOR_OUTPUT * (fminf(fabs(error), 1.0f) / 1.0f);
        if (fabs(output) < min_output)
        {
            output = (output >= 0) ? min_output : -min_output;
        }
    }

    prev_error = non_linear_error;
    prev_derivative = filtered_derivative;

    return output;  // Return the calculated output
}

void Kalman_Filter(float new_angle, float new_rate)
{
    angle_xz_kalman += (new_rate - bias) * dt;

    P[0][0] += Q_angle;
    P[0][1] -= P[0][1] * dt;
    P[1][0] -= P[1][0] * dt;
    P[1][1] += Q_gyro * dt;

    float y = new_angle - angle_xz_kalman;
    float S = P[0][0] + R_angle;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    angle_xz_kalman += K[0] * y;
    bias += K[1] * y;

    float P00_temp = P[0][0];
    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P[0][1];
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P[0][1];
}

void Update_Angle(void)
{
    angle_xz += Gx * (1.0 / 1000.0);
    float angle_xz_acc = Calculate_XZ_Angle(Ax, Az);
    angle_xz_filtered = alpha * angle_xz_filtered + (1 - alpha) * angle_xz_acc;
}

void MPU6050_Init(void)
{
  uint8_t check;
  uint8_t Data;

  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_WHO_AM_I_REG, 1, &check, 1, 1000);

  if (check == 104)
  {
    Data = 0;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_PWR_MGMT_1_REG, 1, &Data, 1, 1000);

    Data = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_SMPLRT_DIV_REG, 1, &Data, 1, 1000);

    Data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

    Data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_GYRO_CONFIG_REG, 1, &Data, 1, 1000);
  }
}

void MPU6050_Read_Accel(void)
{
  uint8_t Rec_Data[6];

  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

  Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
  Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
  Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

  Ax = Accel_X_RAW / 16384.0;
  Ay = Accel_Y_RAW / 16384.0;
  Az = Accel_Z_RAW / 16384.0;
}

void MPU6050_Read_Gyro(void)
{
  uint8_t Rec_Data[6];

  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

  Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
  Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
  Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

  Gx = Gyro_X_RAW / 131.0;
  Gy = Gyro_Y_RAW / 131.0;
  Gz = Gyro_Z_RAW / 131.0;
}

float Calculate_XZ_Angle(float Gx, float Gz)
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

  MPU6050_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
      MPU6050_Read_Accel();
      MPU6050_Read_Gyro();

      angle_xz = Calculate_XZ_Angle(Gx, Gz);
      Update_Angle();
      Kalman_Filter(angle_xz, Gx);

      filtered_angle = angle_xz_filtered+chg;
      encoder_value = TIM3->CNT;

      float setpoint = 0;
      pid_output = calculate_pid(setpoint, filtered_angle);

      pid_output = fmaxf(fminf(pid_output, MAX_MOTOR_OUTPUT), -MAX_MOTOR_OUTPUT);
      motor_value = fabsf(pid_output);

      if (pid_output >= 0)
      {
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, SET);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, SET);
      }
      else
      {
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, RESET);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, RESET);
      }

      TIM1->CCR1 = motor_value;
      TIM1->CCR2 = motor_value;

      a = filtered_angle * 10;
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
   if(htim->Instance == TIM6)
   {
      encoder_arr[count] = encoder_value;
      count++;
      count = count % 2;
   }
}
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
