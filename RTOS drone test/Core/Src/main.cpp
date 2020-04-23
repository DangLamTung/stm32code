/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu9250.hpp"
#include "mpu_data_type.hpp"
#include "EKF.h"
#include "MadgwickAHRS.h"
//#include "IMU_EKF.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim3;
//TIM_HandleTypeDef htim4;


/* USER CODE BEGIN PV */


IMU_data data_imu_raw;
IMU_data data_imu_buffer;
EULER_angle drone_state;
EULER_angle magd_state;
EKF ekf;
uint8_t i;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
double x[7] = {1,0,0,0,0,0,0};
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
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

    init_MPU();
    calibration_IMU();

    x[4] = bGx*DEC2RAD;
    x[5] = bGy*DEC2RAD;
    x[6] = bGz*DEC2RAD;

    ekf.loadEKF(x,P,Q, R_full);

    HAL_TIM_Base_Start(&htim4);
//	  data_imu_buffer.Gyro_x = 0.5;
//	  data_imu_buffer.Gyro_y = 0.9;
//	  data_imu_buffer.Gyro_z = 0.8;

//	  gyro_angle_r += data_imu_buffer.Gyro_x*0.01;
//	  gyro_angle_p += data_imu_buffer.Gyro_y*0.01;
//	  gyro_angle_y += data_imu_buffer.Gyro_z*0.01;

//	  gyro_angle_r += (data_imu_buffer.Gyro_x - bGx)*0.01;
//	  gyro_angle_p += (data_imu_buffer.Gyro_y - bGy)*0.01;
//	  gyro_angle_y += (data_imu_buffer.Gyro_z - bGz)*0.01;
//
//	  magd_state.roll = gyro_angle_r;
//	  magd_state.pitch = gyro_angle_p;
//	  magd_state.yaw = gyro_angle_y;

//	  data_imu_buffer.Acc_x =  0;
//	  data_imu_buffer.Acc_y =  0;
//	  data_imu_buffer.Acc_z =  1;
  /* USER CODE END 2 */



  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	//	  gyro_angle_r += data_imu_buffer.Gyro_x*0.01;
	//	  gyro_angle_p += data_imu_buffer.Gyro_y*0.01;
	//	  gyro_angle_y += data_imu_buffer.Gyro_z*0.01;

	//	  gyro_angle_r += (data_imu_buffer.Gyro_x - bGx)*0.01;
	//	  gyro_angle_p += (data_imu_buffer.Gyro_y - bGy)*0.01;
	//	  gyro_angle_y += (data_imu_buffer.Gyro_z - bGz)*0.01;
	//
	//	  magd_state.roll = gyro_angle_r;
	//	  magd_state.pitch = gyro_angle_p;
	//	  magd_state.yaw = gyro_angle_y;
	          data_imu_raw = process_MPU(true);
              int start,end,time;
              start = htim4.Instance->CNT;
//			  ekf.updateEKF(data_imu_raw, 0.01);

			 //
			 //		  data_imu_buffer.Acc_x =  gyro_angle_r;
			 //		  data_imu_buffer.Acc_y =  gyro_angle_p;
			 //		  data_imu_buffer.Acc_z =  gyro_angle_y;
			 	  drone_state = complementary_filter(data_imu_raw, 0.01, 0.99);
			 	 end = htim4.Instance->CNT;
			 	 time = end -start;
//			  data_imu_buffer.Acc_x =  0;
//			  data_imu_buffer.Acc_y +=  0.001;
//			  data_imu_buffer.Acc_z -=  0.001;

//
//		  drone_state = IMU_EKF(x, 0.01,P,Q,R_full,data_imu_buffer);


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
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */


/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 167;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */


/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM3){
		if(i != 0){
			  data_imu_buffer.Gyro_x = 0.5;
			  data_imu_buffer.Gyro_y = 0.9;
			  data_imu_buffer.Gyro_z = 0.8;

		//	  gyro_angle_r += data_imu_buffer.Gyro_x*0.01;
		//	  gyro_angle_p += data_imu_buffer.Gyro_y*0.01;
		//	  gyro_angle_y += data_imu_buffer.Gyro_z*0.01;

		//	  gyro_angle_r += (data_imu_buffer.Gyro_x - bGx)*0.01;
		//	  gyro_angle_p += (data_imu_buffer.Gyro_y - bGy)*0.01;
		//	  gyro_angle_y += (data_imu_buffer.Gyro_z - bGz)*0.01;
		//
		//	  magd_state.roll = gyro_angle_r;
		//	  magd_state.pitch = gyro_angle_p;
		//	  magd_state.yaw = gyro_angle_y;

//			  data_imu_buffer.Acc_x =  0;
//			  data_imu_buffer.Acc_y +=  0.001;
//			  data_imu_buffer.Acc_z -=  0.001;

	//
//		  drone_state = IMU_EKF(x, 0.01,P,Q,R_full,data_imu_buffer);

		}
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
