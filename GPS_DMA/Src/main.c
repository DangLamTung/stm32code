/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gps.h"
#include "nokia5110.h"
#include "graphic.h"
#include "mpu6050.h"
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define DMA_RX_BUFFER_SIZE          64
uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];

#define UART_BUFFER_SIZE            256
uint8_t UART_Buffer[UART_BUFFER_SIZE];

char Rx_indx,Rx_indx1, Rx_data[2], Rx_Buffer[100], location[100],Rx_data1[1], Rx_Buffer1[100],Transfer_cplt;
char* c = "OK";
uint8_t start;
char test[100];
char delim[] = ",";

char * pch;
GNRMC gps;
char buffer[10];
char temp_[20];
char time[8];
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	  {
	      if (__HAL_TIM_GET_IT_SOURCE(&htim1,TIM_IT_UPDATE)) {
	    	  LCD_clrScr();
	    	  LCD_print("Time:", 0, 0);
	    	  LCD_print(time, 30, 0);

	    	  LCD_print("Lat:", 0, 1);
	    	  itoa(gps.Lat_Deg , buffer, 10);
	    	  LCD_print(buffer, 20, 1);
	    	  LCD_print(":", 35, 1);
	    	  ftoa(gps.Lat_Minute , buffer, 5);
	    	  LCD_print(buffer, 36, 1);

	    	  LCD_print("Lon:", 0, 2);
	    	  itoa(gps.Lon_Deg , buffer, 10);
	    	  LCD_print(buffer, 20, 2);
	    	  LCD_print(":", 38, 2);
	    	  ftoa(gps.Lon_Minute , buffer, 5);
	    	  LCD_print(buffer, 40, 2);

	    	  LCD_print("Speed:", 0, 3);
	    	  ftoa(gps.Speed , buffer, 5);
	    	  LCD_print(buffer, 36, 3);
	          //LCD_print(location, 0, 0);
	          ftoa(yaw , buffer, 2);
	          LCD_print("Heading:", 0, 5);
	          LCD_print(buffer, 48, 5);
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, Rx_data, 1);
 //HAL_UART_Receive_IT(&huart2, Rx_data1,1);

  LCD_setRST(GPIOB, GPIO_PIN_10);
  LCD_setCE(GPIOB, GPIO_PIN_1);
  LCD_setDC(GPIOB, GPIO_PIN_0);
  LCD_setDIN(GPIOA, GPIO_PIN_7);
  LCD_setCLK(GPIOA, GPIO_PIN_5);
  LCD_init();
  //LCD_clrScr();
  HAL_Delay(1000);

  LCD_invert(0);
  //LCD_putChar('a');
  LCD_print("GPS", 0, 4);
  HAL_Delay(1000);
  LCD_clrScr();
  LCD_goXY(0, 0);
  LCD_printBuffer(mari);
  HAL_Delay(1000);
  HAL_TIM_Base_Start_IT(&htim1);
  init_MPU();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //LCD_clrScr();
//	  LCD_goXY(0, 0);
//	  LCD_putChar('a');

      process_MPU();
//	  HAL_Delay(10);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 719;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void substring(char s[], char sub[], int p, int l) {
   int c = 0;

   while (c < l) {
      sub[c] = s[p+c-1];
      c++;
   }
   sub[c] = '\0';
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t i;
    if (huart->Instance == USART1)  //current UART
        {
        if (Rx_indx==0) {for (i=0;i<100;i++) Rx_Buffer[i]=0;}   //clear Rx_Buffer before receiving new data

        if (Rx_data[0]!=10) //if received data different from ascii 13 (enter)
            {
            Rx_Buffer[Rx_indx++]=Rx_data[0];    //add data to Rx_Buffer
            }
        else            //if received data = 13
            {
        	//Rx_Buffer[Rx_indx++] = 13;
            Rx_indx=0;
            char * t = "\n";
            Transfer_cplt=1;//transfer complete, data is ready to read
            if(strstr (Rx_Buffer,"GNRMC")!= NULL ){

            char * n = " ";
            //HAL_UART_Transmit(&huart2, (uint8_t *) Rx_Buffer, sizeof(Rx_Buffer),1000);
            strcpy(location,Rx_Buffer);
            pch = strtok (Rx_Buffer,",");

            int i = 0;

            while (pch != NULL)
              {
            	i++;
            	char tem[2];
            	char deg[8];
            	strcpy(temp_, pch);
            	switch(i){
            	       case 2:
                               for(int j= 0; j<8;j++){
                            	   time[j] = NULL;
                               }
            	    	       substring(temp_, tem, 1, 2);
            	               gps.UTC_Hour = atoi(tem) + 7;
            	               itoa(gps.UTC_Hour,tem,10);
            	               strcat(time, tem);
            	               strcat(time, ":");
            	               substring(temp_, tem, 3, 2);
            	               strcat(time, tem);
            	               strcat(time, ":");
            	               gps.UTC_Min = atoi(tem);
            	               substring(temp_, tem, 5, 2);
            	               strcat(time, tem);
            	               gps.UTC_Sec = atoi(tem);
            	       break;
            	       case 3:
            	    	       gps.Pos = temp_[0];
            	       break;
            	       case 4:
            	    	       substring(temp_, tem, 1, 2);
            	               gps.Lat_Deg = atoi(tem);
            	               substring(temp_, deg, 3, 8);
            	               gps.Lat_Minute = strtod(deg,NULL);

            	       break;
            	       case 5:
            	               gps.Lat_Dir = temp_[0];
            	               break;
            	       case 6:
            	               substring(temp_, tem, 1, 3);
            	               gps.Lon_Deg = atoi(tem);
            	               substring(temp_, deg, 4, 8);
            	               gps.Lon_Minute = strtod(deg,NULL);

            	              break;
            	       case 7:
            	                gps.Lon_Dir = temp_[0];
            	              break;
            	       case 8:
            	                gps.Speed = strtod(temp_,NULL) * 0.514444856;
            	              break;

            	}
            	//CDC_Transmit_FS(temp, strlen(temp));
            	HAL_UART_Transmit(&huart2,&temp_, strlen(temp_),1000);
            	//CDC_Transmit_FS(n, strlen(n));
            	HAL_UART_Transmit(&huart2, (uint8_t *) n, sizeof(n),1000);
                pch = strtok (NULL, ",");



              }
//
//            			sscanf(Rx_Buffer,"$GNRMC,%2hhd%2hhd%2hhd.%3hd,%f,%c,%f,%c,%f,%f,%d,%f,%c,*%c\r\n",
//            					gps.UTC_Hour,gps.UTC_Min,gps.UTC_Sec,gps.UTC_MicroSec,gps.Lat,gps.Lat_Dir,
//								gps.Lon,gps.Lon_Dir,gps.Speed,gps.Track,gps.date,gps.mag_v,gps.var_dir,gps.mode_ind
//								);
            HAL_UART_Transmit(&huart2, (uint8_t *) t, sizeof(t),1000);

            }

            }

        HAL_UART_Receive_IT(&huart1, (uint8_t *) Rx_data, 1);   //activate UART receive interrupt every time
        }
    if (huart->Instance == USART2)  //current UART
            {
            if (Rx_indx1==0) {for (i=0;i<100;i++) Rx_Buffer1[i]=0;
            LCD_clrScr();}   //clear Rx_Buffer before receiving new data
            if(Rx_data1[0] == '0'){
            	start = 1;

            }
            if (Rx_indx1 < 100 && start == 1 ) //if received data different from ascii 13 (enter)
                {
                Rx_Buffer1[Rx_indx1++]=Rx_data1[0];    //add data to Rx_Buffer
                }
            else            //if received data = 13
                {
            	//Rx_Buffer[Rx_indx++] = 13;
                Rx_indx1=0;
                start = 0;
                char * t = "\n";
                Transfer_cplt=1;//transfer complete, data is ready to read
          	    //LCD_print(c, 0, 0);
          	  //HAL_Delay(4000);
                }

            HAL_UART_Receive_IT(&huart2, (uint8_t *) Rx_data1, 1);   //activate UART receive interrupt every time
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
