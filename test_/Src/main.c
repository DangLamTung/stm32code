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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "graphic.h"
#include <stdlib.h>
#include "nokia5110.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t bird_pos;
uint8_t bar_pos;
uint8_t colision;
uint8_t dead;
uint8_t game;
int score;
volatile uint8_t debouncing;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int lower = 5, upper = 7, count = 1;
void print_number(uint8_t x, uint8_t number){
	 LCD_goXY(x, 1);
	 for(int i = 0; i<5;i++){
		 LCD_goXY(x, i + 1);
	 for(int j = 0; j<25;j++){
		  		  LCD_write(myBitmap[number][j + 25*i],  LCD_DATA);
		}
	 }
}
char buffer [504];
void shift_img(){

	for(int i = 0; i<504; i++)
		buffer[i] = 0x00;
	for(int i = 0 ; i<3;i++ ){
		for(int j = 0; j<20; j++)
		buffer[i*84 +j] = bird[20*i + j  ];
	}
}
void LCD_set_pixel(uint8_t x, uint8_t y, uint8_t pixel){
  if(x >= LCD_WIDTH)
    x = LCD_WIDTH - 1;
  if(y >= LCD_HEIGHT)
    y = LCD_HEIGHT - 1;

  if(pixel != 0){
    buffer[x + (y / 8) * LCD_WIDTH] |= 1 << (y % 8);
  }
  else{
    buffer[x + (y / 8) * LCD_WIDTH] &= ~(1 << (y % 8));
  }
}

void LCD_set_pixel_im(uint8_t x, uint8_t y, uint8_t pixel){
  if(x >= LCD_WIDTH)
    x = LCD_WIDTH - 1;
  if(y >= LCD_HEIGHT)
    y = LCD_HEIGHT - 1;

  if(pixel != 0){
	  buffer[x + (y / 8) * LCD_WIDTH] |= 1 << (y % 8);
	  if(buffer[x + (y / 8) * LCD_WIDTH] == 0){
}
  }
else{

}
}

void delete_bar(uint8_t x, uint8_t y, uint8_t height, uint8_t width,uint8_t gap){

	for(int i = x; i<x+width; i++){
		for(int j = y;j<y+height-6;j++){
			LCD_set_pixel(i,j, 0);
		}
	}
	for(int i = x; i<x+width; i++){
		for(int j = 48;j>  (y+height)+ gap + 6;j--){
				LCD_set_pixel(i,j, 0);
			}
		}
	if(x<82 && x>2){
	for(int i = x-2; i<x+width +2 ; i++){
			for(int j = y+height-5;j<y+height;j++){
				LCD_set_pixel(i,j, 0);
			}
		}
	for(int i = x-2; i<x+width +2 ; i++){
				for(int j = (y+height)+ gap + 5;j>  (y+height)+ gap;j--){
					LCD_set_pixel(i,j,0);
				}
			}
	}
}
void draw_bar(uint8_t x, uint8_t y, uint8_t height, uint8_t width, uint8_t gap){

	for(int i = x; i<x+width; i++){
		for(int j = y;j<y+height-6;j++){
			LCD_set_pixel(i,j, 1);
		}
	}
	for(int i = x; i<x+width; i++){
		for(int j = 48;j>  (y+height)+ gap + 6;j--){
			LCD_set_pixel(i,j, 1);
		}
	}

	if(x<82 && x>2){
	for(int i = x-2; i<x+width +2 ; i++){
			for(int j = y+height-5;j<y+height;j++){
				LCD_set_pixel(i,j, 1);
			}
		}
	for(int i = x-2; i<x+width +2 ; i++){
				for(int j = (y+height)+ gap + 5;j>  (y+height)+ gap;j--){
					LCD_set_pixel(i,j, 1);
				}
			}

//	for(int i = x-2; i<x+width +2 ; i++){
//				for(int j = 48;j>(y+height)- gap ;j++){
//					LCD_set_pixel(i,j, 1);
//				}
//			}
	}
//	LCD_printBuffer(buffer);
}
void shift_im(){
	uint16_t temp[60];
	uint8_t im[80];
	for(int i = 0 ; i<3;i++ ){
			for(int j = 0; j<20; j++)
				temp[20*i + j] = bird[20*i + j  ];
		}
	for(int i = 0; i<60; i++){
		temp[i] <<5;
	}
////	for(int i = 0; i<20; i++){
////		im[i] =(uint8_t) temp[i] ;
////	}
//	for(int i = 20; i<40; i++){
//			im[i] = (uint8_t)temp[i - 20] & 0x00FF;
//		}
//	for(int i = 20; i<40; i++){
//				im[i] = (uint8_t)im[i] | ((uint8_t)temp[i] && 0xFF00);
//			}
//   for(int i = 40; i<60; i++){
//	   im[i] =(uint8_t) temp[i-20] & 0x00FF;
//   }
//   for(int i = 60; i<80; i++){
//  	   im[i] = (uint8_t)temp[i] & 0xFF00;
//     }
   for(int i = 0; i<504; i++)
   		buffer[i] = 0x00;
   	for(int i = 0 ; i<4;i++ ){
   		for(int j = 0; j<20; j++)
   		buffer[i*84 +j] = im[20*i + j  ];
   	}
}
void deleteBuffer(){
	 for(int i = 0; i<504; i++)
	   		buffer[i] = 0x00;
}
void printImage(uint8_t x, uint8_t y, char data [], uint8_t sx,uint8_t sy){
//	 for(int i = 0; i<504; i++)
//	   		buffer[i] = 0x00;
		  for(int i = 0 ; i<sx;i++ ){
		     		for(int j = 0; j<sy; j++){
		     			for(int k = 0; k<8;k++){
			             LCD_set_pixel_im(j+x , y + k + 8*(sx -1 - i) , data[i*sy+j] & (1 << (7 - k)));
		     			}
		     		}
		 	  }
}
void printImageReverse(uint8_t x, uint8_t y, char data [], uint8_t sx,uint8_t sy){
//	 for(int i = 0; i<504; i++)
//	   		buffer[i] = 0x00;
		  for(int i = 0; i<sx;i++ ){
		     		for(int j = 0; j<sy; j++){
		     			for(int k = 7; k>0;k--){
			             LCD_set_pixel_im((j +x) ,  y + k + 8*i ,data[i*sy+(sy-j)] & (1<< (7-k)));
		     			}
		     		}
		 	  }
}
void deleteImageReverse(uint8_t x, uint8_t y, uint8_t sx,uint8_t sy){
//	 for(int i = 0; i<504; i++)
//	   		buffer[i] = 0x00;
		  for(int i = 0; i<sx;i++ ){
		     		for(int j = 0; j<sy; j++){
		     			for(int k = 7; k>0;k--){
			             LCD_set_pixel_im((j +x) ,  y + k + 8*i ,0);
		     			}
		     		}
		 	  }
}
void deleteImage(uint8_t x, uint8_t y, uint8_t sx,uint8_t sy){
//	 for(int i = 0; i<504; i++)
//	   		buffer[i] = 0x00;
		  for(int i = 0 ; i<sx;i++ ){
		     		for(int j = 0; j<sy; j++){
		     			for(int k = 0; k<8;k++){

			             LCD_set_pixel(j+x , y + k + 8*(sx -1 - i) , 0);
		     			}
		     		}
		 	  }

}
void printChar(uint8_t x, uint8_t y, char c){

	printImageReverse(x,y, ASCII[c - 0x20], 1,6);

}
void delChar(uint8_t x, uint8_t y){

	deleteImage(x,y, 1,6);
}
void printStr(char *str, uint8_t x, uint8_t y){

  uint8_t i = x;
  uint8_t str_len = strlen(str);
  for(int j = str_len-1; j >= 0;j--){
	  printChar(i,y,str[j]);
	  i+=6;
  }
}
void deleteStr(char *str, uint8_t x, uint8_t y){

  uint8_t i = x;
  uint8_t str_len = strlen(str);
  for(int j = str_len-1; j >= 0;j--){
	  delChar(i,y);
	  i+=6;
  }
}
int  printRandoms(int lower, int upper)
{
        int num;
        num = (rand() % (upper - lower + 1)) + lower;
        return num;
}
void lcd_init(){
	  LCD_setRST(GPIOB, GPIO_PIN_2);
	  LCD_setCE(GPIOB, GPIO_PIN_1);
	  LCD_setDC(GPIOB, GPIO_PIN_0);
	  LCD_setDIN(GPIOA, GPIO_PIN_7);
	  LCD_setCLK(GPIOA, GPIO_PIN_5);
	  LCD_init();
	  LCD_invert(0);
	  LCD_printBuffer(mari);
	  HAL_Delay(1000);
	  LCD_clrScr();
}
void RTC_show(){
	   RTC_TimeTypeDef timeStruct;
	   int hour, min, sec;
	  HAL_RTC_GetTime(&hrtc,&timeStruct,RTC_FORMAT_BIN);
	  hour = timeStruct.Hours;
	  min = timeStruct.Minutes;
	  sec = timeStruct.Seconds;

	  int hour_10 = hour % 10;
		  int hour_d  = hour/10;

		  int min_10 = min % 10;
		  int min_d  = min/10;

	 	  print_number(0,hour_d);
		  print_number(20, hour_10 );
		  print_number(40,min_d );
		  print_number(60,min_10);
		  HAL_Delay(500);

		  for(int i =0; i<84;i+=2){
			  if(i<40){

		  LCD_print("SD_not_found", 0, 0);

    }
  }
}
void Flappy_Bird(uint8_t game){
	  dead = 0;
	  int bar_height = 20;
	  bird_pos = 20;
	  if(game == 0)
	  {

	    /* USER CODE END WHILE */

	    /* USER CODE BEGIN 3 */



		  LCD_goXY(0, 0);
	      draw_bar(bar_pos, 0,bar_height,8, 15);

	      printImage(15, bird_pos,  bird, 2,16);
	      char c;
	      colision = 0;
	      if(bar_pos>15 && bar_pos<23){

	         if(bird_pos > bar_height -3 && bird_pos < bar_height + 3  ){
	        	 colision = 0;
	        	 printChar(0, 0, '0');
	             c = '0';
	         }
	         else{
	        	 colision = 1;
	        	 printChar(0, 0, '1');
	        	 c = '1';
	        	 dead = 1;
	        	 while(1){
	        		 LCD_printBuffer(over);
	        		 HAL_Delay(100);
	        	 }
	         }
	      }
	      if(bar_pos == 23 && dead ==0){
	    	  score++;
	      }
	      char  score_buffer [10];
	      itoa(score,score_buffer,10);
	      printStr("Score", 48,  40);
	      printStr(score_buffer, 30,  40);
	      LCD_printBuffer(buffer);

	      HAL_Delay(100);
	      delChar(0, 0);
	      delChar(30,  40);
		  deleteImage(15, bird_pos, 2,16);
		  delete_bar(bar_pos, 0,bar_height,8, 15);

		  if(bird_pos>0)
		  bird_pos--;
		  if(bar_pos>0){
		  bar_pos--;

		  }
		  else{
		  bar_pos = 84;
		  bar_height = printRandoms(5, 43);
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
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();

  HAL_Delay(1000);


   //LCD_putChar('a');   LCD_clrScr();
while(1){
	  if(debouncing>=2){
			  bird_pos+=2;
			  debouncing =0;

		  }
   if(game == 0){
       Flappy_Bird(game);
   }
   else{
	   LCD_printBuffer(mari);
   }
}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV128;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 0x12;
  sTime.Minutes = 0x39;
  sTime.Seconds = 0x50;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
  while(1)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
