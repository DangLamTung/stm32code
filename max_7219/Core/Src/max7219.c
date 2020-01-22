#include "max7219.h"
void write_data(uint16_t data, uint8_t mode){
	CS_H
//    HAL_SPI_Transmit(&hspi1, (uint8_t *)data, 2, 100);
	  for(int i = 0; i < 8; i++){

		CLK_L
	    HAL_GPIO_WritePin(DIN_PORT, DIN_PIN, !!(data & (1 << (8 - i))));
	    HAL_Delay(1);
	    CLK_H
		HAL_Delay(1);
	  }
    CS_L
}
