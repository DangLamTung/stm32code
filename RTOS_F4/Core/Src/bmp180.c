#include "bmp180.h"

void initBMP(){
    while (HAL_I2C_IsDeviceReady(&hi2c1, (uint8_t) BMP_ADDRESS, 3, 2) != HAL_OK) {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,GPIO_PIN_SET);
    	HAL_Delay(500);
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,GPIO_PIN_RESET);
    	HAL_Delay(500);
      }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_SET);

    uint8_t reg;
    uint8_t status;
    uint8_t d[2];
    reg = 0xD0;

	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)BMP_ADDRESS, &reg, 1, 1000) != HAL_OK);
	while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)BMP_ADDRESS, &status,1, 1000) != HAL_OK);
    if(status == 0x55){
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_SET);
    }
    else{
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,GPIO_PIN_SET);
    	HAL_Delay(500);
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,GPIO_PIN_RESET);
    }
/*Get calibration coff*/
    uint8_t count = 0;
    for(reg = 0xAA; reg<= 0xBF; reg++){
    while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)BMP_ADDRESS, &reg, 1, 1000) != HAL_OK);
    while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)BMP_ADDRESS,  &calib_data[count],1, 1000) != HAL_OK);
    count++;
    }

	AC1 = (int16_t)(calib_data[0] << 8 | calib_data[1]);
	AC2 = (int16_t)(calib_data[2] << 8 | calib_data[3]);
	AC3 = (int16_t)(calib_data[4] << 8 | calib_data[5]);

	AC4 = (uint16_t)(calib_data[6] << 8 | calib_data[7]);
	AC5 = (uint16_t)(calib_data[8] << 8 | calib_data[9]);
	AC6 = (uint16_t)(calib_data[10] << 8 | calib_data[11]);

    B1 = (int16_t)(calib_data[12] << 8 | calib_data[13]);
    B2 = (int16_t)(calib_data[14] << 8 | calib_data[15]);


    MB = (int16_t)(calib_data[16] << 8 | calib_data[17]);
    MC = (int16_t)(calib_data[18] << 8 | calib_data[19]);
    MD = (int16_t)(calib_data[20] << 8 | calib_data[21]);
}

long readBMP(){
    uint8_t reg;
    uint8_t status;
    uint8_t d[3];

	d[0] = MEM_CONTROL;
	d[1] = start_read;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)BMP_ADDRESS,(uint8_t *)d,2,1000)!=HAL_OK);
	osDelay(5);

    reg = out_msb;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)BMP_ADDRESS, &reg, 1, 1000) != HAL_OK);
	while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)BMP_ADDRESS, &d[0],1, 1000) != HAL_OK);

    reg = out_lsb;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)BMP_ADDRESS, &reg, 1, 1000) != HAL_OK);
	while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)BMP_ADDRESS, &d[1],1, 1000) != HAL_OK);
    UT = d[0]<<8 | d[1];

	d[0] = MEM_CONTROL;
	d[1] = 0x34 | (oss<<6);
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)BMP_ADDRESS,(uint8_t *)d,2,1000)!=HAL_OK);

	osDelay(28);

    reg = out_msb;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)BMP_ADDRESS, &reg, 1, 1000) != HAL_OK);
	while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)BMP_ADDRESS, &d[0],1, 1000) != HAL_OK);

    reg = out_lsb;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)BMP_ADDRESS, &reg, 1, 1000) != HAL_OK);
	while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)BMP_ADDRESS, &d[1],1, 1000) != HAL_OK);

    reg = xlsb;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)BMP_ADDRESS, &reg, 1, 1000) != HAL_OK);
	while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)BMP_ADDRESS, &d[2],1, 1000) != HAL_OK);

	UP = (d[0]<<16 | d[1]<<8 | d[2])>>(8-oss);
    p = 0;
	x1 = (UT - AC6)*AC5>>15;
    x2 = (MC<<11)/(x1 + MD);
    b5 = x1 + x2;

    T = (b5 + 8)>>4;


	  b6 = b5 - 4000;
	  // Calculate B3
	  x1 = (B2 * (b6 * b6)>>12)>>11;
	  x2 = (AC2 * b6)>>11;
	  x3 = x1 + x2;
	  b3 = (((((long)AC1)*4 + x3)<<oss) + 2)>>2;

	  // Calculate B4
	  x1 = (AC3 * b6)>>13;
	  x2 = (B1 * ((b6 * b6)>>12))>>16;
	  x3 = ((x1 + x2) + 2)>>2;
	  b4 = (AC4 * (unsigned long)(x3 + 32768))>>15;

	  b7 = ((unsigned long)(UP - b3) * (50000>>oss));
	  if (b7 < 0x80000000)
	    p = (b7<<1)/b4;
	  else
	    p = (b7/b4)<<1;

	  x1 = (p>>8) * (p>>8);
	  x1 = (x1 * 3038)>>16;
	  x2 = (-7357 * p)>>16;
	  p += (x1 + x2 + 3791)>>4;
   return p;
}
double sample_BMP(){
	double temp_p = 0;
for(int i = 0; i<5; i++){
	temp_p += readBMP();
}
   temp_p = temp_p/5.0;
   return temp_p;
}

double get_alt_bmp(){
	double p = sample_BMP();
	double temp = pow(p/101325.0,0.1902949571836);
	double alt = 44330*(1- temp);
	return alt;
}
