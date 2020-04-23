/*
 * mpu6050.c
 *
 *  Created on: Jun 12, 2019
 *      Author: PC
 */
#include "mpu6050.h"
#include <stdlib.h>
#include <stdio.h>

#define MAX_PRECISION	(10)
#define alpha 0.96


static const double rounders[MAX_PRECISION + 1] =
{
	0.5,				// 0
	0.05,				// 1
	0.005,				// 2
	0.0005,				// 3
	0.00005,			// 4
	0.000005,			// 5
	0.0000005,			// 6
	0.00000005,			// 7
	0.000000005,		// 8
	0.0000000005,		// 9
	0.00000000005		// 10
};
char * ftoa(double f, char * buf, int precision)
{
	char * ptr = buf;
	char * p = ptr;
	char * p1;
	char c;
	long intPart;

	// check precision bounds
	if (precision > MAX_PRECISION)
		precision = MAX_PRECISION;

	// sign stuff
	if (f < 0)
	{
		f = -f;
		*ptr++ = '-';
	}

	if (precision < 0)  // negative precision == automatic precision guess
	{
		if (f < 1.0) precision = 6;
		else if (f < 10.0) precision = 5;
		else if (f < 100.0) precision = 4;
		else if (f < 1000.0) precision = 3;
		else if (f < 10000.0) precision = 2;
		else if (f < 100000.0) precision = 1;
		else precision = 0;
	}

	// round value according the precision
	if (precision)
		f += rounders[precision];

	// integer part...
	intPart = f;
	f -= intPart;

	if (!intPart)
		*ptr++ = '0';
	else
	{
		// save start pointer
		p = ptr;

		// convert (reverse order)
		while (intPart)
		{
			*p++ = '0' + intPart % 10;
			intPart /= 10;
		}

		// save end pos
		p1 = p;

		// reverse result
		while (p > ptr)
		{
			c = *--p;
			*p = *ptr;
			*ptr++ = c;
		}

		// restore end pos
		ptr = p1;
	}

	// decimal part
	if (precision)
	{
		// place decimal point
		*ptr++ = '.';

		// convert
		while (precision--)
		{
			f *= 10.0;
			c = f;
			*ptr++ = '0' + c;
			f -= c;
		}
	}

	// terminating zero
	*ptr = 0;

	return buf;
}

char init_MPU(){
    char status = 1;
    uint8_t d[2];
    uint8_t device_address = MPU_ADDRESS;

    char rx_data[25];
    if (HAL_I2C_IsDeviceReady(&hi2c2, device_address, 3, 1000) != HAL_OK) {
    	strcpy( rx_data, "No Device \r \n");
      }
      else{
    	strcpy( rx_data, "Device found \r \n");
      }
//    HAL_UART_Transmit(&huart1, (uint8_t *)&rx_data, 25, 100);
	/* Try to transmit via I2C */
	d[0] = PWR_MGMT_1;
    d[1] = 0;
	if(HAL_I2C_Master_Transmit(&hi2c2,(uint16_t)device_address , (uint8_t *)d, 2, 1000) != HAL_OK)
	{
		status = 0;
	}
	/* Set data sample rate */
	d[0] = SAMPLE_RATE;
	d[1] = sample_1khz;
	while(HAL_I2C_Master_Transmit(&hi2c2,(uint16_t)device_address,(uint8_t *)d,2,1000)!=HAL_OK);

	d[0] = GYRO_CONFIG;
	d[1] = gyro_con;
	while(HAL_I2C_Master_Transmit(&hi2c2,(uint16_t)device_address,(uint8_t *)d,2,1000)!=HAL_OK);

	d[0] = ACCEL_CONFIG;
	d[1] = 0x00;
	while(HAL_I2C_Master_Transmit(&hi2c2,(uint16_t)device_address,(uint8_t *)d,2,1000)!=HAL_OK);

	d[0] = INIT_ENB;
	d[1] = inter;
	while(HAL_I2C_Master_Transmit(&hi2c2,(uint16_t)device_address,(uint8_t *)d,2,1000)!=HAL_OK);

  return status;
}
mpu_data_raw read_MPU(){
	uint8_t data[13];
    uint8_t reg = ACCEL_XOUT_H;
    uint8_t device_address = MPU_ADDRESS;

    mpu_data_raw raw;
    /* Read accelerometer reg*/
    while(HAL_I2C_Master_Transmit(&hi2c2,(uint16_t)device_address, &reg, 1, 1000) != HAL_OK);
    while(HAL_I2C_Master_Receive(&hi2c2,(uint16_t)device_address, data,14, 1000) != HAL_OK);

		/* Format */

    raw.Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);
    raw.Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
    raw.Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);
	temp = (int16_t)(data[6] << 8 | data[7]);
	raw.Gyroscope_X = (int16_t)(data[8] << 8 | data[9]);
	raw.Gyroscope_Y = (int16_t)(data[10] << 8 | data[11]);
	raw.Gyroscope_Z = (int16_t)(data[12] << 8 | data[13]);
	raw.Temperature = (float)(temp)/340.0 + (float)36.5;
    return raw;
}

void process_MPU(){
	float Acc_x,Acc_y,Acc_z,Gyro_x,Gyro_y,Gyro_z,roll,pitch,roll_com;

	uint8_t data[13];
	uint8_t reg = ACCEL_XOUT_H;
	uint8_t device_address = MPU_ADDRESS;

	while(HAL_I2C_Master_Transmit(&hi2c2,(uint16_t)device_address, &reg, 1, 1000) != HAL_OK);
	while(HAL_I2C_Master_Receive(&hi2c2,(uint16_t)device_address, data,14, 1000) != HAL_OK);

	Acc_x = (int16_t)(data[0] << 8 | data[1]);
	Acc_y = (int16_t)(data[2] << 8 | data[3]);
	Acc_z = (int16_t)(data[4] << 8 | data[5]);

    temp = (int16_t)(data[6] << 8 | data[7]);

    Gyro_x = (int16_t)(data[8] << 8 | data[9]);
    Gyro_y = (int16_t)(data[10] << 8 | data[11]);
    Gyro_z = (int16_t)(data[12] << 8 | data[13]);

	Acc_x = (Acc_x-1092)/((float)accel_factor);
	Acc_y = (Acc_y-252)/((float)accel_factor);
	Acc_z = (Acc_z - 2792)/((float)accel_factor);

	Gyro_x = (Gyro_x + 75)/gyro_factor;
	Gyro_y = (Gyro_y + 10)/gyro_factor;
	Gyro_z = (Gyro_z + 22)/gyro_factor;


	char buffer[7];
	char n = ' ';
	char r = ' ';
	roll = atan2(Acc_y,Acc_z)*RAD2DEC;
	//roll = atan2(Acc_y,Acc_z)*RAD2DEC;
	pitch = atan(-Acc_x/sqrt(Acc_y*Acc_y+Acc_z*Acc_z))*RAD2DEC;


	complementary_filter(pitch,Gyro_x,0.0116);
//    ftoa(com_angle, buffer, 2);
//    HAL_UART_Transmit(&huart1, buffer, 4, 1000);
//    HAL_UART_Transmit(&huart1, &n, 1, 1000);
////
////    ftoa(Acc_y, buffer, 2);
////    HAL_UART_Transmit(&huart1, buffer, 7, 1000);
////    HAL_UART_Transmit(&huart1, &n, 1, 1000);
////
////    ftoa(Acc_z, buffer, 2);
////    HAL_UART_Transmit(&huart1, buffer, 7, 1000);
////    HAL_UART_Transmit(&huart1, &n, 1, 1000);
////
////    ftoa(Gyro_x, buffer, 2);
////    HAL_UART_Transmit(&huart1, buffer, 7, 1000);
////    HAL_UART_Transmit(&huart1, &n, 1, 1000);
////
////    ftoa(Gyro_y, buffer, 2);
////    HAL_UART_Transmit(&huart1, buffer, 7, 1000);
////    HAL_UART_Transmit(&huart1, &n, 1, 1000);
////
////    ftoa(Gyro_z, buffer, 2);
////    HAL_UART_Transmit(&huart1, buffer, 7, 1000);
//    HAL_UART_Transmit(&huart1, &r, 1, 1000);
}
void complementary_filter(float angle_acc,float gyro_rate,float dt){
	com_angle = alpha*(com_angle + dt*gyro_rate) + (1-alpha)*angle_acc;
}





