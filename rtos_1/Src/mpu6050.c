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
#define alpha 0.8

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
    uint8_t magnet_address = MAG_ADRRESS;
    mag_ava = 0;
  	char rx_data[25];
    if (HAL_I2C_IsDeviceReady(&hi2c1, device_address, 3, 2) != HAL_OK) {
    	strcpy( rx_data, "No Device \r \n");
      }
      else{
    	  strcpy( rx_data, "Device found \r \n");
      }
    //CDC_Transmit_FS((uint8_t *)&rx_data, sizeof(rx_data));
    HAL_UART_Transmit(&huart2,rx_data, strlen(rx_data),1000);

	/* Try to transmit via I2C */
	d[0] = PWR_MGMT_1;
    d[1] = 0;
	if(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)device_address , (uint8_t *)d, 2, 1000) != HAL_OK)
	{
		status = 0;
	}
	/* Set data sample rate */
	d[0] = SAMPLE_RATE;
	d[1] = sample_1khz;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)device_address,(uint8_t *)d,2,1000)!=HAL_OK);

	d[0] = GYRO_CONFIG;
	d[1] = gyro_con;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)device_address,(uint8_t *)d,2,1000)!=HAL_OK);

	d[0] = ACCEL_CONFIG;
	d[1] = 0x00;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)device_address,(uint8_t *)d,2,1000)!=HAL_OK);

	d[0] = INIT_ENB;
	d[1] = inter;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)device_address,(uint8_t *)d,2,1000)!=HAL_OK);

	d[0] = USER_CTRL;
    d[1] = zero;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)device_address,(uint8_t *)d,2,1000)!=HAL_OK);

	d[0] = INT_BYPASS;
	d[1] = stop_i2c_master;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)device_address,(uint8_t *)d,2,1000)!=HAL_OK);

	if (HAL_I2C_IsDeviceReady(&hi2c1, magnet_address, 3, 200) != HAL_OK) {
	    	strcpy( rx_data, "No Device \r \n");
	      }
	      else{
	    	    d[0] = CNTL1_AD;
	    	  	d[1] = ROM_MODE;
	    	  	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t) magnet_address,(uint8_t *)d,2,2)!=HAL_OK);

	    	  //	HAL_Delay(100);
	    	  	 for(long i = 1000000; i>0 ;i--){

	    	  		    	      }

	    	  	uint8_t sensitive = ASAX_AD;
	    	  	uint8_t asa [3];
	    	      while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)magnet_address, &sensitive, 1, 1000) != HAL_OK);
	    	      while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)magnet_address, &asa,3, 1000) != HAL_OK);

	    	      asax = (asa[0]-128)*0.5/128+1;
	    	      asay = (asa[1]-128)*0.5/128+1;
	    	      asaz = (asa[2]-128)*0.5/128+1;

	    	      d[0] = CNTL1_AD;
	    	      d[1] = zero;
	    	      while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)magnet_address,(uint8_t *)d,2,1000)!=HAL_OK);

	    	      for(long i = 1000000; i>0 ;i--){

	    	      }

	    	      d[0] = CNTL1_AD;
	    	      d[1] = 0x16;
	    	      while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)magnet_address,(uint8_t *)d,2,1000)!=HAL_OK);
	    	      for(long i = 1000000; i>0 ;i--){

	    	    }
	    	      //HAL_Delay(100);
	      }
	   // CDC_Transmit_FS((uint8_t *)&rx_data, strlen(rx_data));


  return status;
}
mpu_data_raw read_MPU(){
	uint8_t data[13];
    uint8_t reg = ACCEL_XOUT_H;
    uint8_t device_address = MPU_ADDRESS;

    mpu_data_raw raw;
    /* Read accelerometer reg*/
    while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)device_address, &reg, 1, 1000) != HAL_OK);
    while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)device_address, data,14, 1000) != HAL_OK);

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

	float Acc_x,Acc_y,Acc_z,Gyro_x,Gyro_y,Gyro_z,Mag_x,Mag_y,Mag_z,roll,pitch,yaw,roll_com;
	char buffer[7];
	char n[1] = "\n";

	mag_ava = 0;

	uint8_t data[13];
	uint8_t mag_data[7];

	uint8_t status;
	uint8_t reg = ACCEL_XOUT_H;
	uint8_t device_address = MPU_ADDRESS;
	uint8_t magnet_address = MAG_ADRRESS;

	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)device_address, &reg, 1, 1000) != HAL_OK);
	while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)device_address, data,14, 1000) != HAL_OK);

	Acc_x = (int16_t)(data[0] << 8 | data[1]);
	Acc_y = (int16_t)(data[2] << 8 | data[3]);
	Acc_z = (int16_t)(data[4] << 8 | data[5]);

    temp = (int16_t)(data[6] << 8 | data[7]);

    Gyro_x = (int16_t)(data[8] << 8 | data[9]);
    Gyro_y = (int16_t)(data[10] << 8 | data[11]);
    Gyro_z = (int16_t)(data[12] << 8 | data[13]);

//	Acc_x = (Acc_x-1092)/((float)accel_factor);
//	Acc_y = (Acc_y-252)/((float)accel_factor);
//	Acc_z = (Acc_z - 2792)/((float)accel_factor);
//
//	Gyro_x = (Gyro_x + 75)/gyro_factor;
//	Gyro_y = (Gyro_y + 10)/gyro_factor;
//	Gyro_z = (Gyro_z + 22)/gyro_factor;

    reg = 0x02;
//
//    ftoa(Gyro_x, buffer, 2);
//    strcat(buffer," ");
//    HAL_UART_Transmit(&huart2,buffer, strlen(buffer),1000);
//
//    ftoa(Gyro_y, buffer, 2);
//    strcat(buffer," ");
//    HAL_UART_Transmit(&huart2,buffer, strlen(buffer),1000);
//
//    ftoa(Gyro_z, buffer, 2);
//    strcat(buffer," ");
//    HAL_UART_Transmit(&huart2,buffer, strlen(buffer),1000);
//
//    ftoa(Acc_x, buffer, 2);
//    strcat(buffer," ");
//    HAL_UART_Transmit(&huart2,buffer, strlen(buffer),1000);
//
//    ftoa(Acc_y, buffer, 2);
//    strcat(buffer," ");
//    HAL_UART_Transmit(&huart2,buffer, strlen(buffer),1000);
//
//    ftoa(Acc_z, buffer, 2);
//    strcat(buffer,"\n");
//    HAL_UART_Transmit(&huart2,buffer, strlen(buffer),1000);

	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)magnet_address, &reg, 1, 1000) != HAL_OK);
	while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)magnet_address, &status,1, 1000) != HAL_OK);
    if(status == 3){
    	mag_ava = 1;
    	reg = 0x03;
    	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)magnet_address, &reg, 1, 1000) != HAL_OK);
    	while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)magnet_address, &mag_data,7, 1000) != HAL_OK);
    	//if(!(mag_data[6]|MAGIC_OVERFLOW_MASK)){
    		Mag_x = (int16_t)(mag_data[0] | (mag_data[1]<<8));
    		Mag_y = (int16_t)(mag_data[2] | (mag_data[3]<<8));
    		Mag_z = (int16_t)(mag_data[4] | (mag_data[5]<<8));
            if(magX_min < Mag_x && Mag_x < magX_max )
    		Mag_x = (Mag_x*asax*mRes  - mag_offet_x)*scale_x;
            if(magY_min < Mag_y && Mag_y < magY_max )
    		Mag_y = (Mag_y*asay*mRes  - mag_offet_y)*scale_y;
            if(magZ_min < Mag_z && Mag_z < magZ_max )
    		Mag_z = (Mag_z*asaz*mRes  - mag_offet_z)*scale_z;

            yaw = atan2(Mag_y,Mag_x)*RAD2DEC;


            itoa(Gyro_x, buffer, 10);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart2,buffer, strlen(buffer),1000);

            itoa(Gyro_y, buffer, 10);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart2,buffer, strlen(buffer),1000);

            itoa(Gyro_z, buffer, 10);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart2,buffer, strlen(buffer),1000);

            itoa(Acc_x, buffer, 10);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart2,buffer, strlen(buffer),1000);

            itoa(Acc_y, buffer, 10);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart2,buffer, strlen(buffer),1000);

            itoa(Acc_z, buffer, 10);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart2,buffer, strlen(buffer),1000);

    	    ftoa(Mag_x, buffer, 4);
    	    strcat(buffer," ");
    	    HAL_UART_Transmit(&huart2,buffer, strlen(buffer),1000);

    	    ftoa(Mag_y, buffer, 4);
    	    strcat(buffer," ");
    	    HAL_UART_Transmit(&huart2,buffer, strlen(buffer),1000);

    	    ftoa(Mag_z, buffer, 4);
    	    strcat(buffer,"\n");
    	    HAL_UART_Transmit(&huart2,buffer, strlen(buffer),1000);
//
//    	    ftoa(yaw , buffer, 4);
//    	    strcat(buffer,"\n");
//    	    HAL_UART_Transmit(&huart2,buffer, strlen(buffer),1000);

    	//}
    	//reg = 0x02;
//    	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)magnet_address, &reg, 1, 1000) != HAL_OK);
//    	while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)magnet_address, &status,1, 1000) != HAL_OK);
    }

	roll = atan2(Acc_y,Acc_z)*RAD2DEC;
	//roll = atan2(Acc_y,Acc_z)*RAD2DEC;
	pitch = atan(-Acc_x/sqrt(Acc_y*Acc_y+Acc_z*Acc_z))*RAD2DEC;


	complementary_filter(pitch,Gyro_x,0.0116);
//	 ftoa(com_angle , buffer, 2);
//	 strcat(buffer,"\n");
	// HAL_UART_Transmit(&huart2,buffer, strlen(buffer),1000);

    //CDC_Transmit_FS((uint8_t *)buffer, sizeof(buffer));

}
void complementary_filter(float angle_acc,float gyro_rate,float dt){
	com_angle = alpha*(com_angle + dt*gyro_rate) + (1-alpha)*angle_acc;
}
void calib_MPU(){

	float Acc_x,Acc_y,Acc_z,Gyro_x,Gyro_y,Gyro_z,roll,pitch;

		uint8_t data[13];
		uint8_t reg = ACCEL_XOUT_H;
		uint8_t device_address = MPU_ADDRESS;
	    uint8_t register_address = WHO_AM_I_REG;
     for(int i = 0; i<50;i++){
		while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)device_address, &reg, 1, 1000) != HAL_OK);
		while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)device_address, data,6, 1000) != HAL_OK);

		Acc_x += (int16_t)(data[0] << 8 | data[1]);
		Acc_y += (int16_t)(data[2] << 8 | data[3]);
		Acc_z += (int16_t)(data[4] << 8 | data[5]);

       //	    temp = (int16_t)(data[6] << 8 | data[7]);

	    Gyro_x += (int16_t)(data[8] << 8 | data[9]);
	    Gyro_y += (int16_t)(data[10] << 8 | data[11]);
	    Gyro_z += (int16_t)(data[12] << 8 | data[13]);
     }
     	Acc_x = Acc_x/100;
     	Acc_y = Acc_y/100;
     	Acc_z = Acc_z/100;

     	Gyro_x = Gyro_x/100;
     	Gyro_y = Gyro_y/100;
     	Gyro_z = Gyro_z/100;

//    bAx = Acc_x;
//    bAy = Acc_y;
//    bAz = Acc_z - 16384;
//
//    bGx = Gyro_x;
//    bGy = Gyro_y;
//    bGz = Gyro_z;


}



