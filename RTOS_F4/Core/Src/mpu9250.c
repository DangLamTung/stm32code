/*
 * mpu6050.c
 *
 *  Created on: Jun 12, 2019
 *      Author: PC
 */
#include "mpu9250.h"
#include <stdlib.h>

#define MAX_PRECISION	(10)
#define alpha 0.99

static const double dv1[6] = { 1.0, 1.3085, 1.0, 1.0, -1.9382, 0.94 };
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
float A_m[9] = {

	    0.0044 ,   0.0002  , -0.0000,
	    0.0002  ,  0.0045  ,  0.0000,
	   -0.0000  ,  0.0000  ,  0.0053};
float b_m[3] = {
		-41.2889,
		   94.2928,
		    2.2679};
double x[7] = {1 , 0,0,0,-0.2639,-0.2873,-0.2661};
double Q[49]= {  0,0,0,0,0,0,0,
        0,0,0,0,0,0, 0,
         0,0,0,0,0,0,0,
          0,0,0,0,0,0,0,
		  0,0, 0 ,0 ,1e-7 ,0, 0,
          0,0,0,0,0,1e-7,0,
          0,0,0,0,0,0, 1e-7 };
double P[49]= {100, 0, 0, 0, 0, 0, 0,
	       0,100,0, 0, 0, 0, 0,
	       0, 0,100,0, 0, 0, 0,
	       0, 0, 0,100,0, 0, 0,
	       0, 0, 0, 0,100,0, 0,
	       0, 0, 0, 0, 0,100,0,
	       0, 0, 0, 0, 0, 0,100};
double R_full[36]= {  9.99e-06,0,0,0,0,0,
                     0,9.99e-06,0,0,0,0,
                     0,0,9.99e-06,0,0,0,
                     0,0,0,2e-6,0,0,
                     0,0,0,0,2e-6,0,
                     0,0,0,0,0,2e-6};
double R[9]= {   7.6941e-07,0,0,
                     0,2.0176e-07,0,
                     0,0,1.5981e-07
                 };
float b0 = 1.000;
float b1 = 1.3085;
float b2 = 1.0000;
float roll_acc;
float a1 = -1.9382;
float a2 = 0.9400;
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



  	char rx_data[25];
    while (HAL_I2C_IsDeviceReady(&hi2c1, device_address, 3, 2) != HAL_OK) {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_SET);
    	HAL_Delay(500);
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_RESET);
      }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_SET);


//    HAL_UART_Transmit(&huart3,rx_data, strlen(rx_data),1000);

	/* Try to transmit via I2C */
	d[0] = PWR_MGMT_1;
    d[1] = 0;
	if(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)device_address , (uint8_t *)d, 2, 1000) != HAL_OK)
	{

	}

	/* Set data sample rate */

	d[0] = SAMPLE_RATE;
	d[1] = sample_1khz;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)device_address,(uint8_t *)d,2,1000)!=HAL_OK);

	d[0] = CONFIG;
	d[1] = lpf;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)device_address,(uint8_t *)d,2,1000)!=HAL_OK);

	d[0] = GYRO_CONFIG;
	d[1] = gyro_con;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)device_address,(uint8_t *)d,2,1000)!=HAL_OK);

	d[0] = ACCEL_CONFIG;
	d[1] = 0x00;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)device_address,(uint8_t *)d,2,1000)!=HAL_OK);

	d[0] = ACCEL_CONFIG2;
	d[1] = 0x02;
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
	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_RESET);
	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_SET);
	    	HAL_Delay(500);
	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_RESET);
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
  return status;
}


IMU_data process_MPU(){
	IMU_data data_raw;

	float Acc_x_,Acc_y_,Acc_z_,Gyro_x_,Gyro_y_,Gyro_z_;
	float Acc_x_e,Acc_y_e,Acc_z_e,Gyro_x_e,Gyro_y_e,Gyro_z_e;
	char buffer[8];

	char Rx_Data[50];


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

	Acc_x= (Acc_x - bAx )/16384.0;
	Acc_y = (Acc_y - bAy)/16384.0;
	Acc_z = (Acc_z - bAz)/16384.0;

	Gyro_x = (Gyro_x - bGx)/16.4;
	Gyro_y = (Gyro_y - bGy)/16.4;
	Gyro_z = (Gyro_z - bGz)/16.4;


    reg = 0x02;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)magnet_address, &reg, 1, 1000) != HAL_OK);
	while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)magnet_address, &status,1, 1000) != HAL_OK);
    if(status == 3){
    	reg = 0x03;
    	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)magnet_address, &reg, 1, 1000) != HAL_OK);
    	while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)magnet_address, &mag_data,7, 1000) != HAL_OK);
//    	if(!(mag_data[6]|MAGIC_OVERFLOW_MASK)){
    		Mag_x = (int16_t)(mag_data[0] | (mag_data[1]<<8));
    		Mag_y = (int16_t)(mag_data[2] | (mag_data[3]<<8));
    		Mag_z = (int16_t)(mag_data[4] | (mag_data[5]<<8));

            float m[3] = {Mag_x,Mag_y,Mag_z};
        	float temp[3] ;
            temp[0] = m[0] - b_m[0];
            temp[1] = m[1] - b_m[1];
            temp[2] = m[2] - b_m[2];

            magnet_calib[0] = A_m[0]*temp[0] + A_m[1]*temp[1] +A_m[2]*temp[2];
            magnet_calib[1] = A_m[3]*temp[0] + A_m[4]*temp[1] +A_m[5]*temp[2];
            magnet_calib[2] = A_m[6]*temp[0] + A_m[7]*temp[1] +A_m[8]*temp[2];

            data_raw.Gyro_x = Gyro_x;
            data_raw.Gyro_y = Gyro_y;
            data_raw.Gyro_z = Gyro_z;
            data_raw.Acc_x = Acc_x;
            data_raw.Acc_y = Acc_y;
            data_raw.Acc_z = Acc_z;
            data_raw.Mag_x = magnet_calib[0];
            data_raw.Mag_y = magnet_calib[1];
            data_raw.Mag_z = magnet_calib[2];
}
    return data_raw;
}
void process_magnet(){
	uint8_t data[13];
	uint8_t mag_data[7];

	uint8_t status;
	uint8_t reg = ACCEL_XOUT_H;
	uint8_t device_address = MPU_ADDRESS;
	uint8_t magnet_address = MAG_ADRRESS;
	 reg = 0x02;
		while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)magnet_address, &reg, 1, 1000) != HAL_OK);
		while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)magnet_address, &status,1, 1000) != HAL_OK);
	    if(status == 3){
	    	reg = 0x03;
	    	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)magnet_address, &reg, 1, 1000) != HAL_OK);
	    	while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)magnet_address, &mag_data,7, 1000) != HAL_OK);
	//    	if(!(mag_data[6]|MAGIC_OVERFLOW_MASK)){
	    		Mag_x = (int16_t)(mag_data[0] | (mag_data[1]<<8));
	    		Mag_y = (int16_t)(mag_data[2] | (mag_data[3]<<8));
	    		Mag_z = (int16_t)(mag_data[4] | (mag_data[5]<<8));

	            float m[3] = {Mag_x,Mag_y,Mag_z};
	        	float temp[3] ;
	            temp[0] = m[0] - b_m[0];
	            temp[1] = m[1] - b_m[1];
	            temp[2] = m[2] - b_m[2];

	            magnet_calib[0] = A_m[0]*temp[0] + A_m[1]*temp[1] +A_m[2]*temp[2];
	            magnet_calib[1] = A_m[3]*temp[0] + A_m[4]*temp[1] +A_m[5]*temp[2];
	            magnet_calib[2] = A_m[6]*temp[0] + A_m[7]*temp[1] +A_m[8]*temp[2];

	}
}
void complementary_filter(float Gyro_x,float Gyro_y,float Gyro_z,float  Acc_x,float Acc_y,float Acc_z, float magnet_calib1, float magnet_calib2, float magnet_calib3,float dt){
	/*Low pass filter*/

	/*Complemetary filter*/

	float roll_acc, pitch_acc,yaw_mag;
    roll_acc = atan2(Acc_y,Acc_z)*RAD2DEC;
    pitch_acc = atan(Acc_x/sqrt(Acc_y*Acc_y + Acc_z*Acc_z))*RAD2DEC;
    yaw_mag = atan2(magnet_calib2,magnet_calib1)*RAD2DEC;
	com_angle_r = alpha*(com_angle_r + dt*Gyro_x) + (1-alpha)*roll_acc;
	com_angle_p = alpha*(com_angle_p + dt*Gyro_y) + (1-alpha)*pitch_acc;
	com_angle_y = alpha*(com_angle_y + dt*Gyro_z) + (1-alpha)*yaw_mag;
}



void quat2euler(float a, float b,float c, float d){

	float sinr = 2*(a*b + c * d);
	float cosr = 1 - 2*(b*b + c * c);
	roll = atan2(sinr, cosr);


	float sinp = 2*( a*c - d*b);
    if (sinp >= 1)
        pitch = PI/2;
    else{
    	if(sinp <= -1){
        pitch = -PI/2;
    }
    else{
        pitch = asin(sinp);
    }
    }


	float siny = 2*( a*d + c*b);
	float cosy = 1 - 2*( b*b + d*d);
	yaw = atan2(siny, cosy);

    roll = roll*180/PI;
    pitch = pitch*180/PI;
    yaw = yaw*180/PI;
}

void calibration_IMU(){
    /*This function is performed when the sensor is fully stationary, we assume that MPU has been inited*/
	    char buffer[8];
	    print_msg("Calibrating the sensor....\n");
		uint8_t data[13];
		uint8_t mag_data[7];
		uint8_t status;
		uint8_t reg = ACCEL_XOUT_H;
		uint8_t device_address = MPU_ADDRESS;
		uint8_t magnet_address = MAG_ADRRESS;
        for(int i = 0; i<100; i++){

		while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)device_address, &reg, 1, 1000) != HAL_OK);
		while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)device_address, data,14, 1000) != HAL_OK);

		Acc_x = (int16_t)(data[0] << 8 | data[1])/accel_factor;
		Acc_y = (int16_t)(data[2] << 8 | data[3])/accel_factor;
		Acc_z = (int16_t)(data[4] << 8 | data[5])/accel_factor;

	    temp = (int16_t)(data[6] << 8 | data[7]);

	    Gyro_x = (int16_t)(data[8] << 8 | data[9])/gyro_factor;
	    Gyro_y = (int16_t)(data[10] << 8 | data[11])/gyro_factor;
	    Gyro_z = (int16_t)(data[12] << 8 | data[13])/gyro_factor;

	    bAx += Acc_x;
	    bAy += Acc_y;
	    bAz += Acc_z;

	    bGx += Gyro_x;
	    bGy += Gyro_y;
	    bGz += Gyro_z;
	    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
	    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
	    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
	    HAL_Delay(50);
        }

     bAx /= 100;
     bAy /= 100;
     bAz /= 100;
     g = sqrt(bAx*bAx + bAy*bAy + bAz*bAz);

     bGx /= 100;
     bGy /= 100;
     bGz /= 100;

	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,GPIO_PIN_RESET);
}

//void calibration_magnet(){
//	reg = 0x02;
//	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)magnet_address, &reg, 1, 1000) != HAL_OK);
//	while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)magnet_address, &status,1, 1000) != HAL_OK);
//	if(status == 3){
//	reg = 0x03;
//	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)magnet_address, &reg, 1, 1000) != HAL_OK);
//	while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)magnet_address, &mag_data,7, 1000) != HAL_OK);
//	//    	if(!(mag_data[6]|MAGIC_OVERFLOW_MASK)){
//	    		Mag_x = (int16_t)(mag_data[0] | (mag_data[1]<<8));
//	    		Mag_y = (int16_t)(mag_data[2] | (mag_data[3]<<8));
//	    		Mag_z = (int16_t)(mag_data[4] | (mag_data[5]<<8));
//}

void print_raw(IMU_data data_raw){
	        char buffer[10];
            ftoa(data_raw.Gyro_x, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,buffer, strlen(buffer),1000);

            ftoa(data_raw.Gyro_y, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,buffer, strlen(buffer),1000);

            ftoa(data_raw.Gyro_z, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,buffer, strlen(buffer),1000);

            ftoa(data_raw.Acc_x, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,buffer, strlen(buffer),1000);

            ftoa(data_raw.Acc_y, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,buffer, strlen(buffer),1000);

            ftoa(data_raw.Acc_z, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,buffer, strlen(buffer),1000);
////
            ftoa(data_raw.Mag_x, buffer, 2);
         	strcat(buffer," ");
         	HAL_UART_Transmit(&huart3,buffer, strlen(buffer),1000);

         	ftoa(data_raw.Mag_y, buffer, 2);
         	strcat(buffer," ");
         	HAL_UART_Transmit(&huart3,buffer, strlen(buffer),1000);

         	ftoa(data_raw.Mag_z, buffer, 2);
         	strcat(buffer,"\n");
         	HAL_UART_Transmit(&huart3,buffer, strlen(buffer),1000);

}
void print_magnet(char mode){
	char buffer[10];
	if(mode == 1){
	                	    ftoa(magnet_calib[0], buffer, 2);
	                	    strcat(buffer," ");
	            //    	    CDC_Transmit_FS((uint8_t *)&BU, sizeof(rx_data));
	                	    HAL_UART_Transmit(&huart3,buffer, strlen(buffer),1000);

	                	    ftoa(magnet_calib[1], buffer, 2);
	                	    strcat(buffer," ");
	                	    HAL_UART_Transmit(&huart3,buffer, strlen(buffer),1000);

	                	    ftoa(magnet_calib[2], buffer, 2);
	                	    strcat(buffer,"\n");


	}
	else{

	}
}
void print_Euler(){

}
void print_msg(char*msg){
	HAL_UART_Transmit(&huart3,msg, strlen(msg),1000);
}



