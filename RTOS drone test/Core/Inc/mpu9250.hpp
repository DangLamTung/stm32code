/*
 * mpu9250.hpp
 *
 *  Created on: Feb 9, 2020
 *      Author: tung
 */

#ifndef INC_MPU9250_HPP_
#define INC_MPU9250_HPP_

/*
 * mpu9250.h
 *
 *  Created on: Jun 12, 2019
 *      Author: PC
 */



#include "mpu_data_type.hpp"
#define  MPU_ADDRESS     0xD0
#define  MAG_ADRRESS     0x18

/*MPU6050 register address*/
#define  WHO_AM_I_REG    0x75
#define  SAMPLE_RATE 0x19
#define  PWR_MGMT_1 0x6B
#define  CONFIG 0x1A
#define  GYRO_CONFIG  0x1B
#define  ACCEL_CONFIG  0x1C
#define  ACCEL_CONFIG2  0x1D
#define  INIT_ENB 0x38
#define  ACCEL_XOUT_H 0x3B
#define  GYRO_ZOUT_L 0x48


/*Magnetometer config*/
#define  USER_CTRL  0x6A
#define  INT_BYPASS 0x37
#define  CNTL1_AD   0x0A
#define  ROM_MODE   0x1F
#define  ASAX_AD    0x10
#define  MAGIC_OVERFLOW_MASK 0x8
/*MPU config */
#define  MPU_START 0x69
#define  inter 0x01
#define  gyro_con 0x18
#define  gyroXF 1
#define  sample_1khz 7
#define  lpf 0x01
#define  reg1 0x68
#define  reg2 0x69

#define  zero 0x00
#define  stop_i2c_master 0x22



uint8_t data;
uint8_t data_raw[13];


float temp,roll,pitch;
float asax,asay,asaz;

#define  PI 3.141592654
#define  RAD2DEC 57.29577951

#define mRes  1.49938949939 /*resolution for magnetometer*/
#define accel_factor 16384.0
#define gyro_factor 16.4

double bAx, bAy, bAz, bGx, bGy, bGz;

double ax_ref, ay_ref, az_ref;
double g_[3];
double g;
/*ellipsoid magnetometer calibration matrixes*/

float magnet_calib [3];


char init_MPU();

IMU_data process_MPU();

float Acc_x,Acc_y,Acc_z,Gyro_x,Gyro_y,Gyro_z,Mag_x,Mag_y,Mag_z;
float Acc_x_,Acc_y_,Acc_z_,Gyro_x_,Gyro_y_,Gyro_z_,Mag_x_,Mag_y_,Mag_z_;
/*Complementary filter constant*/

volatile float com_angle_r;
volatile float com_angle_p;
volatile float com_angle_y;

volatile float gyro_angle_r;
volatile float gyro_angle_p;
volatile float gyro_angle_y;

/*Kalman filter constants*/
float t;
double x_k[4];
double P_plus_k[49];





float w,w1,w2;
float w_,w1_,w2_;
float w__,w1__,w2__;

static const double dv1[6] = { 1.0, 1.3085, 1.0, 1.0, -1.9382, 0.94 };

float A_m[9] = {

	    0.0044 ,   0.0002  , -0.0000,
	    0.0002  ,  0.0045  ,  0.0000,
	   -0.0000  ,  0.0000  ,  0.0053};
float b_m[3] = {
		-41.2889,
		   94.2928,
		    2.2679};

float b0 = 1.000;
float b1 = 1.3085;
float b2 = 1.0000;
float roll_acc;
float a1 = -1.9382;
float a2 = 0.9400;

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
	    	      while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)magnet_address, (uint8_t *)asa,(uint16_t) 3, 1000) != HAL_OK);

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
void adding_raw(){
	  Gyro_x_ += Gyro_x;
	  Gyro_y_  += Gyro_y;
	  Gyro_z_  += Gyro_z;
	  Acc_x_  += Acc_x;
	  Acc_y_  += Acc_y;
	  Acc_z_  += Acc_z;
}

void delete_raw(){
	  Gyro_x_ = 0;
	  Gyro_y_ = 0;
	  Gyro_z_ = 0;
	  Acc_x_  = 0;
	  Acc_y_  = 0;
	  Acc_z_  = 0;

}


IMU_data process_MPU(bool EKF){
	IMU_data data_raw;
	uint8_t data[13];

	uint8_t reg = ACCEL_XOUT_H;
	uint8_t device_address = MPU_ADDRESS;


	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)device_address, &reg, 1, 1000) != HAL_OK);
	while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)device_address, data,14, 1000) != HAL_OK);

	Acc_x = (int16_t)(data[0] << 8 | data[1]);
	Acc_y = (int16_t)(data[2] << 8 | data[3]);
	Acc_z = (int16_t)(data[4] << 8 | data[5]);

    temp = (int16_t)(data[6] << 8 | data[7]);

    Gyro_x = (int16_t)(data[8] << 8 | data[9]);
    Gyro_y = (int16_t)(data[10] << 8 | data[11]);
    Gyro_z = (int16_t)(data[12] << 8 | data[13]);

	Acc_x= (Acc_x)/16384.0 - bAx ;
	Acc_y = (Acc_y)/16384.0 - bAy;
	Acc_z = (Acc_z)/16384.0 + bAz;
    if(!EKF){
	Gyro_x = (Gyro_x )/16.4- bGx;
	Gyro_y = (Gyro_y )/16.4- bGy;
	Gyro_z = (Gyro_z )/16.4- bGz;
    }
    else{
    	Gyro_x = (Gyro_x )/16.4;
    	Gyro_y = (Gyro_y )/16.4;
    	Gyro_z = (Gyro_z )/16.4;
    }
    data_raw.Gyro_x = Gyro_x;
    data_raw.Gyro_y = Gyro_y;
    data_raw.Gyro_z = Gyro_z;
    data_raw.Acc_x = Acc_x;
    data_raw.Acc_y = Acc_y;
    data_raw.Acc_z = Acc_z;
    return data_raw;
}
MAG_data process_magnet(){
	MAG_data temp;
	uint8_t mag_data[7];

	uint8_t status;
	uint8_t reg = ACCEL_XOUT_H;
	uint8_t magnet_address = MAG_ADRRESS;
	 reg = 0x02;
		while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)magnet_address, &reg, 1, 1000) != HAL_OK);
		while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)magnet_address, &status,1, 1000) != HAL_OK);
	    if(status == 3){
	    	reg = 0x03;
	    	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)magnet_address, &reg, 1, 1000) != HAL_OK);
	    	while(HAL_I2C_Master_Receive(&hi2c1,(uint16_t)magnet_address, (uint8_t *)mag_data,7, 1000) != HAL_OK);
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
	    temp.Mag_x = magnet_calib[0];
	    temp.Mag_y = magnet_calib[1];
	    temp.Mag_z = magnet_calib[2];
	    return temp;
}
EULER_angle complementary_filter(IMU_data data, float dt, float alpha){
	EULER_angle temp;
	float pitch_acc, roll_acc;
	if(data.Acc_x != 0 && data.Acc_y != 0 && data.Acc_z !=0){
    roll_acc = atan2(data.Acc_y,data.Acc_z)*RAD2DEC;
    if(roll_acc<0){
    	roll_acc+=180;
    }
    else{
    	if(roll_acc!=0){
    		roll_acc-=180;
    	}
    }
    pitch_acc = atan(data.Acc_x/sqrt(data.Acc_y*data.Acc_y + data.Acc_z*data.Acc_z))*RAD2DEC;

	com_angle_r = alpha*(com_angle_r + dt*data.Gyro_x) + (1-alpha)*roll_acc;
	com_angle_p = alpha*(com_angle_p + dt*data.Gyro_y) + (1-alpha)*pitch_acc;
	}
    temp.pitch = com_angle_p;
    temp.roll = com_angle_r;

    return temp;
}

EULER_angle quat2euler(quaternion q){
	float q0,q1,q2,q3,r,p,y;

	EULER_angle angle_e;
	q0 = q.q0;
	q1 = q.q1;
	q2 = q.q2;
	q3 = q.q3;

	float sinr = 2*(q0*q1 + q2 * q3);
	float cosr = 1 - 2*(q1*q1 + q2 * q2);
	r = atan2(sinr, cosr);


		float sinp = 2*( q0*q2 - q3*q1);
	    if (sinp >= 1)
	    	p = PI/2;
	    else{
	    	if(sinp <= -1){
	    	p = -PI/2;
	    }
	    else{
	    	p = asin(sinp);
	    }
	    }


		float siny = 2*( q0*q3 + q2*q1);
		float cosy = 1 - 2*( q1*q1 + q3*q3);
		y = atan2(siny, cosy);

		angle_e.roll = r*RAD2DEC;
		angle_e.pitch = p*RAD2DEC;
		angle_e.yaw = y*RAD2DEC;
		return angle_e;
}
void calibration_IMU(){
    /*This function is performed when the sensor is fully stationary, we assume that MPU has been inited*/
//	    print_msg("Calibrating the sensor....\n");
		uint8_t data[13];
		uint8_t reg = ACCEL_XOUT_H;
		uint8_t device_address = MPU_ADDRESS;

        for(int i = 0; i<200; i++){

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
	    HAL_Delay(30);
        }

     bAx /= 200;
     bAy /= 200;
     bAz /= 200;
     g = sqrt(bAx*bAx + bAy*bAy + bAz*bAz);
     bAz = -1 - bAz;
     bGx /= 200;
     bGy /= 200;
     bGz /= 200;

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





#endif /* INC_MPU9250_HPP_ */
