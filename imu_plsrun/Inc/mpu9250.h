/*
 * mpu9250.h
 *
 *  Created on: Jun 12, 2019
 *      Author: PC
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include "stm32f1xx_hal.h"

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
#define  reg1 0x68
#define  reg2 0x69

#define  zero 0x00
#define  stop_i2c_master 0x22

#define  PI 3.141592654
#define  RAD2DEC 57.29577951

uint8_t data;
uint8_t data_raw[13];


float temp,roll,pitch;
float asax,asay,asaz;

#define  PI 3.141592654
#define  RAD2DEC 57.29577951

#define mRes  1.49938949939 /*resolution for magnetometer*/
#define accel_factor 16384.0
#define gyro_factor 16.4

/*magnetometer calibaration constant*/
#define magX_max 650.53
#define magX_min -223.9

#define magY_max 651.67
#define magY_min -103.27

#define magZ_max 34.09
#define magZ_min -553.92

#define mag_offet_x -138.84
#define mag_offet_y -137.395
#define mag_offet_z 29.75

#define scale_x 0.4721372748895686
#define scale_y 0.5150588453340749
#define scale_z -0.9437935133299374

/*ellipsoid magnetometer calibration matrixes*/
float A_m[9] = {
	    0.0044 ,   0.0002  , -0.0000,
	    0.0002  ,  0.0045  ,  0.0000,
	   -0.0000  ,  0.0000  ,  0.0053};
float b_m[3] = { -41.2889,
		   94.2928,
		    2.2679};
float magnet_calib [3];

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

char init_MPU();

void process_MPU();

/*Complementary filter constant*/

volatile float com_angle;
volatile float pre_com_angle;
void complementary_filter(float roll_acc,float gyro_acc,float dt);
/*Kalman filter constants*/
float t;
float x_k[4];
float P_plus_k[49];
float x[7] = {1 , 0,0,0,-0.2639,-0.2873,-0.2661};
float roll, pitch, yaw;

void quat2euler(float a, float b,float c, float d);

float Q[49]= {  0,0,0,0,0,0,0,
        0,0,0,0,0,0, 0,
         0,0,0,0,0,0,0,
          0,0,0,0,0,0,0,
		  0,0, 0 ,0 ,0.00001 ,0, 0,
         0,0,0,0,0,0.00001,0,
        0,0,0,0,0,0, 0.00001 };
float P[49]= {100, 0, 0, 0, 0, 0, 0,
	       0,100,0, 0, 0, 0, 0,
	       0, 0,100,0, 0, 0, 0,
	       0, 0, 0,100,0, 0, 0,
	       0, 0, 0, 0,100,0, 0,
	       0, 0, 0, 0, 0,100,0,
	       0, 0, 0, 0, 0, 0,100};
float R_full[36]= {  0.000178,0,0,0,0,0,
                     0,0.000134,0,0,0,0,
                     0,0,0.000192,0,0,0,
                     0,0,0,0.005,0,0,
                     0,0,0,0,0.005,0,
                     0,0,0,0,0,0.005};
#endif /* MPU6050_H_ */
