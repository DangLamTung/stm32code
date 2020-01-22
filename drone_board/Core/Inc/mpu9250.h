/*
 * mpu9250.h
 *
 *  Created on: Jun 12, 2019
 *      Author: PC
 */

#ifndef MPU9250_H_
#define MPU9250_H_

#include "stm32f4xx_hal.h"

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

double bAx, bAy, bAz, bGx, bGy, bGz;

double ax_ref, ay_ref, az_ref;
float roll_acc, pitch_acc,yaw_mag;
float roll_gyro, pitch_gyro;
double g_[3];
double g;
/*ellipsoid magnetometer calibration matrixes*/
float A_m[9] = {

	    0.0044 ,   0.0002  , -0.0000,
	    0.0002  ,  0.0045  ,  0.0000,
	   -0.0000  ,  0.0000  ,  0.0053};
float b_m[3] = {
		-41.2889,
		   94.2928,
		    2.2679};
float magnet_calib [3];

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart3;

char init_MPU();

uint8_t process_MPU();

float Acc_x,Acc_y,Acc_z,Gyro_x,Gyro_y,Gyro_z,Mag_x,Mag_y,Mag_z;
/*Complementary filter constant*/

volatile float com_angle_r;
volatile float com_angle_p;
volatile float com_angle_y;

float med_gyroX,med_gyroY,med_gyroZ;
float med_accX,med_accY,med_accZ;

void complementary_filter(float Gyro_x,float Gyro_y,float Gyro_z,float  Acc_x,float Acc_y,float Acc_z, float dt);
/*Kalman filter constants*/
 count;
float t;
double x_k[4];
double P_plus_k[49];
double x[7] = {1 , 0,0,0,-0.2639,-0.2873,-0.2661};
float roll, pitch, yaw;

void quat2euler(float a, float b,float c, float d);

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
float y1,y2,y3,y3_;


float w,w1,w2;
float w_,w1_,w2_;
float w__,w1__,w2__;
float w01,w11,w21;

float b0 = 1.000;
float b1 = 1.3085;
float b2 = 1.0000;
float roll_acc;
float a1 = -1.9382;
float a2 = 0.9400;
#endif /* MPU9250_H_ */
