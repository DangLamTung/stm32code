/*
 * mpu6050.h
 *
 *  Created on: Jun 12, 2019
 *      Author: PC
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include "stm32f1xx_hal.h"


#define MPU_ADDRESS     0xD0
/*MPU6050 register address*/
#define  WHO_AM_I_REG    0x75
#define  SAMPLE_RATE 0x19
#define  PWR_MGMT_1 0x6B
#define  CONFIG 0x1A
#define  GYRO_CONFIG  0x1B
#define  ACCEL_CONFIG  0x1C
#define  INIT_ENB 0x38
#define  ACCEL_XOUT_H 0x3B
#define  GYRO_ZOUT_L 0x48
/*MPU config */
#define  MPU_START 0x69
#define  inter 0x01
#define  gyro_con 0x18
#define  gyroXF 1
#define  sample_1khz 7
#define  reg1 0x68
#define  reg2 0x69
#define  PI 3.141592654
#define  RAD2DEC 57.29577951

uint8_t data;
uint8_t data_raw[13];

float temp,roll,pitch;
float bAx, bAy, bAz, bGx, bGy, bGz;

#define  PI 3.141592654
#define  RAD2DEC 57.29577951

#define accel_factor 16384.0
#define gyro_factor 16.4

char * ftoa(double f, char * buf, int precision);

typedef struct  {
	int16_t Accelerometer_X; /*!< Accelerometer value X axis */
	int16_t Accelerometer_Y; /*!< Accelerometer value Y axis */
	int16_t Accelerometer_Z; /*!< Accelerometer value Z axis */
	int16_t Gyroscope_X;     /*!< Gyroscope value X axis */
	int16_t Gyroscope_Y;     /*!< Gyroscope value Y axis */
	int16_t Gyroscope_Z;     /*!< Gyroscope value Z axis */
	float   Temperature;       /*!< Temperature in degrees */
} mpu_data_raw;

typedef struct  {
	float roll;
	float pitch;
	float yaw;

	float gyroX;
	float gyroY;
	float gyroZ;
	float   Temperature;       /*!< Temperature in degrees */

} mpu_data_processed;


I2C_HandleTypeDef hi2c2;
UART_HandleTypeDef huart1;

char init_MPU();
void calib_MPU();
mpu_data_raw read_MPU();
void process_MPU();

/*Complementary filter constant*/

volatile float com_angle;
volatile float kalman_angle;
volatile float pre_com_angle;
void complementary_filter(float roll_acc,float gyro_acc,float dt);

#endif /* MPU6050_H_ */
