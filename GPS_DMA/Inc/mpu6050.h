/*
 * mpu6050.h
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

#define mag_offet_x 213.315
#define mag_offet_y 301.8
#define mag_offet_z -331.3

#define scale_x 0.31511301752494364
#define scale_y 0.3044515403371304
#define scale_z -0.2891795191694093

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


I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

char init_MPU();
mpu_data_raw read_MPU();
void process_MPU();

/*Complementary filter constant*/

volatile float com_angle , yaw;
volatile float pre_com_angle;
void complementary_filter(float roll_acc,float gyro_acc,float dt);

#endif /* MPU6050_H_ */
