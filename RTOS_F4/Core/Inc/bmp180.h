#include "stm32f4xx_hal.h"

#define  BMP_ADDRESS     0xEE
#define  MEM_CONTROL     0xF4

#define out_msb          0xF6
#define out_lsb          0xF7
#define xlsb             0xF8

#define  start_read      0x2E
#define  oss             3
uint8_t calib_data[22];
int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD;
uint16_t AC4, AC5, AC6;

long x1, x2, x3, b3, b6, p, T;
unsigned long b4, b5, b7;
long UT,UP;

I2C_HandleTypeDef hi2c1;

void initBMP();
long readBMP();
double sample_BMP();
double get_alt_bmp();

