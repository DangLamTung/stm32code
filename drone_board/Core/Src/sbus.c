/*
 * sbus.c
 *
 *  Created on: Nov 26, 2019
 *      Author: Đặng Lâm Tùng
 */
#include "sbus.h"
void sbus_decode(uint8_t data[6]){
	esc_value1 = (data[0] << 3) | ((data[1] & 0b11100000)>>5);
	esc_value2 = ((data[1] & 0b00011111)<<6)|((data[2] & 0b11111100)>>2);
	esc_value3 = (((data[2] &0b00000011)<<9)|(data[3]<<1))|((data[4] & 0b10000000)>>7);
	esc_value4 = ((data[4] & 0b01111111)<<4)|(data[5])>>4;
}
