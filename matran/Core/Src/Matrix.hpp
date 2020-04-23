/*
 * Matrix.hpp
 *
 *  Created on: Feb 7, 2020
 *      Author: tung
 */

#include "stm32f4xx_hal.h"
/*
 * Matrix.h
 *
 *  Created on: Feb 7, 2020
 *      Author: tung
 */
#include "main.h"
#include <stdlib.h>
#include <string.h>
#define MAX_PRECISION	(10)
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

char * ftoa(double f, char * buf, int precision);


class Matrix {

public:
	int row;
	int col;
    double *data;
	Matrix(int row, int col){
	    this->row = row;
	    this->col = col;
	    this->data = (double *)malloc(row * col * sizeof(double));

	    for (int i = 0; i < row; i++)
	    	   for (int j = 0; j < col; j++){
	    		   this->data[i*col + j] = 0;
	    	   }
//	    			  this->data[i*col + j] = 0;
	}
	virtual ~Matrix();
    int get_value(double* data){
    	for (int i = 0; i < row; i++)
    	    for (int j = 0; j < col; j++)
    	    	this->data[i*col + j] = data[i*col + j];
    	return 0;
    }
    int print(UART_HandleTypeDef huart){
    	char buffer[10];
    	for (int i = 0; i < row; i++){
    	    	    for (int j = 0; j < col; j++){
    	    	    	ftoa(this->data[i*col + j], buffer, 2);
    	    	    	strcat(buffer," ");
    	    	    	HAL_UART_Transmit(&huart,(uint8_t*) buffer, strlen(buffer),100);
    	    	    }
    	            HAL_UART_Transmit(&huart,(uint8_t*) "\n", 1,100);
    	}
    	HAL_UART_Transmit(&huart,(uint8_t*) "\n", 1,100);
    	return 0;
    }
    double det();
};

Matrix inverse(Matrix a);
Matrix transpose(Matrix A);
Matrix diag_mat(int row, int col);
Matrix mul_mat(Matrix mat1, Matrix mat2);


