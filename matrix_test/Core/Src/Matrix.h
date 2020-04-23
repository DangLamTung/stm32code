/*
 * Matrix.h
 *
 *  Created on: Feb 7, 2020
 *      Author: tung
 */

#ifndef SRC_MATRIX_H_
#define SRC_MATRIX_H_
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
class Matrix {

public:
	int row;
	int col;
    double *data;
	Matrix(int row, int col);
	virtual ~Matrix();
    void get_value(double data[]);
};

Matrix inverse(Matrix a);
Matrix diag_mat(int row, int col);
Matrix transpose(Matrix A) ;
Matrix inverse(Matrix a);
#endif /* SRC_MATRIX_H_ */
