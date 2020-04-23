/*
 * Matrix.cpp
 *
 *  Created on: Feb 7, 2020
 *      Author: tung
 */

#include "Matrix.h"

Matrix::Matrix(int row, int col,double data[]){
    this->row = row;
    this->col = col;
    this->data = (double *)malloc(row * col * sizeof(double));
    for (int i = 0; i < row; i++)
    	   for (int j = 0; j < col; j++)
    			  this->data[i*col + j] = 0;
}

Matrix::~Matrix() {
	// TODO Auto-generated destructor stub
	  free(this->data);
}
Matrix::get_value(double data[]){
	for (int i = 0; i < row; i++)
	    for (int j = 0; j < col; j++)
	    	this->data[i*col + j] = data[i*col + j];
}

Matrix diag_mat(int row, int col) {

	// TODO Auto-generated constructor stub
	Matrix m = Matrix(row,col);
	for (int i = 0; i < row; i++)
		for (int j = 0; j < col; j++)
           if(i==j)
			 m.data[i*col + j] = 1;
   return m;
}
Matrix mul_mat(Matrix mat1, Matrix mat2) {
   Matrix temp = Matrix(mat1.row, mat2.col);
    if(mat2.row != mat1.col){

    }
    else{
   for (int i = 0; i < mat1.row; i++)
	   for(int j = 0 ; j < mat2.col; j++)
	       for(int k = 0; k < mat1.col ; k++)
	    	   temp.data[i*temp.col +j] += mat1.data[i*mat1.col + k] * mat2.data[k*mat2.col + j];
    }
   return temp;
}

Matrix transpose(Matrix A) {
    Matrix L = Matrix(A.col, A.row);

    for (int i = 0; i < A.row; i++)
        for (int j = 0; j < A.col; j++) {

                L.data[j * L.row+ i] = A.data[i * A.col + j];

        }

    return L;
}
