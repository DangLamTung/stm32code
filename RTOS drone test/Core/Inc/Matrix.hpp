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


class Matrix {

public:
	int row;
	int col;
	double data[49];
	bool inv = true;
	Matrix(int row, int col);
	virtual ~Matrix();
    int get_value(double* data){
    	for (int i = 0; i < row; i++)
    	    for (int j = 0; j < col; j++)
    	    	this->data[i*col + j] = data[i*col + j];
    	return 0;
    }
    int print(UART_HandleTypeDef huart){
    	char buffer[20];
    	for (int i = 0; i < row; i++){
    	    	    for (int j = 0; j < col; j++){
    	    	    	ftoa(this->data[i*col + j], buffer, 5);
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

Matrix::Matrix(int row, int col){
	    this->row = row;
	    this->col = col;


	    for (int i = 0; i < row; i++)
	    	   for (int j = 0; j < col; j++){
	    		   this->data[i*col + j] = 0;
	    	   }
//	    			  this->data[i*col + j] = 0;
	}

Matrix::~Matrix() {
	// TODO Auto-generated destructor stub

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
Matrix getCofactor(Matrix A, int p, int q)
{
    Matrix temp = Matrix(A.row - 1,A.col - 1);
    uint8_t i,j,h,k;
    h = 0; k = 0;
    for(i = 0; i<A.row ; i++){
        for(j = 0; j<A.col;j++){
            if(p != i && q != j){
                temp.data[h*temp.col + k++] = A.data[i * A.col + j];
                // cout<< A.data[i * A.col + j] << " ";
                    if (j == A.col - 1)
                {
                    k = 0;
                    h++;
                }
            }

        }
        // cout<< "\n";
    }
    return temp;
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
Matrix add_mat(Matrix mat1, Matrix mat2) {
   Matrix temp = Matrix(mat1.row, mat2.col);
    if(mat2.row != mat1.row){

    }
    else{
   for (int i = 0; i < mat1.row; i++)
	   for(int j = 0 ; j < mat2.col; j++)
	    	   temp.data[i*temp.col +j] = mat1.data[i*mat1.col + j] + mat2.data[i*mat2.col + j];
    }
   return temp;
}
Matrix sub_mat(Matrix mat1, Matrix mat2) {
   Matrix temp = Matrix(mat1.row, mat2.col);
    if(mat2.row != mat1.row){

    }
    else{
   for (int i = 0; i < mat1.row; i++)
	   for(int j = 0 ; j < mat2.col; j++)
	    	   temp.data[i*temp.col +j] = mat1.data[i*mat1.col + j] - mat2.data[i*mat2.col + j];
    }
   return temp;
}

Matrix transpose(Matrix A) {
    Matrix L = Matrix(A.col, A.row);

    for (int i = 0; i < A.row; i++)
        for (int j = 0; j < A.col; j++) {
                L.data[j * L.col+ i] = A.data[i * A.col + j];
        }

    return L;
}
double determinant(Matrix mat)
{
    double D = 0; // Initialize result

    //  Base case : if matrix contains single element
    if (mat.row == 1)
        return mat.data[0];

    Matrix L = Matrix(mat.col, mat.row); // To store cofactors

    double sign = 1.0;  // To store sign multiplier

     // Iterate for each element of first row
    for (int f = 0; f < mat.col; f++)
    {
        // Getting Cofactor of mat[0][f]
        L =  getCofactor(mat, 0, f);
        D += sign * mat.data[f] * determinant(L);
        // terms are to be added with alternate sign
        sign = -sign;
    }

    return D;
}
Matrix adjoint(Matrix a)
{
    Matrix adj = Matrix(a.row,a.col);
    if (a.row == 1)
    {
        adj.data[0] = 1;

    }

    // temp is used to store cofactors of A[][]
    int sign = 1;


    for (int i=0; i<a.row; i++)
    {
        for (int j=0; j<a.row; j++)
        {
            // Get cofactor of A[i][j]
            Matrix temp = getCofactor(a,i, j);

            // sign of adj[j][i] positive if sum of row
            // and column indexes is even.
            sign = ((i+j)%2==0)? 1: -1;

            // Interchanging rows and columns to get the
            // transpose of the cofactor matrix
            adj.data[j * a.col + i] = (sign)*(determinant(temp));
        }
    }
    return adj;
}

// Function to calculate and store inverse, returns false if
// matrix is singular
Matrix inverse(Matrix a)
{
    Matrix inverse = Matrix(a.col,a.col);
    Matrix temp = Matrix(a.col,2*a.col);
    uint8_t i,j,k;
    double d;
    int n = temp.row;
    int n2 = temp.col;
    for (i = 0; i < n; i++)
           for (j = 0; j < n2; j++){
               if (j == (i + n))
                   temp.data[i*n2 + j] = 1;
               if(j < n){
                   temp.data[i*n2 +j] = a.data[i*n +j];
              }
           }
//    temp.print();
       /************** partial pivoting **************/
       for (i = n - 1; i >= 1; i--)
       {
           if (temp.data[(i - 1)*n2 + 0] < temp.data[i*n2 + 0])
               for (j = 0; j < n2; j++)
               {
                   d = temp.data[i*n2 + j];
                   temp.data[i*n2 + j] = temp.data[(i - 1)*n2 + j];
                   temp.data[(i - 1)*n2 + j] = d;
               }
       }
//       temp.print();
////
//       /********** reducing to diagonal  matrix ***********/
//

       for (i = 0; i < n; i++)
       {
           for (j = 0; j < n2; j++)
               if (j != i)
               {
            	   if(temp.data[i*n2 + i]!= 0){
                   d = temp.data[j*n2 + i] / temp.data[i*n2 + i];
                   for (k = 0; k < n2; k++)
                	   temp.data[j*n2 + k] -= temp.data[i*n2 + k] * d;
            	   }
            	   else{
            		   inverse.inv = false;
            		   return inverse;
            	   }
               }
       }

//       /************** reducing to unit matrix *************/
//       temp.print();
       for (i = 0; i < n; i++)
       {
           d = temp.data[i*n2 + i];
           for (j = 0; j <= n2; j++)
        	   temp.data[i*n2 + j] = temp.data[i*n2 + j] / d;
       }
//       temp.print();
       for(i = 0; i < n; i++){
    	   for(j = 0; j < n; j++){
    		   inverse.data[i*n + j] = temp.data[i*n2 + j + n];
    	   }
       }
    return inverse;
}


