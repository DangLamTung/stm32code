
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "matrix.h"
Mat init_mat(int row, int col) {


	Mat m;
	m.row = row;
	m.col = col;
		m.data = (double *)malloc(m.row * m.col * sizeof(double));
		for (int i = 0; i < row; i++)
		   for (int j = 0; j < col; j++)
			 m.data[i*col + j] = 0.0;
   return m;
}

void free_mat(Mat a) {
   free(a.data);
}

Mat diag_mat(int row, int col) {

	// TODO Auto-generated constructor stub
	Mat m;
	m.row = row;
	m.col = col;
		m.data = (double *)malloc(m.row * m.col * sizeof(double));
		for (int i = 0; i < row; i++)
		   for (int j = 0; j < col; j++)
             if(i==j)
			 m.data[i*col + j] = 1;
             else
             {
                 m.data[i*col + j] = 0;
             }

   return m;
}
Mat mul_mat(Mat mat1, Mat mat2) {
   Mat temp = init_mat(mat1.row, mat2.col);
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

Mat cholesky(Mat A) {
    Mat L = init_mat(A.row, A.col);

    for (int i = 0; i < A.row; i++)
        for (int j = 0; j < (i+1); j++) {
            double s = 0;
            for (int k = 0; k < j; k++)
                s += L.data[i * A.row + k] * L.data[i * A.row + k];
            if(i == j) {
                L.data[i * A.row + j] =  sqrt(A.data[i * A.row + j] - s) ;
                // cout<< i << " " << s <<" " <<  A.data[i * A.row + j] <<"\n";
            }

            else{
             L.data[i * A.row + j] =  (1.0 / L.data[j * A.row + j] * (A.data[i * A.row + j] - s));
            }
        }

    return L;
}
// Mat cholesky_inv(Mat m){
//     Mat temp = init_mat(m.row,m.col);
//     int i,j,k;
//     double sum = 0;
//     for(i =0; i <m.col; i++){
//     temp.data[i*temp.col + i] = 1/m.data[i*m.col + i];
//        for(j = i+1 ; j < m.row; j++){
//            for(k = 0; k < j; k++){
//                sum -= m.data[j*m.col + k]* m.data[k*m.col + i];
//            }

//        } //upper
//     }
// }

Mat getCofactor(Mat A, int p, int q)
{
    Mat temp = init_mat(A.row - 1,A.col - 1);
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
Mat transpose(Mat A) {
    Mat L = init_mat(A.col, A.row);

    for (int i = 0; i < A.row; i++)
        for (int j = 0; j < A.col; j++) {

                L.data[j * L.row+ i] = A.data[i * A.col + j];

        }

    return L;
}
double determinant(Mat mat)
{
    double D = 0; // Initialize result

    //  Base case : if matrix contains single element
    if (mat.row == 1)
        return mat.data[0];

    Mat L = init_mat(mat.col, mat.row); // To store cofactors

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
Mat adjoint(Mat a)
{
    Mat adj = init_mat(a.row,a.col);
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
            Mat temp = getCofactor(a,i, j);

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
Mat inverse(Mat a)
{
    // Find determinant of A[][]
    float det = determinant(a);

    // Find adjoint
    Mat adj = adjoint(a);
    Mat inverse = init_mat(a.col,a.col);

    for (int i=0; i<a.col; i++)
        for (int j=0; j<a.col; j++)
            inverse.data[i*a.col + j] = adj.data[i*a.col + j]/det;

    return inverse;
}

double det_cholesky(Mat A) {
    // Mat L = cholesky(A,A.row );
    double det = 1.0;
    for (int i = 0; i < A.row; i++)
        for (int j = 0; j < A.col; j++)
        if(i==j)
           det = det*A.data[i*A.col + j];
    return det;
}
