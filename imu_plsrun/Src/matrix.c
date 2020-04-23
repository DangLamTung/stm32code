#include "matrix.h"

Mat init_mat(int row, int col) {

	// TODO Auto-generated constructor stub
	Mat m;
	m.row = row;
	m.col = col;
		m.data = (float *)malloc(m.row * m.col * sizeof(float));
		for (int i = 0; i < row; i++)
		   for (int j = 0; j < col; j++)
			 m.data[i*col + j] = 0.0;
   return m;
}
Mat data_mat(int row, int col, float*data_input) {
	// TODO Auto-generated constructor stub
	Mat m = init_mat(row, col);
	for (int i = 0; i < m.row; i++)
	    for (int j = 0; j < m.col; j++)
			 m.data[i*m.col + j] = data_input[i*m.col + j];
    return m;
}

Mat add_mat(Mat mat1, Mat mat2) {
   Mat temp = init_mat(mat1.row, mat1.col);
   for (int i = 0; i < mat1.row; i++)
	   for(int j = 0 ; j < mat2.col; j++)
	    	   temp.data[i*temp.col +j] = mat1.data[i*mat1.col + j] + mat2.data[i*mat2.col + j];
   return temp;
}
Mat mul_mat(Mat mat1, Mat mat2) {
   Mat temp = init_mat(mat1.row, mat2.col);
   if(mat2.row != mat1.col)
   for (int i = 0; i < mat1.row; i++)
	   for(int j = 0 ; j < mat2.col; j++)
	       for(int k = 0; k < mat1.col ; k++)
	    	   temp.data[i*temp.col +j] += mat1.data[i*mat1.col + k] * mat2.data[k*mat2.col + j];
   return temp;
}


