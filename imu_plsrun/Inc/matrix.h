#include <stdlib.h>
#include <stdio.h>
typedef struct{
   int row;
   int col;
   float* data;
} Mat;
Mat init_mat(int row, int col);
Mat add_mat(Mat mat1, Mat mat2);
Mat mul_mat(Mat mat1, Mat mat2);
