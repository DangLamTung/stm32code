typedef struct
{
   int row;
   int col;
   double *data;
} Mat;

Mat init_mat(int row, int col);
Mat inverse(Mat a);
