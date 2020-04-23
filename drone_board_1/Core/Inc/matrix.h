typedef struct
{
   int row;
   int col;
   double *data;
} Mat;

Mat init_mat(int row, int col);
Mat inverse(Mat a);
Mat diag_mat(int row, int col) ;
Mat transpose(Mat A) ;
Mat inverse(Mat a);
