
#include "kalman.h"



static void mulvec(double * a, double * x, double * y, int m, int n)
{
    int i, j;

    for(i=0; i<m; ++i) {
        y[i] = 0;
        for(j=0; j<n; ++j)
            y[i] += x[j] * a[i*n+j];
    }
}



void ekf(float*a,float*w, float*m,float t){
    float wx = (t*w[0])/2;
    float wy = (t*w[1])/2;
    float wz = (t*w[2])/2;

	float B[4][4]= {{  0, -wx, -wy, -wz},
	     { wx,         0,  wz, -wy},
	     { wy, -wz,         0,  wx},
		 { wz,  wy, -wx,         0}};

	float Bt[4][4] = {{  0, wx, wy, wz},
		              { -wx,    0,  -wz, wy},
		              { -wy, wz,     0,  -wx},
			          { -wz, - wy, wx,    0}};
   float temp;
	int i, j, k;
	    for (i = 0; i < 4; i++)
	    {
	        for (j = 0; j < 4; j++)
	        {

	            for (k = 0; k < 4; k++)
	                temp += F[i][k]*P[k][j];
	            P[i][j] = temp;
	        }

	    }

//	mulmat(P,Ft,P,4,4,4);

	x[0] = x[0] + x[1]*wx + x[2]*wy + x[3]*wz;
	x[1] = (x[1] - x[0]*wx + x[3]*wy - x[2]*wz);
	x[2] =	(x[2] - x[3]*wx -x[0]*wy + x[1]*wz);
	x[3] =	(x[3] + x[2]*wx - x[1]*wy - x[0]*wz);

    float H[3][4] = {{-2*x[2], -2*x[3] , -2*x[0], -2*x[1]},
    	 {2*x[1],  2*x[0], -2*x[3] , -2*x[2]},
         {-2*x[0],  2*x[1],  2*x[2], -2*x[3]}};





}


