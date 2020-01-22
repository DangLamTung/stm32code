
float x[7] ; //state vector
float F[4][4];   //state trans matrix
float P[4][4]; //covariance matrix
float Q [4];   //state trans matrix
float y[3];
float R[6];

void ekf(float*a,float* w, float*m,float t);
