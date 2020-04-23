/*
 * File: EKF.h
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 19-Nov-2019 23:24:33
 */

#ifndef EKF_H
#define EKF_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>

/* Function Declarations */
extern void EKF(float x[7], float t, float P[49], const float Q[49], const float
                R[36], float Ax, float Ay, float Az, float Mx, float My, float
                Mz, float Gx, float Gy, float Gz);

#endif

/*
 * File trailer for EKF.h
 *
 * [EOF]
 */

