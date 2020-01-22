/*
 * File: IMU_EKF.h
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 24-Nov-2019 16:29:44
 */

#ifndef IMU_EKF_H
#define IMU_EKF_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>

/* Function Declarations */
extern void IMU_EKF(double x[7], double t, double P[49], const double Q[49],
                    const double R[9], double Ax, double Ay, double Az, double
                    Gx, double Gy, double Gz);

#endif

/*
 * File trailer for IMU_EKF.h
 *
 * [EOF]
 */
