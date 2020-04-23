/*
 * File: EKF.h
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 24-Nov-2019 09:52:46
 */

#ifndef EKF_H
#define EKF_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>


/* Function Declarations */
extern void EKF(double x[7], double t, double P[49], const double Q[49], const
                double R[36], double Ax, double Ay, double Az, double Mx, double
                My, double Mz, double Gx, double Gy, double Gz);

#endif

/*
 * File trailer for EKF.h
 *
 * [EOF]
 */
