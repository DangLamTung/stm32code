/*
 * File: IMU_EKF.h
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 06-Sep-2019 10:24:15
 */

#ifndef IMU_EKF_H
#define IMU_EKF_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>


/* Function Declarations */
extern void IMU_EKF(double u_k_1, const double y_meas[9], const double
                    x_hat_plus_k_1[7], const double P_plus_k_1[49], const double
                    Q_k[49], const double R_k[81], double T, double tau_rx,
                    double tau_ry, double tau_rz, double ax_inertial, double
                    ay_inertial, double az_inertial, double mx_inertial, double
                    my_inertial, double mz_inertial, double x_hat_plus_k[7],
                    double P_plus_k[49]);

#endif

/*
 * File trailer for IMU_EKF.h
 *
 * [EOF]
 */
