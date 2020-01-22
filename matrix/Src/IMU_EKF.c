/*
 * File: IMU_EKF.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 06-Sep-2019 10:24:15
 */

/* Include Files */
#include <math.h>
#include "IMU_EKF.h"
#include "inv.h"

/* Function Definitions */

/*
 * Arguments    : double u_k_1
 *                const double y_meas[9]
 *                const double x_hat_plus_k_1[7]
 *                const double P_plus_k_1[49]
 *                const double Q_k[49]
 *                const double R_k[81]
 *                double T
 *                double tau_rx
 *                double tau_ry
 *                double tau_rz
 *                double ax_inertial
 *                double ay_inertial
 *                double az_inertial
 *                double mx_inertial
 *                double my_inertial
 *                double mz_inertial
 *                double x_hat_plus_k[7]
 *                double P_plus_k[49]
 * Return Type  : void
 */
void IMU_EKF(double u_k_1, const double y_meas[9], const double x_hat_plus_k_1[7],
             const double P_plus_k_1[49], const double Q_k[49], const double
             R_k[81], double T, double tau_rx, double tau_ry, double tau_rz,
             double ax_inertial, double ay_inertial, double az_inertial, double
             mx_inertial, double my_inertial, double mz_inertial, double
             x_hat_plus_k[7], double P_plus_k[49])
{
  double F[49];
  double b_norm;
  double F_tmp;
  double b_F_tmp;
  double c_F_tmp;
  double d_F_tmp;
  double e_F_tmp;
  double f_F_tmp;
  double g_F_tmp;
  double F_tmp_tmp;
  double h_F_tmp;
  double i_F_tmp;
  double x_hat_minus_k[7];
  double h_k[9];
  int i0;
  double H[63];
  int i1;
  int j_F_tmp;
  double b_F[49];
  static const signed char iv0[7] = { 1, 0, 0, 0, 0, 0, 0 };

  static const signed char iv1[7] = { 0, 1, 0, 0, 0, 0, 0 };

  int i2;
  static const signed char iv2[7] = { 0, 0, 1, 0, 0, 0, 0 };

  double P_minus_k[49];
  double b_H[81];
  double dv0[81];
  double b_tmp[63];
  double c_H[63];
  double h_k_tmp;
  double b_h_k_tmp;
  double c_h_k_tmp;
  static const signed char b_I[49] = { 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 1 };

  double K[63];
  double c_I[49];
  (void)u_k_1;

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /*  Extended Kalman filter function */
  /*  Input u_k_1: prior control signal */
  /*  Input y_meas: measurement data (gyroscope,  */
  /*                accelerometer, magnetometer) */
  /*  Input x_hat_plus_k_1: prior estimation */
  /*  Input P_plus_k_1: prior covariance of estimation */
  /*                    error */
  /*  Output x_hat_plus_k: posteriori estimation */
  /*  Output P_plus_k: posteriori covariance of estimation */
  /*                   error */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /*  System state and measurement funtions */
  /*  f1 = T*((-1/tau_rx)*x1 + w_rx) + x1; */
  /*  f2 = T*((-1/tau_ry)*x2 + w_ry) + x2; */
  /*  f3 = T*((-1/tau_rz)*x3 + w_rz) + x3; */
  /*  f4 = T*(1/2)*( x1*x7 - x2*x6 + x3*x5) + x4; */
  /*  f5 = T*(1/2)*( x1*x6 + x2*x7 - x3*x4) + x5; */
  /*  f6 = T*(1/2)*(-x1*x5 + x2*x4 + x3*x7) + x6; */
  /*  f7 = T*(1/2)*(-x1*x4 - x2*x5 - x3*x6) + x7; */
  /*  ax_inertial = 0; */
  /*  ay_inertial = 0; */
  /*  az_inertial = 9.81; */
  /*  mx_inertial = 25.97; */
  /*  my_inertial = 9.25; */
  /*  mz_inertial = -9.14; */
  /*  yAE = [ 0; 0; 9.81]; */
  /*  yME = [ 25.97; 9.25; -9.14]; */
  /*  R = [ (x4^2-x5^2-x6^2+x7^2)  2*(x4*x5+x6*x7)         2*(x4*x6-x5*x7);  */
  /* 		2*(x4*x5-x6*x7)        (-x4^2+x5^2-x6^2+x7^2)  2*(x5*x6+x4*x7); */
  /* 		2*(x4*x6+x5*x7)        2*(x5*x6-x4*x7)         (-x4^2-x5^2+x6^2+x7^2)]; */
  /*   */
  /*  h1 = x1 + v1; */
  /*  h2 = x2 + v2; */
  /*  h3 = x3 + v3; */
  /*  yAB = R*yAE + [v4; v5; v6] */
  /*  h4 = yAB(1); */
  /*  h5 = yAB(2); */
  /*  h6 = yAB(3); */
  /*  yMB = R*yME + [v7; v8; v9] */
  /*  h7 = yMB(1); */
  /*  h8 = yMB(2); */
  /*  h9 = yMB(3); */
  /*  (a) Compute the following partial derivative matrices: */
  F[0] = 1.0 - T / tau_rx;
  F[7] = 0.0;
  F[14] = 0.0;
  F[21] = 0.0;
  F[28] = 0.0;
  F[35] = 0.0;
  F[42] = 0.0;
  F[1] = 0.0;
  F[8] = 1.0 - T / tau_ry;
  F[15] = 0.0;
  F[22] = 0.0;
  F[29] = 0.0;
  F[36] = 0.0;
  F[43] = 0.0;
  F[2] = 0.0;
  F[9] = 0.0;
  F[16] = 1.0 - T / tau_rz;
  F[23] = 0.0;
  F[30] = 0.0;
  F[37] = 0.0;
  F[44] = 0.0;
  b_norm = T * x_hat_plus_k_1[6] / 2.0;
  F[3] = b_norm;
  F_tmp = T * x_hat_plus_k_1[5];
  b_F_tmp = -F_tmp / 2.0;
  F[10] = b_F_tmp;
  c_F_tmp = T * x_hat_plus_k_1[4];
  F[17] = c_F_tmp / 2.0;
  F[24] = 1.0;
  d_F_tmp = T * x_hat_plus_k_1[2];
  e_F_tmp = d_F_tmp / 2.0;
  F[31] = e_F_tmp;
  f_F_tmp = T * x_hat_plus_k_1[1];
  g_F_tmp = -f_F_tmp / 2.0;
  F[38] = g_F_tmp;
  F_tmp_tmp = T * x_hat_plus_k_1[0];
  h_F_tmp = F_tmp_tmp / 2.0;
  F[45] = h_F_tmp;
  F[4] = F_tmp / 2.0;
  F[11] = b_norm;
  F_tmp = T * x_hat_plus_k_1[3];
  i_F_tmp = -F_tmp / 2.0;
  F[18] = i_F_tmp;
  d_F_tmp = -d_F_tmp / 2.0;
  F[25] = d_F_tmp;
  F[32] = 1.0;
  F[39] = h_F_tmp;
  f_F_tmp /= 2.0;
  F[46] = f_F_tmp;
  c_F_tmp = -c_F_tmp / 2.0;
  F[5] = c_F_tmp;
  F[12] = F_tmp / 2.0;
  F[19] = b_norm;
  F[26] = f_F_tmp;
  b_norm = -F_tmp_tmp / 2.0;
  F[33] = b_norm;
  F[40] = 1.0;
  F[47] = e_F_tmp;
  F[6] = i_F_tmp;
  F[13] = c_F_tmp;
  F[20] = b_F_tmp;
  F[27] = b_norm;
  F[34] = g_F_tmp;
  F[41] = d_F_tmp;
  F[48] = 1.0;

  /*  (b) Perform the time update of the state estimate and estimation-error covariance as follows:  */
  /*  P_minus_k = F*P_plus_k_1*F.' + L*Q_k*L.'; */
  /*  v1=0; v2=0; v3=0; v4=0; v5=0; v6=0; v7=0; v8=0; v9=0; w_rx=0; w_ry=0; w_rz=0; */
  /*  f1 = T*((-1/tau_rx)*x1 + w_rx) + x1; */
  /*  f2 = T*((-1/tau_ry)*x2 + w_ry) + x2; */
  /*  f3 = T*((-1/tau_rz)*x3 + w_rz) + x3; */
  /*  f4 = T*(1/2)*( x1*x7 - x2*x6 + x3*x5) + x4; */
  /*  f5 = T*(1/2)*( x1*x6 + x2*x7 - x3*x4) + x5; */
  /*  f6 = T*(1/2)*(-x1*x5 + x2*x4 + x3*x7) + x6; */
  /*  f7 = T*(1/2)*(-x1*x4 - x2*x5 - x3*x6) + x7; */
  x_hat_minus_k[0] = T * (-1.0 / tau_rx * x_hat_plus_k_1[0]) + x_hat_plus_k_1[0];
  x_hat_minus_k[1] = T * (-1.0 / tau_ry * x_hat_plus_k_1[1]) + x_hat_plus_k_1[1];
  x_hat_minus_k[2] = T * (-1.0 / tau_rz * x_hat_plus_k_1[2]) + x_hat_plus_k_1[2];
  x_hat_minus_k[3] = T * 0.5 * ((x_hat_plus_k_1[0] * x_hat_plus_k_1[6] -
    x_hat_plus_k_1[1] * x_hat_plus_k_1[5]) + x_hat_plus_k_1[2] * x_hat_plus_k_1
    [4]) + x_hat_plus_k_1[3];
  x_hat_minus_k[4] = T * 0.5 * ((x_hat_plus_k_1[0] * x_hat_plus_k_1[5] +
    x_hat_plus_k_1[1] * x_hat_plus_k_1[6]) - x_hat_plus_k_1[2] * x_hat_plus_k_1
    [3]) + x_hat_plus_k_1[4];
  x_hat_minus_k[5] = T * 0.5 * ((-x_hat_plus_k_1[0] * x_hat_plus_k_1[4] +
    x_hat_plus_k_1[1] * x_hat_plus_k_1[3]) + x_hat_plus_k_1[2] * x_hat_plus_k_1
    [6]) + x_hat_plus_k_1[5];
  x_hat_minus_k[6] = T * 0.5 * ((-x_hat_plus_k_1[0] * x_hat_plus_k_1[3] -
    x_hat_plus_k_1[1] * x_hat_plus_k_1[4]) - x_hat_plus_k_1[2] * x_hat_plus_k_1
    [5]) + x_hat_plus_k_1[6];

  /*  (c) Compute the following partial derivative matrices: */
  h_k[0] = x_hat_minus_k[0];
  h_k[1] = x_hat_minus_k[1];
  h_k[2] = x_hat_minus_k[2];
  for (i0 = 0; i0 < 7; i0++) {
    for (i1 = 0; i1 < 7; i1++) {
      j_F_tmp = i0 + 7 * i1;
      b_F[j_F_tmp] = 0.0;
      b_norm = 0.0;
      for (i2 = 0; i2 < 7; i2++) {
        b_norm += F[i0 + 7 * i2] * P_plus_k_1[i2 + 7 * i1];
      }

      b_F[j_F_tmp] = b_norm;
    }

    for (i1 = 0; i1 < 7; i1++) {
      b_norm = 0.0;
      for (i2 = 0; i2 < 7; i2++) {
        b_norm += b_F[i0 + 7 * i2] * F[i1 + 7 * i2];
      }

      j_F_tmp = i0 + 7 * i1;
      P_minus_k[j_F_tmp] = b_norm + Q_k[j_F_tmp];
    }

    H[9 * i0] = iv0[i0];
    H[1 + 9 * i0] = iv1[i0];
    H[2 + 9 * i0] = iv2[i0];
  }

  H[3] = 0.0;
  H[12] = 0.0;
  H[21] = 0.0;
  b_norm = 2.0 * az_inertial * x_hat_minus_k[5];
  F_tmp = 2.0 * ay_inertial * x_hat_minus_k[4];
  H[30] = (4.0 * ax_inertial * x_hat_minus_k[3] + F_tmp) + b_norm;
  b_F_tmp = 2.0 * az_inertial * x_hat_minus_k[6];
  c_F_tmp = 2.0 * ay_inertial * x_hat_minus_k[3];
  H[39] = c_F_tmp - b_F_tmp;
  d_F_tmp = 2.0 * az_inertial * x_hat_minus_k[3];
  e_F_tmp = 2.0 * ay_inertial * x_hat_minus_k[6];
  H[48] = e_F_tmp + d_F_tmp;
  f_F_tmp = 2.0 * az_inertial * x_hat_minus_k[4];
  g_F_tmp = 2.0 * ay_inertial * x_hat_minus_k[5];
  H[57] = (4.0 * ax_inertial * x_hat_minus_k[6] + g_F_tmp) - f_F_tmp;
  H[4] = 0.0;
  H[13] = 0.0;
  H[22] = 0.0;
  F_tmp_tmp = 2.0 * ax_inertial * x_hat_minus_k[4];
  H[31] = F_tmp_tmp + b_F_tmp;
  b_F_tmp = 2.0 * ax_inertial * x_hat_minus_k[3];
  H[40] = (b_F_tmp + 4.0 * ay_inertial * x_hat_minus_k[4]) + b_norm;
  b_norm = 2.0 * ax_inertial * x_hat_minus_k[6];
  H[49] = f_F_tmp - b_norm;
  f_F_tmp = 2.0 * ax_inertial * x_hat_minus_k[5];
  H[58] = (4.0 * ay_inertial * x_hat_minus_k[6] - f_F_tmp) + d_F_tmp;
  H[5] = 0.0;
  H[14] = 0.0;
  H[23] = 0.0;
  H[32] = f_F_tmp - e_F_tmp;
  H[41] = b_norm + g_F_tmp;
  H[50] = (b_F_tmp + F_tmp) + 4.0 * az_inertial * x_hat_minus_k[5];
  H[59] = (F_tmp_tmp - c_F_tmp) + 4.0 * az_inertial * x_hat_minus_k[6];
  H[6] = 0.0;
  H[15] = 0.0;
  H[24] = 0.0;
  b_norm = 2.0 * mz_inertial * x_hat_minus_k[5];
  F_tmp = 2.0 * my_inertial * x_hat_minus_k[4];
  H[33] = (4.0 * mx_inertial * x_hat_minus_k[3] + F_tmp) + b_norm;
  b_F_tmp = 2.0 * mz_inertial * x_hat_minus_k[6];
  c_F_tmp = 2.0 * my_inertial * x_hat_minus_k[3];
  H[42] = c_F_tmp - b_F_tmp;
  d_F_tmp = 2.0 * mz_inertial * x_hat_minus_k[3];
  e_F_tmp = 2.0 * my_inertial * x_hat_minus_k[6];
  H[51] = e_F_tmp + d_F_tmp;
  f_F_tmp = 2.0 * mz_inertial * x_hat_minus_k[4];
  g_F_tmp = 2.0 * my_inertial * x_hat_minus_k[5];
  H[60] = (4.0 * mx_inertial * x_hat_minus_k[6] + g_F_tmp) - f_F_tmp;
  H[7] = 0.0;
  H[16] = 0.0;
  H[25] = 0.0;
  F_tmp_tmp = 2.0 * mx_inertial * x_hat_minus_k[4];
  H[34] = F_tmp_tmp + b_F_tmp;
  b_F_tmp = 2.0 * mx_inertial * x_hat_minus_k[3];
  H[43] = (b_F_tmp + 4.0 * my_inertial * x_hat_minus_k[4]) + b_norm;
  b_norm = 2.0 * mx_inertial * x_hat_minus_k[6];
  H[52] = f_F_tmp - b_norm;
  f_F_tmp = 2.0 * mx_inertial * x_hat_minus_k[5];
  H[61] = (4.0 * my_inertial * x_hat_minus_k[6] - f_F_tmp) + d_F_tmp;
  H[8] = 0.0;
  H[17] = 0.0;
  H[26] = 0.0;
  H[35] = f_F_tmp - e_F_tmp;
  H[44] = b_norm + g_F_tmp;
  H[53] = (b_F_tmp + F_tmp) + 4.0 * mz_inertial * x_hat_minus_k[5];
  H[62] = (F_tmp_tmp - c_F_tmp) + 4.0 * mz_inertial * x_hat_minus_k[6];

  /*  M = [ 1, 0, 0, 0, 0, 0, 0, 0, 0; */
  /*  	    0, 1, 0, 0, 0, 0, 0, 0, 0; */
  /*  	    0, 0, 1, 0, 0, 0, 0, 0, 0; */
  /*  	    0, 0, 0, 1, 0, 0, 0, 0, 0; */
  /*  	    0, 0, 0, 0, 1, 0, 0, 0, 0; */
  /*  	    0, 0, 0, 0, 0, 1, 0, 0, 0; */
  /*  	    0, 0, 0, 0, 0, 0, 1, 0, 0; */
  /*  	    0, 0, 0, 0, 0, 0, 0, 1, 0; */
  /*  	    0, 0, 0, 0, 0, 0, 0, 0, 1]; */
  /*  (d) Perform the measurement update of the state estimate and estimation error covariance as follows:    */
  /*  K = P_minus_k * H.' * inv(H*P_minus_k*H.' + M*R_k*M.'); */
  /*  note: H and M are unit matrices */
  for (i0 = 0; i0 < 9; i0++) {
    for (i1 = 0; i1 < 7; i1++) {
      j_F_tmp = i0 + 9 * i1;
      b_tmp[i1 + 7 * i0] = H[j_F_tmp];
      c_H[j_F_tmp] = 0.0;
      b_norm = 0.0;
      for (i2 = 0; i2 < 7; i2++) {
        b_norm += H[i0 + 9 * i2] * P_minus_k[i2 + 7 * i1];
      }

      c_H[j_F_tmp] = b_norm;
    }
  }

  for (i0 = 0; i0 < 9; i0++) {
    for (i1 = 0; i1 < 9; i1++) {
      b_norm = 0.0;
      for (i2 = 0; i2 < 7; i2++) {
        b_norm += c_H[i0 + 9 * i2] * b_tmp[i2 + 7 * i1];
      }

      j_F_tmp = i0 + 9 * i1;
      b_H[j_F_tmp] = b_norm + R_k[j_F_tmp];
    }
  }

  inv(b_H, dv0);

  /*  v1=0; v2=0; v3=0; v4=0; v5=0; v6=0; v7=0; v8=0; v9=0; */
  b_norm = x_hat_minus_k[3] * x_hat_minus_k[3];
  F_tmp = x_hat_minus_k[4] * x_hat_minus_k[4];
  b_F_tmp = b_norm - F_tmp;
  c_F_tmp = x_hat_minus_k[5] * x_hat_minus_k[5];
  d_F_tmp = x_hat_minus_k[6] * x_hat_minus_k[6];
  e_F_tmp = 2.0 * x_hat_minus_k[3] * x_hat_minus_k[4];
  f_F_tmp = 2.0 * x_hat_minus_k[5] * x_hat_minus_k[6];
  g_F_tmp = 2.0 * x_hat_minus_k[3] * x_hat_minus_k[5];
  F_tmp_tmp = 2.0 * x_hat_minus_k[4] * x_hat_minus_k[6];
  h_F_tmp = (b_F_tmp - c_F_tmp) + d_F_tmp;
  i_F_tmp = e_F_tmp + f_F_tmp;
  h_k_tmp = g_F_tmp - F_tmp_tmp;
  h_k[3] = (ax_inertial * h_F_tmp + ay_inertial * i_F_tmp) + az_inertial *
    h_k_tmp;
  b_h_k_tmp = 2.0 * x_hat_minus_k[3] * x_hat_minus_k[6];
  c_h_k_tmp = 2.0 * x_hat_minus_k[4] * x_hat_minus_k[5];
  b_F_tmp = (b_F_tmp + c_F_tmp) - d_F_tmp;
  e_F_tmp -= f_F_tmp;
  f_F_tmp = b_h_k_tmp + c_h_k_tmp;
  h_k[4] = (-ay_inertial * b_F_tmp + ax_inertial * e_F_tmp) + az_inertial *
    f_F_tmp;
  c_F_tmp = ((b_norm + F_tmp) - c_F_tmp) - d_F_tmp;
  d_F_tmp = g_F_tmp + F_tmp_tmp;
  g_F_tmp = b_h_k_tmp - c_h_k_tmp;
  h_k[5] = (-az_inertial * c_F_tmp + ax_inertial * d_F_tmp) - ay_inertial *
    g_F_tmp;
  h_k[6] = (mx_inertial * h_F_tmp + my_inertial * i_F_tmp) + mz_inertial *
    h_k_tmp;
  h_k[7] = (-my_inertial * b_F_tmp + mx_inertial * e_F_tmp) + mz_inertial *
    f_F_tmp;
  h_k[8] = (-mz_inertial * c_F_tmp + mx_inertial * d_F_tmp) - my_inertial *
    g_F_tmp;

  /*  P_plus_k = (I - K*H)*(P_minus_k)*(I - K*H)' + K*R*K'; */
  /*  note: H and M are unit matrices */
  for (i0 = 0; i0 < 7; i0++) {
    for (i1 = 0; i1 < 9; i1++) {
      j_F_tmp = i0 + 7 * i1;
      c_H[j_F_tmp] = 0.0;
      b_norm = 0.0;
      for (i2 = 0; i2 < 7; i2++) {
        b_norm += P_minus_k[i0 + 7 * i2] * b_tmp[i2 + 7 * i1];
      }

      c_H[j_F_tmp] = b_norm;
    }

    for (i1 = 0; i1 < 9; i1++) {
      j_F_tmp = i0 + 7 * i1;
      K[j_F_tmp] = 0.0;
      b_norm = 0.0;
      for (i2 = 0; i2 < 9; i2++) {
        b_norm += c_H[i0 + 7 * i2] * dv0[i2 + 9 * i1];
      }

      K[j_F_tmp] = b_norm;
    }

    for (i1 = 0; i1 < 7; i1++) {
      j_F_tmp = i0 + 7 * i1;
      F[j_F_tmp] = 0.0;
      b_norm = 0.0;
      for (i2 = 0; i2 < 9; i2++) {
        b_norm += K[i0 + 7 * i2] * H[i2 + 9 * i1];
      }

      F[j_F_tmp] = b_norm;
    }
  }

  for (i0 = 0; i0 < 49; i0++) {
    b_F[i0] = (double)b_I[i0] - F[i0];
  }

  for (i0 = 0; i0 < 7; i0++) {
    for (i1 = 0; i1 < 7; i1++) {
      j_F_tmp = i0 + 7 * i1;
      c_I[j_F_tmp] = 0.0;
      b_norm = 0.0;
      for (i2 = 0; i2 < 7; i2++) {
        b_norm += b_F[i0 + 7 * i2] * P_minus_k[i2 + 7 * i1];
      }

      c_I[j_F_tmp] = b_norm;
    }
  }

  for (i0 = 0; i0 < 7; i0++) {
    for (i1 = 0; i1 < 7; i1++) {
      j_F_tmp = i0 + 7 * i1;
      b_F[i1 + 7 * i0] = (double)b_I[j_F_tmp] - F[j_F_tmp];
    }

    for (i1 = 0; i1 < 9; i1++) {
      j_F_tmp = i0 + 7 * i1;
      b_tmp[j_F_tmp] = 0.0;
      b_norm = 0.0;
      for (i2 = 0; i2 < 9; i2++) {
        b_norm += K[i0 + 7 * i2] * R_k[i2 + 9 * i1];
      }

      b_tmp[j_F_tmp] = b_norm;
    }
  }

  for (i0 = 0; i0 < 7; i0++) {
    for (i1 = 0; i1 < 7; i1++) {
      j_F_tmp = i0 + 7 * i1;
      P_plus_k[j_F_tmp] = 0.0;
      b_norm = 0.0;
      for (i2 = 0; i2 < 7; i2++) {
        b_norm += c_I[i0 + 7 * i2] * b_F[i2 + 7 * i1];
      }

      P_plus_k[j_F_tmp] = b_norm;
    }
  }

  for (i0 = 0; i0 < 7; i0++) {
    for (i1 = 0; i1 < 7; i1++) {
      j_F_tmp = i0 + 7 * i1;
      b_F[j_F_tmp] = 0.0;
      b_norm = 0.0;
      for (i2 = 0; i2 < 9; i2++) {
        b_norm += b_tmp[i0 + 7 * i2] * K[i1 + 7 * i2];
      }

      b_F[j_F_tmp] = b_norm;
    }
  }

  for (i0 = 0; i0 < 49; i0++) {
    P_plus_k[i0] += b_F[i0];
  }

  for (i0 = 0; i0 < 9; i0++) {
    h_k[i0] = y_meas[i0] - h_k[i0];
  }

  for (i0 = 0; i0 < 7; i0++) {
    b_norm = 0.0;
    for (i1 = 0; i1 < 9; i1++) {
      b_norm += K[i0 + 7 * i1] * h_k[i1];
    }

    x_hat_plus_k[i0] = x_hat_minus_k[i0] + b_norm;
  }

  b_norm = sqrt(((x_hat_plus_k[3] * x_hat_plus_k[3] + x_hat_plus_k[4] *
                  x_hat_plus_k[4]) + x_hat_plus_k[5] * x_hat_plus_k[5]) +
                x_hat_plus_k[6] * x_hat_plus_k[6]);
  x_hat_plus_k[3] /= b_norm;
  x_hat_plus_k[4] /= b_norm;
  x_hat_plus_k[5] /= b_norm;
  x_hat_plus_k[6] /= b_norm;

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
}

/*
 * File trailer for IMU_EKF.c
 *
 * [EOF]
 */
