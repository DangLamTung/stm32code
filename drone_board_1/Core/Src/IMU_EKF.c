/*
 * File: IMU_EKF.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 24-Nov-2019 16:29:44
 */

/* Include Files */
#include <math.h>
#include <string.h>
#include "IMU_EKF.h"

/* Function Definitions */

/*
 * Arguments    : double x[7]
 *                double t
 *                double P[49]
 *                const double Q[49]
 *                const double R[9]
 *                double Ax
 *                double Ay
 *                double Az
 *                double Gx
 *                double Gy
 *                double Gz
 * Return Type  : void
 */
void IMU_EKF(double x[7], double t, double P[49], const double Q[49], const
             double R[9], double Ax, double Ay, double Az, double Gx, double Gy,
             double Gz)
{
  double s_trans[12];
  int i0;
  int p2;
  double A[49];
  static const signed char iv0[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 1 };

  int p1;
  static const signed char iv1[21] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 1, 0, 0, 0, 1 };

  double b_s_trans[21];
  double b_A[7];
  double absx11;
  double c_s_trans[7];
  int itmp;
  double H_full[21];
  double c_A[49];
  int p3;
  double b_x[9];
  double c[9];
  double absx21;
  double absx31;
  double b_Gx[3];
  double K_full[21];

  /*        y = [  mag(k,1),mag(k,2), mag(k,3)]; */
  s_trans[0] = -x[1];
  s_trans[4] = -x[2];
  s_trans[8] = -x[3];
  s_trans[1] = x[0];
  s_trans[5] = -x[3];
  s_trans[9] = x[2];
  s_trans[2] = x[3];
  s_trans[6] = x[0];
  s_trans[10] = -x[1];
  s_trans[3] = -x[2];
  s_trans[7] = x[1];
  s_trans[11] = x[0];

  /*        */
  for (i0 = 0; i0 < 4; i0++) {
    p2 = i0 << 2;
    A[7 * i0] = iv0[p2];
    A[1 + 7 * i0] = iv0[1 + p2];
    A[2 + 7 * i0] = iv0[2 + p2];
    A[3 + 7 * i0] = iv0[3 + p2];
  }

  for (i0 = 0; i0 < 3; i0++) {
    p2 = i0 << 2;
    p1 = 7 * (i0 + 4);
    A[p1] = -s_trans[p2] * 0.5 * t;
    A[1 + p1] = -s_trans[1 + p2] * 0.5 * t;
    A[2 + p1] = -s_trans[2 + p2] * 0.5 * t;
    A[3 + p1] = -s_trans[3 + p2] * 0.5 * t;
  }

  for (i0 = 0; i0 < 7; i0++) {
    A[7 * i0 + 4] = iv1[3 * i0];
    A[7 * i0 + 5] = iv1[1 + 3 * i0];
    A[7 * i0 + 6] = iv1[2 + 3 * i0];
  }

  /*   */
  /*  %predict route */
  /*   */
  for (i0 = 0; i0 < 3; i0++) {
    p1 = i0 << 2;
    b_s_trans[7 * i0] = s_trans[p1] * 0.5 * t;
    b_s_trans[1 + 7 * i0] = s_trans[1 + p1] * 0.5 * t;
    b_s_trans[2 + 7 * i0] = s_trans[2 + p1] * 0.5 * t;
    b_s_trans[3 + 7 * i0] = s_trans[3 + p1] * 0.5 * t;
    b_s_trans[7 * i0 + 4] = 0.0;
    b_s_trans[7 * i0 + 5] = 0.0;
    b_s_trans[7 * i0 + 6] = 0.0;
  }

  for (i0 = 0; i0 < 7; i0++) {
    b_A[i0] = 0.0;
    absx11 = 0.0;
    for (itmp = 0; itmp < 7; itmp++) {
      absx11 += A[i0 + 7 * itmp] * x[itmp];
    }

    b_A[i0] = absx11;
    c_s_trans[i0] = 0.0;
    c_s_trans[i0] = (b_s_trans[i0] * Gx + b_s_trans[i0 + 7] * Gy) + b_s_trans[i0
      + 14] * Gz;
  }

  for (i0 = 0; i0 < 7; i0++) {
    x[i0] = b_A[i0] + c_s_trans[i0];
    for (itmp = 0; itmp < 7; itmp++) {
      p2 = i0 + 7 * itmp;
      c_A[p2] = 0.0;
      absx11 = 0.0;
      for (p3 = 0; p3 < 7; p3++) {
        absx11 += A[i0 + 7 * p3] * P[p3 + 7 * itmp];
      }

      c_A[p2] = absx11;
    }
  }

  for (i0 = 0; i0 < 7; i0++) {
    for (itmp = 0; itmp < 7; itmp++) {
      absx11 = 0.0;
      for (p3 = 0; p3 < 7; p3++) {
        absx11 += c_A[i0 + 7 * p3] * A[itmp + 7 * p3];
      }

      p3 = i0 + 7 * itmp;
      P[p3] = absx11 + Q[p3];
    }
  }

  absx11 = -2.0 * -x[2];
  H_full[0] = absx11;
  H_full[3] = -2.0 * x[3];
  H_full[6] = -2.0 * -x[0];
  H_full[9] = -2.0 * x[1];
  H_full[1] = -2.0 * x[1];
  H_full[4] = -2.0 * x[0];
  H_full[7] = -2.0 * x[3];
  H_full[10] = -2.0 * x[2];
  H_full[2] = -2.0 * x[0];
  H_full[5] = -2.0 * -x[1];
  H_full[8] = absx11;
  H_full[11] = -2.0 * x[3];
  for (i0 = 0; i0 < 3; i0++) {
    p1 = 3 * (i0 + 4);
    H_full[p1] = 0.0;
    H_full[1 + p1] = 0.0;
    H_full[2 + p1] = 0.0;
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (itmp = 0; itmp < 7; itmp++) {
      p1 = i0 + 3 * itmp;
      b_s_trans[p1] = 0.0;
      absx11 = 0.0;
      for (p3 = 0; p3 < 7; p3++) {
        absx11 += H_full[i0 + 3 * p3] * P[p3 + 7 * itmp];
      }

      b_s_trans[p1] = absx11;
    }

    for (itmp = 0; itmp < 3; itmp++) {
      absx11 = 0.0;
      for (p3 = 0; p3 < 7; p3++) {
        absx11 += b_s_trans[i0 + 3 * p3] * H_full[itmp + 3 * p3];
      }

      p1 = i0 + 3 * itmp;
      c[p1] = absx11 + R[p1];
    }
  }

  memcpy(&b_x[0], &c[0], 9U * sizeof(double));
  p1 = 0;
  p2 = 3;
  p3 = 6;
  absx11 = fabs(c[0]);
  absx21 = fabs(c[1]);
  absx31 = fabs(c[2]);
  if ((absx21 > absx11) && (absx21 > absx31)) {
    p1 = 3;
    p2 = 0;
    b_x[0] = c[1];
    b_x[1] = c[0];
    b_x[3] = c[4];
    b_x[4] = c[3];
    b_x[6] = c[7];
    b_x[7] = c[6];
  } else {
    if (absx31 > absx11) {
      p1 = 6;
      p3 = 0;
      b_x[0] = c[2];
      b_x[2] = c[0];
      b_x[3] = c[5];
      b_x[5] = c[3];
      b_x[6] = c[8];
      b_x[8] = c[6];
    }
  }

  b_x[1] /= b_x[0];
  b_x[2] /= b_x[0];
  b_x[4] -= b_x[1] * b_x[3];
  b_x[5] -= b_x[2] * b_x[3];
  b_x[7] -= b_x[1] * b_x[6];
  b_x[8] -= b_x[2] * b_x[6];
  if (fabs(b_x[5]) > fabs(b_x[4])) {
    itmp = p2;
    p2 = p3;
    p3 = itmp;
    absx11 = b_x[1];
    b_x[1] = b_x[2];
    b_x[2] = absx11;
    absx11 = b_x[4];
    b_x[4] = b_x[5];
    b_x[5] = absx11;
    absx11 = b_x[7];
    b_x[7] = b_x[8];
    b_x[8] = absx11;
  }

  b_x[5] /= b_x[4];
  b_x[8] -= b_x[5] * b_x[7];
  absx11 = (b_x[5] * b_x[1] - b_x[2]) / b_x[8];
  absx21 = -(b_x[1] + b_x[7] * absx11) / b_x[4];
  c[p1] = ((1.0 - b_x[3] * absx21) - b_x[6] * absx11) / b_x[0];
  c[p1 + 1] = absx21;
  c[p1 + 2] = absx11;
  absx11 = -b_x[5] / b_x[8];
  absx21 = (1.0 - b_x[7] * absx11) / b_x[4];
  c[p2] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
  c[p2 + 1] = absx21;
  c[p2 + 2] = absx11;
  absx11 = 1.0 / b_x[8];
  absx21 = -b_x[7] * absx11 / b_x[4];
  c[p3] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
  c[p3 + 1] = absx21;
  c[p3 + 2] = absx11;
  for (i0 = 0; i0 < 7; i0++) {
    for (itmp = 0; itmp < 3; itmp++) {
      p1 = i0 + 7 * itmp;
      b_s_trans[p1] = 0.0;
      absx11 = 0.0;
      for (p3 = 0; p3 < 7; p3++) {
        absx11 += P[i0 + 7 * p3] * H_full[itmp + 3 * p3];
      }

      b_s_trans[p1] = absx11;
    }

    for (itmp = 0; itmp < 3; itmp++) {
      p1 = i0 + 7 * itmp;
      K_full[p1] = 0.0;
      K_full[p1] = (b_s_trans[i0] * c[3 * itmp] + b_s_trans[i0 + 7] * c[1 + 3 *
                    itmp]) + b_s_trans[i0 + 14] * c[2 + 3 * itmp];
    }
  }

  /*        y; */
  /*      m = [ transpose(y) H*x]; */
  b_Gx[0] = Ax;
  b_Gx[1] = Ay;
  b_Gx[2] = Az;
  for (i0 = 0; i0 < 3; i0++) {
    absx11 = 0.0;
    for (itmp = 0; itmp < 7; itmp++) {
      absx11 += H_full[i0 + 3 * itmp] * x[itmp];
    }

    b_Gx[i0] -= absx11;
  }

  for (i0 = 0; i0 < 7; i0++) {
    x[i0] += (K_full[i0] * b_Gx[0] + K_full[i0 + 7] * b_Gx[1]) + K_full[i0 + 14]
      * b_Gx[2];
  }

  memset(&A[0], 0, 49U * sizeof(double));
  for (p1 = 0; p1 < 7; p1++) {
    A[p1 + 7 * p1] = 1.0;
  }

  for (i0 = 0; i0 < 7; i0++) {
    for (itmp = 0; itmp < 7; itmp++) {
      p2 = i0 + 7 * itmp;
      c_A[p2] = A[p2] - ((K_full[i0] * H_full[3 * itmp] + K_full[i0 + 7] *
                          H_full[1 + 3 * itmp]) + K_full[i0 + 14] * H_full[2 + 3
                         * itmp]);
    }
  }

  for (i0 = 0; i0 < 7; i0++) {
    for (itmp = 0; itmp < 7; itmp++) {
      p2 = i0 + 7 * itmp;
      A[p2] = 0.0;
      absx11 = 0.0;
      for (p3 = 0; p3 < 7; p3++) {
        absx11 += c_A[i0 + 7 * p3] * P[p3 + 7 * itmp];
      }

      A[p2] = absx11;
    }
  }

  memcpy(&P[0], &A[0], 49U * sizeof(double));
  absx11 = sqrt(((x[0] * x[0] + x[1] * x[1]) + x[2] * x[2]) + x[3] * x[3]);
  x[0] /= absx11;
  x[1] /= absx11;
  x[2] /= absx11;
  x[3] /= absx11;
}

/*
 * File trailer for IMU_EKF.c
 *
 * [EOF]
 */
