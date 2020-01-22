/*
 * File: EKF.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 24-Nov-2019 09:52:46
 */

/* Include Files */
#include <math.h>
#include <string.h>
#include "EKF.h"

/* Function Definitions */

/*
 * Arguments    : double x[7]
 *                double t
 *                double P[49]
 *                const double Q[49]
 *                const double R[36]
 *                double Ax
 *                double Ay
 *                double Az
 *                double Mx
 *                double My
 *                double Mz
 *                double Gx
 *                double Gy
 *                double Gz
 * Return Type  : void
 */
void EKF(double x[7], double t, double P[49], const double Q[49], const double
         R[36], double Ax, double Ay, double Az, double Mx, double My, double Mz,
         double Gx, double Gy, double Gz)
{
  double s_trans[12];
  int i0;
  int jy;
  double A[49];
  static const signed char iv0[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 1 };

  int iy;
  static const signed char iv1[21] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 1, 0, 0, 0, 1 };

  double b_s_trans[21];
  double b_A[7];
  double smax;
  double c_s_trans[7];
  int k;
  double H_full[42];
  double c_A[49];
  double s;
  int kAcol;
  double H_full_tmp;
  double c[36];
  int j;
  int jj;
  double b_H_full[42];
  signed char p[6];
  int jp1j;
  signed char ipiv[6];
  int n;
  int ix;
  double b_x[36];
  double b_Ax[6];
  double K_full[42];

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
    jy = i0 << 2;
    A[7 * i0] = iv0[jy];
    A[1 + 7 * i0] = iv0[1 + jy];
    A[2 + 7 * i0] = iv0[2 + jy];
    A[3 + 7 * i0] = iv0[3 + jy];
  }

  for (i0 = 0; i0 < 3; i0++) {
    jy = i0 << 2;
    iy = 7 * (i0 + 4);
    A[iy] = -s_trans[jy] * 0.5 * t;
    A[1 + iy] = -s_trans[1 + jy] * 0.5 * t;
    A[2 + iy] = -s_trans[2 + jy] * 0.5 * t;
    A[3 + iy] = -s_trans[3 + jy] * 0.5 * t;
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
    iy = i0 << 2;
    b_s_trans[7 * i0] = s_trans[iy] * 0.5 * t;
    b_s_trans[1 + 7 * i0] = s_trans[1 + iy] * 0.5 * t;
    b_s_trans[2 + 7 * i0] = s_trans[2 + iy] * 0.5 * t;
    b_s_trans[3 + 7 * i0] = s_trans[3 + iy] * 0.5 * t;
    b_s_trans[7 * i0 + 4] = 0.0;
    b_s_trans[7 * i0 + 5] = 0.0;
    b_s_trans[7 * i0 + 6] = 0.0;
  }

  for (i0 = 0; i0 < 7; i0++) {
    b_A[i0] = 0.0;
    smax = 0.0;
    for (k = 0; k < 7; k++) {
      smax += A[i0 + 7 * k] * x[k];
    }

    b_A[i0] = smax;
    c_s_trans[i0] = 0.0;
    c_s_trans[i0] = (b_s_trans[i0] * Gx + b_s_trans[i0 + 7] * Gy) + b_s_trans[i0
      + 14] * Gz;
  }

  for (i0 = 0; i0 < 7; i0++) {
    x[i0] = b_A[i0] + c_s_trans[i0];
    for (k = 0; k < 7; k++) {
      jy = i0 + 7 * k;
      c_A[jy] = 0.0;
      smax = 0.0;
      for (kAcol = 0; kAcol < 7; kAcol++) {
        smax += A[i0 + 7 * kAcol] * P[kAcol + 7 * k];
      }

      c_A[jy] = smax;
    }
  }

  for (i0 = 0; i0 < 7; i0++) {
    for (k = 0; k < 7; k++) {
      smax = 0.0;
      for (kAcol = 0; kAcol < 7; kAcol++) {
        smax += c_A[i0 + 7 * kAcol] * A[k + 7 * kAcol];
      }

      kAcol = i0 + 7 * k;
      P[kAcol] = smax + Q[kAcol];
    }
  }

  smax = -2.0 * -x[2];
  H_full[0] = smax;
  H_full[6] = -2.0 * x[3];
  s = -2.0 * -x[0];
  H_full[12] = s;
  H_full[18] = -2.0 * x[1];
  H_full[1] = -2.0 * x[1];
  H_full[7] = -2.0 * x[0];
  H_full[13] = -2.0 * x[3];
  H_full[19] = -2.0 * x[2];
  H_full[2] = -2.0 * x[0];
  H_full_tmp = -2.0 * -x[1];
  H_full[8] = H_full_tmp;
  H_full[14] = smax;
  H_full[20] = -2.0 * x[3];
  H_full[3] = -2.0 * x[3];
  H_full[9] = -2.0 * x[2];
  H_full[15] = -2.0 * x[1];
  H_full[21] = -2.0 * x[0];
  H_full[4] = -2.0 * x[0];
  H_full[10] = H_full_tmp;
  H_full[16] = -2.0 * x[2];
  H_full[22] = -2.0 * -x[3];
  H_full[5] = H_full_tmp;
  H_full[11] = s;
  H_full[17] = -2.0 * x[3];
  H_full[23] = -2.0 * x[2];
  for (i0 = 0; i0 < 3; i0++) {
    iy = 6 * (i0 + 4);
    H_full[iy] = 0.0;
    H_full[iy + 3] = 0.0;
    H_full[1 + iy] = 0.0;
    H_full[iy + 4] = 0.0;
    H_full[2 + iy] = 0.0;
    H_full[iy + 5] = 0.0;
  }

  memset(&c[0], 0, 36U * sizeof(double));
  for (i0 = 0; i0 < 6; i0++) {
    for (k = 0; k < 7; k++) {
      iy = i0 + 6 * k;
      b_H_full[iy] = 0.0;
      smax = 0.0;
      for (kAcol = 0; kAcol < 7; kAcol++) {
        smax += H_full[i0 + 6 * kAcol] * P[kAcol + 7 * k];
      }

      b_H_full[iy] = smax;
    }

    for (k = 0; k < 6; k++) {
      smax = 0.0;
      for (kAcol = 0; kAcol < 7; kAcol++) {
        smax += b_H_full[i0 + 6 * kAcol] * H_full[k + 6 * kAcol];
      }

      iy = i0 + 6 * k;
      b_x[iy] = smax + R[iy];
    }

    ipiv[i0] = (signed char)(1 + i0);
  }

  for (j = 0; j < 5; j++) {
    kAcol = j * 7;
    jj = j * 7;
    jp1j = kAcol + 2;
    n = 6 - j;
    jy = 0;
    ix = kAcol;
    smax = fabs(b_x[kAcol]);
    for (k = 2; k <= n; k++) {
      ix++;
      s = fabs(b_x[ix]);
      if (s > smax) {
        jy = k - 1;
        smax = s;
      }
    }

    if (b_x[jj + jy] != 0.0) {
      if (jy != 0) {
        iy = j + jy;
        ipiv[j] = (signed char)(iy + 1);
        ix = j;
        for (k = 0; k < 6; k++) {
          smax = b_x[ix];
          b_x[ix] = b_x[iy];
          b_x[iy] = smax;
          ix += 6;
          iy += 6;
        }
      }

      i0 = (jj - j) + 6;
      for (n = jp1j; n <= i0; n++) {
        b_x[n - 1] /= b_x[jj];
      }
    }

    n = 4 - j;
    jy = kAcol + 6;
    iy = jj;
    for (kAcol = 0; kAcol <= n; kAcol++) {
      smax = b_x[jy];
      if (b_x[jy] != 0.0) {
        ix = jj + 1;
        i0 = iy + 8;
        k = (iy - j) + 12;
        for (jp1j = i0; jp1j <= k; jp1j++) {
          b_x[jp1j - 1] += b_x[ix] * -smax;
          ix++;
        }
      }

      jy += 6;
      iy += 6;
    }
  }

  for (i0 = 0; i0 < 6; i0++) {
    p[i0] = (signed char)(1 + i0);
  }

  for (k = 0; k < 5; k++) {
    if (ipiv[k] > 1 + k) {
      iy = ipiv[k] - 1;
      jy = p[iy];
      p[iy] = p[k];
      p[k] = (signed char)jy;
    }
  }

  for (k = 0; k < 6; k++) {
    jp1j = p[k] - 1;
    c[k + 6 * jp1j] = 1.0;
    for (j = k + 1; j < 7; j++) {
      if (c[(j + 6 * jp1j) - 1] != 0.0) {
        i0 = j + 1;
        for (n = i0; n < 7; n++) {
          iy = (n + 6 * jp1j) - 1;
          c[iy] -= c[(j + 6 * jp1j) - 1] * b_x[(n + 6 * (j - 1)) - 1];
        }
      }
    }
  }

  for (j = 0; j < 6; j++) {
    jy = 6 * j;
    for (k = 5; k >= 0; k--) {
      kAcol = 6 * k;
      i0 = k + jy;
      if (c[i0] != 0.0) {
        c[i0] /= b_x[k + kAcol];
        for (n = 0; n < k; n++) {
          jp1j = n + jy;
          c[jp1j] -= c[i0] * b_x[n + kAcol];
        }
      }
    }
  }

  for (i0 = 0; i0 < 7; i0++) {
    for (k = 0; k < 6; k++) {
      iy = i0 + 7 * k;
      b_H_full[iy] = 0.0;
      smax = 0.0;
      for (kAcol = 0; kAcol < 7; kAcol++) {
        smax += P[i0 + 7 * kAcol] * H_full[k + 6 * kAcol];
      }

      b_H_full[iy] = smax;
    }

    for (k = 0; k < 6; k++) {
      iy = i0 + 7 * k;
      K_full[iy] = 0.0;
      smax = 0.0;
      for (kAcol = 0; kAcol < 6; kAcol++) {
        smax += b_H_full[i0 + 7 * kAcol] * c[kAcol + 6 * k];
      }

      K_full[iy] = smax;
    }
  }

  /*        y; */
  /*      m = [ transpose(y) H*x]; */
  b_Ax[0] = Ax;
  b_Ax[1] = Ay;
  b_Ax[2] = Az;
  b_Ax[3] = Mx;
  b_Ax[4] = My;
  b_Ax[5] = Mz;
  for (i0 = 0; i0 < 6; i0++) {
    smax = 0.0;
    for (k = 0; k < 7; k++) {
      smax += H_full[i0 + 6 * k] * x[k];
    }

    b_Ax[i0] -= smax;
  }

  for (i0 = 0; i0 < 7; i0++) {
    smax = 0.0;
    for (k = 0; k < 6; k++) {
      smax += K_full[i0 + 7 * k] * b_Ax[k];
    }

    x[i0] += smax;
  }

  memset(&A[0], 0, 49U * sizeof(double));
  for (k = 0; k < 7; k++) {
    A[k + 7 * k] = 1.0;
  }

  for (i0 = 0; i0 < 7; i0++) {
    for (k = 0; k < 7; k++) {
      smax = 0.0;
      for (kAcol = 0; kAcol < 6; kAcol++) {
        smax += K_full[i0 + 7 * kAcol] * H_full[kAcol + 6 * k];
      }

      jy = i0 + 7 * k;
      c_A[jy] = A[jy] - smax;
    }
  }

  for (i0 = 0; i0 < 7; i0++) {
    for (k = 0; k < 7; k++) {
      jy = i0 + 7 * k;
      A[jy] = 0.0;
      smax = 0.0;
      for (kAcol = 0; kAcol < 7; kAcol++) {
        smax += c_A[i0 + 7 * kAcol] * P[kAcol + 7 * k];
      }

      A[jy] = smax;
    }
  }

  memcpy(&P[0], &A[0], 49U * sizeof(double));
  smax = sqrt(((x[0] * x[0] + x[1] * x[1]) + x[2] * x[2]) + x[3] * x[3]);
  x[0] /= smax;
  x[1] /= smax;
  x[2] /= smax;
  x[3] /= smax;
}

/*
 * File trailer for EKF.c
 *
 * [EOF]
 */
