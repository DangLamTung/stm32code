/*
 * File: inv.c
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
 * Arguments    : const double x[81]
 *                double y[81]
 * Return Type  : void
 */
void inv(const double x[81], double y[81])
{
  int i3;
  double b_x[81];
  int j;
  signed char ipiv[9];
  int b;
  int jj;
  int k;
  signed char p[9];
  int jp1j;
  int n;
  int jy;
  int iy;
  int ix;
  double smax;
  double s;
  int i;
  for (i3 = 0; i3 < 81; i3++) {
    y[i3] = 0.0;
    b_x[i3] = x[i3];
  }

  for (i3 = 0; i3 < 9; i3++) {
    ipiv[i3] = (signed char)(1 + i3);
  }

  for (j = 0; j < 8; j++) {
    b = j * 10;
    jj = j * 10;
    jp1j = b + 2;
    n = 9 - j;
    jy = 0;
    ix = b;
    smax = fabs(b_x[b]);
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
        for (k = 0; k < 9; k++) {
          smax = b_x[ix];
          b_x[ix] = b_x[iy];
          b_x[iy] = smax;
          ix += 9;
          iy += 9;
        }
      }

      i3 = (jj - j) + 9;
      for (i = jp1j; i <= i3; i++) {
        b_x[i - 1] /= b_x[jj];
      }
    }

    n = 7 - j;
    jy = b + 9;
    iy = jj;
    for (b = 0; b <= n; b++) {
      smax = b_x[jy];
      if (b_x[jy] != 0.0) {
        ix = jj + 1;
        i3 = iy + 11;
        jp1j = (iy - j) + 18;
        for (i = i3; i <= jp1j; i++) {
          b_x[i - 1] += b_x[ix] * -smax;
          ix++;
        }
      }

      jy += 9;
      iy += 9;
    }
  }

  for (i3 = 0; i3 < 9; i3++) {
    p[i3] = (signed char)(1 + i3);
  }

  for (k = 0; k < 8; k++) {
    if (ipiv[k] > 1 + k) {
      iy = ipiv[k] - 1;
      jy = p[iy];
      p[iy] = p[k];
      p[k] = (signed char)jy;
    }
  }

  for (k = 0; k < 9; k++) {
    iy = p[k] - 1;
    y[k + 9 * iy] = 1.0;
    for (j = k + 1; j < 10; j++) {
      if (y[(j + 9 * iy) - 1] != 0.0) {
        i3 = j + 1;
        for (i = i3; i < 10; i++) {
          b = (i + 9 * iy) - 1;
          y[b] -= y[(j + 9 * iy) - 1] * b_x[(i + 9 * (j - 1)) - 1];
        }
      }
    }
  }

  for (j = 0; j < 9; j++) {
    jy = 9 * j;
    for (k = 8; k >= 0; k--) {
      iy = 9 * k;
      i3 = k + jy;
      if (y[i3] != 0.0) {
        y[i3] /= b_x[k + iy];
        for (i = 0; i < k; i++) {
          b = i + jy;
          y[b] -= y[i3] * b_x[i + iy];
        }
      }
    }
  }
}

/*
 * File trailer for inv.c
 *
 * [EOF]
 */
