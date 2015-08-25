// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Terence R.F.Nonweiler
// =============================================================================
// CACM Algorithm 326 Roots of low order polynomials
// CACM  (Apr 1968) p269
// Translated into c and programmed by M.Dow
// ANUSF, Australian National University, Canberra, Australia
// m.dow@anu.edu.au
// Suite of procedures for finding the (complex) roots of the
// quadratic, cubic or quartic polynomials by explicit algebraic methods.
// Each Returns
// x=r[1][k] + i r[2][k]  k=1,...,n, where n={2,3,4}
// as roots of
// sum_{k=0:n} p[k] x^(n-k) =0
// Assume p[0]<>0 (overflows otherwise)
// =============================================================================

#ifndef QUARTIC_H
#define QUARTIC_H

#include "real.h"
namespace chrono {
int QUADROOTS(real p[5], real r[3][5]) {
  /*
  Array r[3][5]  p[5]
  Roots of poly p[0] x^2 + p[1] x+p[2]=0
  x=r[1][k] + i r[2][k]  k=1,2
  */
  real b, c, d;
  b = -p[1] / p[0] / 2;
  c = p[2] / p[0];
  d = b * b - c;
  if (d > 0) {
    if (b > 0)
      b = r[1][2] = sqrt(d) + b;
    else
      b = r[1][2] = -sqrt(d) + b;
    r[1][1] = c / b;
    r[2][1] = r[2][2] = 0;
  } else {
    d = r[2][1] = sqrt(-d);
    r[2][2] = -d;
    r[1][1] = r[1][2] = b;
  }
  return (0);
}
int CUBICROOTS(real p[5], real r[3][5]) {
  /*
  Array r[3][5]  p[5]
  Roots of poly p[0] x^3 + p[1] x^2...+p[3]=0
  x=r[1][k] + i r[2][k]  k=1,...,3
  Assumes 0<arctan(x)<pi/2 for x>0
  */

  real s, t, b, c, d;
  int k;
  if (p[0] != 1)
    for (k = 1; k < 4; k++)
      p[k] = p[k] / p[0];
  p[0] = 1;
  s = p[1] / 3.0;
  t = s * p[1];
  b = 0.5 * (s * (t / 1.5 - p[2]) + p[3]);
  t = (t - p[2]) / 3.0;
  c = t * t * t;
  d = b * b - c;
  if (d >= 0) {
    d = pow((sqrt(d) + fabs(b)), 1.0 / 3.0);
    // printf("d=%f\n",d);
    if (d != 0) {
      if (b > 0)
        b = -d;
      else
        b = d;
      c = t / b;
    }
    d = r[2][2] = sqrt(0.75) * (b - c);
    b = b + c;
    c = r[1][2] = -0.5 * b - s;
    if ((b > 0 && s <= 0) || (b < 0 && s > 0)) {
      r[1][1] = c;
      r[2][1] = -d;
      r[1][3] = b - s;
      r[2][3] = 0;
    } else {
      r[1][1] = b - s;
      r[2][1] = 0;
      r[1][3] = c;
      r[2][3] = -d;
    }
  } /* end 2 equal or complex roots */
  else {
    if (b == 0)
      d = atan(1.0) / 1.5;
    else
      d = atan(sqrt(-d) / fabs(b)) / 3.0;
    if (b < 0)
      b = sqrt(t) * 2.0;
    else
      b = -2.0 * sqrt(t);
    c = cos(d) * b;
    t = -sqrt(0.75) * sin(d) * b - 0.5 * c;
    d = -t - c - s;
    c = c - s;
    t = t - s;
    if (fabs(c) > fabs(t))
      r[1][3] = c;
    else {
      r[1][3] = t;
      t = c;
    }
    if (fabs(d) > fabs(t))
      r[1][2] = d;
    else {
      r[1][2] = t;
      t = d;
    }
    r[1][1] = t;
    for (k = 1; k < 4; k++)
      r[2][k] = 0;
  }
  return (0);
}
int BIQUADROOTS(real p[5], real r[3][5])
/* add _ if calling from fortran */
/*
Array r[3][5]  p[5]
Roots of poly p[0] x^4 + p[1] x^3...+p[4]=0
x=r[1][k] + i r[2][k]  k=1,...,4
*/

{
  real a, b, c, d, e;
  int k, j;
  if (p[0] != 1.0) {
    for (k = 1; k < 5; k++)
      p[k] = p[k] / p[0];
    p[0] = 1;
  }
  e = 0.25 * p[1];
  b = 2 * e;
  c = b * b;
  d = 0.75 * c;
  b = p[3] + b * (c - p[2]);
  a = p[2] - d;
  c = p[4] + e * (e * a - p[3]);
  a = a - d;
  p[1] = 0.5 * a;
  p[2] = (p[1] * p[1] - c) * 0.25;
  p[3] = b * b / (-64.0);
  if (p[3] < -1e-6) {
    CUBICROOTS(p, r);
    for (k = 1; k < 4; k++) {
      if (r[2][k] == 0 && r[1][k] > 0) {
        d = r[1][k] * 4;
        a = a + d;
        if (a >= 0 && b >= 0)
          p[1] = sqrt(d);
        else if (a <= 0 && b <= 0)
          p[1] = sqrt(d);
        else
          p[1] = -sqrt(d);
        b = 0.5 * (a + b / p[1]);
        goto QUAD;
      }
    }
  }
  if (p[2] < 0) {
    b = sqrt(c);
    d = b + b - a;
    p[1] = 0;
    if (d > 0)
      p[1] = sqrt(d);
  } else {
    if (p[1] > 0)
      b = sqrt(p[2]) * 2.0 + p[1];
    else
      b = -sqrt(p[2]) * 2.0 + p[1];
    if (b != 0)
      p[1] = 0;
    else {
      for (k = 1; k < 5; k++) {
        r[1][k] = -e;
        r[2][k] = 0;
      }
      goto END;
    }
  }

QUAD:
  p[2] = c / b;
  QUADROOTS(p, r);
  for (k = 1; k < 3; k++)
    for (j = 1; j < 3; j++)
      r[j][k + 2] = r[j][k];
  p[1] = -p[1];
  p[2] = b;
  QUADROOTS(p, r);
  for (k = 1; k < 5; k++)
    r[1][k] = r[1][k] - e;
END:
  ;
  return (0);
}
}
#endif
