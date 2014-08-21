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
// Authors: Hammad Mazhar
// =============================================================================
//
// ChronoParallel unit testing common functions
// =============================================================================

#include <stdio.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <float.h>
#include <chrono_parallel/math/ChParallelMath.h>
#include <core/ChVector.h>
#include <core/ChQuaternion.h>
#include <core/ChMatrix.h>
#include <core/ChMatrix33.h>

using namespace chrono;
real3 ToReal3(
      const ChVector<> & a) {
   return real3(a.x, a.y, a.z);

}

ChVector<> ToChVector(
      const real3 & a) {
   return ChVector<>(a.x, a.y, a.z);

}

ChQuaternion<> ToChQuaternion(
      const real4 & a) {
   return ChQuaternion<>(a.w, a.x, a.y, a.z);

}

real4 ToReal4(
      const ChQuaternion<> & a) {
   return real4(a.e0, a.e1, a.e2, a.e3);

}

ChMatrix33<> ToChMatrix33(
      const M33 & a) {

   ChMatrix33<> tmp;
   tmp.PasteVector(ToChVector(a.U), 0, 0);
   tmp.PasteVector(ToChVector(a.V), 0, 1);
   tmp.PasteVector(ToChVector(a.W), 0, 2);

   return tmp;

}

M33 ToM33(
      const ChMatrix33<> & a) {

   M33 tmp;
   tmp.U = ToReal3(a.ClipVector(0, 0));
   tmp.V = ToReal3(a.ClipVector(0, 1));
   tmp.W = ToReal3(a.ClipVector(0, 2));

   return tmp;

}

void StrictEqual(
      const int & x,
      const int & y) {
  if (x != y) {
      std::cout << x << " does not equal " << y << std::endl;
      exit(1);
  }
}

void StrictEqual(
      const real & x,
      const real & y) {
   if (x != y) {
      std::cout << x << " does not equal " << y << std::endl;
      exit(1);
   }
}

void StrictEqual(
      const real3 & a,
      const real3 & b) {
   StrictEqual(a.x, b.x);
   StrictEqual(a.y, b.y);
   StrictEqual(a.z, b.z);
}

void StrictEqual(
      const real4 & a,
      const real4 & b) {
   StrictEqual(a.w, b.w);
   StrictEqual(a.x, b.x);
   StrictEqual(a.y, b.y);
   StrictEqual(a.z, b.z);
}

void StrictEqual(
      const M33 & a,
      const M33 & b) {
   StrictEqual(a.U, b.U);
   StrictEqual(a.V, b.V);
   StrictEqual(a.W, b.W);
}

void WeakEqual(
      const real & x,
      const real & y,
      float COMPARE_EPS = FLT_EPSILON * 5) {
   if (fabs(x - y) > COMPARE_EPS) {
      std::cout << x << " does not equal " << y << " " << fabs(x - y) << std::endl;
      exit(1);
   }
}

void WeakEqual(
      const real3 & a,
      const real3 & b,
      float COMPARE_EPS = FLT_EPSILON * 5) {
   WeakEqual(a.x, b.x, COMPARE_EPS);
   WeakEqual(a.y, b.y, COMPARE_EPS);
   WeakEqual(a.z, b.z, COMPARE_EPS);
}

void WeakEqual(
      const real4 & a,
      const real4 & b,
      float COMPARE_EPS = FLT_EPSILON * 5) {
   WeakEqual(a.w, b.w, COMPARE_EPS);
   WeakEqual(a.x, b.x, COMPARE_EPS);
   WeakEqual(a.y, b.y, COMPARE_EPS);
   WeakEqual(a.z, b.z, COMPARE_EPS);
}

void WeakEqual(
      const M33 & a,
      const M33 & b,
      float COMPARE_EPS = FLT_EPSILON * 5) {
   WeakEqual(a.U, b.U, COMPARE_EPS);
   WeakEqual(a.V, b.V, COMPARE_EPS);
   WeakEqual(a.W, b.W, COMPARE_EPS);
}
