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

#include <cstdio>
#include <vector>
#include <cmath>
#include <iostream>
#include <cfloat>

#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChMatrixDynamic.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono_parallel/math/matrix.h"
#include "chrono_parallel/math/other_types.h"

using namespace chrono;

real3 ToReal3(const ChVector<real>& a) {
    return real3(a.x(), a.y(), a.z());
}

ChVector<real> ToChVector(const real3& a) {
    return ChVector<real>(a.x, a.y, a.z);
}

ChQuaternion<real> ToChQuaternion(const quaternion& a) {
    return ChQuaternion<real>(a.w, a.x, a.y, a.z);
}

quaternion ToQuaternion(const ChQuaternion<real>& a) {
    return quaternion(a.e0(), a.e1(), a.e2(), a.e3());
}

ChMatrix33<real> ToChMatrix33(const Mat33& a) {
    ChMatrix33<real> tmp;
    tmp.SetElement(0, 0, a[0]);
    tmp.SetElement(1, 0, a[1]);
    tmp.SetElement(2, 0, a[2]);

    tmp.SetElement(0, 1, a[4]);
    tmp.SetElement(1, 1, a[5]);
    tmp.SetElement(2, 1, a[6]);

    tmp.SetElement(0, 2, a[8]);
    tmp.SetElement(1, 2, a[9]);
    tmp.SetElement(2, 2, a[10]);

    return tmp;
}

Mat33 ToMat33(const ChMatrix33<real>& a) {
    return Mat33(a(0, 0), a(1, 0), a(2, 0), a(0, 1), a(1, 1), a(2, 1), a(0, 2), a(1, 2), a(2, 2));
}

void StrictEqual(const int& x, const int& y) {
    if (x != y) {
        std::cout << x << " does not equal " << y << std::endl;
        exit(1);
    }
}

void StrictEqual(const uint& x, const uint& y) {
    if (x != y) {
        std::cout << x << " does not equal " << y << std::endl;
        exit(1);
    }
}

void StrictEqual(const real& x, const real& y) {
    if (x != y) {
        std::cout << x << " does not equal " << y << std::endl;
        exit(1);
    }
}

void StrictEqual(const real3& a, const real3& b) {
    StrictEqual(a.x, b.x);
    StrictEqual(a.y, b.y);
    StrictEqual(a.z, b.z);
}

void StrictEqual(const real4& a, const real4& b) {
    StrictEqual(a.w, b.w);
    StrictEqual(a.x, b.x);
    StrictEqual(a.y, b.y);
    StrictEqual(a.z, b.z);
}

void StrictEqual(const Mat33& a, const Mat33& b) {
    StrictEqual(a.col(0), b.col(0));
    StrictEqual(a.col(1), b.col(1));
    StrictEqual(a.col(2), b.col(2));
}

void StrictEqual(const uvec4& a, const uvec4& b) {
    StrictEqual(a.x, b.x);
    StrictEqual(a.y, b.y);
    StrictEqual(a.z, b.z);
    StrictEqual(a.w, b.w);
}

void WeakEqual(const real& x, const real& y, real COMPARE_EPS = C_EPSILON) {
    if (Abs(x - y) > COMPARE_EPS) {
        printf("%f does not equal %f %.20e\n", x, y, Abs(x - y));
        exit(1);
    }
}

void WeakEqual(const real3& a, const real3& b, real COMPARE_EPS = C_EPSILON) {
    WeakEqual(a.x, b.x, COMPARE_EPS);
    WeakEqual(a.y, b.y, COMPARE_EPS);
    WeakEqual(a.z, b.z, COMPARE_EPS);
}

void WeakEqual(const real4& a, const real4& b, real COMPARE_EPS = C_EPSILON) {
    WeakEqual(a.x, b.x, COMPARE_EPS);
    WeakEqual(a.y, b.y, COMPARE_EPS);
    WeakEqual(a.z, b.z, COMPARE_EPS);
    WeakEqual(a.w, b.w, COMPARE_EPS);
}

void WeakEqual(const quaternion& a, const quaternion& b, real COMPARE_EPS = C_EPSILON) {
    WeakEqual(a.w, b.w, COMPARE_EPS);
    WeakEqual(a.x, b.x, COMPARE_EPS);
    WeakEqual(a.y, b.y, COMPARE_EPS);
    WeakEqual(a.z, b.z, COMPARE_EPS);
}

void WeakEqual(const Mat33& a, const Mat33& b, real COMPARE_EPS = C_EPSILON) {
    WeakEqual(a[0], b[0], COMPARE_EPS);
    WeakEqual(a[1], b[1], COMPARE_EPS);
    WeakEqual(a[2], b[2], COMPARE_EPS);
    WeakEqual(a[4], b[4], COMPARE_EPS);
    WeakEqual(a[5], b[5], COMPARE_EPS);
    WeakEqual(a[6], b[6], COMPARE_EPS);
    WeakEqual(a[8], b[8], COMPARE_EPS);
    WeakEqual(a[9], b[9], COMPARE_EPS);
    WeakEqual(a[10], b[10], COMPARE_EPS);
}
void WeakEqual(const SymMat33& a, const Mat33& b, real COMPARE_EPS = C_EPSILON) {
    WeakEqual(a[0], b[0], COMPARE_EPS);   // x11
    WeakEqual(a[1], b[1], COMPARE_EPS);   // x21
    WeakEqual(a[2], b[2], COMPARE_EPS);   // x31
    WeakEqual(a[3], b[5], COMPARE_EPS);   // x22
    WeakEqual(a[4], b[6], COMPARE_EPS);   // x32
    WeakEqual(a[5], b[10], COMPARE_EPS);  // x33
}
void WeakEqual(const SymMat22& a, const SymMat22& b, real COMPARE_EPS = C_EPSILON) {
    WeakEqual(a.x11, b.x11, COMPARE_EPS);
    WeakEqual(a.x21, b.x21, COMPARE_EPS);
    WeakEqual(a.x22, b.x22, COMPARE_EPS);
}
void OutputRowMatrix(const ChMatrixDynamic<real>& x) {
    for (int ic = 0; ic < x.GetRows(); ic++) {
        std::cout << x(ic, 0) << std::endl;
    }
}
void PrintMat33(const Mat33& A) {
    printf("[%f %f %f]\n[%f %f %f]\n[%f %f %f]\n[%f %f %f]\n", A[0], A[4], A[8], A[1], A[5], A[9], A[2], A[6], A[10],
           A[3], A[7], A[11]);
}
