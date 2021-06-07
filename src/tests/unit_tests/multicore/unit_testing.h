// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar, Radu Serban
// =============================================================================
//
// Chrono::Multicore unit testing common functions
// =============================================================================

#include <cfloat>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>
#include <algorithm>

#include "gtest/gtest.h"

#include "chrono/core/ChMatrix33.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector.h"

#include "chrono_multicore/math/matrix.h"
#include "chrono_multicore/math/other_types.h"

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
    tmp(0, 0) = a[0];
    tmp(1, 0) = a[1];
    tmp(2, 0) = a[2];

    tmp(0, 1) = a[4];
    tmp(1, 1) = a[5];
    tmp(2, 1) = a[6];

    tmp(0, 2) = a[8];
    tmp(1, 2) = a[9];
    tmp(2, 2) = a[10];

    return tmp;
}

Mat33 ToMat33(const ChMatrix33<real>& a) {
    return Mat33(a(0, 0), a(1, 0), a(2, 0), a(0, 1), a(1, 1), a(2, 1), a(0, 2), a(1, 2), a(2, 2));
}

// -----------------------------------------------------------------------------

void Assert_eq(const ChVector<>& a, const ChVector<>& b) {
    ASSERT_EQ(a.x(), b.x());
    ASSERT_EQ(a.y(), b.y());
    ASSERT_EQ(a.z(), b.z());
}

void Assert_eq(const ChQuaternion<>& a, const ChQuaternion<>& b) {
    ASSERT_EQ(a.e0(), b.e0());
    ASSERT_EQ(a.e1(), b.e1());
    ASSERT_EQ(a.e2(), b.e2());
    ASSERT_EQ(a.e3(), b.e3());
}

void Assert_eq(const real3& a, const real3& b) {
    ASSERT_EQ(a.x, b.x);
    ASSERT_EQ(a.y, b.y);
    ASSERT_EQ(a.z, b.z);
}

void Assert_eq(const real4& a, const real4& b) {
    ASSERT_EQ(a.w, b.w);
    ASSERT_EQ(a.x, b.x);
    ASSERT_EQ(a.y, b.y);
    ASSERT_EQ(a.z, b.z);
}

void Assert_eq(const uvec4& a, const uvec4& b) {
    ASSERT_EQ(a.x, b.x);
    ASSERT_EQ(a.y, b.y);
    ASSERT_EQ(a.z, b.z);
    ASSERT_EQ(a.w, b.w);
}

void Assert_eq(const Mat33& a, const Mat33& b) {
    Assert_eq(a.col(0), b.col(0));
    Assert_eq(a.col(1), b.col(1));
    Assert_eq(a.col(2), b.col(2));
}

// -----------------------------------------------------------------------------

void Assert_near(const ChVector<>& a, const ChVector<>& b, real COMPARE_EPS = C_EPSILON) {
    ASSERT_NEAR(a.x(), b.x(), COMPARE_EPS);
    ASSERT_NEAR(a.y(), b.y(), COMPARE_EPS);
    ASSERT_NEAR(a.z(), b.z(), COMPARE_EPS);
}

void Assert_near(const ChQuaternion<>& a, const ChQuaternion<>& b, real COMPARE_EPS = C_EPSILON) {
    ASSERT_NEAR(a.e0(), b.e0(), COMPARE_EPS);
    ASSERT_NEAR(a.e1(), b.e1(), COMPARE_EPS);
    ASSERT_NEAR(a.e2(), b.e2(), COMPARE_EPS);
    ASSERT_NEAR(a.e3(), b.e3(), COMPARE_EPS);
}

void Assert_near(const real3& a, const real3& b, real COMPARE_EPS = C_EPSILON) {
    ASSERT_NEAR(a.x, b.x, COMPARE_EPS);
    ASSERT_NEAR(a.y, b.y, COMPARE_EPS);
    ASSERT_NEAR(a.z, b.z, COMPARE_EPS);
}

void Assert_near(const real4& a, const real4& b, real COMPARE_EPS = C_EPSILON) {
    ASSERT_NEAR(a.x, b.x, COMPARE_EPS);
    ASSERT_NEAR(a.y, b.y, COMPARE_EPS);
    ASSERT_NEAR(a.z, b.z, COMPARE_EPS);
    ASSERT_NEAR(a.w, b.w, COMPARE_EPS);
}

void Assert_near(const quaternion& a, const quaternion& b, real COMPARE_EPS = C_EPSILON) {
    ASSERT_NEAR(a.w, b.w, COMPARE_EPS);
    ASSERT_NEAR(a.x, b.x, COMPARE_EPS);
    ASSERT_NEAR(a.y, b.y, COMPARE_EPS);
    ASSERT_NEAR(a.z, b.z, COMPARE_EPS);
}

void Assert_near(const Mat33& a, const Mat33& b, real COMPARE_EPS = C_EPSILON) {
    ASSERT_NEAR(a[0], b[0], COMPARE_EPS);
    ASSERT_NEAR(a[1], b[1], COMPARE_EPS);
    ASSERT_NEAR(a[2], b[2], COMPARE_EPS);
    ASSERT_NEAR(a[4], b[4], COMPARE_EPS);
    ASSERT_NEAR(a[5], b[5], COMPARE_EPS);
    ASSERT_NEAR(a[6], b[6], COMPARE_EPS);
    ASSERT_NEAR(a[8], b[8], COMPARE_EPS);
    ASSERT_NEAR(a[9], b[9], COMPARE_EPS);
    ASSERT_NEAR(a[10], b[10], COMPARE_EPS);
}

void Assert_near(const SymMat33& a, const Mat33& b, real COMPARE_EPS = C_EPSILON) {
    ASSERT_NEAR(a[0], b[0], COMPARE_EPS);   // x11
    ASSERT_NEAR(a[1], b[1], COMPARE_EPS);   // x21
    ASSERT_NEAR(a[2], b[2], COMPARE_EPS);   // x31
    ASSERT_NEAR(a[3], b[5], COMPARE_EPS);   // x22
    ASSERT_NEAR(a[4], b[6], COMPARE_EPS);   // x32
    ASSERT_NEAR(a[5], b[10], COMPARE_EPS);  // x33
}

void Assert_near(const SymMat22& a, const SymMat22& b, real COMPARE_EPS = C_EPSILON) {
    ASSERT_NEAR(a.x11, b.x11, COMPARE_EPS);
    ASSERT_NEAR(a.x21, b.x21, COMPARE_EPS);
    ASSERT_NEAR(a.x22, b.x22, COMPARE_EPS);
}

// -----------------------------------------------------------------------------

void Assert_near(const real a[], const real b[], int n, real COMPARE_EPS = C_EPSILON) {
    // Local copies (will be modified)
    std::vector<real> av(a, a + n);
    std::vector<real> bv(b, b + n);
    // Sort a and b in lexicographic order
    std::sort(av.begin(), av.end());
    std::sort(bv.begin(), bv.end());
    // Loop and compare
    for (int i = 0; i < n; i++)
        ASSERT_NEAR(av[i], bv[i], COMPARE_EPS);
}

void Assert_near(const std::vector<real>& a, const std::vector<real>& b, real COMPARE_EPS = C_EPSILON) {
    // Local copies (will be modified)
    auto av = a;
    auto bv = b;
    // Sort a and b in lexicographic order
    std::sort(av.begin(), av.end());
    std::sort(bv.begin(), bv.end());
    // Loop and compare
    for (int i = 0; i < av.size(); i++)
        ASSERT_NEAR(av[i], bv[i], COMPARE_EPS);
}

void Assert_near(const real3 a[], const real3 b[], int n, real COMPARE_EPS = C_EPSILON) {
    // Local copies (will be modified)
    std::vector<real3> av(a, a + n);
    std::vector<real3> bv(b, b + n);
    // Sort a and b in lexicographic order
    std::sort(av.begin(), av.end(), [](real3 u, real3 v) { return std::tie(u.x, u.y, u.z) < std::tie(v.x, v.y, v.z); });
    std::sort(bv.begin(), bv.end(), [](real3 u, real3 v) { return std::tie(u.x, u.y, u.z) < std::tie(v.x, v.y, v.z); });
    // Loop and compare
    for (int i = 0; i < n; i++)
        Assert_near(av[i], bv[i], COMPARE_EPS);
}

void Assert_near(const std::vector<real3>& a, const std::vector<real3>& b, real COMPARE_EPS = C_EPSILON) {
    // Local copies (will be modified)
    auto av = a;
    auto bv = b;
    // Sort a and b in lexicographic order
    std::sort(av.begin(), av.end(), [](real3 u, real3 v) { return std::tie(u.x, u.y, u.z) < std::tie(v.x, v.y, v.z); });
    std::sort(bv.begin(), bv.end(), [](real3 u, real3 v) { return std::tie(u.x, u.y, u.z) < std::tie(v.x, v.y, v.z); });
    // Loop and compare
    for (int i = 0; i < av.size(); i++)
        Assert_near(av[i], bv[i], COMPARE_EPS);
}
