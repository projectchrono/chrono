// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
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
// Description: definition of some convenience functions for math operations
// =============================================================================

#pragma once

#include "chrono/core/ChVector3.h"
#include "chrono/core/ChVector2.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChMatrix33.h"

#include "chrono/multicore_math/real.h"
#include "chrono/multicore_math/real2.h"
#include "chrono/multicore_math/real3.h"
#include "chrono/multicore_math/real4.h"
#include "chrono/multicore_math/matrix.h"
#include "chrono/multicore_math/other_types.h"

namespace chrono {

/// @addtogroup chrono_mc_math
/// @{

/// Computes the nearest power of two to the given value and returns it
static inline uint nearest_pow(const uint& num) {
    uint n = num > 0 ? num - 1 : 0;
    n |= n >> 1;
    n |= n >> 2;
    n |= n >> 4;
    n |= n >> 8;
    n |= n >> 16;
    n++;
    return n;
}

/// Given a frame with origin 'p' and orientation 'q', transform the
/// position vector 'rl' expressed in the local frame into the parent
/// frame:  rp = p + A * rl
static inline real3 TransformLocalToParent(const real3& p, const quaternion& q, const real3& rl) {
    return p + Rotate(rl, q);
}

/// Given a frame with origin 'p' and orientation 'q', transform the
/// position vector 'rp' expressed in the parent frame into the local
/// frame:  rl = A^T * (rp - p)
static inline real3 TransformParentToLocal(const real3& p, const quaternion& q, const real3& rp) {
    return RotateT(rp - p, q);
}

class real3_int {
  public:
    real3_int() {}
    real3_int(real3 a, int b) : v(a), i(b) {}

    real3 v;
    int i;
};

// -----------------------------------------------------------------------------

static ChVector3<real> ToChVector(const real3& v) {
    return ChVector3<real>(v.x, v.y, v.z);
}

static real3 FromChVector(const ChVector3<real>& v) {
    return real3(v.x(), v.y(), v.z());
}

static ChVector2<real> ToChVector(const real2& v) {
    return ChVector2<real>(v.x, v.y);
}

static real2 FromChVector(const ChVector2<real>& v) {
    return real2(v.x(), v.y());
}

static ChQuaternion<real> ToChQuaternion(const quaternion& q) {
    return ChQuaternion<real>(q.w, q.x, q.y, q.z);
}

static quaternion FromChQuaternion(const ChQuaternion<real>& q) {
    return quaternion(q.e0(), q.e1(), q.e2(), q.e3());
}

static ChMatrix33<real> ToChMatrix33(const Mat33& a) {
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

static Mat33 FromChMatrix33(const ChMatrix33<real>& a) {
    return Mat33(a(0, 0), a(1, 0), a(2, 0), a(0, 1), a(1, 1), a(2, 1), a(0, 2), a(1, 2), a(2, 2));
}

/// @} chrono_mc_math

}  // end namespace chrono
