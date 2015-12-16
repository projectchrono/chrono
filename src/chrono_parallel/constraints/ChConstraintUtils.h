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
// Authors: Hammad Mazhar
// =============================================================================
// Utility functions used by both broadphase algorithms
// =============================================================================

#pragma once

#include "chrono_parallel/ChDataManager.h"

namespace chrono {

template <typename T>
static void inline SetRow3(T& D, const int row, const int col, const real3& A) {
    D.set(row, col + 0, A.x);
    D.set(row, col + 1, A.y);
    D.set(row, col + 2, A.z);
}
template <typename T>
static void inline SetRow6(T& D, const int row, const int col, const real3& A, const real3& B) {
    D.set(row, col + 0, A.x);
    D.set(row, col + 1, A.y);
    D.set(row, col + 2, A.z);

    D.set(row, col + 3, B.x);
    D.set(row, col + 4, B.y);
    D.set(row, col + 5, B.z);
}
template <typename T>
static void inline SetCol3(T& D, const int row, const int col, const real3& A) {
    D.set(row + 0, col, A.x);
    D.set(row + 1, col, A.y);
    D.set(row + 2, col, A.z);
}
template <typename T>
static void inline SetCol6(T& D, const int row, const int col, const real3& A, const real3& B) {
    D.set(row + 0, col, A.x);
    D.set(row + 1, col, A.y);
    D.set(row + 2, col, A.z);

    D.set(row + 3, col, B.x);
    D.set(row + 4, col, B.y);
    D.set(row + 5, col, B.z);
}
CH_PARALLEL_API
static void Orthogonalize(real3& Vx, real3& Vy, real3& Vz) {
    real3 mVsingular = real3(0, 1, 0);
    Vz = Cross(Vx, mVsingular);
    real mzlen = Length(Vz);
    // was near singularity? change singularity reference vector!
    if (mzlen < real(0.0001)) {
        mVsingular = real3(1, 0, 0);
        Vz = Cross(Vx, mVsingular);
        mzlen = Length(Vz);
    }
    Vz = Vz / mzlen;
    Vy = Cross(Vz, Vx);
}
CH_PARALLEL_API
static void Compute_Jacobian(const quaternion& quat,
                             const real3& U,
                             const real3& V,
                             const real3& W,
                             const real3& point,
                             real3& T1,
                             real3& T2,
                             real3& T3) {
    quaternion quaternion_conjugate = ~quat;
    real3 sbar = Rotate(point, quaternion_conjugate);

    T1 = Cross(Rotate(U, quaternion_conjugate), sbar);
    T2 = Cross(Rotate(V, quaternion_conjugate), sbar);
    T3 = Cross(Rotate(W, quaternion_conjugate), sbar);
}
CH_PARALLEL_API
static void Compute_Jacobian_Rolling(const quaternion& quat,
                                     const real3& U,
                                     const real3& V,
                                     const real3& W,
                                     real3& T1,
                                     real3& T2,
                                     real3& T3) {
    quaternion quaternion_conjugate = ~quat;

    T1 = Rotate(U, quaternion_conjugate);
    T2 = Rotate(V, quaternion_conjugate);
    T3 = Rotate(W, quaternion_conjugate);
}
}
