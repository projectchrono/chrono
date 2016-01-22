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
static void inline AppendRow3(T& D, const int row, const int col, const real init) {
    // printf("%d [%d %d %d]\n", row, col + 0, col + 1, col + 2);
    D.append(row, col + 0, init);
    D.append(row, col + 1, init);
    D.append(row, col + 2, init);
}
template <typename T>
static void inline AppendRow3(T& D, const int row, const int col, const real3 init) {
    D.append(row, col + 0, init.x);
    D.append(row, col + 1, init.y);
    D.append(row, col + 2, init.z);
}
template <typename T>
static void inline AppendRow6(T& D, const int row, const int col, const real init) {
    // printf("%d [%d %d %d] [%d %d %d]\n", row, col + 0, col + 1, col + 2, col + 3, col + 4, col + 5);
    D.append(row, col + 0, init);
    D.append(row, col + 1, init);
    D.append(row, col + 2, init);

    D.append(row, col + 3, init);
    D.append(row, col + 4, init);
    D.append(row, col + 5, init);
}
//
template <typename T>
static void inline AppendRow3Weak(T& D, const int row, const int col, const real init) {
    D.weakAppend(row, col + 0, init);
    D.weakAppend(row, col + 1, init);
    D.weakAppend(row, col + 2, init);
}
template <typename T>
static void inline AppendRow6Weak(T& D, const int row, const int col, const real init) {
    D.weakAppend(row, col + 0, init);
    D.weakAppend(row, col + 1, init);
    D.weakAppend(row, col + 2, init);

    D.weakAppend(row, col + 3, init);
    D.weakAppend(row, col + 4, init);
    D.weakAppend(row, col + 5, init);
}
//
template <typename T>
static void inline SetRow3Check(T& D, const int row, const int col, const real3& A) {
    //    printf("%d [%d %d %d]\n", row, col + 0, col + 1, col + 2);
        if (D.find(row, col + 0) == D.end(row)) {
            printf("fail: %d %d\n", row, col + 0);
            exit(1);
        }
        if (D.find(row, col + 1) == D.end(row)) {
            printf("fail: %d %d\n", row, col + 1);
            exit(1);
        }
        if (D.find(row, col + 2) == D.end(row)) {
            printf("fail: %d %d\n", row, col + 2);
            exit(1);
        }

    if (A.x != 0.0) {
        D.set(row, col + 0, A.x);
    }
    if (A.y != 0.0) {
        D.set(row, col + 1, A.y);
    }
    if (A.z != 0.0) {
        D.set(row, col + 2, A.z);
    }
}
template <typename T>
static void inline SetRow3Weak(T& D, const int row, const int col, const real3& A) {
    // assert(D.find( row, col + 0)!=D.end(row));
    // assert(D.find( row, col + 1)!=D.end(row));
    // assert(D.find( row, col + 2)!=D.end(row));

    if (A.x != 0.0) {
        D.weakSet(row, col + 0, A.x);
    }
    if (A.y != 0.0) {
        D.weakSet(row, col + 1, A.y);
    }
    if (A.z != 0.0) {
        D.weakSet(row, col + 2, A.z);
    }
}
template <typename T>
static void inline SetRow6Check(T& D, const int row, const int col, const real3& A, const real3& B) {
    //    printf("%d [%d %d %d] [%d %d %d]\n", row, col + 0, col + 1, col + 2, col + 3, col + 4, col + 5);
        if (D.find(row, col + 0) == D.end(row)) {
            printf("fail: %d %d\n", row, col + 0);
            exit(1);
        }
        if (D.find(row, col + 1) == D.end(row)) {
            printf("fail: %d %d\n", row, col + 1);
            exit(1);
        }
        if (D.find(row, col + 2) == D.end(row)) {
            printf("fail: %d %d\n", row, col + 2);
            exit(1);
        }
        if (D.find(row, col + 3) == D.end(row)) {
            printf("fail: %d %d\n", row, col + 3);
            exit(1);
        }
        if (D.find(row, col + 4) == D.end(row)) {
            printf("fail: %d %d\n", row, col + 4);
            exit(1);
        }
        if (D.find(row, col + 5) == D.end(row)) {
            printf("fail: %d %d\n", row, col + 5);
            exit(1);
        }

    if (A.x != 0.0) {
        D.set(row, col + 0, A.x);
    }
    if (A.y != 0.0) {
        D.set(row, col + 1, A.y);
    }
    if (A.z != 0.0) {
        D.set(row, col + 2, A.z);
    }

    if (B.x != 0.0) {
        D.set(row, col + 3, B.x);
    }
    if (B.y != 0.0) {
        D.set(row, col + 4, B.y);
    }
    if (B.z != 0.0) {
        D.set(row, col + 5, B.z);
    }
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

#define NORMAL_J                \
    real3 U_A = Rotate(U, q_a); \
    T3 = Cross(U_A, sbar_a.v);  \
    real3 U_B = Rotate(U, q_b); \
    T6 = Cross(U_B, sbar_b.v);

#define SLIDING_J               \
    real3 V_A = Rotate(V, q_a); \
    real3 W_A = Rotate(W, q_a); \
    T4 = Cross(V_A, sbar_a.v);  \
    T5 = Cross(W_A, sbar_a.v);  \
                                \
    real3 V_B = Rotate(V, q_b); \
    real3 W_B = Rotate(W, q_b); \
    T7 = Cross(V_B, sbar_b.v);  \
    T8 = Cross(W_B, sbar_b.v);

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
