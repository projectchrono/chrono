// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
// Constraint utility functions
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
    // printf("Append %d [%d %d %d]\n", row, col + 0, col + 1, col + 2);
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
void Orthogonalize(real3& Vx, real3& Vy, real3& Vz);

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
void Compute_Jacobian(const quaternion& quat,
                      const real3& U,
                      const real3& V,
                      const real3& W,
                      const real3& point,
                      real3& T1,
                      real3& T2,
                      real3& T3);

CH_PARALLEL_API
void Compute_Jacobian_Rolling(const quaternion& quat,
                              const real3& U,
                              const real3& V,
                              const real3& W,
                              real3& T1,
                              real3& T2,
                              real3& T3);

#define Loop_Over_Rigid_Neighbors(X)                    \
    for (int p = 0; p < (signed)num_fluid_bodies; p++) {\
        int start = contact_counts[p];                  \
        int end = contact_counts[p + 1];                \
        for (int index = start; index < end; index++) { \
            int i = index - start;                      \
            X                                           \
        }                                               \
    }

#define Loop_Over_Fluid_Neighbors(X)                                                             \
    for (int body_a = 0; body_a < (signed)num_fluid_bodies; body_a++) {                                  \
        real3 pos_p = sorted_pos[body_a];                                                        \
        for (int i = 0; i < data_manager->host_data.c_counts_3dof_3dof[body_a]; i++) {           \
            int body_b = data_manager->host_data.neighbor_3dof_3dof[body_a * max_neighbors + i]; \
            if (body_a == body_b) {                                                              \
                continue;                                                                        \
            }                                                                                    \
            if (body_a > body_b) {                                                               \
                continue;                                                                        \
            }                                                                                    \
            real3 xij = pos_p - sorted_pos[body_b];                                              \
            X;                                                                                   \
            index++;                                                                             \
        }                                                                                        \
    }

CH_PARALLEL_API
bool Cone_generalized_rigid(real& gamma_n, real& gamma_u, real& gamma_v, const real& mu);

CH_PARALLEL_API
void AppendRigidFluidBoundary(const real contact_mu,
                              const uint num_fluid_bodies,
                              const uint body_offset,
                              const uint start_boundary,
                              ChParallelDataManager* data_manager);

CH_PARALLEL_API
void ProjectRigidFluidBoundary(const real contact_mu,
                               const real contact_cohesion,
                               const uint num_fluid_bodies,
                               const uint start_boundary,
                               real* gamma,
                               ChParallelDataManager* data_manager);

CH_PARALLEL_API
void ComplianceRigidFluidBoundary(const real contact_mu,
                                  const real contact_compliance,
                                  const real alpha,
                                  const uint start_boundary,
                                  ChParallelDataManager* data_manager);

CH_PARALLEL_API
void CorrectionRigidFluidBoundary(const real contact_mu,
                                  const real contact_cohesion,
                                  const real alpha,
                                  const real contact_recovery_speed,
                                  const uint num_fluid_bodies,
                                  const uint start_boundary,
                                  ChParallelDataManager* data_manager);

CH_PARALLEL_API
void BuildRigidFluidBoundary(const real contact_mu,
                             const uint num_fluid_bodies,
                             const uint body_offset,
                             const uint start_boundary,
                             ChParallelDataManager* data_manager);
}
