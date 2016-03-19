// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
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
// Description: MPM and FLIP utility functions
// =============================================================================

#pragma once
#include "chrono_parallel/math/real3.h"
#include "chrono_parallel/math/other_types.h"
#include "chrono_parallel/math/svd.h"
#define one_third 1. / 3
#define two_thirds 2. / 3
#define three_halves 3. / 2
#define four_thirds 4. / 3
#define three_fourths 3. / 4
#define five_thirds 5. / 3
#define one_sixth 1. / 6
#define one_ninth 1. / 9
#define one_twelfth 1. / 12
#define one_twenty_fourth 1. / 24
#define one_twenty_seventh 1. / 27
#define one_sixtieth 1. / 60
#define thirteen_over_twelve 13. / 12
#define root_two Sqrt(2.)
#define root_three Sqrt(3.)
#define root_six Sqrt(6.)
#define root_two_thirds Sqrt(2. / 3)
#define one_over_root_two 1. / Sqrt(2.)
#define one_over_root_three 1. / Sqrt(3.)

namespace chrono {
// Interpolation Functions

CUDA_HOST_DEVICE static real N_Tri(const real x) {
    if (Abs(x) < real(1.0)) {
        return 1.0 - Abs(x);
    }
    return real(0.0);
}
CUDA_HOST_DEVICE static real N_Tri(const real3& X, real inv_grid_dx) {
    return N_Tri(X.x * inv_grid_dx) * N_Tri(X.y * inv_grid_dx) * N_Tri(X.z * inv_grid_dx);
}

CUDA_HOST_DEVICE static real N(const real x) {
    if (Abs(x) < real(1.0)) {
        return real(0.5) * Cube(Abs(x)) - Sqr(x) + two_thirds;
    } else if (Abs(x) < real(2.0)) {
        return -one_sixth * Cube(Abs(x)) + Sqr(x) - real(2.0) * Abs(x) + four_thirds;
    }
    return real(0.0);
}

CUDA_HOST_DEVICE static real N(const real3& X, real inv_grid_dx) {
    return N(X.x * inv_grid_dx) * N(X.y * inv_grid_dx) * N(X.z * inv_grid_dx);
}

CUDA_HOST_DEVICE static real N_tight(const real x) {
    if (Abs(x) >= real(0.0) && Abs(x) < real(.5)) {
        return -Sqr(x) + three_fourths;
    } else if (Abs(x) >= real(1.0) && Abs(x) < real(three_halves)) {
        return .5 * Sqr(x) - real(three_halves) * Abs(x) + (9.0 / 8.0);
    }
    return real(0.0);
}

CUDA_HOST_DEVICE static real N_tight(const real3& X, real inv_grid_dx) {
    return N_tight(X.x * inv_grid_dx) * N_tight(X.y * inv_grid_dx) * N_tight(X.z * inv_grid_dx);
}

CUDA_HOST_DEVICE static real dN(const real x) {
    if (Abs(x) < real(1.0)) {
        return real(1.5) * Sign(x) * Sqr(x) - real(2.0) * x;
    } else if (Abs(x) < real(2.0)) {
        return -real(0.5) * Sign(x) * Sqr(x) + real(2.0) * x - real(2.0) * Sign(x);
    }
    return real(0.0);
}

CUDA_HOST_DEVICE static real3 dN(const real3& X, real inv_grid_dx) {
    real3 val = real3(0);
    real3 T = X * inv_grid_dx;
    val.x = dN(T.x) * inv_grid_dx * N(T.y) * N(T.z);
    val.y = N(T.x) * dN(T.y) * inv_grid_dx * N(T.z);
    val.z = N(T.x) * N(T.y) * dN(T.z) * inv_grid_dx;
    return val;
    //
    //    for (int axis = 0; axis < 3; ++axis) {
    //        val[axis] = 1.;
    //        for (int other_axis = 0; other_axis < 3; ++other_axis) {
    //            if (other_axis == axis)
    //                val[axis] *= dN(X[axis] * inv_grid_dx) * inv_grid_dx;
    //            else
    //                val[axis] *= N(X[other_axis] * inv_grid_dx);
    //        }
    //    }
    // return val;
}

CUDA_HOST_DEVICE static inline int GridCoord(real x, real inv_bin_edge, real minimum) {
    real l = x - minimum;
    int c = Round(l * inv_bin_edge);
    return c;
}

CUDA_HOST_DEVICE static inline int GridHash(int x, int y, int z, const vec3& bins_per_axis) {
    return ((z * bins_per_axis.y) * bins_per_axis.x) + (y * bins_per_axis.x) + x;
}
CUDA_HOST_DEVICE static inline int GridHash(const int x,
                                            const int y,
                                            const int z,
                                            const int bins_per_axisx,
                                            const int bins_per_axisy,
                                            const int bins_per_axisz) {
    return ((z * bins_per_axisy) * bins_per_axisx) + (y * bins_per_axisx) + x;
}
CUDA_HOST_DEVICE static inline vec3 GridDecode(int hash, const vec3& bins_per_axis) {
    vec3 decoded_hash;
    decoded_hash.x = hash % (bins_per_axis.x * bins_per_axis.y) % bins_per_axis.x;
    decoded_hash.y = (hash % (bins_per_axis.x * bins_per_axis.y)) / bins_per_axis.x;
    decoded_hash.z = hash / (bins_per_axis.x * bins_per_axis.y);
    return decoded_hash;
}

CUDA_HOST_DEVICE static inline real3 NodeLocation(int i, int j, int k, real bin_edge, real3 min_bounding_point) {
    real3 node_location;
    node_location.x = i * bin_edge + min_bounding_point.x;
    node_location.y = j * bin_edge + min_bounding_point.y;
    node_location.z = k * bin_edge + min_bounding_point.z;
    return node_location;
}

#define LOOPOVERNODESY(X, Y)                                                                       \
    const int cx = GridCoord(xi.x, inv_bin_edge, min_bounding_point.x);                            \
    const int cy = GridCoord(xi.y, inv_bin_edge, min_bounding_point.y);                            \
    const int cz = GridCoord(xi.z, inv_bin_edge, min_bounding_point.z);                            \
                                                                                                   \
    for (int i = cx - Y; i <= cx + Y; ++i) {                                                       \
        for (int j = cy - Y; j <= cy + Y; ++j) {                                                   \
            for (int k = cz - Y; k <= cz + Y; ++k) {                                               \
                const int current_node = GridHash(i, j, k, bins_per_axis);                         \
                real3 current_node_location = NodeLocation(i, j, k, bin_edge, min_bounding_point); \
                X                                                                                  \
            }                                                                                      \
        }                                                                                          \
    }

#define LOOPOVERNODES(X)                                                                           \
    const int cx = GridCoord(xi.x, inv_bin_edge, min_bounding_point.x);                            \
    const int cy = GridCoord(xi.y, inv_bin_edge, min_bounding_point.y);                            \
    const int cz = GridCoord(xi.z, inv_bin_edge, min_bounding_point.z);                            \
                                                                                                   \
    for (int i = cx - 2; i <= cx + 2; ++i) {                                                       \
        for (int j = cy - 2; j <= cy + 2; ++j) {                                                   \
            for (int k = cz - 2; k <= cz + 2; ++k) {                                               \
                const int current_node = GridHash(i, j, k, bins_per_axis);                         \
                real3 current_node_location = NodeLocation(i, j, k, bin_edge, min_bounding_point); \
                X                                                                                  \
            }                                                                                      \
        }                                                                                          \
    }

#define LOOPONERING(X)                                                                             \
    const int cx = GridCoord(xi.x, inv_bin_edge, min_bounding_point.x);                            \
    const int cy = GridCoord(xi.y, inv_bin_edge, min_bounding_point.y);                            \
    const int cz = GridCoord(xi.z, inv_bin_edge, min_bounding_point.z);                            \
                                                                                                   \
    for (int i = cx - 1; i <= cx + 1; ++i) {                                                       \
        for (int j = cy - 1; j <= cy + 1; ++j) {                                                   \
            for (int k = cz - 1; k <= cz + 1; ++k) {                                               \
                const int current_node = GridHash(i, j, k, bins_per_axis);                         \
                real3 current_node_location = NodeLocation(i, j, k, bin_edge, min_bounding_point); \
                X                                                                                  \
            }                                                                                      \
        }                                                                                          \
    }

#define Potential_Energy_Derivative_Helper()                                      \
    real JP = Determinant(FP);                                                    \
    real JE = Determinant(FE);                                                    \
    /* Paper: Equation 2 */                                                       \
    real current_mu = mu * Exp(hardening_coefficient * (real(1.0) - JP));         \
    real current_lambda = lambda * Exp(hardening_coefficient * (real(1.0) - JP)); \
    Mat33 UE, VE;                                                                 \
    real3 EE;                                                                     \
    SVD(FE, UE, EE, VE);                                                          \
    /* Perform a polar decomposition, FE=RE*SE, RE is the Unitary part*/          \
    Mat33 RE = MultTranspose(UE, VE);

CUDA_HOST_DEVICE static Mat33 Potential_Energy_Derivative(const Mat33& FE,
                                                          const Mat33& FP,
                                                          real mu,
                                                          real lambda,
                                                          real hardening_coefficient) {
    Potential_Energy_Derivative_Helper();

    return real(2.) * current_mu * (FE - RE) + current_lambda * JE * (JE - real(1.)) * InverseTranspose(FE);
}
CUDA_HOST_DEVICE static Mat33 Solve_dR(Mat33 R, Mat33 S, Mat33 W) {
    Mat33 A;
    real3 b;
    // setup 3x3 system
    // Multiply out (R^TdR)*S + S*(R^TdR) and because it is skew symmetric we will only have three unknowns
    A[0] = S[8];
    A[1] = S[4];
    A[2] = -(S[5] + S[10]);

    A[4] = S[9];
    A[5] = -(S[0] + S[10]);
    A[6] = S[4];

    A[8] = -(S[0] + S[5]);
    A[9] = S[9];
    A[10] = S[8];
    // dF^TR is just the transpose of W which is R^TdF, this is the right hand side
    b[0] = W[4] - W[1];
    b[1] = W[2] - W[8];
    b[2] = W[9] - W[6];

    // solve for R^TdR
    real3 r = Inverse(A) * b;
    Mat33 rx = SkewSymmetric(r);

    Mat33 dR = R * rx;
    return dR;
}
CUDA_HOST_DEVICE static Mat33 Rotational_Derivative(const Mat33& F, const Mat33& dF) {
    Mat33 U, V, R, S, W;
    real3 E;
    SVD(F, U, E, V);
    // Perform polar decomposition F = R*S
    R = MultTranspose(U, V);
    S = V * MultTranspose(Mat33(E), V);
    // See tech report end of page 2
    W = TransposeMult(R, dF);

    return Solve_dR(R, S, W);
}

CUDA_HOST_DEVICE static Mat33 Rotational_Derivative_Simple(const Mat33& r, const Mat33& s, const Mat33& df) {
    // Assumes SVD has already been done
    Mat33 dR;
    real t1 = s[10] + s[0] + s[5];
    real t2 = Pow(s[4], 2);
    real t3 = s[0] * s[5];
    real t4 = df[0] * r[4] - df[4] * r[0] + df[1] * r[5] - df[5] * r[1] + df[2] * r[6] - df[6] * r[2];
    real t5 = s[0] + s[5];
    real t6 = Pow(s[8], 2);
    real t7 = Pow(s[9], 2);
    real t8 = Pow(s[0], 2);
    real t9 = s[4] * s[8];
    real t10 = 2;
    t8 = t10 * (t3 * s[10] - t9 * s[9]) + t5 * Pow(s[10], 2) - t2 * t5 - t6 * s[0] - (t7 - t8 - t3) * s[5] -
         (-Pow(s[5], 2) + t6 + t7 - t8) * s[10];
    t9 = (s[5] + s[10]) * s[9] + t9;
    t10 = df[0] * r[8] - df[8] * r[0] + df[1] * r[9] - df[9] * r[1] + df[2] * r[10] - df[10] * r[2];
    real t11 = (s[0] + s[10]) * s[8] + s[4] * s[9];
    real t12 = df[4] * r[8] - df[8] * r[4] + df[5] * r[9] - df[9] * r[5] + df[6] * r[10] - df[10] * r[6];
    t8 = 0.1e1 / t8;
    t2 = ((t1 * s[10] - t2 + t3) * t4 - t9 * t10 + t11 * t12) * t8;
    t3 = t5 * s[4] + s[8] * s[9];
    t5 = (-t10 * (t1 * s[5] + s[0] * s[10] - t6) + t12 * t3 + t4 * t9) * t8;
    t1 = (-t10 * t3 + t11 * t4 + t12 * (t1 * s[0] + s[5] * s[10] - t7)) * t8;
    dR[0] = df[0] - (t2 * r[4] - t5 * r[8]);
    dR[4] = df[4] - (t1 * r[8] - t2 * r[0]);
    dR[8] = df[8] - (-t1 * r[4] + t5 * r[0]);
    dR[1] = df[1] - (t2 * r[5] - t5 * r[9]);
    dR[5] = df[5] - (t1 * r[9] - t2 * r[1]);
    dR[9] = df[9] - (-t1 * r[5] + t5 * r[1]);
    dR[2] = df[2] - (t2 * r[6] - t5 * r[10]);
    dR[6] = df[6] - (t1 * r[10] - t2 * r[2]);
    dR[10] = df[10] - (-t1 * r[6] + t5 * r[2]);
    return dR;
}

CUDA_HOST_DEVICE inline void Vol_APFunc(const Mat33& s,
                                        const Mat33& r,
                                        const real* df,
                                        const Mat33& fe,
                                        const real current_mu,
                                        real* output) {
    real t103 = s[5] + s[0];
    real t58 = s[5] * s[0];
    real t102 = s[8] * s[4];
    real t101 = s[10] * s[10] - s[4] * s[4];
    real t100 = s[5] * s[5] - s[8] * s[8];
    real t99 = -s[9] * s[9] + s[0] * s[0];
    real t17 = fe[10] * df[8] + fe[9] * df[5] + fe[8] * df[2];
    real t18 = fe[6] * df[8] + fe[5] * df[5] + fe[4] * df[2];
    real t19 = fe[2] * df[8] + fe[1] * df[5] + fe[0] * df[2];
    real t20 = fe[10] * df[7] + fe[9] * df[4] + fe[8] * df[1];
    real t21 = fe[6] * df[7] + fe[5] * df[4] + fe[4] * df[1];
    real t22 = fe[2] * df[7] + fe[1] * df[4] + fe[0] * df[1];
    real t23 = fe[10] * df[6] + fe[9] * df[3] + fe[8] * df[0];
    real t24 = fe[6] * df[6] + fe[5] * df[3] + fe[4] * df[0];
    real t25 = fe[2] * df[6] + fe[1] * df[3] + fe[0] * df[0];
    real t98 = t103 * s[10] + t58;
    real t28 = t103 * s[4] + s[9] * s[8];
    real t27 = s[9] * s[4] + (s[10] + s[0]) * s[8];
    real t26 = t102 + (s[10] + s[5]) * s[9];
    real t16 = real(1.0) /
               (real(02.0) * s[9] * t102 + (t100 + t101) * s[0] + (t99 + t101) * s[5] + (t100 + t99 + 2 * t58) * s[10]);
    real t15 = t18 * r[2] - t19 * r[6] + t21 * r[1] - t22 * r[5] + t24 * r[0] - t25 * r[4];
    real t14 = t17 * r[6] - t18 * r[10] + t20 * r[5] - t21 * r[9] + t23 * r[4] - t24 * r[8];
    real t13 = -t17 * r[2] + t19 * r[10] - t20 * r[1] + t22 * r[9] - t23 * r[0] + t25 * r[8];
    real t12 = (t27 * t15 + t28 * t13 + (t98 + t99) * t14) * t16;
    real t11 = (t26 * t15 + (t98 + t100) * t13 + t28 * t14) * t16;
    real t10 = ((t98 + t101) * t15 + t26 * t13 + t27 * t14) * t16;
    real t9 = t10 * r[4] - t11 * r[8] + t25;
    real t8 = -t10 * r[0] + t12 * r[8] + t24;
    real t7 = t11 * r[0] - t12 * r[4] + t23;
    real t6 = t10 * r[5] - t11 * r[9] + t22;
    real t5 = -t10 * r[1] + t12 * r[9] + t21;
    real t4 = t11 * r[1] - t12 * r[5] + t20;
    real t3 = t10 * r[6] - t11 * r[10] + t19;
    real t2 = -t10 * r[2] + t12 * r[10] + t18;
    real t1 = t11 * r[2] - t12 * r[6] + t17;

    output[0] = (t7 * fe[8] + fe[4] * t8 + fe[0] * t9) * current_mu;
    output[1] = (t4 * fe[8] + t5 * fe[4] + t6 * fe[0]) * current_mu;
    output[2] = (t1 * fe[8] + t2 * fe[4] + t3 * fe[0]) * current_mu;

    output[3] = (fe[9] * t7 + fe[5] * t8 + fe[1] * t9) * current_mu;
    output[4] = (t4 * fe[9] + t5 * fe[5] + t6 * fe[1]) * current_mu;
    output[5] = (t1 * fe[9] + t2 * fe[5] + t3 * fe[1]) * current_mu;

    output[6] = (fe[10] * t7 + fe[6] * t8 + fe[2] * t9) * current_mu;
    output[7] = (t4 * fe[10] + t5 * fe[6] + t6 * fe[2]) * current_mu;
    output[8] = (t1 * fe[10] + t2 * fe[6] + t3 * fe[2]) * current_mu;
}

CUDA_HOST_DEVICE static void SplitPotential_Energy_Derivative(const Mat33& FE,
                                                              const Mat33& FP,
                                                              real mu,
                                                              real lambda,
                                                              real hardening_coefficient,
                                                              Mat33& Deviatoric,
                                                              Mat33& Dilational) {
    Potential_Energy_Derivative_Helper();

    Deviatoric = real(2.) * current_mu * (FE - RE);
    Dilational = current_lambda * JE * (JE - real(1.)) * InverseTranspose(FE);
}
CUDA_HOST_DEVICE static Mat33 Potential_Energy_Derivative_Deviatoric(const Mat33& FE,
                                                                     const Mat33& FP,
                                                                     real mu,
                                                                     real lambda,
                                                                     real hardening_coefficient) {
    Potential_Energy_Derivative_Helper();

    return real(2.) * current_mu * (FE - RE);
}

CUDA_HOST_DEVICE static void SplitPotential_Energy(const Mat33& FE,
                                                   const Mat33& FP,
                                                   real mu,
                                                   real lambda,
                                                   real hardening_coefficient,
                                                   real& Deviatoric,
                                                   real& Dilational) {
    Potential_Energy_Derivative_Helper();

    Deviatoric = current_mu * Trace(Transpose(FE - RE) * (FE - RE));
    Dilational = current_lambda / 2.0 * (JE - real(1.));
}
}
