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
#define root_two sqrtf(2.)
#define root_three sqrtf(3.)
#define root_six sqrtf(6.)
#define root_two_thirds sqrtf(2. / 3)
#define one_over_root_two 1. / sqrtf(2.)
#define one_over_root_three 1. / sqrtf(3.)

namespace chrono {
// Interpolation Functions

CUDA_HOST_DEVICE static float N_Tri(const float x) {
    if (fabsf(x) < float(1.0)) {
        return 1.0 - fabsf(x);
    }
    return float(0.0);
}
CUDA_HOST_DEVICE static float N_Tri(const float3& X, float inv_grid_dx) {
    return N_Tri(X.x * inv_grid_dx) * N_Tri(X.y * inv_grid_dx) * N_Tri(X.z * inv_grid_dx);
}

CUDA_HOST_DEVICE static float N(const float x) {
    if (fabsf(x) < float(1.0)) {
        return float(0.5) * Cube(fabsf(x)) - Sqr(x) + two_thirds;
    } else if (fabsf(x) < float(2.0)) {
        return -one_sixth * Cube(fabsf(x)) + Sqr(x) - float(2.0) * fabsf(x) + four_thirds;
    }
    return float(0.0);
}

CUDA_HOST_DEVICE static float N(const float3& X, float inv_grid_dx) {
    return N(X.x * inv_grid_dx) * N(X.y * inv_grid_dx) * N(X.z * inv_grid_dx);
}

CUDA_HOST_DEVICE static float N_tight(const float x) {
    if (fabsf(x) >= float(0.0) && fabsf(x) < float(.5)) {
        return -Sqr(x) + three_fourths;
    } else if (fabsf(x) >= float(1.0) && fabsf(x) < float(three_halves)) {
        return .5 * Sqr(x) - float(three_halves) * fabsf(x) + (9.0 / 8.0);
    }
    return float(0.0);
}

CUDA_HOST_DEVICE static float N_tight(const float3& X, float inv_grid_dx) {
    return N_tight(X.x * inv_grid_dx) * N_tight(X.y * inv_grid_dx) * N_tight(X.z * inv_grid_dx);
}

CUDA_HOST_DEVICE static float dN(const float x) {
    if (fabsf(x) < float(1.0)) {
        return float(1.5) * Sign(x) * Sqr(x) - float(2.0) * x;
    } else if (fabsf(x) < float(2.0)) {
        return -float(0.5) * Sign(x) * Sqr(x) + float(2.0) * x - float(2.0) * Sign(x);
    }
    return float(0.0);
}

CUDA_HOST_DEVICE static float3 dN(const float3& X, float inv_grid_dx) {
    float3 val = make_float3(0, 0, 0);
    float3 T = X * inv_grid_dx;
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

CUDA_HOST_DEVICE static inline int GridCoord(float x, float inv_bin_edge, float minimum) {
    float l = x - minimum;
    int c = roundf(l * inv_bin_edge);
    return c;
}

CUDA_HOST_DEVICE static inline int GridHash(int x, int y, int z, const int3& bins_per_axis) {
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
CUDA_HOST_DEVICE static inline int3 GridDecode(int hash, const int3& bins_per_axis) {
    int3 decoded_hash;
    decoded_hash.x = hash % (bins_per_axis.x * bins_per_axis.y) % bins_per_axis.x;
    decoded_hash.y = (hash % (bins_per_axis.x * bins_per_axis.y)) / bins_per_axis.x;
    decoded_hash.z = hash / (bins_per_axis.x * bins_per_axis.y);
    return decoded_hash;
}

CUDA_HOST_DEVICE static inline float3 NodeLocation(int i, int j, int k, float bin_edge, float3 min_bounding_point) {
    float3 node_location;
    node_location.x = i * bin_edge + min_bounding_point.x;
    node_location.y = j * bin_edge + min_bounding_point.y;
    node_location.z = k * bin_edge + min_bounding_point.z;
    return node_location;
}

#define LOOPOVERNODESY(X, Y)                                                                        \
    const int cx = GridCoord(xi.x, inv_bin_edge, min_bounding_point.x);                             \
    const int cy = GridCoord(xi.y, inv_bin_edge, min_bounding_point.y);                             \
    const int cz = GridCoord(xi.z, inv_bin_edge, min_bounding_point.z);                             \
                                                                                                    \
    for (int i = cx - Y; i <= cx + Y; ++i) {                                                        \
        for (int j = cy - Y; j <= cy + Y; ++j) {                                                    \
            for (int k = cz - Y; k <= cz + Y; ++k) {                                                \
                const int current_node = GridHash(i, j, k, bins_per_axis);                          \
                float3 current_node_location = NodeLocation(i, j, k, bin_edge, min_bounding_point); \
                X                                                                                   \
            }                                                                                       \
        }                                                                                           \
    }

#define LOOPOVERNODES(X)                                                                            \
    const int cx = GridCoord(xi.x, inv_bin_edge, min_bounding_point.x);                             \
    const int cy = GridCoord(xi.y, inv_bin_edge, min_bounding_point.y);                             \
    const int cz = GridCoord(xi.z, inv_bin_edge, min_bounding_point.z);                             \
                                                                                                    \
    for (int i = cx - 2; i <= cx + 2; ++i) {                                                        \
        for (int j = cy - 2; j <= cy + 2; ++j) {                                                    \
            for (int k = cz - 2; k <= cz + 2; ++k) {                                                \
                const int current_node = GridHash(i, j, k, bins_per_axis);                          \
                float3 current_node_location = NodeLocation(i, j, k, bin_edge, min_bounding_point); \
                X                                                                                   \
            }                                                                                       \
        }                                                                                           \
    }

#define LOOPONERING(X)                                                                              \
    const int cx = GridCoord(xi.x, inv_bin_edge, min_bounding_point.x);                             \
    const int cy = GridCoord(xi.y, inv_bin_edge, min_bounding_point.y);                             \
    const int cz = GridCoord(xi.z, inv_bin_edge, min_bounding_point.z);                             \
                                                                                                    \
    for (int i = cx - 1; i <= cx + 1; ++i) {                                                        \
        for (int j = cy - 1; j <= cy + 1; ++j) {                                                    \
            for (int k = cz - 1; k <= cz + 1; ++k) {                                                \
                const int current_node = GridHash(i, j, k, bins_per_axis);                          \
                float3 current_node_location = NodeLocation(i, j, k, bin_edge, min_bounding_point); \
                X                                                                                   \
            }                                                                                       \
        }                                                                                           \
    }

#define Potential_Energy_Derivative_Helper()                                         \
    float JP = Determinant(FP);                                                      \
    float JE = Determinant(FE);                                                      \
    /* Paper: Equation 2 */                                                          \
    float current_mu = mu * expf(hardening_coefficient * (float(1.0) - JP));         \
    float current_lambda = lambda * expf(hardening_coefficient * (float(1.0) - JP)); \
    Mat33f UE, VE;                                                                   \
    float3 EE;                                                                       \
    SVD(FE, UE, EE, VE);                                                             \
    /* Perform a polar decomposition, FE=RE*SE, RE is the Unitary part*/             \
    Mat33f RE = MultTranspose(UE, VE);

CUDA_HOST_DEVICE static Mat33f Potential_Energy_Derivative(const Mat33f& FE,
                                                           const Mat33f& FP,
                                                           float mu,
                                                           float lambda,
                                                           float hardening_coefficient) {
    Potential_Energy_Derivative_Helper();

    return float(2.) * current_mu * (FE - RE) + current_lambda * JE * (JE - float(1.)) * InverseTranspose(FE);
}
CUDA_HOST_DEVICE static Mat33f Solve_dR(Mat33f R, Mat33f S, Mat33f W) {
    Mat33f A;
    float3 b;
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
    b.x = W[4] - W[1];
    b.y = W[2] - W[8];
    b.z = W[9] - W[6];

    // solve for R^TdR
    float3 r = Inverse(A) * b;
    Mat33f rx = SkewSymmetric(r);

    Mat33f dR = R * rx;
    return dR;
}
CUDA_HOST_DEVICE static Mat33f Rotational_Derivative(const Mat33f& F, const Mat33f& dF) {
    Mat33f U, V, R, S, W;
    float3 E;
    SVD(F, U, E, V);
    // Perform polar decomposition F = R*S
    R = MultTranspose(U, V);
    S = V * MultTranspose(Mat33f(E), V);
    // See tech report end of page 2
    W = TransposeMult(R, dF);

    return Solve_dR(R, S, W);
}

CUDA_HOST_DEVICE static Mat33f Rotational_Derivative_Simple(const Mat33f& r, const Mat33f& s, const Mat33f& df) {
    // Assumes SVD has already been done
    Mat33f dR;
    float t1 = s[10] + s[0] + s[5];
    float t2 = powf(s[4], 2);
    float t3 = s[0] * s[5];
    float t4 = df[0] * r[4] - df[4] * r[0] + df[1] * r[5] - df[5] * r[1] + df[2] * r[6] - df[6] * r[2];
    float t5 = s[0] + s[5];
    float t6 = powf(s[8], 2);
    float t7 = powf(s[9], 2);
    float t8 = powf(s[0], 2);
    float t9 = s[4] * s[8];
    float t10 = 2;
    t8 = t10 * (t3 * s[10] - t9 * s[9]) + t5 * powf(s[10], 2) - t2 * t5 - t6 * s[0] - (t7 - t8 - t3) * s[5] -
         (-powf(s[5], 2) + t6 + t7 - t8) * s[10];
    t9 = (s[5] + s[10]) * s[9] + t9;
    t10 = df[0] * r[8] - df[8] * r[0] + df[1] * r[9] - df[9] * r[1] + df[2] * r[10] - df[10] * r[2];
    float t11 = (s[0] + s[10]) * s[8] + s[4] * s[9];
    float t12 = df[4] * r[8] - df[8] * r[4] + df[5] * r[9] - df[9] * r[5] + df[6] * r[10] - df[10] * r[6];
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

CUDA_HOST_DEVICE inline void Vol_APFunc(const Mat33f& s,
                                        const Mat33f& r,
                                        const float* df,
                                        const Mat33f& fe,
                                        const float current_mu,
                                        float* output) {
    float t103 = s[5] + s[0];
    float t58 = s[5] * s[0];
    float t102 = s[8] * s[4];
    float t101 = s[10] * s[10] - s[4] * s[4];
    float t100 = s[5] * s[5] - s[8] * s[8];
    float t99 = -s[9] * s[9] + s[0] * s[0];
    float t17 = fe[10] * df[8] + fe[9] * df[5] + fe[8] * df[2];
    float t18 = fe[6] * df[8] + fe[5] * df[5] + fe[4] * df[2];
    float t19 = fe[2] * df[8] + fe[1] * df[5] + fe[0] * df[2];
    float t20 = fe[10] * df[7] + fe[9] * df[4] + fe[8] * df[1];
    float t21 = fe[6] * df[7] + fe[5] * df[4] + fe[4] * df[1];
    float t22 = fe[2] * df[7] + fe[1] * df[4] + fe[0] * df[1];
    float t23 = fe[10] * df[6] + fe[9] * df[3] + fe[8] * df[0];
    float t24 = fe[6] * df[6] + fe[5] * df[3] + fe[4] * df[0];
    float t25 = fe[2] * df[6] + fe[1] * df[3] + fe[0] * df[0];
    float t98 = t103 * s[10] + t58;
    float t28 = t103 * s[4] + s[9] * s[8];
    float t27 = s[9] * s[4] + (s[10] + s[0]) * s[8];
    float t26 = t102 + (s[10] + s[5]) * s[9];
    float t16 = float(1.0) / (float(02.0) * s[9] * t102 + (t100 + t101) * s[0] + (t99 + t101) * s[5] +
                              (t100 + t99 + 2 * t58) * s[10]);
    float t15 = t18 * r[2] - t19 * r[6] + t21 * r[1] - t22 * r[5] + t24 * r[0] - t25 * r[4];
    float t14 = t17 * r[6] - t18 * r[10] + t20 * r[5] - t21 * r[9] + t23 * r[4] - t24 * r[8];
    float t13 = -t17 * r[2] + t19 * r[10] - t20 * r[1] + t22 * r[9] - t23 * r[0] + t25 * r[8];
    float t12 = (t27 * t15 + t28 * t13 + (t98 + t99) * t14) * t16;
    float t11 = (t26 * t15 + (t98 + t100) * t13 + t28 * t14) * t16;
    float t10 = ((t98 + t101) * t15 + t26 * t13 + t27 * t14) * t16;
    float t9 = t10 * r[4] - t11 * r[8] + t25;
    float t8 = -t10 * r[0] + t12 * r[8] + t24;
    float t7 = t11 * r[0] - t12 * r[4] + t23;
    float t6 = t10 * r[5] - t11 * r[9] + t22;
    float t5 = -t10 * r[1] + t12 * r[9] + t21;
    float t4 = t11 * r[1] - t12 * r[5] + t20;
    float t3 = t10 * r[6] - t11 * r[10] + t19;
    float t2 = -t10 * r[2] + t12 * r[10] + t18;
    float t1 = t11 * r[2] - t12 * r[6] + t17;

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

CUDA_HOST_DEVICE static void SplitPotential_Energy_Derivative(const Mat33f& FE,
                                                              const Mat33f& FP,
                                                              float mu,
                                                              float lambda,
                                                              float hardening_coefficient,
                                                              Mat33f& Deviatoric,
                                                              Mat33f& Dilational) {
    Potential_Energy_Derivative_Helper();

    Deviatoric = float(2.) * current_mu * (FE - RE);
    Dilational = current_lambda * JE * (JE - float(1.)) * InverseTranspose(FE);
}
CUDA_HOST_DEVICE static Mat33f Potential_Energy_Derivative_Deviatoric(const Mat33f& FE,
                                                                      const Mat33f& FP,
                                                                      float mu,
                                                                      float hardening_coefficient,
                                                                      Mat33f& RE,
                                                                      Mat33f& SE) {
    float JP = Determinant(FP);
    float current_mu = mu * expf(hardening_coefficient * (float(1.0) - JP));
    Mat33f UE, VE;
    float3 EE;
    SVD(FE, UE, EE, VE); /* Perform a polar decomposition, FE=RE*SE, RE is the Unitary part*/
    RE = MultTranspose(UE, VE);
    SE = VE * MultTranspose(Mat33f(EE), VE);
    // RE[3] = JP;
    RE[3] = current_mu;
    return float(2.) * current_mu * (FE - RE);
}

CUDA_HOST_DEVICE static void SplitPotential_Energy(const Mat33f& FE,
                                                   const Mat33f& FP,
                                                   float mu,
                                                   float lambda,
                                                   float hardening_coefficient,
                                                   float& Deviatoric,
                                                   float& Dilational) {
    Potential_Energy_Derivative_Helper();

    Deviatoric = current_mu * Trace(Transpose(FE - RE) * (FE - RE));
    Dilational = current_lambda / 2.0 * (JE - float(1.));
}

CUDA_HOST_DEVICE static inline Mat33f B__Z(const Mat33f& Z, const Mat33f& F, float Ja, float a, const Mat33f& H) {
    return Ja * (Z + a * (DoubleDot(H, Z)) * F);
}

CUDA_HOST_DEVICE static inline Mat33f Z__B(const Mat33f& Z, const Mat33f& F, float Ja, float a, const Mat33f& H) {
    return Ja * (Z + a * (DoubleDot(F, Z)) * H);
}

CUDA_HOST_DEVICE static Mat33f d2PsidFdF(const Mat33f& Z,  // This is deltaF
                                         const Mat33f& F,  // This is FE_hat
                                         const Mat33f& FP,
                                         const Mat33f& RE,
                                         const Mat33f& SE,
                                         float mu,
                                         float hardening_coefficient) {
#if 1
    float a = -1.0 / 3.0;
    float Ja = powf(Determinant(F), a);
    Mat33f H = InverseTranspose(F);
    // float JP = RE[3];  // Determinant(FP);
    float current_mu = RE[3];  // mu * expf(hardening_coefficient * (float(1.0) - JP));
    Mat33f FE = Ja * F;
    // Mat33f A = Potential_Energy_Derivative_Deviatoric(Ja * F, FP, mu, hardening_coefficient);
    //    Mat33f UE, VE;
    //    float3 EE;
    //    SVD(FE, UE, EE, VE); /* Perform a polar decomposition, FE=RE*SE, RE is the Unitary part*/
    //    Mat33f RE = MultTranspose(UE, VE);
    //    Mat33f SE = VE * MultTranspose(Mat33f(EE), VE);

    Mat33f A = float(2.) * current_mu * (FE - RE);

    Mat33f B_Z = B__Z(Z, F, Ja, a, H);
    Mat33f WE = TransposeMult(RE, B_Z);
    // C is the original second derivative
    Mat33f C_B_Z = 2 * current_mu * (B_Z - Solve_dR(RE, SE, WE));
    Mat33f P1 = Z__B(C_B_Z, F, Ja, a, H);
    Mat33f P2 = a * DoubleDot(H, Z) * Z__B(A, F, Ja, a, H);
    Mat33f P3 = a * Ja * DoubleDot(A, Z) * H;
    Mat33f P4 = -a * Ja * DoubleDot(A, F) * H * TransposeMult(Z, H);

    return P1 + P2 + P3 + P4;
#else
    float current_mu = RE[3];
    Mat33f WE = TransposeMult(RE, Z);
    return 2 * current_mu * (Z - Solve_dR(RE, SE, WE));
#endif
}
}
