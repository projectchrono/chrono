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
    A[0] = S[6];
    A[1] = S[3];
    A[2] = -(S[4] + S[8]);

    A[3] = S[7];
    A[4] = -(S[0] + S[8]);
    A[5] = S[3];

    A[6] = -(S[0] + S[4]);
    A[7] = S[7];
    A[8] = S[6];
    // dF^TR is just the transpose of W which is R^TdF, this is the right hand side
    b.x = W[3] - W[1];
    b.y = W[2] - W[6];
    b.z = W[7] - W[5];

    // solve for R^TdR
    float3 r = Inverse(A) * b;
    Mat33f rx = SkewSymmetric(r);

    Mat33f dR = R * rx;
    return dR;
}
// CUDA_HOST_DEVICE static Mat33f Rotational_Derivative(const Mat33f& F, const Mat33f& dF) {
//    Mat33f U, V, R, S, W;
//    float3 E;
//    SVD(F, U, E, V);
//    // Perform polar decomposition F = R*S
//    R = MultTranspose(U, V);
//    S = V * MultTranspose(Mat33f(E), V);
//    // See tech report end of page 2
//    W = TransposeMult(R, dF);
//
//    return Solve_dR(R, S, W);
//}
//
// CUDA_HOST_DEVICE static void SplitPotential_Energy_Derivative(const Mat33f& FE,
//                                                              const Mat33f& FP,
//                                                              float mu,
//                                                              float lambda,
//                                                              float hardening_coefficient,
//                                                              Mat33f& Deviatoric,
//                                                              Mat33f& Dilational) {
//    Potential_Energy_Derivative_Helper();
//
//    Deviatoric = float(2.) * current_mu * (FE - RE);
//    Dilational = current_lambda * JE * (JE - float(1.)) * InverseTranspose(FE);
//}
CUDA_HOST_DEVICE static Mat33f Potential_Energy_Derivative_Deviatoric(const Mat33f& FE,
                                                                      float current_mu,
                                                                      Mat33f& RE,
                                                                      Mat33f& SE) {
    Mat33f UE, VE;
    float3 EE;
    SVD(FE, UE, EE, VE); /* Perform a polar decomposition, FE=RE*SE, RE is the Unitary part*/
    RE = MultTranspose(UE, VE);
    SE = VE * MultTranspose(Mat33f(EE), VE);
    return float(2.) * current_mu * (FE - RE);
}
// CUDA_HOST_DEVICE static void SplitPotential_Energy(const Mat33f& FE,
//                                                   const Mat33f& FP,
//                                                   float mu,
//                                                   float lambda,
//                                                   float hardening_coefficient,
//                                                   float& Deviatoric,
//                                                   float& Dilational) {
//    Potential_Energy_Derivative_Helper();
//
//    Deviatoric = current_mu * Trace(Transpose(FE - RE) * (FE - RE));
//    Dilational = current_lambda / 2.0 * (JE - float(1.));
//}

CUDA_HOST_DEVICE static inline Mat33f B__Z(const Mat33f& Z, const Mat33f& F, float Ja, float a, const Mat33f& H) {
    return Ja * (Z + a * (DoubleDot(H, Z)) * F);
}

CUDA_HOST_DEVICE static inline Mat33f Z__B(const Mat33f& Z, const Mat33f& F, float Ja, float a, const Mat33f& H) {
    return Ja * (Z + a * (DoubleDot(F, Z)) * H);
}

CUDA_HOST_DEVICE static Mat33f d2PsidFdF(const Mat33f& Z,  // This is deltaF
                                         const Mat33f& F,  // This is FE_hat
                                         const Mat33f& RE,
                                         const Mat33f& SE,
                                         const float current_mu) {
#if 1
    float a = -1.0 / 3.0;
    float Ja = powf(Determinant(F), a);
    Mat33f H = InverseTranspose(F);
    Mat33f FE = Ja * F;
    // Mat33f A = Potential_Energy_Derivative_Deviatoric(Ja * F, current_mu);
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
    float JP = Determinant(FP);
    float current_mu = mu * expf(hardening_coefficient * (float(1.0) - JP));
    Mat33f WE = TransposeMult(RE, Z);
    return 2 * current_mu * (Z - Solve_dR(RE, SE, WE));
#endif
}
}
