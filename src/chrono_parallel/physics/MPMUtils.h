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
CUDA_HOST_DEVICE static inline Mat33f Solve_dR(Mat33f R, Mat33f S, Mat33f W) {
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
CUDA_HOST_DEVICE static inline Mat33f Potential_Energy_Derivative_Deviatoric(const Mat33f& FE,
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

CUDA_HOST_DEVICE static inline Mat33f d2PsidFdF(const Mat33f& Z,  // This is deltaF
                                                const Mat33f& F,  // This is FE_hat
                                                const Mat33f& RE,
                                                const Mat33f& SE,
                                                const float current_mu) {
#if 1
    float a = -one_third;
    float Ja = powf(Determinant(F), a);
    Mat33f H = InverseTranspose(F);
    Mat33f FE = Ja * F;
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

CUDA_HOST_DEVICE static Mat33f d2PsidFdFO(const Mat33f& z,   // This is deltaF
                                          const Mat33f& fe,  // This is FE_hat
                                          const Mat33f& re,
                                          const Mat33f& se,
                                          const float current_mu) {
    float t1 = fe[0] * fe[4];
    float t3 = fe[0] * fe[7];
    float t5 = fe[3] * fe[1];
    float t7 = fe[3] * fe[7];
    float t9 = fe[6] * fe[1];
    float t11 = fe[6] * fe[4];
    float t13 = t1 * fe[8] - t11 * fe[2] - t3 * fe[5] - t5 * fe[8] + t7 * fe[2] + t9 * fe[5];
    float t14 = powf(t13, 1. / 3.);
    float t15 = 1. / t14;
    float t18 = fe[4] * fe[8] - fe[7] * fe[5];
    float t19 = 1. / t13;
    float t24 = fe[1] * fe[8] - fe[7] * fe[2];
    float t29 = fe[1] * fe[5] - fe[4] * fe[2];
    float t34 = fe[3] * fe[8] - fe[6] * fe[5];
    float t39 = fe[0] * fe[8] - fe[6] * fe[2];
    float t44 = fe[0] * fe[5] - fe[3] * fe[2];
    float t47 = t7 - t11;
    float t50 = t3 - t9;
    float t53 = t1 - t5;
    float t56 = -t18 * t19 * z[0] + t24 * t19 * z[3] - t29 * t19 * z[6] + t34 * t19 * z[1] - t39 * t19 * z[4] +
                t44 * t19 * z[7] - t47 * t19 * z[2] + t50 * t19 * z[5] - t53 * t19 * z[8];
    float t58 = z[0] + t56 * fe[0] / 3.;
    float t60 = se[0] * se[4];
    float t61 = se[0] * se[8];
    float t62 = powf(se[3], 2.);
    float t63 = se[4] * se[8];
    float t64 = powf(se[8], 2.);
    float t66 = powf(se[0], 2.);
    float t70 = powf(se[6], 2.);
    float t72 = powf(se[4], 2.);
    float t78 = se[3] * se[6];
    float t83 = powf(se[7], 2.);
    float t87 = 2. * t60 * se[8] - se[0] * t62 - t62 * se[4] + se[0] * t64 + se[4] * t64 + t66 * se[4] + t66 * se[8] -
                se[0] * t70 - t70 * se[8] + se[0] * t72 + t72 * se[8] - 2. * t78 * se[7] - se[4] * t83 - t83 * se[8];
    float t88 = 1. / t87;
    float t90 = re[0] * t15;
    float t92 = z[3] + t56 * fe[3] / 3.;
    float t94 = re[1] * t15;
    float t96 = z[4] + t56 * fe[4] / 3.;
    float t98 = re[2] * t15;
    float t100 = z[5] + t56 * fe[5] / 3.;
    float t102 = re[3] * t15;
    float t104 = re[4] * t15;
    float t106 = z[1] + t56 * fe[1] / 3.;
    float t108 = re[5] * t15;
    float t110 = z[2] + t56 * fe[2] / 3.;
    float t112 = t98 * t100 - t102 * t58 - t104 * t106 - t108 * t110 + t90 * t92 + t94 * t96;
    float t117 = (se[4] * se[7] + se[7] * se[8] + t78) * t88;
    float t118 = re[6] * t15;
    float t120 = re[7] * t15;
    float t122 = re[8] * t15;
    float t125 = z[6] + t56 * fe[6] / 3.;
    float t128 = z[7] + t56 * fe[7] / 3.;
    float t131 = z[8] + t56 * fe[8] / 3.;
    float t133 = t120 * t106 + t122 * t110 + t118 * t58 - t90 * t125 - t94 * t128 - t98 * t131;
    float t139 = (se[0] * se[6] + se[3] * se[7] + se[6] * se[8]) * t88;
    float t146 = -t122 * t100 + t102 * t125 + t104 * t128 + t108 * t131 - t118 * t92 - t120 * t96;
    float t148 = -(t60 + t61 - t62 + t63 + t64) * t88 * t112 - t117 * t133 - t139 * t146;
    float t158 = (se[0] * se[3] + se[3] * se[4] + se[6] * se[7]) * t88;
    float t160 = t117 * t112 + (t60 + t61 - t70 + t72 + t63) * t88 * t133 + t158 * t146;
    float t162 = -re[3] * t148 + t15 * t58 - re[6] * t160;
    float t165 = fe[0] * current_mu;
    float t167 = fe[3] * current_mu;
    float t175 = -t139 * t112 - t158 * t133 - (t66 + t60 + t61 + t63 - t83) * t88 * t146;
    float t177 = re[0] * t148 + t15 * t92 - re[6] * t175;
    float t179 = fe[6] * current_mu;
    float t183 = t15 * t125 + re[0] * t160 + re[3] * t175;
    float t185 = fe[1] * current_mu;
    float t189 = t15 * t106 - re[4] * t148 - re[7] * t160;
    float t191 = fe[4] * current_mu;
    float t195 = re[1] * t148 + t15 * t96 - re[7] * t175;
    float t197 = fe[7] * current_mu;
    float t201 = t15 * t128 + re[1] * t160 + re[4] * t175;
    float t203 = fe[2] * current_mu;
    float t207 = t15 * t110 - re[5] * t148 - re[8] * t160;
    float t209 = fe[5] * current_mu;
    float t213 = t15 * t100 + re[2] * t148 - re[8] * t175;
    float t215 = fe[8] * current_mu;
    float t219 = t15 * t131 + re[2] * t160 + re[5] * t175;
    float t221 = -t165 * t162 - t167 * t177 - t179 * t183 - t185 * t189 - t191 * t195 - t197 * t201 - t203 * t207 -
                 t209 * t213 - t215 * t219;
    float t227 = t15 * fe[0] - re[0];
    float t228 = current_mu * t227;
    float t231 = t15 * fe[3] - re[3];
    float t232 = current_mu * t231;
    float t235 = t15 * fe[6] - re[6];
    float t236 = current_mu * t235;
    float t239 = t15 * fe[1] - re[1];
    float t240 = current_mu * t239;
    float t243 = t15 * fe[4] - re[4];
    float t244 = current_mu * t243;
    float t247 = t15 * fe[7] - re[7];
    float t248 = current_mu * t247;
    float t251 = t15 * fe[2] - re[2];
    float t252 = current_mu * t251;
    float t255 = t15 * fe[5] - re[5];
    float t256 = current_mu * t255;
    float t259 = t15 * fe[8] - re[8];
    float t260 = current_mu * t259;
    float t262 = -t228 * z[0] - t232 * z[3] - t236 * z[6] - t240 * z[1] - t244 * z[4] - t248 * z[7] - t252 * z[2] -
                 t256 * z[5] - t260 * z[8];
    float t263 = 2. / 3. * t262 * t15;
    float t274 = -t165 * t227 - t167 * t231 - t179 * t235 - t185 * t239 - t191 * t243 - t197 * t247 - t203 * t251 -
                 t209 * t255 - t215 * t259;
    float t280 = 1. / t14 / t13;
    float t281 = -2. * t280 * t262;
    float t284 = -2. * t280 * t274;
    float t291 = -t284 * t18 * z[0] + t284 * t24 * z[3] - t284 * t29 * z[6];
    float t300 = -t284 * t18 * z[1] + t284 * t24 * z[4] - t284 * t29 * z[7];
    float t309 = -t284 * t18 * z[2] + t284 * t24 * z[5] - t284 * t29 * z[8];
    float t372 = t284 * t34 * z[0] - t284 * t39 * z[3] + t284 * t44 * z[6];
    float t381 = t284 * t34 * z[1] - t284 * t39 * z[4] + t284 * t44 * z[7];
    float t390 = t284 * t34 * z[2] - t284 * t39 * z[5] + t284 * t44 * z[8];
    float t453 = -t284 * t47 * z[0] + t284 * t50 * z[3] - t284 * t53 * z[6];
    float t462 = -t284 * t47 * z[1] + t284 * t50 * z[4] - t284 * t53 * z[7];
    float t471 = -t284 * t47 * z[2] + t284 * t50 * z[5] - t284 * t53 * z[8];
    Mat33f unknown;

    unknown[0] = t15 * (2. * current_mu * t162 + 2. / 3. * t221 * t18 * t19) +
                 t263 * (2. * t228 + 2. / 3. * t274 * t18 * t19) - t281 * t18 / 3. - t291 * t18 * t19 / 3. +
                 t300 * t34 * t19 / 3. - t309 * t47 * t19 / 3.;
    unknown[3] = t15 * (2. * current_mu * t177 - 2. / 3. * t221 * t24 * t19) +
                 t263 * (2. * t232 - 2. / 3. * t274 * t24 * t19) + t281 * t24 / 3. + t291 * t24 * t19 / 3. -
                 t300 * t39 * t19 / 3. + t309 * t50 * t19 / 3.;
    unknown[6] = t15 * (2. * current_mu * t183 + 2. / 3. * t221 * t29 * t19) +
                 t263 * (2. * t236 + 2. / 3. * t274 * t29 * t19) - t281 * t29 / 3. - t291 * t29 * t19 / 3. +
                 t300 * t44 * t19 / 3. - t309 * t53 * t19 / 3.;
    unknown[1] = t15 * (2. * current_mu * t189 - 2. / 3. * t221 * t34 * t19) +
                 t263 * (2. * t240 - 2. / 3. * t274 * t34 * t19) + t281 * t34 / 3. - t372 * t18 * t19 / 3. +
                 t381 * t34 * t19 / 3. - t390 * t47 * t19 / 3.;
    unknown[4] = t15 * (2. * current_mu * t195 + 2. / 3. * t221 * t39 * t19) +
                 t263 * (2. * t244 + 2. / 3. * t274 * t39 * t19) - t281 * t39 / 3. + t372 * t24 * t19 / 3. -
                 t381 * t39 * t19 / 3. + t390 * t50 * t19 / 3.;
    unknown[7] = t15 * (2. * current_mu * t201 - 2. / 3. * t221 * t44 * t19) +
                 t263 * (2. * t248 - 2. / 3. * t274 * t44 * t19) + t281 * t44 / 3. - t372 * t29 * t19 / 3. +
                 t381 * t44 * t19 / 3. - t390 * t53 * t19 / 3.;
    unknown[2] = t15 * (2. * current_mu * t207 + 2. / 3. * t221 * t47 * t19) +
                 t263 * (2. * t252 + 2. / 3. * t274 * t47 * t19) - t281 * t47 / 3. - t453 * t18 * t19 / 3. +
                 t462 * t34 * t19 / 3. - t471 * t47 * t19 / 3.;
    unknown[5] = t15 * (2. * current_mu * t213 - 2. / 3. * t221 * t50 * t19) +
                 t263 * (2. * t256 - 2. / 3. * t274 * t50 * t19) + t281 * t50 / 3. + t453 * t24 * t19 / 3. -
                 t462 * t39 * t19 / 3. + t471 * t50 * t19 / 3.;
    unknown[8] = t15 * (2. * current_mu * t219 + 2. / 3. * t221 * t53 * t19) +
                 t263 * (2. * t260 + 2. / 3. * t274 * t53 * t19) - t281 * t53 / 3. - t453 * t29 * t19 / 3. +
                 t462 * t44 * t19 / 3. - t471 * t53 * t19 / 3.;
    return unknown;
}
}
