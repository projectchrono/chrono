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
#define one_third 1.f / 3.f
#define two_thirds 2.f / 3.f
#define three_halves 3.f / 2.f
#define four_thirds 4.f / 3.f
#define three_fourths 3.f / 4.f
#define five_thirds 5.f / 3.f
#define one_sixth 1.f / 6.f
#define one_ninth 1.f / 9.f
#define one_twelfth 1.f / 12.f
#define one_twenty_fourth 1.f / 24.f
#define one_twenty_seventh 1.f / 27.f
#define one_sixtieth 1.f / 60.f
#define thirteen_over_twelve 13.f / 12.f
#define root_two sqrtf(2.f)
#define root_three sqrtf(3.f)
#define root_six sqrtf(6.f)
#define root_two_thirds sqrtf(2.f / 3.f)
#define one_over_root_two 1.f / sqrtf(2.f)
#define one_over_root_three 1.f / sqrtf(3.f)

namespace chrono {
// Interpolation Functions

CUDA_HOST_DEVICE static float N_Tri(const float x) {
    if (fabsf(x) < float(1.0)) {
        return float(1.0) - fabsf(x);
    }
    return float(0.0);
}
CUDA_HOST_DEVICE static float N_Tri(const float3& X, const float inv_grid_dx) {
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
        return float(.5) * Sqr(x) - float(three_halves) * fabsf(x) + float(9.0 / 8.0);
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

CUDA_HOST_DEVICE static float3 dN(const float3& X, const float inv_grid_dx) {
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

CUDA_HOST_DEVICE static inline int GridCoord(const float x, const float inv_bin_edge, const float minimum) {
    float l = x - minimum;
    int c = (int)roundf(l * inv_bin_edge);
    return c;
}

CUDA_HOST_DEVICE static inline int GridHash(const int x, const int y, const int z, const int3& bins_per_axis) {
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
CUDA_HOST_DEVICE static inline int3 GridDecode(const int hash, const int3& bins_per_axis) {
    int3 decoded_hash;
    decoded_hash.x = hash % (bins_per_axis.x * bins_per_axis.y) % bins_per_axis.x;
    decoded_hash.y = (hash % (bins_per_axis.x * bins_per_axis.y)) / bins_per_axis.x;
    decoded_hash.z = hash / (bins_per_axis.x * bins_per_axis.y);
    return decoded_hash;
}

CUDA_HOST_DEVICE static inline float3 NodeLocation(const int i,
                                                   const int j,
                                                   const int k,
                                                   const float bin_edge,
                                                   const float3& min_bounding_point) {
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

//#define Potential_Energy_Derivative_Helper()                                         \
//    float JP = Determinant(FP);                                                      \
//    float JE = Determinant(FE);                                                      \
//    /* Paper: Equation 2 */                                                          \
//    float current_mu = mu * expf(hardening_coefficient * (float(1.0) - JP));         \
//    float current_lambda = lambda * expf(hardening_coefficient * (float(1.0) - JP)); \
//    Mat33f UE, VE;                                                                   \
//    float3 EE;                                                                       \
//    SVD(FE, UE, EE, VE);                                                             \
//    /* Perform a polar decomposition, FE=RE*SE, RE is the Unitary part*/             \
//    Mat33f RE = MultTranspose(UE, VE);

//CUDA_HOST_DEVICE static Mat33f Potential_Energy_Derivative(const Mat33f& FE,
//                                                           const Mat33f& FP,
//                                                           const float mu,
//                                                           const float lambda,
//                                                           const float hardening_coefficient) {
//    Potential_Energy_Derivative_Helper();
//
//    return float(2.) * current_mu * (FE - RE) + current_lambda * JE * (JE - float(1.)) * InverseTranspose(FE);
//}
CUDA_HOST_DEVICE static inline Mat33f Solve_dR(const Mat33f& R, const SymMat33f& S, const Mat33f& W) {
    Mat33f A;
    float3 b;
    // setup 3x3 system
    // Multiply out (R^TdR)*S + S*(R^TdR) and because it is skew symmetric we will only have three unknowns
    A[0] = S[2];
    A[1] = S[1];
    A[2] = -(S[3] + S[5]);

    A[3] = S[4];
    A[4] = -(S[0] + S[5]);
    A[5] = S[1];

    A[6] = -(S[0] + S[3]);
    A[7] = S[4];
    A[8] = S[2];

    // 013
    //-24
    //--5

    // 0--
    // 13-
    // 245

    // 036  //11 12 13
    //-47  //21 22 23
    //--8  //31 32 33
    // dF^TR is just the transpose of W which is R^TdF, this is the right hand side
    b.x = W[3] - W[1];
    b.y = W[2] - W[6];
    b.z = W[7] - W[5];

    // solve for R^TdR
    float3 r = InverseUnsafe(A) * b;
    // Mat33f rx = SkewSymmetric(r);
    // Mat33f dR =R * rx

    Mat33f dR = MultSkew(R, r);
    return dR;
}

CUDA_HOST_DEVICE static inline void Solve_dRBZ(const Mat33f& R, const Mat33f& S, const Mat33f& W, Mat33f& B_Z) {
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

    B_Z = B_Z - (R * rx);
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
                                                                             const float current_mu,
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

CUDA_HOST_DEVICE static inline Mat33f B__Z(const Mat33f& Z,
                                           const Mat33f& F,
                                           const float Ja,
                                           const float a,
                                           const Mat33f& H) {
    return Ja * (Z + (a * DoubleDot(H, Z)) * F);
}

CUDA_HOST_DEVICE static inline Mat33f Z__B(const Mat33f& Z,
                                           const Mat33f& F,
                                           const float Ja,
                                           const float a,
                                           const Mat33f& H) {
    return Ja * (Z + (a * DoubleDot(F, Z)) * H);
}
//
// CUDA_HOST_DEVICE static inline Mat33f d2PsidFdF(const Mat33f& Z,  // This is deltaF
//                                                const Mat33f& F,  // This is FE_hat
//                                                const Mat33f& RE,
//                                                const Mat33f& SE,
//                                                const float current_mu) {
//#if 1
//    float a = -one_third;
//    float Ja = powf(Determinant(F), a);
//    Mat33f H = InverseTranspose(F);
//    Mat33f FE = Ja * F;
//    Mat33f A = float(2.) * current_mu * (FE - RE);
//
//    Mat33f B_Z = B__Z(Z, F, Ja, a, H);
//    Mat33f WE = TransposeMult(RE, B_Z);
//    // C is the original second derivative
//    Mat33f C_B_Z = 2 * current_mu * (B_Z - Solve_dR(RE, SE, WE));
//    Mat33f P1 = Z__B(C_B_Z, F, Ja, a, H);
//    Mat33f P2 = a * DoubleDot(H, Z) * Z__B(A, F, Ja, a, H);
//    Mat33f P3 = a * Ja * DoubleDot(A, Z) * H;
//    Mat33f P4 = -a * Ja * DoubleDot(A, F) * H * TransposeMult(Z, H);
//
//    return P1 + P2 + P3 + P4;
//#else
//    float JP = Determinant(FP);
//    float current_mu = mu * expf(hardening_coefficient * (float(1.0) - JP));
//    Mat33f WE = TransposeMult(RE, Z);
//    return 2 * current_mu * (Z - Solve_dR(RE, SE, WE));
//#endif
//}
//
// CUDA_HOST_DEVICE static inline Mat33f d2PsidFdFO(const Mat33f& F,        // This is deltaF
//                                                 const Mat33f& delta_F,  // This is deltaF
//                                                 const Mat33f& m_FE,
//                                                 const Mat33f& RE,
//                                                 const Mat33f& SE,
//                                                 const float current_mu,
//                                                 const float mvolume) {
//    float a = -one_third;
//
//    Mat33f VAP;
//    float t1 = F[1] * F[5] - F[4] * F[2];
//    float t2 = -F[1] * F[8] + F[7] * F[2];
//    float t3 = F[4] * F[8] - F[7] * F[5];
//    float t4 = t1 * F[6] + t2 * F[3] + t3 * F[0];
//    float t5 = powf(t4, a);
//    float t6 = delta_F[0] * m_FE[0];
//    float t7 = delta_F[3] * m_FE[1];
//    float t8 = delta_F[6] * m_FE[2];
//    float t9 = t8 + t6 + t7;
//    t2 = -t2;
//    float t10 = delta_F[0] * m_FE[3];
//    float t11 = delta_F[3] * m_FE[4];
//    float t12 = delta_F[6] * m_FE[5];
//    float t13 = t12 + t10 + t11;
//    float t14 = delta_F[0] * m_FE[6];
//    float t15 = delta_F[3] * m_FE[7];
//    float t16 = delta_F[6] * m_FE[8];
//    float t17 = t14 + t15 + t16;
//    float t18 = F[3] * F[8] - F[6] * F[5];
//    float t19 = delta_F[1] * m_FE[0];
//    float t20 = delta_F[4] * m_FE[1];
//    float t21 = delta_F[7] * m_FE[2];
//    float t22 = t19 + t20 + t21;
//    float t23 = F[0] * F[8] - F[6] * F[2];
//    float t24 = delta_F[1] * m_FE[3];
//    float t25 = delta_F[4] * m_FE[4];
//    float t26 = delta_F[7] * m_FE[5];
//    float t27 = t25 + t26 + t24;
//    float t28 = F[0] * F[5] - F[3] * F[2];
//    float t29 = delta_F[1] * m_FE[6];
//    float t30 = delta_F[4] * m_FE[7];
//    float t31 = delta_F[7] * m_FE[8];
//    float t32 = t30 + t31 + t29;
//    float t33 = F[3] * F[7] - F[6] * F[4];
//    float t34 = delta_F[2] * m_FE[0];
//    float t35 = delta_F[5] * m_FE[1];
//    float t36 = delta_F[8] * m_FE[2];
//    float t37 = t36 + t34 + t35;
//    float t38 = F[0] * F[7] - F[6] * F[1];
//    float t39 = delta_F[2] * m_FE[3];
//    float t40 = delta_F[5] * m_FE[4];
//    float t41 = delta_F[8] * m_FE[5];
//    float t42 = t41 + t39 + t40;
//    float t43 = F[0] * F[4] - F[3] * F[1];
//    float t44 = delta_F[2] * m_FE[6];
//    float t45 = delta_F[5] * m_FE[7];
//    float t46 = delta_F[8] * m_FE[8];
//    float t47 = t44 + t45 + t46;
//    t4 = 1.0f / t4;
//    float t48 = t3 * t9;
//    float t49 = t1 * t17;
//    float t50 = t18 * t22;
//    float t51 = t28 * t32;
//    float t52 = t33 * t37;
//    float t53 = t43 * t47;
//    float t54 = t23 * t27;
//    float t55 = t38 * t42;
//    float t56 = t2 * t13;
//    float t57 = a * (-t54 + t55 + t56 - t53 + t50 + t51 - t52 - t48 - t49) * t4;
//    t6 = -t57 * F[0] + t6 + t7 + t8;
//    t7 = SE[0] + SE[4] + SE[8];
//    t8 = powf(SE[3], 2.0f);
//    float t58 = SE[0] * SE[4];
//    float t59 = SE[0] + SE[4];
//    float t60 = powf(SE[7], 2.0f);
//    float t61 = powf(SE[0], 2.0f);
//    float t62 = powf(SE[6], 2.0f);
//    float t63 = SE[3] * SE[6];
//    float t64 = 2;
//    t61 = t64 * (t58 * SE[8] - t63 * SE[7]) + t59 * powf(SE[8], 2.0f) - t59 * t8 - t62 * SE[0] +
//          (t58 - t60 + t61) * SE[4] + (powf(SE[4], 2.0f) - t60 + t61 - t62) * SE[8];
//    t10 = -t57 * F[3] + t10 + t11 + t12;
//    t11 = -t57 * F[4] + t24 + t25 + t26;
//    t12 = t57 * F[5] - t39 - t40 - t41;
//    t19 = t57 * F[1] - t19 - t20 - t21;
//    t20 = t57 * F[2] - t34 - t35 - t36;
//    t21 = (-t10 * RE[0] - t11 * RE[1] + t12 * RE[2] - t19 * RE[4] - t20 * RE[5] + t6 * RE[3]) * t5;
//    t24 = SE[4] + SE[8];
//    t25 = t24 * SE[7] + t63;
//    t14 = t57 * F[6] - t14 - t15 - t16;
//    t15 = t57 * F[7] - t29 - t30 - t31;
//    t16 = t57 * F[8] - t44 - t45 - t46;
//    t26 = (t14 * RE[0] + t15 * RE[1] + t16 * RE[2] - t19 * RE[7] - t20 * RE[8] + t6 * RE[6]) * t5;
//    t29 = SE[0] + SE[8];
//    t30 = t29 * SE[6] + SE[3] * SE[7];
//    t31 = (t10 * RE[6] + t11 * RE[7] - t12 * RE[8] + t14 * RE[3] + t15 * RE[4] + t16 * RE[5]) * t5;
//    t34 = 1.0f / t61;
//    t8 = ((t7 * SE[8] + t58 - t8) * t21 - t25 * t26 + t30 * t31) * t34;
//    t35 = t59 * SE[3] + SE[6] * SE[7];
//    t25 = (t21 * t25 - t26 * (t7 * SE[4] + SE[0] * SE[8] - t62) + t31 * t35) * t34;
//    t6 = t25 * RE[6] + t5 * t6 - t8 * RE[3];
//    t7 = (t21 * t30 - t26 * t35 + t31 * (t7 * SE[0] + SE[4] * SE[8] - t60)) * t34;
//    t10 = t10 * t5 - t7 * RE[6] + t8 * RE[0];
//    t14 = t14 * t5 + t25 * RE[0] - t7 * RE[3];
//    t19 = t19 * t5 - t25 * RE[7] + t8 * RE[4];
//    t11 = t11 * t5 - t7 * RE[7] + t8 * RE[1];
//    t15 = t15 * t5 + t25 * RE[1] - t7 * RE[4];
//    t20 = -t20 * t5 + t25 * RE[8] - t8 * RE[5];
//    t8 = t12 * t5 + t7 * RE[8] - t8 * RE[2];
//    t7 = t16 * t5 + t25 * RE[2] - t7 * RE[5];
//    t12 = (-t10 * F[3] - t11 * F[4] + t14 * F[6] + t15 * F[7] + t19 * F[1] - t20 * F[2] - t6 * F[0] + t7 * F[8] +
//           t8 * F[5]) *
//          current_mu;
//    t16 = t3 * t4;
//    t21 = t16 * a;
//    t25 = -t17 * t59 - t24 * t37 - t27 * t29 + (t22 + t42) * SE[3] + (t9 + t47) * SE[6] + (t13 + t32) * SE[7];
//    t24 = -t24 * F[2] - t29 * F[4] - t59 * F[6] + (F[1] + F[5]) * SE[3] + (F[0] + F[8]) * SE[6] + (F[3] + F[7]) *
//    SE[7];
//    t26 = t4 * t24;
//    t29 = t26 * t5 * a;
//    t30 = t29 * (-t56 + t48 + t49);
//    t31 = t29 * (t1 * t32 - t2 * t27 + t22 * t3);
//    t34 = t29 * (t1 * t47 - t2 * t42 + t3 * t37);
//    t35 = t64 * t5;
//    t6 = t4 * (t18 * t31 - t3 * t30 - t33 * t34) + (t21 * t24 + t16 + SE[6]) * t25 * t5 * a +
//         t35 * (-t21 * t12 + t6 * current_mu);
//    t16 = t2 * t4;
//    t21 = t26 * a;
//    t10 = t35 * (t16 * a * t12 + t10 * current_mu) - t4 * (-t2 * t30 + t23 * t31 - t34 * t38) -
//          (t21 * t2 + t16 - SE[7]) * t25 * t5 * a;
//    t16 = t1 * t4;
//    t14 = -t35 * (t16 * a * t12 + t14 * current_mu) + t4 * (-t1 * t30 + t28 * t31 - t34 * t43) +
//          (t21 * t1 + t16 - SE[0] - SE[4]) * t25 * t5 * a;
//    t16 = t18 * t4;
//    t24 = t29 * (-t13 * t23 + t17 * t28 + t18 * t9);
//    t26 = t29 * (-t54 + t50 + t51);
//    t30 = t29 * (t18 * t37 - t23 * t42 + t28 * t47);
//    t16 = t35 * (t16 * a * t12 - t19 * current_mu) - t4 * (t18 * t26 - t24 * t3 - t30 * t33) -
//          (t21 * t18 + t16 - SE[3]) * t25 * t5 * a;
//    t19 = t23 * t4;
//    t11 = -t35 * (t19 * a * t12 - t11 * current_mu) + t4 * (-t2 * t24 + t23 * t26 - t30 * t38) +
//          (t21 * t23 + t19 - SE[0] - SE[8]) * t25 * t5 * a;
//    t19 = t28 * t4;
//    t15 = t35 * (t19 * a * t12 - t15 * current_mu) - t4 * (-t1 * t24 + t26 * t28 - t30 * t43) -
//          (t21 * t28 + t19 - SE[7]) * t25 * t5 * a;
//    t19 = t33 * t4;
//    t9 = t29 * (-t13 * t38 + t17 * t43 + t33 * t9);
//    t13 = t29 * (t22 * t33 - t27 * t38 + t32 * t43);
//    t17 = t29 * (-t55 + t52 + t53);
//    t3 = -t35 * (t19 * a * t12 - t20 * current_mu) + t4 * (t13 * t18 - t17 * t33 - t3 * t9) +
//         (t21 * t33 + t19 - SE[4] - SE[8]) * t25 * t5 * a;
//    t18 = t38 * t4;
//    t2 = t35 * (t18 * a * t12 - t8 * current_mu) - t4 * (t13 * t23 - t17 * t38 - t2 * t9) -
//         (t21 * t38 + t18 - SE[3]) * t25 * t5 * a;
//    t8 = t43 * t4;
//    t1 = -t4 * (t1 * t9 - t13 * t28 + t17 * t43) - (-t21 * t43 - t8 - SE[6]) * t25 * t5 * a -
//         t35 * (t8 * a * t12 + t7 * current_mu);
//    VAP[0] = (t10 * m_FE[3] + t14 * m_FE[6] + t6 * m_FE[0]) * mvolume;
//    VAP[3] = (t10 * m_FE[4] + t14 * m_FE[7] + t6 * m_FE[1]) * mvolume;
//    VAP[6] = (t10 * m_FE[5] + t14 * m_FE[8] + t6 * m_FE[2]) * mvolume;
//    VAP[1] = (t11 * m_FE[3] + t15 * m_FE[6] + t16 * m_FE[0]) * mvolume;
//    VAP[4] = (t11 * m_FE[4] + t15 * m_FE[7] + t16 * m_FE[1]) * mvolume;
//    VAP[7] = (t11 * m_FE[5] + t15 * m_FE[8] + t16 * m_FE[2]) * mvolume;
//    VAP[2] = (t1 * m_FE[6] + t2 * m_FE[3] + t3 * m_FE[0]) * mvolume;
//    VAP[5] = (t1 * m_FE[7] + t2 * m_FE[4] + t3 * m_FE[1]) * mvolume;
//    VAP[8] = (t1 * m_FE[8] + t2 * m_FE[5] + t3 * m_FE[2]) * mvolume;
//
//    return VAP;
//}
}
