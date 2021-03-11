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
// Author:Arman Pazouki, Milad Rakhsha, Wei Hu
// =============================================================================
// This file contains miscellaneous macros and utilities used in the SPH code.
// =============================================================================

#ifndef CH_SPH_GENERAL_CUH
#define CH_SPH_GENERAL_CUH
// ----------------------------------------------------------------------------
// CUDA headers
// ----------------------------------------------------------------------------
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <device_launch_parameters.h>
#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/utils/ChUtilsDevice.cuh"
#include "chrono_fsi/ChFsiDataManager.cuh"
#include "chrono_fsi/physics/ChParams.cuh"
#include "chrono_fsi/math/ChFsiLinearSolver.h"
#include "chrono_fsi/math/ExactLinearSolvers.cuh"
#include "chrono_fsi/math/custom_math.h"

// #include <cstdio>

namespace chrono {
namespace fsi {

/// I made this const variables static in order to be able to use them in a different translation units in the utils
__constant__ static fsi::SimParams paramsD;
__constant__ static fsi::NumberOfObjects numObjectsD;

void CopyParams_NumberOfObjects(std::shared_ptr<SimParams> paramsH, std::shared_ptr<NumberOfObjects> numObjectsH);
//#define W3 W3_Spline
//#define W2 W2_Spline
//#define GradW GradW_Spline
//
//#define W3h W3H_KERNEL
//#define GradWh W3H_GRADW
//#define W_kappa 3

#define W3h W3h_Spline
#define W3h_GPU W3h_Spline_GPU
#define GradWh GradWh_Spline
// #define W3h W3h_High
// #define GradWh GradWh_High
#define W_kappa 2

//--------------------------------------------------------------------------------------------------------------------------------
// 3D SPH kernel function, W3_SplineA
__device__ inline Real W3_Spline(Real d) {  // d is positive. h is the sph particle radius (i.e. h in
                                            // the document) d is the distance of 2 particles
    Real h = paramsD.HSML;
    Real q = fabs(d) / h;
    if (q < 1) {
        return (0.25f / (PI * h * h * h) * (cube(2 - q) - 4 * cube(1 - q)));
    }
    if (q < 2) {
        return (0.25f / (PI * h * h * h) * cube(2 - q));
    }
    return 0;
}
//--------------------------------------------------------------------------------------------------------------------------------
// 3D SPH kernel function, W3_SplineA
__host__ __device__ inline Real W3h_Spline(Real d, Real h) {  // d is positive. h is the sph particle radius (i.e. h in
                                                              // the document) d is the distance of 2 particles
    Real invh = 1.0f / h;
    Real q = fabs(d) * invh;
    if (q < 1) {
        return (0.25f * (INVPI * invh * invh * invh) * (cube(2 - q) - 4 * cube(1 - q)));
    }
    if (q < 2) {
        return (0.25f * (INVPI * invh * invh * invh) * cube(2 - q));
    }
    return 0;
}
__device__ inline Real W3h_Spline_GPU(Real d, Real h) {  // d is positive. h is the sph particle radius (i.e. h in
                                                         // the document) d is the distance of 2 particles
    Real invh = paramsD.INVHSML;
    Real q = fabs(d) * invh;
    if (q < 1) {
        return (0.25f * (INVPI * invh * invh * invh) * (cube(2 - q) - 4 * cube(1 - q)));
    }
    if (q < 2) {
        return (0.25f * (INVPI * invh * invh * invh) * cube(2 - q));
    }
    return 0;
}
//--------------------------------------------------------------------------------------------------------------------------------
// Johnson kernel 1996b
__host__ __device__ inline Real W3h_High(Real d, Real h) {  // d is positive. h is the sph particle radius (i.e. h in
                                                            // the document) d is the distance of 2 particles
    Real q = fabs(d) / h;
    if (q < 2) {
        return (5.0f / (4.0f * PI * h * h * h) * (3.0 / 16.0 * square(q) - 3.0 / 4.0 * q + 3.0 / 4.0));
    }
    return 0;
}

//// 3D SPH kernel function, W3_SplineA
__device__ inline Real W3H_KERNEL(Real d, Real h) {  // d is positive. h is the sph particle radius (i.e. h in
                                                     // the document) d is the distance of 2 particles
    Real q = fabs(d) / h;
    if (q < 1) {
        return (3.0 / (359 * PI * h * h * h) *
                (pow(3 - q, Real(5)) - 6 * pow(2 - q, Real(5)) + 15 * pow(1 - q, Real(5))));
    }
    if (q < 2) {
        return (3.0 / (359 * PI * h * h * h) * (pow(3 - q, Real(5)) - 6 * pow(2 - q, Real(5))));
    }
    if (q < 3) {
        return (3.0 / (359 * PI * h * h * h) * (pow(3 - q, Real(5))));
    }
    return 0;
}
__device__ inline Real3 W3H_GRADW(Real3 d, Real h) {  // d is positive. h is the sph particle radius (i.e. h in
                                                      // the document) d is the distance of 2 particles
    Real q = length(d) / h;
    if (fabs(q) < 1e-10)
        return mR3(0.0);

    if (q < 1) {
        return (-15.0 / (359 * PI * h * h * h * h * h * q) * d *
                (pow(3 - q, Real(4)) - 6 * pow(2 - q, Real(4)) + 15 * pow(1 - q, Real(4))));
    }
    if (q < 2) {
        return (-15.0 / (359 * PI * h * h * h * h * h * q) * d * (pow(3 - q, Real(4)) - 6 * pow(2 - q, Real(4))));
    }
    if (q < 3) {
        return (-15.0 / (359 * PI * h * h * h * h * h * q) * d * (pow(3 - q, Real(4))));
    }
    return mR3(0);
}

////--------------------------------------------------------------------------------------------------------------------------------
////2D SPH kernel function, W2_SplineA
//__device__ inline Real W2_Spline(Real d) { // d is positive. h is the sph
// particle radius (i.e. h in the document) d
// is the distance of 2 particles
//	Real h = paramsD.HSML;
//	Real q = fabs(d) / h;
//	if (q < 1) {
//		return (5 / (14 * PI * h * h) * (cube(2 - q) - 4 * cube(1 - q)));
//	}
//	if (q < 2) {
//		return (5 / (14 * PI * h * h) * cube(2 - q));
//	}
//	return 0;
//}
////--------------------------------------------------------------------------------------------------------------------------------
////3D SPH kernel function, W3_QuadraticA
//__device__ inline Real W3_Quadratic(Real d, Real h) { // d is positive. h is
// the sph particle radius (i.e. h in the
// document) d is the distance of 2 particles
//	Real q = fabs(d) / h;
//	if (q < 2) {
//		return (1.25f / (PI * h * h * h) * .75f * (square(.5f * q) -
// q + 1));
//	}
//	return 0;
//}
////--------------------------------------------------------------------------------------------------------------------------------
////2D SPH kernel function, W2_QuadraticA
//__device__ inline Real W2_Quadratic(Real d, Real h) { // d is positive. h is
// the sph particle radius (i.e. h in the
// document) d is the distance of 2 particles
//	Real q = fabs(d) / h;
//	if (q < 2) {
//		return (2.0f / (PI * h * h) * .75f * (square(.5f * q) - q +
// 1));
//	}
//	return 0;
//}
//--------------------------------------------------------------------------------------------------------------------------------
// Gradient of the kernel function
// d: magnitude of the distance of the two particles
// dW * dist3 gives the gradiant of W3_Quadratic, where dist3 is the distance
// vector of the two particles, (dist3)a =
// pos_a - pos_b
__device__ inline Real3 GradW_Spline(Real3 d) {  // d is positive. r is the sph particle radius (i.e. h
                                                 // in the document) d is the distance of 2 particles
    Real h = paramsD.HSML;
    Real q = length(d) / h;
    bool less1 = (q < 1);
    bool less2 = (q < 2);
    return (less1 * (3 * q - 4) + less2 * (!less1) * (-q + 4.0f - 4.0f / q)) * .75f * (INVPI)*pow(h, Real(-5)) * d;
    //	if (q < 1) {
    //		return .75f * (INVPI) *pow(h, Real(-5))* (3 * q - 4) * d;
    //	}
    //	if (q < 2) {
    //		return .75f * (INVPI) *pow(h, Real(-5))* (-q + 4.0f - 4.0f / q) *
    // d;
    //	}
    //	return mR3(0);
}

__device__ inline Real3 GradWh_Spline(Real3 d, Real h) {  // d is positive. r is the sph particle radius (i.e. h
                                                          // in the document) d is the distance of 2 particles
    Real q = length(d) * paramsD.INVHSML;
    Real INVHSML5 = paramsD.INVHSML*paramsD.INVHSML*paramsD.INVHSML*paramsD.INVHSML*paramsD.INVHSML;

    if (abs(q) < EPSILON)
        return mR3(0.0);
    bool less1 = (q < 1);
    bool less2 = (q < 2);
    return (less1 * (3 * q - 4.0f) + less2 * (!less1) * (-q + 4.0f - 4.0f / q)) * .75f * INVPI * INVHSML5 * d;
}

__device__ inline Real3 GradWh_High(Real3 d, Real h) {  // d is positive. r is the sph particle radius (i.e. h
                                                        // in the document) d is the distance of 2 particles
    Real q = length(d) / h;
    if (abs(q) < EPSILON)
        return mR3(0.0);
    bool less2 = (q < 2);
    return (3.0 / 8.0 * q - 3.0 / 4.0) * 5.0 / 4.0 / q * (INVPI)*pow(h, Real(-5)) * d * less2;
}

////--------------------------------------------------------------------------------------------------------------------------------
////Gradient of the kernel function
//// d: magnitude of the distance of the two particles
//// dW * dist3 gives the gradiant of W3_Quadratic, where dist3 is the distance
/// vector of the two particles, (dist3)a =
/// pos_a - pos_b
//__device__ inline Real3 GradW_Quadratic(Real3 d, Real h) { // d is positive. r
// is the sph particle radius (i.e. h in
// the document) d is the distance of 2 particles
//	Real q = length(d) / h;
//	if (q < 2) {
//		return 1.25f / (PI * pow(h, Real(5))) * .75f * (.5f - 1.0f / q) *
// d;
//	}
//	return mR3(0);
//}
//--------------------------------------------------------------------------------------------------------------------------------

__device__ inline Real EOS_new(Real rho, Real V_max, Real Min_rho) {
    if (rho < paramsD.rho0)
        rho = paramsD.rho0;

    //    Real gama = 7;
    //    Real B = paramsD.Cs * paramsD.Cs * paramsD.rho0 * V_max * V_max / gama;
    //    return B * (pow(rho / paramsD.rho0, gama) - 1) + paramsD.BASEPRES;

    //    Real B = paramsD.Cs * paramsD.Cs * paramsD.rho0 / gama;
    return paramsD.Cs * paramsD.Cs * (rho / paramsD.rho0 - 1);
}

//--------------------------------------------------------------------------------------------------------------------------------
// Eos is also defined in SDKCollisionSystem.cu
// fluid equation of state
__device__ inline Real Eos(Real rho, Real type) {
    // if (rho < paramsD.rho0) //
    //     rho = paramsD.rho0; //
    //******************************
    // Real gama = 7;
    // Real B = 100 * paramsD.rho0 * paramsD.v_Max * paramsD.v_Max / gama;
    // return B * (pow(rho / paramsD.rho0, gama) - 1) + paramsD.BASEPRES; //
    return paramsD.Cs * paramsD.Cs * (rho - paramsD.rho0);  //
}
//--------------------------------------------------------------------------------------------------------------------------------
__device__ inline Real InvEos(Real pw) {
    // Real gama = 7;
    // Real B = 100 * paramsD.rho0 * paramsD.v_Max * paramsD.v_Max / gama;  // 200;//314e6; //c^2 * paramsD.rho0 / gama
    //                                                                      // where c = 1484 m/s for water
    // Real powerComp = (pw - paramsD.BASEPRES) / B + 1.0;
    // Real rho = (powerComp > 0) ? paramsD.rho0 * pow(powerComp, 1.0 / gama)
    //                            : -paramsD.rho0 * pow(fabs(powerComp),
    //                                                  1.0 / gama);  // did this since CUDA is
    //                                                                // stupid and freaks out by
    //                                                                // negative^(1/gama)
    Real rho = pw / (paramsD.Cs * paramsD.Cs) + paramsD.rho0;  //
    return rho;
}
//--------------------------------------------------------------------------------------------------------------------------------
// ferrariCi
__device__ inline Real FerrariCi(Real rho) {
    int gama = 7;
    Real B = 100 * paramsD.rho0 * paramsD.v_Max * paramsD.v_Max / gama;  // 200;//314e6; //c^2 * paramsD.rho0 / gama
                                                                         // where c = 1484 m/s for water
    return sqrt(gama * B / paramsD.rho0) * pow(rho / paramsD.rho0, 0.5 * (gama - 1));
}
//--------------------------------------------------------------------------------------------------------------------------------

__device__ inline Real3 Modify_Local_PosB(Real3& b, Real3 a) {
    Real3 dist3 = a - b;
    b.x += ((dist3.x > 0.5f * paramsD.boxDims.x) ? paramsD.boxDims.x : 0);
    b.x -= ((dist3.x < -0.5f * paramsD.boxDims.x) ? paramsD.boxDims.x : 0);

    b.y += ((dist3.y > 0.5f * paramsD.boxDims.y) ? paramsD.boxDims.y : 0);
    b.y -= ((dist3.y < -0.5f * paramsD.boxDims.y) ? paramsD.boxDims.y : 0);

    b.z += ((dist3.z > 0.5f * paramsD.boxDims.z) ? paramsD.boxDims.z : 0);
    b.z -= ((dist3.z < -0.5f * paramsD.boxDims.z) ? paramsD.boxDims.z : 0);

    dist3 = a - b;
    // modifying the markers perfect overlap
    Real d = length(dist3);
    if (d < paramsD.epsMinMarkersDis * paramsD.HSML) {
        dist3 = mR3(paramsD.epsMinMarkersDis * paramsD.HSML, 0, 0);
    }
    b = a - dist3;
    return (dist3);
}

/**
 * @brief Distance
 * @details
 *          Distance between two particles, considering the periodic boundary
 * condition
 *
 * @param a Position of Particle A
 * @param b Position of Particle B
 *
 * @return Distance vector (distance in x, distance in y, distance in z)
 */
__device__ inline Real3 Distance(Real3 a, Real3 b) {
    //	Real3 dist3 = a - b;
    //	dist3.x -= ((dist3.x > 0.5f * paramsD.boxDims.x) ? paramsD.boxDims.x :
    // 0);
    //	dist3.x += ((dist3.x < -0.5f * paramsD.boxDims.x) ? paramsD.boxDims.x :
    // 0);
    //
    //	dist3.y -= ((dist3.y > 0.5f * paramsD.boxDims.y) ? paramsD.boxDims.y :
    // 0);
    //	dist3.y += ((dist3.y < -0.5f * paramsD.boxDims.y) ? paramsD.boxDims.y :
    // 0);
    //
    //	dist3.z -= ((dist3.z > 0.5f * paramsD.boxDims.z) ? paramsD.boxDims.z :
    // 0);
    //	dist3.z += ((dist3.z < -0.5f * paramsD.boxDims.z) ? paramsD.boxDims.z :
    // 0);
    return Modify_Local_PosB(b, a);
}

//--------------------------------------------------------------------------------------------------------------------------------
// first comp of q is rotation, last 3 components are axis of rot

__device__ inline void RotationMatirixFromQuaternion(Real3& AD1, Real3& AD2, Real3& AD3, const Real4& q) {
    AD1 = 2 * mR3(0.5f - q.z * q.z - q.w * q.w, q.y * q.z - q.x * q.w, q.y * q.w + q.x * q.z);
    AD2 = 2 * mR3(q.y * q.z + q.x * q.w, 0.5f - q.y * q.y - q.w * q.w, q.z * q.w - q.x * q.y);
    AD3 = 2 * mR3(q.y * q.w - q.x * q.z, q.z * q.w + q.x * q.y, 0.5f - q.y * q.y - q.z * q.z);
}
//--------------------------------------------------------------------------------------------------------------------------------

__device__ inline Real3 InverseRotate_By_RotationMatrix_DeviceHost(const Real3& A1,
                                                                   const Real3& A2,
                                                                   const Real3& A3,
                                                                   const Real3& r3) {
    return mR3(A1.x * r3.x + A2.x * r3.y + A3.x * r3.z, A1.y * r3.x + A2.y * r3.y + A3.y * r3.z,
               A1.z * r3.x + A2.z * r3.y + A3.z * r3.z);
}
//--------------------------------------------------------------------------------------------------------------------------------

/**
 * @brief calcGridHash
 * @details  See SDKCollisionSystem.cuh
 */
__device__ inline int3 calcGridPos(Real3 p) {
    int3 gridPos;
    if (paramsD.cellSize.x * paramsD.cellSize.y * paramsD.cellSize.z == 0)
        printf("calcGridPos=%f,%f,%f\n", paramsD.cellSize.x, paramsD.cellSize.y, paramsD.cellSize.z);

    gridPos.x = (int)floor((p.x - paramsD.worldOrigin.x) / paramsD.cellSize.x);
    gridPos.y = (int)floor((p.y - paramsD.worldOrigin.y) / paramsD.cellSize.y);
    gridPos.z = (int)floor((p.z - paramsD.worldOrigin.z) / paramsD.cellSize.z);
    return gridPos;
}

/**
 * @brief calcGridHash
 * @details  See SDKCollisionSystem.cuh
 */

__device__ inline uint calcGridHash(int3 gridPos) {
    gridPos.x -= ((gridPos.x >= paramsD.gridSize.x) ? paramsD.gridSize.x : 0);
    gridPos.y -= ((gridPos.y >= paramsD.gridSize.y) ? paramsD.gridSize.y : 0);
    gridPos.z -= ((gridPos.z >= paramsD.gridSize.z) ? paramsD.gridSize.z : 0);

    gridPos.x += ((gridPos.x < 0) ? paramsD.gridSize.x : 0);
    gridPos.y += ((gridPos.y < 0) ? paramsD.gridSize.y : 0);
    gridPos.z += ((gridPos.z < 0) ? paramsD.gridSize.z : 0);

    return gridPos.z * paramsD.gridSize.y * paramsD.gridSize.x + gridPos.y * paramsD.gridSize.x + gridPos.x;
}

////--------------------------------------------------------------------------------------------------------------------------------
inline __device__ Real Strain_Rate(Real3 grad_ux, Real3 grad_uy, Real3 grad_uz) {
    grad_ux.y = (grad_uy.x + grad_ux.y) * 0.5;
    grad_ux.z = (grad_uz.x + grad_ux.z) * 0.5;

    grad_uy.x = grad_ux.y;
    grad_uy.z = (grad_uy.z + grad_uz.y) * 0.5;

    grad_uz.x = grad_ux.z;
    grad_uz.y = grad_uy.z;

    return sqrt(                                    //
        0.5 * (length(grad_ux) * length(grad_ux) +  //
               length(grad_uy) * length(grad_uy) +  //
               length(grad_uz) * length(grad_uz))   //
    );
}

////--------------------------------------------------------------------------------------------------------------------------------
inline __device__ Real Tensor_Norm(Real* T) {
    return sqrt(                                          //
        0.5 * (T[0] * T[0] + T[1] * T[1] + T[2] * T[2] +  //
               T[3] * T[3] + T[4] * T[4] + T[5] * T[5] +  //
               T[6] * T[6] + T[7] * T[7] + T[8] * T[8])   //
    );
}
////--------------------------------------------------------------------------------------------------------------------------------
inline __device__ Real Sym_Tensor_Norm(Real3 xx_yy_zz, Real3 xy_xz_yz) {
    return sqrt(0.5 * (xx_yy_zz.x * xx_yy_zz.x + xx_yy_zz.y * xx_yy_zz.y + xx_yy_zz.z * xx_yy_zz.z +
                       2 * xy_xz_yz.x * xy_xz_yz.x + 2 * xy_xz_yz.y * xy_xz_yz.y + 2 * xy_xz_yz.z * xy_xz_yz.z));
}
////--------------------------------------------------------------------------------------------------------------------------------
inline __device__ Real Inertia_num(Real Strain_rate, Real rho, Real p, Real diam) {
    Real I = Strain_rate * diam * sqrt(rho / rmaxr(p, EPSILON));
    return rminr(1e3, I);
}

////--------------------------------------------------------------------------------------------------------------------------------
inline __device__ Real mu_I(Real Strain_rate, Real I) {
    Real mu = 0;
    if (paramsD.mu_of_I == friction_law::constant)
        mu = paramsD.mu_fric_s;
    else if (paramsD.mu_of_I == friction_law::linear)
        mu = paramsD.mu_fric_s + paramsD.mu_I_b * I;
    else
        mu = paramsD.mu_fric_s + (paramsD.mu_fric_2 - paramsD.mu_fric_s) * (I / (paramsD.mu_I0 + I));

    return mu;
}
////--------------------------------------------------------------------------------------------------------------------------------
inline __device__ Real mu_eff(Real Strain_rate, Real p, Real mu_I) {
    return rmaxr(mu_I * rmaxr(p, 0.0) / Strain_rate, paramsD.mu_max);
}

////--------------------------------------------------------------------------------------------------------------------------------
inline __device__ Real Herschel_Bulkley_stress(Real Strain_rate, Real k, Real n, Real tau0) {
    Real tau = tau0 + k * pow(Strain_rate, n);
    return tau;
}
////--------------------------------------------------------------------------------------------------------------------------------
inline __device__ Real Herschel_Bulkley_mu_eff(Real Strain_rate, Real k, Real n, Real tau0) {
    Real mu_eff = tau0 / Strain_rate + k * pow(Strain_rate, n - 1);
    return rminr(mu_eff, paramsD.mu_max);
}
////--------------------------------------------------------------------------------------------------------------------------------
inline __device__ void BCE_Vel_Acc(int i_idx,
                                   Real3& myAcc,
                                   Real3& V_prescribed,

                                   Real4* sortedPosRad,
                                   int4 updatePortion,
                                   uint* gridMarkerIndexD,

                                   Real4* qD,
                                   Real3* rigidSPH_MeshPos_LRF_D,
                                   Real3* posRigid_fsiBodies_D,
                                   Real4* velMassRigid_fsiBodies_D,
                                   Real3* omegaVelLRF_fsiBodies_D,
                                   Real3* accRigid_fsiBodies_D,
                                   Real3* omegaAccLRF_fsiBodies_D,
                                   uint* rigidIdentifierD,

                                   Real3* pos_fsi_fea_D,
                                   Real3* vel_fsi_fea_D,
                                   Real3* acc_fsi_fea_D,
                                   uint* FlexIdentifierD,
                                   const int numFlex1D,
                                   uint2* CableElementsNodes,
                                   uint4* ShellelementsNodes) {
    int Original_idx = gridMarkerIndexD[i_idx];

    // See if this belongs to a fixed boundary
    if (Original_idx >= updatePortion.x && Original_idx < updatePortion.y) {
        myAcc = mR3(0.0);
        V_prescribed = mR3(0.0);
        if (paramsD.Apply_BC_U)
            V_prescribed = user_BC_U(mR3(sortedPosRad[i_idx]));
    } else if (Original_idx >= updatePortion.y && Original_idx < updatePortion.z) {
        int rigidIndex = rigidIdentifierD[Original_idx - updatePortion.y];

        Real4 q4 = qD[rigidIndex];
        Real3 a1, a2, a3;
        RotationMatirixFromQuaternion(a1, a2, a3, q4);
        Real3 rigidSPH_MeshPos_LRF__ = rigidSPH_MeshPos_LRF_D[Original_idx - updatePortion.y];

        Real3 p_com = mR3(posRigid_fsiBodies_D[rigidIndex]);
        Real3 v_com = mR3(velMassRigid_fsiBodies_D[rigidIndex]);
        Real3 a_com = accRigid_fsiBodies_D[rigidIndex];
        Real3 angular_v_com = omegaVelLRF_fsiBodies_D[rigidIndex];
        Real3 angular_a_com = omegaAccLRF_fsiBodies_D[rigidIndex];
        Real3 p_rel = mR3(sortedPosRad[i_idx]) - p_com;
        Real3 omegaCrossS = cross(angular_v_com, rigidSPH_MeshPos_LRF__);
        V_prescribed = v_com + mR3(dot(a1, omegaCrossS), dot(a2, omegaCrossS), dot(a3, omegaCrossS));
        //        V_prescribed = v_com + cross(angular_v_com, rigidSPH_MeshPos_LRF);

        Real3 alphaCrossS = cross(angular_a_com, rigidSPH_MeshPos_LRF__);
        Real3 alphaCrossScrossS = cross(angular_v_com, cross(angular_v_com, rigidSPH_MeshPos_LRF__));
        //        myAcc = a_com + cross(angular_a_com, p_rel) + cross(angular_v_com, cross(angular_v_com,
        //        rigidSPH_MeshPos_LRF__));

        myAcc = a_com + mR3(dot(a1, alphaCrossS), dot(a2, alphaCrossS), dot(a3, alphaCrossS)) +
                mR3(dot(a1, alphaCrossScrossS), dot(a2, alphaCrossScrossS), dot(a3, alphaCrossScrossS));

        // Or not, Flexible bodies for sure
    } else if (Original_idx >= updatePortion.z && Original_idx < updatePortion.w) {
        int FlexIndex = FlexIdentifierD[Original_idx - updatePortion.z];

        if (FlexIndex < numFlex1D) {
            int nA = CableElementsNodes[FlexIndex].x;
            int nB = CableElementsNodes[FlexIndex].y;

            Real3 pos_fsi_fea_D_nA = pos_fsi_fea_D[nA];
            Real3 pos_fsi_fea_D_nB = pos_fsi_fea_D[nB];

            Real3 vel_fsi_fea_D_nA = vel_fsi_fea_D[nA];
            Real3 vel_fsi_fea_D_nB = vel_fsi_fea_D[nB];

            Real3 acc_fsi_fea_D_nA = acc_fsi_fea_D[nA];
            Real3 acc_fsi_fea_D_nB = acc_fsi_fea_D[nB];

            Real3 dist3 = mR3(sortedPosRad[i_idx]) - pos_fsi_fea_D_nA;
            Real3 x_dir = (pos_fsi_fea_D_nB - pos_fsi_fea_D_nA);
            Real Cable_x = length(x_dir);
            x_dir = x_dir / length(x_dir);
            Real dx = dot(dist3, x_dir);

            Real2 N_cable = Cables_ShapeFunctions(dx / Cable_x);
            Real NA = N_cable.x;
            Real NB = N_cable.y;

            V_prescribed = NA * vel_fsi_fea_D_nA + NB * vel_fsi_fea_D_nB;
            myAcc = NA * acc_fsi_fea_D_nA + NB * acc_fsi_fea_D_nB;

            //            printf("FlexIndex=%d, CableElementsNodes=%d,%d, NA=%f, Nb=%f, xi=%f\n", FlexIndex, nA, nB, NA,
            //            NB,
            //                   dx / Cable_x);
        }
        if (FlexIndex >= numFlex1D) {
            int nA = ShellelementsNodes[FlexIndex - numFlex1D].x;
            int nB = ShellelementsNodes[FlexIndex - numFlex1D].y;
            int nC = ShellelementsNodes[FlexIndex - numFlex1D].z;
            int nD = ShellelementsNodes[FlexIndex - numFlex1D].w;

            Real3 pos_fsi_fea_D_nA = pos_fsi_fea_D[nA];
            Real3 pos_fsi_fea_D_nB = pos_fsi_fea_D[nB];
            Real3 pos_fsi_fea_D_nC = pos_fsi_fea_D[nC];
            Real3 pos_fsi_fea_D_nD = pos_fsi_fea_D[nD];

            Real3 vel_fsi_fea_D_nA = vel_fsi_fea_D[nA];
            Real3 vel_fsi_fea_D_nB = vel_fsi_fea_D[nB];
            Real3 vel_fsi_fea_D_nC = vel_fsi_fea_D[nC];
            Real3 vel_fsi_fea_D_nD = vel_fsi_fea_D[nD];

            Real3 acc_fsi_fea_D_nA = acc_fsi_fea_D[nA];
            Real3 acc_fsi_fea_D_nB = acc_fsi_fea_D[nB];
            Real3 acc_fsi_fea_D_nC = acc_fsi_fea_D[nC];
            Real3 acc_fsi_fea_D_nD = acc_fsi_fea_D[nD];

            Real3 Shell_center = 0.25 * (pos_fsi_fea_D_nA + pos_fsi_fea_D_nB + pos_fsi_fea_D_nC + pos_fsi_fea_D_nD);

            // Note that this must be the i_idx itself not the Original_idx
            Real3 dist3 = mR3(sortedPosRad[i_idx]) - Shell_center;

            Real Shell_x =
                0.25 * (length(pos_fsi_fea_D_nB - pos_fsi_fea_D_nA) + length(pos_fsi_fea_D_nC - pos_fsi_fea_D_nD));
            Real Shell_y =
                0.25 * (length(pos_fsi_fea_D_nD - pos_fsi_fea_D_nA) + length(pos_fsi_fea_D_nC - pos_fsi_fea_D_nB));

            Real2 FlexSPH_MeshPos_Natural = mR2(dist3.x / Shell_x, dist3.y / Shell_y);

            //            if (FlexIndex == -1) {
            //                printf("FlexIndex=%d, Original_idx=%d, dist3=%f,%f,%f, Shell_x,y=%f,%f\n", FlexIndex,
            //                Original_idx,
            //                       dist3.x, dist3.y, dist3.z, Shell_x, Shell_y);
            //
            //                printf(
            //                    "FlexIndex=%d, Original_idx=%d, ShellElementsNodes[FlexIndex]=%d,%d,%d,%d,Shell_center
            //                    = %f, %f, "
            //                    "%f, "
            //                    "Ni = %f, %f\n",
            //                    FlexIndex, Original_idx, nA, nB, nC, nD, Shell_center.x, Shell_center.y,
            //                    Shell_center.z, FlexSPH_MeshPos_Natural.x, FlexSPH_MeshPos_Natural.y);
            //            }

            Real4 N_shell = Shells_ShapeFunctions(FlexSPH_MeshPos_Natural.x, FlexSPH_MeshPos_Natural.y);
            Real NA = N_shell.x;
            Real NB = N_shell.y;
            Real NC = N_shell.z;
            Real ND = N_shell.w;
            V_prescribed =
                NA * vel_fsi_fea_D_nA + NB * vel_fsi_fea_D_nB + NC * vel_fsi_fea_D_nC + ND * vel_fsi_fea_D_nD;
            myAcc = NA * acc_fsi_fea_D_nA + NB * acc_fsi_fea_D_nB + NC * acc_fsi_fea_D_nC + ND * acc_fsi_fea_D_nD;

            //            if (length(V_prescribed) > 18e-6) {
            //                //                printf("i_idx=%d, myAcc:(%f,%f,%f) V_prescribed:(%f,%f,%f) \n", i_idx,
            //                myAcc.x,
            //                //                myAcc.y, myAcc.z,
            //                //                       V_prescribed.x, V_prescribed.y, V_prescribed.z);
            //                //                printf("FlexIndex=%d  FlexSPH_MeshPos_LRF_D[index]=%f,%f,
            //                length(V_prescribed)=%f\n",
            //                //                FlexIndex,
            //                //                       FlexSPH_MeshPos_Natural.x, FlexSPH_MeshPos_Natural.y,
            //                length(V_prescribed));
            //            }
        }
    } else {
        printf("i_idx=%d, Original_idx:%d was not found \n\n", i_idx, Original_idx);
    }
}
//--------------------------------------------------------------------------------------------------------------------------------
inline __device__ void grad_scalar(int i_idx,
                                   Real4* sortedPosRad,  // input: sorted positions
                                   Real4* sortedRhoPreMu,
                                   Real* sumWij_inv,
                                   Real* G_i,
                                   Real4* Scalar,
                                   Real3& myGrad,
                                   uint* cellStart,
                                   uint* cellEnd) {
    // Note that this function only calculates the gradient of the first element of the Scalar;
    // This is hard coded like this for now because usually rho appears in Real4 structure
    Real3 posRadA = mR3(sortedPosRad[i_idx]);
    Real h_i = sortedPosRad[i_idx].w;
    int3 gridPos = calcGridPos(posRadA);

    //    printf("update G[%d]= %f,%f,%f  %f,%f,%f, %f,%f,%f\n", i_idx, G_i[i_idx * 9 + 0], G_i[i_idx * 9 + 1],
    //           G_i[i_idx * 9 + 2], G_i[i_idx * 9 + 3], G_i[i_idx * 9 + 4], G_i[i_idx * 9 + 5], G_i[i_idx * 9 + 6],
    //           G_i[i_idx * 9 + 7], G_i[i_idx * 9 + 8]);

    // This is the elements of inverse of G
    Real mGi[9];
    for (int n = 0; n < 9; n++)
        mGi[n] = G_i[i_idx * 9 + n];

    Real3 grad_si = mR3(0.);
    for (int z = -1; z <= 1; z++) {
        for (int y = -1; y <= 1; y++) {
            for (int x = -1; x <= 1; x++) {
                int3 neighbourPos = gridPos + mI3(x, y, z);
                uint gridHash = calcGridHash(neighbourPos);
                // get start of bucket for this cell50
                uint startIndex = cellStart[gridHash];
                if (startIndex != 0xffffffff) {  // cell is not empty
                    uint endIndex = cellEnd[gridHash];

                    for (uint j = startIndex; j < endIndex; j++) {
                        Real3 posRadB = mR3(sortedPosRad[j]);
                        Real3 dist3 = Distance(posRadA, posRadB);
                        Real d = length(dist3);
                        if (d > RESOLUTION_LENGTH_MULT * h_i || sortedRhoPreMu[j].w <= -2)
                            continue;

                        Real h_j = sortedPosRad[j].w;
                        Real h_ij = 0.5 * (h_j + h_i);
                        Real W3 = W3h(d, h_ij);
                        Real3 grad_i_wij = GradWh(dist3, h_ij);
                        Real V_j = sumWij_inv[j];
                        Real3 common_part = mR3(0);
                        common_part.x = grad_i_wij.x * mGi[0] + grad_i_wij.y * mGi[1] + grad_i_wij.z * mGi[2];
                        common_part.y = grad_i_wij.x * mGi[3] + grad_i_wij.y * mGi[4] + grad_i_wij.z * mGi[5];
                        common_part.z = grad_i_wij.x * mGi[6] + grad_i_wij.y * mGi[7] + grad_i_wij.z * mGi[8];
                        grad_si += common_part * (Scalar[j].x - Scalar[i_idx].x) * V_j;
                    }
                }
            }
        }
    }
    myGrad = grad_si;
    //    printf("grad_scalar[%d]= %f,%f,%f\n", i_idx, myGrad.x, myGrad.y, myGrad.z);
}

//--------------------------------------------------------------------------------------------------------------------------------
inline __device__ void grad_vector(int i_idx,
                                   Real4* sortedPosRad,  // input: sorted positions
                                   Real4* sortedRhoPreMu,
                                   Real* sumWij_inv,
                                   Real* G_i,
                                   Real3* Vector,
                                   Real3& myGradx,
                                   Real3& myGrady,
                                   Real3& myGradz,
                                   uint* cellStart,
                                   uint* cellEnd) {
    Real3 posRadA = mR3(sortedPosRad[i_idx]);
    Real h_i = sortedPosRad[i_idx].w;
    int3 gridPos = calcGridPos(posRadA);

    // This is the elements of inverse of G
    Real mGi[9] = {0.0};
    for (int i = 0; i < 9; i++)
        mGi[i] = G_i[i_idx * 9 + i];

    Real3 common_part = mR3(0.);
    Real3 grad_Vx = mR3(0.);
    Real3 grad_Vy = mR3(0.);
    Real3 grad_Vz = mR3(0.);

    for (int z = -1; z <= 1; z++) {
        for (int y = -1; y <= 1; y++) {
            for (int x = -1; x <= 1; x++) {
                int3 neighbourPos = gridPos + mI3(x, y, z);
                uint gridHash = calcGridHash(neighbourPos);
                // get start of bucket for this cell50
                uint startIndex = cellStart[gridHash];
                if (startIndex != 0xffffffff) {  // cell is not empty
                    uint endIndex = cellEnd[gridHash];

                    for (uint j = startIndex; j < endIndex; j++) {
                        Real3 posRadB = mR3(sortedPosRad[j]);
                        Real3 dist3 = Distance(posRadA, posRadB);
                        Real d = length(dist3);
                        if (d > RESOLUTION_LENGTH_MULT * h_i || sortedRhoPreMu[j].w <= -2)
                            continue;

                        Real h_j = sortedPosRad[j].w;
                        Real h_ij = 0.5 * (h_j + h_i);
                        Real W3 = W3h(d, h_ij);
                        Real3 grad_i_wij = GradWh(dist3, h_ij);
                        Real V_j = sumWij_inv[j];
                        common_part.x = grad_i_wij.x * mGi[0] + grad_i_wij.y * mGi[1] + grad_i_wij.z * mGi[2];
                        common_part.y = grad_i_wij.x * mGi[3] + grad_i_wij.y * mGi[4] + grad_i_wij.z * mGi[5];
                        common_part.z = grad_i_wij.x * mGi[6] + grad_i_wij.y * mGi[7] + grad_i_wij.z * mGi[8];
                        grad_Vx += common_part * (Vector[i_idx].x - Vector[j].x) * V_j;
                        grad_Vy += common_part * (Vector[i_idx].y - Vector[j].y) * V_j;
                        grad_Vz += common_part * (Vector[i_idx].z - Vector[j].z) * V_j;
                    }
                }
            }
        }
    }
    myGradx = grad_Vx;
    myGrady = grad_Vy;
    myGradz = grad_Vz;
    //    printf("grad_vector[%d]= %f,%f,%f  %f,%f,%f, %f,%f,%f\n", i_idx, myGradx.x, myGradx.y, myGradx.z, myGrady.x,
    //           myGrady.y, myGrady.z, myGradz.x, myGradz.y, myGradz.z);
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void calc_A_tensor(Real* A_tensor,
                              Real* G_tensor,
                              Real4* sortedPosRad,
                              Real4* sortedRhoPreMu,
                              Real* sumWij_inv,
                              uint* cellStart,
                              uint* cellEnd,
                              const size_t numAllMarkers,
                              volatile bool* isErrorD);
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void calc_L_tensor(Real* A_tensor,
                              Real* L_tensor,
                              Real* G_tensor,
                              Real4* sortedPosRad,
                              Real4* sortedRhoPreMu,
                              Real* sumWij_inv,
                              uint* cellStart,
                              uint* cellEnd,
                              const size_t numAllMarkers,
                              volatile bool* isErrorD);

//--------------------------------------------------------------------------------------------------------------------------------
__global__ void calcRho_kernel(Real4* sortedPosRad,  // input: sorted positionsmin(
                               Real4* sortedRhoPreMu,
                               Real* sumWij_inv,
                               uint* cellStart,
                               uint* cellEnd,
                               uint* mynumContact,
                               const size_t numAllMarkers,
                               volatile bool* isErrorD);

//--------------------------------------------------------------------------------------------------------------------------------
__global__ void calcNormalizedRho_kernel(Real4* sortedPosRad,  // input: sorted positions
                                         Real3* sortedVelMas,
                                         Real4* sortedRhoPreMu,
                                         Real* sumWij_inv,
                                         Real* G_i,
                                         Real3* normals,
                                         Real* Color,
                                         uint* cellStart,
                                         uint* cellEnd,
                                         const size_t numAllMarkers,
                                         volatile bool* isErrorD);
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void calcNormalizedRho_Gi_fillInMatrixIndices(Real4* sortedPosRad,  // input: sorted positions
                                                         Real3* sortedVelMas,
                                                         Real4* sortedRhoPreMu,
                                                         Real* sumWij_inv,
                                                         Real* G_i,
                                                         Real3* normals,
                                                         uint* csrColInd,
                                                         uint* numContacts,
                                                         uint* cellStart,
                                                         uint* cellEnd,
                                                         const size_t numAllMarkers,
                                                         volatile bool* isErrorD);
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Function_Gradient_Laplacian_Operator(Real4* sortedPosRad,  // input: sorted positions
                                                     Real3* sortedVelMas,
                                                     Real4* sortedRhoPreMu,
                                                     Real* sumWij_inv,
                                                     Real* G_tensor,
                                                     Real* L_tensor,
                                                     Real* A_L,   // velocity Laplacian matrix;
                                                     Real3* A_G,  // This is a matrix in a way that A*p gives the gradp
                                                     Real* A_f,
                                                     uint* csrColInd,
                                                     uint* numContacts,
                                                     const size_t numAllMarkers,
                                                     volatile bool* isErrorD);
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Jacobi_SOR_Iter(Real4* sortedRhoPreMu,
                                Real* A_Matrix,
                                Real3* V_old,
                                Real3* V_new,
                                Real3* b3vec,
                                Real* q_old,  // q=p^(n+1)-p^n
                                Real* q_new,  // q=p^(n+1)-p^n
                                Real* b1vec,
                                const uint* csrColInd,
                                const uint* numContacts,
                                size_t numAllMarkers,
                                bool _3dvector,
                                volatile bool* isErrorD);
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Update_AND_Calc_Res(Real4* sortedRhoPreMu,
                                    Real3* V_old,
                                    Real3* V_new,
                                    Real* q_old,
                                    Real* q_new,
                                    Real* Residuals,
                                    const size_t numAllMarkers,
                                    bool _3dvector,
                                    volatile bool* isErrorD);
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Initialize_Variables(Real4* sortedRhoPreMu,
                                     Real* p_old,
                                     Real3* sortedVelMas,
                                     Real3* V_new,
                                     const size_t numAllMarkers,
                                     volatile bool* isErrorD);

//--------------------------------------------------------------------------------------------------------------------------------
__global__ void UpdateDensity(Real3* vis_vel,
                              Real3* XSPH_Vel,
                              Real3* new_vel,       // Write
                              Real4* sortedPosRad,  // Read
                              Real4* sortedRhoPreMu,
                              Real* sumWij_inv,
                              uint* cellStart,
                              uint* cellEnd,
                              size_t numAllMarkers,
                              volatile bool* isErrorD);

}  // namespace fsi
}  // namespace chrono
#endif
