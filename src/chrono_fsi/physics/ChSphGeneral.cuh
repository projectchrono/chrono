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

#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <device_launch_parameters.h>

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/utils/ChUtilsDevice.cuh"
#include "chrono_fsi/physics/ChSystemFsi_impl.cuh"
#include "chrono_fsi/physics/ChParams.h"
#include "chrono_fsi/math/ChFsiLinearSolver.h"
#include "chrono_fsi/math/custom_math.h"

namespace chrono {
namespace fsi {

/// Declared as const variables static in order to be able to use them in a different translation units in the utils
__constant__ static SimParams paramsD;
__constant__ static ChCounters numObjectsD;

/// Short define of the kernel function
#define W3h W3h_Spline

/// Short define of the kernel function gradient
#define GradWh GradWh_Spline

void CopyParams_NumberOfObjects(std::shared_ptr<SimParams> paramsH, std::shared_ptr<ChCounters> numObjectsH);

// 3D kernel function
//--------------------------------------------------------------------------------------------------------------------------------
// Cubic Spline SPH kernel function
__device__ inline Real W3h_Spline(Real d, Real h) {  // d is positive. h is the sph kernel length (i.e. h in
                                                     // the document) d is the distance of 2 particles
    Real invh = paramsD.INVHSML;
    Real q = fabs(d) * invh;
    if (q < 1) {
        return (0.25f * (INVPI * cube(invh)) * (cube(2 - q) - 4 * cube(1 - q)));
    }
    if (q < 2) {
        return (0.25f * (INVPI * cube(invh)) * cube(2 - q));
    }
    return 0;
}
//--------------------------------------------------------------------------------------------------------------------------------
// Johnson kernel 1996b
__device__ inline Real W3h_High(Real d, Real h) {  // d is positive. h is the sph kernel length (i.e. h in
                                                   // the document) d is the distance of 2 particles
    Real invh = paramsD.INVHSML;
    Real q = fabs(d) * invh;
    if (q < 2) {
        return (1.25f * (INVPI * cube(invh)) * (0.1875f * square(q) - 0.75f * q + 0.75f));
    }
    return 0;
}
//--------------------------------------------------------------------------------------------------------------------------------
// Quintic Spline SPH kernel function
__device__ inline Real W3h_Quintic(Real d, Real h) {  // d is positive. h is the sph kernel length (i.e. h in
                                                      // the document) d is the distance of 2 particles
    Real invh = paramsD.INVHSML;
    Real q = fabs(d) * invh;
    Real coeff = 8.35655e-3;  // 3/359
    if (q < 1) {
        return (coeff * INVPI * cube(invh) * (quintic(3 - q) - 6 * quintic(2 - q) + 15 * quintic(1 - q)));
    }
    if (q < 2) {
        return (coeff * INVPI * cube(invh) * (quintic(3 - q) - 6 * quintic(2 - q)));
    }
    if (q < 3) {
        return (coeff * INVPI * cube(invh) * (quintic(3 - q)));
    }
    return 0;
}

// Gradient of the kernel function
//--------------------------------------------------------------------------------------------------------------------------------
// Gradient of Cubic Spline SPH kernel function
__device__ inline Real3 GradWh_Spline(Real3 d, Real h) {  // d is positive. r is the sph kernel length (i.e. h
                                                          // in the document) d is the distance of 2 particles
    Real invh = paramsD.INVHSML;
    Real q = length(d) * invh;
    if (abs(q) < EPSILON)
        return mR3(0.0);
    bool less1 = (q < 1);
    bool less2 = (q < 2);
    return (less1 * (3 * q - 4.0f) + less2 * (!less1) * (-q + 4.0f - 4.0f / q)) * .75f * INVPI * quintic(invh) * d;
}
//--------------------------------------------------------------------------------------------------------------------------------
// Gradient of Johnson kernel 1996b
__device__ inline Real3 GradWh_High(Real3 d, Real h) {  // d is positive. r is the sph kernel length (i.e. h
                                                        // in the document) d is the distance of 2 particles
    Real invh = paramsD.INVHSML;
    Real q = length(d) * invh;
    if (abs(q) < EPSILON)
        return mR3(0.0);
    bool less2 = (q < 2);
    return (3.0 / 8.0 * q - 3.0 / 4.0) * 5.0 / 4.0 / q * INVPI * (1.0 / quintic(h)) * d * less2;
}
//--------------------------------------------------------------------------------------------------------------------------------
// Gradient of Quintic Spline SPH kernel function
__device__ inline Real3 W3h_Quintic(Real3 d, Real h) {  // d is positive. h is the sph kernel length (i.e. h in
                                                        // the document) d is the distance of 2 particles
    Real invh = paramsD.INVHSML;
    Real q = length(d) * invh;
    if (fabs(q) < 1e-10)
        return mR3(0.0);
    Real coeff = -4.178273e-2;  // -15/359
    if (q < 1) {
        return (coeff * (INVPI * quintic(invh) / q) * d * (quartic(3 - q) - 6 * quartic(2 - q) + 15 * quartic(1 - q)));
    }
    if (q < 2) {
        return (coeff * (INVPI * quintic(invh) / q) * d * (quartic(3 - q) - 6 * quartic(2 - q)));
    }
    if (q < 3) {
        return (coeff * (INVPI * quintic(invh) / q) * d * (quartic(3 - q)));
    }
    return mR3(0);
}

//--------------------------------------------------------------------------------------------------------------------------------
// fluid equation of state
__device__ inline Real Eos(Real rho, Real type) {
    // if (rho < paramsD.rho0) //
    //     rho = paramsD.rho0; //
    // Real gama = 7;
    // Real B = 100 * paramsD.rho0 * paramsD.v_Max * paramsD.v_Max / gama;
    // return B * (pow(rho / paramsD.rho0, gama) - 1) + paramsD.BASEPRES; //
    return paramsD.Cs * paramsD.Cs * (rho - paramsD.rho0);  //
}
//--------------------------------------------------------------------------------------------------------------------------------
// Inverse of equation of state
__device__ inline Real InvEos(Real pw) {
    Real rho = pw / (paramsD.Cs * paramsD.Cs) + paramsD.rho0;  //
    return rho;
}
//--------------------------------------------------------------------------------------------------------------------------------
// ferrariCi
__device__ inline Real FerrariCi(Real rho) {
    int gama = 7;
    Real B = 100 * paramsD.rho0 * paramsD.v_Max * paramsD.v_Max / gama;
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
    Real dd = dist3.x * dist3.x + dist3.y * dist3.y + dist3.z * dist3.z;
    Real MinD = paramsD.epsMinMarkersDis * paramsD.HSML;
    Real sq_MinD = MinD * MinD;
    if (dd < sq_MinD) {
        dist3 = mR3(MinD, 0, 0);
    }
    b = a - dist3;
    return (dist3);
}

__device__ inline Real3 Distance(Real3 a, Real3 b) {
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
__device__ inline int3 calcGridPos(Real3 p) {
    int3 gridPos;
    if (paramsD.cellSize.x * paramsD.cellSize.y * paramsD.cellSize.z == 0)
        printf("calcGridPos=%f,%f,%f\n", paramsD.cellSize.x, paramsD.cellSize.y, paramsD.cellSize.z);

    gridPos.x = (int)floor((p.x - paramsD.worldOrigin.x) / (paramsD.cellSize.x));
    gridPos.y = (int)floor((p.y - paramsD.worldOrigin.y) / (paramsD.cellSize.y));
    gridPos.z = (int)floor((p.z - paramsD.worldOrigin.z) / (paramsD.cellSize.z));
    return gridPos;
}
//--------------------------------------------------------------------------------------------------------------------------------
__device__ inline uint calcGridHash(int3 gridPos) {
    gridPos.x -= ((gridPos.x >= paramsD.gridSize.x) ? paramsD.gridSize.x : 0);
    gridPos.y -= ((gridPos.y >= paramsD.gridSize.y) ? paramsD.gridSize.y : 0);
    gridPos.z -= ((gridPos.z >= paramsD.gridSize.z) ? paramsD.gridSize.z : 0);

    gridPos.x += ((gridPos.x < 0) ? paramsD.gridSize.x : 0);
    gridPos.y += ((gridPos.y < 0) ? paramsD.gridSize.y : 0);
    gridPos.z += ((gridPos.z < 0) ? paramsD.gridSize.z : 0);

    return gridPos.z * paramsD.gridSize.y * paramsD.gridSize.x + gridPos.y * paramsD.gridSize.x + gridPos.x;
}

//--------------------------------------------------------------------------------------------------------------------------------
__device__ inline uint calcCellID(uint3 cellPos) {
    if (cellPos.x < paramsD.gridSize.x && cellPos.y < paramsD.gridSize.y && cellPos.z < paramsD.gridSize.z) {
        return cellPos.z * paramsD.gridSize.x * paramsD.gridSize.y + cellPos.y * paramsD.gridSize.x + cellPos.x;
    } else {
        printf("shouldn't be here\n");
        return paramsD.gridSize.x * paramsD.gridSize.y * paramsD.gridSize.z;
    }
}

//--------------------------------------------------------------------------------------------------------------------------------
__device__ inline uint getCellPos(int trialCellPos, uint ub) {
    if (trialCellPos >= 0 && trialCellPos < ub) {
        return (uint)trialCellPos;
    } else if (trialCellPos < 0) {
        return (uint)(trialCellPos + ub);
    } else {
        return (uint)(trialCellPos - ub);
    }
    return (uint)trialCellPos;
}

//--------------------------------------------------------------------------------------------------------------------------------
inline __device__ uint getCenterCellID(const uint* numPartsInCenterCells, const uint threadID) {
    uint offsets[9] = {0};
    for (int i = 0; i < 8; ++i) {
        offsets[i + 1] = numPartsInCenterCells[i];
    }
    uint left = 0;
    uint right = 8;
    while (left < right) {
        uint mid = (left + right) / 2;
        if (offsets[mid] < threadID) {
            left = mid + 1;
        } else if (offsets[mid] > threadID) {
            right = mid;
        } else {
            return mid;
        }
    }
    return left - 1;
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
    if (paramsD.mu_of_I == FrictionLaw::CONSTANT)
        mu = paramsD.mu_fric_s;
    else if (paramsD.mu_of_I == FrictionLaw::NONLINEAR)
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
                                   Real3& myAcc,         // output: BCE marker acceleration
                                   Real3& V_prescribed,  // output: BCE marker velocity
                                   Real4* sortedPosRad,
                                   int4 updatePortion,
                                   uint* gridMarkerIndexD,
                                   Real4* qD,
                                   Real3* rigid_BCEcoords_D,
                                   Real3* posRigid_fsiBodies_D,
                                   Real4* velMassRigid_fsiBodies_D,
                                   Real3* omegaVelLRF_fsiBodies_D,
                                   Real3* accRigid_fsiBodies_D,
                                   Real3* omegaAccLRF_fsiBodies_D,
                                   uint* rigid_BCEsolids_D,

                                   Real3* flex1D_vel_fsi_fea_D,  // vel of fea 1d element
                                   Real3* flex1D_acc_fsi_fea_D,  // acc of fea 1d element
                                   Real3* flex2D_vel_fsi_fea_D,  // vel of fea 2d element
                                   Real3* flex2D_acc_fsi_fea_D,  // acc of fea 2d element

                                   uint2* flex1D_Nodes_D,      // segment node indices
                                   uint3* flex1D_BCEsolids_D,  // association of flex BCEs with a mesh and segment
                                   Real3* flex1D_BCEcoords_D,  // local coordinates of BCE markers on FEA 1-D segments
                                   uint3* flex2D_Nodes_D,      // triangle node indices
                                   uint3* flex2D_BCEsolids_D,  // association of flex BCEs with a mesh and face
                                   Real3* flex2D_BCEcoords_D   // local coordinates of BCE markers on FEA 2-D faces
) {
    int Original_idx = gridMarkerIndexD[i_idx];

    // See if this belongs to a fixed boundary
    if (Original_idx >= updatePortion.x && Original_idx < updatePortion.y) {
        myAcc = mR3(0.0);
        V_prescribed = mR3(0.0);
        if (paramsD.Apply_BC_U)
            V_prescribed = user_BC_U(mR3(sortedPosRad[i_idx]));
    } else if (Original_idx >= updatePortion.y && Original_idx < updatePortion.z) {
        int rigidIndex = rigid_BCEsolids_D[Original_idx - updatePortion.y];

        Real4 q4 = qD[rigidIndex];
        Real3 a1, a2, a3;
        RotationMatirixFromQuaternion(a1, a2, a3, q4);
        Real3 rigidSPH_MeshPos_LRF__ = rigid_BCEcoords_D[Original_idx - updatePortion.y];

        // Real3 p_com = mR3(posRigid_fsiBodies_D[rigidIndex]);
        Real3 v_com = mR3(velMassRigid_fsiBodies_D[rigidIndex]);
        Real3 a_com = accRigid_fsiBodies_D[rigidIndex];
        Real3 angular_v_com = omegaVelLRF_fsiBodies_D[rigidIndex];
        Real3 angular_a_com = omegaAccLRF_fsiBodies_D[rigidIndex];
        // Real3 p_rel = mR3(sortedPosRad[i_idx]) - p_com;
        Real3 omegaCrossS = cross(angular_v_com, rigidSPH_MeshPos_LRF__);
        V_prescribed = v_com + mR3(dot(a1, omegaCrossS), dot(a2, omegaCrossS), dot(a3, omegaCrossS));
        // V_prescribed = v_com + cross(angular_v_com, rigidSPH_MeshPos_LRF);

        Real3 alphaCrossS = cross(angular_a_com, rigidSPH_MeshPos_LRF__);
        Real3 alphaCrossScrossS = cross(angular_v_com, cross(angular_v_com, rigidSPH_MeshPos_LRF__));
        // myAcc = a_com + cross(angular_a_com, p_rel) + cross(angular_v_com, cross(angular_v_com,
        // rigidSPH_MeshPos_LRF__));

        myAcc = a_com + mR3(dot(a1, alphaCrossS), dot(a2, alphaCrossS), dot(a3, alphaCrossS)) +
                mR3(dot(a1, alphaCrossScrossS), dot(a2, alphaCrossScrossS), dot(a3, alphaCrossScrossS));

        // Or not, Flexible bodies for sure
    } else if (Original_idx >= updatePortion.z && Original_idx < updatePortion.w) {
        int FlexIndex = Original_idx - updatePortion.z;  // offset index for bce markers on flex bodies

        // FlexIndex iterates through both 1D and 2D ones
        if (FlexIndex < numObjectsD.numFlexMarkers1D) {
            // 1D element case
            uint3 flex_solid = flex1D_BCEsolids_D[FlexIndex];  // associated flex mesh and segment
            // Luning TODO: do we need flex_mesh and flex_mesh_seg?
            ////uint flex_mesh = flex_solid.x;                 // index of associated mesh
            ////uint flex_mesh_seg = flex_solid.y;             // index of segment in associated mesh
            uint flex_seg = flex_solid.z;  // index of segment in global list

            uint2 seg_nodes = flex1D_Nodes_D[flex_seg];    // indices of the 2 nodes on associated segment
            Real3 A0 = flex1D_acc_fsi_fea_D[seg_nodes.x];  // (absolute) acceleration of node 0
            Real3 A1 = flex1D_acc_fsi_fea_D[seg_nodes.y];  // (absolute) acceleration of node 1

            Real3 V0 = flex1D_vel_fsi_fea_D[seg_nodes.x];  // (absolute) acceleration of node 0
            Real3 V1 = flex1D_vel_fsi_fea_D[seg_nodes.y];  // (absolute) acceleration of node 1

            Real lambda0 = flex1D_BCEcoords_D[FlexIndex].x;  // segment coordinate
            Real lambda1 = 1 - lambda0;                      // segment coordinate

            V_prescribed = V0 * lambda0 + V1 * lambda1;
            myAcc = A0 * lambda0 + A1 * lambda1;
        }
        if (FlexIndex >= numObjectsD.numFlexMarkers1D) {
            int flex2d_index = FlexIndex - numObjectsD.numFlexMarkers1D;

            uint3 flex_solid = flex2D_BCEsolids_D[flex2d_index];  // associated flex mesh and face
            ////uint flex_mesh = flex_solid.x;                 // index of associated mesh
            ////uint flex_mesh_tri = flex_solid.y;             // index of triangle in associated mesh
            uint flex_tri = flex_solid.z;  // index of triangle in global list

            auto tri_nodes = flex2D_Nodes_D[flex_tri];     // indices of the 3 nodes on associated face
            Real3 A0 = flex2D_acc_fsi_fea_D[tri_nodes.x];  // (absolute) acceleration of node 0
            Real3 A1 = flex2D_acc_fsi_fea_D[tri_nodes.y];  // (absolute) acceleration of node 1
            Real3 A2 = flex2D_acc_fsi_fea_D[tri_nodes.z];  // (absolute) acceleration of node 2

            Real3 V0 = flex2D_vel_fsi_fea_D[tri_nodes.x];  // (absolute) acceleration of node 0
            Real3 V1 = flex2D_vel_fsi_fea_D[tri_nodes.y];  // (absolute) acceleration of node 1
            Real3 V2 = flex2D_vel_fsi_fea_D[tri_nodes.z];  // (absolute) acceleration of node 2

            Real lambda0 = flex2D_BCEcoords_D[flex2d_index].x;  // barycentric coordinate
            Real lambda1 = flex2D_BCEcoords_D[flex2d_index].y;  // barycentric coordinate
            Real lambda2 = 1 - lambda0 - lambda1;               // barycentric coordinate

            V_prescribed = V0 * lambda0 + V1 * lambda1 + V2 * lambda2;
            myAcc = A0 * lambda0 + A1 * lambda1 + A2 * lambda2;
        }

    } else {
        printf("i_idx=%d, Original_idx:%d was not found \n\n", i_idx, Original_idx);
    }
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

__global__ void neighborSearchNum(const Real4* sortedPosRad,
                                  const Real4* sortedRhoPreMu,
                                  const uint* cellStart,
                                  const uint* cellEnd,
                                  const uint* activityIdentifierD,
                                  uint* numNeighborsPerPart,
                                  volatile bool* isErrorD);

__global__ void neighborSearchID(const Real4* sortedPosRad,
                                 const Real4* sortedRhoPreMu,
                                 const uint* cellStart,
                                 const uint* cellEnd,
                                 const uint* activityIdentifierD,
                                 const uint* numNeighborsPerPart,
                                 uint* neighborList,
                                 volatile bool* isErrorD);

}  // namespace fsi
}  // namespace chrono
#endif
