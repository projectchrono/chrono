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
// Author: Milad Rakhsha
// =============================================================================
#include <cstdio>
#include <algorithm>
#include <cstring>
#include <fstream>
#include <sstream>
#include <thrust/execution_policy.h>
#include <thrust/extrema.h>
#include <thrust/sort.h>
#include "cublas_v2.h"
#include "chrono_fsi/physics/ChFsiForceI2SPH.cuh"
#include "chrono_fsi/physics/ChSphGeneral.cuh"

//==========================================================================================================================================
namespace chrono {
namespace fsi {
struct my_Functor {
    Real ave;
    my_Functor(Real s) { ave = s; }
    __host__ __device__ void operator()(Real& i) { i -= ave; }
};
struct my_Functor_real4y {
    Real ave;
    my_Functor_real4y(Real s) { ave = s; }
    __host__ __device__ void operator()(Real4& i) { i.y -= ave; }
};

__global__ void Viscosity_correction(Real4* sortedPosRad,  // input: sorted positions
                                     Real3* sortedVelMas,
                                     Real4* sortedRhoPreMu,
                                     Real4* sortedRhoPreMu_old,
                                     Real3* sortedTauXxYyZz,
                                     Real3* sortedTauXyXzYz,
                                     Real4* sr_tau_I_mu_i,

                                     const Real* A_L,
                                     const Real3* A_G,
                                     const Real* A_f,
                                     const Real* sumWij_inv,
                                     const uint* csrColInd,
                                     const uint* numContacts,
                                     int4 updatePortion,
                                     uint* gridMarkerIndexD,

                                     size_t numAllMarkers,
                                     Real delta_t,
                                     Real gamma_y,

                                     volatile bool* isErrorD) {
    uint i_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (i_idx >= numAllMarkers) {
        return;
    }
    uint csrStartIdx = numContacts[i_idx];
    uint csrEndIdx = numContacts[i_idx + 1];

    //    bool Fluid_Marker = sortedRhoPreMu_old[i_idx].w == -1.0;
    //    bool Boundary_Marker = sortedRhoPreMu_old[i_idx].w > -1.0;
    Real mu_ave = 0.0;
    Real p_ave = 0.0;
    Real3 posRadA = mR3(sortedPosRad[i_idx]);
    Real h_i = sortedPosRad[i_idx].w;
    Real3 grad_ux = mR3(0.0), grad_uy = mR3(0.0), grad_uz = mR3(0.0);
    for (int count = csrStartIdx; count < csrEndIdx; count++) {
        int j = csrColInd[count];
        Real3 posRadB = mR3(sortedPosRad[j]);
        Real3 rij = Distance(posRadA, posRadB);
        Real d = length(rij);
        Real3 eij = rij / d;
        Real h_j = sortedPosRad[j].w;
        Real m_j = cube(h_j * paramsD.MULT_INITSPACE) * paramsD.rho0;
        Real W3 = 0.5 * (W3h(d, h_i) + W3h(d, h_j));
        Real3 grad_i_wij = 0.5 * (GradWh(rij, h_i) + GradWh(rij, h_j));

        if (sortedRhoPreMu_old[i_idx].w != -1)
            continue;
        bool fluid = sortedRhoPreMu_old[j].w == -1;
        Real3 coeff = -m_j / sortedRhoPreMu_old[j].x * grad_i_wij;
        grad_ux += coeff * (sortedVelMas[i_idx].x - sortedVelMas[j].x * fluid);
        grad_uy += coeff * (sortedVelMas[i_idx].y - sortedVelMas[j].y * fluid);
        grad_uz += coeff * (sortedVelMas[i_idx].z - sortedVelMas[j].z * fluid);

        //        grad_ux += A_G[count] * sortedVelMas[j].x;
        //        grad_uy += A_G[count] * sortedVelMas[j].y;
        //        grad_uz += A_G[count] * sortedVelMas[j].z;
        mu_ave += A_f[count] * sortedRhoPreMu_old[j].z;
        p_ave += A_f[count] * sortedRhoPreMu_old[j].y;
    }

    Real sr = Strain_Rate(grad_ux, grad_uy, grad_uz);

    Real mu_0 = Herschel_Bulkley_mu_eff(paramsD.HB_sr0, paramsD.HB_k, paramsD.HB_n, paramsD.HB_tau0);
    mu_0 = paramsD.mu_max;
    Real tau_yeild = paramsD.HB_tau0;

    if (paramsD.non_newtonian) {
        if (sr < tau_yeild / paramsD.mu_max)
            sortedRhoPreMu[i_idx].z = paramsD.mu_max;
        //        if (sr < rmaxr(gamma_y, paramsD.HB_sr0))
        //    if (sr < paramsD.HB_sr0)
        //        sortedRhoPreMu[i_idx].z = paramsD.HB_tau0 * pow(sr / paramsD.HB_sr0, 1000.0) / paramsD.HB_sr0;
        //    else if (sortedRhoPreMu_old[i_idx].x < paramsD.rho0)
        //        sortedRhoPreMu[i_idx].z = mu_ave;
        else
            sortedRhoPreMu[i_idx].z = Herschel_Bulkley_mu_eff(sr, paramsD.HB_k, paramsD.HB_n, tau_yeild);
    }
    sr_tau_I_mu_i[i_idx].x = sr;
    sr_tau_I_mu_i[i_idx].y = Sym_Tensor_Norm(sortedTauXxYyZz[i_idx], sortedTauXyXzYz[i_idx]);
}
//--------------------------------------------------------------------------------------------------------------------------------

__global__ void V_star_Predictor(Real4* sortedPosRad,  // input: sorted positions
                                 Real3* sortedVelMas,
                                 Real4* sortedRhoPreMu,
                                 Real3* sortedTauXxYyZz,
                                 Real3* sortedTauXyXzYz,
                                 Real* A_Matrix,
                                 Real3* Bi,
                                 Real3* v_old,

                                 const Real* A_L,
                                 const Real3* A_G,
                                 const Real* A_f,

                                 const Real* sumWij_inv,
                                 Real3* Normals,

                                 const uint* csrColInd,
                                 const uint* numContacts,

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
                                 size_t numFlex1D,
                                 uint2* CableElementsNodes,
                                 uint4* ShellelementsNodes,

                                 int4 updatePortion,
                                 uint* gridMarkerIndexD,

                                 size_t numAllMarkers,
                                 Real delta_t,

                                 volatile bool* isErrorD) {
    uint i_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (i_idx >= numAllMarkers) {
        return;
    }

    // Crank-Nicolson
    Real CN = 0.5;
    Real CN2 = 0.5;

    uint csrStartIdx = numContacts[i_idx];
    uint csrEndIdx = numContacts[i_idx + 1];  //- uint(paramsD.Pressure_Constraint);

    //    if (paramsD.Pressure_Constraint) {
    //        A_Matrix[csrEndIdx + 1] = 0;
    //        A_Matrix[numContacts[numAllMarkers] + i_idx] = 0;
    //    }

    bool Fluid_Marker = sortedRhoPreMu[i_idx].w == -1.0;
    bool Boundary_Marker = sortedRhoPreMu[i_idx].w > -1.0;

    if (sortedRhoPreMu[i_idx].w <= -2) {
        A_Matrix[csrStartIdx] = 1;
        Bi[i_idx] = mR3(0.0);
        return;
    }

    //    Real rho0 = paramsD.rho0;
    Real rhoi = sortedRhoPreMu[i_idx].x;
    Real mu_i = sortedRhoPreMu[i_idx].z;
    Real3 grad_rho_i = mR3(0.0), grad_mu_i = mR3(0.0);
    Real3 grad_ux = mR3(0.0), grad_uy = mR3(0.0), grad_uz = mR3(0.0);
    Real3 gradP = mR3(0.0), Laplacian_u = mR3(0.0);
    Real3 posRadA = mR3(sortedPosRad[i_idx]);
    Real h_i = sortedPosRad[i_idx].w;

    //    bool full_support = (csrEndIdx - csrStartIdx) > 0.9 * paramsD.num_neighbors;
    //    bool full_support = (rhoi >= paramsD.rho0);
    //    printf("full support nn=%d, rho_i=%f\n", csrEndIdx - csrStartIdx, rhoi);
    //    bool ON_FREE_SURFACE = (rhoi < paramsD.rho0);
    //    bool ON_FREE_SURFACE = (rhoi < 0.90 * paramsD.rho0 || csrEndIdx - csrStartIdx < 20);
    //    bool ON_FREE_SURFACE = false;

    //    bool ON_FREE_SURFACE = (csrEndIdx - csrStartIdx < 20);
    int num_fluid = 0;
    Real3 granular_source = mR3(0.0);
    for (int count = csrStartIdx; count < csrEndIdx; count++) {
        int j = csrColInd[count];
        grad_rho_i += A_G[count] * sortedRhoPreMu[j].x;
        grad_mu_i += A_G[count] * sortedRhoPreMu[j].z;
        Real3 posRadB = mR3(sortedPosRad[j]);
        Real3 rij = Distance(posRadA, posRadB);
        Real d = length(rij);
        Real3 eij = rij / d;
        Real h_j = sortedPosRad[j].w;
        Real m_j = cube(h_j * paramsD.MULT_INITSPACE) * paramsD.rho0;
        Real W3 = 0.5 * (W3h(d, h_i) + W3h(d, h_j));
        Real3 grad_i_wij = 0.5 * (GradWh(rij, h_i) + GradWh(rij, h_j));

        Real3 coeff = -m_j / sortedRhoPreMu[j].x * grad_i_wij;
        grad_ux += coeff * (sortedVelMas[i_idx].x - sortedVelMas[j].x);
        grad_uy += coeff * (sortedVelMas[i_idx].y - sortedVelMas[j].y);
        grad_uz += coeff * (sortedVelMas[i_idx].z - sortedVelMas[j].z);
        //        grad_ux += A_G[count] * sortedVelMas[j].x;
        //        grad_uy += A_G[count] * sortedVelMas[j].y;
        //        grad_uz += A_G[count] * sortedVelMas[j].z;
        Laplacian_u += A_L[count] * sortedVelMas[j];
        gradP += A_G[count] * sortedRhoPreMu[j].y;
        if (sortedRhoPreMu[j].w == -1)
            num_fluid++;
    }
    bool full_support = true;  //(rhoi >= 0.2 * paramsD.rho0) && num_fluid > uint(0.1 * paramsD.num_neighbors);

    Real sr = Strain_Rate(grad_ux, grad_uy, grad_uz);
    Real I = Inertia_num(sr, sortedRhoPreMu[i_idx].x, sortedRhoPreMu[i_idx].y, paramsD.ave_diam);
    Real mu_i_p = mu_I(sr, I) * rmaxr(sortedRhoPreMu[i_idx].y, 0);
    Real3 sgn_grad_ux = mu_i_p * sgn(grad_ux), sgn_grad_uy = mu_i_p * sgn(grad_uy), sgn_grad_uz = mu_i_p * sgn(grad_uz);
    Real3 graduxT = mR3(grad_ux.x, grad_uy.x, grad_uz.x);
    Real3 graduyT = mR3(grad_ux.y, grad_uy.y, grad_uz.y);
    Real3 graduzT = mR3(grad_ux.z, grad_uy.z, grad_uz.z);

    Real3 grad_mu_dot_gradu_u = mR3(dot(grad_mu_i, grad_ux), dot(grad_mu_i, grad_uy), dot(grad_mu_i, grad_uz));
    Real3 grad_mu_dot_gradu_uT = mR3(dot(grad_mu_i, graduxT), dot(grad_mu_i, graduyT), dot(grad_mu_i, graduzT));

    //======================== Interior ===========================
    if (Fluid_Marker) {
        // Navier-Stokes
        if (full_support) {
            for (int count = csrStartIdx; count < csrEndIdx; count++) {
                //                int j = csrColInd[count];
                A_Matrix[count] = -CN * mu_i * A_L[count] * full_support +  //
                                  -CN2 * paramsD.non_newtonian * dot(grad_mu_i, A_G[count]) * full_support;
            }
            A_Matrix[csrStartIdx] += rhoi / delta_t;
            Bi[i_idx] += rhoi * sortedVelMas[i_idx] / delta_t +                     // forward euler term from lhs
                         -gradP * !paramsD.USE_NonIncrementalProjection             // Pressure Gradient
                         + (1 - CN) * mu_i * Laplacian_u                            // viscous term;
                         + (1 - CN2) * paramsD.non_newtonian * grad_mu_dot_gradu_u  // Non-Newtonian term
                         + paramsD.non_newtonian * grad_mu_dot_gradu_uT             // Non - Newtonian term
                         + rhoi * (paramsD.gravity + paramsD.bodyForce3);           // body force

        } else {
            A_Matrix[csrStartIdx] = 1.0;
            Bi[i_idx] = sortedVelMas[i_idx] + (paramsD.gravity + paramsD.bodyForce3) * delta_t;
        }
    }
    //======================== Boundary ===========================
    else if (Boundary_Marker) {
        Real h_i = sortedPosRad[i_idx].w;
        Real3 posRadA = mR3(sortedPosRad[i_idx]);
        Real den = 0.0;

        for (uint count = csrStartIdx + 1; count < csrEndIdx; count++) {
            uint j = csrColInd[count];
            if (sortedRhoPreMu[j].w != -1)
                continue;
            Real3 posRadB = mR3(sortedPosRad[j]);
            Real3 rij = Distance(posRadA, posRadB);
            Real h_j = sortedPosRad[j].w;
            Real h_ij = 0.5 * (h_j + h_i);
            Real W3 = W3h(length(rij), h_ij);
            A_Matrix[count] = W3;
            // A_Matrix[count] = A_f[count];
            den += W3;
        }

        Real3 myAcc = mR3(0);
        Real3 V_prescribed = mR3(0);

        BCE_Vel_Acc(i_idx, myAcc, V_prescribed, sortedPosRad, updatePortion, gridMarkerIndexD, qD,
                    rigidSPH_MeshPos_LRF_D, posRigid_fsiBodies_D, velMassRigid_fsiBodies_D, omegaVelLRF_fsiBodies_D,
                    accRigid_fsiBodies_D, omegaAccLRF_fsiBodies_D, rigidIdentifierD, pos_fsi_fea_D, vel_fsi_fea_D,
                    acc_fsi_fea_D, FlexIdentifierD, numFlex1D, CableElementsNodes, ShellelementsNodes);

        if (den < EPSILON) {
            A_Matrix[csrStartIdx] = 1.0;
            Bi[i_idx] = V_prescribed;
        } else {
            A_Matrix[csrStartIdx] = den;
            Bi[i_idx] = 2 * V_prescribed * den;

            //                       + gradP / sortedRhoPreMu[i_idx].x * delta_t * paramsD.USE_NonIncrementalProjection
            //                       * den;
        }
    }

    //    v_old[i_idx] = sortedVelMas[i_idx];

    if (abs(A_Matrix[csrStartIdx]) < EPSILON)
        printf("V_star_Predictor %d A_Matrix[csrStartIdx]= %f, type=%f \n", i_idx, A_Matrix[csrStartIdx],
               sortedRhoPreMu[i_idx].w);
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Pressure_Equation(Real4* sortedPosRad,  // input: sorted positions
                                  Real3* sortedVelMas,
                                  Real4* sortedRhoPreMu,
                                  Real* A_Matrix,
                                  Real* Bi,
                                  Real3* Vstar,
                                  Real* q_new,

                                  const Real* A_f,
                                  const Real* A_L,
                                  const Real3* A_G,
                                  const Real* sumWij_inv,
                                  Real3* Normals,
                                  uint* csrColInd,
                                  const uint* numContacts,

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
                                  size_t numFlex1D,
                                  uint2* CableElementsNodes,
                                  uint4* ShellelementsNodes,

                                  int4 updatePortion,
                                  uint* gridMarkerIndexD,
                                  size_t numAllMarkers,
                                  size_t numFluidMarkers,
                                  Real delta_t,
                                  volatile bool* isErrorD) {
    uint i_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (i_idx >= numAllMarkers) {
        return;
    }

    uint csrStartIdx = numContacts[i_idx];
    uint csrEndIdx = numContacts[i_idx + 1];  //- uint(paramsD.Pressure_Constraint);

    bool Fluid_Marker = sortedRhoPreMu[i_idx].w == -1.0;
    bool Boundary_Marker = sortedRhoPreMu[i_idx].w > -1.0;

    if (sortedRhoPreMu[i_idx].w <= -2) {
        A_Matrix[csrStartIdx] = 1.0;
        Bi[i_idx] = 0.0;
        return;
    }

    Real rhoi = sortedRhoPreMu[i_idx].x;
    Real TIME_SCALE = paramsD.DensityBaseProjetion ? (delta_t * delta_t) : delta_t;
    //    Real TIME_SCALE = 1.0;

    //    bool ON_FREE_SURFACE = (rhoi < paramsD.rho0 || csrEndIdx - csrStartIdx < paramsD.num_neighbors * 0.5);
    bool full_support = (rhoi >= 0.8 * paramsD.rho0) && (csrEndIdx - csrStartIdx) > uint(0.2 * paramsD.num_neighbors);

    //    bool ON_FREE_SURFACE = (rhoi < paramsD.rho0);
    //    Real rho0 = paramsD.rho0;
    Real3 body_force = paramsD.gravity + paramsD.bodyForce3;
    Real3 grad_rho_i = mR3(0.0);
    Real div_vi_star = 0;
    Real div_vi = 0;

    // Calculating the div.v* and grad(rho)
    for (int count = csrStartIdx; count < csrEndIdx; count++) {
        int j = csrColInd[count];
        div_vi_star += dot(A_G[count], Vstar[j]);
        div_vi += dot(A_G[count], sortedVelMas[j]);
        grad_rho_i += A_G[count] * sortedRhoPreMu[j].x;
    }

    Real rhoi_star = rhoi - paramsD.rho0 * div_vi_star * delta_t;
    //    Real rhoi_star = rhoi + dot(grad_rho_i, Vstar[i_idx] * delta_t);

    //======================== Interior ===========================
    if (Fluid_Marker) {
        if (full_support || !paramsD.Conservative_Form) {
            for (int count = csrStartIdx; count < csrEndIdx; count++) {
                // Note that including the second term creates problems with density based projection
                A_Matrix[count] = 1 / rhoi * A_L[count] - 1.0 / (rhoi * rhoi) * dot(grad_rho_i, A_G[count]);
            }

            Real alpha = paramsD.Alpha;  // square(rhoi / paramsD.rho0);
            //            alpha = (alpha > 1) ? 1.0 : alpha;
            if (paramsD.DensityBaseProjetion)
                Bi[i_idx] = alpha * (paramsD.rho0 - rhoi_star) / paramsD.rho0 * (TIME_SCALE / (delta_t * delta_t)) +
                            +0 * (1 - alpha) * div_vi_star * (TIME_SCALE / delta_t);
            else
                Bi[i_idx] = div_vi_star * (TIME_SCALE / delta_t);
        } else {
            //            clearRow(i_idx, csrStartIdx, csrEndIdx, A_Matrix, Bi);
            //            for (int count = csrStartIdx; count < csrEndIdx; count++) {
            //                A_Matrix[count] = A_f[count];
            //            }
            Bi[i_idx] = paramsD.BASEPRES;
            A_Matrix[csrStartIdx] = 1.0;
        }

        //======================= Boundary ===========================
    } else if (Boundary_Marker && paramsD.bceType != BceVersion::ADAMI) {
        Real3 my_normal = Normals[i_idx];
        for (int count = csrStartIdx; count < csrEndIdx; count++) {
            uint j = csrColInd[count];

            if (sortedRhoPreMu[j].w == -1.0 || dot(my_normal, mR3(sortedPosRad[i_idx] - sortedPosRad[j])) > 0) {
                A_Matrix[count] = -dot(A_G[count], my_normal);
                A_Matrix[csrStartIdx] += +dot(A_G[count], my_normal);
            }
        }

        Bi[i_idx] = 0;
        if (abs(A_Matrix[csrStartIdx]) < EPSILON) {
            clearRow(i_idx, csrStartIdx, csrEndIdx, A_Matrix, Bi);
            for (int count = csrStartIdx; count < csrEndIdx; count++)
                A_Matrix[count] = A_f[count];
            A_Matrix[csrStartIdx] -= 1.0;
            Bi[i_idx] = 0.0;
        }

        //        Real Scale = A_Matrix[csrStartIdx];
        //        if (abs(Scale) > EPSILON)
        //            for (int count = csrStartIdx; count < csrEndIdx; count++)
        //                A_Matrix[count] = A_Matrix[count] / Scale;

        //======================= Boundary Adami===========================
    } else if (Boundary_Marker && paramsD.bceType == BceVersion::ADAMI && paramsD.USE_NonIncrementalProjection) {
        Real h_i = sortedPosRad[i_idx].w;
        //        Real Vi = sumWij_inv[i_idx];
        Real3 posRadA = mR3(sortedPosRad[i_idx]);
        Real3 myAcc = mR3(0);
        Real3 V_prescribed = mR3(0);
        BCE_Vel_Acc(i_idx, myAcc, V_prescribed, sortedPosRad, updatePortion, gridMarkerIndexD, qD,
                    rigidSPH_MeshPos_LRF_D, posRigid_fsiBodies_D, velMassRigid_fsiBodies_D, omegaVelLRF_fsiBodies_D,
                    accRigid_fsiBodies_D, omegaAccLRF_fsiBodies_D, rigidIdentifierD, pos_fsi_fea_D, vel_fsi_fea_D,
                    acc_fsi_fea_D, FlexIdentifierD, numFlex1D, CableElementsNodes, ShellelementsNodes);
        Real pRHS = 0.0;
        Real den = 0.0;

        for (int count = csrStartIdx; count < csrEndIdx; count++) {
            uint j = csrColInd[count];
            if (sortedRhoPreMu[j].w != -1.0)
                continue;
            Real3 posRadB = mR3(sortedPosRad[j]);
            Real3 rij = Distance(posRadA, posRadB);
            Real h_j = sortedPosRad[j].w;
            Real h_ij = 0.5 * (h_j + h_i);
            Real W3 = W3h(length(rij), h_ij);
            // fluid pressures are actually p*TIME_SCALE, so divide by TIME_SCALE to get the actual formula
            A_Matrix[count] = -W3;
            // pressure of the boundary marker should be calculated as p*TIME_SCALE
            // so divide its multiplier by TIME_SCALE
            Real rho_bar = 0.5 * (sortedRhoPreMu[i_idx].x + sortedRhoPreMu[j].x);

            den += W3;
            pRHS += dot(body_force - myAcc, rij) * rho_bar * W3;
        }

        if (abs(den) > EPSILON) {
            A_Matrix[csrStartIdx] = den;
            Bi[i_idx] = pRHS;
            // Scale to make the diagonal element 1

        } else {
            A_Matrix[csrStartIdx] = 1.0;
            Bi[i_idx] = paramsD.BASEPRES;
        }

        q_new[i_idx] = sortedRhoPreMu[i_idx].y * TIME_SCALE;
        Bi[i_idx] *= TIME_SCALE;
    }

    //    if (paramsD.Pressure_Constraint) {
    //        A_Matrix[csrEndIdx] = (double)Fluid_Marker / (double)numFluidMarkers;
    //        csrColInd[csrEndIdx] = numAllMarkers;
    //        uint last_row_start = numContacts[numAllMarkers];
    //        A_Matrix[last_row_start + i_idx] = (double)Fluid_Marker / (double)numFluidMarkers;
    //        csrColInd[last_row_start + i_idx] = i_idx;
    //    }

    //    // Diagonal preconditioner
    //    if (abs(A_Matrix[csrStartIdx]) > EPSILON) {
    //        for (int count = csrStartIdx + 1; count < csrEndIdx; count++)
    //            A_Matrix[count] /= A_Matrix[csrStartIdx];
    //        Bi[i_idx] /= A_Matrix[csrStartIdx];
    //        A_Matrix[csrStartIdx] = 1.0;
    //    }

    //    if (sortedRhoPreMu[i_idx].w > -1)
    //    A_Matrix[csrStartIdx] = 1.0 + paramsD.epsMinMarkersDis;

    //    if (abs(A_Matrix[csrStartIdx]) < EPSILON)
    //        printf("Pressure_Equation %d A_Matrix[csrStartIdx]= %f, type=%f \n", i_idx, A_Matrix[csrStartIdx],
    //               sortedRhoPreMu[i_idx].w);
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Velocity_Correction_and_update(Real4* sortedPosRad,
                                               Real4* sortedPosRad_old,

                                               Real4* sortedRhoPreMu,
                                               Real4* sortedRhoPreMu_old,

                                               Real3* sortedVelMas,
                                               Real3* sortedVelMas_old,

                                               Real3* sortedTauXxYyZz,
                                               Real3* sortedTauXyXzYz,

                                               Real4* sr_tau_I_mu_i,

                                               Real3* sortedVisVel,
                                               Real4* derivVelRho,

                                               Real3* Vstar,
                                               Real* q_i,  // q=p^(n+1)-p^n

                                               const Real* A_f,
                                               const Real3* A_G,
                                               const Real* A_L,
                                               const uint* csrColInd,
                                               const uint* numContacts,
                                               size_t numAllMarkers,
                                               const Real MaxVel,
                                               Real delta_t,
                                               volatile bool* isErrorD) {
    uint i_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (i_idx >= numAllMarkers) {
        return;
    }
    // Note that every variable that is used inside the for loops should not be overwritten later otherwise there would
    // be a race condition. For such variables one must use the old values.
    uint csrStartIdx = numContacts[i_idx];
    uint csrEndIdx = numContacts[i_idx + 1];  //- uint(paramsD.Pressure_Constraint);
    //    Real m_i = cube(sortedPosRad_old[i_idx].w * paramsD.MULT_INITSPACE) * paramsD.rho0;
    Real m_i = paramsD.markerMass;
    Real TIME_SCALE = paramsD.DensityBaseProjetion ? (delta_t * delta_t) : delta_t;
    //    Real TIME_SCALE = 1.0;

    Real3 grad_p_nPlus1 = mR3(0.0);

    Real divV_star = 0;
    Real rho_i = sortedRhoPreMu_old[i_idx].x;
    Real h_i = sortedPosRad_old[i_idx].w;
    Real3 posA = mR3(sortedPosRad_old[i_idx]);
    Real3 grad_q_i_conservative = mR3(0.0), grad_q_i_consistent = mR3(0.0), laplacian_V = mR3(0.0);
    Real3 grad_ux = mR3(0.0), grad_uy = mR3(0.0), grad_uz = mR3(0.0);

    for (int count = csrStartIdx; count < csrEndIdx; count++) {
        uint j = csrColInd[count];
        //        Real m_j = cube(sortedPosRad_old[j].w * paramsD.MULT_INITSPACE) * paramsD.rho0;
        Real m_j = paramsD.markerMass;
        Real rho_j = sortedRhoPreMu_old[j].x;
        Real3 rij = Distance(posA, mR3(sortedPosRad_old[j]));
        Real h_j = sortedPosRad_old[j].w;
        Real h_ij = 0.5 * (h_j + h_i);
        Real3 gradW = GradWh(rij, h_ij);
        bool fluid_j = sortedRhoPreMu_old[j].w == -1;
        bool fluid_i = sortedRhoPreMu_old[i_idx].w == -1;

        if (fluid_i || fluid_j) {
            Real3 coeff = -m_j / sortedRhoPreMu_old[j].x * gradW;
            grad_ux += coeff * (Vstar[i_idx].x * fluid_i - Vstar[j].x * fluid_j);
            grad_uy += coeff * (Vstar[i_idx].y * fluid_i - Vstar[j].y * fluid_j);
            grad_uz += coeff * (Vstar[i_idx].z * fluid_i - Vstar[j].z * fluid_j);
            //            grad_ux += A_G[count] * Vstar[j].x;
            //            grad_uy += A_G[count] * Vstar[j].y;
            //            grad_uz += A_G[count] * Vstar[j].z;
        }

        // No forces for BCE to BCE markers
        if (sortedRhoPreMu_old[j].w > -1 && sortedRhoPreMu_old[i_idx].w > -1)
            continue;

        grad_q_i_conservative += m_j * rho_i * (q_i[i_idx] / (rho_i * rho_i) + q_i[j] / (rho_j * rho_j)) * gradW;
        grad_q_i_consistent += A_G[count] * q_i[j];
        divV_star += dot(A_G[count], Vstar[j]);
        grad_p_nPlus1 += A_G[count] * (sortedRhoPreMu_old[j].y + q_i[j] / TIME_SCALE);
        laplacian_V += A_L[count] * sortedVelMas_old[j];
    }

    Real3 grad_q_i = (paramsD.Conservative_Form ? grad_q_i_conservative : grad_q_i_consistent) / TIME_SCALE;
    Real3 Pressure_correction_term = -grad_q_i * (delta_t) / paramsD.rho0;

    //    if (rho_i < paramsD.rho0)
    //        Pressure_correction_term = mR3(0);

    //    if (!(isfinite(q_i[i_idx]))) {
    //        printf("Error! particle %d q_i is NAN: thrown from ChFsiForceI2SPH.cu  %f\n", i_idx, q_i[i_idx]);
    //    }
    Real3 V_new = Vstar[i_idx] + Pressure_correction_term;
    //    if (sortedRhoPreMu[i_idx].w == -1)
    //        printf("Vstar=%f, Pressure_correction_term=%f\n", Vstar[i_idx], Pressure_correction_term);

    //    Real4 x_new = sortedPosRad[i_idx] + mR4(delta_t / 2 * (V_new + sortedVelMas_old[i_idx]), 0.0);
    Real4 x_new = sortedPosRad_old[i_idx] + mR4(delta_t * (V_new), 0.0);

    sortedVelMas[i_idx] = V_new;
    //    sortedRhoPreMu[i_idx].x = sortedRhoPreMu_old[i_idx].x - delta_t * sortedRhoPreMu_old[i_idx].x * divV_star;
    Real mu_i = sortedRhoPreMu_old[i_idx].z;

    Real3 FS_force = (-grad_q_i_conservative / TIME_SCALE + laplacian_V * mu_i + paramsD.bodyForce3 + paramsD.gravity) *
                     m_i / sortedRhoPreMu_old[i_idx].x;
    derivVelRho[i_idx] = mR4(FS_force, 0.0);

    //    if (sortedRhoPreMu[i_idx].w > 0 && length(derivVelRho[i_idx]) > 0)
    //        printf("%f\t", length(derivVelRho[i_idx]));

    //    Real3 m_dv_dt = m_i * (V_new - sortedVelMas_old[i_idx]) / delta_t;
    //    derivVelRho[i_idx] = mR4(-m_dv_dt, 0);

    if (paramsD.USE_NonIncrementalProjection)
        sortedRhoPreMu[i_idx].y = (q_i[i_idx]) / TIME_SCALE;
    else
        sortedRhoPreMu[i_idx].y += q_i[i_idx] + dot(grad_p_nPlus1, mR3(x_new - sortedPosRad_old[i_idx]));

    Real3 updatedTauXxYyZz = sortedTauXxYyZz[i_idx];
    Real3 updatedTauXyXzYz = sortedTauXyXzYz[i_idx];

    Real tau_norm = Sym_Tensor_Norm(updatedTauXxYyZz, updatedTauXyXzYz);
    ////Real yeild_tau = sr_tau_I_mu_i[i_idx].w * rmaxr(sortedRhoPreMu_old[i_idx].y, 0);
    sr_tau_I_mu_i[i_idx].y = Sym_Tensor_Norm(sortedTauXxYyZz[i_idx], sortedTauXyXzYz[i_idx]);

    if (sortedRhoPreMu_old[i_idx].w == -1.0) {
        sortedPosRad[i_idx] = x_new;
    }

    //    if (!(isfinite(sortedPosRad[i_idx].x) && isfinite(sortedPosRad[i_idx].y) && isfinite(sortedPosRad[i_idx].z)))
    //    {
    //        printf("Error! particle %d position is NAN: thrown from ChFsiForceI2SPH.cu  %f,%f,%f,%f\n", i_idx,
    //               sortedPosRad[i_idx].x, sortedPosRad[i_idx].y, sortedPosRad[i_idx].z, sortedPosRad[i_idx].w);
    //    }
    //    if (!(isfinite(sortedRhoPreMu[i_idx].x) && isfinite(sortedRhoPreMu[i_idx].y) &&
    //          isfinite(sortedRhoPreMu[i_idx].z))) {
    //        printf("Error! particle %d rhoPreMu is NAN: thrown from ChFsiForceI2SPH.cu %f,%f,%f,%f\n", i_idx,
    //               sortedRhoPreMu[i_idx].x, sortedRhoPreMu[i_idx].y, sortedRhoPreMu[i_idx].z,
    //               sortedRhoPreMu[i_idx].w);
    //    }
    //
    //    if (!(isfinite(sortedVelMas[i_idx].x) && isfinite(sortedVelMas[i_idx].y) && isfinite(sortedVelMas[i_idx].z)))
    //    {
    //        printf("Error! particle %d velocity is NAN: thrown from ChFsiForceI2SPH.cu %f,%f,%f\n", i_idx,
    //               sortedVelMas[i_idx].x, sortedVelMas[i_idx].y, sortedVelMas[i_idx].z);
    //    }
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Shifting(Real4* sortedPosRad,
                         Real4* sortedPosRad_old,
                         Real4* sortedRhoPreMu,
                         Real4* sortedRhoPreMu_old,
                         Real3* sortedVelMas,
                         Real3* sortedVelMas_old,
                         Real3* sortedVisVel,
                         const Real* A_f,
                         const Real3* A_G,
                         const uint* csrColInd,
                         const uint* numContacts,
                         size_t numAllMarkers,
                         const Real MaxVel,
                         Real delta_t,
                         volatile bool* isErrorD) {
    uint i_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (i_idx >= numAllMarkers) {
        return;
    }

    uint csrStartIdx = numContacts[i_idx];
    uint csrEndIdx = numContacts[i_idx + 1];  //- uint(paramsD.Pressure_Constraint);
    Real3 inner_sum = mR3(0.0), shift_r = mR3(0.0);
    Real mi_bar = 0.0, r0 = 0.0;  // v_bar = 0.0;
    Real3 xSPH_Sum = mR3(0.0);
    if (!(isfinite(sortedPosRad_old[i_idx].x) && isfinite(sortedPosRad_old[i_idx].y) &&
          isfinite(sortedPosRad_old[i_idx].z))) {
        printf("Error! particle %d position is NAN: thrown from 1 Shifting ChFsiForceI2SPH.cu  %f,%f,%f,%f\n", i_idx,
               sortedPosRad_old[i_idx].x, sortedPosRad_old[i_idx].y, sortedPosRad_old[i_idx].z,
               sortedPosRad_old[i_idx].w);
    }
    if (!(isfinite(sortedRhoPreMu_old[i_idx].x) && isfinite(sortedRhoPreMu[i_idx].y) &&
          isfinite(sortedRhoPreMu_old[i_idx].z))) {
        printf("Error! particle %d rhoPreMu is NAN: thrown from 1 Shifting ChFsiForceI2SPH.cu %f,%f,%f,%f\n", i_idx,
               sortedRhoPreMu_old[i_idx].x, sortedRhoPreMu_old[i_idx].y, sortedRhoPreMu_old[i_idx].z,
               sortedRhoPreMu_old[i_idx].w);
    }

    if (!(isfinite(sortedVelMas_old[i_idx].x) && isfinite(sortedVelMas_old[i_idx].y) &&
          isfinite(sortedVelMas_old[i_idx].z))) {
        printf("Error! particle %d velocity is NAN: thrown from 1 Shifting ChFsiForceI2SPH.cu %f,%f,%f\n", i_idx,
               sortedVelMas_old[i_idx].x, sortedVelMas_old[i_idx].y, sortedVelMas_old[i_idx].z);
    }
    for (int count = csrStartIdx; count < csrEndIdx; count++) {
        uint j = csrColInd[count];
        Real3 rij = Distance(mR3(sortedPosRad_old[i_idx]), mR3(sortedPosRad_old[j]));
        Real d = length(rij);
        Real m_j = paramsD.markerMass;

        if (sortedRhoPreMu_old[j].w == -1.0) {
            Real h_ij = 0.5 * (sortedPosRad_old[j].w + sortedPosRad_old[i_idx].w);
            Real Wd = W3h(d, h_ij);
            Real rho_bar = 0.5 * (sortedRhoPreMu_old[i_idx].x + sortedRhoPreMu_old[j].x);
            xSPH_Sum += (sortedVelMas_old[j] - sortedVelMas_old[i_idx]) * Wd * m_j / rho_bar;
        }
        //        v_bar += length(A_f[count] * (sortedVelMas_old[j]));
        //        Real m_j = cube(sortedPosRad_old[j].w * paramsD.MULT_INITSPACE) * paramsD.rho0;

        mi_bar += m_j;
        r0 += d;
        if (d > 0)
            //        inner_sum += rij / (d * d * d);
            inner_sum += m_j * rij / (d * d * d);
    }

    if (sortedRhoPreMu_old[i_idx].w == -1.0) {
        r0 /= (csrEndIdx - csrStartIdx + 1);
        mi_bar /= (csrEndIdx - csrStartIdx + 1);
    }

    if (abs(mi_bar) > EPSILON)
        shift_r = paramsD.beta_shifting * r0 * r0 * length(MaxVel) * delta_t * inner_sum / mi_bar;
    //    shift_r = paramsD.beta_shifting * r0 * r0 * length(MaxVel) * delta_t * inner_sum;

    Real3 grad_p = mR3(0.0);
    Real3 grad_rho = mR3(0.0);
    Real3 grad_ux = mR3(0.0);
    Real3 grad_uy = mR3(0.0);
    Real3 grad_uz = mR3(0.0);
    Real p_smooth = 0, rho_smooth = 0;

    for (int count = csrStartIdx; count < csrEndIdx; count++) {
        uint j = csrColInd[count];
        grad_p += A_G[count] * sortedRhoPreMu_old[j].y;
        grad_rho += A_G[count] * sortedRhoPreMu_old[j].x;
        grad_ux += A_G[count] * sortedVelMas_old[j].x;
        grad_uy += A_G[count] * sortedVelMas_old[j].y;
        grad_uz += A_G[count] * sortedVelMas_old[j].z;
        p_smooth += A_f[count] * (sortedRhoPreMu_old[j].y);
        rho_smooth += A_f[count] * (sortedRhoPreMu_old[j].x);
    }

    if (sortedRhoPreMu_old[i_idx].w == -1.0) {
        sortedPosRad[i_idx] += mR4(shift_r, 0.0);
        sortedRhoPreMu[i_idx].y += dot(shift_r, grad_p);
        sortedRhoPreMu[i_idx].x += dot(shift_r, grad_rho);
        sortedVelMas[i_idx].x += dot(shift_r, grad_ux);
        sortedVelMas[i_idx].y += dot(shift_r, grad_uy);
        sortedVelMas[i_idx].z += dot(shift_r, grad_uz);
        sortedVelMas[i_idx] += paramsD.EPS_XSPH * xSPH_Sum;
        //        sortedPosRad[i_idx] += mR4(paramsD.EPS_XSPH * xSPH_Sum * delta_t, 0.0);
    }

    Real3 vis_vel = mR3(0.0);
    for (int count = csrStartIdx; count < csrEndIdx; count++) {
        uint j = csrColInd[count];
        if (sortedRhoPreMu_old[j].w == -1.0)
            vis_vel += A_f[count] * (sortedVelMas_old[j]);
    }

    sortedVisVel[i_idx] = vis_vel;

    if (!(isfinite(sortedPosRad[i_idx].x) && isfinite(sortedPosRad[i_idx].y) && isfinite(sortedPosRad[i_idx].z))) {
        printf("Error! particle %d position is NAN: thrown from Shifting ChFsiForceI2SPH.cu  %f,%f,%f,%f\n", i_idx,
               sortedPosRad[i_idx].x, sortedPosRad[i_idx].y, sortedPosRad[i_idx].z, sortedPosRad[i_idx].w);
    }
    if (!(isfinite(sortedRhoPreMu[i_idx].x) && isfinite(sortedRhoPreMu[i_idx].y) &&
          isfinite(sortedRhoPreMu[i_idx].z))) {
        printf("Error! particle %d rhoPreMu is NAN: thrown from Shifting ChFsiForceI2SPH.cu %f,%f,%f,%f\n", i_idx,
               sortedRhoPreMu[i_idx].x, sortedRhoPreMu[i_idx].y, sortedRhoPreMu[i_idx].z, sortedRhoPreMu[i_idx].w);
    }

    if (!(isfinite(sortedVelMas[i_idx].x) && isfinite(sortedVelMas[i_idx].y) && isfinite(sortedVelMas[i_idx].z))) {
        printf("Error! particle %d velocity is NAN: thrown from Shifting ChFsiForceI2SPH.cu %f,%f,%f\n", i_idx,
               sortedVelMas[i_idx].x, sortedVelMas[i_idx].y, sortedVelMas[i_idx].z);
    }
}
//==========================================================================================================================================
//==========================================================================================================================================
//==========================================================================================================================================
ChFsiForceI2SPH::ChFsiForceI2SPH(std::shared_ptr<ChBce> otherBceWorker,
                                 std::shared_ptr<SphMarkerDataD> otherSortedSphMarkersD,
                                 std::shared_ptr<ProximityDataD> otherMarkersProximityD,
                                 std::shared_ptr<FsiGeneralData> otherFsiGeneralData,
                                 std::shared_ptr<SimParams> otherParamsH,
                                 std::shared_ptr<ChCounters> otherNumObjects,
                                 bool verb)
    : ChFsiForce(otherBceWorker,
                 otherSortedSphMarkersD,
                 otherMarkersProximityD,
                 otherFsiGeneralData,
                 otherParamsH,
                 otherNumObjects,
                 verb) {
    CopyParams_NumberOfObjects(paramsH, numObjectsH);
}

ChFsiForceI2SPH::~ChFsiForceI2SPH() {}

void ChFsiForceI2SPH::Initialize() {
    ChFsiForce::Initialize();
    cudaMemcpyToSymbolAsync(paramsD, paramsH.get(), sizeof(SimParams));
    cudaMemcpyToSymbolAsync(numObjectsD, numObjectsH.get(), sizeof(ChCounters));
    cudaMemcpyFromSymbol(paramsH.get(), paramsD, sizeof(SimParams));
    cudaDeviceSynchronize();
    CopyParams_NumberOfObjects(paramsH, numObjectsH);
    int numAllMarkers = (int)numObjectsH->numAllMarkers;
    _sumWij_inv.resize(numAllMarkers);
    Normals.resize(numAllMarkers);
    G_i.resize(numAllMarkers * 9);
    A_i.resize(numAllMarkers * 27);
    L_i.resize(numAllMarkers * 6);
    Contact_i.resize(numAllMarkers);
    V_star_new.resize(numAllMarkers);
    V_star_old.resize(numAllMarkers);
    q_new.resize(numAllMarkers);
    q_old.resize(numAllMarkers);
    b1Vector.resize(numAllMarkers);
    b3Vector.resize(numAllMarkers);
    Residuals.resize(numAllMarkers);
    isErrorH = (bool*)malloc(sizeof(bool));
    cudaMalloc((void**)&isErrorD, sizeof(bool));
    cudaMalloc((void**)&isErrorD2, sizeof(bool));
    *isErrorH = false;
    cudaMemcpy(isErrorD, isErrorH, sizeof(bool), cudaMemcpyHostToDevice);
    cudaMemcpy(isErrorD2, isErrorH, sizeof(bool), cudaMemcpyHostToDevice);
}

void ChFsiForceI2SPH::PreProcessor(std::shared_ptr<SphMarkerDataD> otherSphMarkersD,
                                   bool calcLaplacianOperator) {
    numAllMarkers = numObjectsH->numAllMarkers;
    Contact_i.resize(numAllMarkers);
    uint numThreads, numBlocks;
    computeGridSize((int)numAllMarkers, 128, numBlocks, numThreads);
    thrust::fill(Contact_i.begin(), Contact_i.end(), 0);
    thrust::fill(_sumWij_inv.begin(), _sumWij_inv.end(), 1e-3);
    thrust::fill(A_i.begin(), A_i.end(), 0);
    thrust::fill(L_i.begin(), L_i.end(), 0);
    thrust::fill(G_i.begin(), G_i.end(), 0);

    //============================================================================================================
    *isErrorH = false;
    cudaMemcpy(isErrorD, isErrorH, sizeof(bool), cudaMemcpyHostToDevice);
    calcRho_kernel<<<numBlocks, numThreads>>>(
        mR4CAST(sortedSphMarkersD->posRadD), mR4CAST(sortedSphMarkersD->rhoPresMuD), R1CAST(_sumWij_inv),
        U1CAST(markersProximityD->cellStartD), U1CAST(markersProximityD->cellEndD), U1CAST(Contact_i), numAllMarkers,
        isErrorD);
    ChUtilsDevice::Sync_CheckError(isErrorH, isErrorD, "calcRho_kernel");
    uint LastVal = Contact_i[numAllMarkers - 1];
    thrust::exclusive_scan(Contact_i.begin(), Contact_i.end(), Contact_i.begin());
    Contact_i.push_back(LastVal + Contact_i[numAllMarkers - 1]);
    //    int first = Contact_i[0];
    //    int second = Contact_i[1];
    //
    //    printf("Contact_i[0]=%d,Contact_i[1]=%d\n", first, second);
    NNZ = Contact_i[numAllMarkers];
    csrValGradient.resize(NNZ);
    csrValLaplacian.resize(NNZ);
    csrValFunciton.resize(NNZ);
    AMatrix.resize(NNZ);
    csrColInd.resize(NNZ);
    thrust::fill(csrValGradient.begin(), csrValGradient.end(), mR3(0.0));
    thrust::fill(csrValLaplacian.begin(), csrValLaplacian.end(), 0.0);
    thrust::fill(csrValFunciton.begin(), csrValFunciton.end(), 0.0);
    thrust::fill(csrColInd.begin(), csrColInd.end(), 0.0);

    calcNormalizedRho_Gi_fillInMatrixIndices<<<numBlocks, numThreads>>>(
        mR4CAST(sortedSphMarkersD->posRadD), mR3CAST(sortedSphMarkersD->velMasD),
        mR4CAST(sortedSphMarkersD->rhoPresMuD), R1CAST(_sumWij_inv), R1CAST(G_i), mR3CAST(Normals), U1CAST(csrColInd),
        U1CAST(Contact_i), U1CAST(markersProximityD->cellStartD), U1CAST(markersProximityD->cellEndD), numAllMarkers,
        isErrorD);
    ChUtilsDevice::Sync_CheckError(isErrorH, isErrorD, "calcNormalizedRho_Gi_fillInMatrixIndices");

    //============================================================================================================
    double A_L_Tensor_GradLaplacian = clock();

    if (calcLaplacianOperator && !paramsH->Conservative_Form) {
        printf("| calc_A_tensor+");
        calc_A_tensor<<<numBlocks, numThreads>>>(R1CAST(A_i), R1CAST(G_i), mR4CAST(sortedSphMarkersD->posRadD),
                                                 mR4CAST(sortedSphMarkersD->rhoPresMuD), R1CAST(_sumWij_inv),
                                                 U1CAST(csrColInd), U1CAST(Contact_i), numAllMarkers, isErrorD);
        ChUtilsDevice::Sync_CheckError(isErrorH, isErrorD, "calc_A_tensor");
        calc_L_tensor<<<numBlocks, numThreads>>>(R1CAST(A_i), R1CAST(L_i), R1CAST(G_i),
                                                 mR4CAST(sortedSphMarkersD->posRadD),
                                                 mR4CAST(sortedSphMarkersD->rhoPresMuD), R1CAST(_sumWij_inv),
                                                 U1CAST(csrColInd), U1CAST(Contact_i), numAllMarkers, isErrorD);
        ChUtilsDevice::Sync_CheckError(isErrorH, isErrorD, "calc_L_tensor");
    }

    Function_Gradient_Laplacian_Operator<<<numBlocks, numThreads>>>(
        mR4CAST(sortedSphMarkersD->posRadD), mR3CAST(sortedSphMarkersD->velMasD),
        mR4CAST(sortedSphMarkersD->rhoPresMuD), R1CAST(_sumWij_inv), R1CAST(G_i), R1CAST(L_i), R1CAST(csrValLaplacian),
        mR3CAST(csrValGradient), R1CAST(csrValFunciton), U1CAST(csrColInd), U1CAST(Contact_i), numAllMarkers, isErrorD);
    ChUtilsDevice::Sync_CheckError(isErrorH, isErrorD, "Gradient_Laplacian_Operator");
    double Gradient_Laplacian_Operator = (clock() - A_L_Tensor_GradLaplacian) / (double)CLOCKS_PER_SEC;
}

//==========================================================================================================================================
//==========================================================================================================================================
//==========================================================================================================================================
void ChFsiForceI2SPH::ForceSPH(std::shared_ptr<SphMarkerDataD> otherSphMarkersD,
                               std::shared_ptr<FsiBodiesDataD> otherFsiBodiesD,
                               std::shared_ptr<FsiMeshDataD> otherFsiMeshD) {
    if (paramsH->bceType == BceVersion::ADAMI && !paramsH->USE_NonIncrementalProjection) {
        throw std::runtime_error(
            "\nADAMI boundary condition is only applicable to non-incremental Projection method. Please "
            "revise the BC scheme or set USE_NonIncrementalProjection to true.!\n");
    }

    CopyParams_NumberOfObjects(paramsH, numObjectsH);

    sphMarkersD = otherSphMarkersD;
    fsiCollisionSystem->ArrangeData(sphMarkersD);

    thrust::device_vector<Real3>::iterator iter =
        thrust::max_element(sortedSphMarkersD->velMasD.begin(), sortedSphMarkersD->velMasD.end(), compare_Real3_mag());
    Real MaxVel = length(*iter);

    thrust::device_vector<Real4>::iterator iter_mu = thrust::max_element(
        sortedSphMarkersD->rhoPresMuD.begin(), sortedSphMarkersD->rhoPresMuD.end(), compare_Real4_z());
    Real Maxmu = length(*iter_mu);

    Real dt_CFL = paramsH->Co_number * paramsH->HSML / 2.0 / MaxVel;
    Real dt_nu = 0.2 * paramsH->HSML * paramsH->HSML / (paramsH->mu0 / paramsH->rho0);
    Real dt_body = 0.1 * sqrt(paramsH->HSML / length(paramsH->bodyForce3 + paramsH->gravity));
    Real dt = std::min(dt_body, std::min(dt_CFL, dt_nu));

    if (!paramsH->Adaptive_time_stepping) {
        if (verbose)
            printf("| time step=%.3e, dt_Max=%.3e, dt_CFL=%.3e (CFL=%.2g), dt_nu=%.3e, dt_body=%.3e\n", paramsH->dT,
                   paramsH->dT_Max, dt_CFL, paramsH->Co_number, dt_nu, dt_body);
    } else {
        if (dt / paramsH->dT_Max > 0.51 && dt < paramsH->dT_Max)
            paramsH->dT = paramsH->dT_Max * 0.5;
        else
            paramsH->dT = std::min(dt, paramsH->dT_Max);

        if (verbose)
            printf("| time step=%.3e, dt_Max=%.3e, dt_CFL=%.3e (CFL=%.2g), dt_nu=%.3e, dt_body=%.3e\n", paramsH->dT,
                   paramsH->dT_Max, dt_CFL, paramsH->Co_number, dt_nu, dt_body);

        CopyParams_NumberOfObjects(paramsH, numObjectsH);
    }

    size_t end_fluid = numObjectsH->numGhostMarkers + numObjectsH->numHelperMarkers + numObjectsH->numFluidMarkers;
    size_t end_bndry = end_fluid + numObjectsH->numBoundaryMarkers;
    size_t end_rigid = end_bndry + numObjectsH->numRigidMarkers;
    size_t end_flex = end_rigid + numObjectsH->numFlexMarkers;
    int4 updatePortion = mI4((int)end_fluid, (int)end_bndry, (int)end_rigid, (int)end_flex);

    if (verbose)
        std::cout << "update portion: " << updatePortion.x << " " << updatePortion.y << " " << updatePortion.z << " "
                  << updatePortion.w << std::endl;

    //=====calcRho_kernel=== calc_A_tensor==calc_L_tensor==Function_Gradient_Laplacian_Operator=================
    ChFsiForceI2SPH::PreProcessor(sortedSphMarkersD);

    //==========================================================================================================
    uint numThreads, numBlocks;
    computeGridSize((int)numAllMarkers + 1, 256, numBlocks, numThreads);
    thrust::device_vector<Real4> rhoPresMuD_old = sortedSphMarkersD->rhoPresMuD;
    thrust::device_vector<Real4> posRadD_old = sortedSphMarkersD->posRadD;
    thrust::device_vector<Real3> velMasD_old = sortedSphMarkersD->velMasD;
    thrust::device_vector<Real4> sr_tau_I_mu_i(numAllMarkers, mR4(0.0));

    thrust::fill(V_star_old.begin(), V_star_old.end(), mR3(0.0));
    thrust::fill(V_star_new.begin(), V_star_new.end(), mR3(0.0));
    thrust::fill(b3Vector.begin(), b3Vector.end(), mR3(0.0));
    thrust::fill(Residuals.begin(), Residuals.end(), 0.0);
    Real yeild_strain = MaxVel / paramsH->HSML * 0.05;

    if (paramsH->non_newtonian) {
        Viscosity_correction<<<numBlocks, numThreads>>>(
            mR4CAST(sortedSphMarkersD->posRadD), mR3CAST(sortedSphMarkersD->velMasD),
            mR4CAST(sortedSphMarkersD->rhoPresMuD), mR4CAST(rhoPresMuD_old), mR3CAST(sortedSphMarkersD->tauXxYyZzD),
            mR3CAST(sortedSphMarkersD->tauXyXzYzD), mR4CAST(sr_tau_I_mu_i), R1CAST(csrValLaplacian),
            mR3CAST(csrValGradient), R1CAST(csrValFunciton), R1CAST(_sumWij_inv), U1CAST(csrColInd), U1CAST(Contact_i),

            updatePortion, U1CAST(markersProximityD->gridMarkerIndexD), numAllMarkers, paramsH->dT, yeild_strain,
            isErrorD);
        ChUtilsDevice::Sync_CheckError(isErrorH, isErrorD, "Viscosity_correction");
    }

    //============================================V_star_Predictor===============================================
    double LinearSystemClock_V = clock();
    V_star_Predictor<<<numBlocks, numThreads>>>(
        mR4CAST(sortedSphMarkersD->posRadD), mR3CAST(sortedSphMarkersD->velMasD),
        mR4CAST(sortedSphMarkersD->rhoPresMuD), mR3CAST(sortedSphMarkersD->tauXxYyZzD),
        mR3CAST(sortedSphMarkersD->tauXyXzYzD), R1CAST(AMatrix), mR3CAST(b3Vector), mR3CAST(V_star_old),
        R1CAST(csrValLaplacian), mR3CAST(csrValGradient), R1CAST(csrValFunciton), R1CAST(_sumWij_inv), mR3CAST(Normals),
        U1CAST(csrColInd), U1CAST(Contact_i),

        mR4CAST(otherFsiBodiesD->q_fsiBodies_D), mR3CAST(fsiGeneralData->rigidSPH_MeshPos_LRF_D),
        mR3CAST(otherFsiBodiesD->posRigid_fsiBodies_D), mR4CAST(otherFsiBodiesD->velMassRigid_fsiBodies_D),
        mR3CAST(otherFsiBodiesD->omegaVelLRF_fsiBodies_D), mR3CAST(otherFsiBodiesD->accRigid_fsiBodies_D),
        mR3CAST(otherFsiBodiesD->omegaAccLRF_fsiBodies_D), U1CAST(fsiGeneralData->rigidIdentifierD),

        mR3CAST(otherFsiMeshD->pos_fsi_fea_D), mR3CAST(otherFsiMeshD->vel_fsi_fea_D),
        mR3CAST(otherFsiMeshD->acc_fsi_fea_D), U1CAST(fsiGeneralData->FlexIdentifierD),

        numObjectsH->numFlexBodies1D, U2CAST(fsiGeneralData->CableElementsNodes),
        U4CAST(fsiGeneralData->ShellElementsNodes), updatePortion, U1CAST(markersProximityD->gridMarkerIndexD),
        numAllMarkers, paramsH->dT, isErrorD);
    ChUtilsDevice::Sync_CheckError(isErrorH, isErrorD, "V_star_Predictor");

    int Iteration = 0;
    Real MaxRes = 100;
    while ((MaxRes > 1e-10 || Iteration < 3) && Iteration < paramsH->LinearSolver_Max_Iter) {
        Jacobi_SOR_Iter<<<numBlocks, numThreads>>>(mR4CAST(sortedSphMarkersD->rhoPresMuD), R1CAST(AMatrix),
                                                   mR3CAST(V_star_old), mR3CAST(V_star_new), mR3CAST(b3Vector),
                                                   R1CAST(q_old), R1CAST(q_new), R1CAST(b1Vector), U1CAST(csrColInd),
                                                   U1CAST(Contact_i), numAllMarkers, true, isErrorD);
        ChUtilsDevice::Sync_CheckError(isErrorH, isErrorD, "Jacobi_SOR_Iter");
        Update_AND_Calc_Res<<<numBlocks, numThreads>>>(mR4CAST(sortedSphMarkersD->rhoPresMuD), mR3CAST(V_star_old),
                                                       mR3CAST(V_star_new), R1CAST(q_old), R1CAST(q_new),
                                                       R1CAST(Residuals), numAllMarkers, true, isErrorD);
        ChUtilsDevice::Sync_CheckError(isErrorH, isErrorD, "Update_AND_Calc_Res");
        Iteration++;
        thrust::device_vector<Real>::iterator iter = thrust::max_element(Residuals.begin(), Residuals.end());
        auto position = iter - Residuals.begin();
        MaxRes = *iter;
        if (paramsH->Verbose_monitoring)
            printf("Iter= %.4d, Res= %.4e\n", Iteration, MaxRes);
    }
    //
    //    thrust::device_vector<Real3>::iterator iter =
    //        thrust::max_element(V_star_new.begin(), V_star_new.end(), compare_Real3_mag());
    //    unsigned int position = iter - V_star_new.begin();
    //    Real MaxVel = length(*iter);
    //
    //    uint FixedMarker = 0;
    double V_star_Predictor = (clock() - LinearSystemClock_V) / (double)CLOCKS_PER_SEC;
    printf("| V_star_Predictor Equation: %f (sec) - Final Residual=%.3e - #Iter=%d\n", V_star_Predictor, MaxRes,
           Iteration);
    //==================================================Pressure_Equation==============================================
    Iteration = 0;
    MaxRes = 100;
    double LinearSystemClock_p = clock();

    //    if (paramsH->Pressure_Constraint) {
    //        AMatrix.resize(NNZ + numAllMarkers + 1);
    //        csrColInd.resize(NNZ + numAllMarkers + 1);
    //        b1Vector.resize(numAllMarkers + 1);
    //        Contact_i.push_back(Contact_i[numAllMarkers] + numAllMarkers + 1);
    //        q_old.resize(numAllMarkers + 1);
    //        q_new.resize(numAllMarkers + 1);
    //        int Contact_i_last = Contact_i[numAllMarkers + 1];
    //        AMatrix[Contact_i[numAllMarkers + 1] - 1] = 0;
    //        csrColInd[Contact_i[numAllMarkers + 1] - 1] = numAllMarkers;
    //        //        printf(" NNZ=%d, NNZ_new=%d, Contact_i.size()=%d, csrColInd.size()=%d, Contact_i_last=%d\n",
    //        NNZ,
    //        //               AMatrix.size(), Contact_i.size(), csrColInd.size(), Contact_i_last);
    //    }

    Real TIME_SCALE = paramsH->DensityBaseProjetion ? (paramsH->dT * paramsH->dT) : paramsH->dT;

    thrust::fill(AMatrix.begin(), AMatrix.end(), 0.0);
    thrust::fill(b1Vector.begin(), b1Vector.end(), 0.0);
    thrust::fill(q_old.begin(), q_old.end(), paramsH->Pressure_Constraint * paramsH->BASEPRES);
    thrust::fill(q_new.begin(), q_new.end(), paramsH->Pressure_Constraint * paramsH->BASEPRES);

    Pressure_Equation<<<numBlocks, numThreads>>>(
        mR4CAST(sortedSphMarkersD->posRadD), mR3CAST(sortedSphMarkersD->velMasD),
        mR4CAST(sortedSphMarkersD->rhoPresMuD), R1CAST(AMatrix), R1CAST(b1Vector), mR3CAST(V_star_new), R1CAST(q_new),
        R1CAST(csrValFunciton), R1CAST(csrValLaplacian), mR3CAST(csrValGradient), R1CAST(_sumWij_inv), mR3CAST(Normals),
        U1CAST(csrColInd), U1CAST(Contact_i),

        mR4CAST(otherFsiBodiesD->q_fsiBodies_D), mR3CAST(fsiGeneralData->rigidSPH_MeshPos_LRF_D),
        mR3CAST(otherFsiBodiesD->posRigid_fsiBodies_D), mR4CAST(otherFsiBodiesD->velMassRigid_fsiBodies_D),
        mR3CAST(otherFsiBodiesD->omegaVelLRF_fsiBodies_D), mR3CAST(otherFsiBodiesD->accRigid_fsiBodies_D),
        mR3CAST(otherFsiBodiesD->omegaAccLRF_fsiBodies_D), U1CAST(fsiGeneralData->rigidIdentifierD),

        mR3CAST(otherFsiMeshD->pos_fsi_fea_D), mR3CAST(otherFsiMeshD->vel_fsi_fea_D),
        mR3CAST(otherFsiMeshD->acc_fsi_fea_D), U1CAST(fsiGeneralData->FlexIdentifierD),

        numObjectsH->numFlexBodies1D, U2CAST(fsiGeneralData->CableElementsNodes),
        U4CAST(fsiGeneralData->ShellElementsNodes), updatePortion, U1CAST(markersProximityD->gridMarkerIndexD),
        numAllMarkers, numObjectsH->numFluidMarkers, paramsH->dT, isErrorD);
    ChUtilsDevice::Sync_CheckError(isErrorH, isErrorD, "Pressure_Equation");

    Real Ave_RHS = thrust::reduce(b1Vector.begin(), b1Vector.end(), 0.0) / numAllMarkers;

    my_Functor mf(Ave_RHS);
    if (paramsH->Pressure_Constraint) {
        thrust::for_each(b1Vector.begin(), b1Vector.end(), mf);
        Real Ave_after = thrust::reduce(b1Vector.begin(), b1Vector.end(), 0.0) / numAllMarkers;
        printf("Ave RHS =%f, Ave after removing null space=%f\n", Ave_RHS, Ave_after);
    }
    //    if (paramsH->Pressure_Constraint) {
    //        uint csrStartIdx = Contact_i[0];
    //        uint csrEndIdx = Contact_i[1] - 1;
    //        // The average q should be zero for incremental projection method
    //        b1Vector[numAllMarkers] = paramsH->BASEPRES * paramsH->USE_NonIncrementalProjection;
    //        int Start_last = Contact_i[numAllMarkers];
    //        for (int count = csrStartIdx; count < csrEndIdx; count++) {
    //            int j = csrColInd[count];
    //            AMatrix[j + Start_last] += AMatrix[count];
    //        }
    //        b1Vector[numAllMarkers] += b1Vector[0];
    //        AMatrix[Start_last] = 1;
    //    }

    if (paramsH->USE_LinearSolver) {
        if (paramsH->PPE_Solution_type != PPESolutionType::FORM_SPARSE_MATRIX) {
            printf(
                "You should paramsH->PPE_Solution_type == FORM_SPARSE_MATRIX in order to use the "
                "chrono_fsi linear "
                "solvers\n");
            exit(0);
        }
        myLinearSolver->SetVerbose(paramsH->Verbose_monitoring);
        myLinearSolver->SetAbsRes(paramsH->LinearSolver_Abs_Tol);
        myLinearSolver->SetRelRes(paramsH->LinearSolver_Rel_Tol);
        myLinearSolver->SetIterationLimit(paramsH->LinearSolver_Max_Iter);

        if (paramsH->PPE_Solution_type != PPESolutionType::FORM_SPARSE_MATRIX) {
            printf(
                "You should paramsH->PPE_Solution_type == FORM_SPARSE_MATRIX in order to use the "
                "chrono_fsi linear "
                "solvers\n");
            exit(0);
        }
        myLinearSolver->Solve((int)numAllMarkers, NNZ, R1CAST(AMatrix), U1CAST(Contact_i), U1CAST(csrColInd),
                              R1CAST(q_new), R1CAST(b1Vector));

        cudaCheckError();
        MaxRes = myLinearSolver->GetResidual();
        Iteration = myLinearSolver->GetNumIterations();
        if (myLinearSolver->GetSolverStatus()) {
            std::cout << " Linear solver converged to " << myLinearSolver->GetResidual() << " tolerance";
            std::cout << " after " << myLinearSolver->GetNumIterations() << " iterations" << std::endl;
        } else {
            std::cout << "Failed to converge after " << myLinearSolver->GetNumIterations() << " iterations"
                      << std::endl;
        }
    }

    if (!paramsH->USE_LinearSolver || !myLinearSolver->GetSolverStatus()) {
        thrust::fill(Residuals.begin(), Residuals.end(), 0.0);
        while ((MaxRes > paramsH->LinearSolver_Abs_Tol || Iteration < 3) &&
               Iteration < paramsH->LinearSolver_Max_Iter) {
            Jacobi_SOR_Iter<<<numBlocks, numThreads>>>(
                mR4CAST(sortedSphMarkersD->rhoPresMuD), R1CAST(AMatrix), mR3CAST(V_star_old), mR3CAST(V_star_new),
                mR3CAST(b3Vector), R1CAST(q_old), R1CAST(q_new), R1CAST(b1Vector), U1CAST(csrColInd), U1CAST(Contact_i),
                numAllMarkers, false, isErrorD);
            ChUtilsDevice::Sync_CheckError(isErrorH, isErrorD, "Jacobi_SOR_Iter");

            //            if (paramsH->Pressure_Constraint) {
            //                Real sum_last = 0;
            //                cublasHandle_t cublasHandle = 0;
            //                uint Start_last = Contact_i[numAllMarkers];
            //                cublasDdot(cublasHandle, numAllMarkers, R1CAST(b1Vector), 1,
            //                           (double*)thrust::raw_pointer_cast(&AMatrix[Start_last]), 1, &sum_last);
            //                cudaDeviceSynchronize();
            //                b1Vector[numAllMarkers] += b1Vector[0];
            //                q_new[numAllMarkers] = b1Vector[numAllMarkers] - sum_last -
            //                                       q_new[numAllMarkers] * AMatrix[Contact_i[numAllMarkers + 1] - 1];
            //            }mu_s_
            Update_AND_Calc_Res<<<numBlocks, numThreads>>>(
                mR4CAST(sortedSphMarkersD->rhoPresMuD), mR3CAST(V_star_old), mR3CAST(V_star_new), R1CAST(q_old),
                R1CAST(q_new), R1CAST(Residuals), numAllMarkers + 0 * uint(paramsH->Pressure_Constraint), false,
                isErrorD);
            ChUtilsDevice::Sync_CheckError(isErrorH, isErrorD, "Update_AND_Calc_Res");
            Iteration++;
            thrust::device_vector<Real>::iterator iter = thrust::max_element(Residuals.begin(), Residuals.end());
            auto position = iter - Residuals.begin();
            MaxRes = *iter;

            if (paramsH->Verbose_monitoring)
                printf("Iter= %.4d, Res= %.4e\n", Iteration, MaxRes);
        }
    }
    //    Real4_y unary_op_p;
    //    thrust::plus<Real> binary_op;
    //    Real Ave_pressure = thrust::transform_reduce(sortedSphMarkersD->rhoPresMuD.begin(),
    //                                                 sortedSphMarkersD->rhoPresMuD.end(), unary_op_p, 0.0, binary_op)
    //                                                 /
    //                        (numObjectsH->numFluidMarkers);

    Real Ave_pressure = thrust::reduce(q_new.begin(), q_new.end(), 0.0) / (numObjectsH->numAllMarkers) / TIME_SCALE;

    thrust::for_each(b1Vector.begin(), b1Vector.end(), mf);
    Real Ave_after = thrust::reduce(b1Vector.begin(), b1Vector.end(), 0.0) / numAllMarkers;
    printf("Ave RHS =%.3e, Ave after removing null space=%.3e\n", Ave_RHS, Ave_after);

    double Pressure_Computation = (clock() - LinearSystemClock_p) / (double)CLOCKS_PER_SEC;
    printf("| Pressure Poisson Equation: %f (sec) - Final Residual=%.3e - #Iter=%d, Ave_p=%.3e\n", Pressure_Computation,
           MaxRes, Iteration, Ave_pressure);
    //==================================Velocity_Correction_and_update============================================
    double updateClock = clock();
    rhoPresMuD_old = sortedSphMarkersD->rhoPresMuD;
    posRadD_old = sortedSphMarkersD->posRadD;
    velMasD_old = sortedSphMarkersD->velMasD;

    thrust::fill(vel_vis_Sorted_D.begin(), vel_vis_Sorted_D.end(), mR3(0.0));

    // should not be initialized to zero since moving weighted average is going to be applied
    thrust::fill(derivVelRhoD_Sorted_D.begin(), derivVelRhoD_Sorted_D.end(), mR4(0.0));

    Velocity_Correction_and_update<<<numBlocks, numThreads>>>(
        mR4CAST(sortedSphMarkersD->posRadD), mR4CAST(posRadD_old), mR4CAST(sortedSphMarkersD->rhoPresMuD),
        mR4CAST(rhoPresMuD_old), mR3CAST(sortedSphMarkersD->velMasD), mR3CAST(velMasD_old),
        mR3CAST(sortedSphMarkersD->tauXxYyZzD), mR3CAST(sortedSphMarkersD->tauXyXzYzD), mR4CAST(sr_tau_I_mu_i),
        mR3CAST(vel_vis_Sorted_D), mR4CAST(derivVelRhoD_Sorted_D), mR3CAST(V_star_new), R1CAST(q_new),
        R1CAST(csrValFunciton), mR3CAST(csrValGradient), R1CAST(csrValLaplacian), U1CAST(csrColInd), U1CAST(Contact_i),
        numAllMarkers, MaxVel, paramsH->dT, isErrorD);
    ChUtilsDevice::Sync_CheckError(isErrorH, isErrorD, "Velocity_Correction_and_update");

    CopySortedToOriginal_NonInvasive_R3(fsiGeneralData->vis_vel_SPH_D, vel_vis_Sorted_D,
                                        markersProximityD->gridMarkerIndexD);
    CopySortedToOriginal_NonInvasive_R4(fsiGeneralData->derivVelRhoD_old, derivVelRhoD_Sorted_D,
                                        markersProximityD->gridMarkerIndexD);
    CopySortedToOriginal_NonInvasive_R3(sphMarkersD->velMasD, sortedSphMarkersD->velMasD,
                                        markersProximityD->gridMarkerIndexD);
    CopySortedToOriginal_NonInvasive_R4(sphMarkersD->rhoPresMuD, sortedSphMarkersD->rhoPresMuD,
                                        markersProximityD->gridMarkerIndexD);
    CopySortedToOriginal_NonInvasive_R4(sphMarkersD->posRadD, sortedSphMarkersD->posRadD,
                                        markersProximityD->gridMarkerIndexD);
    CopySortedToOriginal_NonInvasive_R3(sphMarkersD->tauXxYyZzD, sortedSphMarkersD->tauXxYyZzD,
                                        markersProximityD->gridMarkerIndexD);
    CopySortedToOriginal_NonInvasive_R3(sphMarkersD->tauXyXzYzD, sortedSphMarkersD->tauXyXzYzD,
                                        markersProximityD->gridMarkerIndexD);
    fsiCollisionSystem->ArrangeData(sphMarkersD);
    ChFsiForceI2SPH::PreProcessor(sortedSphMarkersD, false);

    rhoPresMuD_old = sortedSphMarkersD->rhoPresMuD;
    posRadD_old = sortedSphMarkersD->posRadD;
    velMasD_old = sortedSphMarkersD->velMasD;
    //
    Shifting<<<numBlocks, numThreads>>>(
        mR4CAST(sortedSphMarkersD->posRadD), mR4CAST(posRadD_old), mR4CAST(sortedSphMarkersD->rhoPresMuD),
        mR4CAST(rhoPresMuD_old), mR3CAST(sortedSphMarkersD->velMasD), mR3CAST(velMasD_old), mR3CAST(vel_vis_Sorted_D),
        R1CAST(csrValFunciton), mR3CAST(csrValGradient), U1CAST(csrColInd), U1CAST(Contact_i), numAllMarkers, MaxVel,
        paramsH->dT, isErrorD);
    ChUtilsDevice::Sync_CheckError(isErrorH, isErrorD, "Shifting");
    Real4_x unary_op(paramsH->rho0);
    thrust::plus<Real> binary_op;
    Real Ave_density_Err = thrust::transform_reduce(sortedSphMarkersD->rhoPresMuD.begin(),
                                                    sortedSphMarkersD->rhoPresMuD.end(), unary_op, 0.0, binary_op) /
                           (numObjectsH->numFluidMarkers * paramsH->rho0);

    double updateComputation = (clock() - updateClock) / (double)CLOCKS_PER_SEC;
    Real Re = paramsH->L_Characteristic * paramsH->rho0 * MaxVel / paramsH->mu0;

    printf("| Velocity_Correction_and_update: %f (sec), Ave_density_Err=%.3e, Re=%.1f\n", updateComputation,
           Ave_density_Err, Re);

    // post-processing for conservative formulation
    if (paramsH->Conservative_Form && paramsH->ClampPressure) {
        Real minP = thrust::transform_reduce(sphMarkersD->rhoPresMuD.begin(), sphMarkersD->rhoPresMuD.end(),
                                             Real4_y_min(), 1e9, thrust::minimum<Real>());
        my_Functor_real4y negate(minP);
        thrust::for_each(sortedSphMarkersD->rhoPresMuD.begin(), sortedSphMarkersD->rhoPresMuD.end(), negate);
        printf("Shifting min pressure of %.3e to 0\n", minP);
    }
    //============================================================================================================
    CopySortedToOriginal_NonInvasive_R4(fsiGeneralData->sr_tau_I_mu_i, sr_tau_I_mu_i,
                                        markersProximityD->gridMarkerIndexD);
    CopySortedToOriginal_NonInvasive_R3(sphMarkersD->velMasD, sortedSphMarkersD->velMasD,
                                        markersProximityD->gridMarkerIndexD);
    CopySortedToOriginal_NonInvasive_R4(sphMarkersD->rhoPresMuD, sortedSphMarkersD->rhoPresMuD,
                                        markersProximityD->gridMarkerIndexD);
    CopySortedToOriginal_NonInvasive_R4(sphMarkersD->posRadD, sortedSphMarkersD->posRadD,
                                        markersProximityD->gridMarkerIndexD);
    //    CopySortedToOriginal_NonInvasive_R3(sphMarkersD->tauXxYyZzD, sortedSphMarkersD->tauXxYyZzD,
    //                                        markersProximityD->gridMarkerIndexD);
    //    CopySortedToOriginal_NonInvasive_R3(sphMarkersD->tauXyXzYzD, sortedSphMarkersD->tauXyXzYzD,
    //                                        markersProximityD->gridMarkerIndexD);

    csrValGradient.clear();
    csrValLaplacian.clear();
    csrValFunciton.clear();
    AMatrix.clear();
    Contact_i.clear();
    csrColInd.clear();
}  // namespace fsi
}  // namespace fsi
}  // namespace chrono
