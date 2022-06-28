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
// Author:Arman Pazouki, Milad Rakhsha
// =============================================================================

// This file contains miscellaneous macros and utilities used in the SPH code.
// ****************************************************************************
#ifndef CH_SPH_GENERAL_CU
#define CH_SPH_GENERAL_CU

// ----------------------------------------------------------------------------
// CUDA headers
// ----------------------------------------------------------------------------
#include "chrono_fsi/physics/ChSphGeneral.cuh"

namespace chrono {
namespace fsi {

void CopyParams_NumberOfObjects(std::shared_ptr<SimParams> paramsH, std::shared_ptr<ChCounters> numObjectsH) {
    cudaMemcpyToSymbolAsync(paramsD, paramsH.get(), sizeof(SimParams));
    cudaMemcpyToSymbolAsync(numObjectsD, numObjectsH.get(), sizeof(ChCounters));
    cudaDeviceSynchronize();
}

//--------------------------------------------------------------------------------------------------------------------------------
__global__ void calc_A_tensor(Real* A_tensor,
                              Real* G_tensor,
                              Real4* sortedPosRad,
                              Real4* sortedRhoPreMu,
                              Real* sumWij_inv,
                              uint* csrColInd,
                              uint* numContacts,
                              const size_t numAllMarkers,
                              volatile bool* isErrorD) {
    uint i_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (i_idx >= numAllMarkers) {
        return;
    }

    // Remember : we want to solve 6x6 system Bi*l=-[1 0 0 1 0 1]'
    // elements of matrix B depends on tensor A

    uint csrStartIdx = numContacts[i_idx];
    uint csrEndIdx = numContacts[i_idx + 1];
    Real3 posRadA = mR3(sortedPosRad[i_idx]);
    Real h_i = sortedPosRad[i_idx].w;
    Real m_i = cube(h_i * paramsD.MULT_INITSPACE) * paramsD.rho0;
    Real A_ijk[27] = {0.0};

    Real Gi[9] = {0.0};
    for (int i = 0; i < 9; i++)
        Gi[i] = G_tensor[i_idx * 9 + i];

    // get address in grid
    int3 gridPos = calcGridPos(posRadA);

    for (int count = csrStartIdx; count < csrEndIdx; count++) {
        int j = csrColInd[count];
        Real3 posRadB = mR3(sortedPosRad[j]);
        Real3 rij = Distance(posRadA, posRadB);
        Real d = length(rij);
        if (d > RESOLUTION_LENGTH_MULT * h_i || sortedRhoPreMu[j].w <= -2)
            continue;
        Real h_j = sortedPosRad[j].w;
        Real m_j = cube(h_j * paramsD.MULT_INITSPACE) * paramsD.rho0;
        Real h_ij = 0.5 * (h_j + h_i);
        Real3 grad_ij = GradWh(rij, h_ij);
        Real V_j = sumWij_inv[j];
        Real com_part = 0;
        com_part = (Gi[0] * grad_ij.x + Gi[1] * grad_ij.y + Gi[2] * grad_ij.z) * V_j;
        A_ijk[0] += rij.x * rij.x * com_part;  // 111
        A_ijk[1] += rij.x * rij.y * com_part;  // 112
        A_ijk[2] += rij.x * rij.z * com_part;  // 113
        A_ijk[3] += rij.y * rij.x * com_part;  // 121
        A_ijk[4] += rij.y * rij.y * com_part;  // 122
        A_ijk[5] += rij.y * rij.z * com_part;  // 123
        A_ijk[6] += rij.z * rij.x * com_part;  // 131
        A_ijk[7] += rij.z * rij.y * com_part;  // 132
        A_ijk[8] += rij.z * rij.z * com_part;  // 133
        com_part = (Gi[3] * grad_ij.x + Gi[4] * grad_ij.y + Gi[5] * grad_ij.z) * V_j;
        A_ijk[9] += rij.x * rij.x * com_part;   // 211
        A_ijk[10] += rij.x * rij.y * com_part;  // 212
        A_ijk[11] += rij.x * rij.z * com_part;  // 213
        A_ijk[12] += rij.y * rij.x * com_part;  // 221
        A_ijk[13] += rij.y * rij.y * com_part;  // 222
        A_ijk[14] += rij.y * rij.z * com_part;  // 223
        A_ijk[15] += rij.z * rij.x * com_part;  // 231
        A_ijk[16] += rij.z * rij.y * com_part;  // 232
        A_ijk[17] += rij.z * rij.z * com_part;  // 233
        com_part = (Gi[6] * grad_ij.x + Gi[7] * grad_ij.y + Gi[8] * grad_ij.z) * V_j;
        A_ijk[18] += rij.x * rij.x * com_part;  // 311
        A_ijk[19] += rij.x * rij.y * com_part;  // 312
        A_ijk[20] += rij.x * rij.z * com_part;  // 313
        A_ijk[21] += rij.y * rij.x * com_part;  // 321
        A_ijk[22] += rij.y * rij.y * com_part;  // 322
        A_ijk[23] += rij.y * rij.z * com_part;  // 323
        A_ijk[24] += rij.z * rij.x * com_part;  // 331
        A_ijk[25] += rij.z * rij.y * com_part;  // 332
        A_ijk[26] += rij.z * rij.z * com_part;  // 333
    }

    for (int i = 0; i < 27; i++)
        A_tensor[i_idx * 9 + i] = A_ijk[i];

}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void calc_L_tensor(Real* A_tensor,
                              Real* L_tensor,
                              Real* G_tensor,
                              Real4* sortedPosRad,
                              Real4* sortedRhoPreMu,
                              Real* sumWij_inv,
                              uint* csrColInd,
                              uint* numContacts,
                              const size_t numAllMarkers,
                              volatile bool* isErrorD) {
    uint i_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (i_idx >= numAllMarkers) {
        return;
    }

    if (sortedRhoPreMu[i_idx].w != -1) {
        return;
    }

    // Remember : we want to solve 6x6 system Bi*l=-[1 0 0 1 0 1]'
    // elements of matrix B depends on tensor A

    uint csrStartIdx = numContacts[i_idx];
    uint csrEndIdx = numContacts[i_idx + 1];  // - paramsD.Pressure_Constraint;
    Real3 posRadA = mR3(sortedPosRad[i_idx]);
    Real h_i = sortedPosRad[i_idx].w;
    Real m_i = cube(h_i * paramsD.MULT_INITSPACE) * paramsD.rho0;
    Real B[36] = {0.0};

    //    Real Gi[9] = {0.0};
    //    for (int i = 0; i < 9; i++)
    //        Gi[i] = G_tensor[i_idx * 9 + i];

    Real A_ijk[27] = {0.0};
    for (int i = 0; i < 27; i++)
        A_ijk[i] = A_tensor[i_idx * 27 + i];

    // get address in grid
    int3 gridPos = calcGridPos(posRadA);

    for (int count = csrStartIdx; count < csrEndIdx; count++) {
        int j = csrColInd[count];
        Real3 posRadB = mR3(sortedPosRad[j]);
        Real3 rij = Distance(posRadA, posRadB);
        Real d = length(rij);
        if (d > RESOLUTION_LENGTH_MULT * h_i || sortedRhoPreMu[j].w <= -2)
            continue;
        Real3 eij = rij / d;

        Real h_j = sortedPosRad[j].w;
        Real m_j = cube(h_j * paramsD.MULT_INITSPACE) * paramsD.rho0;
        Real h_ij = 0.5 * (h_j + h_i);
        Real3 grad_ij = GradWh(rij, h_ij);
        Real V_j = sumWij_inv[j];
        Real com_part = 0;
        // mn=11

        Real XX = (eij.x * grad_ij.x);
        Real XY = (eij.x * grad_ij.y + eij.y * grad_ij.x);
        Real XZ = (eij.x * grad_ij.z + eij.z * grad_ij.x);
        Real YY = (eij.y * grad_ij.y);
        Real YZ = (eij.y * grad_ij.z + eij.z * grad_ij.y);
        Real ZZ = (eij.z * grad_ij.z);

        com_part = (A_ijk[0] * eij.x + A_ijk[9] * eij.y + A_ijk[18] * eij.z + rij.x * eij.x) * V_j;
        B[6 * 0 + 0] += com_part * XX;  // 11
        B[6 * 0 + 1] += com_part * XY;  // 12
        B[6 * 0 + 2] += com_part * XZ;  // 13
        B[6 * 0 + 3] += com_part * YY;  // 14
        B[6 * 0 + 4] += com_part * YZ;  // 15
        B[6 * 0 + 5] += com_part * ZZ;  // 15
        // mn=12
        com_part = (A_ijk[1] * eij.x + A_ijk[10] * eij.y + A_ijk[19] * eij.z + rij.x * eij.y) * V_j;
        B[6 * 1 + 0] += com_part * XX;  // 21
        B[6 * 1 + 1] += com_part * XY;  // 22
        B[6 * 1 + 2] += com_part * XZ;  // 23
        B[6 * 1 + 3] += com_part * YY;  // 24
        B[6 * 1 + 4] += com_part * YZ;  // 25
        B[6 * 1 + 5] += com_part * ZZ;  // 25

        // mn=13
        com_part = (A_ijk[2] * eij.x + A_ijk[11] * eij.y + A_ijk[20] * eij.z + rij.x * eij.z) * V_j;
        B[6 * 2 + 0] += com_part * XX;  // 31
        B[6 * 2 + 1] += com_part * XY;  // 32
        B[6 * 2 + 2] += com_part * XZ;  // 33
        B[6 * 2 + 3] += com_part * YY;  // 34
        B[6 * 2 + 4] += com_part * YZ;  // 35
        B[6 * 2 + 5] += com_part * ZZ;  // 36

        // Note that we skip mn=21 since it is similar to mn=12
        // mn=22
        com_part = (A_ijk[4] * eij.x + A_ijk[13] * eij.y + A_ijk[22] * eij.z + rij.y * eij.y) * V_j;
        B[6 * 3 + 0] += com_part * XX;  // 41
        B[6 * 3 + 1] += com_part * XY;  // 42
        B[6 * 3 + 2] += com_part * XZ;  // 43
        B[6 * 3 + 3] += com_part * YY;  // 44
        B[6 * 3 + 4] += com_part * YZ;  // 45
        B[6 * 3 + 5] += com_part * ZZ;  // 46

        // mn=23
        com_part = (A_ijk[5] * eij.x + A_ijk[14] * eij.y + A_ijk[23] * eij.z + rij.y * eij.z) * V_j;
        B[6 * 4 + 0] += com_part * XX;  // 51
        B[6 * 4 + 1] += com_part * XY;  // 52
        B[6 * 4 + 2] += com_part * XZ;  // 53
        B[6 * 4 + 3] += com_part * YY;  // 54
        B[6 * 4 + 4] += com_part * YZ;  // 55
        B[6 * 4 + 5] += com_part * ZZ;  // 56
        // mn=33
        com_part = (A_ijk[8] * eij.x + A_ijk[17] * eij.y + A_ijk[26] * eij.z + rij.z * eij.z) * V_j;
        B[6 * 5 + 0] += com_part * XX;  // 61
        B[6 * 5 + 1] += com_part * XY;  // 62
        B[6 * 5 + 2] += com_part * XZ;  // 63
        B[6 * 5 + 3] += com_part * YY;  // 64
        B[6 * 5 + 4] += com_part * YZ;  // 65
        B[6 * 5 + 5] += com_part * ZZ;  // 66
    }

    inv6xdelta_mn(B, &L_tensor[6 * i_idx]);
}

//--------------------------------------------------------------------------------------------------------------------------------
__global__ void calcRho_kernel(Real4* sortedPosRad,
                               Real4* sortedRhoPreMu,
                               Real* sumWij_inv,
                               uint* cellStart,
                               uint* cellEnd,
                               uint* mynumContact,
                               const size_t numAllMarkers,
                               volatile bool* isErrorD) {
    uint i_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (i_idx >= numAllMarkers) {
        return;
    }

    if (sortedRhoPreMu[i_idx].w == -2) {
        mynumContact[i_idx] = 1;
        return;
    }

    Real3 posRadA = mR3(sortedPosRad[i_idx]);
    Real h_i = sortedPosRad[i_idx].w;
    Real m_i = cube((h_i * paramsD.MULT_INITSPACE)) * paramsD.rho0;

    Real sum_mW = 0;
    Real sum_W = 0.0;
    uint mcon = 1;
    // get address in grid
    int3 gridPos = calcGridPos(posRadA);
    for (int z = -1; z <= 1; z++)
        for (int y = -1; y <= 1; y++)
            for (int x = -1; x <= 1; x++) {
                int3 neighbourPos = gridPos + mI3(x, y, z);
                uint gridHash = calcGridHash(neighbourPos);
                uint startIndex = cellStart[gridHash];
                if (startIndex != 0xffffffff) {
                    uint endIndex = cellEnd[gridHash];
                    for (uint j = startIndex; j < endIndex; j++) {
                        Real3 posRadB = mR3(sortedPosRad[j]);
                        Real3 dist3 = Distance(posRadA, posRadB);
                        Real d = length(dist3);

                        if (d > RESOLUTION_LENGTH_MULT * h_i || sortedRhoPreMu[j].w <= -2)
                            continue;
                        if (i_idx != j)
                            mcon++;
                        Real h_j = sortedPosRad[j].w;
                        Real m_j = cube(h_j * paramsD.MULT_INITSPACE) * paramsD.rho0;
                        Real W3 = W3h(d, 0.5 * (h_j + h_i));
                        sum_mW += m_j * W3;
                        sum_W += W3;
                    }
                }
            }
    mynumContact[i_idx] = mcon;
    // Adding neighbor contribution is done!
    sumWij_inv[i_idx] = m_i / sum_mW;
    sortedRhoPreMu[i_idx].x = sum_mW;

    if ((sortedRhoPreMu[i_idx].x > 2 * paramsD.rho0 || sortedRhoPreMu[i_idx].x < 0) && sortedRhoPreMu[i_idx].w == -1)
        printf("(calcRho_kernel)too large/small density marker %d, rho=%f, sum_W=%f, m_i=%f\n", i_idx,
               sortedRhoPreMu[i_idx].x, sum_W, m_i);
}

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
                                         volatile bool* isErrorD) {
    uint i_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (i_idx >= numAllMarkers || sortedRhoPreMu[i_idx].w <= -2) {
        return;
    }
    //    Real3 gravity = paramsD.gravity;
    Real RHO_0 = paramsD.rho0;
    //    Real IncompressibilityFactor = paramsD.IncompressibilityFactor;
    //    dxi_over_Vi[i_idx] = 1e10;
    if (sortedRhoPreMu[i_idx].w == -2)
        return;
    Real3 posRadA = mR3(sortedPosRad[i_idx]);
    Real h_i = sortedPosRad[i_idx].w;
    //    Real m_i = cube(h_i * paramsD.MULT_INITSPACE) * paramsD.rho0;
    Real sum_mW = 0;
    Real sum_Wij_inv = 0;
    Real C = 0;
    // get address in grid
    int3 gridPos = calcGridPos(posRadA);

    // This is the elements of inverse of G
    Real mGi[9] = {0.0};
    Real theta_i = sortedRhoPreMu[i_idx].w + 1;
    if (theta_i > 1)
        theta_i = 1;
    Real3 mynormals = mR3(0.0);

    // examine neighbouring cells
    for (int z = -1; z <= 1; z++)
        for (int y = -1; y <= 1; y++)
            for (int x = -1; x <= 1; x++) {
                int3 neighbourPos = gridPos + mI3(x, y, z);
                uint gridHash = calcGridHash(neighbourPos);
                // get start of bucket for this cell50
                uint startIndex = cellStart[gridHash];
                if (startIndex != 0xffffffff) {  // cell is not empty
                                                 // iterate over particles in this cell
                    uint endIndex = cellEnd[gridHash];

                    for (uint j = startIndex; j < endIndex; j++) {
                        Real3 posRadB = mR3(sortedPosRad[j]);
                        Real3 dist3 = Distance(posRadA, posRadB);
                        Real3 dv3 = Distance(sortedVelMas[i_idx], sortedVelMas[j]);
                        Real d = length(dist3);
                        Real h_j = sortedPosRad[j].w;
                        Real m_j = cube(h_j * 1) * paramsD.rho0;
                        C += m_j * Color[i_idx] / sortedRhoPreMu[i_idx].x * W3h(d, 0.5 * (h_j + h_i));

                        if (d > RESOLUTION_LENGTH_MULT * h_i || sortedRhoPreMu[j].w <= -2)
                            continue;
                        Real V_j = sumWij_inv[j];

                        Real h_ij = 0.5 * (h_j + h_i);
                        Real W3 = W3h(d, h_ij);
                        Real3 grad_i_wij = GradWh(dist3, h_ij);
                        Real theta_j = sortedRhoPreMu[j].w + 1;
                        if (theta_j > 1)
                            theta_j = 1;

                        if (sortedRhoPreMu[i_idx].w == -3 && sortedRhoPreMu[j].w == -3)
                            mynormals += grad_i_wij * V_j;
                        if (sortedRhoPreMu[i_idx].w != -3)
                            mynormals += (theta_j - theta_i) * grad_i_wij * V_j;

                        mGi[0] -= dist3.x * grad_i_wij.x * V_j;
                        mGi[1] -= dist3.x * grad_i_wij.y * V_j;
                        mGi[2] -= dist3.x * grad_i_wij.z * V_j;
                        mGi[3] -= dist3.y * grad_i_wij.x * V_j;
                        mGi[4] -= dist3.y * grad_i_wij.y * V_j;
                        mGi[5] -= dist3.y * grad_i_wij.z * V_j;
                        mGi[6] -= dist3.z * grad_i_wij.x * V_j;
                        mGi[7] -= dist3.z * grad_i_wij.y * V_j;
                        mGi[8] -= dist3.z * grad_i_wij.z * V_j;
                        sum_mW += m_j * W3;
                        sum_Wij_inv += sumWij_inv[j] * W3;
                    }
                }
            }

    normals[i_idx] = mynormals;

    if (length(mynormals) > EPSILON)
        normals[i_idx] = mynormals / length(mynormals);

    Real Det = (mGi[0] * mGi[4] * mGi[8] - mGi[0] * mGi[5] * mGi[7] - mGi[1] * mGi[3] * mGi[8] +
                mGi[1] * mGi[5] * mGi[6] + mGi[2] * mGi[3] * mGi[7] - mGi[2] * mGi[4] * mGi[6]);
    G_i[i_idx * 9 + 0] = (mGi[4] * mGi[8] - mGi[5] * mGi[7]) / Det;
    G_i[i_idx * 9 + 1] = -(mGi[1] * mGi[8] - mGi[2] * mGi[7]) / Det;
    G_i[i_idx * 9 + 2] = (mGi[1] * mGi[5] - mGi[2] * mGi[4]) / Det;
    G_i[i_idx * 9 + 3] = -(mGi[3] * mGi[8] - mGi[5] * mGi[6]) / Det;
    G_i[i_idx * 9 + 4] = (mGi[0] * mGi[8] - mGi[2] * mGi[6]) / Det;
    G_i[i_idx * 9 + 5] = -(mGi[0] * mGi[5] - mGi[2] * mGi[3]) / Det;
    G_i[i_idx * 9 + 6] = (mGi[3] * mGi[7] - mGi[4] * mGi[6]) / Det;
    G_i[i_idx * 9 + 7] = -(mGi[0] * mGi[7] - mGi[1] * mGi[6]) / Det;
    G_i[i_idx * 9 + 8] = (mGi[0] * mGi[4] - mGi[1] * mGi[3]) / Det;

    // if (sortedRhoPreMu[i_idx].x > RHO_0)
    //     IncompressibilityFactor = 1;
    // sortedRhoPreMu[i_idx].x = (sum_mW / sum_W_sumWij_inv - RHO_0) * IncompressibilityFactor + RHO_0;
    // sortedRhoPreMu[i_idx].x = (sum_mW - RHO_0) * IncompressibilityFactor + RHO_0;

    sortedRhoPreMu[i_idx].x = sum_mW / sum_Wij_inv;

    if ((sortedRhoPreMu[i_idx].x > 5 * RHO_0 || sortedRhoPreMu[i_idx].x < RHO_0 / 5) && sortedRhoPreMu[i_idx].w == -1)
        printf(
            "calcNormalizedRho_kernel-- sortedRhoPreMu[i_idx].w=%f, h=%f, sum_mW=%f, "
            "sum_W_sumWij_inv=%.4e, sortedRhoPreMu[i_idx].x=%.4e\n",
            sortedRhoPreMu[i_idx].w, sortedPosRad[i_idx].w, sum_mW, sum_Wij_inv, sortedRhoPreMu[i_idx].x);

}

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
                                                         volatile bool* isErrorD) {
    uint i_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (i_idx >= numAllMarkers) {
        return;
    }
    Real RHO_0 = paramsD.rho0;
    uint csrStartIdx = numContacts[i_idx] + 1;  // Reserve the starting index for the A_ii
    Real3 posRadA = mR3(sortedPosRad[i_idx]);
    Real h_i = sortedPosRad[i_idx].w;
    Real sum_mW = 0;
    Real sum_mW_rho = 0;

    Real sum_W_sumWij_inv = 0;
    // get address in grid
    int3 gridPos = calcGridPos(posRadA);
    csrColInd[csrStartIdx - 1] = i_idx;
    uint nextCol = csrStartIdx;

    if (sortedRhoPreMu[i_idx].w == -2)
        return;

    Real theta_i = sortedRhoPreMu[i_idx].w + 1;
    if (theta_i > 1)
        theta_i = 1;

    Real3 mynormals = mR3(0.0);
    // This is the elements of inverse of G
    Real mGi[9] = {0.0};
    // examine neighbouring cells
    for (int z = -1; z <= 1; z++)
        for (int y = -1; y <= 1; y++)
            for (int x = -1; x <= 1; x++) {
                int3 neighbourPos = gridPos + mI3(x, y, z);
                uint gridHash = calcGridHash(neighbourPos);
                // get start of bucket for this cell50
                uint startIndex = cellStart[gridHash];
                if (startIndex != 0xffffffff) {  // cell is not empty
                                                 // iterate over particles in this cell
                    uint endIndex = cellEnd[gridHash];

                    for (uint j = startIndex; j < endIndex; j++) {
                        Real3 posRadB = mR3(sortedPosRad[j]);
                        Real3 rij = Distance(posRadA, posRadB);
                        Real3 dv3 = Distance(sortedVelMas[i_idx], sortedVelMas[j]);
                        Real d = length(rij);
                        Real h_j = sortedPosRad[j].w;
                        Real m_j = cube(h_j * paramsD.MULT_INITSPACE) * paramsD.rho0;
                        Real h_ij = 0.5 * (h_j + h_i);
                        Real W3 = W3h(d, h_ij);
                        Real3 grad_i_wij = GradWh(rij, h_ij);

                        Real V_j = sumWij_inv[j];

                        if (d > RESOLUTION_LENGTH_MULT * h_i || sortedRhoPreMu[j].w <= -2)
                            continue;
                        if (i_idx != j) {
                            csrColInd[nextCol] = j;
                            nextCol++;
                        }

                        Real theta_j = sortedRhoPreMu[j].w + 1;
                        if (theta_j > 1)
                            theta_j = 1;

                        if (sortedRhoPreMu[i_idx].w == -3 && sortedRhoPreMu[j].w == -3)
                            mynormals += grad_i_wij * V_j;
                        if (sortedRhoPreMu[i_idx].w != -3)
                            mynormals += (theta_j - theta_i) * grad_i_wij * V_j;

                        mGi[0] -= rij.x * grad_i_wij.x * V_j;
                        mGi[1] -= rij.x * grad_i_wij.y * V_j;
                        mGi[2] -= rij.x * grad_i_wij.z * V_j;
                        mGi[3] -= rij.y * grad_i_wij.x * V_j;
                        mGi[4] -= rij.y * grad_i_wij.y * V_j;
                        mGi[5] -= rij.y * grad_i_wij.z * V_j;
                        mGi[6] -= rij.z * grad_i_wij.x * V_j;
                        mGi[7] -= rij.z * grad_i_wij.y * V_j;
                        mGi[8] -= rij.z * grad_i_wij.z * V_j;
                        sum_mW += m_j * W3;
                        // sum_mW += sortedRhoPreMu[j].x * W3;
                        // sum_mW += m_j * sumWij_inv[j];
                        // sum_mW += sortedRhoPreMu[j].x * W3 * V_j;

                        sum_mW_rho += W3 * m_j / sortedRhoPreMu[j].x;
                        // sum_W_sumWij_inv += W3 * sumWij_inv[j];
                    }
                }
            }

    normals[i_idx] = mynormals;

    if (length(mynormals) > EPSILON)
        normals[i_idx] = mynormals / length(mynormals);

    if (sortedRhoPreMu[i_idx].w == -3)
        normals[i_idx] *= -1;
    Real Det = (mGi[0] * mGi[4] * mGi[8] - mGi[0] * mGi[5] * mGi[7] - mGi[1] * mGi[3] * mGi[8] +
                mGi[1] * mGi[5] * mGi[6] + mGi[2] * mGi[3] * mGi[7] - mGi[2] * mGi[4] * mGi[6]);
    if (abs(Det) < EPSILON && sortedRhoPreMu[i_idx].w != -3) {
        for (int i = 0; i < 9; i++) {
            G_i[i_idx * 9 + i] = 0.0;
            G_i[i_idx * 9 + 0] = 1;
            G_i[i_idx * 9 + 4] = 1;
            G_i[i_idx * 9 + 8] = 1;
        }
    } else {
        G_i[i_idx * 9 + 0] = (mGi[4] * mGi[8] - mGi[5] * mGi[7]) / Det;
        G_i[i_idx * 9 + 1] = -(mGi[1] * mGi[8] - mGi[2] * mGi[7]) / Det;
        G_i[i_idx * 9 + 2] = (mGi[1] * mGi[5] - mGi[2] * mGi[4]) / Det;
        G_i[i_idx * 9 + 3] = -(mGi[3] * mGi[8] - mGi[5] * mGi[6]) / Det;
        G_i[i_idx * 9 + 4] = (mGi[0] * mGi[8] - mGi[2] * mGi[6]) / Det;
        G_i[i_idx * 9 + 5] = -(mGi[0] * mGi[5] - mGi[2] * mGi[3]) / Det;
        G_i[i_idx * 9 + 6] = (mGi[3] * mGi[7] - mGi[4] * mGi[6]) / Det;
        G_i[i_idx * 9 + 7] = -(mGi[0] * mGi[7] - mGi[1] * mGi[6]) / Det;
        G_i[i_idx * 9 + 8] = (mGi[0] * mGi[4] - mGi[1] * mGi[3]) / Det;
    }
    //    sortedRhoPreMu[i_idx].x = sum_mW / sum_mW_rho;
    //    sortedRhoPreMu[i_idx].x = sum_mW / sum_W;
    //    sortedRhoPreMu[i_idx].x = sum_mW;

    if ((sortedRhoPreMu[i_idx].x > 5 * RHO_0 || sortedRhoPreMu[i_idx].x < RHO_0 / 5) && sortedRhoPreMu[i_idx].w > -2)
        printf(
            "calcNormalizedRho_kernel-- sortedRhoPreMu[i_idx].w=%f, h=%f, sum_mW=%f, "
            "sum_W_sumWij_inv=%.4e, sortedRhoPreMu[i_idx].x=%.4e\n",
            sortedRhoPreMu[i_idx].w, sortedPosRad[i_idx].w, sum_mW, sum_W_sumWij_inv, sortedRhoPreMu[i_idx].x);
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Function_Gradient_Laplacian_Operator(Real4* sortedPosRad,  // input: sorted positions
                                                     Real3* sortedVelMas,
                                                     Real4* sortedRhoPreMu,
                                                     Real* sumWij_inv,
                                                     Real* G_tensor,
                                                     Real* L_tensor,
                                                     Real* A_L,   // Laplacian Operator matrix
                                                     Real3* A_G,  // Gradient Operator matrix
                                                     Real* A_f,   // Function Operator matrix
                                                     // A_L, A_G are in system level;
                                                     // A_G* p gives gradp, A_L*p gives Delta^2p
                                                     uint* csrColInd,
                                                     uint* numContacts,
                                                     const size_t numAllMarkers,
                                                     volatile bool* isErrorD) {
    uint i_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (i_idx >= numAllMarkers)
        return;

    if (sortedRhoPreMu[i_idx].w <= -2)
        return;

    //    Real RHO_0 = paramsD.rho0;
    uint csrStartIdx = numContacts[i_idx];
    uint csrEndIdx = numContacts[i_idx + 1];  //- paramsD.Pressure_Constraint;
    Real3 posRadA = mR3(sortedPosRad[i_idx]);
    Real h_i = sortedPosRad[i_idx].w;
    // get address in grid
    int3 gridPos = calcGridPos(posRadA);
    // This is the elements of inverse of G
    Real mGi[9] = {0.0};
    Real Li[6] = {0.0};
    Real3 LaplacainVi = mR3(0.0);
    Real NormGi = 0;
    Real NormLi = 0;

    for (int i = 0; i < 9; i++) {
        mGi[i] = G_tensor[i_idx * 9 + i];
        NormGi += abs(mGi[i]);
    }
    for (int i = 0; i < 6; i++) {
        Li[i] = L_tensor[i_idx * 6 + i];
        NormLi += abs(Li[i]);
    }

    Real V_i = sumWij_inv[i_idx];
    Real m_i = cube(h_i * paramsD.MULT_INITSPACE) * paramsD.rho0;
    Real rhoi = sortedRhoPreMu[i_idx].x;
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

        Real V_j = sumWij_inv[j];
        A_f[count] = V_j * W3;
        if (paramsD.Conservative_Form) {
            if (paramsD.gradient_type == 0) {
                Real Coeff = V_j;
                A_G[count] = Coeff * grad_i_wij;
                A_G[csrStartIdx] -= Coeff * grad_i_wij;
            } else if (paramsD.gradient_type == 1) {
                Real Coeff = V_j;
                A_G[count] = Coeff * grad_i_wij;
                A_G[csrStartIdx] += Coeff * grad_i_wij;
            } else if (paramsD.gradient_type == 2) {
                Real3 comm = m_j * rhoi * grad_i_wij;
                A_G[count] = 1.0 / (sortedRhoPreMu[j].x * sortedRhoPreMu[j].x) * comm;
                A_G[csrStartIdx] += 1.0 / (rhoi * rhoi) * comm;
            } else {
                Real3 comm = 1.0 / V_i * (V_j * V_j + V_i * V_i) / (rhoi + sortedRhoPreMu[j].x) * grad_i_wij;
                A_G[count] = rhoi * comm;
                A_G[csrStartIdx] += sortedRhoPreMu[j].x * comm;
            }
        } else {
            Real Coeff = V_j;
            A_G[count].x = Coeff * (grad_i_wij.x * mGi[0] + grad_i_wij.y * mGi[1] + grad_i_wij.z * mGi[2]);
            A_G[count].y = Coeff * (grad_i_wij.x * mGi[3] + grad_i_wij.y * mGi[4] + grad_i_wij.z * mGi[5]);
            A_G[count].z = Coeff * (grad_i_wij.x * mGi[6] + grad_i_wij.y * mGi[7] + grad_i_wij.z * mGi[8]);
            A_G[csrStartIdx].x -= Coeff * (grad_i_wij.x * mGi[0] + grad_i_wij.y * mGi[1] + grad_i_wij.z * mGi[2]);
            A_G[csrStartIdx].y -= Coeff * (grad_i_wij.x * mGi[3] + grad_i_wij.y * mGi[4] + grad_i_wij.z * mGi[5]);
            A_G[csrStartIdx].z -= Coeff * (grad_i_wij.x * mGi[6] + grad_i_wij.y * mGi[7] + grad_i_wij.z * mGi[8]);
        }
    }

    for (int count = csrStartIdx; count < csrEndIdx; count++) {
        int j = csrColInd[count];
        Real3 posRadB = mR3(sortedPosRad[j]);
        Real3 rij = Distance(posRadA, posRadB);
        Real d = length(rij);
        Real3 eij = rij / d;
        Real h_j = sortedPosRad[j].w;
        Real m_j = cube(h_j * paramsD.MULT_INITSPACE) * paramsD.rho0;
        Real h_ij = 0.5 * (h_j + h_i);
        Real W3 = W3h(d, h_ij);
        Real3 grad_ij = GradWh(rij, h_ij);
        Real V_j = sumWij_inv[j];
        if (d < EPSILON)
            continue;
        if (paramsD.Conservative_Form) {
            if (paramsD.laplacian_type == 0) {
                Real commonterm = 1.0 / V_j * (V_j * V_j + V_i * V_i) * dot(rij, grad_ij);
                A_L[count] -= commonterm / (d * d + h_ij * h_ij * paramsD.epsMinMarkersDis);        // j
                A_L[csrStartIdx] += commonterm / (d * d + h_ij * h_ij * paramsD.epsMinMarkersDis);  // i
                for (int count_in = csrStartIdx; count_in < csrEndIdx; count_in++) {
                    A_L[count_in] -= commonterm * dot(A_G[count_in], eij);  // k
                }
            } else if (paramsD.laplacian_type == 1) {
                Real comm = 2.0 / rhoi * m_j * dot(rij, grad_ij) / (d * d + h_ij * h_ij * paramsD.epsMinMarkersDis);
                A_L[count] = -comm;        // j
                A_L[csrStartIdx] += comm;  // i
            } else {
                Real comm = 2.0 / V_i * (V_j * V_j + V_i * V_i) * dot(rij, grad_ij) /
                            (d * d + h_ij * h_ij * paramsD.epsMinMarkersDis);
                A_L[count] = -comm;        // j
                A_L[csrStartIdx] += comm;  // i
            }
        } else {
            Real commonterm = 1.0 / V_j * (V_j * V_j + V_i * V_i) *
                              (Li[0] * eij.x * grad_ij.x + Li[1] * eij.x * grad_ij.y + Li[2] * eij.x * grad_ij.z +
                               Li[1] * eij.y * grad_ij.x + Li[3] * eij.y * grad_ij.y + Li[4] * eij.y * grad_ij.z +
                               Li[2] * eij.z * grad_ij.x + Li[4] * eij.z * grad_ij.y + Li[5] * eij.z * grad_ij.z);

            A_L[count] -= commonterm / (d + h_ij * paramsD.epsMinMarkersDis);        // j
            A_L[csrStartIdx] += commonterm / (d + h_ij * paramsD.epsMinMarkersDis);  // i

            for (int count_in = csrStartIdx; count_in < csrEndIdx; count_in++) {
                A_L[count_in] -= commonterm * dot(A_G[count_in], eij);  // k
            }
        }

        if (!(isfinite(A_L[count]))) {
            printf("Error! A_L ChSPHGeneral.cu !\n");
        }
    }
}
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
                                volatile bool* isErrorD) {
    uint i_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (i_idx >= numAllMarkers) {
        return;
    }

    uint startIdx = numContacts[i_idx] + 1;  // Reserve the starting index for the A_ii
    uint endIdx = numContacts[i_idx + 1];    //- uint(_3dvector && paramsD.Pressure_Constraint);

    if (_3dvector) {
        Real3 aij_vj = mR3(0.0);
        for (int myIdx = startIdx; myIdx < endIdx; myIdx++) {
            int j = csrColInd[myIdx];
            aij_vj += A_Matrix[myIdx] * V_old[j];
        }
        V_new[i_idx] = (b3vec[i_idx] - aij_vj) / A_Matrix[startIdx - 1];
    } else {
        Real aij_pj = 0.0;
        for (int myIdx = startIdx; myIdx < endIdx; myIdx++) {
            aij_pj += A_Matrix[myIdx] * q_old[csrColInd[myIdx]];
        }
        q_new[i_idx] = (b1vec[i_idx] - aij_pj) / A_Matrix[startIdx - 1];
    }
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Update_AND_Calc_Res(Real4* sortedRhoPreMu,
                                    Real3* V_old,
                                    Real3* V_new,
                                    Real* q_old,
                                    Real* q_new,
                                    Real* Residuals,
                                    const size_t numAllMarkers,
                                    bool _3dvector,
                                    volatile bool* isErrorD) {
    uint i_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (i_idx >= numAllMarkers) {
        return;
    }
    //    Real omega = _3dvector ? 1.0 : paramsD.PPE_relaxation;
    Real omega = paramsD.PPE_relaxation;

    Real res = 0;
    if (_3dvector) {
        V_new[i_idx] = (1 - omega) * V_old[i_idx] + omega * V_new[i_idx];
        res = length(V_old[i_idx] - V_new[i_idx]);
        V_old[i_idx] = V_new[i_idx];

    } else {
        q_new[i_idx] = (1 - omega) * q_old[i_idx] + omega * q_new[i_idx];
        res = abs(q_old[i_idx] - q_new[i_idx]);
        q_old[i_idx] = q_new[i_idx];
    }
    Residuals[i_idx] = res;
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Initialize_Variables(Real4* sortedRhoPreMu,
                                     Real* p_old,
                                     Real3* sortedVelMas,
                                     Real3* V_new,
                                     const size_t numAllMarkers,
                                     volatile bool* isErrorD) {
    const uint i_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (i_idx >= numAllMarkers) {
        return;
    }
    if (sortedRhoPreMu[i_idx].w <= -2) {
        return;
    }

    p_old[i_idx] = sortedRhoPreMu[i_idx].y;  // This needs consistency p_old is old but v_new is new !!
    if (sortedRhoPreMu[i_idx].w > -1) {
        sortedVelMas[i_idx] = V_new[i_idx];
    }
}

//--------------------------------------------------------------------------------------------------------------------------------
__global__ void UpdateDensity(Real3* vis_vel,
                              Real3* XSPH_Vel,
                              Real3* sortedVelMas,  // Write
                              Real4* sortedPosRad,  // Read
                              Real4* sortedRhoPreMu,
                              Real* sumWij_inv,
                              uint* cellStart,
                              uint* cellEnd,
                              size_t numAllMarkers,
                              volatile bool* isErrorD) {
    uint i_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (i_idx >= numAllMarkers) {
        return;
    }
    if (sortedRhoPreMu[i_idx].w <= -2) {
        sortedRhoPreMu[i_idx].x = 0;
        sortedRhoPreMu[i_idx].y = 0;
        sortedRhoPreMu[i_idx].z = 0;
        return;
    }
    Real dT = paramsD.dT;
    Real rho_plus = 0;
    Real3 Vel_i = sortedVelMas[i_idx];

    Real3 posi = mR3(sortedPosRad[i_idx]);
    if ((sortedRhoPreMu[i_idx].x > 2 * paramsD.rho0 || sortedRhoPreMu[i_idx].x < 0) && sortedRhoPreMu[i_idx].w < 0)
        printf("(UpdateDensity-0)too large/small density marker %d, type=%f\n", i_idx, sortedRhoPreMu[i_idx].w);
    Real h_i = sortedPosRad[i_idx].w;
    int3 gridPos = calcGridPos(posi);

    Real3 normalizedV_n = mR3(0);
    Real normalizedV_d = 0.0;
    Real sumW = 0.0;
    Real3 xSPH_Sum = mR3(0.);
    for (int z = -1; z <= 1; z++) {
        for (int y = -1; y <= 1; y++) {
            for (int x = -1; x <= 1; x++) {
                int3 neighbourPos = gridPos + mI3(x, y, z);
                uint gridHash = calcGridHash(neighbourPos);
                // get start of bucket for this cell
                uint startIndex = cellStart[gridHash];
                uint endIndex = cellEnd[gridHash];
                for (uint j = startIndex; j < endIndex; j++) {
                    Real3 posj = mR3(sortedPosRad[j]);
                    Real3 dist3 = Distance(posi, posj);
                    Real d = length(dist3);
                    if (d > RESOLUTION_LENGTH_MULT * h_i || sortedRhoPreMu[j].w <= -2 ||
                        (sortedRhoPreMu[i_idx].w >= 0 && sortedRhoPreMu[j].w >= 0))
                        continue;
                    Real3 Vel_j = sortedVelMas[j];
                    Real h_j = sortedPosRad[j].w;
                    Real m_j = cube(h_j * paramsD.MULT_INITSPACE) * paramsD.rho0;
                    Real h_ij = 0.5 * (h_j + h_i);
                    Real3 grad_i_wij = GradWh(dist3, h_ij);
                    rho_plus += m_j * dot((Vel_i - Vel_j), grad_i_wij) * sumWij_inv[j];
                    Real Wd = W3h(d, h_ij);
                    sumW += Wd;

                    normalizedV_n += Vel_j * Wd * m_j / sortedRhoPreMu[j].x;
                    normalizedV_d += Wd * m_j / sortedRhoPreMu[j].x;

                    if (sortedRhoPreMu[j].w != -1)
                        continue;
                    Real rho_bar = 0.5 * (sortedRhoPreMu[i_idx].x + sortedRhoPreMu[j].x);
                    xSPH_Sum += (Vel_j - Vel_i) * Wd * m_j / rho_bar;
                }
            }
        }
    }
    if (abs(sumW) > EPSILON) {
        vis_vel[i_idx] = normalizedV_n / normalizedV_d;
    }
    XSPH_Vel[i_idx] = xSPH_Sum;  //

    sortedRhoPreMu[i_idx].x += rho_plus * dT;

    if ((sortedRhoPreMu[i_idx].x > 2 * paramsD.rho0 || sortedRhoPreMu[i_idx].x < 0) && sortedRhoPreMu[i_idx].w < 0)
        printf("(UpdateDensity-1)too large/small density marker %d, type=%f\n", i_idx, sortedRhoPreMu[i_idx].w);
}

}  // namespace fsi
}  // namespace chrono
#endif
