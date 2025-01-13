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
// Author: Arman Pazouki, Wei Hu, Luning Bakke
// =============================================================================

#include <thrust/extrema.h>
#include <thrust/remove.h>
#include <thrust/sort.h>

#include "chrono_fsi/sph/physics/ChFsiForceExplicitSPH.cuh"
#include "chrono_fsi/sph/physics/ChSphGeneral.cuh"
#include "chrono_fsi/sph/math/ExactLinearSolvers.cuh"

namespace chrono {
namespace fsi {
namespace sph {

//--------------------------------------------------------------------------------------------------------------------------------
__device__ __inline__ void calc_G_Matrix(Real4* sortedPosRad,
                                         Real3* sortedVelMas,
                                         Real4* sortedRhoPreMu,
                                         Real* G_i,
                                         const uint* numNeighborsPerPart,
                                         const uint* neighborList,
                                         uint* indexOfIndex) {
    uint id = blockIdx.x * blockDim.x + threadIdx.x;
    if (id >= countersD.numAllMarkers)
        return;

    // uint index = indexOfIndex[id];
    uint index = id;

    if (sortedRhoPreMu[index].w > -0.5f && sortedRhoPreMu[index].w < 0.5f)
        return;

    Real3 posRadA = mR3(sortedPosRad[index]);
    Real SuppRadii = paramsD.h_multiplier * paramsD.h;
    Real SqRadii = SuppRadii * SuppRadii;

    // get address in grid
    int3 gridPos = calcGridPos(posRadA);
    // This is the elements of inverse of G
    Real mGi[9] = {0.0};

    uint NLStart = numNeighborsPerPart[index];
    uint NLEnd = numNeighborsPerPart[index + 1];
    // examine neighbouring cells
    for (int n = NLStart; n < NLEnd; n++) {
        uint j = neighborList[n];
        if (j == index) {
            continue;
        }
        Real3 posRadB = mR3(sortedPosRad[j]);
        Real3 rij = Distance(posRadA, posRadB);
        Real dd = rij.x * rij.x + rij.y * rij.y + rij.z * rij.z;
        if (dd > SqRadii || sortedRhoPreMu[j].w < -1.5)
            continue;
        Real3 grad_i_wij = GradW3h(paramsD.kernel_type, rij, paramsD.ooh);
        Real3 grw_vj = grad_i_wij * paramsD.volume0;
        mGi[0] -= rij.x * grw_vj.x;
        mGi[1] -= rij.x * grw_vj.y;
        mGi[2] -= rij.x * grw_vj.z;
        mGi[3] -= rij.y * grw_vj.x;
        mGi[4] -= rij.y * grw_vj.y;
        mGi[5] -= rij.y * grw_vj.z;
        mGi[6] -= rij.z * grw_vj.x;
        mGi[7] -= rij.z * grw_vj.y;
        mGi[8] -= rij.z * grw_vj.z;
    }

    Real Det = (mGi[0] * mGi[4] * mGi[8] - mGi[0] * mGi[5] * mGi[7] - mGi[1] * mGi[3] * mGi[8] +
                mGi[1] * mGi[5] * mGi[6] + mGi[2] * mGi[3] * mGi[7] - mGi[2] * mGi[4] * mGi[6]);
    if (abs(Det) > 0.01) {
        Real OneOverDet = 1.0 / Det;
        G_i[0] = (mGi[4] * mGi[8] - mGi[5] * mGi[7]) * OneOverDet;
        G_i[1] = -(mGi[1] * mGi[8] - mGi[2] * mGi[7]) * OneOverDet;
        G_i[2] = (mGi[1] * mGi[5] - mGi[2] * mGi[4]) * OneOverDet;
        G_i[3] = -(mGi[3] * mGi[8] - mGi[5] * mGi[6]) * OneOverDet;
        G_i[4] = (mGi[0] * mGi[8] - mGi[2] * mGi[6]) * OneOverDet;
        G_i[5] = -(mGi[0] * mGi[5] - mGi[2] * mGi[3]) * OneOverDet;
        G_i[6] = (mGi[3] * mGi[7] - mGi[4] * mGi[6]) * OneOverDet;
        G_i[7] = -(mGi[0] * mGi[7] - mGi[1] * mGi[6]) * OneOverDet;
        G_i[8] = (mGi[0] * mGi[4] - mGi[1] * mGi[3]) * OneOverDet;
    } else {
        for (int i = 0; i < 9; i++) {
            G_i[i] = 0.0;
        }
        G_i[0] = 1;
        G_i[4] = 1;
        G_i[8] = 1;
    }
}

//--------------------------------------------------------------------------------------------------------------------------------
__device__ __inline__ void calc_A_Matrix(Real4* sortedPosRad,
                                         Real3* sortedVelMas,
                                         Real4* sortedRhoPreMu,
                                         Real* A_i,
                                         Real* G_i,
                                         const uint* numNeighborsPerPart,
                                         const uint* neighborList,
                                         uint* indexOfIndex) {
    uint id = blockIdx.x * blockDim.x + threadIdx.x;
    if (id >= countersD.numAllMarkers)
        return;

    // uint index = indexOfIndex[id];
    uint index = id;

    if (sortedRhoPreMu[index].w > -0.5f && sortedRhoPreMu[index].w < 0.5f)
        return;

    Real3 posRadA = mR3(sortedPosRad[index]);
    Real SuppRadii = paramsD.h_multiplier * paramsD.h;
    Real SqRadii = SuppRadii * SuppRadii;

    // get address in grid
    int3 gridPos = calcGridPos(posRadA);

    uint NLStart = numNeighborsPerPart[index];
    uint NLEnd = numNeighborsPerPart[index + 1];
    // examine neighbouring cells
    for (int n = NLStart; n < NLEnd; n++) {
        uint j = neighborList[n];
        if (j == index) {
            continue;
        }
        Real3 posRadB = mR3(sortedPosRad[j]);
        Real3 rij = Distance(posRadA, posRadB);
        Real dd = rij.x * rij.x + rij.y * rij.y + rij.z * rij.z;
        if (dd > SqRadii || sortedRhoPreMu[j].w < -1.5)
            continue;
        Real3 grad_ij = GradW3h(paramsD.kernel_type, rij, paramsD.ooh);
        Real V_j = paramsD.markerMass / paramsD.rho0;
        Real com_part = 0;
        com_part = (G_i[0] * grad_ij.x + G_i[1] * grad_ij.y + G_i[2] * grad_ij.z) * V_j;
        A_i[0] += rij.x * rij.x * com_part;  // 111
        A_i[1] += rij.x * rij.y * com_part;  // 112
        A_i[2] += rij.x * rij.z * com_part;  // 113
        A_i[3] += rij.y * rij.x * com_part;  // 121
        A_i[4] += rij.y * rij.y * com_part;  // 122
        A_i[5] += rij.y * rij.z * com_part;  // 123
        A_i[6] += rij.z * rij.x * com_part;  // 131
        A_i[7] += rij.z * rij.y * com_part;  // 132
        A_i[8] += rij.z * rij.z * com_part;  // 133
        com_part = (G_i[3] * grad_ij.x + G_i[4] * grad_ij.y + G_i[5] * grad_ij.z) * V_j;
        A_i[9] += rij.x * rij.x * com_part;   // 211
        A_i[10] += rij.x * rij.y * com_part;  // 212
        A_i[11] += rij.x * rij.z * com_part;  // 213
        A_i[12] += rij.y * rij.x * com_part;  // 221
        A_i[13] += rij.y * rij.y * com_part;  // 222
        A_i[14] += rij.y * rij.z * com_part;  // 223
        A_i[15] += rij.z * rij.x * com_part;  // 231
        A_i[16] += rij.z * rij.y * com_part;  // 232
        A_i[17] += rij.z * rij.z * com_part;  // 233
        com_part = (G_i[6] * grad_ij.x + G_i[7] * grad_ij.y + G_i[8] * grad_ij.z) * V_j;
        A_i[18] += rij.x * rij.x * com_part;  // 311
        A_i[19] += rij.x * rij.y * com_part;  // 312
        A_i[20] += rij.x * rij.z * com_part;  // 313
        A_i[21] += rij.y * rij.x * com_part;  // 321
        A_i[22] += rij.y * rij.y * com_part;  // 322
        A_i[23] += rij.y * rij.z * com_part;  // 323
        A_i[24] += rij.z * rij.x * com_part;  // 331
        A_i[25] += rij.z * rij.y * com_part;  // 332
        A_i[26] += rij.z * rij.z * com_part;  // 333
    }
}

//--------------------------------------------------------------------------------------------------------------------------------
__device__ __inline__ void calc_L_Matrix(Real4* sortedPosRad,
                                         Real3* sortedVelMas,
                                         Real4* sortedRhoPreMu,
                                         Real* A_i,
                                         Real* L_i,
                                         Real* G_i,
                                         const uint* numNeighborsPerPart,
                                         const uint* neighborList,
                                         uint* indexOfIndex) {
    uint id = blockIdx.x * blockDim.x + threadIdx.x;
    if (id >= countersD.numAllMarkers)
        return;

    // uint index = indexOfIndex[id];
    uint index = id;

    if (sortedRhoPreMu[index].w > -0.5f && sortedRhoPreMu[index].w < 0.5f)
        return;

    Real3 posRadA = mR3(sortedPosRad[index]);
    Real SuppRadii = paramsD.h_multiplier * paramsD.h;
    Real SqRadii = SuppRadii * SuppRadii;

    Real B[36] = {0.0};
    Real L[6] = {0.0};

    // get address in grid
    int3 gridPos = calcGridPos(posRadA);
    uint NLStart = numNeighborsPerPart[index];
    uint NLEnd = numNeighborsPerPart[index + 1];
    // examine neighbouring cells
    for (int n = NLStart; n < NLEnd; n++) {
        uint j = neighborList[n];
        if (j == index) {
            continue;
        }
        Real3 posRadB = mR3(sortedPosRad[j]);
        Real3 rij = Distance(posRadA, posRadB);
        Real dd = rij.x * rij.x + rij.y * rij.y + rij.z * rij.z;
        if (dd > SqRadii || sortedRhoPreMu[j].w < -1.5)
            continue;
        Real d = length(rij);
        Real3 eij = rij / d;

        Real3 grad_ij = GradW3h(paramsD.kernel_type, rij, paramsD.ooh);
        Real V_j = paramsD.markerMass / paramsD.rho0;
        Real com_part = 0;
        // mn=11

        Real XX = (eij.x * grad_ij.x);
        Real XY = (eij.x * grad_ij.y + eij.y * grad_ij.x);
        Real XZ = (eij.x * grad_ij.z + eij.z * grad_ij.x);
        Real YY = (eij.y * grad_ij.y);
        Real YZ = (eij.y * grad_ij.z + eij.z * grad_ij.y);
        Real ZZ = (eij.z * grad_ij.z);

        com_part = (A_i[0] * eij.x + A_i[9] * eij.y + A_i[18] * eij.z + rij.x * eij.x) * V_j;
        B[6 * 0 + 0] += com_part * XX;  // 11
        B[6 * 0 + 1] += com_part * XY;  // 12
        B[6 * 0 + 2] += com_part * XZ;  // 13
        B[6 * 0 + 3] += com_part * YY;  // 14
        B[6 * 0 + 4] += com_part * YZ;  // 15
        B[6 * 0 + 5] += com_part * ZZ;  // 15
        // mn=12
        com_part = (A_i[1] * eij.x + A_i[10] * eij.y + A_i[19] * eij.z + rij.x * eij.y) * V_j;
        B[6 * 1 + 0] += com_part * XX;  // 21
        B[6 * 1 + 1] += com_part * XY;  // 22
        B[6 * 1 + 2] += com_part * XZ;  // 23
        B[6 * 1 + 3] += com_part * YY;  // 24
        B[6 * 1 + 4] += com_part * YZ;  // 25
        B[6 * 1 + 5] += com_part * ZZ;  // 25

        // mn=13
        com_part = (A_i[2] * eij.x + A_i[11] * eij.y + A_i[20] * eij.z + rij.x * eij.z) * V_j;
        B[6 * 2 + 0] += com_part * XX;  // 31
        B[6 * 2 + 1] += com_part * XY;  // 32
        B[6 * 2 + 2] += com_part * XZ;  // 33
        B[6 * 2 + 3] += com_part * YY;  // 34
        B[6 * 2 + 4] += com_part * YZ;  // 35
        B[6 * 2 + 5] += com_part * ZZ;  // 36

        // Note that we skip mn=21 since it is similar to mn=12
        // mn=22
        com_part = (A_i[4] * eij.x + A_i[13] * eij.y + A_i[22] * eij.z + rij.y * eij.y) * V_j;
        B[6 * 3 + 0] += com_part * XX;  // 41
        B[6 * 3 + 1] += com_part * XY;  // 42
        B[6 * 3 + 2] += com_part * XZ;  // 43
        B[6 * 3 + 3] += com_part * YY;  // 44
        B[6 * 3 + 4] += com_part * YZ;  // 45
        B[6 * 3 + 5] += com_part * ZZ;  // 46

        // mn=23
        com_part = (A_i[5] * eij.x + A_i[14] * eij.y + A_i[23] * eij.z + rij.y * eij.z) * V_j;
        B[6 * 4 + 0] += com_part * XX;  // 51
        B[6 * 4 + 1] += com_part * XY;  // 52
        B[6 * 4 + 2] += com_part * XZ;  // 53
        B[6 * 4 + 3] += com_part * YY;  // 54
        B[6 * 4 + 4] += com_part * YZ;  // 55
        B[6 * 4 + 5] += com_part * ZZ;  // 56
        // mn=33
        com_part = (A_i[8] * eij.x + A_i[17] * eij.y + A_i[26] * eij.z + rij.z * eij.z) * V_j;
        B[6 * 5 + 0] += com_part * XX;  // 61
        B[6 * 5 + 1] += com_part * XY;  // 62
        B[6 * 5 + 2] += com_part * XZ;  // 63
        B[6 * 5 + 3] += com_part * YY;  // 64
        B[6 * 5 + 4] += com_part * YZ;  // 65
        B[6 * 5 + 5] += com_part * ZZ;  // 66
    }

    inv6xdelta_mn(B, L);
    L_i[0] = L[0];
    L_i[1] = L[1];
    L_i[2] = L[2];
    L_i[3] = L[1];
    L_i[4] = L[3];
    L_i[5] = L[4];
    L_i[6] = L[2];
    L_i[7] = L[4];
    L_i[8] = L[5];

    // Real Det = (L_i[0] * L_i[4] * L_i[8] - L_i[0] * L_i[5] * L_i[7] - L_i[1] * L_i[3] * L_i[8] +
    //             L_i[1] * L_i[5] * L_i[6] + L_i[2] * L_i[3] * L_i[7] - L_i[2] * L_i[4] * L_i[6]);
    // if (abs(Det) < 0.01) {
    //     for (int i = 0; i < 9; i++) {
    //         L_i[0 * 9 + i] = 0.0;
    //         L_i[0 * 9 + 0] = 1;
    //         L_i[0 * 9 + 4] = 1;
    //         L_i[0 * 9 + 8] = 1;
    //     }
    // }
    // printf("L Det %f\n", Det);
}

//--------------------------------------------------------------------------------------------------------------------------------
__global__ void calIndexOfIndex(uint* indexOfIndex, uint* identityOfIndex, uint* gridMarkerIndex) {
    uint id = blockIdx.x * blockDim.x + threadIdx.x;
    if (id >= countersD.numAllMarkers)
        return;

    indexOfIndex[id] = id;
    if (gridMarkerIndex[id] >= countersD.numFluidMarkers &&
        gridMarkerIndex[id] < countersD.numFluidMarkers + countersD.numBoundaryMarkers) {
        identityOfIndex[id] = 1;
    } else {
        identityOfIndex[id] = 0;
    }
}

//--------------------------------------------------------------------------------------------------------------------------------
// Only happens for density reinitialization?
//////////////////////////////////////////////////
__global__ void calcRho_kernel(Real4* sortedPosRad,
                               Real4* sortedRhoPreMu,
                               Real4* sortedRhoPreMu_old,
                               const uint* numNeighborsPerPart,
                               const uint* neighborList,
                               int density_reinit,
                               volatile bool* error_flag) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numAllMarkers)
        return;

    if (sortedRhoPreMu[index].w > -0.5 && sortedRhoPreMu[index].w < 0.5)
        return;

    sortedRhoPreMu_old[index].y = Eos(sortedRhoPreMu_old[index].x, paramsD.eos_type);

    Real3 posRadA = mR3(sortedPosRad[index]);
    Real SuppRadii = paramsD.h_multiplier * paramsD.h;
    Real SqRadii = SuppRadii * SuppRadii;

    Real sum_mW = 0;
    Real sum_mW_rho = 0.0000001;
    Real sum_W = 0.0;
    uint NLStart = numNeighborsPerPart[index];
    uint NLEnd = numNeighborsPerPart[index + 1];

    // get address in grid
    int3 gridPos = calcGridPos(posRadA);
    for (int n = NLStart; n < NLEnd; n++) {
        uint j = neighborList[n];
        if (j == index) {
            continue;
        }
        Real3 posRadB = mR3(sortedPosRad[j]);
        Real3 dist3 = Distance(posRadA, posRadB);
        Real dd = dist3.x * dist3.x + dist3.y * dist3.y + dist3.z * dist3.z;
        if (dd > SqRadii)
            continue;
        if (sortedRhoPreMu_old[j].w > -1.5 && sortedRhoPreMu_old[j].w < -0.5) {
            Real m_j = paramsD.markerMass;
            Real d = length(dist3);
            Real W3 = W3h(paramsD.kernel_type, d, paramsD.ooh);
            sum_mW += m_j * W3;
            sum_W += W3;
            sum_mW_rho += m_j * W3 / sortedRhoPreMu_old[j].x;
        }
    }

    // sortedRhoPreMu[index].x = sum_mW;
    if ((density_reinit == 0) && (sortedRhoPreMu[index].w > -1.5) && (sortedRhoPreMu[index].w < -0.5))
        sortedRhoPreMu[index].x = sum_mW / sum_mW_rho;

    if ((sortedRhoPreMu[index].x > 3 * paramsD.rho0 || sortedRhoPreMu[index].x < 0.01 * paramsD.rho0) &&
        (sortedRhoPreMu[index].w > -1.5) && (sortedRhoPreMu[index].w < -0.5))
        printf("(calcRho_kernel)density marker %d, sum_mW=%f, sum_W=%f\n", index, sum_mW, sum_W);
}

//--------------------------------------------------------------------------------------------------------------------------------
__global__ void calcKernelSupport(const Real4* sortedPosRad,
                                  const Real4* sortedRhoPreMu,
                                  Real2* sortedKernelSupport,
                                  const uint* mapOriginalToSorted,
                                  const uint* numNeighborsPerPart,
                                  const uint* neighborList,
                                  volatile bool* error_flag) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numAllMarkers)
        return;

    uint NLStart = numNeighborsPerPart[index];
    uint NLEnd = numNeighborsPerPart[index + 1];
    Real3 posRadA = mR3(sortedPosRad[index]);

    Real W0 = W3h(paramsD.kernel_type, 0, paramsD.ooh);
    Real sum_W_all = W0;
    Real sum_W_identical = W0;
    Real index_type = sortedRhoPreMu[index].w;

    // Use the neighbors list
    for (int i = NLStart; i < NLEnd; i++) {
        uint j = neighborList[i];
        Real3 posRadB = mR3(sortedPosRad[j]);
        Real3 rij = Distance(posRadA, posRadB);
        Real d = length(rij);
        Real W3 = W3h(paramsD.kernel_type, d, paramsD.ooh);
        sum_W_all += W3;
        if (abs(index_type - sortedRhoPreMu[j].w) < 0.001) {
            sum_W_identical += W3;
        }
    }

    sortedKernelSupport[index].x = sum_W_all;
    sortedKernelSupport[index].y = sum_W_identical;
}

//--------------------------------------------------------------------------------------------------------------------------------
__device__ __inline__ void modifyPressure(Real4& rhoPresMuB, const Real3& dist3Alpha) {
    // body force in x direction
    rhoPresMuB.y = (dist3Alpha.x > 0.5 * paramsD.boxDims.x) ? (rhoPresMuB.y - paramsD.delta_pressure.x) : rhoPresMuB.y;
    rhoPresMuB.y = (dist3Alpha.x < -0.5 * paramsD.boxDims.x) ? (rhoPresMuB.y + paramsD.delta_pressure.x) : rhoPresMuB.y;
    // body force in y direction
    rhoPresMuB.y = (dist3Alpha.y > 0.5 * paramsD.boxDims.y) ? (rhoPresMuB.y - paramsD.delta_pressure.y) : rhoPresMuB.y;
    rhoPresMuB.y = (dist3Alpha.y < -0.5 * paramsD.boxDims.y) ? (rhoPresMuB.y + paramsD.delta_pressure.y) : rhoPresMuB.y;
    // body force in z direction
    rhoPresMuB.y = (dist3Alpha.z > 0.5 * paramsD.boxDims.z) ? (rhoPresMuB.y - paramsD.delta_pressure.z) : rhoPresMuB.y;
    rhoPresMuB.y = (dist3Alpha.z < -0.5 * paramsD.boxDims.z) ? (rhoPresMuB.y + paramsD.delta_pressure.z) : rhoPresMuB.y;
}

//--------------------------------------------------------------------------------------------------------------------------------
__device__ inline Real3 CubicSolve(Real aa, Real bb, Real cc, Real dd) {
    Real disc, q, r, dum1, dum2, term1, r13;
    bb /= aa;
    cc /= aa;
    dd /= aa;
    if (aa == 0) {
        return mR3(0, 0, 0);
    }
    if (abs(bb) < 1e-9) {
        return mR3(0, 0, 0);
    }
    if (abs(cc) < 1e-9) {
        return mR3(0, 0, 0);
    }
    if (abs(dd) < 1e-9) {
        return mR3(0, 0, 0);
    }
    q = (3.0 * cc - (bb * bb)) / 9.0;
    r = -(27.0 * dd) + bb * (9.0 * cc - 2.0 * (bb * bb));
    r /= 54.0;
    disc = q * q * q + r * r;
    term1 = (bb / 3.0);

    /*     dataForm.x1Im.value = 0; //The first root is always real.
        if (disc > 0) { // one root real, two are complex
            s = r + Math.sqrt(disc);
            s = ((s < 0) ? -Math.pow(-s, (1.0/3.0)) : Math.pow(s, (1.0/3.0)));
            t = r - Math.sqrt(disc);
            t = ((t < 0) ? -Math.pow(-t, (1.0/3.0)) : Math.pow(t, (1.0/3.0)));
            dataForm.x1Re.value = -term1 + s + t;
            term1 += (s + t)/2.0;
            dataForm.x3Re.value = dataForm.x2Re.value = -term1;
            term1 = Math.sqrt(3.0)*(-t + s)/2;
            dataForm.x2Im.value = term1;
            dataForm.x3Im.value = -term1;
            return;
        }
        // End if (disc > 0)
        // The remaining options are all real
        dataForm.x3Im.value = dataForm.x2Im.value = 0;
        if (disc == 0){ // All roots real, at least two are equal.
            r13 = ((r < 0) ? -Math.pow(-r,(1.0/3.0)) : Math.pow(r,(1.0/3.0)));
            dataForm.x1Re.value = -term1 + 2.0*r13;
            dataForm.x3Re.value = dataForm.x2Re.value = -(r13 + term1);
            return;
        } // End if (disc == 0)
    */

    Real xRex, xRey, xRez;
    // have complex root
    if (disc > 0) {
        xRex = 0.0;
        xRey = 0.0;
        xRez = 0.0;
        return mR3(xRex, xRey, xRez);
    }
    // All roots real, at least two are equal.
    if (disc == 0) {
        if (r < 0) {
            r13 = pow(-r, (1.0 / 3.0));
        } else {
            r13 = pow(r, (1.0 / 3.0));
        }
        xRex = -term1 + 2.0 * r13;
        xRey = -(r13 + term1);
        xRez = xRey;
        return mR3(xRex, xRey, xRez);
    }
    // All roots are real and unequal (to get here, q < 0)
    q = -q;
    dum1 = q * q * q;
    dum2 = r / (sqrt(dum1 + 1.0e-9));
    if ((dum2 >= 0) && (dum2 <= 1)) {
        dum1 = acos(dum2);
    } else {
        xRex = 0.0;
        xRey = 0.0;
        xRez = 0.0;
        return mR3(xRex, xRey, xRez);
    }
    r13 = 2.0 * sqrt(q);
    xRex = -term1 + r13 * cos(dum1 / 3.0);
    xRey = -term1 + r13 * cos((dum1 + 2.0 * 3.1415926) / 3.0);
    xRez = -term1 + r13 * cos((dum1 + 4.0 * 3.1415926) / 3.0);

    return mR3(xRex, xRey, xRez);
}

//--------------------------------------------------------------------------------------------------------------------------------
__device__ inline Real3 CubicEigen(Real4 c1, Real4 c2, Real4 c3) {
    Real a = c1.x;
    Real b = c1.y;
    Real c = c1.z;
    Real d = c1.w;

    Real l = c2.x;
    Real m = c2.y;
    Real n = c2.z;
    Real k = c2.w;

    Real p = c3.x;
    Real q = c3.y;
    Real r = c3.z;
    Real s = c3.w;

    Real D = (a * m * r + b * p * n + c * l * q) - (a * n * q + b * l * r + c * m * p) + 1.0e-9;
    Real x = ((b * r * k + c * m * s + d * n * q) - (b * n * s + c * q * k + d * m * r)) / D;
    Real y = ((a * n * s + c * p * k + d * l * r) - (a * r * k + c * l * s + d * n * p)) / D;
    Real z = ((a * q * k + b * l * s + d * m * p) - (a * m * s + b * p * k + d * l * q)) / D;

    b = b + 1.0e-9;
    x = 1.0e0;
    z = (-l + a * m / b) / (n - c * m / b);
    y = (-a - c * z) / b;
    Real R = sqrt(x * x + y * y + z * z);
    x = x / R;
    y = y / R;
    z = z / R;

    // if(abs(D) < 1){
    //     return mR3(0,0,0);
    // }

    // if(abs(m) < 0.1){
    //     x=0;
    //     y=1;
    //     z=0;
    //     return mR3(x,y,z);
    // }
    // else{
    //     y=0;
    //     if(abs(c) > 0.1){
    //         x=1;
    //         z=-a/c;
    //         return mR3(x,y,z);
    //     }
    //     if(abs(a) > 0.1){
    //         z=1;
    //         x=-c/a;
    //         return mR3(x,y,z);
    //     }
    // }

    return mR3(x, y, z);
}

//--------------------------------------------------------------------------------------------------------------------------------
__device__ inline Real4 DifVelocityRho(Real3 dist3,
                                       Real d,
                                       Real4 posRadA,
                                       Real4 posRadB,
                                       Real3 velMasA,
                                       Real3 velMasB,
                                       Real4 rhoPresMuA,
                                       Real4 rhoPresMuB) {
    if (IsBceMarker(rhoPresMuA.w) && IsBceMarker(rhoPresMuB.w))
        return mR4(0.0);

    Real3 gradW = GradW3h(paramsD.kernel_type, dist3, paramsD.ooh);

    // Continuty equation
    Real derivRho = paramsD.markerMass * dot(velMasA - velMasB, gradW);

    if (paramsD.USE_Delta_SPH) {
        // diffusion term in continuity equation, this helps smoothing out the large oscillation in pressure
        // field see S. Marrone et al., "delta-SPH model for simulating violent impact flows", Computer Methods in
        // Applied Mechanics and Engineering, 200(2011), pp 1526 --1542.
        Real Psi = paramsD.density_delta * paramsD.h * paramsD.Cs * paramsD.markerMass / rhoPresMuB.x * 2. *
                   (rhoPresMuA.x - rhoPresMuB.x) / (d * d + paramsD.epsMinMarkersDis * paramsD.h * paramsD.h);
        derivRho += Psi * dot(dist3, gradW);
    }

    Real3 derivV;
    switch (paramsD.viscosity_type) {
        case ViscosityType::ARTIFICIAL_UNILATERAL: {
            //  pressure component
            derivV = -paramsD.markerMass *
                     (rhoPresMuA.y / (rhoPresMuA.x * rhoPresMuA.x) + rhoPresMuB.y / (rhoPresMuB.x * rhoPresMuB.x)) *
                     gradW;

            // artificial viscosity part, see Monaghan 1997, mainly for water
            Real vAB_dot_rAB = dot(velMasA - velMasB, dist3);
            if (vAB_dot_rAB < 0) {
                Real mu_ab = paramsD.h * vAB_dot_rAB / (d * d + paramsD.epsMinMarkersDis * paramsD.h * paramsD.h);
                Real Pi_ab = -paramsD.Ar_vis_alpha * paramsD.Cs * 2. / (rhoPresMuA.x + rhoPresMuB.x) *
                             paramsD.markerMass * mu_ab;
                derivV.x -= Pi_ab * gradW.x;
                derivV.y -= Pi_ab * gradW.y;
                derivV.z -= Pi_ab * gradW.z;
            }
            break;
        }
        case ViscosityType::LAMINAR: {
            // laminar physics-based viscosity, directly from the Momentum equation, see Arman's PhD thesis, eq.(2.12)
            // and Morris et al.,"Modeling Low Reynolds Number Incompressible Flows Using SPH, 1997" suitable for
            // Poiseulle flow, or oil, honey, etc
            Real rAB_Dot_GradWh = dot(dist3, gradW);
            Real rAB_Dot_GradWh_OverDist = rAB_Dot_GradWh / (d * d + paramsD.epsMinMarkersDis * paramsD.h * paramsD.h);
            derivV = -paramsD.markerMass *
                         (rhoPresMuA.y / (rhoPresMuA.x * rhoPresMuA.x) + rhoPresMuB.y / (rhoPresMuB.x * rhoPresMuB.x)) *
                         gradW +
                     paramsD.markerMass * 8.0f * paramsD.mu0 * rAB_Dot_GradWh_OverDist * (velMasA - velMasB) /
                         square(rhoPresMuA.x + rhoPresMuB.x);
            break;
        }
    }
    return mR4(derivV, derivRho);
}

//--------------------------------------------------------------------------------------------------------------------------------
__device__ inline Real4 DifVelocityRho_ElasticSPH(Real W_ini_inv,
                                                  Real W_AB,
                                                  Real3 gradW,
                                                  Real3 dist3,
                                                  Real d,
                                                  Real invd,
                                                  Real4 posRadA,
                                                  Real4 posRadB,
                                                  Real3 velMasA_in,
                                                  Real3 velMasB_in,
                                                  Real4 rhoPresMuA,
                                                  Real4 rhoPresMuB,
                                                  Real3 tauXxYyZz_A_in,
                                                  Real3 tauXyXzYz_A_in,
                                                  Real3 tauXxYyZz_B_in,
                                                  Real3 tauXyXzYz_B_in) {
    if (IsBceMarker(rhoPresMuA.w) && IsBceMarker(rhoPresMuB.w))
        return mR4(0.0);

    Real3 velMasA = velMasA_in;
    Real3 velMasB = velMasB_in;
    Real3 tauXxYyZz_A = tauXxYyZz_A_in;
    Real3 tauXxYyZz_B = tauXxYyZz_B_in;
    Real3 tauXyXzYz_A = tauXyXzYz_A_in;
    Real3 tauXyXzYz_B = tauXyXzYz_B_in;

    /*if (IsFluidParticle(rhoPresMuA.w) && IsBceMarker(rhoPresMuB.w)) {
        tauXxYyZz_B = tauXxYyZz_A;
        tauXyXzYz_B = tauXyXzYz_A;
        // velMasB = 2.0*velMasB - velMasA; // noslip BC
    }
    if (IsBceMarker(rhoPresMuA.w) && IsFluidParticle(rhoPresMuB.w)) {
        tauXxYyZz_A = tauXxYyZz_B;
        tauXyXzYz_A = tauXyXzYz_B;
        // velMasA = 2.0*velMasA - velMasB; // noslip BC
    }*/

    Real Mass = paramsD.markerMass;
    Real MassOverRho = Mass * paramsD.invrho0 * paramsD.invrho0;
    Real3 MA_gradW = gradW * MassOverRho;

    Real derivVx = (tauXxYyZz_A.x + tauXxYyZz_B.x) * MA_gradW.x + (tauXyXzYz_A.x + tauXyXzYz_B.x) * MA_gradW.y +
                   (tauXyXzYz_A.y + tauXyXzYz_B.y) * MA_gradW.z;
    Real derivVy = (tauXyXzYz_A.x + tauXyXzYz_B.x) * MA_gradW.x + (tauXxYyZz_A.y + tauXxYyZz_B.y) * MA_gradW.y +
                   (tauXyXzYz_A.z + tauXyXzYz_B.z) * MA_gradW.z;
    Real derivVz = (tauXyXzYz_A.y + tauXyXzYz_B.y) * MA_gradW.x + (tauXyXzYz_A.z + tauXyXzYz_B.z) * MA_gradW.y +
                   (tauXxYyZz_A.z + tauXxYyZz_B.z) * MA_gradW.z;

    // TODO: Visco-plastic model
    // Real vel = length(velMasA);
    // if(vel > 0.3){
    //     Real rAB_Dot_GradWh = dot(dist3, gradW);
    //     Real rAB_Dot_GradWh_OverDist = rAB_Dot_GradWh / (d * d + paramsD.epsMinMarkersDis * paramsD.h *
    //     paramsD.h); Real3 derivV = - paramsD.markerMass *(rhoPresMuA.y / (rhoPresMuA.x * rhoPresMuA.x) +
    //     rhoPresMuB.y / (rhoPresMuB.x * rhoPresMuB.x)) * gradW
    //                    + paramsD.markerMass * (8.0f * multViscosity) * paramsD.mu_fric_s
    //                    * pow(rhoPresMuA.x + rhoPresMuB.x, Real(-2)) * rAB_Dot_GradWh_OverDist * (velMasA - velMasB);
    //     derivVx = derivV.x;
    //     derivVy = derivV.y;
    //     derivVz = derivV.z;
    // }
    Real derivM1 = 0.0;
    Real vAB_rAB = dot(velMasA - velMasB, dist3);
    switch (paramsD.viscosity_type) {
        case ViscosityType::ARTIFICIAL_UNILATERAL: {
            // Artificial Viscosity from Monaghan 1997
            // This has no viscous forces in the seperation phase - used in SPH codes simulating fluids
            if (vAB_rAB < 0.0) {
                Real nu = -paramsD.Ar_vis_alpha * paramsD.h * paramsD.Cs * paramsD.invrho0;
                derivM1 = -Mass * (nu * vAB_rAB / (d * d + paramsD.epsMinMarkersDis * paramsD.h * paramsD.h));
            }

            break;
        }
        case ViscosityType::ARTIFICIAL_BILATERAL: {
            // Artificial viscosity treatment from J J Monaghan (2005) "Smoothed particle hydrodynamics"
            // Here there is viscous force added even during the seperation phase - makes the simulation more stable
            Real nu = -paramsD.Ar_vis_alpha * paramsD.h * paramsD.Cs * paramsD.invrho0;
            derivM1 = -Mass * (nu * vAB_rAB / (d * d + paramsD.epsMinMarkersDis * paramsD.h * paramsD.h));
            break;
        }
    }

    derivVx += derivM1 * gradW.x;
    derivVy += derivM1 * gradW.y;
    derivVz += derivM1 * gradW.z;
    // }

    // Artifical pressure to handle tensile instability issue.
    // A complete artifical stress should be implemented in the future.
    /*if (paramsD.Coh_coeff > 1e-5) {
        Real Pa = -1.0 / 3.0 * (tauXxYyZz_A.x + tauXxYyZz_A.y + tauXxYyZz_A.z);
        if (Pa < 0.0) {
            Real Pb = -1.0 / 3.0 * (tauXxYyZz_B.x + tauXxYyZz_B.y + tauXxYyZz_B.z);
            Real epsi = 0.5;
            Real Ra = Pa * epsi * paramsD.invrho0 * paramsD.invrho0;
            Real Rb = Pb * epsi * paramsD.invrho0 * paramsD.invrho0;
            Real fAB = W_AB * W_ini_inv;
            Real small_F = Mass * pow(fAB, 3.0) * (Ra + Rb);
            derivVx += small_F * gradW.x;
            derivVy += small_F * gradW.y;
            derivVz += small_F * gradW.z;
        }
    }*/

    // TOTO: Damping force
    // if (1 == 0) {
    //     Real xi0 = paramsD.Vis_Dam;
    //     Real E0 = paramsD.E_young;
    //     Real h0 = paramsD.h;
    //     Real Cd = xi0 * sqrt(E0 / (rhoA * h0 * h0));
    //     derivVx -= Cd * velMasA.x;
    //     derivVy -= Cd * velMasA.y;
    //     derivVz -= Cd * velMasA.z;
    // }

    // Real derivRho = Mass * dot(vel_XSPH_A - vel_XSPH_B, gradW);
    return mR4(derivVx, derivVy, derivVz, 0.0);
}

//--------------------------------------------------------------------------------------------------------------------------------
__device__ inline Real3 GradientOperator(float G_i[9],
                                         Real3 dist3,
                                         Real4 posRadA,
                                         Real4 posRadB,
                                         Real fA,
                                         Real fB,
                                         Real4 rhoPresMuA,
                                         Real4 rhoPresMuB) {
    Real3 gradW = GradW3h(paramsD.kernel_type, dist3, paramsD.ooh);
    Real3 gradW_new;
    gradW_new.x = G_i[0] * gradW.x + G_i[1] * gradW.y + G_i[2] * gradW.z;
    gradW_new.y = G_i[3] * gradW.x + G_i[4] * gradW.y + G_i[5] * gradW.z;
    gradW_new.z = G_i[6] * gradW.x + G_i[7] * gradW.y + G_i[8] * gradW.z;

    Real Vol = paramsD.markerMass / rhoPresMuB.x;
    Real fji = fB - fA;
    Real Gra_ij_x = fji * gradW_new.x * Vol;
    Real Gra_ij_y = fji * gradW_new.y * Vol;
    Real Gra_ij_z = fji * gradW_new.z * Vol;

    return mR3(Gra_ij_x, Gra_ij_y, Gra_ij_z);
}

//--------------------------------------------------------------------------------------------------------------------------------
__device__ inline Real4 LaplacianOperator(float G_i[9],
                                          float L_i[9],
                                          Real3 dist3,
                                          Real4 posRadA,
                                          Real4 posRadB,
                                          Real fA,
                                          Real fB,
                                          Real4 rhoPresMuA,
                                          Real4 rhoPresMuB) {
    Real3 gradW = GradW3h(paramsD.kernel_type, dist3, paramsD.ooh);
    Real d = length(dist3);
    Real3 eij = dist3 / d;

    Real Vol = paramsD.markerMass / rhoPresMuB.x;
    Real fij = fA - fB;

    Real ex_Gwx = eij.x * gradW.x;
    Real ex_Gwy = eij.x * gradW.y;
    Real ex_Gwz = eij.x * gradW.z;
    Real ey_Gwx = eij.y * gradW.x;
    Real ey_Gwy = eij.y * gradW.y;
    Real ey_Gwz = eij.y * gradW.z;
    Real ez_Gwx = eij.z * gradW.x;
    Real ez_Gwy = eij.z * gradW.y;
    Real ez_Gwz = eij.z * gradW.z;

    Real Part1 = L_i[0] * ex_Gwx + L_i[1] * ex_Gwy + L_i[2] * ex_Gwz + L_i[3] * ey_Gwx + L_i[4] * ey_Gwy +
                 L_i[5] * ey_Gwz + L_i[6] * ez_Gwx + L_i[7] * ez_Gwy + L_i[8] * ez_Gwz;
    Real Part2 = fij / d * Vol;
    Real3 Part3 = mR3(-eij.x, -eij.y, -eij.z) * Vol;

    return mR4(2.0 * Part1 * Part2, Part3.x * (2.0 * Part1), Part3.y * (2.0 * Part1), Part3.z * (2.0 * Part1));
}

// Luning: why is there an upper case EOS?
//__global__ void EOS(Real4* sortedRhoPreMu, volatile bool* isErrorD) {
//    uint index = blockIdx.x * blockDim.x + threadIdx.x;
//    if (index >= numObjectsD.numAllMarkers)
//        return;
//    sortedRhoPreMu[index].y = Eos(sortedRhoPreMu[index].x);
//}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Navier_Stokes(uint* indexOfIndex,
                              Real4* sortedDerivVelRho,
                              Real3* sortedXSPHandShift,
                              Real4* sortedPosRad,
                              Real3* sortedVelMas,
                              Real4* sortedRhoPreMu,
                              uint* gridMarkerIndex,
                              const uint* numNeighborsPerPart,
                              const uint* neighborList,
                              volatile bool* error_flag) {
    uint id = blockIdx.x * blockDim.x + threadIdx.x;
    if (id >= countersD.numAllMarkers)
        return;

    // uint index = indexOfIndex[id];
    uint index = id;

    // Do nothing for fixed wall BCE particles
    if (sortedRhoPreMu[index].w > -0.5 && sortedRhoPreMu[index].w < 0.5) {
        sortedDerivVelRho[index] = mR4(0.0);
        return;
    }

    Real3 posRadA = mR3(sortedPosRad[index]);
    Real3 velMasA = sortedVelMas[index];
    Real4 rhoPresMuA = sortedRhoPreMu[index];
    Real4 derivVelRho = mR4(0.0);
    Real SuppRadii = paramsD.h_multiplier * paramsD.h;
    Real SqRadii = SuppRadii * SuppRadii;

    uint NLStart = numNeighborsPerPart[index];
    uint NLEnd = numNeighborsPerPart[index + 1];

    Real G_i[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    Real L_i[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    if (paramsD.USE_Consistent_G)
        calc_G_Matrix(sortedPosRad, sortedVelMas, sortedRhoPreMu, G_i, numNeighborsPerPart, neighborList, indexOfIndex);

    if (paramsD.USE_Consistent_L) {
        Real A_i[27] = {0.0};
        calc_A_Matrix(sortedPosRad, sortedVelMas, sortedRhoPreMu, A_i, G_i, numNeighborsPerPart, neighborList,
                      indexOfIndex);
        calc_L_Matrix(sortedPosRad, sortedVelMas, sortedRhoPreMu, A_i, L_i, G_i, numNeighborsPerPart, neighborList,
                      indexOfIndex);
    }
    float Gi[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    float Li[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    Gi[0] = G_i[0];
    Gi[1] = G_i[1];
    Gi[2] = G_i[2];
    Gi[3] = G_i[3];
    Gi[4] = G_i[4];
    Gi[5] = G_i[5];
    Gi[6] = G_i[6];
    Gi[7] = G_i[7];
    Gi[8] = G_i[8];
    Li[0] = L_i[0];
    Li[1] = L_i[1];
    Li[2] = L_i[2];
    Li[3] = L_i[3];
    Li[4] = L_i[4];
    Li[5] = L_i[5];
    Li[6] = L_i[6];
    Li[7] = L_i[7];
    Li[8] = L_i[8];

    Real3 preGra = mR3(0.0);
    Real3 velxGra = mR3(0.0);
    Real3 velyGra = mR3(0.0);
    Real3 velzGra = mR3(0.0);
    Real4 velxLap = mR4(0.0);
    Real4 velyLap = mR4(0.0);
    Real4 velzLap = mR4(0.0);

    Real vA = length(velMasA);
    Real vAdT = vA * paramsD.dT;

    // get address in grid
    int3 gridPos = calcGridPos(posRadA);
    Real3 inner_sum = mR3(0.0);
    Real sum_w_i = W3h(paramsD.kernel_type, 0, paramsD.ooh) * paramsD.volume0;

    for (int n = NLStart; n < NLEnd; n++) {
        uint j = neighborList[n];
        if (j == index) {
            continue;
        }
        Real3 posRadB = mR3(sortedPosRad[j]);
        Real3 dist3 = Distance(posRadA, posRadB);
        Real dd = dist3.x * dist3.x + dist3.y * dist3.y + dist3.z * dist3.z;
        if (dd > SqRadii)
            continue;
        Real4 rhoPresMuB = sortedRhoPreMu[j];

        // no solid-solid force
        if (IsBceMarker(rhoPresMuA.w) && IsBceMarker(rhoPresMuB.w))
            continue;

        Real d = length(dist3);

        // modifyPressure(rhoPresMuB, dist3Alpha);
        // if (!IsFinite(rhoPresMuB)) {
        //     printf("Error! particle rhoPresMuB is NAN: thrown from modifyPressure !\n");
        // }
        Real3 velMasB = sortedVelMas[j];

        derivVelRho +=
            DifVelocityRho(dist3, d, sortedPosRad[index], sortedPosRad[j], velMasA, velMasB, rhoPresMuA, rhoPresMuB);

        if (paramsD.USE_Consistent_G && paramsD.USE_Consistent_L) {
            preGra += GradientOperator(Gi, dist3, sortedPosRad[index], sortedPosRad[j], -rhoPresMuA.y, rhoPresMuB.y,
                                       rhoPresMuA, rhoPresMuB);
            velxGra += GradientOperator(Gi, dist3, sortedPosRad[index], sortedPosRad[j], velMasA.x, velMasB.x,
                                        rhoPresMuA, rhoPresMuB);
            velyGra += GradientOperator(Gi, dist3, sortedPosRad[index], sortedPosRad[j], velMasA.y, velMasB.y,
                                        rhoPresMuA, rhoPresMuB);
            velzGra += GradientOperator(Gi, dist3, sortedPosRad[index], sortedPosRad[j], velMasA.z, velMasB.z,
                                        rhoPresMuA, rhoPresMuB);
            velxLap += LaplacianOperator(Gi, Li, dist3, sortedPosRad[index], sortedPosRad[j], velMasA.x, velMasB.x,
                                         rhoPresMuA, rhoPresMuB);
            velyLap += LaplacianOperator(Gi, Li, dist3, sortedPosRad[index], sortedPosRad[j], velMasA.y, velMasB.y,
                                         rhoPresMuA, rhoPresMuB);
            velzLap += LaplacianOperator(Gi, Li, dist3, sortedPosRad[index], sortedPosRad[j], velMasA.z, velMasB.z,
                                         rhoPresMuA, rhoPresMuB);
        }

        if (d > paramsD.h * 1.0e-9)
            sum_w_i = sum_w_i + W3h(paramsD.kernel_type, d, paramsD.ooh) * paramsD.volume0;
    }

    if (paramsD.USE_Consistent_G && paramsD.USE_Consistent_L) {
        Real nu = paramsD.mu0 / paramsD.rho0;
        Real dvxdt = -preGra.x / rhoPresMuA.x +
                     (velxLap.x + velxGra.x * velxLap.y + velxGra.y * velxLap.z + velxGra.z * velxLap.w) * nu;
        Real dvydt = -preGra.y / rhoPresMuA.x +
                     (velyLap.x + velyGra.x * velyLap.y + velyGra.y * velyLap.z + velyGra.z * velyLap.w) * nu;
        Real dvzdt = -preGra.z / rhoPresMuA.x +
                     (velzLap.x + velzGra.x * velzLap.y + velzGra.y * velzLap.z + velzGra.z * velzLap.w) * nu;
        Real drhodt = -paramsD.rho0 * (velxGra.x + velyGra.y + velzGra.z);

        Real Det_G = (Gi[0] * Gi[4] * Gi[8] - Gi[0] * Gi[5] * Gi[7] - Gi[1] * Gi[3] * Gi[8] + Gi[1] * Gi[5] * Gi[6] +
                      Gi[2] * Gi[3] * Gi[7] - Gi[2] * Gi[4] * Gi[6]);
        Real Det_L = (Li[0] * Li[4] * Li[8] - Li[0] * Li[5] * Li[7] - Li[1] * Li[3] * Li[8] + Li[1] * Li[5] * Li[6] +
                      Li[2] * Li[3] * Li[7] - Li[2] * Li[4] * Li[6]);

        if (IsSphParticle(rhoPresMuA.w)) {
            if (Det_G > 0.9 && Det_G < 1.1 && Det_L > 0.9 && Det_L < 1.1 && sum_w_i > 0.9) {
                derivVelRho = mR4(dvxdt, dvydt, dvzdt, drhodt);
            }
        }
    }

    if (!IsFinite(derivVelRho)) {
        printf("Error! particle derivVel is NAN: thrown from ChFsiForceExplicitSPH.cu, collideD !\n");
        *error_flag = true;
    }

    // add gravity and other body force to fluid markers
    if (IsSphParticle(rhoPresMuA.w)) {
        Real3 totalFluidBodyForce3 = paramsD.bodyForce3 + paramsD.gravity;
        derivVelRho += mR4(totalFluidBodyForce3);
    }

    sortedDerivVelRho[index] = derivVelRho;

    Real det_r_max = 0.05 * vAdT;
    Real det_r_A = length(inner_sum);
    if (det_r_A < det_r_max) {
        sortedXSPHandShift[index] = inner_sum;
    } else {
        sortedXSPHandShift[index] = inner_sum * det_r_max / (det_r_A + 1e-9);
    }
}

//--------------------------------------------------------------------------------------------------------------------------------
// Boundary condition application for Navier-Stokes using Holmes's method
// The density and pressure of the BCE markers are extrapolated along with the velocity (no-slip).
// See https://onlinelibrary-wiley-com.ezproxy.library.wisc.edu/doi/pdfdirect/10.1002/nag.898 (for velocity
// and https://www.sciencedirect.com/science/article/pii/S002199911200229X?ref=cra_js_challenge&fr=RR-1 (for pressure
// and density extrapolation)
__global__ void Boundary_NavierStokes_Holmes(const uint* activityIdentifierD,
                                             const uint* numNeighborsPerPart,
                                             const uint* neighborList,
                                             const Real4* sortedPosRadD,
                                             const Real2* sortedKernelSupport,
                                             Real3* bceAcc,
                                             Real4* sortedRhoPresMuD,
                                             Real3* sortedVelMasD,
                                             volatile bool* error_flag) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numAllMarkers)
        return;

    if (activityIdentifierD[index] == 0) {
        return;
    }

    // Ignore all fluid particles
    if (IsFluidParticle(sortedRhoPresMuD[index].w)) {
        return;
    }

    Real3 posRadA = mR3(sortedPosRadD[index]);
    Real SuppRadii = paramsD.h_multiplier * paramsD.h;
    uint NLStart = numNeighborsPerPart[index];
    uint NLEnd = numNeighborsPerPart[index + 1];
    Real sum_pw = 0.0f;
    Real3 sum_rhorw = mR3(0.0);
    Real sum_w = 0.0f;
    Real3 sum_vw = mR3(0.0);

    // Requirements for the Holmes method
    Real2 kernelSupport = sortedKernelSupport[index];
    Real chi_BCE = kernelSupport.x / kernelSupport.y;
    Real dBCE = SuppRadii * (2.0 * chi_BCE - 1.0);
    int predicateBCE = (dBCE < 0.0);
    dBCE = predicateBCE ? 0.01 * SuppRadii : dBCE;
    Real3 prescribedVel = (IsBceSolidMarker(sortedRhoPresMuD[index].w)) ? (sortedVelMasD[index]) : mR3(0.0);
    Real3 velMasB_new = mR3(0.0);

    for (int n = NLStart + 1; n < NLEnd; n++) {
        uint j = neighborList[n];

        // only consider fluid neighbors
        if (IsBceMarker(sortedRhoPresMuD[j].w)) {
            continue;
        }

        Real3 posRadB = mR3(sortedPosRadD[j]);
        Real3 rij = Distance(posRadA, posRadB);
        Real d = length(rij);
        Real W3 = W3h(paramsD.kernel_type, d, paramsD.ooh);
        sum_w += W3;
        sum_pw += sortedRhoPresMuD[j].y * W3;
        sum_rhorw += sortedRhoPresMuD[j].x * rij * W3;
        sum_vw += sortedVelMasD[j] * W3;

        // Compute the shortest perpendicular distance with the information about kernel support
        Real chi_Fluid = sortedKernelSupport[j].x / sortedKernelSupport[j].y;
        Real dFluid = SuppRadii * (2.0 * chi_Fluid - 1.0);
        int predicateFluid = (dFluid < 0.0);
        dFluid = predicateFluid ? 0.01 * SuppRadii : dFluid;

        Real dFluidBCE = dBCE / dFluid;
        // Use predication to avoid branching
        int predicateAB = (dFluidBCE > 0.5);
        dFluidBCE = predicateAB ? 0.5 : dFluidBCE;

        velMasB_new = dFluidBCE * (prescribedVel - sortedVelMasD[j]) + prescribedVel;
    }

    sortedVelMasD[index] = velMasB_new;
    if (sum_w > EPSILON) {
        sortedRhoPresMuD[index].y = (sum_pw + dot(paramsD.gravity - bceAcc[index], sum_rhorw)) / sum_w;
        sortedRhoPresMuD[index].x = InvEos(sortedRhoPresMuD[index].y, paramsD.eos_type);
    } else {
        sortedRhoPresMuD[index].y = 0.0f;
        sortedVelMasD[index] = mR3(0.0);
    }
}

//--------------------------------------------------------------------------------------------------------------------------------
// Boundary condition application for Elastic SPH using Holmes's method
// The Stress tensor of the BCE markers are extrapolated along with the velocity (no-slip). For stress exploration
// an Adami-like method is used. However, for velocity extrapolation, The Holmes method is used.
// See https://www.sciencedirect.com/science/article/pii/S0266352X19300941 (for stress extrapolation)
// and https://onlinelibrary-wiley-com.ezproxy.library.wisc.edu/doi/pdfdirect/10.1002/nag.898 (for velocity
// extrapolation)
__global__ void Boundary_Elastic_Holmes(const uint* activityIdentifierD,
                                        const uint* numNeighborsPerPart,
                                        const uint* neighborList,
                                        const Real4* sortedPosRadD,
                                        const Real2* sortedKernelSupport,
                                        Real3* bceAcc,
                                        Real4* sortedRhoPresMuD,
                                        Real3* sortedVelMasD,
                                        Real3* sortedTauXxYyZz,
                                        Real3* sortedTauXyXzYz,
                                        volatile bool* error_flag) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numAllMarkers)
        return;

    if (activityIdentifierD[index] == 0) {
        return;
    }

    // Ignore all fluid particles
    if (IsFluidParticle(
            sortedRhoPresMuD[index].w)) {  // TODO: This array is only used for obtaining marker type - seems wasteful
        return;
    }

    Real3 posRadA = mR3(sortedPosRadD[index]);
    Real SuppRadii = paramsD.h_multiplier * paramsD.h;
    uint NLStart = numNeighborsPerPart[index];
    uint NLEnd = numNeighborsPerPart[index + 1];
    Real sum_w = 0.0f;
    Real3 sum_vw = mR3(0.0);
    Real3 sum_rhorw = mR3(0.0);
    Real3 sum_tauD = mR3(0.0);
    Real3 sum_tauO = mR3(0.0);

    // Requirements for the Holmes method
    Real2 kernelSupport = sortedKernelSupport[index];
    Real chi_BCE = kernelSupport.x / kernelSupport.y;
    Real dBCE = SuppRadii * (2.0 * chi_BCE - 1.0);
    int predicateBCE = (dBCE < 0.0);
    dBCE = predicateBCE ? 0.01 * SuppRadii : dBCE;
    Real3 prescribedVel = (IsBceSolidMarker(sortedRhoPresMuD[index].w)) ? (sortedVelMasD[index]) : mR3(0.0);
    Real3 velMasB_new = mR3(0.0);

    for (int n = NLStart + 1; n < NLEnd; n++) {
        uint j = neighborList[n];

        // only consider fluid neighbors
        if (IsBceMarker(
                sortedRhoPresMuD[j].w)) {  // TODO: This array is only used for obtaining marker type - seems wasteful
            continue;
        }

        Real3 posRadB = mR3(sortedPosRadD[j]);
        Real3 rij = Distance(posRadA, posRadB);
        Real d = length(rij);
        Real W3 = W3h(paramsD.kernel_type, d, paramsD.ooh);
        sum_w += W3;
        sum_vw += sortedVelMasD[j] * W3;
        sum_rhorw += paramsD.rho0 * rij * W3;  // Since density is constant in Elastic SPH
        sum_tauD += sortedTauXxYyZz[j] * W3;
        sum_tauO += sortedTauXyXzYz[j] * W3;

        // Compute the shortest perpendicular distance with the information about kernel support
        Real chi_Fluid = sortedKernelSupport[j].x / sortedKernelSupport[j].y;
        Real dFluid = SuppRadii * (2.0 * chi_Fluid - 1.0);
        int predicateFluid = (dFluid < 0.0);
        dFluid = predicateFluid ? 0.01 * SuppRadii : dFluid;

        Real dFluidBCE = dBCE / dFluid;
        // Use predication to avoid branching
        int predicateAB = (dFluidBCE > 0.5);
        dFluidBCE = predicateAB ? 0.5 : dFluidBCE;

        velMasB_new = dFluidBCE * (prescribedVel - sortedVelMasD[j]) + prescribedVel;
    }

    sortedVelMasD[index] = velMasB_new;
    if (sum_w > EPSILON) {
        sortedTauXxYyZz[index] = (sum_tauD + dot(paramsD.gravity - bceAcc[index], sum_rhorw)) / sum_w;
        sortedTauXyXzYz[index] = sum_tauO / sum_w;
    } else {
        sortedTauXxYyZz[index] = mR3(0.0);
        sortedTauXyXzYz[index] = mR3(0.0);
    }
}

//--------------------------------------------------------------------------------------------------------------------------------
// Boundary condition application for Navier-Stokes with Adami's method
// The pressure and density of the BCE markers are extrapolated along with the velocity (no-slip)
// See https://www.sciencedirect.com/science/article/pii/S002199911200229X?ref=cra_js_challenge&fr=RR-1
__global__ void Boundary_NavierStokes_Adami(const uint* activityIdentifierD,
                                            const uint* numNeighborsPerPart,
                                            const uint* neighborList,
                                            const Real4* sortedPosRadD,
                                            Real3* bceAcc,
                                            Real4* sortedRhoPresMuD,
                                            Real3* sortedVelMasD,
                                            volatile bool* error_flag) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numAllMarkers)
        return;

    if (activityIdentifierD[index] == 0) {
        return;
    }

    // Ignore all fluid particles
    if (IsFluidParticle(sortedRhoPresMuD[index].w)) {
        return;
    }

    Real3 posRadA = mR3(sortedPosRadD[index]);
    uint NLStart = numNeighborsPerPart[index];
    uint NLEnd = numNeighborsPerPart[index + 1];
    Real sum_pw = 0.0f;
    Real3 sum_rhorw = mR3(0.0);
    Real sum_w = 0.0f;
    Real3 sum_vw = mR3(0.0);

    for (int n = NLStart + 1; n < NLEnd; n++) {
        uint j = neighborList[n];

        // only consider fluid neighbors
        if (IsBceMarker(sortedRhoPresMuD[j].w)) {
            continue;
        }

        Real3 posRadB = mR3(sortedPosRadD[j]);
        Real3 rij = Distance(posRadA, posRadB);
        Real d = length(rij);
        Real W3 = W3h(paramsD.kernel_type, d, paramsD.ooh);
        sum_w += W3;
        sum_pw += sortedRhoPresMuD[j].y * W3;
        sum_rhorw += sortedRhoPresMuD[j].x * rij * W3;
        sum_vw += sortedVelMasD[j] * W3;
    }

    if (sum_w > EPSILON) {
        Real3 prescribedVel = (IsBceSolidMarker(sortedRhoPresMuD[index].w)) ? (2.0f * sortedVelMasD[index]) : mR3(0.0);
        sortedVelMasD[index] = prescribedVel - sum_vw / sum_w;
        sortedRhoPresMuD[index].y = (sum_pw + dot(paramsD.gravity - bceAcc[index], sum_rhorw)) / sum_w;
        sortedRhoPresMuD[index].x = InvEos(sortedRhoPresMuD[index].y, paramsD.eos_type);
    } else {
        sortedVelMasD[index] = mR3(0.0);
        sortedRhoPresMuD[index].y = 0.0f;
        sortedVelMasD[index] = mR3(0.0);
    }
}

//--------------------------------------------------------------------------------------------------------------------------------
// Boundary condition application for Elastic SPH using Adami's method
// The Stress tensor of the BCE markers are extrapolated along with the velocity (no-slip)
// See https://www.sciencedirect.com/science/article/pii/S0266352X19300941 (for stress extrapolation)
// and https://www.sciencedirect.com/science/article/pii/S002199911200229X?ref=cra_js_challenge&fr=RR-1 (for velocity
// extrapolation)
__global__ void Boundary_Elastic_Adami(const uint* activityIdentifierD,
                                       const uint* numNeighborsPerPart,
                                       const uint* neighborList,
                                       const Real4* sortedPosRadD,
                                       Real3* bceAcc,
                                       Real4* sortedRhoPresMuD,
                                       Real3* sortedVelMasD,
                                       Real3* sortedTauXxYyZz,
                                       Real3* sortedTauXyXzYz,
                                       volatile bool* error_flag) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numAllMarkers)
        return;

    if (activityIdentifierD[index] == 0) {
        return;
    }

    // Ignore all fluid particles
    if (IsFluidParticle(
            sortedRhoPresMuD[index].w)) {  // TODO: This array is only used for obtaining marker type - seems wasteful
        return;
    }

    Real3 posRadA = mR3(sortedPosRadD[index]);
    uint NLStart = numNeighborsPerPart[index];
    uint NLEnd = numNeighborsPerPart[index + 1];
    Real sum_w = 0.0f;
    Real3 sum_vw = mR3(0.0);
    Real3 sum_rhorw = mR3(0.0);
    Real3 sum_tauD = mR3(0.0);
    Real3 sum_tauO = mR3(0.0);

    for (int n = NLStart + 1; n < NLEnd; n++) {
        uint j = neighborList[n];

        // only consider fluid neighbors
        if (IsBceMarker(
                sortedRhoPresMuD[j].w)) {  // TODO: This array is only used for obtaining marker type - seems wasteful
            continue;
        }

        Real3 posRadB = mR3(sortedPosRadD[j]);
        Real3 rij = Distance(posRadA, posRadB);
        Real d = length(rij);
        Real W3 = W3h(paramsD.kernel_type, d, paramsD.ooh);
        sum_w += W3;
        sum_vw += sortedVelMasD[j] * W3;
        sum_rhorw += paramsD.rho0 * rij * W3;  // Since density is constant in Elastic SPH
        sum_tauD += sortedTauXxYyZz[j] * W3;
        sum_tauO += sortedTauXyXzYz[j] * W3;
    }

    if (sum_w > EPSILON) {
        Real3 prescribedVel = (IsBceSolidMarker(sortedRhoPresMuD[index].w)) ? (2.0f * sortedVelMasD[index]) : mR3(0.0);
        sortedVelMasD[index] = prescribedVel - sum_vw / sum_w;
        sortedTauXxYyZz[index] = (sum_tauD + dot(paramsD.gravity - bceAcc[index], sum_rhorw)) / sum_w;
        sortedTauXyXzYz[index] = sum_tauO / sum_w;
    } else {
        sortedVelMasD[index] = mR3(0.0);
        sortedTauXxYyZz[index] = mR3(0.0);
        sortedTauXyXzYz[index] = mR3(0.0);
    }
}

//--------------------------------------------------------------------------------------------------------------------------------
__global__ void NS_SSR(const uint* activityIdentifierD,
                       const Real4* sortedPosRad,
                       const Real3* sortedVelMas,
                       const Real4* sortedRhoPreMu,
                       const Real3* sortedTauXxYyZz,
                       const Real3* sortedTauXyXzYz,
                       const uint* numNeighborsPerPart,
                       const uint* neighborList,
                       Real4* sortedDerivVelRho,
                       Real3* sortedDerivTauXxYyZz,
                       Real3* sortedDerivTauXyXzYz,
                       Real3* sortedXSPHandShift,
                       uint* sortedFreeSurfaceIdD,
                       volatile bool* error_flag) {
    uint id = blockIdx.x * blockDim.x + threadIdx.x;
    if (id >= countersD.numAllMarkers)
        return;

    // uint index = sortedActivityIdD[id];
    uint index = id;
    if (activityIdentifierD[index] == 0) {
        return;
    }

    if (IsBceWallMarker(sortedRhoPreMu[index].w))
        return;

    Real3 posRadA = mR3(sortedPosRad[index]);
    Real3 velMasA = sortedVelMas[index];
    Real4 rhoPresMuA = sortedRhoPreMu[index];
    Real3 TauXxYyZzA = sortedTauXxYyZz[index];
    Real3 TauXyXzYzA = sortedTauXyXzYz[index];
    Real4 derivVelRho = mR4(0.0);
    Real3 deltaV = mR3(0.0);

    Real tauxx = sortedTauXxYyZz[index].x;
    Real tauyy = sortedTauXxYyZz[index].y;
    Real tauzz = sortedTauXxYyZz[index].z;
    Real tauxy = sortedTauXyXzYz[index].x;
    Real tauxz = sortedTauXyXzYz[index].y;
    Real tauyz = sortedTauXyXzYz[index].z;
    Real dTauxx = 0.0f;
    Real dTauyy = 0.0f;
    Real dTauzz = 0.0f;
    Real dTauxy = 0.0f;
    Real dTauxz = 0.0f;
    Real dTauyz = 0.0f;
    uint NLStart = numNeighborsPerPart[index];
    uint NLEnd = numNeighborsPerPart[index + 1];

    // Calculate the correction matrix for gradient operator
    Real G_i[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    if (paramsD.USE_Consistent_G) {
        Real mGi[9] = {0.0};
        for (int n = NLStart; n < NLEnd; n++) {
            uint j = neighborList[n];
            Real3 posRadB = mR3(sortedPosRad[j]);
            Real3 rij = Distance(posRadA, posRadB);
            Real3 grad_i_wij = GradW3h(paramsD.kernel_type, rij, paramsD.ooh);
            Real3 grw_vj = grad_i_wij * paramsD.volume0;
            mGi[0] -= rij.x * grw_vj.x;
            mGi[1] -= rij.x * grw_vj.y;
            mGi[2] -= rij.x * grw_vj.z;
            mGi[3] -= rij.y * grw_vj.x;
            mGi[4] -= rij.y * grw_vj.y;
            mGi[5] -= rij.y * grw_vj.z;
            mGi[6] -= rij.z * grw_vj.x;
            mGi[7] -= rij.z * grw_vj.y;
            mGi[8] -= rij.z * grw_vj.z;
        }
        Real Det = (mGi[0] * mGi[4] * mGi[8] - mGi[0] * mGi[5] * mGi[7] - mGi[1] * mGi[3] * mGi[8] +
                    mGi[1] * mGi[5] * mGi[6] + mGi[2] * mGi[3] * mGi[7] - mGi[2] * mGi[4] * mGi[6]);
        if (abs(Det) > 0.01) {
            Real OneOverDet = 1.0 / Det;
            G_i[0] = (mGi[4] * mGi[8] - mGi[5] * mGi[7]) * OneOverDet;
            G_i[1] = -(mGi[1] * mGi[8] - mGi[2] * mGi[7]) * OneOverDet;
            G_i[2] = (mGi[1] * mGi[5] - mGi[2] * mGi[4]) * OneOverDet;
            G_i[3] = -(mGi[3] * mGi[8] - mGi[5] * mGi[6]) * OneOverDet;
            G_i[4] = (mGi[0] * mGi[8] - mGi[2] * mGi[6]) * OneOverDet;
            G_i[5] = -(mGi[0] * mGi[5] - mGi[2] * mGi[3]) * OneOverDet;
            G_i[6] = (mGi[3] * mGi[7] - mGi[4] * mGi[6]) * OneOverDet;
            G_i[7] = -(mGi[0] * mGi[7] - mGi[1] * mGi[6]) * OneOverDet;
            G_i[8] = (mGi[0] * mGi[4] - mGi[1] * mGi[3]) * OneOverDet;
        }
    }

    Real radii = paramsD.d0 * Real(1.241);  // 1.129;
    Real invRadii = 1 / radii;

    Real vA = length(velMasA);
    Real vAdT = vA * paramsD.dT;
    Real bs_vAdT = paramsD.beta_shifting * vAdT;

    Real3 inner_sum = mR3(0.0);
    Real sum_w_i = W3h(paramsD.kernel_type, 0, paramsD.ooh) * paramsD.volume0;
    Real w_ini_inv = 1 / W3h(paramsD.kernel_type, paramsD.d0, paramsD.ooh);
    int N_ = 1;
    int N_s = 0;

    // Get the interaction from neighbor particles
    // NLStart + 1 because the first element in neighbor list is the particle itself
    for (int n = NLStart + 1; n < NLEnd; n++) {
        uint j = neighborList[n];
        Real4 rhoPresMuB = sortedRhoPreMu[j];
        if (IsBceMarker(rhoPresMuA.w) && IsBceMarker(rhoPresMuB.w))
            continue;  // No BCE-BCE interaction

        Real3 posRadB = mR3(sortedPosRad[j]);
        Real3 dist3 = Distance(posRadA, posRadB);
        Real d = length(dist3);
        Real invd = 1 / d;
        Real3 velMasB = sortedVelMas[j];
        Real3 TauXxYyZzB = sortedTauXxYyZz[j];
        Real3 TauXyXzYzB = sortedTauXyXzYz[j];

        // Correct the kernel function gradient
        Real w_AB = W3h(paramsD.kernel_type, d, paramsD.ooh);
        Real3 gradW = GradW3h(paramsD.kernel_type, dist3, paramsD.ooh);
        if (paramsD.USE_Consistent_G) {
            Real3 gradW_new;
            gradW_new.x = G_i[0] * gradW.x + G_i[1] * gradW.y + G_i[2] * gradW.z;
            gradW_new.y = G_i[3] * gradW.x + G_i[4] * gradW.y + G_i[5] * gradW.z;
            gradW_new.z = G_i[6] * gradW.x + G_i[7] * gradW.y + G_i[8] * gradW.z;
            gradW = gradW_new;
        }
        // Calculate dv/dt
        derivVelRho += DifVelocityRho_ElasticSPH(w_ini_inv, w_AB, gradW, dist3, d, invd, sortedPosRad[index],
                                                 sortedPosRad[j], velMasA, velMasB, rhoPresMuA, rhoPresMuB, TauXxYyZzA,
                                                 TauXyXzYzA, TauXxYyZzB, TauXyXzYzB);
        // Calculate dsigma/dt
        if (sortedRhoPreMu[index].w < -0.5f) {
            // start to calculate the stress rate
            Real3 vAB = velMasA - velMasB;
            Real3 vAB_h = 0.5f * vAB * paramsD.volume0;
            // entries of strain rate tensor
            Real exx = -2.0f * vAB_h.x * gradW.x;
            Real eyy = -2.0f * vAB_h.y * gradW.y;
            Real ezz = -2.0f * vAB_h.z * gradW.z;
            Real exy = -vAB_h.x * gradW.y - vAB_h.y * gradW.x;
            Real exz = -vAB_h.x * gradW.z - vAB_h.z * gradW.x;
            Real eyz = -vAB_h.y * gradW.z - vAB_h.z * gradW.y;
            // entries of rotation rate (spin) tensor
            Real wxy = -vAB_h.x * gradW.y + vAB_h.y * gradW.x;
            Real wxz = -vAB_h.x * gradW.z + vAB_h.z * gradW.x;
            Real wyz = -vAB_h.y * gradW.z + vAB_h.z * gradW.y;

            Real edia = 0.3333333333333f * (exx + eyy + ezz);
            Real twoG = 2.0f * paramsD.G_shear;
            Real K_edia = paramsD.K_bulk * 1.0 * edia;
            dTauxx += twoG * (exx - edia) + 2.0f * (tauxy * wxy + tauxz * wxz) + K_edia;
            dTauyy += twoG * (eyy - edia) - 2.0f * (tauxy * wxy - tauyz * wyz) + K_edia;
            dTauzz += twoG * (ezz - edia) - 2.0f * (tauxz * wxz + tauyz * wyz) + K_edia;
            dTauxy += twoG * exy - (tauxx * wxy - tauxz * wyz) + (wxy * tauyy + wxz * tauyz);
            dTauxz += twoG * exz - (tauxx * wxz + tauxy * wyz) + (wxy * tauyz + wxz * tauzz);
            dTauyz += twoG * eyz - (tauxy * wxz + tauyy * wyz) - (wxy * tauxz - wyz * tauzz);
        }

        // Do integration for the kernel function, calculate the XSPH term
        if (d > paramsD.h * 1.0e-9f) {
            Real Wab = W3h(paramsD.kernel_type, d, paramsD.ooh);

            // Integration of the kernel function
            sum_w_i += Wab * paramsD.volume0;

            // XSPH
            if (IsSphParticle(rhoPresMuB.w))
                deltaV += paramsD.volume0 * (velMasB - velMasA) * Wab;

            N_ = N_ + 1;
        }

        // Find particles that have contact with this particle
        if (IsFluidParticle(rhoPresMuB.w) && d < 1.25f * radii) {
            Real Pen = (radii - d) * invRadii;
            Real3 r_0 = bs_vAdT * invd * dist3;
            Real3 r_s = r_0 * Pen;
            if (d < 1.0f * radii) {
                inner_sum += 3.0f * r_s;
            } else if (d < 1.1f * radii) {
                inner_sum += 1.0f * r_s;
            } else {
                inner_sum += 0.1f * 1.0f * (-r_0);
            }

            N_s = N_s + 1;
        }
    }

    // Check particles who have not enough neighbor particles (only for granular now)
    if (sum_w_i < paramsD.C_Wi) {
        sortedFreeSurfaceIdD[index] = 1;
    } else {
        sortedFreeSurfaceIdD[index] = 0;
    }

    // Calculate the shifting vector
    Real det_r_max = 0.05f * vAdT;
    Real det_r_A = length(inner_sum);
    if (det_r_A < det_r_max) {
        sortedXSPHandShift[index] = inner_sum;
    } else {
        sortedXSPHandShift[index] = inner_sum * det_r_max / (det_r_A + 1e-9f);
    }

    // Add the XSPH term into the shifting vector
    sortedXSPHandShift[index] += paramsD.EPS_XSPH * deltaV * paramsD.dT;

    // Get the shifting velocity
    sortedXSPHandShift[index] = sortedXSPHandShift[index] / paramsD.dT;

    // Add gravity and other body force to fluid markers
    if (IsSphParticle(rhoPresMuA.w)) {
        Real3 totalFluidBodyForce3 = paramsD.bodyForce3 + paramsD.gravity;
        derivVelRho += mR4(totalFluidBodyForce3, 0.0f);
    }

    sortedDerivVelRho[index] = derivVelRho;
    sortedDerivTauXxYyZz[index] = mR3(dTauxx, dTauyy, dTauzz);
    sortedDerivTauXyXzYz[index] = mR3(dTauxy, dTauxz, dTauyz);
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void CalcVel_XSPH_D(uint* indexOfIndex,
                               Real3* vel_XSPH_Sorted_D,
                               Real4* sortedPosRad,
                               Real3* sortedVelMas,
                               Real4* sortedRhoPreMu,
                               uint* gridMarkerIndex,
                               const uint* numNeighborsPerPart,
                               const uint* neighborList,
                               volatile bool* error_flag) {
    uint id = blockIdx.x * blockDim.x + threadIdx.x;
    if (id >= countersD.numAllMarkers)
        return;

    // uint index = indexOfIndex[id];
    uint index = id;

    // Do nothing for wall
    if (sortedRhoPreMu[index].w > -0.5 && sortedRhoPreMu[index].w < 0.5) {
        return;
    }

    Real4 rhoPreMuA = sortedRhoPreMu[index];
    Real3 velMasA = sortedVelMas[index];
    Real SuppRadii = paramsD.h_multiplier * paramsD.h;
    Real SqRadii = SuppRadii * SuppRadii;
    uint NLStart = numNeighborsPerPart[index];
    uint NLEnd = numNeighborsPerPart[index + 1];

    Real3 posRadA = mR3(sortedPosRad[index]);
    Real3 deltaV = mR3(0);

    // get address in grid
    int3 gridPos = calcGridPos(posRadA);
    Real3 inner_sum = mR3(0.0);
    // Real mi_bar = 0.0, r0 = 0.0;
    Real3 dV = mR3(0.0f);
    // examine neighbouring cells
    for (int n = NLStart; n < NLEnd; n++) {
        uint j = neighborList[n];
        if (j == index) {
            continue;
        }
        Real3 posRadB = mR3(sortedPosRad[j]);
        Real3 dist3 = Distance(posRadA, posRadB);
        Real dd = dist3.x * dist3.x + dist3.y * dist3.y + dist3.z * dist3.z;
        if (dd > SqRadii)
            continue;
        Real4 rhoPresMuB = sortedRhoPreMu[j];
        if (!IsSphParticle(rhoPresMuB.w))
            continue;
        Real3 velMasB = sortedVelMas[j];
        Real rho_bar = 0.5 * (rhoPreMuA.x + rhoPresMuB.x);
        Real d = length(dist3);
        deltaV += paramsD.markerMass * (velMasB - velMasA) * W3h(paramsD.kernel_type, d, paramsD.ooh) / rho_bar;
    }

    vel_XSPH_Sorted_D[index] = paramsD.EPS_XSPH * deltaV + vel_XSPH_Sorted_D[index] / paramsD.dT;

    if (!IsFinite(vel_XSPH_Sorted_D[index])) {
        printf("Error! particle vXSPH is NAN: thrown from ChFsiForceExplicitSPH.cu, CalcVel_XSPH_D !\n");
        *error_flag = true;
    }
}

// ===============================================================================================================================

ChFsiForceExplicitSPH::ChFsiForceExplicitSPH(FsiDataManager& data_mgr, BceManager& bce_mgr, bool verbose)
    : ChFsiForce(data_mgr, bce_mgr, verbose) {
    CopyParametersToDevice(m_data_mgr.paramsH, m_data_mgr.countersH);
    density_initialization = 0;
}

ChFsiForceExplicitSPH::~ChFsiForceExplicitSPH() {}

//--------------------------------------------------------------------------------------------------------------------------------

void ChFsiForceExplicitSPH::Initialize() {
    ChFsiForce::Initialize();
    cudaMemcpyToSymbolAsync(paramsD, m_data_mgr.paramsH.get(), sizeof(SimParams));
    cudaMemcpyToSymbolAsync(countersD, m_data_mgr.countersH.get(), sizeof(Counters));
}

//--------------------------------------------------------------------------------------------------------------------------------
void ChFsiForceExplicitSPH::ForceSPH(std::shared_ptr<SphMarkerDataD> sortedSphMarkers_D,
                                     Real time,
                                     bool firstHalfStep) {
    m_sortedSphMarkers_D = sortedSphMarkers_D;
    m_bce_mgr.updateBCEAcc();
    CollideWrapper(time, firstHalfStep);
    CalculateXSPH_velocity();
}

//--------------------------------------------------------------------------------------------------------------------------------
void ChFsiForceExplicitSPH::neighborSearch() {
    bool* error_flagD;
    cudaMallocErrorFlag(error_flagD);
    cudaResetErrorFlag(error_flagD);

    // thread per particle
    uint numBlocksShort, numThreadsShort;
    computeGridSize(m_data_mgr.countersH->numAllMarkers, 256, numBlocksShort, numThreadsShort);

    // Execute the kernel
    thrust::fill(m_data_mgr.numNeighborsPerPart.begin(), m_data_mgr.numNeighborsPerPart.end(), 0);

    // start neighbor search
    // first pass
    neighborSearchNum<<<numBlocksShort, numThreadsShort>>>(
        mR4CAST(m_sortedSphMarkers_D->posRadD), mR4CAST(m_sortedSphMarkers_D->rhoPresMuD),
        U1CAST(m_data_mgr.markersProximity_D->cellStartD), U1CAST(m_data_mgr.markersProximity_D->cellEndD),
        U1CAST(m_data_mgr.activityIdentifierD), U1CAST(m_data_mgr.numNeighborsPerPart), error_flagD);
    cudaCheckErrorFlag(error_flagD, "neighborSearchNum");

    // in-place exclusive scan for num of neighbors
    thrust::exclusive_scan(m_data_mgr.numNeighborsPerPart.begin(), m_data_mgr.numNeighborsPerPart.end(),
                           m_data_mgr.numNeighborsPerPart.begin());
    // std::cout << "numNeighbors: " << m_data_mgr.numNeighborsPerPart.back() << std::endl;
    m_data_mgr.neighborList.resize(m_data_mgr.numNeighborsPerPart.back());
    thrust::fill(m_data_mgr.neighborList.begin(), m_data_mgr.neighborList.end(), 0);

    // second pass
    neighborSearchID<<<numBlocksShort, numThreadsShort>>>(
        mR4CAST(m_sortedSphMarkers_D->posRadD), mR4CAST(m_sortedSphMarkers_D->rhoPresMuD),
        U1CAST(m_data_mgr.markersProximity_D->cellStartD), U1CAST(m_data_mgr.markersProximity_D->cellEndD),
        U1CAST(m_data_mgr.activityIdentifierD), U1CAST(m_data_mgr.numNeighborsPerPart), U1CAST(m_data_mgr.neighborList),
        error_flagD);
    cudaCheckErrorFlag(error_flagD, "neighborSearchID");

    cudaFreeErrorFlag(error_flagD);
}

//--------------------------------------------------------------------------------------------------------------------------------
void ChFsiForceExplicitSPH::CollideWrapper(Real time, bool firstHalfStep) {
    bool* error_flagD;
    cudaMallocErrorFlag(error_flagD);
    cudaResetErrorFlag(error_flagD);

    // thread per particle
    uint numBlocks, numThreads;
    computeGridSize((int)m_data_mgr.countersH->numAllMarkers, 256, numBlocks, numThreads);

    // Re-Initialize the density after several time steps if needed
    if (density_initialization >= m_data_mgr.paramsH->densityReinit) {
        thrust::device_vector<Real4> rhoPresMuD_old = m_sortedSphMarkers_D->rhoPresMuD;
        printf("Re-initializing density after %d steps.\n", m_data_mgr.paramsH->densityReinit);
        calcRho_kernel<<<numBlocks, numThreads>>>(mR4CAST(m_sortedSphMarkers_D->posRadD),
                                                  mR4CAST(m_sortedSphMarkers_D->rhoPresMuD), mR4CAST(rhoPresMuD_old),
                                                  U1CAST(m_data_mgr.numNeighborsPerPart),
                                                  U1CAST(m_data_mgr.neighborList), density_initialization, error_flagD);
        cudaCheckErrorFlag(error_flagD, "calcRho_kernel");
        density_initialization = 0;
    }
    density_initialization++;

    // Perform Proxmity search at specified frequency
    if (firstHalfStep &&
        (time < 1e-6 ||
         int(round(time / m_data_mgr.paramsH->dT)) % m_data_mgr.paramsH->num_proximity_search_steps == 0))
        neighborSearch();

    // Execute the kernel
    if (m_data_mgr.paramsH->elastic_SPH) {  // For granular material
        if (m_data_mgr.paramsH->boundary_type == BoundaryType::ADAMI) {
            Boundary_Elastic_Adami<<<numBlocks, numThreads>>>(
                U1CAST(m_data_mgr.activityIdentifierD), U1CAST(m_data_mgr.numNeighborsPerPart),
                U1CAST(m_data_mgr.neighborList), mR4CAST(m_sortedSphMarkers_D->posRadD), mR3CAST(m_data_mgr.bceAcc),
                mR4CAST(m_sortedSphMarkers_D->rhoPresMuD), mR3CAST(m_sortedSphMarkers_D->velMasD),
                mR3CAST(m_sortedSphMarkers_D->tauXxYyZzD), mR3CAST(m_sortedSphMarkers_D->tauXyXzYzD), error_flagD);
            cudaCheckErrorFlag(error_flagD, "Boundary_ElasticSPH_Adami");
        } else {
            thrust::device_vector<Real2> sortedKernelSupport(m_data_mgr.countersH->numAllMarkers);
            // Calculate the kernel support of each particle
            calcKernelSupport<<<numBlocks, numThreads>>>(
                mR4CAST(m_sortedSphMarkers_D->posRadD), mR4CAST(m_sortedSphMarkers_D->rhoPresMuD),
                mR2CAST(sortedKernelSupport), U1CAST(m_data_mgr.markersProximity_D->mapOriginalToSorted),
                U1CAST(m_data_mgr.numNeighborsPerPart), U1CAST(m_data_mgr.neighborList), error_flagD);
            cudaCheckErrorFlag(error_flagD, "calcKernelSupport");
            // https://onlinelibrary-wiley-com.ezproxy.library.wisc.edu/doi/pdfdirect/10.1002/nag.898
            Boundary_Elastic_Holmes<<<numBlocks, numThreads>>>(
                U1CAST(m_data_mgr.activityIdentifierD), U1CAST(m_data_mgr.numNeighborsPerPart),
                U1CAST(m_data_mgr.neighborList), mR4CAST(m_sortedSphMarkers_D->posRadD), mR2CAST(sortedKernelSupport),
                mR3CAST(m_data_mgr.bceAcc), mR4CAST(m_sortedSphMarkers_D->rhoPresMuD),
                mR3CAST(m_sortedSphMarkers_D->velMasD), mR3CAST(m_sortedSphMarkers_D->tauXxYyZzD),
                mR3CAST(m_sortedSphMarkers_D->tauXyXzYzD), error_flagD);
            cudaCheckErrorFlag(error_flagD, "Boundary_ElasticSPH_Holmes");
        }

        // execute the kernel Navier_Stokes and Shear_Stress_Rate in one kernel
        NS_SSR<<<numBlocks, numThreads>>>(
            U1CAST(m_data_mgr.activityIdentifierD), mR4CAST(m_sortedSphMarkers_D->posRadD),
            mR3CAST(m_sortedSphMarkers_D->velMasD), mR4CAST(m_sortedSphMarkers_D->rhoPresMuD),
            mR3CAST(m_sortedSphMarkers_D->tauXxYyZzD), mR3CAST(m_sortedSphMarkers_D->tauXyXzYzD),
            U1CAST(m_data_mgr.numNeighborsPerPart), U1CAST(m_data_mgr.neighborList), mR4CAST(m_data_mgr.derivVelRhoD),
            mR3CAST(m_data_mgr.derivTauXxYyZzD), mR3CAST(m_data_mgr.derivTauXyXzYzD), mR3CAST(m_data_mgr.vel_XSPH_D),
            U1CAST(m_data_mgr.freeSurfaceIdD), error_flagD);
        cudaCheckErrorFlag(error_flagD, "NS_SSR");

    } else {  // For fluid
        if (m_data_mgr.paramsH->boundary_type == BoundaryType::ADAMI) {
            Boundary_NavierStokes_Adami<<<numBlocks, numThreads>>>(
                U1CAST(m_data_mgr.activityIdentifierD), U1CAST(m_data_mgr.numNeighborsPerPart),
                U1CAST(m_data_mgr.neighborList), mR4CAST(m_sortedSphMarkers_D->posRadD), mR3CAST(m_data_mgr.bceAcc),
                mR4CAST(m_sortedSphMarkers_D->rhoPresMuD), mR3CAST(m_sortedSphMarkers_D->velMasD), error_flagD);
            cudaCheckErrorFlag(error_flagD, "Boundary_NavierStokes_Adami");
        } else {
            thrust::device_vector<Real2> sortedKernelSupport(m_data_mgr.countersH->numAllMarkers);
            // Calculate the kernel support of each particle
            calcKernelSupport<<<numBlocks, numThreads>>>(
                mR4CAST(m_sortedSphMarkers_D->posRadD), mR4CAST(m_sortedSphMarkers_D->rhoPresMuD),
                mR2CAST(sortedKernelSupport), U1CAST(m_data_mgr.markersProximity_D->mapOriginalToSorted),
                U1CAST(m_data_mgr.numNeighborsPerPart), U1CAST(m_data_mgr.neighborList), error_flagD);
            cudaCheckErrorFlag(error_flagD, "calcKernelSupport");
            // https://onlinelibrary-wiley-com.ezproxy.library.wisc.edu/doi/pdfdirect/10.1002/nag.898
            Boundary_NavierStokes_Holmes<<<numBlocks, numThreads>>>(
                U1CAST(m_data_mgr.activityIdentifierD), U1CAST(m_data_mgr.numNeighborsPerPart),
                U1CAST(m_data_mgr.neighborList), mR4CAST(m_sortedSphMarkers_D->posRadD), mR2CAST(sortedKernelSupport),
                mR3CAST(m_data_mgr.bceAcc), mR4CAST(m_sortedSphMarkers_D->rhoPresMuD),
                mR3CAST(m_sortedSphMarkers_D->velMasD), error_flagD);
            cudaCheckErrorFlag(error_flagD, "Boundary_NavierStokes_Holmes");
        }

        // Find the index which is related to the wall boundary particle
        thrust::device_vector<uint> indexOfIndex(m_data_mgr.countersH->numAllMarkers);
        thrust::device_vector<uint> identityOfIndex(m_data_mgr.countersH->numAllMarkers);
        calIndexOfIndex<<<numBlocks, numThreads>>>(U1CAST(indexOfIndex), U1CAST(identityOfIndex),
                                                   U1CAST(m_data_mgr.markersProximity_D->gridMarkerIndexD));
        thrust::remove_if(indexOfIndex.begin(), indexOfIndex.end(), identityOfIndex.begin(), thrust::identity<int>());

        // execute the kernel
        // TOUnderstand: Why is the blocks NumBlocks1 and threads NumThreads1?
        Navier_Stokes<<<numBlocks, numThreads>>>(
            U1CAST(indexOfIndex), mR4CAST(m_data_mgr.derivVelRhoD), mR3CAST(m_data_mgr.vel_XSPH_D),
            mR4CAST(m_sortedSphMarkers_D->posRadD), mR3CAST(m_sortedSphMarkers_D->velMasD),
            mR4CAST(m_sortedSphMarkers_D->rhoPresMuD), U1CAST(m_data_mgr.markersProximity_D->gridMarkerIndexD),
            U1CAST(m_data_mgr.numNeighborsPerPart), U1CAST(m_data_mgr.neighborList), error_flagD);
        cudaCheckErrorFlag(error_flagD, "Navier_Stokes");
    }

    cudaFreeErrorFlag(error_flagD);
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChFsiForceExplicitSPH::CalculateXSPH_velocity() {
    bool* error_flagD;
    cudaMallocErrorFlag(error_flagD);
    cudaResetErrorFlag(error_flagD);

    // Calculate vel_XSPH
    if (m_data_mgr.vel_XSPH_D.size() != m_data_mgr.countersH->numAllMarkers) {
        printf("m_data_mgr.vel_XSPH_D.size() %zd countersH->numAllMarkers %zd \n", m_data_mgr.vel_XSPH_D.size(),
               m_data_mgr.countersH->numAllMarkers);
        throw std::runtime_error(
            "Error! size error m_data_mgr.vel_XSPH_D Thrown from "
            "CalculateXSPH_velocity!\n");
    }

    if (!m_data_mgr.paramsH->elastic_SPH) {
        // thread per particle
        uint numBlocks, numThreads;
        computeGridSize((int)m_data_mgr.countersH->numAllMarkers, 256, numBlocks, numThreads);

        thrust::fill(m_data_mgr.vel_XSPH_D.begin(), m_data_mgr.vel_XSPH_D.end(), mR3(0.0));

        // Find the index which is related to the wall boundary particle
        thrust::device_vector<uint> indexOfIndex(m_data_mgr.countersH->numAllMarkers);
        thrust::device_vector<uint> identityOfIndex(m_data_mgr.countersH->numAllMarkers);
        calIndexOfIndex<<<numBlocks, numThreads>>>(U1CAST(indexOfIndex), U1CAST(identityOfIndex),
                                                   U1CAST(m_data_mgr.markersProximity_D->gridMarkerIndexD));
        thrust::remove_if(indexOfIndex.begin(), indexOfIndex.end(), identityOfIndex.begin(), thrust::identity<int>());

        // Execute the kernel
        CalcVel_XSPH_D<<<numBlocks, numThreads>>>(
            U1CAST(indexOfIndex), mR3CAST(m_data_mgr.vel_XSPH_D), mR4CAST(m_sortedSphMarkers_D->posRadD),
            mR3CAST(m_sortedSphMarkers_D->velMasD), mR4CAST(m_sortedSphMarkers_D->rhoPresMuD),
            U1CAST(m_data_mgr.markersProximity_D->gridMarkerIndexD), U1CAST(m_data_mgr.numNeighborsPerPart),
            U1CAST(m_data_mgr.neighborList), error_flagD);
        cudaCheckErrorFlag(error_flagD, "CalcVel_XSPH_D");
    }

    cudaFreeErrorFlag(error_flagD);
}

}  // namespace sph
}  // namespace fsi
}  // namespace chrono
