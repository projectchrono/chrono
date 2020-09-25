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
// Author: Arman Pazouki, Milad Rakhsha
// =============================================================================
//
// Base class for processing boundary condition enforcing (bce) markers forces
// in fsi system.//
// =============================================================================

#include "chrono_fsi/physics/ChBce.cuh"  //for FsiGeneralData

namespace chrono {
namespace fsi {

// double precision atomic add function
__device__ double atomicAdd(double* address, double val) {
    unsigned long long int* address_as_ull = (unsigned long long int*)address;

    unsigned long long int old = *address_as_ull, assumed;

    do {
        assumed = old;
        old = atomicCAS(address_as_ull, assumed, __double_as_longlong(val + __longlong_as_double(assumed)));
    } while (assumed != old);

    return __longlong_as_double(old);
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Populate_RigidSPH_MeshPos_LRF_kernel(Real3* rigidSPH_MeshPos_LRF_D,
                                                     Real4* posRadD,
                                                     uint* rigidIdentifierD,
                                                     Real3* posRigidD,
                                                     Real4* qD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numRigid_SphMarkers) {
        return;
    }
    int rigidIndex = rigidIdentifierD[index];
    uint rigidMarkerIndex = index + numObjectsD.startRigidMarkers;  // updatePortion = [start, end]
                                                                    // index of the update portion
    Real4 q4 = qD[rigidIndex];
    Real3 a1, a2, a3;
    RotationMatirixFromQuaternion(a1, a2, a3, q4);
    Real3 dist3 = mR3(posRadD[rigidMarkerIndex]) - posRigidD[rigidIndex];
    Real3 dist3LF = InverseRotate_By_RotationMatrix_DeviceHost(a1, a2, a3, dist3);
    rigidSPH_MeshPos_LRF_D[index] = dist3LF;
}

//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Populate_FlexSPH_MeshPos_LRF_kernel(Real3* FlexSPH_MeshPos_LRF_D,
                                                    Real3* FlexSPH_MeshPos_LRF_H,
                                                    Real4* posRadD,
                                                    uint* FlexIdentifierD,
                                                    const int numFlex1D,
                                                    uint2* CableElementsNodes,
                                                    uint4* ShellElementsNodes,
                                                    Real3* pos_fsi_fea_D,
                                                    Real Spacing) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numFlex_SphMarkers) {
        return;
    }
    //  int numFlexSphMarkers = numObjectsD.numFlex_SphMarkers;

    int FlexIndex = FlexIdentifierD[index];
    uint FlexMarkerIndex = index + numObjectsD.startFlexMarkers;  // updatePortion = [start, end]
    //  printf("FlexInd ex=%d, FlexMarkerIndex=%d\n", FlexIndex, FlexMarkerIndex);

    if (FlexIndex < numFlex1D) {
        uint2 cableNodes = CableElementsNodes[FlexIndex];
        //    printf("FlexIndex=%d, CableElementsNodes[FlexIndex]=%d,%d\n", FlexIndex, cableNodes.x, cableNodes.y);

        Real3 pos_fsi_fea_D_nA = pos_fsi_fea_D[cableNodes.x];
        Real3 pos_fsi_fea_D_nB = pos_fsi_fea_D[cableNodes.y];
        Real3 dist3 = mR3(posRadD[FlexMarkerIndex]) - pos_fsi_fea_D_nA;
        Real3 x_dir = pos_fsi_fea_D_nB - pos_fsi_fea_D_nA;
        Real Cable_x = length(x_dir);
        //    printf("dist3= (%f, %f, %f)\n", dist3.x, dist3.y, dist3.z);
        x_dir = x_dir / length(x_dir);
        Real norm_dir_length = length(cross(dist3, x_dir));

        Real3 y_dir = mR3(-x_dir.y, x_dir.x, 0) + mR3(-x_dir.z, 0, x_dir.x) + mR3(0, -x_dir.z, x_dir.y);
        y_dir = y_dir / length(y_dir);
        Real3 z_dir = cross(x_dir, y_dir);
        //    printf("x_dir= (%f, %f, %f), y_dir= (%f, %f, %f), z_dir= (%f, %f, %f)\n", x_dir.x, x_dir.y, x_dir.z,
        //    y_dir.x,
        //           y_dir.y, y_dir.z, z_dir.x, z_dir.y, z_dir.z);
        Real dx = dot(dist3, x_dir);
        Real dy = dot(dist3, y_dir);
        Real dz = dot(dist3, z_dir);
        if (abs(dy) > 0)
            dy /= Spacing;
        if (abs(dz) > 0)
            dz /= Spacing;

        //                FlexSPH_MeshPos_LRF_D[index] = mR3(FlexSPH_MeshPos_LRF_H[index].x, dy, dz);
        FlexSPH_MeshPos_LRF_D[index] = mR3(dx / Cable_x, dy, dz);

        //        printf("FlexSPH_MeshPos_LRF_D[%d]=%f, %f, %f, Cable_x=%f, dx=%f\n", index,
        //        FlexSPH_MeshPos_LRF_D[index].x,
        //               FlexSPH_MeshPos_LRF_D[index].y, FlexSPH_MeshPos_LRF_D[index].z, Cable_x, dx);
    }
    if (FlexIndex >= numFlex1D) {
        uint4 shellNodes = ShellElementsNodes[FlexIndex - numFlex1D];
        //    printf("FlexIndex=%d, ShellElementsNodes[FlexIndex]=%d,%d,%d,%d\n", FlexIndex, shellNodes.x, shellNodes.y,
        //           shellNodes.z, shellNodes.w);
        Real3 pos_fsi_fea_D_nA = pos_fsi_fea_D[shellNodes.x];
        Real3 pos_fsi_fea_D_nB = pos_fsi_fea_D[shellNodes.y];
        Real3 pos_fsi_fea_D_nC = pos_fsi_fea_D[shellNodes.z];
        Real3 pos_fsi_fea_D_nD = pos_fsi_fea_D[shellNodes.w];

        Real3 Shell_center = 0.25 * (pos_fsi_fea_D_nA + pos_fsi_fea_D_nB + pos_fsi_fea_D_nC + pos_fsi_fea_D_nD);
        Real Shell_x = 0.25 * length(pos_fsi_fea_D_nB - pos_fsi_fea_D_nA + pos_fsi_fea_D_nC - pos_fsi_fea_D_nD);
        Real Shell_y = 0.25 * length(pos_fsi_fea_D_nD - pos_fsi_fea_D_nA + pos_fsi_fea_D_nC - pos_fsi_fea_D_nB);
        Real3 dist3 = mR3(posRadD[FlexMarkerIndex]) - Shell_center;

        Real3 physic_to_natural = mR3(1.0 / Shell_x, 1.0 / Shell_y, 1);
        Real3 pos_physical = FlexSPH_MeshPos_LRF_H[index];
        Real3 pos_natural = mR3(pos_physical.x * physic_to_natural.x, pos_physical.y * physic_to_natural.y,
                                pos_physical.z * physic_to_natural.z);
        //        Real4 N_shell = Shells_ShapeFunctions(pos_natural.x, pos_natural.y);

        Real3 n1 = normalize(cross(pos_fsi_fea_D_nB - pos_fsi_fea_D_nA, pos_fsi_fea_D_nC - pos_fsi_fea_D_nB));
        Real3 n2 = normalize(cross(pos_fsi_fea_D_nC - pos_fsi_fea_D_nB, pos_fsi_fea_D_nD - pos_fsi_fea_D_nC));
        Real3 n3 = normalize(cross(pos_fsi_fea_D_nD - pos_fsi_fea_D_nC, pos_fsi_fea_D_nA - pos_fsi_fea_D_nD));
        Real3 n4 = normalize(cross(pos_fsi_fea_D_nA - pos_fsi_fea_D_nD, pos_fsi_fea_D_nB - pos_fsi_fea_D_nA));
        Real3 Normal = normalize(n1 + n2 + n3 + n4);
        Real zSide = dot(Normal, dist3) / Spacing;

        //        Real3 inPlanePoint = mR3(posRadD[FlexMarkerIndex]) - dot(Normal, dist3) * Normal;
        //
        //        dist3 = inPlanePoint - Shell_center;

        //        Real3 x_dir = normalize((pos_fsi_fea_D_nB - pos_fsi_fea_D_nA) + (pos_fsi_fea_D_nC -
        //        pos_fsi_fea_D_nD)); Real3 y_dir = normalize((pos_fsi_fea_D_nD - pos_fsi_fea_D_nA) + (pos_fsi_fea_D_nC
        //        - pos_fsi_fea_D_nB)); Real3 Normal = normalize(cross(x_dir, y_dir)); Real3 y_dir = cross(Normal,
        //        x_dir);

        //        Real dx = dot(dist3, x_dir);
        //        Real dy = dot(dist3, y_dir);

        //        Real2 eta = mR2(0.), zeta = mR2(0.);
        //        Real2 p = mR2(dx, dy);
        //        Real2 p1 = mR2(dot(pos_fsi_fea_D_nA - Shell_center, x_dir), dot(pos_fsi_fea_D_nA - Shell_center,
        //        y_dir)); Real2 p2 = mR2(dot(pos_fsi_fea_D_nB - Shell_center, x_dir), dot(pos_fsi_fea_D_nB -
        //        Shell_center, y_dir)); Real2 p3 = mR2(dot(pos_fsi_fea_D_nC - Shell_center, x_dir),
        //        dot(pos_fsi_fea_D_nC - Shell_center, y_dir)); Real2 p4 = mR2(dot(pos_fsi_fea_D_nD - Shell_center,
        //        x_dir), dot(pos_fsi_fea_D_nD - Shell_center, y_dir)); solver2x2(p1, p2, p3, p4, p, eta, zeta);

        //        printf(" FlexIndex=%d FlexMarkerIndex:%d center=%f,%f,%f x_dir=%f,%f,%f, y_dir= %f,%f,%f\n",
        //        FlexIndex,
        //               FlexMarkerIndex, Shell_center.x, Shell_center.y, Shell_center.z, x_dir.x, x_dir.y, x_dir.z,
        //               y_dir.x, y_dir.y, y_dir.z);

        //        printf(" FlexMarkerIndex:%d center=%f,%f,%f dist3=%f,%f,%f, normal= %f,%f,%f, zside= %f\n",
        //        FlexMarkerIndex,
        //               Shell_center.x, Shell_center.y, Shell_center.z, dist3.x, dist3.y, dist3.z, Normal.x, Normal.y,
        //               Normal.z, zSide);

        //        FlexSPH_MeshPos_LRF_D[index] = mR3(dx / Shell_x, dy / Shell_y, zSide);
        //        FlexSPH_MeshPos_LRF_D[index] = mR3(eta.x, zeta.y, zSide);
        //        FlexSPH_MeshPos_LRF_D[index] = mR3(pos_natural.x, pos_natural.y, zSide);
        FlexSPH_MeshPos_LRF_D[index] = FlexSPH_MeshPos_LRF_H[index];

        //        printf("FlexIndex=%d FlexMarkerIndex:%d FlexSPH_MeshPos_LRF_D[index]=%f,%f,%f\n", FlexIndex,
        //        FlexMarkerIndex,
        //               FlexSPH_MeshPos_LRF_D[index].x, FlexSPH_MeshPos_LRF_D[index].y,
        //               FlexSPH_MeshPos_LRF_D[index].z);
    }
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Calc_Flex_FSI_ForcesD(Real3* FlexSPH_MeshPos_LRF_D,
                                      uint* FlexIdentifierD,
                                      const int numFlex1D,
                                      uint2* CableElementsNodes,  // This is the connectivity of FEA mesh.
                                      uint4* ShellElementsNodes,  // This is the connectivity of FEA mesh.
                                      Real4* derivVelRhoD,
                                      Real4* derivVelRhoD_old,
                                      Real3* pos_fsi_fea_D,
                                      Real3* Flex_FSI_ForcesD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numFlex_SphMarkers) {
        return;
    }
    //  int numFlexSphMarkers = numObjectsD.numFlex_SphMarkers;

    int FlexIndex = FlexIdentifierD[index];
    uint FlexMarkerIndex = index + numObjectsD.startFlexMarkers;  // updatePortion = [start, end]
    derivVelRhoD[FlexMarkerIndex] =
        derivVelRhoD[FlexMarkerIndex] * paramsD.Beta + derivVelRhoD_old[FlexMarkerIndex] * (1 - paramsD.Beta);
    if (FlexIndex < numFlex1D) {
        Real2 N_cable = Cables_ShapeFunctions(FlexSPH_MeshPos_LRF_D[index].x);
        Real NA = N_cable.x;
        Real NB = N_cable.y;

        int nA = CableElementsNodes[FlexIndex].x;
        int nB = CableElementsNodes[FlexIndex].y;
        //        printf(" FlexMarkerIndex=%d, FlexIndex=%d nA:%d nB=%d, xi=%f, idx=%d, N.A= %f, N.B=%f\n",
        //        FlexMarkerIndex,
        //               FlexIndex, nA, nB, FlexSPH_MeshPos_LRF_D[index].x, index, NA, NB);

        atomicAdd(&(Flex_FSI_ForcesD[nA].x), NA * (double)derivVelRhoD[FlexMarkerIndex].x);
        atomicAdd(&(Flex_FSI_ForcesD[nA].y), NA * (double)derivVelRhoD[FlexMarkerIndex].y);
        atomicAdd(&(Flex_FSI_ForcesD[nA].z), NA * (double)derivVelRhoD[FlexMarkerIndex].z);

        atomicAdd(&(Flex_FSI_ForcesD[nB].x), NB * (double)derivVelRhoD[FlexMarkerIndex].x);
        atomicAdd(&(Flex_FSI_ForcesD[nB].y), NB * (double)derivVelRhoD[FlexMarkerIndex].y);
        atomicAdd(&(Flex_FSI_ForcesD[nB].z), NB * (double)derivVelRhoD[FlexMarkerIndex].z);
    }
    if (FlexIndex >= numFlex1D) {
        Real4 N_shell = Shells_ShapeFunctions(FlexSPH_MeshPos_LRF_D[index].x, FlexSPH_MeshPos_LRF_D[index].y);

        Real NA = N_shell.x;
        Real NB = N_shell.y;
        Real NC = N_shell.z;
        Real ND = N_shell.w;

        int nA = ShellElementsNodes[FlexIndex - numFlex1D].x;
        int nB = ShellElementsNodes[FlexIndex - numFlex1D].y;
        int nC = ShellElementsNodes[FlexIndex - numFlex1D].z;
        int nD = ShellElementsNodes[FlexIndex - numFlex1D].w;
        //        printf(
        //            "FlexMarkerIndex=%d, FlexIndex=%d nA,nB,nC,nD=%d,%d,%d,%d, N_shell= %f,%f,%f,%f, "
        //            "FlexSPH_.x=%f, FlexSPH_.y=%f\n",
        //            FlexMarkerIndex, FlexIndex, nA, nB, nC, nD, N_shell.x, N_shell.y, N_shell.z, N_shell.w,
        //            FlexSPH_MeshPos_LRF_D[index].x, FlexSPH_MeshPos_LRF_D[index].y);

        atomicAdd(&(Flex_FSI_ForcesD[nA].x), NA * (double)derivVelRhoD[FlexMarkerIndex].x);
        atomicAdd(&(Flex_FSI_ForcesD[nA].y), NA * (double)derivVelRhoD[FlexMarkerIndex].y);
        atomicAdd(&(Flex_FSI_ForcesD[nA].z), NA * (double)derivVelRhoD[FlexMarkerIndex].z);

        atomicAdd(&(Flex_FSI_ForcesD[nB].x), NB * (double)derivVelRhoD[FlexMarkerIndex].x);
        atomicAdd(&(Flex_FSI_ForcesD[nB].y), NB * (double)derivVelRhoD[FlexMarkerIndex].y);
        atomicAdd(&(Flex_FSI_ForcesD[nB].z), NB * (double)derivVelRhoD[FlexMarkerIndex].z);

        atomicAdd(&(Flex_FSI_ForcesD[nC].x), NC * (double)derivVelRhoD[FlexMarkerIndex].x);
        atomicAdd(&(Flex_FSI_ForcesD[nC].y), NC * (double)derivVelRhoD[FlexMarkerIndex].y);
        atomicAdd(&(Flex_FSI_ForcesD[nC].z), NC * (double)derivVelRhoD[FlexMarkerIndex].z);

        atomicAdd(&(Flex_FSI_ForcesD[nD].x), ND * (double)derivVelRhoD[FlexMarkerIndex].x);
        atomicAdd(&(Flex_FSI_ForcesD[nD].y), ND * (double)derivVelRhoD[FlexMarkerIndex].y);
        atomicAdd(&(Flex_FSI_ForcesD[nD].z), ND * (double)derivVelRhoD[FlexMarkerIndex].z);
    }
}
//--------------------------------------------------------------------------------------------------------------------------------
// collide a particle against all other particles in a given cell
// Arman : revisit equation 10 of tech report, is it only on fluid or it is on
// all markers
__device__ void BCE_modification_Share(Real3& sumVW,
                                       Real3& sumRhoRW,
                                       Real& sumPW,
                                       Real& sumWFluid,
                                       int& isAffectedV,
                                       int& isAffectedP,
                                       int3 gridPos,
                                       Real3 posRadA,
                                       Real4* sortedPosRad,
                                       Real3* sortedVelMas,
                                       Real4* sortedRhoPreMu,
                                       uint* cellStart,
                                       uint* cellEnd) {
    uint gridHash = calcGridHash(gridPos);
    // get start of bucket for this cell
    uint startIndex = cellStart[gridHash];
    uint endIndex = cellEnd[gridHash];

    for (uint j = startIndex; j < endIndex; j++) {
        Real3 posRadB = mR3(sortedPosRad[j]);
        Real3 dist3 = Distance(posRadA, posRadB);
        Real d = length(dist3);
        Real4 rhoPresMuB = sortedRhoPreMu[j];
        if (d > RESOLUTION_LENGTH_MULT * paramsD.HSML || rhoPresMuB.w > -1.0)
            continue;

        Real Wd = W3h(d, sortedPosRad[j].w);
        Real3 velMasB = sortedVelMas[j];
        sumVW += velMasB * Wd;
        sumRhoRW += rhoPresMuB.x * dist3 * Wd;
        sumPW += rhoPresMuB.y * Wd;
        sumWFluid += Wd;
    }
}

//--------------------------------------------------------------------------------------------------------------------------------
__global__ void new_BCE_VelocityPressure(Real4* velMassRigid_fsiBodies_D,
                                         uint* rigidIdentifierD,
                                         Real3* velMas_ModifiedBCE,    // input: sorted velocities
                                         Real4* rhoPreMu_ModifiedBCE,  // input: sorted velocities
                                         Real4* sortedPosRad,          // input: sorted positions
                                         Real3* sortedVelMas,          // input: sorted velocities
                                         Real4* sortedRhoPreMu,
                                         uint* cellStart,
                                         uint* cellEnd,
                                         uint* mapOriginalToSorted,

                                         Real3* bceAcc,
                                         int3 updatePortion,
                                         volatile bool* isErrorD) {
    uint bceIndex = blockIdx.x * blockDim.x + threadIdx.x;
    uint sphIndex = bceIndex + updatePortion.x;  // updatePortion = [start, end] index of the update portion
    if (bceIndex >= updatePortion.z - updatePortion.x) {
        return;
    }

    uint idA = mapOriginalToSorted[sphIndex];

    Real4 rhoPreMuA = sortedRhoPreMu[idA];
    Real3 posRadA = mR3(sortedPosRad[idA]);
    Real3 velMasA = sortedVelMas[idA];
    int isAffectedV = 0;
    int isAffectedP = 0;
    //    Real3 v_p;
    //    if (bceIndex > updatePortion.y) {
    //        int rigidIndex = rigidIdentifierD[bceIndex - updatePortion.y];
    //        v_p = mR3(velMassRigid_fsiBodies_D[rigidIndex]);
    //        if (bceIndex == 10057)
    //            printf(" rigidIndex=%d, bceIndex=%d, v_p=%f,%f,%f, velMasA=%f,%f,%f\n", rigidIndex, bceIndex, v_p.x,
    //            v_p.y,
    //                   v_p.z, velMasA.x, velMasA.y, velMasA.z);
    //
    //    } else {
    //        v_p = mR3(0.0);
    //    }

    Real3 sumVW = mR3(0);
    Real3 sumRhoRW = mR3(0);
    Real sumPW = 0;
    Real sumWFluid = 0;

    // get address in grid
    int3 gridPos = calcGridPos(posRadA);

    /// if (gridPos.x == paramsD.gridSize.x-1) printf("****aha %d %d\n",
    /// gridPos.x, paramsD.gridSize.x);

    // examine neighbouring cells
    for (int z = -1; z <= 1; z++) {
        for (int y = -1; y <= 1; y++) {
            for (int x = -1; x <= 1; x++) {
                int3 neighbourPos = gridPos + mI3(x, y, z);
                BCE_modification_Share(sumVW, sumRhoRW, sumPW, sumWFluid, isAffectedV, isAffectedP, neighbourPos,
                                       posRadA, sortedPosRad, sortedVelMas, sortedRhoPreMu, cellStart, cellEnd);
            }
        }
    }

    if (abs(sumWFluid) > EPSILON) {
        Real3 modifiedBCE_v = 2 * velMasA - sumVW / sumWFluid;
        velMas_ModifiedBCE[bceIndex] = modifiedBCE_v;
        // pressure
        Real3 a3 = mR3(0);
        if (fabs(rhoPreMuA.w) > 0) {  // rigid BCE
            int rigidBceIndex = sphIndex - numObjectsD.startRigidMarkers;
            if (rigidBceIndex < 0 || rigidBceIndex >= numObjectsD.numRigid_SphMarkers) {
                printf(
                    "Error! marker index out of bound: thrown from "
                    "SDKCollisionSystem.cu, new_BCE_VelocityPressure !\n");
                *isErrorD = true;
                return;
            }
            a3 = bceAcc[rigidBceIndex];
        }
        Real pressure = (sumPW + dot(paramsD.gravity - a3, sumRhoRW)) / sumWFluid;  //(in fact:  (paramsD.gravity -
        // aW), but aW for moving rigids
        // is hard to calc. Assume aW is
        // zero for now
        Real density = InvEos(pressure);
        rhoPreMu_ModifiedBCE[bceIndex] = mR4(density, pressure, rhoPreMuA.z, rhoPreMuA.w);
    } else {
        rhoPreMu_ModifiedBCE[bceIndex] = mR4(paramsD.rho0, paramsD.BASEPRES, paramsD.mu0, rhoPreMuA.w);
        velMas_ModifiedBCE[bceIndex] = mR3(0.0);
    }

    sortedVelMas[idA] = velMas_ModifiedBCE[bceIndex];
    sortedRhoPreMu[idA] = rhoPreMu_ModifiedBCE[bceIndex];
}
//--------------------------------------------------------------------------------------------------------------------------------
// calculate marker acceleration, required in ADAMI
__global__ void calcBceAcceleration_kernel(Real3* bceAcc,
                                           Real4* q_fsiBodies_D,
                                           Real3* accRigid_fsiBodies_D,
                                           Real3* omegaVelLRF_fsiBodies_D,
                                           Real3* omegaAccLRF_fsiBodies_D,
                                           Real3* rigidSPH_MeshPos_LRF_D,
                                           const uint* rigidIdentifierD) {
    uint bceIndex = blockIdx.x * blockDim.x + threadIdx.x;
    if (bceIndex >= numObjectsD.numRigid_SphMarkers) {
        return;
    }

    int rigidBodyIndex = rigidIdentifierD[bceIndex];
    Real3 acc3 = accRigid_fsiBodies_D[rigidBodyIndex];  // linear acceleration (CM)

    Real4 q4 = q_fsiBodies_D[rigidBodyIndex];
    Real3 a1, a2, a3;
    RotationMatirixFromQuaternion(a1, a2, a3, q4);
    Real3 wVel3 = omegaVelLRF_fsiBodies_D[rigidBodyIndex];
    Real3 rigidSPH_MeshPos_LRF = rigidSPH_MeshPos_LRF_D[bceIndex];
    Real3 wVelCrossS = cross(wVel3, rigidSPH_MeshPos_LRF);
    Real3 wVelCrossWVelCrossS = cross(wVel3, wVelCrossS);
    acc3 += mR3(dot(a1, wVelCrossWVelCrossS), dot(a2, wVelCrossWVelCrossS),
                dot(a3, wVelCrossWVelCrossS));  // centrigugal acceleration

    Real3 wAcc3 = omegaAccLRF_fsiBodies_D[rigidBodyIndex];
    Real3 wAccCrossS = cross(wAcc3, rigidSPH_MeshPos_LRF);
    acc3 += mR3(dot(a1, wAccCrossS), dot(a2, wAccCrossS), dot(a3, wAccCrossS));  // tangential acceleration

    //	printf("linear acc %f %f %f point acc %f %f %f \n", accRigid3.x,
    // accRigid3.y, accRigid3.z, acc3.x, acc3.y,
    // acc3.z);
    bceAcc[bceIndex] = acc3;
}
//--------------------------------------------------------------------------------------------------------------------------------
// updates the rigid body particles
__global__ void UpdateRigidMarkersPositionVelocityD(Real4* posRadD,
                                                    Real3* velMasD,
                                                    Real3* rigidSPH_MeshPos_LRF_D,
                                                    uint*  rigidIdentifierD,
                                                    Real3* posRigidD,
                                                    Real4* velMassRigidD,
                                                    Real3* omegaLRF_D,
                                                    Real4* qD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numRigid_SphMarkers) {
        return;
    }
    uint rigidMarkerIndex = index + numObjectsD.startRigidMarkers;  // updatePortion = [start, end]
                                                                    // index of the update portion
    int rigidBodyIndex = rigidIdentifierD[index];

    Real4 q4 = qD[rigidBodyIndex];
    Real3 a1, a2, a3;
    RotationMatirixFromQuaternion(a1, a2, a3, q4);

    Real3 rigidSPH_MeshPos_LRF = rigidSPH_MeshPos_LRF_D[index];

    // position
    Real h = posRadD[rigidMarkerIndex].w;
    Real3 p_Rigid = posRigidD[rigidBodyIndex];
    Real3 pos = p_Rigid + mR3(dot(a1, rigidSPH_MeshPos_LRF), dot(a2, rigidSPH_MeshPos_LRF), dot(a3, rigidSPH_MeshPos_LRF));
    posRadD[rigidMarkerIndex] = mR4(pos, h);

    // velocity
    Real4 vM_Rigid = velMassRigidD[rigidBodyIndex];
    Real3 omega3 = omegaLRF_D[rigidBodyIndex];
    Real3 omegaCrossS = cross(omega3, rigidSPH_MeshPos_LRF);
    velMasD[rigidMarkerIndex] = mR3(vM_Rigid) + mR3(dot(a1, omegaCrossS), dot(a2, omegaCrossS), dot(a3, omegaCrossS));
}
//--------------------------------------------------------------------------------------------------------------------------------
// Real3 *posRadD, uint *FlexIdentifierD, Real3 *posFlex_fsiBodies_nA_D, Real3 *posFlex_fsiBodies_nB_D,
//    Real3 *posFlex_fsiBodies_nC_D, Real3 *posFlex_fsiBodies_nD_D

__global__ void UpdateFlexMarkersPositionVelocityAccD(Real4* posRadD,
                                                      Real3* FlexSPH_MeshPos_LRF_D,
                                                      Real3* velMasD,
                                                      const uint* FlexIdentifierD,
                                                      const int numFlex1D,
                                                      uint2* CableElementsNodes,
                                                      uint4* ShellelementsNodes,
                                                      Real3* pos_fsi_fea_D,
                                                      Real3* vel_fsi_fea_D,
                                                      Real Spacing) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numFlex_SphMarkers) {
        return;
    }
    //  int numFlexSphMarkers = numObjectsD.numFlex_SphMarkers;

    int FlexIndex = FlexIdentifierD[index];
    //  printf(" %d FlexIndex= %d\n", index, FlexIndex);

    uint FlexMarkerIndex = index + numObjectsD.startFlexMarkers;  // updatePortion = [start, end]

    if (FlexIndex < numFlex1D) {
        uint2 CableNodes = CableElementsNodes[FlexIndex];
        Real3 pos_fsi_fea_D_nA = pos_fsi_fea_D[CableNodes.x];
        Real3 pos_fsi_fea_D_nB = pos_fsi_fea_D[CableNodes.y];
        //    printf("CableElementsNodes[%d]=%d,%d\n", FlexIndex, CableNodes.x, CableNodes.y);

        Real3 x_dir = pos_fsi_fea_D_nB - pos_fsi_fea_D_nA;
        Real L = length(x_dir);
        x_dir = x_dir / length(x_dir);
        Real3 y_dir = mR3(-x_dir.y, x_dir.x, 0) + mR3(-x_dir.z, 0, x_dir.x) + mR3(0, -x_dir.z, x_dir.y);
        y_dir = y_dir / length(y_dir);
        Real3 z_dir = cross(x_dir, y_dir);

        Real2 N_cable = Cables_ShapeFunctions(FlexSPH_MeshPos_LRF_D[index].x);
        //        Real2 N_cable = Cables_ShapeFunctions(length(mR3(posRadD[FlexMarkerIndex]) - pos_fsi_fea_D_nA) / L);

        Real NA = N_cable.x;
        Real NB = N_cable.y;

        Real3 vel_fsi_fea_D_nA = vel_fsi_fea_D[CableNodes.x];
        Real3 vel_fsi_fea_D_nB = vel_fsi_fea_D[CableNodes.y];

        Real3 physic_to_natural = mR3(1 / L, 1, 1);
        Real3 pos_natural = mR3(FlexSPH_MeshPos_LRF_D[index].x * physic_to_natural.x,
                                FlexSPH_MeshPos_LRF_D[index].y * physic_to_natural.y,
                                FlexSPH_MeshPos_LRF_D[index].z * physic_to_natural.z);

        //        printf(" %d pos_natural= %f,%f,%f, length(x_dir)=%f\n", FlexMarkerIndex, pos_natural.x, pos_natural.y,
        //               pos_natural.z, L);

        Real2 Nnew = Cables_ShapeFunctions(FlexSPH_MeshPos_LRF_D[index].x);
        Real h = posRadD[FlexMarkerIndex].w;
        Real3 tempPos = Nnew.x * pos_fsi_fea_D_nA + Nnew.y * pos_fsi_fea_D_nB +
                        FlexSPH_MeshPos_LRF_D[index].y * y_dir * Spacing +
                        FlexSPH_MeshPos_LRF_D[index].z * z_dir * Spacing;

        posRadD[FlexMarkerIndex] = mR4(tempPos, h);
        velMasD[FlexMarkerIndex] = NA * vel_fsi_fea_D_nA + NB * vel_fsi_fea_D_nB;
    }

    if (FlexIndex >= numFlex1D) {
        uint4 shellNodes = ShellelementsNodes[FlexIndex - numFlex1D];
        Real3 pos_fsi_fea_D_nA = pos_fsi_fea_D[shellNodes.x];
        Real3 pos_fsi_fea_D_nB = pos_fsi_fea_D[shellNodes.y];
        Real3 pos_fsi_fea_D_nC = pos_fsi_fea_D[shellNodes.z];
        Real3 pos_fsi_fea_D_nD = pos_fsi_fea_D[shellNodes.w];

        Real3 Shell_center = 0.25 * (pos_fsi_fea_D_nA + pos_fsi_fea_D_nB + pos_fsi_fea_D_nC + pos_fsi_fea_D_nD);

        Real3 dist3 = mR3(posRadD[FlexMarkerIndex]) - Shell_center;
        //        Real3 dist3 = FlexSPH_MeshPos_LRF_D[index] - Shell_center;
        //  printf(" %d dist3= %f,%f,%f center= %f,%f,%f\n", FlexMarkerIndex, dist3.x, dist3.y, dist3.z, Shell_center.x,
        //         Shell_center.y, Shell_center.z);

        Real3 x_dir = ((pos_fsi_fea_D_nB - pos_fsi_fea_D_nA) + (pos_fsi_fea_D_nC - pos_fsi_fea_D_nD));

        //        Real3 y_dir = ((pos_fsi_fea_D_nD - pos_fsi_fea_D_nA) + (pos_fsi_fea_D_nC - pos_fsi_fea_D_nB));
        //
        //        Real3 Normal = normalize(cross(x_dir, y_dir));

        Real3 n1 = normalize(cross(pos_fsi_fea_D_nB - pos_fsi_fea_D_nA, pos_fsi_fea_D_nC - pos_fsi_fea_D_nB));
        Real3 n2 = normalize(cross(pos_fsi_fea_D_nC - pos_fsi_fea_D_nB, pos_fsi_fea_D_nD - pos_fsi_fea_D_nC));
        Real3 n3 = normalize(cross(pos_fsi_fea_D_nD - pos_fsi_fea_D_nC, pos_fsi_fea_D_nA - pos_fsi_fea_D_nD));
        Real3 n4 = normalize(cross(pos_fsi_fea_D_nA - pos_fsi_fea_D_nD, pos_fsi_fea_D_nB - pos_fsi_fea_D_nA));
        Real3 Normal = normalize(n1 + n2 + n3 + n4);
        Real3 y_dir = cross(Normal, x_dir);

        Real Shell_x = 0.25 * length(pos_fsi_fea_D_nB - pos_fsi_fea_D_nA + pos_fsi_fea_D_nC - pos_fsi_fea_D_nD);
        Real Shell_y = 0.25 * length(pos_fsi_fea_D_nD - pos_fsi_fea_D_nA + pos_fsi_fea_D_nC - pos_fsi_fea_D_nB);

        Real3 physic_to_natural = mR3(1 / Shell_x, 1 / Shell_y, 1);
        Real3 pos_physical = dist3;
        Real3 pos_natural = mR3(pos_physical.x * physic_to_natural.x, pos_physical.y * physic_to_natural.y,
                                pos_physical.z * physic_to_natural.z);

        //  printf(" %d Shell (x,y)= %f,%f\n", FlexMarkerIndex, Shell_x, Shell_y);
        //
        //  Real2 FlexSPH_MeshPos_Natural = mR2(dist3.x / length(x_dir) / 4.0, dist3.y / length(y_dir) / 4.0);
        //
        //  printf(" %d FlexSPH_MeshPos_Natural= %f,%f,%f\n", FlexMarkerIndex, FlexSPH_MeshPos_Natural.x,
        //         FlexSPH_MeshPos_Natural.y, FlexSPH_MeshPos_LRF_D[index].z);

        //        Real4 N_shell = Shells_ShapeFunctions(pos_natural.x, pos_natural.y);

        Real4 N_shell = Shells_ShapeFunctions(FlexSPH_MeshPos_LRF_D[index].x, FlexSPH_MeshPos_LRF_D[index].y);

        Real NA = N_shell.x;
        Real NB = N_shell.y;
        Real NC = N_shell.z;
        Real ND = N_shell.w;

        Real3 vel_fsi_fea_D_nA = vel_fsi_fea_D[shellNodes.x];
        Real3 vel_fsi_fea_D_nB = vel_fsi_fea_D[shellNodes.y];
        Real3 vel_fsi_fea_D_nC = vel_fsi_fea_D[shellNodes.z];
        Real3 vel_fsi_fea_D_nD = vel_fsi_fea_D[shellNodes.w];

        Real h = posRadD[FlexMarkerIndex].w;
        Real3 tempPos = NA * pos_fsi_fea_D_nA + NB * pos_fsi_fea_D_nB + NC * pos_fsi_fea_D_nC + ND * pos_fsi_fea_D_nD +
                        Normal * FlexSPH_MeshPos_LRF_D[index].z * Spacing;

        //        if (index < 32)
        //            printf(" FlexMarkerIndex:%d FlexSPH_MeshPos_LRF_D=%f,%f,%f\n", index,
        //            FlexSPH_MeshPos_LRF_D[index].x,
        //                   FlexSPH_MeshPos_LRF_D[index].y, FlexSPH_MeshPos_LRF_D[index].z);

        posRadD[FlexMarkerIndex] = mR4(tempPos, h);

        velMasD[FlexMarkerIndex] =
            NA * vel_fsi_fea_D_nA + NB * vel_fsi_fea_D_nB + NC * vel_fsi_fea_D_nC + ND * vel_fsi_fea_D_nD;

        //        printf(" FlexMarkerIndex:%d center=%f,%f,%f x_dir=%f,%f,%f, y_dir= %f,%f,%f\n", FlexMarkerIndex,
        //        Shell_center.x,
        //               Shell_center.y, Shell_center.z, x_dir.x, x_dir.y, x_dir.z, y_dir.x, y_dir.y, y_dir.z);
    }
}

//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Calc_Rigid_FSI_ForcesD_TorquesD(Real3* rigid_FSI_ForcesD,
                                                Real3* rigid_FSI_TorquesD,
                                                Real4* derivVelRhoD,
                                                Real4* derivVelRhoD_old,

                                                Real4* posRadD,
                                                uint* rigidIdentifierD,
                                                Real3* posRigidD,
                                                Real3* rigidSPH_MeshPos_LRF_D,
                                                Real4* qD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numRigid_SphMarkers) {
        return;
    }
    int RigidIndex = rigidIdentifierD[index];
    uint rigidMarkerIndex = index + numObjectsD.startRigidMarkers;  // updatePortion = [start, end]
    derivVelRhoD[rigidMarkerIndex] = derivVelRhoD[rigidMarkerIndex] * paramsD.Beta 
                                   + derivVelRhoD_old[rigidMarkerIndex] * (1 - paramsD.Beta);

    atomicAdd(&(rigid_FSI_ForcesD[RigidIndex].x), (double)derivVelRhoD[rigidMarkerIndex].x);
    atomicAdd(&(rigid_FSI_ForcesD[RigidIndex].y), (double)derivVelRhoD[rigidMarkerIndex].y);
    atomicAdd(&(rigid_FSI_ForcesD[RigidIndex].z), (double)derivVelRhoD[rigidMarkerIndex].z);

    Real3 dist3 = Distance(mR3(posRadD[rigidMarkerIndex]), posRigidD[RigidIndex]);
    Real3 mtorque = cross(dist3, mR3(derivVelRhoD[rigidMarkerIndex]));

    atomicAdd(&(rigid_FSI_TorquesD[RigidIndex].x), (double)mtorque.x);
    atomicAdd(&(rigid_FSI_TorquesD[RigidIndex].y), (double)mtorque.y);
    atomicAdd(&(rigid_FSI_TorquesD[RigidIndex].z), (double)mtorque.z);
}

//--------------------------------------------------------------------------------------------------------------------------------
ChBce::ChBce(std::shared_ptr<SphMarkerDataD> otherSortedSphMarkersD,
             std::shared_ptr<ProximityDataD> otherMarkersProximityD,
             std::shared_ptr<FsiGeneralData> otherFsiGeneralData,
             std::shared_ptr<SimParams> otherParamsH,
             std::shared_ptr<NumberOfObjects> otherNumObjects)
    : sortedSphMarkersD(otherSortedSphMarkersD),
      markersProximityD(otherMarkersProximityD),
      fsiGeneralData(otherFsiGeneralData),
      paramsH(otherParamsH),
      numObjectsH(otherNumObjects) {
    totalSurfaceInteractionRigid4.resize(0);
    torqueMarkersD.resize(0);
    dummyIdentify.resize(0);
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChBce::Finalize(std::shared_ptr<SphMarkerDataD> sphMarkersD,
                     std::shared_ptr<FsiBodiesDataD> fsiBodiesD,
                     std::shared_ptr<FsiMeshDataD> fsiMeshD) {
    cudaMemcpyToSymbolAsync(paramsD, paramsH.get(), sizeof(SimParams));
    cudaMemcpyToSymbolAsync(numObjectsD, numObjectsH.get(), sizeof(NumberOfObjects));
    CopyParams_NumberOfObjects(paramsH, numObjectsH);
    totalSurfaceInteractionRigid4.resize(numObjectsH->numRigidBodies);
    dummyIdentify.resize(numObjectsH->numRigidBodies);
    torqueMarkersD.resize(numObjectsH->numRigid_SphMarkers);

    // Resizing the arrays used to modify the BCE velocity and pressure according
    // to ADAMI

    int haveGhost = (numObjectsH->numGhostMarkers > 0) ? 1 : 0;
    int haveHelper = (numObjectsH->numHelperMarkers > 0) ? 1 : 0;

    int numFlexAndRigidAndBoundaryMarkers =
        fsiGeneralData
            ->referenceArray[2 + haveHelper + haveGhost + numObjectsH->numRigidBodies + numObjectsH->numFlexBodies1D +
                             numObjectsH->numFlexBodies2D - 1]
            .y -
        fsiGeneralData->referenceArray[haveHelper + haveGhost].y;
    printf("numFlexAndRigidAndBoundaryMarkers= %d, All= %zd\n", numFlexAndRigidAndBoundaryMarkers,
           numObjectsH->numBoundaryMarkers + numObjectsH->numRigid_SphMarkers + numObjectsH->numFlex_SphMarkers);

    if ((numObjectsH->numBoundaryMarkers + numObjectsH->numRigid_SphMarkers + numObjectsH->numFlex_SphMarkers) !=
        numFlexAndRigidAndBoundaryMarkers) {
        throw std::runtime_error("Error! number of flex and rigid and boundary markers are saved incorrectly!\n");
    }
    velMas_ModifiedBCE.resize(numFlexAndRigidAndBoundaryMarkers);
    rhoPreMu_ModifiedBCE.resize(numFlexAndRigidAndBoundaryMarkers);

    // Populate local position of BCE markers
    Populate_RigidSPH_MeshPos_LRF(sphMarkersD, fsiBodiesD);
    Populate_FlexSPH_MeshPos_LRF(sphMarkersD, fsiMeshD);
}
//--------------------------------------------------------------------------------------------------------------------------------
ChBce::~ChBce() {
    // TODO
}

////--------------------------------------------------------------------------------------------------------------------------------
void ChBce::MakeRigidIdentifier() {
    if (numObjectsH->numRigidBodies > 0) {
        int haveGhost = (numObjectsH->numGhostMarkers > 0) ? 1 : 0;
        int haveHelper = (numObjectsH->numHelperMarkers > 0) ? 1 : 0;

        for (size_t rigidSphereA = 0; rigidSphereA < numObjectsH->numRigidBodies; rigidSphereA++) {
            int4 referencePart = fsiGeneralData->referenceArray[haveHelper + haveGhost + 2 + rigidSphereA];
            if (referencePart.z != 1) {
                printf(
                    " Error! in accessing rigid bodies. Reference array indexing is "
                    "wrong\n");
                return;
            }
            int2 updatePortion = mI2(referencePart);  // first two component of the
            thrust::fill(fsiGeneralData->rigidIdentifierD.begin() + (updatePortion.x - numObjectsH->startRigidMarkers),
                         fsiGeneralData->rigidIdentifierD.begin() + (updatePortion.y - numObjectsH->startRigidMarkers),
                         rigidSphereA);
        }
    }
}
////--------------------------------------------------------------------------------------------------------------------------------
void ChBce::MakeFlexIdentifier() {
    if ((numObjectsH->numFlexBodies1D + numObjectsH->numFlexBodies2D) > 0) {
        fsiGeneralData->FlexIdentifierD.resize(numObjectsH->numFlex_SphMarkers);

        for (int CableNum = 0; CableNum < numObjectsH->numFlexBodies1D; CableNum++) {
            int4 referencePart = fsiGeneralData->referenceArray_FEA[CableNum];
            //      printf(" Item Index for this Flex body is %d. ", 2 + numObjectsH->numRigidBodies + CableNum);
            //      printf(" .x=%d, .y=%d, .z=%d, .w=%d", referencePart.x, referencePart.y, referencePart.z,
            //      referencePart.w);

            if (referencePart.z != 2) {
                printf(
                    " Error! in accessing flex bodies. Reference array indexing is "
                    "wrong\n");
                return;
            }
            int2 updatePortion = mI2(referencePart);
            thrust::fill(fsiGeneralData->FlexIdentifierD.begin() + (updatePortion.x - numObjectsH->startFlexMarkers),
                         fsiGeneralData->FlexIdentifierD.begin() + (updatePortion.y - numObjectsH->startFlexMarkers),
                         CableNum);

            printf("From %d to %d FlexIdentifierD=%d\n", updatePortion.x, updatePortion.y, CableNum);
        }

        for (size_t shellNum = 0; shellNum < numObjectsH->numFlexBodies2D; shellNum++) {
            int4 referencePart = fsiGeneralData->referenceArray_FEA[numObjectsH->numFlexBodies1D + shellNum];
            //      printf(" Item Index for this Flex body is %d. ",
            //             2 + numObjectsH->numRigidBodies + numObjectsH->numFlexBodies1D + shellNum);
            //      printf(" .x=%d, .y=%d, .z=%d, .w=%d", referencePart.x, referencePart.y, referencePart.z,
            //      referencePart.w);

            if (referencePart.z != 3) {
                printf(
                    " Error! in accessing flex bodies. Reference array indexing is "
                    "wrong\n");
                return;
            }
            int2 updatePortion = mI2(referencePart);
            thrust::fill(fsiGeneralData->FlexIdentifierD.begin() + (updatePortion.x - numObjectsH->startFlexMarkers),
                         fsiGeneralData->FlexIdentifierD.begin() + (updatePortion.y - numObjectsH->startFlexMarkers),
                         shellNum + numObjectsH->numFlexBodies1D);

            //      printf("From %d to %d FlexIdentifierD=%d\n", updatePortion.x, updatePortion.y,
            //             shellNum + numObjectsH->numFlexBodies1D);
        }
    }
}
////--------------------------------------------------------------------------------------------------------------------------------

void ChBce::Populate_RigidSPH_MeshPos_LRF(std::shared_ptr<SphMarkerDataD> sphMarkersD,
                                          std::shared_ptr<FsiBodiesDataD> fsiBodiesD) {
    if (numObjectsH->numRigidBodies == 0) {
        return;
    }

    MakeRigidIdentifier();

    uint nBlocks_numRigid_SphMarkers;
    uint nThreads_SphMarkers;
    computeGridSize((uint)numObjectsH->numRigid_SphMarkers, 256, nBlocks_numRigid_SphMarkers, nThreads_SphMarkers);

    Populate_RigidSPH_MeshPos_LRF_kernel<<<nBlocks_numRigid_SphMarkers, nThreads_SphMarkers>>>(
        mR3CAST(fsiGeneralData->rigidSPH_MeshPos_LRF_D), mR4CAST(sphMarkersD->posRadD),
        U1CAST(fsiGeneralData->rigidIdentifierD), mR3CAST(fsiBodiesD->posRigid_fsiBodies_D),
        mR4CAST(fsiBodiesD->q_fsiBodies_D));
    cudaDeviceSynchronize();
    cudaCheckError();

    UpdateRigidMarkersPositionVelocity(sphMarkersD, fsiBodiesD);
}
////--------------------------------------------------------------------------------------------------------------------------------

void ChBce::Populate_FlexSPH_MeshPos_LRF(std::shared_ptr<SphMarkerDataD> sphMarkersD,
                                         std::shared_ptr<FsiMeshDataD> fsiMeshD) {
    if ((numObjectsH->numFlexBodies1D + numObjectsH->numFlexBodies2D) == 0) {
        return;
    }

    MakeFlexIdentifier();

    uint nBlocks_numFlex_SphMarkers;
    uint nThreads_SphMarkers;
    computeGridSize((uint)numObjectsH->numFlex_SphMarkers, 256, nBlocks_numFlex_SphMarkers, nThreads_SphMarkers);

    //  printf(
    //      "size of FlexSPH_MeshPos_LRF_D = %d and posRadD=%d, in "
    //      "ChBce::Populate_FlexSPH_MeshPos_LRF\n",
    //      fsiGeneralData->FlexSPH_MeshPos_LRF_D.size(), sphMarkersD->posRadD.size());
    //
    //  printf(
    //      "size of FlexIdentifierD = %d, numObjectsH->numFlexBodies1D =%d, numObjectsH->numFlexBodies2D "
    //      "=%d,in ChBce::Populate_FlexSPH_MeshPos_LRF\n",
    //      fsiGeneralData->FlexIdentifierD.size(), numObjectsH->numFlexBodies1D, numObjectsH->numFlexBodies2D);
    //
    //  printf(
    //      "size of CableElementsNodes = %d and ShellElementsNodes=%d, fsiMeshD->pos_fsi_fea_D =%d, in "
    //      "ChBce::Populate_FlexSPH_MeshPos_LRF\n",
    //      fsiGeneralData->CableElementsNodes.size(), fsiGeneralData->ShellElementsNodes.size(),
    //      fsiMeshD->pos_fsi_fea_D.size());

    thrust::device_vector<Real3> FlexSPH_MeshPos_LRF_H = fsiGeneralData->FlexSPH_MeshPos_LRF_H;
    Populate_FlexSPH_MeshPos_LRF_kernel<<<nBlocks_numFlex_SphMarkers, nThreads_SphMarkers>>>(
        mR3CAST(fsiGeneralData->FlexSPH_MeshPos_LRF_D), mR3CAST(FlexSPH_MeshPos_LRF_H), mR4CAST(sphMarkersD->posRadD),
        U1CAST(fsiGeneralData->FlexIdentifierD), (int)numObjectsH->numFlexBodies1D,
        U2CAST(fsiGeneralData->CableElementsNodes), U4CAST(fsiGeneralData->ShellElementsNodes),
        mR3CAST(fsiMeshD->pos_fsi_fea_D), paramsH->HSML * paramsH->MULT_INITSPACE_Shells);

    cudaDeviceSynchronize();
    cudaCheckError();

    UpdateFlexMarkersPositionVelocity(sphMarkersD, fsiMeshD);
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChBce::RecalcSortedVelocityPressure_BCE(std::shared_ptr<FsiBodiesDataD> fsiBodiesD,
                                             thrust::device_vector<Real3>& velMas_ModifiedBCE,
                                             thrust::device_vector<Real4>& rhoPreMu_ModifiedBCE,
                                             const thrust::device_vector<Real4>& sortedPosRad,
                                             const thrust::device_vector<Real3>& sortedVelMas,
                                             const thrust::device_vector<Real4>& sortedRhoPreMu,
                                             const thrust::device_vector<uint>& cellStart,
                                             const thrust::device_vector<uint>& cellEnd,
                                             const thrust::device_vector<uint>& mapOriginalToSorted,
                                             const thrust::device_vector<Real3>& bceAcc,
                                             int3 updatePortion) {
    bool *isErrorH, *isErrorD;
    isErrorH = (bool*)malloc(sizeof(bool));
    cudaMalloc((void**)&isErrorD, sizeof(bool));
    *isErrorH = false;
    cudaMemcpy(isErrorD, isErrorH, sizeof(bool), cudaMemcpyHostToDevice);
    //------------------------------------------------------------------------

    // thread per particle
    uint numThreads, numBlocks;
    computeGridSize(updatePortion.z - updatePortion.x, 64, numBlocks, numThreads);

    //    printf("rigid size %d %d %d %d\n", fsiGeneralData->rigidIdentifierD.size(),
    //           fsiBodiesD->velMassRigid_fsiBodies_D.size(), updatePortion.y, updatePortion.x);

    new_BCE_VelocityPressure<<<numBlocks, numThreads>>>(
        mR4CAST(fsiBodiesD->velMassRigid_fsiBodies_D), U1CAST(fsiGeneralData->rigidIdentifierD),
        mR3CAST(velMas_ModifiedBCE),
        mR4CAST(rhoPreMu_ModifiedBCE),  // input: sorted velocities
        mR4CAST(sortedPosRad), mR3CAST(sortedVelMas), mR4CAST(sortedRhoPreMu), U1CAST(cellStart), U1CAST(cellEnd),
        U1CAST(mapOriginalToSorted), mR3CAST(bceAcc), updatePortion, isErrorD);

    cudaDeviceSynchronize();
    cudaCheckError()

        //------------------------------------------------------------------------
        cudaMemcpy(isErrorH, isErrorD, sizeof(bool), cudaMemcpyDeviceToHost);
    if (*isErrorH == true) {
        throw std::runtime_error("Error! program crashed in  new_BCE_VelocityPressure!\n");
    }
    cudaFree(isErrorD);
    free(isErrorH);
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChBce::CalcBceAcceleration(thrust::device_vector<Real3>& bceAcc,
                                const thrust::device_vector<Real4>& q_fsiBodies_D,
                                const thrust::device_vector<Real3>& accRigid_fsiBodies_D,
                                const thrust::device_vector<Real3>& omegaVelLRF_fsiBodies_D,
                                const thrust::device_vector<Real3>& omegaAccLRF_fsiBodies_D,
                                const thrust::device_vector<Real3>& rigidSPH_MeshPos_LRF_D,
                                const thrust::device_vector<uint>& rigidIdentifierD,
                                int numRigid_SphMarkers) {
    // thread per particle
    uint numThreads, numBlocks;
    computeGridSize(numRigid_SphMarkers, 64, numBlocks, numThreads);

    calcBceAcceleration_kernel<<<numBlocks, numThreads>>>(
        mR3CAST(bceAcc), mR4CAST(q_fsiBodies_D), mR3CAST(accRigid_fsiBodies_D), mR3CAST(omegaVelLRF_fsiBodies_D),
        mR3CAST(omegaAccLRF_fsiBodies_D), mR3CAST(rigidSPH_MeshPos_LRF_D), U1CAST(rigidIdentifierD));

    cudaDeviceSynchronize();
    cudaCheckError();
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChBce::ModifyBceVelocity(std::shared_ptr<SphMarkerDataD> sphMarkersD, std::shared_ptr<FsiBodiesDataD> fsiBodiesD) {
    // modify BCE velocity and pressure
    int numRigidAndBoundaryMarkers =
        fsiGeneralData->referenceArray[2 + numObjectsH->numRigidBodies - 1].y - fsiGeneralData->referenceArray[0].y;
    if ((numObjectsH->numBoundaryMarkers + numObjectsH->numRigid_SphMarkers) != numRigidAndBoundaryMarkers) {
        throw std::runtime_error(
            "Error! number of rigid and boundary markers are "
            "saved incorrectly. Thrown from "
            "ModifyBceVelocity!\n");
    }
    if (!(velMas_ModifiedBCE.size() == numRigidAndBoundaryMarkers &&
          rhoPreMu_ModifiedBCE.size() == numRigidAndBoundaryMarkers)) {
        throw std::runtime_error(
            "Error! size error velMas_ModifiedBCE and "
            "rhoPreMu_ModifiedBCE. Thrown from "
            "ModifyBceVelocity!\n");
    }
    int3 updatePortion = mI3(fsiGeneralData->referenceArray[0].y, fsiGeneralData->referenceArray[1].y,
                             fsiGeneralData->referenceArray[2 + numObjectsH->numRigidBodies - 1].y);
    if (paramsH->bceType == ADAMI) {
        thrust::device_vector<Real3> bceAcc(numObjectsH->numRigid_SphMarkers);
        if (numObjectsH->numRigid_SphMarkers > 0) {
            CalcBceAcceleration(bceAcc, fsiBodiesD->q_fsiBodies_D, fsiBodiesD->accRigid_fsiBodies_D,
                                fsiBodiesD->omegaVelLRF_fsiBodies_D, fsiBodiesD->omegaAccLRF_fsiBodies_D,
                                fsiGeneralData->rigidSPH_MeshPos_LRF_D, fsiGeneralData->rigidIdentifierD,
                                (int)numObjectsH->numRigid_SphMarkers);
        }
        RecalcSortedVelocityPressure_BCE(
            fsiBodiesD, velMas_ModifiedBCE, rhoPreMu_ModifiedBCE, sortedSphMarkersD->posRadD,
            sortedSphMarkersD->velMasD, sortedSphMarkersD->rhoPresMuD, markersProximityD->cellStartD,
            markersProximityD->cellEndD, markersProximityD->mapOriginalToSorted, bceAcc, updatePortion);
        bceAcc.clear();
    } else {
        thrust::copy(sphMarkersD->velMasD.begin() + updatePortion.x, sphMarkersD->velMasD.begin() + updatePortion.y,
                     velMas_ModifiedBCE.begin());
        thrust::copy(sphMarkersD->rhoPresMuD.begin() + updatePortion.x,
                     sphMarkersD->rhoPresMuD.begin() + updatePortion.y, rhoPreMu_ModifiedBCE.begin());
    }
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChBce::Rigid_Forces_Torques(std::shared_ptr<SphMarkerDataD> sphMarkersD,
                                 std::shared_ptr<FsiBodiesDataD> fsiBodiesD) {
    if (numObjectsH->numRigidBodies == 0) {
        return;
    }

    thrust::fill(fsiGeneralData->rigid_FSI_ForcesD.begin(), fsiGeneralData->rigid_FSI_ForcesD.end(), mR3(0));
    thrust::fill(fsiGeneralData->rigid_FSI_TorquesD.begin(), fsiGeneralData->rigid_FSI_TorquesD.end(), mR3(0));

    uint nBlocks_numRigid_SphMarkers;
    uint nThreads_SphMarkers;
    computeGridSize((uint)numObjectsH->numRigid_SphMarkers, 256, nBlocks_numRigid_SphMarkers, nThreads_SphMarkers);
    Calc_Rigid_FSI_ForcesD_TorquesD<<<nBlocks_numRigid_SphMarkers, nThreads_SphMarkers>>>(
        mR3CAST(fsiGeneralData->rigid_FSI_ForcesD), mR3CAST(fsiGeneralData->rigid_FSI_TorquesD),
        mR4CAST(fsiGeneralData->derivVelRhoD), mR4CAST(fsiGeneralData->derivVelRhoD_old), mR4CAST(sphMarkersD->posRadD),
        U1CAST(fsiGeneralData->rigidIdentifierD), mR3CAST(fsiBodiesD->posRigid_fsiBodies_D),
        mR3CAST(fsiGeneralData->rigidSPH_MeshPos_LRF_D), mR4CAST(fsiBodiesD->q_fsiBodies_D));
    cudaDeviceSynchronize();
    cudaCheckError();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////
void ChBce::Flex_Forces(std::shared_ptr<SphMarkerDataD> sphMarkersD, std::shared_ptr<FsiMeshDataD> fsiMeshD) {
    if ((numObjectsH->numFlexBodies1D + numObjectsH->numFlexBodies2D) == 0) {
        return;
    }

    thrust::fill(fsiGeneralData->Flex_FSI_ForcesD.begin(), fsiGeneralData->Flex_FSI_ForcesD.end(), mR3(0));

    uint nBlocks_numFlex_SphMarkers;
    uint nThreads_SphMarkers;
    computeGridSize((int)numObjectsH->numFlex_SphMarkers, 256, nBlocks_numFlex_SphMarkers, nThreads_SphMarkers);

    Calc_Flex_FSI_ForcesD<<<nBlocks_numFlex_SphMarkers, nThreads_SphMarkers>>>(
        mR3CAST(fsiGeneralData->FlexSPH_MeshPos_LRF_D), U1CAST(fsiGeneralData->FlexIdentifierD),
        (int)numObjectsH->numFlexBodies1D, U2CAST(fsiGeneralData->CableElementsNodes),
        U4CAST(fsiGeneralData->ShellElementsNodes), mR4CAST(fsiGeneralData->derivVelRhoD),
        mR4CAST(fsiGeneralData->derivVelRhoD_old), mR3CAST(fsiMeshD->pos_fsi_fea_D),
        mR3CAST(fsiGeneralData->Flex_FSI_ForcesD));
    cudaDeviceSynchronize();
    cudaCheckError();
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChBce::UpdateRigidMarkersPositionVelocity(std::shared_ptr<SphMarkerDataD> sphMarkersD,
                                               std::shared_ptr<FsiBodiesDataD> fsiBodiesD) {
    if (numObjectsH->numRigidBodies == 0) {
        return;
    }
    uint nBlocks_numRigid_SphMarkers;
    uint nThreads_SphMarkers;
    computeGridSize((int)numObjectsH->numRigid_SphMarkers, 256, nBlocks_numRigid_SphMarkers, nThreads_SphMarkers);
    UpdateRigidMarkersPositionVelocityD<<<nBlocks_numRigid_SphMarkers, nThreads_SphMarkers>>>(
        mR4CAST(sphMarkersD->posRadD), mR3CAST(sphMarkersD->velMasD), mR3CAST(fsiGeneralData->rigidSPH_MeshPos_LRF_D),
        U1CAST(fsiGeneralData->rigidIdentifierD), mR3CAST(fsiBodiesD->posRigid_fsiBodies_D),
        mR4CAST(fsiBodiesD->velMassRigid_fsiBodies_D), mR3CAST(fsiBodiesD->omegaVelLRF_fsiBodies_D),
        mR4CAST(fsiBodiesD->q_fsiBodies_D));
    cudaDeviceSynchronize();
    cudaCheckError();
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChBce::UpdateFlexMarkersPositionVelocity(std::shared_ptr<SphMarkerDataD> sphMarkersD,
                                              std::shared_ptr<FsiMeshDataD> fsiMeshD) {
    if ((numObjectsH->numFlexBodies1D + numObjectsH->numFlexBodies2D) == 0) {
        return;
    }

    uint nBlocks_numFlex_SphMarkers;
    uint nThreads_SphMarkers;
    printf("UpdateFlexMarkersPositionVelocity..\n");

    computeGridSize((int)numObjectsH->numFlex_SphMarkers, 256, nBlocks_numFlex_SphMarkers, nThreads_SphMarkers);
    UpdateFlexMarkersPositionVelocityAccD<<<nBlocks_numFlex_SphMarkers, nThreads_SphMarkers>>>(
        mR4CAST(sphMarkersD->posRadD), mR3CAST(fsiGeneralData->FlexSPH_MeshPos_LRF_D), mR3CAST(sphMarkersD->velMasD),
        U1CAST(fsiGeneralData->FlexIdentifierD), (int)numObjectsH->numFlexBodies1D,
        U2CAST(fsiGeneralData->CableElementsNodes), U4CAST(fsiGeneralData->ShellElementsNodes),
        mR3CAST(fsiMeshD->pos_fsi_fea_D), mR3CAST(fsiMeshD->vel_fsi_fea_D),
        paramsH->HSML * paramsH->MULT_INITSPACE_Shells);
    cudaDeviceSynchronize();
    cudaCheckError();
}

}  // end namespace fsi
}  // end namespace chrono
