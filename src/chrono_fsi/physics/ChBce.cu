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
// Author: Arman Pazouki, Milad Rakhsha, Wei Hu
// =============================================================================
//
// Base class for processing boundary condition enforcing (bce) markers forces
// in FSI system.
// =============================================================================

#include "chrono_fsi/physics/ChBce.cuh"
#include "chrono_fsi/physics/ChSphGeneral.cuh"
#include <type_traits>

namespace chrono {
namespace fsi {

//--------------------------------------------------------------------------------------------------------------------------------
__device__ double atomicAdd_double(double* address, double val) {
    unsigned long long int* address_as_ull = (unsigned long long int*)address;
    unsigned long long int old = *address_as_ull, assumed;

    do {
        assumed = old;
        old = atomicCAS(address_as_ull, assumed, __double_as_longlong(val + __longlong_as_double(assumed)));
    } while (assumed != old);

    return __longlong_as_double(old);
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Populate_RigidSPH_MeshPos_LRF_D(Real3* rigidSPH_MeshPos_LRF_D,
                                                Real4* posRadD,
                                                uint* rigidIdentifierD,
                                                Real3* posRigidD,
                                                Real4* qD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numRigidMarkers)
        return;

    int rigidIndex = rigidIdentifierD[index];
    uint rigidMarkerIndex = index + numObjectsD.startRigidMarkers;
    Real4 q4 = qD[rigidIndex];
    Real3 a1, a2, a3;
    RotationMatirixFromQuaternion(a1, a2, a3, q4);
    Real3 dist3 = mR3(posRadD[rigidMarkerIndex]) - posRigidD[rigidIndex];
    Real3 dist3LF = InverseRotate_By_RotationMatrix_DeviceHost(a1, a2, a3, dist3);
    // Save the coordinates in the local reference of a rigid body
    rigidSPH_MeshPos_LRF_D[index] = dist3LF;
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Populate_FlexSPH_MeshPos_LRF_D(Real3* FlexSPH_MeshPos_LRF_D,
                                               Real3* FlexSPH_MeshPos_LRF_H,
                                               Real4* posRadD,
                                               uint* FlexIdentifierD,
                                               const int numFlex1D,
                                               uint2* CableElementsNodes,
                                               uint4* ShellElementsNodes,
                                               Real3* pos_fsi_fea_D,
                                               Real Spacing) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numFlexMarkers)
        return;

    // The coordinates of BCE in local reference frame is already calculated when created,
    // So only need to copy from host to device here
    FlexSPH_MeshPos_LRF_D[index] = FlexSPH_MeshPos_LRF_H[index];

    // No need to do it again. Keep this code in case of any issues later
    /*int FlexIndex = FlexIdentifierD[index];
    uint FlexMarkerIndex = index + numObjectsD.startFlexMarkers;

    if (FlexIndex < numFlex1D) {
        uint2 cableNodes = CableElementsNodes[FlexIndex];
        Real3 pos_fsi_fea_D_nA = pos_fsi_fea_D[cableNodes.x];
        Real3 pos_fsi_fea_D_nB = pos_fsi_fea_D[cableNodes.y];
        Real3 dist3 = mR3(posRadD[FlexMarkerIndex]) - pos_fsi_fea_D_nA;
        Real3 x_dir = pos_fsi_fea_D_nB - pos_fsi_fea_D_nA;
        Real Cable_x = length(x_dir);
        x_dir = x_dir / length(x_dir);
        Real norm_dir_length = length(cross(dist3, x_dir));

        Real3 y_dir = mR3(-x_dir.y, x_dir.x, 0) + mR3(-x_dir.z, 0, x_dir.x) + mR3(0, -x_dir.z, x_dir.y);
        y_dir = y_dir / length(y_dir);
        Real3 z_dir = cross(x_dir, y_dir);
        Real dx = dot(dist3, x_dir);
        Real dy = dot(dist3, y_dir);
        Real dz = dot(dist3, z_dir);
        if (abs(dy) > 0)
            dy /= Spacing;
        if (abs(dz) > 0)
            dz /= Spacing;

        FlexSPH_MeshPos_LRF_D[index] = mR3(dx / Cable_x, dy, dz);
    }
    if (FlexIndex >= numFlex1D) {
        uint4 shellNodes = ShellElementsNodes[FlexIndex - numFlex1D];
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

        Real3 n1 = normalize(cross(pos_fsi_fea_D_nB - pos_fsi_fea_D_nA, pos_fsi_fea_D_nC - pos_fsi_fea_D_nB));
        Real3 n2 = normalize(cross(pos_fsi_fea_D_nC - pos_fsi_fea_D_nB, pos_fsi_fea_D_nD - pos_fsi_fea_D_nC));
        Real3 n3 = normalize(cross(pos_fsi_fea_D_nD - pos_fsi_fea_D_nC, pos_fsi_fea_D_nA - pos_fsi_fea_D_nD));
        Real3 n4 = normalize(cross(pos_fsi_fea_D_nA - pos_fsi_fea_D_nD, pos_fsi_fea_D_nB - pos_fsi_fea_D_nA));
        Real3 Normal = normalize(n1 + n2 + n3 + n4);
        Real zSide = dot(Normal, dist3) / Spacing;

        FlexSPH_MeshPos_LRF_D[index] = FlexSPH_MeshPos_LRF_H[index];
    }*/
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Calc_Rigid_FSI_Forces_Torques_D(Real3* rigid_FSI_ForcesD,
                                                Real3* rigid_FSI_TorquesD,
                                                Real4* derivVelRhoD,
                                                Real4* derivVelRhoD_old,
                                                Real4* posRadD,
                                                uint* rigidIdentifierD,
                                                Real3* posRigidD,
                                                Real3* rigidSPH_MeshPos_LRF_D,
                                                Real4* qD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numRigidMarkers)
        return;

    int RigidIndex = rigidIdentifierD[index];
    uint rigidMarkerIndex = index + numObjectsD.startRigidMarkers;
    derivVelRhoD[rigidMarkerIndex] =
        (derivVelRhoD[rigidMarkerIndex] * paramsD.Beta + derivVelRhoD_old[rigidMarkerIndex] * (1 - paramsD.Beta)) *
        paramsD.markerMass;

    if (std::is_same<Real, double>::value) {
        atomicAdd_double((double*)&(rigid_FSI_ForcesD[RigidIndex].x), derivVelRhoD[rigidMarkerIndex].x);
        atomicAdd_double((double*)&(rigid_FSI_ForcesD[RigidIndex].y), derivVelRhoD[rigidMarkerIndex].y);
        atomicAdd_double((double*)&(rigid_FSI_ForcesD[RigidIndex].z), derivVelRhoD[rigidMarkerIndex].z);
    } else {
        atomicAdd((float*)&(rigid_FSI_ForcesD[RigidIndex].x), derivVelRhoD[rigidMarkerIndex].x);
        atomicAdd((float*)&(rigid_FSI_ForcesD[RigidIndex].y), derivVelRhoD[rigidMarkerIndex].y);
        atomicAdd((float*)&(rigid_FSI_ForcesD[RigidIndex].z), derivVelRhoD[rigidMarkerIndex].z);
    }
    Real3 dist3 = Distance(mR3(posRadD[rigidMarkerIndex]), posRigidD[RigidIndex]);
    Real3 mtorque = cross(dist3, mR3(derivVelRhoD[rigidMarkerIndex]));

    if (std::is_same<Real, double>::value) {
        atomicAdd_double((double*)&(rigid_FSI_TorquesD[RigidIndex].x), mtorque.x);
        atomicAdd_double((double*)&(rigid_FSI_TorquesD[RigidIndex].y), mtorque.y);
        atomicAdd_double((double*)&(rigid_FSI_TorquesD[RigidIndex].z), mtorque.z);
    } else {
        atomicAdd((float*)&(rigid_FSI_TorquesD[RigidIndex].x), mtorque.x);
        atomicAdd((float*)&(rigid_FSI_TorquesD[RigidIndex].y), mtorque.y);
        atomicAdd((float*)&(rigid_FSI_TorquesD[RigidIndex].z), mtorque.z);
    }
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Calc_Flex_FSI_ForcesD(Real3* FlexSPH_MeshPos_LRF_D,
                                      uint* FlexIdentifierD,
                                      const int numFlex1D,
                                      uint2* CableElementsNodes,
                                      uint4* ShellElementsNodes,
                                      Real4* derivVelRhoD,
                                      Real4* derivVelRhoD_old,
                                      Real3* pos_fsi_fea_D,
                                      Real3* Flex_FSI_ForcesD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numFlexMarkers)
        return;

    int FlexIndex = FlexIdentifierD[index];
    uint FlexMarkerIndex = index + numObjectsD.startFlexMarkers;
    derivVelRhoD[FlexMarkerIndex] =
        (derivVelRhoD[FlexMarkerIndex] * paramsD.Beta + derivVelRhoD_old[FlexMarkerIndex] * (1 - paramsD.Beta)) *
        paramsD.markerMass;

    if (FlexIndex < numFlex1D) {
        Real2 N_cable = Cables_ShapeFunctions(FlexSPH_MeshPos_LRF_D[index].x);
        Real NA = N_cable.x;
        Real NB = N_cable.y;

        int nA = CableElementsNodes[FlexIndex].x;
        int nB = CableElementsNodes[FlexIndex].y;

        if (std::is_same<Real, double>::value) {
            atomicAdd_double((double*)&(Flex_FSI_ForcesD[nA].x), NA * derivVelRhoD[FlexMarkerIndex].x);
            atomicAdd_double((double*)&(Flex_FSI_ForcesD[nA].y), NA * derivVelRhoD[FlexMarkerIndex].y);
            atomicAdd_double((double*)&(Flex_FSI_ForcesD[nA].z), NA * derivVelRhoD[FlexMarkerIndex].z);

            atomicAdd_double((double*)&(Flex_FSI_ForcesD[nB].x), NB * derivVelRhoD[FlexMarkerIndex].x);
            atomicAdd_double((double*)&(Flex_FSI_ForcesD[nB].y), NB * derivVelRhoD[FlexMarkerIndex].y);
            atomicAdd_double((double*)&(Flex_FSI_ForcesD[nB].z), NB * derivVelRhoD[FlexMarkerIndex].z);
        } else {
            atomicAdd((float*)&(Flex_FSI_ForcesD[nA].x), NA * derivVelRhoD[FlexMarkerIndex].x);
            atomicAdd((float*)&(Flex_FSI_ForcesD[nA].y), NA * derivVelRhoD[FlexMarkerIndex].y);
            atomicAdd((float*)&(Flex_FSI_ForcesD[nA].z), NA * derivVelRhoD[FlexMarkerIndex].z);

            atomicAdd((float*)&(Flex_FSI_ForcesD[nB].x), NB * derivVelRhoD[FlexMarkerIndex].x);
            atomicAdd((float*)&(Flex_FSI_ForcesD[nB].y), NB * derivVelRhoD[FlexMarkerIndex].y);
            atomicAdd((float*)&(Flex_FSI_ForcesD[nB].z), NB * derivVelRhoD[FlexMarkerIndex].z);
        }
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

        if (std::is_same<Real, double>::value) {
            atomicAdd_double((double*)&(Flex_FSI_ForcesD[nA].x), NA * derivVelRhoD[FlexMarkerIndex].x);
            atomicAdd_double((double*)&(Flex_FSI_ForcesD[nA].y), NA * derivVelRhoD[FlexMarkerIndex].y);
            atomicAdd_double((double*)&(Flex_FSI_ForcesD[nA].z), NA * derivVelRhoD[FlexMarkerIndex].z);

            atomicAdd_double((double*)&(Flex_FSI_ForcesD[nB].x), NB * derivVelRhoD[FlexMarkerIndex].x);
            atomicAdd_double((double*)&(Flex_FSI_ForcesD[nB].y), NB * derivVelRhoD[FlexMarkerIndex].y);
            atomicAdd_double((double*)&(Flex_FSI_ForcesD[nB].z), NB * derivVelRhoD[FlexMarkerIndex].z);

            atomicAdd_double((double*)&(Flex_FSI_ForcesD[nC].x), NC * derivVelRhoD[FlexMarkerIndex].x);
            atomicAdd_double((double*)&(Flex_FSI_ForcesD[nC].y), NC * derivVelRhoD[FlexMarkerIndex].y);
            atomicAdd_double((double*)&(Flex_FSI_ForcesD[nC].z), NC * derivVelRhoD[FlexMarkerIndex].z);

            atomicAdd_double((double*)&(Flex_FSI_ForcesD[nD].x), ND * derivVelRhoD[FlexMarkerIndex].x);
            atomicAdd_double((double*)&(Flex_FSI_ForcesD[nD].y), ND * derivVelRhoD[FlexMarkerIndex].y);
            atomicAdd_double((double*)&(Flex_FSI_ForcesD[nD].z), ND * derivVelRhoD[FlexMarkerIndex].z);
        } else {
            atomicAdd((float*)&(Flex_FSI_ForcesD[nA].x), NA * derivVelRhoD[FlexMarkerIndex].x);
            atomicAdd((float*)&(Flex_FSI_ForcesD[nA].y), NA * derivVelRhoD[FlexMarkerIndex].y);
            atomicAdd((float*)&(Flex_FSI_ForcesD[nA].z), NA * derivVelRhoD[FlexMarkerIndex].z);

            atomicAdd((float*)&(Flex_FSI_ForcesD[nB].x), NB * derivVelRhoD[FlexMarkerIndex].x);
            atomicAdd((float*)&(Flex_FSI_ForcesD[nB].y), NB * derivVelRhoD[FlexMarkerIndex].y);
            atomicAdd((float*)&(Flex_FSI_ForcesD[nB].z), NB * derivVelRhoD[FlexMarkerIndex].z);

            atomicAdd((float*)&(Flex_FSI_ForcesD[nC].x), NC * derivVelRhoD[FlexMarkerIndex].x);
            atomicAdd((float*)&(Flex_FSI_ForcesD[nC].y), NC * derivVelRhoD[FlexMarkerIndex].y);
            atomicAdd((float*)&(Flex_FSI_ForcesD[nC].z), NC * derivVelRhoD[FlexMarkerIndex].z);

            atomicAdd((float*)&(Flex_FSI_ForcesD[nD].x), ND * derivVelRhoD[FlexMarkerIndex].x);
            atomicAdd((float*)&(Flex_FSI_ForcesD[nD].y), ND * derivVelRhoD[FlexMarkerIndex].y);
            atomicAdd((float*)&(Flex_FSI_ForcesD[nD].z), ND * derivVelRhoD[FlexMarkerIndex].z);
        }
    }
}
//--------------------------------------------------------------------------------------------------------------------------------
__device__ void BCE_modification_Share(Real3& sumVW,
                                       Real3& sumRhoRW,
                                       Real& sumPW,
                                       Real3& sumTauXxYyZzW,
                                       Real3& sumTauXyXzYzW,
                                       Real& sumWFluid,
                                       int& isAffectedV,
                                       int& isAffectedP,
                                       int3 gridPos,
                                       Real3 posRadA,
                                       Real4* sortedPosRad,
                                       Real3* sortedVelMas,
                                       Real4* sortedRhoPreMu,
                                       Real3* sortedTauXxYyZz,
                                       Real3* sortedTauXyXzYz,
                                       uint* cellStart,
                                       uint* cellEnd) {
    uint gridHash = calcGridHash(gridPos);
    // get start of bucket for this cell
    uint startIndex = cellStart[gridHash];
    uint endIndex = cellEnd[gridHash];

    for (uint j = startIndex; j < endIndex; j++) {
        Real3 posRadB = mR3(sortedPosRad[j]);
        Real3 dist3 = Distance(posRadA, posRadB);
        Real dd = dist3.x * dist3.x + dist3.y * dist3.y + dist3.z * dist3.z;
        Real4 rhoPresMuB = sortedRhoPreMu[j];
        Real kernel_radius = RESOLUTION_LENGTH_MULT * paramsD.HSML;
        if (dd > kernel_radius * kernel_radius || rhoPresMuB.w > -0.5)
            continue;
        Real d = length(dist3);
        Real Wd = W3h(d, sortedPosRad[j].w);
        Real3 velMasB = sortedVelMas[j];
        sumVW += velMasB * Wd;
        sumRhoRW += rhoPresMuB.x * dist3 * Wd;
        sumPW += rhoPresMuB.y * Wd;
        sumWFluid += Wd;
        sumTauXxYyZzW += sortedTauXxYyZz[j] * Wd;
        sumTauXyXzYzW += sortedTauXyXzYz[j] * Wd;
    }
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void BCE_VelocityPressureStress(Real3* velMas_ModifiedBCE,
                                           Real4* rhoPreMu_ModifiedBCE,
                                           Real3* tauXxYyZz_ModifiedBCE,
                                           Real3* tauXyXzYz_ModifiedBCE,
                                           Real4* sortedPosRad,
                                           Real3* sortedVelMas,
                                           Real4* sortedRhoPreMu,
                                           Real3* sortedTauXxYyZz,
                                           Real3* sortedTauXyXzYz,
                                           uint* cellStart,
                                           uint* cellEnd,
                                           uint* mapOriginalToSorted,
                                           Real3* bceAcc,
                                           int2 newPortion,
                                           volatile bool* isErrorD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    uint sphIndex = index + newPortion.x;
    if (index >= newPortion.y - newPortion.x)
        return;

    uint bceIndex = index;

    if (paramsD.bceTypeWall == BceVersion::ORIGINAL)
        bceIndex = index + numObjectsD.numBoundaryMarkers;

    uint idA = mapOriginalToSorted[sphIndex];

    Real4 rhoPreMuA = sortedRhoPreMu[idA];
    Real3 posRadA = mR3(sortedPosRad[idA]);
    Real3 velMasA = sortedVelMas[idA];
    int isAffectedV = 0;
    int isAffectedP = 0;

    Real3 sumVW = mR3(0);
    Real3 sumRhoRW = mR3(0);
    Real sumPW = 0;
    Real sumWFluid = 0;
    Real3 sumTauXxYyZzW = mR3(0);
    Real3 sumTauXyXzYzW = mR3(0);

    // get address in grid
    int3 gridPos = calcGridPos(posRadA);

    // examine neighbouring cells
    for (int z = -1; z <= 1; z++) {
        for (int y = -1; y <= 1; y++) {
            for (int x = -1; x <= 1; x++) {
                int3 neighbourPos = gridPos + mI3(x, y, z);
                BCE_modification_Share(sumVW, sumRhoRW, sumPW, sumTauXxYyZzW, sumTauXyXzYzW, sumWFluid, isAffectedV,
                                       isAffectedP, neighbourPos, posRadA, sortedPosRad, sortedVelMas, sortedRhoPreMu,
                                       sortedTauXxYyZz, sortedTauXyXzYz, cellStart, cellEnd);
            }
        }
    }

    if (abs(sumWFluid) > EPSILON) {
        // modify velocity
        Real3 modifiedBCE_v = 2 * velMasA - sumVW / sumWFluid;
        velMas_ModifiedBCE[bceIndex] = modifiedBCE_v;
        // modify pressure and stress
        Real3 aW = mR3(0.0);
        if (rhoPreMuA.w > 0.5 && rhoPreMuA.w < 1.5) {
            // Get acceleration of rigid body's BCE particle
            int rigidBceIndex = sphIndex - numObjectsD.startRigidMarkers;
            if (rigidBceIndex < 0 || rigidBceIndex >= numObjectsD.numRigidMarkers) {
                printf(
                    "Error! particle index out of bound: thrown from "
                    "ChBce.cu, new_BCE_VelocityPressure !\n");
                *isErrorD = true;
                return;
            }
            aW = bceAcc[rigidBceIndex];
        }
        if (rhoPreMuA.w > 1.5 && rhoPreMuA.w < 3.5) {
            // Get acceleration of flexible body's BCE particle
            int flexBceIndex = sphIndex - numObjectsD.startFlexMarkers;
            if (flexBceIndex < 0 || flexBceIndex >= numObjectsD.numFlexMarkers) {
                printf(
                    "Error! particle index out of bound: thrown from "
                    "ChBce.cu, new_BCE_VelocityPressure !\n");
                *isErrorD = true;
                return;
            }
            aW = bceAcc[flexBceIndex + numObjectsD.numRigidMarkers];
        }
        Real pressure = (sumPW + dot(paramsD.gravity - aW, sumRhoRW)) / sumWFluid;
        Real density = InvEos(pressure);
        rhoPreMu_ModifiedBCE[bceIndex] = mR4(density, pressure, rhoPreMuA.z, rhoPreMuA.w);
        if (paramsD.elastic_SPH) {
            Real3 tauXxYyZz = (sumTauXxYyZzW + dot(paramsD.gravity - aW, sumRhoRW)) / sumWFluid;
            Real3 tauXyXzYz = sumTauXyXzYzW / sumWFluid;
            tauXxYyZz_ModifiedBCE[bceIndex] = mR3(tauXxYyZz.x, tauXxYyZz.y, tauXxYyZz.z);
            tauXyXzYz_ModifiedBCE[bceIndex] = mR3(tauXyXzYz.x, tauXyXzYz.y, tauXyXzYz.z);
        }
    } else {
        rhoPreMu_ModifiedBCE[bceIndex] = mR4(paramsD.rho0, paramsD.BASEPRES, paramsD.mu0, rhoPreMuA.w);
        velMas_ModifiedBCE[bceIndex] = mR3(0.0);
        if (paramsD.elastic_SPH) {
            tauXxYyZz_ModifiedBCE[bceIndex] = mR3(0.0);
            tauXyXzYz_ModifiedBCE[bceIndex] = mR3(0.0);
        }
    }

    sortedVelMas[idA] = velMas_ModifiedBCE[bceIndex];
    sortedRhoPreMu[idA] = rhoPreMu_ModifiedBCE[bceIndex];
    if (paramsD.elastic_SPH) {
        sortedTauXxYyZz[idA] = tauXxYyZz_ModifiedBCE[bceIndex];
        sortedTauXyXzYz[idA] = tauXyXzYz_ModifiedBCE[bceIndex];
    }
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void CalcRigidBceAccelerationD(Real3* bceAcc,
                                          Real4* q_fsiBodies_D,
                                          Real3* accRigid_fsiBodies_D,
                                          Real3* omegaVelLRF_fsiBodies_D,
                                          Real3* omegaAccLRF_fsiBodies_D,
                                          Real3* rigidSPH_MeshPos_LRF_D,
                                          const uint* rigidIdentifierD) {
    uint bceIndex = blockIdx.x * blockDim.x + threadIdx.x;
    if (bceIndex >= numObjectsD.numRigidMarkers)
        return;

    int rigidBodyIndex = rigidIdentifierD[bceIndex];

    // linear acceleration (CM)
    Real3 acc3 = accRigid_fsiBodies_D[rigidBodyIndex];

    Real4 q4 = q_fsiBodies_D[rigidBodyIndex];
    Real3 a1, a2, a3;
    RotationMatirixFromQuaternion(a1, a2, a3, q4);
    Real3 wVel3 = omegaVelLRF_fsiBodies_D[rigidBodyIndex];
    Real3 rigidSPH_MeshPos_LRF = rigidSPH_MeshPos_LRF_D[bceIndex];
    Real3 wVelCrossS = cross(wVel3, rigidSPH_MeshPos_LRF);
    Real3 wVelCrossWVelCrossS = cross(wVel3, wVelCrossS);

    // centrigugal acceleration
    acc3 += mR3(dot(a1, wVelCrossWVelCrossS), dot(a2, wVelCrossWVelCrossS), dot(a3, wVelCrossWVelCrossS));

    Real3 wAcc3 = omegaAccLRF_fsiBodies_D[rigidBodyIndex];
    Real3 wAccCrossS = cross(wAcc3, rigidSPH_MeshPos_LRF);

    // tangential acceleration
    acc3 += mR3(dot(a1, wAccCrossS), dot(a2, wAccCrossS), dot(a3, wAccCrossS));

    bceAcc[bceIndex] = acc3;
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void CalcFlexBceAccelerationD(Real3* bceAcc,
                                         Real3* pos_fsi_fea_D,
                                         Real3* vel_fsi_fea_D,
                                         Real3* acc_fsi_fea_D,
                                         Real3* FlexSPH_MeshPos_LRF_D,
                                         uint2* CableElementsNodes,
                                         uint4* ShellelementsNodes,
                                         const uint* FlexIdentifierD,
                                         const int numFlex1D,
                                         const int numFlex2D) {
    uint bceIndex = blockIdx.x * blockDim.x + threadIdx.x;
    if (bceIndex >= numObjectsD.numFlexMarkers)
        return;

    int FlexIndex = FlexIdentifierD[bceIndex];

    // BCE acc on cable elements
    if (FlexIndex < numFlex1D) {
        uint2 CableNodes = CableElementsNodes[FlexIndex];
        Real3 pos_fsi_fea_D_nA = pos_fsi_fea_D[CableNodes.x];
        Real3 pos_fsi_fea_D_nB = pos_fsi_fea_D[CableNodes.y];

        Real3 x_dir = pos_fsi_fea_D_nB - pos_fsi_fea_D_nA;
        x_dir = x_dir / length(x_dir);
        Real3 y_dir = mR3(-x_dir.y, x_dir.x, 0) + mR3(-x_dir.z, 0, x_dir.x) + mR3(0, -x_dir.z, x_dir.y);
        y_dir = y_dir / length(y_dir);
        Real3 z_dir = cross(x_dir, y_dir);

        Real2 N_cable = Cables_ShapeFunctions(FlexSPH_MeshPos_LRF_D[bceIndex].x);

        Real NA = N_cable.x;
        Real NB = N_cable.y;

        Real3 acc_fsi_fea_D_nA = acc_fsi_fea_D[CableNodes.x];
        Real3 acc_fsi_fea_D_nB = acc_fsi_fea_D[CableNodes.y];

        bceAcc[bceIndex + numObjectsD.numRigidMarkers] = NA * acc_fsi_fea_D_nA + NB * acc_fsi_fea_D_nB;
    }
    // BCE acc on shell elements
    if (FlexIndex >= numFlex1D && FlexIndex < numFlex2D) {
        uint4 shellNodes = ShellelementsNodes[FlexIndex - numFlex1D];
        Real3 pos_fsi_fea_D_nA = pos_fsi_fea_D[shellNodes.x];
        Real3 pos_fsi_fea_D_nB = pos_fsi_fea_D[shellNodes.y];
        Real3 pos_fsi_fea_D_nC = pos_fsi_fea_D[shellNodes.z];
        Real3 pos_fsi_fea_D_nD = pos_fsi_fea_D[shellNodes.w];

        Real3 Shell_center = 0.25 * (pos_fsi_fea_D_nA + pos_fsi_fea_D_nB + pos_fsi_fea_D_nC + pos_fsi_fea_D_nD);

        Real3 x_dir = (pos_fsi_fea_D_nB - pos_fsi_fea_D_nA) + (pos_fsi_fea_D_nC - pos_fsi_fea_D_nD);

        Real3 n1 = normalize(cross(pos_fsi_fea_D_nB - pos_fsi_fea_D_nA, pos_fsi_fea_D_nC - pos_fsi_fea_D_nB));
        Real3 n2 = normalize(cross(pos_fsi_fea_D_nC - pos_fsi_fea_D_nB, pos_fsi_fea_D_nD - pos_fsi_fea_D_nC));
        Real3 n3 = normalize(cross(pos_fsi_fea_D_nD - pos_fsi_fea_D_nC, pos_fsi_fea_D_nA - pos_fsi_fea_D_nD));
        Real3 n4 = normalize(cross(pos_fsi_fea_D_nA - pos_fsi_fea_D_nD, pos_fsi_fea_D_nB - pos_fsi_fea_D_nA));
        Real3 Normal = normalize(n1 + n2 + n3 + n4);
        Real3 y_dir = cross(Normal, x_dir);

        Real Shell_x = 0.25 * length(pos_fsi_fea_D_nB - pos_fsi_fea_D_nA + pos_fsi_fea_D_nC - pos_fsi_fea_D_nD);
        Real Shell_y = 0.25 * length(pos_fsi_fea_D_nD - pos_fsi_fea_D_nA + pos_fsi_fea_D_nC - pos_fsi_fea_D_nB);

        Real4 N_shell = Shells_ShapeFunctions(FlexSPH_MeshPos_LRF_D[bceIndex].x, FlexSPH_MeshPos_LRF_D[bceIndex].y);

        Real NA = N_shell.x;
        Real NB = N_shell.y;
        Real NC = N_shell.z;
        Real ND = N_shell.w;

        Real3 acc_fsi_fea_D_nA = acc_fsi_fea_D[shellNodes.x];
        Real3 acc_fsi_fea_D_nB = acc_fsi_fea_D[shellNodes.y];
        Real3 acc_fsi_fea_D_nC = acc_fsi_fea_D[shellNodes.z];
        Real3 acc_fsi_fea_D_nD = acc_fsi_fea_D[shellNodes.w];

        bceAcc[bceIndex + numObjectsD.numRigidMarkers] =
            NA * acc_fsi_fea_D_nA + NB * acc_fsi_fea_D_nB + NC * acc_fsi_fea_D_nC + ND * acc_fsi_fea_D_nD;
    }
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void UpdateRigidMarkersPositionVelocityD(Real4* posRadD,
                                                    Real3* velMasD,
                                                    Real3* rigidSPH_MeshPos_LRF_D,
                                                    uint* rigidIdentifierD,
                                                    Real3* posRigidD,
                                                    Real4* velMassRigidD,
                                                    Real3* omegaLRF_D,
                                                    Real4* qD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numRigidMarkers)
        return;

    uint rigidMarkerIndex = index + numObjectsD.startRigidMarkers;
    int rigidBodyIndex = rigidIdentifierD[index];

    Real4 q4 = qD[rigidBodyIndex];
    Real3 a1, a2, a3;
    RotationMatirixFromQuaternion(a1, a2, a3, q4);

    Real3 rigidSPH_MeshPos_LRF = rigidSPH_MeshPos_LRF_D[index];

    // position
    Real h = posRadD[rigidMarkerIndex].w;
    Real3 p_Rigid = posRigidD[rigidBodyIndex];
    Real3 pos =
        p_Rigid + mR3(dot(a1, rigidSPH_MeshPos_LRF), dot(a2, rigidSPH_MeshPos_LRF), dot(a3, rigidSPH_MeshPos_LRF));
    posRadD[rigidMarkerIndex] = mR4(pos, h);

    // velocity
    Real4 vM_Rigid = velMassRigidD[rigidBodyIndex];
    Real3 omega3 = omegaLRF_D[rigidBodyIndex];
    Real3 omegaCrossS = cross(omega3, rigidSPH_MeshPos_LRF);
    velMasD[rigidMarkerIndex] = mR3(vM_Rigid) + mR3(dot(a1, omegaCrossS), dot(a2, omegaCrossS), dot(a3, omegaCrossS));
}
//--------------------------------------------------------------------------------------------------------------------------------
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
    if (index >= numObjectsD.numFlexMarkers)
        return;

    int FlexIndex = FlexIdentifierD[index];
    uint FlexMarkerIndex = index + numObjectsD.startFlexMarkers;

    if (FlexIndex < numFlex1D) {
        uint2 CableNodes = CableElementsNodes[FlexIndex];
        Real3 pos_fsi_fea_D_nA = pos_fsi_fea_D[CableNodes.x];
        Real3 pos_fsi_fea_D_nB = pos_fsi_fea_D[CableNodes.y];

        Real3 x_dir = pos_fsi_fea_D_nB - pos_fsi_fea_D_nA;
        x_dir = x_dir / length(x_dir);
        Real3 y_dir = mR3(-x_dir.y, x_dir.x, 0) + mR3(-x_dir.z, 0, x_dir.x) + mR3(0, -x_dir.z, x_dir.y);
        y_dir = y_dir / length(y_dir);
        Real3 z_dir = cross(x_dir, y_dir);

        Real2 N_cable = Cables_ShapeFunctions(FlexSPH_MeshPos_LRF_D[index].x);

        Real NA = N_cable.x;
        Real NB = N_cable.y;

        Real3 vel_fsi_fea_D_nA = vel_fsi_fea_D[CableNodes.x];
        Real3 vel_fsi_fea_D_nB = vel_fsi_fea_D[CableNodes.y];

        Real h = posRadD[FlexMarkerIndex].w;
        Real3 tempPos = NA * pos_fsi_fea_D_nA + NB * pos_fsi_fea_D_nB +
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

        Real3 x_dir = (pos_fsi_fea_D_nB - pos_fsi_fea_D_nA) + (pos_fsi_fea_D_nC - pos_fsi_fea_D_nD);

        Real3 n1 = normalize(cross(pos_fsi_fea_D_nB - pos_fsi_fea_D_nA, pos_fsi_fea_D_nC - pos_fsi_fea_D_nB));
        Real3 n2 = normalize(cross(pos_fsi_fea_D_nC - pos_fsi_fea_D_nB, pos_fsi_fea_D_nD - pos_fsi_fea_D_nC));
        Real3 n3 = normalize(cross(pos_fsi_fea_D_nD - pos_fsi_fea_D_nC, pos_fsi_fea_D_nA - pos_fsi_fea_D_nD));
        Real3 n4 = normalize(cross(pos_fsi_fea_D_nA - pos_fsi_fea_D_nD, pos_fsi_fea_D_nB - pos_fsi_fea_D_nA));
        Real3 Normal = normalize(n1 + n2 + n3 + n4);
        Real3 y_dir = cross(Normal, x_dir);

        Real Shell_x = 0.25 * length(pos_fsi_fea_D_nB - pos_fsi_fea_D_nA + pos_fsi_fea_D_nC - pos_fsi_fea_D_nD);
        Real Shell_y = 0.25 * length(pos_fsi_fea_D_nD - pos_fsi_fea_D_nA + pos_fsi_fea_D_nC - pos_fsi_fea_D_nB);

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

        posRadD[FlexMarkerIndex] = mR4(tempPos, h);
        velMasD[FlexMarkerIndex] =
            NA * vel_fsi_fea_D_nA + NB * vel_fsi_fea_D_nB + NC * vel_fsi_fea_D_nC + ND * vel_fsi_fea_D_nD;
    }
}
//--------------------------------------------------------------------------------------------------------------------------------
ChBce::ChBce(std::shared_ptr<SphMarkerDataD> otherSortedSphMarkersD,
             std::shared_ptr<ProximityDataD> otherMarkersProximityD,
             std::shared_ptr<FsiGeneralData> otherFsiGeneralData,
             std::shared_ptr<SimParams> otherParamsH,
             std::shared_ptr<ChCounters> otherNumObjects,
             bool verb)
    : sortedSphMarkersD(otherSortedSphMarkersD),
      markersProximityD(otherMarkersProximityD),
      fsiGeneralData(otherFsiGeneralData),
      paramsH(otherParamsH),
      numObjectsH(otherNumObjects),
      verbose(verb) {
    totalForceRigid.resize(0);
    totalTorqueRigid.resize(0);
}
ChBce::~ChBce() {}
//--------------------------------------------------------------------------------------------------------------------------------
void ChBce::Initialize(std::shared_ptr<SphMarkerDataD> sphMarkersD,
                       std::shared_ptr<FsiBodiesDataD> fsiBodiesD,
                       std::shared_ptr<FsiMeshDataD> fsiMeshD,
                       std::vector<int> fsiBodyBceNum,
                       std::vector<int> fsiShellBceNum,
                       std::vector<int> fsiCableBceNum) {
    cudaMemcpyToSymbolAsync(paramsD, paramsH.get(), sizeof(SimParams));
    cudaMemcpyToSymbolAsync(numObjectsD, numObjectsH.get(), sizeof(ChCounters));
    CopyParams_NumberOfObjects(paramsH, numObjectsH);

    // Resizing the arrays used to modify the BCE velocity and pressure according to ADAMI
    totalForceRigid.resize(numObjectsH->numRigidBodies);
    totalTorqueRigid.resize(numObjectsH->numRigidBodies);

    int haveGhost = (numObjectsH->numGhostMarkers > 0) ? 1 : 0;
    int haveHelper = (numObjectsH->numHelperMarkers > 0) ? 1 : 0;
    int haveRigid = (numObjectsH->numRigidBodies > 0) ? 1 : 0;
    int haveFlex1D = (numObjectsH->numFlexBodies1D > 0) ? 1 : 0;
    int haveFlex2D = (numObjectsH->numFlexBodies2D > 0) ? 1 : 0;

    int num = haveHelper + haveGhost + haveRigid + haveFlex1D + haveFlex2D + 1;
    int numFlexRigidBoundaryMarkers =
        fsiGeneralData->referenceArray[num].y - fsiGeneralData->referenceArray[haveHelper + haveGhost].y;

    if (verbose) {
        printf("Total number of BCE particles = %d\n", numFlexRigidBoundaryMarkers);
        if (paramsH->bceType == BceVersion::ADAMI)
            printf("Boundary condition for rigid and flexible body is: ADAMI\n");
        if (paramsH->bceType == BceVersion::ORIGINAL)
            printf("Boundary condition for rigid and flexible body is: ORIGINAL\n");
        if (paramsH->bceTypeWall == BceVersion::ADAMI)
            printf("Boundary condition for fixed wall is: ADAMI\n");
        if (paramsH->bceTypeWall == BceVersion::ORIGINAL)
            printf("Boundary condition for fixed wall is: ORIGINAL\n");
    }

    int numAllBce = numObjectsH->numBoundaryMarkers + numObjectsH->numRigidMarkers + numObjectsH->numFlexMarkers;
    if (numAllBce != numFlexRigidBoundaryMarkers) {
        throw std::runtime_error(
            "Error! number of flex and rigid and "
            "boundary markers are saved incorrectly!\n");
    }
    velMas_ModifiedBCE.resize(numFlexRigidBoundaryMarkers);
    rhoPreMu_ModifiedBCE.resize(numFlexRigidBoundaryMarkers);
    tauXxYyZz_ModifiedBCE.resize(numFlexRigidBoundaryMarkers);
    tauXyXzYz_ModifiedBCE.resize(numFlexRigidBoundaryMarkers);

    // Populate local position of BCE markers - on rigid bodies
    if (haveRigid)
        Populate_RigidSPH_MeshPos_LRF(sphMarkersD, fsiBodiesD, fsiBodyBceNum);

    // Populate local position of BCE markers - on flexible bodies
    if (haveFlex1D || haveFlex2D)
        Populate_FlexSPH_MeshPos_LRF(sphMarkersD, fsiMeshD, fsiShellBceNum, fsiCableBceNum);
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChBce::Populate_RigidSPH_MeshPos_LRF(std::shared_ptr<SphMarkerDataD> sphMarkersD,
                                          std::shared_ptr<FsiBodiesDataD> fsiBodiesD,
                                          std::vector<int> fsiBodyBceNum) {
    // Create map between a BCE on a rigid body and the associated body ID
    uint start_bce = 0;
    for (int irigid = 0; irigid < fsiBodyBceNum.size(); irigid++) {
        uint end_bce = start_bce + fsiBodyBceNum[irigid];
        thrust::fill(fsiGeneralData->rigidIdentifierD.begin() + start_bce,
                     fsiGeneralData->rigidIdentifierD.begin() + end_bce, irigid);
        start_bce = end_bce;
    }

    uint nBlocks, nThreads;
    computeGridSize((uint)numObjectsH->numRigidMarkers, 256, nBlocks, nThreads);

    Populate_RigidSPH_MeshPos_LRF_D<<<nBlocks, nThreads>>>(
        mR3CAST(fsiGeneralData->rigidSPH_MeshPos_LRF_D), mR4CAST(sphMarkersD->posRadD),
        U1CAST(fsiGeneralData->rigidIdentifierD), mR3CAST(fsiBodiesD->posRigid_fsiBodies_D),
        mR4CAST(fsiBodiesD->q_fsiBodies_D));

    cudaDeviceSynchronize();
    cudaCheckError();

    UpdateRigidMarkersPositionVelocity(sphMarkersD, fsiBodiesD);
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChBce::Populate_FlexSPH_MeshPos_LRF(std::shared_ptr<SphMarkerDataD> sphMarkersD,
                                         std::shared_ptr<FsiMeshDataD> fsiMeshD,
                                         std::vector<int> fsiShellBceNum,
                                         std::vector<int> fsiCableBceNum) {
    // Create map between a BCE on a flexible body and the associated flexible body ID
    uint start_bce = 0;
    for (uint icable = 0; icable < fsiCableBceNum.size(); icable++) {
        uint end_bce = start_bce + fsiCableBceNum[icable];
        thrust::fill(fsiGeneralData->FlexIdentifierD.begin() + start_bce,
                     fsiGeneralData->FlexIdentifierD.begin() + end_bce, icable);
        start_bce = end_bce;
    }

    for (uint ishell = 0; ishell < fsiShellBceNum.size(); ishell++) {
        uint end_bce = start_bce + fsiShellBceNum[ishell];
        thrust::fill(fsiGeneralData->FlexIdentifierD.begin() + start_bce,
                     fsiGeneralData->FlexIdentifierD.begin() + end_bce, ishell + fsiCableBceNum.size());
        start_bce = end_bce;
    }

    uint nBlocks, nThreads;
    computeGridSize((uint)numObjectsH->numFlexMarkers, 256, nBlocks, nThreads);

    thrust::device_vector<Real3> FlexSPH_MeshPos_LRF_H = fsiGeneralData->FlexSPH_MeshPos_LRF_H;
    Populate_FlexSPH_MeshPos_LRF_D<<<nBlocks, nThreads>>>(
        mR3CAST(fsiGeneralData->FlexSPH_MeshPos_LRF_D), mR3CAST(FlexSPH_MeshPos_LRF_H), mR4CAST(sphMarkersD->posRadD),
        U1CAST(fsiGeneralData->FlexIdentifierD), (int)numObjectsH->numFlexBodies1D,
        U2CAST(fsiGeneralData->CableElementsNodes), U4CAST(fsiGeneralData->ShellElementsNodes),
        mR3CAST(fsiMeshD->pos_fsi_fea_D), paramsH->HSML * paramsH->MULT_INITSPACE_Shells);

    cudaDeviceSynchronize();
    cudaCheckError();

    UpdateFlexMarkersPositionVelocity(sphMarkersD, fsiMeshD);
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChBce::ReCalcVelocityPressureStress_BCE(thrust::device_vector<Real3>& velMas_ModifiedBCE,
                                             thrust::device_vector<Real4>& rhoPreMu_ModifiedBCE,
                                             thrust::device_vector<Real3>& tauXxYyZz_ModifiedBCE,
                                             thrust::device_vector<Real3>& tauXyXzYz_ModifiedBCE,
                                             const thrust::device_vector<Real4>& sortedPosRad,
                                             const thrust::device_vector<Real3>& sortedVelMas,
                                             const thrust::device_vector<Real4>& sortedRhoPreMu,
                                             const thrust::device_vector<Real3>& sortedTauXxYyZz,
                                             const thrust::device_vector<Real3>& sortedTauXyXzYz,
                                             const thrust::device_vector<uint>& cellStart,
                                             const thrust::device_vector<uint>& cellEnd,
                                             const thrust::device_vector<uint>& mapOriginalToSorted,
                                             const thrust::device_vector<Real3>& bceAcc,
                                             int4 updatePortion) {
    bool *isErrorH, *isErrorD;
    isErrorH = (bool*)malloc(sizeof(bool));
    cudaMalloc((void**)&isErrorD, sizeof(bool));
    *isErrorH = false;
    cudaMemcpy(isErrorD, isErrorH, sizeof(bool), cudaMemcpyHostToDevice);

    // thread per particle
    int2 newPortion = mI2(updatePortion.x, updatePortion.w);
    if (paramsH->bceTypeWall == BceVersion::ORIGINAL) {
        // Only implement ADAMI BC for rigid body boundary.
        // Implement a simple BC for fixed wall to avoid unnecessary cost.
        newPortion = mI2(updatePortion.y, updatePortion.w);
    }
    uint numBCE = newPortion.y - newPortion.x;
    uint numThreads, numBlocks;
    computeGridSize(numBCE, 256, numBlocks, numThreads);

    BCE_VelocityPressureStress<<<numBlocks, numThreads>>>(
        mR3CAST(velMas_ModifiedBCE), mR4CAST(rhoPreMu_ModifiedBCE), mR3CAST(tauXxYyZz_ModifiedBCE),
        mR3CAST(tauXyXzYz_ModifiedBCE), mR4CAST(sortedPosRad), mR3CAST(sortedVelMas), mR4CAST(sortedRhoPreMu),
        mR3CAST(sortedTauXxYyZz), mR3CAST(sortedTauXyXzYz), U1CAST(cellStart), U1CAST(cellEnd),
        U1CAST(mapOriginalToSorted), mR3CAST(bceAcc), newPortion, isErrorD);

    cudaDeviceSynchronize();
    cudaCheckError()

    cudaMemcpy(isErrorH, isErrorD, sizeof(bool), cudaMemcpyDeviceToHost);
    if (*isErrorH == true)
        throw std::runtime_error("Error! program crashed in new_BCE_VelocityPressure!\n");

    cudaFree(isErrorD);
    free(isErrorH);
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChBce::CalcRigidBceAcceleration(thrust::device_vector<Real3>& bceAcc,
                                     const thrust::device_vector<Real4>& q_fsiBodies_D,
                                     const thrust::device_vector<Real3>& accRigid_fsiBodies_D,
                                     const thrust::device_vector<Real3>& omegaVelLRF_fsiBodies_D,
                                     const thrust::device_vector<Real3>& omegaAccLRF_fsiBodies_D,
                                     const thrust::device_vector<Real3>& rigidSPH_MeshPos_LRF_D,
                                     const thrust::device_vector<uint>& rigidIdentifierD,
                                     int numRigidMarkers) {
    // thread per particle
    uint numThreads, numBlocks;
    computeGridSize(numRigidMarkers, 256, numBlocks, numThreads);

    CalcRigidBceAccelerationD<<<numBlocks, numThreads>>>(
        mR3CAST(bceAcc), mR4CAST(q_fsiBodies_D), mR3CAST(accRigid_fsiBodies_D), mR3CAST(omegaVelLRF_fsiBodies_D),
        mR3CAST(omegaAccLRF_fsiBodies_D), mR3CAST(rigidSPH_MeshPos_LRF_D), U1CAST(rigidIdentifierD));

    cudaDeviceSynchronize();
    cudaCheckError();
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChBce::CalcFlexBceAcceleration(thrust::device_vector<Real3>& bceAcc,
                                    const thrust::device_vector<Real3>& pos_fsi_fea_D,
                                    const thrust::device_vector<Real3>& vel_fsi_fea_D,
                                    const thrust::device_vector<Real3>& acc_fsi_fea_D,
                                    const thrust::device_vector<Real3>& FlexSPH_MeshPos_LRF_D,
                                    const thrust::device_vector<int2>& CableElementsNodes,
                                    const thrust::device_vector<int4>& ShellElementsNodes,
                                    const thrust::device_vector<uint>& FlexIdentifierD,
                                    int numFlexMarkers,
                                    int numFlexBodies1D,
                                    int numFlexBodies2D) {
    // thread per particle
    uint numThreads, numBlocks;
    computeGridSize(numFlexMarkers, 256, numBlocks, numThreads);

    CalcFlexBceAccelerationD<<<numBlocks, numThreads>>>(mR3CAST(bceAcc), mR3CAST(pos_fsi_fea_D), mR3CAST(vel_fsi_fea_D),
                                                        mR3CAST(acc_fsi_fea_D), mR3CAST(FlexSPH_MeshPos_LRF_D),
                                                        U2CAST(CableElementsNodes), U4CAST(ShellElementsNodes),
                                                        U1CAST(FlexIdentifierD), numFlexBodies1D, numFlexBodies2D);

    cudaDeviceSynchronize();
    cudaCheckError();
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChBce::ModifyBceVelocityPressureStress(std::shared_ptr<SphMarkerDataD> sphMarkersD,
                                            std::shared_ptr<FsiBodiesDataD> fsiBodiesD,
                                            std::shared_ptr<FsiMeshDataD> fsiMeshD) {
    int size_ref = fsiGeneralData->referenceArray.size();
    int numBceMarkers = fsiGeneralData->referenceArray[size_ref - 1].y - fsiGeneralData->referenceArray[0].y;
    int N_all = numObjectsH->numBoundaryMarkers + numObjectsH->numRigidMarkers + numObjectsH->numFlexMarkers;
    if (N_all != numBceMarkers) {
        throw std::runtime_error(
            "Error! Number of rigid, flexible and boundary markers are "
            "saved incorrectly. Thrown from ModifyBceVelocityPressureStress!\n");
    }

    if (!(velMas_ModifiedBCE.size() == numBceMarkers && rhoPreMu_ModifiedBCE.size() == numBceMarkers &&
          tauXxYyZz_ModifiedBCE.size() == numBceMarkers && tauXyXzYz_ModifiedBCE.size() == numBceMarkers)) {
        throw std::runtime_error(
            "Error! Size error velMas_ModifiedBCE and "
            "tauXxYyZz_ModifiedBCE and tauXyXzYz_ModifiedBCE and "
            "rhoPreMu_ModifiedBCE. Thrown from ModifyBceVelocityPressureStress!\n");
    }

    // Update portion set to boundary, rigid, and flexible BCE particles
    int4 updatePortion = mI4(fsiGeneralData->referenceArray[0].y, fsiGeneralData->referenceArray[1].y,
        fsiGeneralData->referenceArray[2].y, fsiGeneralData->referenceArray[3].y);

    // Only update boundary BCE particles if no rigid/flexible particles
    if (size_ref == 2) {
        updatePortion.z = fsiGeneralData->referenceArray[1].y;
        updatePortion.w = fsiGeneralData->referenceArray[1].y;
    }

    // Update boundary and rigid/flexible BCE particles 
    if (size_ref == 3)
        updatePortion.w = fsiGeneralData->referenceArray[2].y;


    // ADAMI boundary condition (wall, rigid, flexible)
    if (paramsH->bceType == BceVersion::ADAMI) {
        // Calculate the acceleration of rigid/flexible BCE particles if exist, used for ADAMI BC
        thrust::device_vector<Real3> bceAcc(numObjectsH->numRigidMarkers + numObjectsH->numFlexMarkers);
        // Acceleration of rigid BCE particles
        if (numObjectsH->numRigidMarkers > 0) {
            CalcRigidBceAcceleration(bceAcc, fsiBodiesD->q_fsiBodies_D, fsiBodiesD->accRigid_fsiBodies_D,
                                     fsiBodiesD->omegaVelLRF_fsiBodies_D, fsiBodiesD->omegaAccLRF_fsiBodies_D,
                                     fsiGeneralData->rigidSPH_MeshPos_LRF_D, fsiGeneralData->rigidIdentifierD,
                                     (int)numObjectsH->numRigidMarkers);
        }
        // Acceleration of flexible BCE particles
        if (numObjectsH->numFlexMarkers > 0) {
            CalcFlexBceAcceleration(bceAcc, fsiMeshD->pos_fsi_fea_D, fsiMeshD->vel_fsi_fea_D, fsiMeshD->acc_fsi_fea_D,
                                    fsiGeneralData->FlexSPH_MeshPos_LRF_D, fsiGeneralData->CableElementsNodes,
                                    fsiGeneralData->ShellElementsNodes, fsiGeneralData->FlexIdentifierD,
                                    (int)numObjectsH->numFlexMarkers, (int)numObjectsH->numFlexBodies1D,
                                    (int)numObjectsH->numFlexBodies2D);
        }
        // ADAMI BC for rigid/flexible body, ORIGINAL BC for fixed wall
        if (paramsH->bceTypeWall == BceVersion::ORIGINAL) {
            thrust::copy(sphMarkersD->velMasD.begin() + updatePortion.x, sphMarkersD->velMasD.begin() + updatePortion.y,
                         velMas_ModifiedBCE.begin());
            thrust::copy(sphMarkersD->rhoPresMuD.begin() + updatePortion.x,
                         sphMarkersD->rhoPresMuD.begin() + updatePortion.y, rhoPreMu_ModifiedBCE.begin());
            if (paramsH->elastic_SPH) {
                thrust::copy(sphMarkersD->tauXxYyZzD.begin() + updatePortion.x,
                             sphMarkersD->tauXxYyZzD.begin() + updatePortion.y, tauXxYyZz_ModifiedBCE.begin());
                thrust::copy(sphMarkersD->tauXyXzYzD.begin() + updatePortion.x,
                             sphMarkersD->tauXyXzYzD.begin() + updatePortion.y, tauXyXzYz_ModifiedBCE.begin());
            }
            if (numObjectsH->numRigidMarkers > 0 || numObjectsH->numFlexMarkers > 0) {
                ReCalcVelocityPressureStress_BCE(
                    velMas_ModifiedBCE, rhoPreMu_ModifiedBCE, tauXxYyZz_ModifiedBCE, tauXyXzYz_ModifiedBCE,
                    sortedSphMarkersD->posRadD, sortedSphMarkersD->velMasD, sortedSphMarkersD->rhoPresMuD,
                    sortedSphMarkersD->tauXxYyZzD, sortedSphMarkersD->tauXyXzYzD, markersProximityD->cellStartD,
                    markersProximityD->cellEndD, markersProximityD->mapOriginalToSorted, bceAcc, updatePortion);
            }
        }
        // ADAMI BC for both rigid/flexible body and fixed wall
        else if (paramsH->bceTypeWall == BceVersion::ADAMI) {
            ReCalcVelocityPressureStress_BCE(
                velMas_ModifiedBCE, rhoPreMu_ModifiedBCE, tauXxYyZz_ModifiedBCE, tauXyXzYz_ModifiedBCE,
                sortedSphMarkersD->posRadD, sortedSphMarkersD->velMasD, sortedSphMarkersD->rhoPresMuD,
                sortedSphMarkersD->tauXxYyZzD, sortedSphMarkersD->tauXyXzYzD, markersProximityD->cellStartD,
                markersProximityD->cellEndD, markersProximityD->mapOriginalToSorted, bceAcc, updatePortion);
        }
        bceAcc.clear();
    }
    // ORIGINAL boundary condition for all boundaries (wall, rigid, flexible)
    else {
        thrust::copy(sphMarkersD->velMasD.begin() + updatePortion.x, sphMarkersD->velMasD.begin() + updatePortion.w,
                     velMas_ModifiedBCE.begin());
        thrust::copy(sphMarkersD->rhoPresMuD.begin() + updatePortion.x,
                     sphMarkersD->rhoPresMuD.begin() + updatePortion.w, rhoPreMu_ModifiedBCE.begin());
        if (paramsH->elastic_SPH) {
            thrust::copy(sphMarkersD->tauXxYyZzD.begin() + updatePortion.x,
                         sphMarkersD->tauXxYyZzD.begin() + updatePortion.w, tauXxYyZz_ModifiedBCE.begin());
            thrust::copy(sphMarkersD->tauXyXzYzD.begin() + updatePortion.x,
                         sphMarkersD->tauXyXzYzD.begin() + updatePortion.w, tauXyXzYz_ModifiedBCE.begin());
        }
    }
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChBce::Rigid_Forces_Torques(std::shared_ptr<SphMarkerDataD> sphMarkersD,
                                 std::shared_ptr<FsiBodiesDataD> fsiBodiesD) {
    if (numObjectsH->numRigidBodies == 0)
        return;

    thrust::fill(fsiGeneralData->rigid_FSI_ForcesD.begin(), fsiGeneralData->rigid_FSI_ForcesD.end(), mR3(0));
    thrust::fill(fsiGeneralData->rigid_FSI_TorquesD.begin(), fsiGeneralData->rigid_FSI_TorquesD.end(), mR3(0));

    uint nBlocks, nThreads;
    computeGridSize((uint)numObjectsH->numRigidMarkers, 256, nBlocks, nThreads);

    Calc_Rigid_FSI_Forces_Torques_D<<<nBlocks, nThreads>>>(
        mR3CAST(fsiGeneralData->rigid_FSI_ForcesD), mR3CAST(fsiGeneralData->rigid_FSI_TorquesD),
        mR4CAST(fsiGeneralData->derivVelRhoD), mR4CAST(fsiGeneralData->derivVelRhoD_old), mR4CAST(sphMarkersD->posRadD),
        U1CAST(fsiGeneralData->rigidIdentifierD), mR3CAST(fsiBodiesD->posRigid_fsiBodies_D),
        mR3CAST(fsiGeneralData->rigidSPH_MeshPos_LRF_D), mR4CAST(fsiBodiesD->q_fsiBodies_D));

    cudaDeviceSynchronize();
    cudaCheckError();
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChBce::Flex_Forces(std::shared_ptr<SphMarkerDataD> sphMarkersD, std::shared_ptr<FsiMeshDataD> fsiMeshD) {
    if ((numObjectsH->numFlexBodies1D + numObjectsH->numFlexBodies2D) == 0)
        return;

    thrust::fill(fsiGeneralData->Flex_FSI_ForcesD.begin(), fsiGeneralData->Flex_FSI_ForcesD.end(), mR3(0));

    uint nBlocks, nThreads;
    computeGridSize((int)numObjectsH->numFlexMarkers, 256, nBlocks, nThreads);

    Calc_Flex_FSI_ForcesD<<<nBlocks, nThreads>>>(
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
    if (numObjectsH->numRigidBodies == 0)
        return;

    uint nBlocks, nThreads;
    computeGridSize((int)numObjectsH->numRigidMarkers, 256, nBlocks, nThreads);

    UpdateRigidMarkersPositionVelocityD<<<nBlocks, nThreads>>>(
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
    if ((numObjectsH->numFlexBodies1D + numObjectsH->numFlexBodies2D) == 0)
        return;

    uint nBlocks, nThreads;
    computeGridSize((int)numObjectsH->numFlexMarkers, 256, nBlocks, nThreads);

    UpdateFlexMarkersPositionVelocityAccD<<<nBlocks, nThreads>>>(
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
